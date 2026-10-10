package com.michaelgrundvig.frc.runtime.behavior;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.config.Finding;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.simulation.GamepadSim;

/**
 * The drive team's gamepads record each binding as it's made, composed ones too, with the button's
 * name, how, and the command's name; each press and release logs what it starts; and two commands
 * on one button the same way are a doubt.
 */
@ExtendWith(RobotSim.class)
class BindingsTest {
  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final Bindings bindings = new Bindings(scheduler);
  private final RecordingGamepad driver = bindings.gamepad("driver", 0);
  private final RecordingGamepad operator = bindings.gamepad("operator", 1);

  /** A command that runs until it's canceled, requiring nothing. */
  private static Command named(String name) {
    return Command.noRequirements(
            co -> {
              while (true) {
                co.yield();
              }
            })
        .named(name);
  }

  /** One robot loop: the Driver Station's newest data, then the scheduler. */
  private void loop() {
    RobotSim.stepClock();
    DriverStationBackend.refreshData();
    scheduler.run();
  }

  private List<String> recorded() {
    return bindings.all().stream().map(Binding::toString).toList();
  }

  @Test
  void eachBindingIsRecordedWithItsButtonHowAndCommand() {
    operator.rightTrigger().whileTrue(named("Intake"));
    operator.faceRight().onTrue(named("Stow"));
    operator.leftTrigger(0.2).onFalse(named("Hold"));
    driver.back().toggleOnTrue(named("Spin up"));
    driver.dpadUp().retryWhileTrue(named("Climb"));
    driver.button(30).whileFalse(named("Idle"));

    assertThat(recorded())
        .containsExactly(
            "operator right trigger: while held: Intake",
            "operator face right: on press: Stow",
            "operator left trigger past 0.2: on release: Hold",
            "driver back: toggled on press: Spin up",
            "driver dpad up: while held, again if it ends: Climb",
            "driver button 30: while released: Idle");
    assertThat(bindings.all().get(0))
        .isEqualTo(new Binding("operator", "right trigger", Binding.How.WHILE_HELD, "Intake"));
    assertThat(operator.faceRight()).as("one trigger a button").isSameAs(operator.faceRight());
  }

  /**
   * A trigger made from a button reads as what it is: a gate by its name, either way round, another
   * button, or one on the other gamepad by both names.
   */
  @Test
  void aComposedBindingCarriesItsTermsNames() {
    Gate ready = Gate.of(scheduler, "ready to shoot", () -> true);

    operator.rightBumper().and(ready).whileTrue(named("Shoot"));
    ready.and(operator.leftBumper()).whileTrue(named("Feed"));
    ready.or(operator.faceUp()).onTrue(named("Rumble"));
    driver.leftBumper().and(driver.rightBumper()).onTrue(named("Reset"));
    driver.faceDown().and(operator.faceDown()).onTrue(named("Both"));
    driver.faceUp().or(driver.faceLeft()).negate().onTrue(named("Neither"));
    driver.start().and(() -> true).onTrue(named("Unnamed"));
    driver.leftStick().debounce(Seconds.of(0.1)).onTrue(named("Held"));
    driver
        .rightStick()
        .debounce(Seconds.of(0.1), Debouncer.DebounceType.FALLING)
        .onTrue(named("F"));
    driver.guide().debounce(Seconds.of(0.1), Debouncer.DebounceType.BOTH).onTrue(named("B"));
    driver.dpadDown().risingEdge().onTrue(named("Rise"));
    driver.dpadLeft().fallingEdge().onTrue(named("Fall"));
    driver.dpadRight().multiPress(2, Seconds.of(0.5)).onTrue(named("Twice"));

    assertThat(recorded())
        .containsExactly(
            "operator right bumper and ready to shoot: while held: Shoot",
            "operator left bumper and ready to shoot: while held: Feed",
            "operator face up or ready to shoot: on press: Rumble",
            "driver left bumper and right bumper: on press: Reset",
            "driver face down and operator face down: on press: Both",
            "driver not (face up or face left): on press: Neither",
            "driver start and an unnamed condition: on press: Unnamed",
            "driver left stick for 0.1 s: on press: Held",
            "driver right stick, and 0.1 s after: on press: F",
            "driver guide debounced 0.1 s: on press: B",
            "driver dpad down rising: on press: Rise",
            "driver dpad left falling: on press: Fall",
            "driver dpad right pressed 2 times in 0.5 s: on press: Twice");
  }

  /**
   * A press logs what it starts, that loop only; a release what starts as it's let go; a composed
   * trigger fires as v3's does, in the same loop.
   */
  @Test
  void eachPressAndReleaseLogsTheBindingsItStarts() {
    // Made after the test's Driver Station is reset, which clears a gamepad's axes.
    GamepadSim operatorPad = new GamepadSim(1);
    Command intake = named("Intake");
    Command hold = named("Hold");
    operator.rightTrigger().whileTrue(intake);
    operator.rightTrigger().onFalse(hold);
    operator.faceDown().and(operator.faceUp()).onTrue(named("Both"));
    loop();
    List<Binding> atStart = bindings.fired();

    operatorPad.setRightTriggerAxis(1);
    operatorPad.notifyNewData();
    loop();
    List<Binding> pressed = bindings.fired();
    boolean intaking = scheduler.isRunning(intake);
    loop();
    List<Binding> held = bindings.fired();
    operatorPad.setRightTriggerAxis(0);
    operatorPad.setFaceDownButton(true);
    operatorPad.setFaceUpButton(true);
    operatorPad.notifyNewData();
    loop();
    List<Binding> released = bindings.fired();

    assertThat(atStart)
        .as("released as the robot starts, as v3 takes it")
        .extracting(Binding::command)
        .containsExactly("Hold");
    assertThat(pressed).extracting(Binding::command).containsExactly("Intake");
    assertThat(intaking).isTrue();
    assertThat(held).as("only the loop it's pressed").isEmpty();
    assertThat(released).extracting(Binding::command).containsExactly("Hold", "Both");
    assertThat(scheduler.isRunning(intake)).isFalse();
  }

  /** A binding made inside a running command lives only as long as it, and isn't recorded. */
  @Test
  void aBindingMadeInsideACommandIsntRecorded() {
    scheduler.schedule(
        Command.noRequirements(
                co -> {
                  operator.faceLeft().onTrue(named("While aiming"));
                  co.yield();
                })
            .named("Aim"));
    loop();

    assertThat(bindings.all()).isEmpty();
  }

  /**
   * Two commands bound to one button the same way start together: a doubt, saying what to do. Bound
   * to its press and its release, or to two buttons, they don't.
   */
  @Test
  void twoCommandsOnOneButtonTheSameWayAreADoubt() {
    operator.rightTrigger().whileTrue(named("Intake"));
    operator.rightTrigger().whileTrue(named("Outtake"));
    operator.rightTrigger().onTrue(named("Rumble"));
    operator.faceDown().onTrue(named("Deploy"));
    operator.faceDown().onFalse(named("Stow"));
    driver.faceDown().onTrue(named("Deploy"));
    driver.back().onTrue(named("Reset heading"));
    driver.back().onTrue(named("Reset heading"));

    List<Finding> doubts = bindings.doubts();

    assertThat(doubts).allMatch(doubt -> doubt.level() == Finding.Level.DOUBT);
    assertThat(doubts)
        .extracting(Finding::subject)
        .containsExactly("operator right trigger", "driver back");
    assertThat(doubts)
        .extracting(Finding::message)
        .containsExactly(
            "operator right trigger starts Intake and Outtake while held: one press runs them"
                + " together. If that's meant, make them one command; if not, bind one to another"
                + " button",
            "driver back starts Reset heading on press twice: bind it once");
  }

  /** Released, three on one release: said in a list. */
  @Test
  void threeOnOneReleaseAreSaidTogether() {
    operator.faceUp().onFalse(named("A"));
    operator.faceUp().onFalse(named("B"));
    operator.faceUp().onFalse(named("C"));

    assertThat(bindings.doubts())
        .extracting(Finding::message)
        .containsExactly(
            "operator face up starts A, B and C on release: one release runs them together. If"
                + " that's meant, make them one command; if not, bind one to another button");
  }
}

package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Placement;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;

/**
 * The autos the robot declares, chosen and run, on the template's robot: the robot adds "Do
 * nothing" and starts with it; a fresh command is made every time autonomous starts and cancelled
 * as it ends; what an auto throws never stops the robot; the start an auto names is where the robot
 * is placed.
 */
class AutosSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  /** The simulator's robot's left start, for autos that need one, and where it is for blue. */
  private static final Place LEFT_PLACE =
      Rebuilt.starts(ExampleRobot.SIM.definition().body()).left();

  private static final Pose2d LEFT = LEFT_PLACE.on(RobotSim.FIELD, Alliance.BLUE);

  private Autos autos() {
    return sim.robot().autos();
  }

  /** An auto that does nothing for ever, counting the commands made and the loops they ran. */
  private static final class Counting {
    final List<Command> made = new ArrayList<>();
    int loops;

    Command make() {
      Command command =
          Command.noRequirements(
                  co -> {
                    while (true) {
                      loops++;
                      co.yield();
                    }
                  })
              .named("Counting");
      made.add(command);
      return command;
    }
  }

  private void autonomous(double seconds) {
    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    sim.run(seconds);
    RobotSim.enable(RobotMode.AUTONOMOUS, false);
    sim.run(0.1);
  }

  /**
   * A robot that declares none has "Do nothing", chosen, which starts and ends and moves nothing.
   */
  @Test
  void aRobotWithNoAutosHasDoNothingAndItIsChosen() {
    sim.loops(3);
    Pose2d before = sim.simulation().truePose();

    autonomous(1);

    assertThat(autos().names()).containsExactly("Do nothing");
    assertThat(autos().chosen().name()).isEqualTo("Do nothing");
    assertThat(autos().chosen().start()).as("starts where the Driver Station says").isNull();
    assertThat(autos().lastRun().orElseThrow().describe()).startsWith("Do nothing: COMPLETED in ");
    assertThat(sim.simulation().truePose()).isEqualTo(before);
  }

  /** "Do nothing" is first, then the team's in the order declared; the default stays. */
  @Test
  void declaredAutosFollowDoNothingAndItStaysTheDefault() {
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add("B", LEFT_PLACE, new Counting()::make);
          autos.add("A", LEFT_PLACE, new Counting()::make);
          return autos;
        });
    sim.loops(3);

    assertThat(autos().names()).containsExactly("Do nothing", "B", "A");
    assertThat(autos().chosen().name()).isEqualTo("Do nothing");
  }

  /** Two autos of one name, or a blank name, stop the robot starting. */
  @Test
  void aBlankNameOrTwoOfOneNameStopTheRobotStarting() {
    sim.robot();

    assertThatThrownBy(
            () ->
                sim.controls(
                    (scheduler, robot, joints, drive, driver, operator, autos) -> {
                      autos.add("Left", LEFT_PLACE, new Counting()::make);
                      autos.add("Left", LEFT_PLACE, new Counting()::make);
                      return autos;
                    }))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("two autos are named Left: name each once");
    assertThatThrownBy(
            () ->
                sim.controls(
                    (scheduler, robot, joints, drive, driver, operator, autos) -> {
                      autos.add(" ", LEFT_PLACE, new Counting()::make);
                      return autos;
                    }))
        .hasMessage("an auto needs a name");
    assertThatThrownBy(
            () ->
                sim.controls(
                    (scheduler, robot, joints, drive, driver, operator, autos) -> {
                      autos.add(Autos.DO_NOTHING, LEFT_PLACE, new Counting()::make);
                      return autos;
                    }))
        .hasMessage("two autos are named Do nothing: name each once");
  }

  /** Declared once the robot runs, the mistake is left out and said, never thrown. */
  @Test
  void anAutoDeclaredWhileTheRobotRunsIsLeftOutAndSaid() {
    sim.loops(2);

    autos().add("Late", LEFT_PLACE, new Counting()::make);
    autos().add("Late", LEFT_PLACE, new Counting()::make);

    assertThat(autos().names()).containsExactly("Do nothing", "Late");
    assertThat(autos().refusal())
        .isEqualTo("An auto was left out: two autos are named Late: name each once");
  }

  /** A fresh command each run: running autonomous twice in one boot is the same as once. */
  @Test
  void autonomousMakesAFreshCommandEachRun() {
    Counting counting = new Counting();
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add("Left", LEFT_PLACE, counting::make);
          return autos;
        });
    autos().choose("Left");
    sim.loops(3);
    assertThat(counting.made).as("nothing is made before autonomous").isEmpty();

    autonomous(0.5);
    int first = counting.loops;
    autonomous(0.5);

    assertThat(counting.made).hasSize(2);
    assertThat(counting.made.get(0)).isNotSameAs(counting.made.get(1));
    assertThat(first).as("each run ran its full half second").isGreaterThan(20);
    assertThat(counting.loops).isGreaterThan(2 * first - 5);
  }

  /**
   * What never ends is cut off by the end of autonomous, saying so, and doesn't carry into teleop.
   */
  @Test
  void theEndOfAutonomousCutsOffAnAutoThatNeverEnds() {
    Counting counting = new Counting();
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add("Forever", LEFT_PLACE, counting::make);
          return autos;
        });
    autos().choose("Forever");
    sim.loops(3);

    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    sim.run(1);
    assertThat(sim.scheduler().isRunning(counting.made.get(0))).isTrue();
    RobotSim.enable(RobotMode.TELEOPERATED, true);
    sim.run(0.2);
    int atEnd = counting.loops;
    sim.run(0.5);

    assertThat(sim.scheduler().isRunning(counting.made.get(0))).isFalse();
    assertThat(counting.loops).as("stopped as autonomous ended").isEqualTo(atEnd);
    assertThat(autos().lastRun().orElseThrow().describe())
        .startsWith("Forever: CUT OFF after 1.0")
        .contains(" s, as autonomous ended");
  }

  /** The pit's choice is taken at the start of a loop while disabled, and never while enabled. */
  @Test
  void aChoiceWaitsForTheRobotToBeDisabled() {
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add("Left", LEFT_PLACE, new Counting()::make);
          return autos;
        });
    sim.loops(2);
    RobotSim.setEnabled(true);
    sim.loops(2);

    autos().choose("Left");
    sim.loops(2);
    boolean waited = autos().chosen().name().equals("Do nothing") && autos().pending();
    RobotSim.setEnabled(false);
    sim.loops(2);

    assertThat(waited).as("enabled: the choice waits").isTrue();
    assertThat(autos().chosen().name()).as("disabled: taken").isEqualTo("Left");
    assertThat(autos().pending()).isFalse();
    assertThatThrownBy(() -> autos().choose("Nowhere"))
        .hasMessage("there's no auto Nowhere on the robot: it has Do nothing, Left");
  }

  /** An auto that can't be built runs nothing, says why, and the robot carries on. */
  @Test
  void anAutoThatThrowsAsItIsBuiltRunsNothingAndIsSaid() {
    Counting after = new Counting();
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add(
              "Broken",
              LEFT_PLACE,
              () -> {
                throw new IllegalStateException("no path named Left 9");
              });
          autos.add("Fine", LEFT_PLACE, after::make);
          return autos;
        });
    autos().choose("Broken");
    sim.loops(3);

    autonomous(0.5);

    assertThat(autos().lastRun().orElseThrow().describe())
        .isEqualTo(
            "Broken: FAILED after 0.00 s: it couldn't be built: IllegalStateException: no path named Left 9");
    assertThat(autos().failure()).contains("Broken").contains("no path named Left 9");
    assertThat(sim.scheduler().getRunningCommands()).noneMatch(c -> c.name().equals("Broken"));

    autos().choose("Fine");
    sim.loops(2);
    autonomous(0.2);
    assertThat(autos().failure()).as("the next run clears it").isNull();
    assertThat(after.made).hasSize(1);
  }

  /**
   * One that throws as it runs fails a test in the simulator, and on a robot the loop carries on.
   */
  @Test
  void anAutoThatThrowsAsItRunsIsSaidAndTheRobotCarriesOn() {
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add(
              "Throws",
              LEFT_PLACE,
              () ->
                  Command.noRequirements(
                          co -> {
                            co.yield();
                            throw new IllegalStateException("boom");
                          })
                      .named("Throws"));
          return autos;
        });
    autos().choose("Throws");
    sim.loops(3);
    RobotSim.enable(RobotMode.AUTONOMOUS, true);

    sim.loopOnRobot();
    sim.loopOnRobot();
    sim.loopOnRobot();
    sim.loopOnRobot();

    assertThat(autos().lastRun().orElseThrow().describe())
        .startsWith("Throws: FAILED after ")
        .endsWith(": IllegalStateException: boom");
    assertThat(autos().failure()).contains("Throws").contains("boom");
  }

  /** In the simulator a throw ends the loop, as any command's does, so the bug fails its test. */
  @Test
  void inTheSimulatorAnAutoThatThrowsFailsTheLoop() {
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add(
              "Throws",
              LEFT_PLACE,
              () ->
                  Command.noRequirements(
                          co -> {
                            throw new IllegalStateException("boom");
                          })
                      .named("Throws"));
          return autos;
        });
    autos().choose("Throws");
    sim.loops(3);
    RobotSim.enable(RobotMode.AUTONOMOUS, true);

    assertThatThrownBy(() -> sim.loops(3)).hasMessage("boom");
    assertThat(autos().lastRun().orElseThrow().ending()).isEqualTo(Autos.Ending.FAILED);
  }

  /**
   * An auto's start is where the robot is placed, on either alliance, and the placement check reads
   * it: ready on that start, and not once the robot is driven away from it.
   */
  @Test
  void theAutosStartIsWhereTheRobotIsPlacedAndTheCheckReadsIt() {
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add("Left", LEFT_PLACE, new Counting()::make);
          return autos;
        });
    autos().choose("Left");
    RobotSim.station(AllianceStationID.RED_3);
    sim.loops(5);
    Placement onRed = sim.placement();
    Pose2d red = sim.simulation().truePose();
    RobotSim.station(AllianceStationID.BLUE_3);
    sim.loops(5);
    Placement onBlue = sim.placement();
    Pose2d blue = sim.simulation().truePose();

    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1);
    sim.driver().notifyNewData();
    sim.run(1);
    RobotSim.setEnabled(false);
    sim.loops(3);

    assertThat(blue.getTranslation().getDistance(LEFT.getTranslation())).isLessThan(0.02);
    assertThat(red.getX()).as("red's Left is across the field").isGreaterThan(8.27);
    assertThat(onBlue.status()).isEqualTo(Placement.Status.READY);
    assertThat(onRed.status()).isEqualTo(Placement.Status.READY);
    assertThat(onBlue.describe()).startsWith("Ready on Left: ");
    assertThat(sim.placement().status())
        .as("driven off its start")
        .isEqualTo(Placement.Status.NOT_ON_A_START);
  }

  /** The pit chooses through the Pitwall page's Pit view, as it chooses a venue. */
  @Test
  void thePitChoosesThroughItsCard() throws Exception {
    sim.controls(
        (scheduler, robot, joints, drive, driver, operator, autos) -> {
          autos.add("Left", LEFT_PLACE, new Counting()::make);
          return autos;
        });
    var chooser = sim.pitwall().devices().named("auto").orElseThrow().chooser("auto").orElseThrow();

    chooser.choose("Left");
    sim.loops(2);

    assertThat(autos().chosen().name()).isEqualTo("Left");
    assertThat(sim.pitwall().devices().named("auto").orElseThrow().kind()).isEqualTo("Autonomous");
  }

  /** Without an auto's start, the Driver Station's location picks it, as it did. */
  @Test
  void doNothingStartsWhereTheDriverStationSays() {
    RobotSim.station(AllianceStationID.BLUE_1);
    sim.loops(5);
    Pose2d one = sim.simulation().truePose();
    RobotSim.station(AllianceStationID.BLUE_3);
    sim.loops(5);
    Pose2d three = sim.simulation().truePose();

    assertThat(one.getY()).isGreaterThan(three.getY() + 4);
    assertThat(sim.placement().status()).isEqualTo(Placement.Status.READY);
    assertThat(three.getX()).isCloseTo(one.getX(), within(0.01));
  }
}

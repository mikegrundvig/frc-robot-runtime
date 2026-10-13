package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.world.record.MoveReason;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;
import org.wpilib.simulation.AlertSim;
import org.wpilib.system.RobotController;

/**
 * A joint whose motor controllers stop answering, as the simulator's do when a test silences them:
 * its moves are refused, at the start or mid-way, with the reason; and one whose controllers didn't
 * take their settings, which the simulator's always do, so each here can say so.
 */
@ExtendWith(RobotSim.class)
class JointFaultsSimTest {
  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final JointSimulation simulation = new JointSimulation(ROBOT);
  private Silenceable pivotIo = new Silenceable(new JointIO() {});
  private final Joints joints =
      new Joints(
          ROBOT,
          joint -> {
            Silenceable io = new Silenceable(simulation.io(joint));
            if (joint.equals(PIVOT)) {
              pivotIo = io;
            }
            return io;
          },
          Joints.untuned(),
          scheduler);
  private final JointMechanism pivot = joints.of(PIVOT);

  /** Releases the joints' alerts, so that the next test's robot can raise its own. */
  @AfterEach
  void tearDown() {
    joints.close();
  }

  private void loop() {
    RobotSim.stepClock();
    simulation.step(RobotLoop.PERIOD_SECONDS);
    joints.read(RobotController.getTime(), true);
    scheduler.run();
    joints.write();
  }

  private void loopUntil(BooleanSupplier done) {
    for (int i = 0; i < RobotLoop.loops(3) && !done.getAsBoolean(); i++) {
      loop();
    }
  }

  /** Half a second of loops and one more: long enough for a silent controller to be called so. */
  private static final int SILENT_LOOPS = RobotLoop.loops(0.5) + 1;

  @Test
  void aJointThatIsntAnsweringRefusesItsMoves() {
    simulation.silence(PIVOT, true);
    for (int i = 0; i < SILENT_LOOPS; i++) {
      loop();
    }
    Move out = pivot.goTo(PIVOT_OUT);

    scheduler.schedule(out);
    loopUntil(() -> out.outcome().isPresent());

    assertThat(out.outcome()).contains(Outcome.REFUSED);
    ended(out).refused().beforeItBegan().because(MoveReason.JOINT_NOT_ANSWERING);

    simulation.silence(PIVOT, false);
    Move again = pivot.goTo(PIVOT_OUT);
    scheduler.schedule(again);
    loopUntil(() -> again.outcome().isPresent());

    assertThat(again.arrived()).as("answering again").isTrue();
  }

  /**
   * A move begun while a controller has been silent for less than half a second starts, since the
   * joint still reads as connected, and goes nowhere, since the controller hears nothing. It ends
   * when the controller is called silent, saying why.
   */
  @Test
  void aMoveBegunInTheFirstHalfSecondOfASilenceEndsWhenTheJointIsCalledSilent() {
    simulation.silence(PIVOT, true);
    loop();
    Move out = pivot.goTo(PIVOT_OUT);

    scheduler.schedule(out);
    loopUntil(() -> out.outcome().isPresent());

    assertThat(out.outcome()).contains(Outcome.REFUSED);
    ended(out).refused().partWay().because(MoveReason.JOINT_NOT_ANSWERING);
    assertThat(Math.toDegrees(simulation.position(PIVOT)))
        .as("it was not driven toward out, at 130°")
        .isLessThan(5);
  }

  /**
   * A motor controller that didn't take its settings (the survey found configs built and never
   * applied) has no current limits or ratio: the joint is faulted, and nothing drives it.
   */
  @Test
  void aJointWhoseMotorDidntTakeItsSettingsIsFaulted() {
    pivotIo.unconfigured = "motor 11";
    loop();
    Move out = pivot.goTo(PIVOT_OUT);

    scheduler.schedule(out);
    loopUntil(() -> out.outcome().isPresent());

    assertThat(pivot.state()).isEqualTo(JointMechanism.State.FAULTED);
    assertThat(out.outcome()).contains(Outcome.REFUSED);
    ended(out)
        .refused()
        .because(MoveReason.JOINT_FAULTED)
        .saying(
            "pivot's motor 11 didn't take its settings: check its CAN ID and wiring, then restart the robot");
  }

  @Test
  void aJointThatFallsSilentMidWayEndsItsMove() {
    Move out = pivot.goTo(PIVOT_OUT);
    scheduler.schedule(out);
    for (int i = 0; i < 10; i++) {
      loop();
    }

    simulation.silence(PIVOT, true);
    loopUntil(() -> out.outcome().isPresent());

    assertThat(out.outcome()).contains(Outcome.REFUSED);
    ended(out).refused().partWay().because(MoveReason.JOINT_NOT_ANSWERING);
  }

  /** One loop, with the robot enabled or not. */
  private void loop(boolean enabled) {
    RobotSim.stepClock();
    simulation.step(RobotLoop.PERIOD_SECONDS);
    joints.read(RobotController.getTime(), enabled);
    scheduler.run();
    joints.write();
  }

  /** What the pivot did over some loops: what held it each loop, and each doubt said of it. */
  private record Idled(List<String> heldBy, List<String> doubts) {}

  private Idled idleFor(int loops, boolean enabled) {
    Set<String> heldBy = new LinkedHashSet<>();
    Set<String> doubts = new LinkedHashSet<>();
    for (int i = 0; i < loops; i++) {
      loop(enabled);
      heldBy.add(pivot.heldBy());
      doubts.addAll(joints.switchDoubts());
    }
    return new Idled(List.copyOf(heldBy), List.copyOf(doubts));
  }

  /**
   * A pivot whose position reads as something that isn't a number, left idle. While it already
   * holds somewhere it holds on there. Enabled with nowhere to hold, it's neutral, saying why, and
   * its motor controller is never told anything that isn't a number; reading a number again, it
   * holds where it is. All the while its doubt says the reading isn't a number, and is gone with
   * it. Until 2026-10-11 it took a new hold at NaN every loop, and its motor controller was sent
   * there.
   */
  @Test
  void aJointWhosePositionReadsNoNumberHoldsOnOrGoesNeutralAndSaysSo() {
    String doubt =
        "pivot's position reads NaN, which isn't a number: pivot is neutral while it has nowhere to"
            + " hold; check its encoder and its motor controller's settings";
    Idled whole = idleFor(5, true);
    double held = pivot.goal();

    pivotIo.readsNoNumber = true;
    Idled holdingOn = idleFor(10, true);
    double stillHeld = pivot.goal();
    idleFor(2, false);
    Idled nowhere = idleFor(10, true);
    pivotIo.readsNoNumber = false;
    Idled wholeAgain = idleFor(5, true);

    assertThat(whole).isEqualTo(new Idled(List.of(""), List.of()));
    assertThat(holdingOn)
        .as("it holds on where it held")
        .isEqualTo(new Idled(List.of(""), List.of(doubt)));
    assertThat(stillHeld).isEqualTo(held);
    assertThat(nowhere)
        .as("enabled with nowhere to hold")
        .isEqualTo(new Idled(List.of("pivot's position isn't a number"), List.of(doubt)));
    assertThat(wholeAgain).isEqualTo(new Idled(List.of(""), List.of()));
    assertThat(pivot.goal())
        .as("it holds where it read a number again, having sunk while it was neutral")
        .isCloseTo(pivot.position(), within(1e-3));
    assertThat(pivotIo.notNumbers).as("told its motor controller").isEmpty();
    assertThat(
            Arrays.stream(AlertSim.getActive())
                .filter(alert -> alert.group.equals("Sensors"))
                .map(alert -> alert.text))
        .as("the doubt's alert fell with it")
        .isEmpty();
  }

  /**
   * On a robot without its wiring, a joint has no hardware, and nothing answers. (In a replay its
   * readings, whether it answered among them, come from the log.)
   */
  @Test
  void aJointWithNoHardwareRefusesItsMoves() {
    Scheduler alone = Scheduler.createIndependentScheduler();
    Joints none = new Joints(ROBOT, joint -> new JointIO() {}, Joints.untuned(), alone);
    Move out = none.of(PIVOT).goTo(PIVOT_OUT);

    alone.schedule(out);
    for (int i = 0; i < 3; i++) {
      RobotSim.stepClock();
      none.read(RobotController.getTime(), true);
      alone.run();
      none.write();
    }

    assertThat(out.outcome()).contains(Outcome.REFUSED);
  }
}

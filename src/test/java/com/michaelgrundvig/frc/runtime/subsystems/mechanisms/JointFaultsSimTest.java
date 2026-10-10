package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;
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

  @Test
  void aJointThatIsntAnsweringRefusesItsMoves() {
    simulation.silence(PIVOT, true);
    loop();
    Move out = pivot.goTo(PIVOT_OUT);

    scheduler.schedule(out);
    loopUntil(() -> out.outcome().isPresent());

    assertThat(out.outcome()).contains(Outcome.REFUSED);
    assertThat(out.reason())
        .isEqualTo("pivot → out: REFUSED: pivot isn't answering: its motor controllers are silent");

    simulation.silence(PIVOT, false);
    Move again = pivot.goTo(PIVOT_OUT);
    scheduler.schedule(again);
    loopUntil(() -> again.outcome().isPresent());

    assertThat(again.arrived()).as("answering again").isTrue();
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
    assertThat(out.reason())
        .isEqualTo(
            "pivot → out: REFUSED: pivot's motor 11 didn't take its settings: check its CAN ID and"
                + " wiring, then restart the robot");
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
    assertThat(out.reason())
        .isEqualTo(
            "pivot → out: REFUSED: can't go on: pivot isn't answering: its motor controllers are"
                + " silent");
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

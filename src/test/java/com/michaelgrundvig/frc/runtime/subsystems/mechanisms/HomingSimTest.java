package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.homed;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * Joints read by their motors alone, powered on away from where their definition rests them: they
 * home into a hard stop on the first enable, and moves wait for them.
 */
@ExtendWith(RobotSim.class)
class HomingSimTest {
  /** A robot with one of its joints homed by a recipe instead of stowed by hand. */
  private static RobotDefinition homing(RobotDefinition robot, Joint joint, Homing recipe) {
    return robot.with(joint.retuned(control -> homed(control, recipe)));
  }

  /**
   * Powered on 0.3 m up, the elevator reads 0 m, its reference. Homing pushes it down into its stop
   * and takes that as 0 m; the move to "clear" waits for it, then takes it to the real 0.4 m.
   */
  @Test
  void aJointHomesIntoItsStopOnTheFirstEnableAndMovesWaitForIt() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    Gate ready = elevator.isReady();
    sim.simulation.powerOnAt(ELEVATOR, 0.3);
    Move clear = elevator.goTo(ELEVATOR_CLEAR);

    sim.scheduler.schedule(clear);
    sim.loop();
    JointMechanism.State first = elevator.state();
    boolean readyFirst = ready.getAsBoolean();
    double took = sim.runUntil(() -> clear.outcome().isPresent(), 4);

    assertThat(first).isEqualTo(JointMechanism.State.HOMING);
    assertThat(readyFirst).isFalse();
    assertThat(clear.arrived()).isTrue();
    assertThat(took).as("homing, then the move").isGreaterThan(0.6).isLessThan(3);
    assertThat(elevator.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(ready.getAsBoolean()).isTrue();
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0.4, within(0.01));
    assertThat(elevator.position()).isCloseTo(0.4, within(0.01));
  }

  /**
   * Powered on at its top, 1.2 m up, the elevator takes over 4 s to come down at 2 V: past the 3 s
   * a homing once had, inside the 9.5 s worked out from its travel, twice the 4.5 s its model takes
   * at 2 V and half a second more. While it homes, its state says so.
   */
  @Test
  void aJointHomesFromItsFarEndWithinTheTimeItsTravelTakes() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 1.2);

    sim.loop();
    String homing = elevator.stateShown();
    double took = sim.runUntil(() -> elevator.state() != JointMechanism.State.HOMING, 10);
    sim.loop();

    assertThat(Homing.travelSeconds(ROBOT.math(), ELEVATOR, 2)).isCloseTo(4.51, within(0.01));
    assertThat(homing).isEqualTo("HOMING: toward its lower stop, for at most 9.5 s");
    assertThat(elevator.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(took).isGreaterThan(3).isLessThan(9.5);
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0, within(0.005));
    assertThat(elevator.position()).isCloseTo(0, within(0.005));
  }

  /** Disabled, it waits to home, and so do moves on it; enabled, it homes. */
  @Test
  void itHomesOnlyOnceEnabled() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.setEnabled(false);
    Move clear = elevator.goTo(ELEVATOR_CLEAR);
    sim.scheduler.schedule(clear);

    sim.run(1);
    JointMechanism.State disabled = elevator.state();
    sim.setEnabled(true);
    sim.runUntil(() -> clear.outcome().isPresent(), 3);

    assertThat(disabled).isEqualTo(JointMechanism.State.UNHOMED);
    assertThat(clear.arrived()).isTrue();
  }

  /**
   * A move canceled while it still waits for its joint to home says how long it waited: here nine
   * loops, from its first to the one it is canceled in. The robot had been on for a second by then,
   * and the move had never begun, so the time is the move's own and not the robot's.
   */
  @Test
  void aMoveCanceledWhileItWaitsForHomingSaysHowLongItWaited() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 1.2);
    sim.setEnabled(false);
    sim.run(1);
    sim.setEnabled(true);
    Move clear = elevator.goTo(ELEVATOR_CLEAR);
    sim.scheduler.schedule(clear);
    sim.run(0.2);
    JointMechanism.State waitingOn = elevator.state();

    sim.scheduler.cancel(clear);
    sim.loop();

    assertThat(waitingOn).isEqualTo(JointMechanism.State.HOMING);
    assertThat(clear.outcome()).contains(Outcome.CANCELED);
    assertThat(clear.reason()).startsWith("elevator → clear: CANCELED after 0.18 s");
  }

  /**
   * Powered on near its top, with a homing given too short a time to reach the bottom: it's
   * faulted, says why, and refuses its moves; homed again from its stop, it's ready. The time given
   * wins over the one its travel would give it.
   */
  @Test
  void aHomingThatFindsNoStopFaultsTheJointUntilItHomesAgain() {
    Homing quick = Homing.toHardStop(Sensor.Switch.Limit.End.LOWER, Seconds.of(0.3));
    JointsSimHarness sim = new JointsSimHarness(homing(ROBOT, ELEVATOR, quick));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 1.1);

    sim.run(0.5);
    Move clear = elevator.goTo(ELEVATOR_CLEAR);
    sim.scheduler.schedule(clear);
    sim.runUntil(() -> clear.outcome().isPresent(), 1);
    sim.simulation.place(ELEVATOR, 0); // lowered by hand
    Move home = elevator.home();
    sim.scheduler.schedule(home);
    double took = sim.runUntil(() -> home.outcome().isPresent(), 2);

    String why = "elevator's homing found no stop in 0.3 s, pushing toward its lower one";
    assertThat(clear.outcome()).contains(Outcome.REFUSED);
    assertThat(clear.reason()).isEqualTo("elevator → clear: REFUSED: " + why);
    assertThat(home.arrived()).isTrue();
    assertThat(took).isCloseTo(0.30, within(0.03));
    assertThat(elevator.state()).isEqualTo(JointMechanism.State.READY);
  }

  /**
   * A preset that cancels the elevator's homing, as one bound to a button would, homes it again
   * itself and arrives: a team's elite robots re-arm their homing the same way. Until 2026-10-07 it
   * waited for an enable, which never comes in a match, with no deadline: forever.
   */
  @Test
  void aPresetThatCancelsHomingHomesItAgainAndArrives() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 0.3);
    Move home = elevator.home();
    sim.run(home, 0.1);
    Move clear = elevator.goTo(ELEVATOR_CLEAR);
    sim.scheduler.schedule(clear);
    sim.loop();
    JointMechanism.State again = elevator.state();
    double took = sim.runUntil(() -> clear.outcome().isPresent(), 4);

    assertThat(home.outcome()).contains(Outcome.CANCELED);
    assertThat(again).as("homing again at once").isEqualTo(JointMechanism.State.HOMING);
    assertThat(clear.arrived()).isTrue();
    assertThat(took).as("homing, then the move").isLessThan(3);
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0.4, within(0.01));
    assertThat(elevator.position()).isCloseTo(0.4, within(0.01));
  }

  /**
   * Disabled mid-homing, the elevator is unhomed and a move on it waits, its deadline not yet
   * started; enabled again, it homes from where it is, and the move arrives.
   */
  @Test
  void disabledMidHomingItHomesOnTheNextEnable() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 0.3);
    sim.run(0.1);
    JointMechanism.State homing = elevator.state();
    sim.setEnabled(false);
    Move clear = elevator.goTo(ELEVATOR_CLEAR);
    sim.run(clear, 2);
    JointMechanism.State disabled = elevator.state();
    boolean waiting = clear.outcome().isEmpty();
    sim.setEnabled(true);
    sim.runUntil(() -> clear.outcome().isPresent(), 4);

    assertThat(homing).isEqualTo(JointMechanism.State.HOMING);
    assertThat(disabled).isEqualTo(JointMechanism.State.UNHOMED);
    assertThat(waiting).isTrue();
    assertThat(clear.arrived()).isTrue();
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0.4, within(0.01));
  }

  /**
   * A follow canceled while it still waits for its joint to home has chased nothing, so its outcome
   * has no target to report: it says where the joint read, as any move of a joint held at positions
   * does.
   */
  @Test
  void aFollowCanceledWhileItWaitsForHomingSaysWhereItsJointWas() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 0.3);
    Move follow = elevator.follow(() -> Meters.of(0.5));

    sim.run(follow, 0.1);
    JointMechanism.State waitingOn = elevator.state();
    sim.scheduler.cancel(follow);

    assertThat(waitingOn).isEqualTo(JointMechanism.State.HOMING);
    assertThat(follow.reason())
        .matches("elevator follows a target: CANCELED after 0\\.08 s, elevator at -?0(\\.\\d+)? m");
  }

  /**
   * A homing that faults ends what waits on it, REFUSED with the fault: a move waiting for the
   * first enable's homing, {@code home()} itself, and a preset that canceled a homing and homed the
   * elevator again itself, which so waits no longer than its homing may take.
   */
  @Test
  void aHomingThatFaultsRefusesWhatWaitsOnIt() {
    Homing quick = Homing.toHardStop(Sensor.Switch.Limit.End.LOWER, Seconds.of(0.3));
    JointsSimHarness sim = new JointsSimHarness(homing(ROBOT, ELEVATOR, quick));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 1.1);
    Move clear = elevator.goTo(ELEVATOR_CLEAR);
    sim.scheduler.schedule(clear);
    double waited = sim.runUntil(() -> clear.outcome().isPresent(), 2);
    sim.simulation.place(ELEVATOR, 1.1);
    Move home = elevator.home();
    sim.scheduler.schedule(home);
    double homed = sim.runUntil(() -> home.outcome().isPresent(), 2);
    sim.simulation.place(ELEVATOR, 1.1);
    Move canceled = elevator.home();
    sim.run(canceled, 0.1);
    Move preset = elevator.goTo(ELEVATOR_CLEAR);
    sim.scheduler.schedule(preset);
    double bounded = sim.runUntil(() -> preset.outcome().isPresent(), 2);

    String why = "elevator's homing found no stop in 0.3 s, pushing toward its lower one";
    assertThat(clear.reason()).isEqualTo("elevator → clear: REFUSED: " + why);
    assertThat(waited).isCloseTo(0.32, within(0.03));
    assertThat(home.reason()).isEqualTo("elevator homes: REFUSED: " + why);
    assertThat(homed).isCloseTo(0.32, within(0.03));
    assertThat(canceled.outcome()).contains(Outcome.CANCELED);
    assertThat(preset.reason()).isEqualTo("elevator → clear: REFUSED: " + why);
    assertThat(bounded).as("its homing's 0.3 s").isCloseTo(0.32, within(0.03));
  }

  /** {@code home()} while disabled waits, unhomed, and homes once the robot is enabled. */
  @Test
  void homeWhileDisabledHomesOnceEnabled() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 0.3);
    sim.setEnabled(false);
    Move home = elevator.home();
    sim.run(home, 1);
    JointMechanism.State disabled = elevator.state();
    boolean waiting = home.outcome().isEmpty();
    sim.setEnabled(true);
    sim.runUntil(() -> home.outcome().isPresent(), 3);
    sim.loop(); // its motor's new count, read from the loop after it's found

    assertThat(disabled).isEqualTo(JointMechanism.State.UNHOMED);
    assertThat(waiting).isTrue();
    assertThat(home.arrived()).isTrue();
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0, within(0.005));
    assertThat(elevator.position()).isCloseTo(0, within(0.005));
  }

  @Test
  void aJointThatNeedsNoHomingIsHomeAtOnce() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT);
    Move home = sim.joints.of(PIVOT).home();

    sim.run(home, 0.1);

    assertThat(home.reason()).isEqualTo("pivot homes: ARRIVED in 0.00 s");
  }

  /**
   * The pivot is really past "run", but until it's homed nobody knows: the roller's interlock fails
   * safe and holds it. Homed into its top stop, the pivot is known past "run", and the roller runs.
   */
  @Test
  void anInterlockOnAJointNotYetHomedHoldsFailSafe() {
    RobotDefinition guarded =
        RobotDefinitionBuilder.builder(ROBOT)
            .interlocks(List.of(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN))))
            .build();
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(guarded, PIVOT, Homing.toHardStop(Sensor.Switch.Limit.End.UPPER)));
    JointMechanism pivot = sim.joints.of(PIVOT);
    JointMechanism roller = sim.joints.of(ROLLER);
    Gate run = sim.joints.gate(PIVOT.atLeast(PIVOT_RUN));
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(120));
    sim.scheduler.schedule(roller.run(ROLLER_INTAKING));

    sim.run(0.1);
    JointMechanism.State homing = pivot.state();
    double whileHoming = sim.simulation.velocity(ROLLER);
    String heldBy = roller.heldBy();
    String unknown = run.why();
    sim.run(1.5);

    assertThat(homing).isEqualTo(JointMechanism.State.HOMING);
    assertThat(whileHoming).isCloseTo(0, within(0.5));
    assertThat(heldBy).isEqualTo("roller runs only when pivot at least run");
    assertThat(unknown).isEqualTo("pivot at least run is false: pivot doesn't know where it is");
    assertThat(run.getAsBoolean()).as("known past run, once homed").isTrue();
    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(Math.toDegrees(pivot.position())).isCloseTo(135, within(1.0));
    assertThat(sim.simulation.velocity(ROLLER)).isGreaterThan(100);
  }

  @Test
  void aStillJointAtItsStopHomesInAQuarterSecond() {
    JointsSimHarness sim =
        new JointsSimHarness(
            homing(ROBOT, ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);

    double took = sim.runUntil(() -> elevator.state() == JointMechanism.State.READY, 1);

    assertThat(took)
        .as("still for a quarter second, read a loop late")
        .isCloseTo(0.30, within(0.02));
  }
}

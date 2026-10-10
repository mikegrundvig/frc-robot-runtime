package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.homed;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Sensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A joint whose leading motor controller restarts, as one whose breaker resets or whose power
 * connector comes loose does: it counts its motor from 0 again, and the joint finds where it is
 * again as its kind can. An absolute encoder tells it again; a homed joint homes again; one assumed
 * at rest is faulted, since only power-on puts it there; a roller runs on.
 */
@ExtendWith(RobotSim.class)
class ControllerRestartSimTest {
  /** The pivot, its position from a through-bore on Systemcore's port 1, which the robot reads. */
  private static final Joint.Revolute ON_PORT =
      PIVOT
          .absoluteEncoder(
              MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.3)).inverted())
          .retuned(old -> control(30, 1, profile(8, 30)));

  /** The pivot on a TalonFX that reads a CANcoder, seeded from it once at startup. */
  private static final Joint.Revolute ON_CANCODER =
      PIVOT
          .motors(motors -> motors.on(TALON_FX.can(21)))
          .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.3)))
          .retuned(old -> control(30, 1, profile(8, 30)));

  /** The pivot homed into its bottom stop. */
  private static final Joint.Revolute HOMED =
      PIVOT.retuned(control -> homed(control, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));

  private static double degrees(double radians) {
    return Math.toDegrees(radians);
  }

  /** Moves the pivot out, to 130°, and restarts its controller there. */
  private static JointMechanism outThenRestarted(JointsSimHarness sim) {
    JointMechanism pivot = sim.joints.of(PIVOT);
    Move out = pivot.goTo(PIVOT_OUT);
    sim.scheduler.schedule(out);
    sim.runUntil(() -> out.outcome().isPresent(), 8);
    assertThat(out.arrived()).as("out first").isTrue();
    sim.simulation.restart(PIVOT);
    return pivot;
  }

  /**
   * The fixtures' pivot, taken to be at rest as the robot starts, reads 0° once its controller
   * restarts at 130°: it's faulted, saying why and what to do, and its moves are refused.
   */
  @Test
  void aJointAssumedAtRestIsFaulted() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT);
    JointMechanism pivot = outThenRestarted(sim);

    sim.loop();
    Move run = pivot.goTo(PIVOT_RUN);
    sim.scheduler.schedule(run);
    sim.run(0.1);

    assertThat(pivot.state()).isEqualTo(JointMechanism.State.FAULTED);
    assertThat(pivot.fault())
        .isEqualTo(
            "pivot's motor controller restarted, so where pivot is isn't known: it's taken to be"
                + " at rest only as the robot starts. Stow it, then restart the robot code");
    assertThat(run.outcome()).contains(Outcome.REFUSED);
  }

  /**
   * The pivot homed into its bottom stop, restarted at 130°, is unhomed: the next move homes it
   * again first, as after a canceled homing, then takes it to the real 110°.
   */
  @Test
  void aHomedJointHomesAgainOnItsNextMove() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT.with(HOMED));
    JointMechanism pivot = outThenRestarted(sim);

    sim.loop();
    JointMechanism.State restarted = pivot.state();
    Move run = pivot.goTo(PIVOT_RUN);
    sim.scheduler.schedule(run);
    sim.runUntil(() -> run.outcome().isPresent(), 8);

    assertThat(restarted).isEqualTo(JointMechanism.State.UNHOMED);
    assertThat(run.arrived()).isTrue();
    assertThat(degrees(sim.simulation.position(PIVOT))).isCloseTo(110, within(1.0));
  }

  /** Restarted while it homes, its homing starts over, and finds the stop from where it is. */
  @Test
  void aHomingARestartCutsShortStartsOver() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT.with(HOMED));
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(60));
    Move run = pivot.goTo(PIVOT_RUN);
    sim.scheduler.schedule(run);
    sim.run(0.1);
    JointMechanism.State homing = pivot.state();

    sim.simulation.restart(PIVOT);
    sim.loop();
    JointMechanism.State restarted = pivot.state();
    sim.runUntil(() -> run.outcome().isPresent(), 8);

    assertThat(homing).isEqualTo(JointMechanism.State.HOMING);
    assertThat(restarted).isEqualTo(JointMechanism.State.HOMING);
    assertThat(run.arrived()).isTrue();
    assertThat(degrees(sim.simulation.position(PIVOT))).isCloseTo(110, within(1.0));
  }

  /**
   * Each of its devices' restarts is counted, as its log says them, the leader first, then each
   * follower, then its CANcoder: the pivot on two TalonFXs with a CANcoder, its leader restarted
   * twice.
   */
  @Test
  void eachDevicesRestartsAreCounted() {
    JointsSimHarness sim =
        new JointsSimHarness(
            ROBOT.with(
                unmotored(PIVOT)
                    .motors(
                        Motor.KRAKEN_X60
                            .through(gearing(25))
                            .on(TALON_FX.can(21), TALON_FX.can(22)))
                    .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.3)))
                    .retuned(old -> control(30, 1, profile(8, 30)))));
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.run(0.1);

    sim.simulation.restart(PIVOT);
    sim.run(0.1);
    sim.simulation.restart(PIVOT);
    sim.run(0.1);

    assertThat(pivot.restarts()).containsExactly(2, 0, 0);
  }

  /**
   * The pivot whose encoder the robot reads tells its motor where it is again from the encoder, in
   * the loop its controller says it restarted, reads it from the next, and holds there, rather than
   * being driven toward its goal from 0°.
   */
  @Test
  void aJointWithAnEncoderTheRobotReadsIsToldWhereItIsAgain() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT.with(ON_PORT));
    JointMechanism pivot = outThenRestarted(sim);

    sim.loop();
    sim.loop();
    double told = pivot.position();
    sim.run(0.5);

    assertThat(degrees(told)).isCloseTo(130, within(1.0));
    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(degrees(sim.simulation.position(PIVOT))).isCloseTo(130, within(1.0));
  }

  /**
   * The pivot on a TalonFX that reads its CANcoder is told where it is again by its IO, which seeds
   * its rotor from the CANcoder again: it stays ready, and holds.
   */
  @Test
  void aJointWhoseControllerReadsItsEncoderStaysReady() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT.with(ON_CANCODER));
    JointMechanism pivot = outThenRestarted(sim);

    sim.loop();
    sim.run(0.5);

    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(degrees(pivot.position())).isCloseTo(130, within(1.0));
    assertThat(degrees(sim.simulation.position(PIVOT))).isCloseTo(130, within(1.0));
  }

  /**
   * The fixtures' turret, which turns freely, on a TalonFX that reads a CANcoder turning with it.
   */
  private static final Joint.Revolute TURRET_ON_CANCODER =
      TURRET
          .motors(motors -> motors.on(TALON_FX.can(23)))
          .absoluteEncoder(CANCODER.can(35).offset(Rotations.of(0)))
          .retuned(old -> control(20, 0.5, profile(3, 10)));

  /**
   * The turret, held a turn and a quarter round from where it rests, is seeded again from its
   * CANcoder as its controller restarts: at 90°, inside the CANcoder's turn. It holds where it is,
   * its profile starting again from there. Until 2026-10-08 its profile went on from 450°, a turn
   * away as the controller now counted, and the turret spun a whole turn to get there.
   */
  @Test
  void aJointThatTurnsPastItsEncodersTurnHoldsWhereItIsAsItsControllerRestarts() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT.with(TURRET_ON_CANCODER));
    JointMechanism turret = sim.joints.of(TURRET);
    sim.simulation.place(TURRET, Math.toRadians(450));
    sim.run(0.5);

    sim.simulation.restart(TURRET);
    sim.run(1);

    assertThat(turret.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(degrees(turret.position())).isCloseTo(90, within(1.0));
    assertThat(degrees(sim.simulation.position(TURRET))).isCloseTo(450, within(1.0));
  }

  /** A roller restarted while it runs runs on: it has no place to lose. */
  @Test
  void aRollerRunsOn() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT);
    JointMechanism roller = sim.joints.of(ROLLER);
    sim.scheduler.schedule(roller.run(ROLLER_INTAKING));
    sim.run(1);

    sim.simulation.restart(ROLLER);
    sim.run(1);

    assertThat(roller.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(roller.velocity()).as("1500 rpm").isCloseTo(1500 * 2 * Math.PI / 60, within(5.0));
  }
}

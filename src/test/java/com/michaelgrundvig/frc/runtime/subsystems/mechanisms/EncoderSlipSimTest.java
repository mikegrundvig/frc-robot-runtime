package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A joint whose absolute encoder and motor's count disagree, past what backlash and the encoder's
 * counts explain, for a quarter second: one of them slipped (a magnet loose on its shaft, a belt
 * that skipped, an encoder frozen), so where the joint is isn't known. It's faulted, its moves are
 * refused, and it holds where its motor says, until they agree again; one the robot reads the
 * encoder of is homed again from it. A CANcoder that says its magnet is bad isn't trusted either.
 */
@ExtendWith(RobotSim.class)
class EncoderSlipSimTest {
  /** The pivot's through-bore on port 1, mounted reversed and turned a third of a turn. */
  private static final MechanismWiring.PortEncoder PORT_1 =
      MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.3)).inverted();

  /** Its encoder tells its motor where it is, once. */
  private static final Joint.Revolute SEEDED =
      PIVOT.absoluteEncoder(PORT_1).retuned(old -> control(30, 1, profile(8, 30)));

  /** Its loop closes on its encoder, the robot's. */
  private static final Joint.Revolute CLOSED = SEEDED.encoderOn(PORT_1.closesLoop());

  /** On a SPARK MAX, a through-bore on its data port, read once. */
  private static final Joint.Revolute ON_DATA_PORT =
      unmotored(PIVOT)
          .motor(Motor.NEO, gearing(25), SPARK_MAX.can(21))
          .absoluteEncoder(DATA_PORT.offset(Rotations.of(0.1)))
          .retuned(old -> control(30, 1, profile(8, 30)));

  /** On a TalonFX, a CANcoder it's seeded from once, read beside its rotor. */
  private static final Joint.Revolute ON_CANCODER =
      PIVOT
          .motorsOn(TALON_FX.can(21))
          .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.3)))
          .retuned(old -> control(30, 1, profile(8, 30)));

  private static double degrees(double radians) {
    return Math.toDegrees(radians);
  }

  /** The pivot, out at 130° and holding there. */
  private static JointsSimHarness out(Joint.Revolute pivot) {
    JointsSimHarness sim = new JointsSimHarness(ROBOT.with(pivot));
    Move out = sim.joints.of(PIVOT).goTo(PIVOT_OUT);
    sim.scheduler.schedule(out);
    sim.runUntil(() -> out.outcome().isPresent(), 3);
    sim.run(0.5);
    assertThat(out.arrived()).as("out first").isTrue();
    return sim;
  }

  /** Runs loops until the pivot's faulted, at most 1 s: how long it took, or NaN. */
  private static double untilFaulted(JointsSimHarness sim) {
    JointMechanism pivot = sim.joints.of(PIVOT);
    return sim.runUntil(() -> pivot.state() == JointMechanism.State.FAULTED, 1);
  }

  /**
   * Its encoder's magnet turns 20° on its shaft: a quarter second on, and a loop, the pivot is
   * faulted, saying so and what to do, its moves refused, and it holds where its motor says; put
   * back, the two agree, and it's ready again, where it is.
   */
  @Test
  void aSlippedEncoderFaultsItsJointUntilTheyAgree() {
    JointsSimHarness sim = out(SEEDED);
    JointMechanism pivot = sim.joints.of(PIVOT);

    sim.simulation.slip(PIVOT, Math.toRadians(20));
    double took = untilFaulted(sim);
    String why = pivot.fault();
    Move run = pivot.goTo(PIVOT_RUN);
    sim.scheduler.schedule(run);
    sim.run(0.5);
    double held = degrees(sim.simulation.position(PIVOT));
    sim.simulation.slip(PIVOT, 0);
    sim.loop();

    assertThat(took).as("the first loop a quarter second on").isCloseTo(0.28, within(1e-9));
    assertThat(why)
        .isEqualTo(
            "pivot's absolute encoder and its motor's count are 20° apart, more than 5°: one of"
                + " them slipped, a magnet loose on its shaft or a belt that skipped, so where"
                + " pivot is isn't known, and pivot holds where its motor says until they agree."
                + " Check both, then home it to tell its motor where it is from the encoder again");
    assertThat(run.outcome()).contains(Outcome.REFUSED);
    assertThat(held).isCloseTo(130, within(1.0));
    assertThat(pivot.state()).as("agreeing again").isEqualTo(JointMechanism.State.READY);
  }

  /**
   * Its motor's count slips instead, reading 1.5° with the pivot at 130°, as a belt that skipped
   * might leave it: faulted the same, and homed again, its motor is told where it is from the
   * encoder, and a move takes it to the real 110°.
   */
  @Test
  void aMotorCountThatSlippedIsToldWhereItIsByHomingAgain() {
    JointsSimHarness sim = out(SEEDED);
    JointMechanism pivot = sim.joints.of(PIVOT);

    sim.simulation.powerOnAt(PIVOT, Math.toRadians(130));
    double took = untilFaulted(sim);
    Move home = pivot.home();
    sim.scheduler.schedule(home);
    sim.runUntil(() -> home.outcome().isPresent(), 1);
    Move run = pivot.goTo(PIVOT_RUN);
    sim.scheduler.schedule(run);
    sim.runUntil(() -> run.outcome().isPresent(), 3);

    assertThat(took).isNotNaN();
    assertThat(home.arrived()).isTrue();
    assertThat(run.arrived()).isTrue();
    assertThat(degrees(sim.simulation.position(PIVOT))).isCloseTo(110, within(1.0));
  }

  /**
   * Closed on its encoder, whose magnet then slips 20°, the pivot is driven to where the slipped
   * encoder puts its goal, 20° short, within the quarter second, and then, faulted, held where its
   * motor says, not chased on.
   */
  @Test
  void aLoopClosedOnASlippedEncoderIsHeldOnItsMotor() {
    JointsSimHarness sim = out(CLOSED);

    sim.simulation.slip(PIVOT, Math.toRadians(20));
    double took = untilFaulted(sim);
    double faulted = degrees(sim.simulation.position(PIVOT));
    sim.run(1);

    assertThat(took).isNotNaN();
    assertThat(faulted).as("driven wrong first").isBetween(105.0, 130.0);
    assertThat(degrees(sim.simulation.position(PIVOT))).isCloseTo(faulted, within(2.0));
  }

  /**
   * Closed on an encoder that freezes, the pivot sent in moves by its motor while the encoder says
   * it's still, the robot's loop pushing it on as it never gets there: faulted a quarter second
   * after the two are far enough apart, the move ends, and it's held where its motor said it would
   * stop, which at a full push may be its stop.
   */
  @Test
  void aLoopClosedOnAFrozenEncoderIsFaultedOnceItMoves() {
    JointsSimHarness sim = out(CLOSED);
    JointMechanism pivot = sim.joints.of(PIVOT);

    sim.simulation.freeze(PIVOT, true);
    Move run = pivot.goTo(PIVOT_RUN);
    sim.scheduler.schedule(run);
    double took = untilFaulted(sim);
    sim.run(1);
    double held = degrees(sim.simulation.position(PIVOT));
    sim.run(0.5);

    assertThat(took).isLessThan(0.5);
    assertThat(run.outcome()).contains(Outcome.REFUSED);
    assertThat(run.reason())
        .contains("can't go on: pivot's absolute encoder and its motor's count");
    assertThat(degrees(sim.simulation.position(PIVOT))).as("held").isCloseTo(held, within(0.5));
  }

  /**
   * An encoder its motor controller reads, on a SPARK's data port or a CANcoder beside a TalonFX's
   * rotor, slipped 20°: faulted the same, told to restart the robot code, since its motor is told
   * where it is from the encoder only as the robot starts.
   */
  @Test
  void anEncoderItsControllerReadsIsCheckedTheSame() {
    for (Joint.Revolute pivot : List.of(ON_DATA_PORT, ON_CANCODER)) {
      JointsSimHarness sim = out(pivot);

      sim.simulation.slip(PIVOT, Math.toRadians(20));
      double took = untilFaulted(sim);

      assertThat(took).as(pivot.driven().absoluteEncoder().toString()).isNotNaN();
      assertThat(sim.joints.of(PIVOT).fault())
          .endsWith("Check both, then restart the robot code to tell its motor where it is again");
    }
  }

  /**
   * A turret that turns freely, its encoder on its axle read by the robot or on its SPARK's data
   * port, sent on past 180°, to 190°, where its encoder wraps to -170° while its motor's count goes
   * on: the two agree there. Back at 170°, its encoder slips 20° on its shaft, across its wrap:
   * they're 20° apart, not 340°, and it's faulted for that, held where it is.
   */
  @Test
  void aJointThatTurnsFreelyIsCheckedTheNearestWayRound() {
    Joint.Revolute onPort =
        TURRET
            .absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0)))
            .retuned(old -> control(20, 0.5, profile(3, 10)));
    Joint.Revolute onDataPort =
        unmotored(TURRET)
            .motor(Motor.NEO, gearing(50), SPARK_MAX.can(22))
            .absoluteEncoder(DATA_PORT.offset(Rotations.of(0)))
            .retuned(old -> control(20, 0.5, profile(3, 10)));
    for (Joint.Revolute turret : List.of(onPort, onDataPort)) {
      JointsSimHarness sim = new JointsSimHarness(ROBOT.with(turret));
      JointMechanism mechanism = sim.joints.of(TURRET);
      double past = Double.NaN;
      JointMechanism.State there = null;
      for (double heading : new double[] {170, -170, 170}) {
        Move move = mechanism.goTo(Degrees.of(heading));
        sim.scheduler.schedule(move);
        sim.runUntil(() -> move.outcome().isPresent(), 3);
        sim.run(0.5);
        if (heading < 0) {
          past = degrees(sim.simulation.position(TURRET));
          there = mechanism.state();
        }
      }
      double at = degrees(sim.simulation.position(TURRET));

      sim.simulation.slip(TURRET, Math.toRadians(20));
      double took = sim.runUntil(() -> mechanism.state() == JointMechanism.State.FAULTED, 1);
      String why = mechanism.fault();
      sim.run(0.5);

      String wired = turret == onPort ? "read by the robot" : "on its data port";
      assertThat(past).as(wired).isCloseTo(190, within(1.0));
      assertThat(there).as(wired).isEqualTo(JointMechanism.State.READY);
      assertThat(at).as(wired).isCloseTo(170, within(1.0));
      assertThat(took).as(wired).isNotNaN();
      assertThat(why)
          .as(wired)
          .startsWith(
              "turret's absolute encoder and its motor's count are 20° apart, more than 5°");
      assertThat(degrees(sim.simulation.position(TURRET))).as(wired).isCloseTo(at, within(1.0));
    }
  }

  /** A gap backlash could make, 3°, inside the 5° they may disagree, is no slip. */
  @Test
  void aGapInsideItsMarginIsTrusted() {
    JointsSimHarness sim = out(SEEDED);

    sim.simulation.slip(PIVOT, Math.toRadians(3));
    double took = untilFaulted(sim);

    assertThat(took).isNaN();
  }

  /** A control that trusts the two to disagree by 30°, its chain loose, trusts 20°. */
  @Test
  void aJointsControlCanTrustAWiderGap() {
    JointsSimHarness sim = out(SEEDED.retuned(control -> control.withEncoderGap(Degrees.of(30))));

    sim.simulation.slip(PIVOT, Math.toRadians(20));
    double took = untilFaulted(sim);

    assertThat(took).isNaN();
  }

  /**
   * A CANcoder that says its magnet is too close or too far isn't trusted: the pivot is faulted,
   * saying so, holds where its motor says, or, its TalonFX's loop running on the CANcoder, is
   * stopped; and it's ready again, where it is, once the CANcoder says the magnet is good.
   */
  @Test
  void aCancoderWithABadMagnetIsntTrusted() {
    Joint.Revolute remote =
        ON_CANCODER.encoderOn(CANCODER.can(34).offset(Rotations.of(0.3)).remote());
    for (Joint.Revolute wired : List.of(ON_CANCODER, remote)) {
      boolean[] bad = {false};
      JointsSimHarness sim =
          new JointsSimHarness(
              ROBOT.with(wired),
              (joint, io) ->
                  !joint.equals(PIVOT)
                      ? io
                      : new JointIO() {
                        @Override
                        public void updateInputs(JointIOInputs inputs) {
                          io.updateInputs(inputs);
                          inputs.magnetBad = bad[0];
                        }

                        @Override
                        public void setPosition(double position, double velocity, double volts) {
                          io.setPosition(position, velocity, volts);
                        }

                        @Override
                        public void stop() {
                          io.stop();
                        }
                      });
      JointMechanism pivot = sim.joints.of(PIVOT);
      sim.run(0.2);

      bad[0] = true;
      sim.loop();
      JointMechanism.State badState = pivot.state();
      String why = pivot.fault();
      sim.run(0.5);
      bad[0] = false;
      sim.loop();

      assertThat(badState).isEqualTo(JointMechanism.State.FAULTED);
      assertThat(why)
          .isEqualTo(
              "pivot's CANcoder 34 says its magnet is too close or too far, so what it reads isn't"
                  + " trusted: pivot "
                  + (wired == remote ? "is stopped" : "holds where its motor says")
                  + " until it's good. Check the magnet's gap and that it's centered on the"
                  + " CANcoder");
      assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
      assertThat(degrees(sim.simulation.position(PIVOT)))
          .as("held, or stopped and sagged onto its stop")
          .isCloseTo(wired == remote ? 0 : 1.5, within(0.5));
    }
  }
}

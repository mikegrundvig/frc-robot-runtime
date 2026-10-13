package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.CLAW_BEAM;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INDEXER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.ExampleControls;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Binding;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MoveEnding;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.record.MoveResult;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The guide's controls on the guide's robot, built as the robot builds a team's, each behavior
 * docs/guide.md walks through: the intake and its interlock, its end when the beam sees a piece, a
 * second button that takes the pivot from it, the deploy that backs off, and the aimed shot. Each
 * binding is in the robot's record, with its button and its command.
 */
class ControlsSimTest {
  private static final double RPM = 2 * Math.PI / 60;

  @RegisterExtension final RobotSim sim = new RobotSim();

  /** The guide's robot with the guide's controls, enabled in teleop, a few loops in. */
  private JointSimulation enabled() {
    sim.build(ExampleRobot.COMPETITION, SensorErrors.NONE);
    sim.controls(ExampleControls::new);
    RobotSim.setEnabled(true);
    sim.run(0.1);
    return Objects.requireNonNull(sim.world().joints());
  }

  /** Holds the operator's right trigger, which intakes, for some seconds. */
  private void intakeFor(double seconds) {
    sim.operator().setRightTriggerAxis(1);
    sim.operator().notifyNewData();
    sim.run(seconds);
  }

  @Test
  void theOperatorsTriggerIntakesAndEachBindingIsRecorded() {
    JointSimulation truth = enabled();

    intakeFor(0.2);
    double early = truth.velocity(ROLLER);
    sim.run(1.3);

    assertThat(early).as("held while the pivot swings out").isCloseTo(0, within(1.0));
    assertThat(truth.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
    assertThat(truth.velocity(ROLLER)).as("intaking").isCloseTo(2000 * RPM, within(20.0));
    assertThat(sim.bindings().all().stream().map(Binding::toString))
        .containsExactly(
            "driver back: on press: Reset heading",
            "operator right trigger: while held: Intake",
            "operator face right: on press: pivot → in",
            "operator face down: on press: Deploy",
            "driver right trigger: while held: Shoot");
  }

  /**
   * The beam sees a piece, and the intake is over: the roller's move arrives, the roller, idle,
   * stops, and the pivot holds out. The trigger is still held, and starts nothing again.
   */
  @Test
  void theIntakeEndsWhenTheBeamSeesAPiece() {
    JointSimulation truth = enabled();
    intakeFor(1.5);
    double intaking = truth.velocity(ROLLER);

    sim.world().sensors().trip(CLAW_BEAM, true);
    sim.run(1);

    assertThat(intaking).as("intaking").isCloseTo(2000 * RPM, within(20.0));
    ended(sim.joints().of(ROLLER)).named("roller: intaking until piece in").arrived();
    assertThat(truth.velocity(ROLLER)).as("idle, it stops").isCloseTo(0, within(1.0));
    assertThat(truth.position(PIVOT)).as("held out").isCloseTo(Math.toRadians(130), within(0.03));
  }

  /**
   * One command interrupted by another: with the trigger still held, the operator's face right
   * button sends the pivot in. That move takes the pivot, so Intake, which owns the pivot and the
   * roller, is canceled whole: the roller, idle, stops, and the pivot goes in. The held trigger
   * doesn't start Intake again, since a binding starts on a press.
   */
  @Test
  void aSecondButtonTakesThePivotAndTheIntakeEnds() {
    JointSimulation truth = enabled();
    intakeFor(1.5);
    double intaking = truth.velocity(ROLLER);

    sim.operator().setFaceRightButton(true);
    sim.operator().notifyNewData();
    sim.run(1.5);

    assertThat(intaking).as("intaking").isCloseTo(2000 * RPM, within(20.0));
    ended(sim.joints().of(ROLLER)).named("roller: intaking until piece in").canceled();
    assertThat(truth.velocity(ROLLER)).as("idle, it stops").isCloseTo(0, within(1.0));
    ended(sim.joints().of(PIVOT)).named("pivot → in").arrived();
    assertThat(truth.position(PIVOT)).isCloseTo(Math.toRadians(1.5), within(0.03));
  }

  /**
   * A move whose outcome is handled: jammed where it rests, the pivot never gets out, its move
   * times out at its deadline, and the deploy sends it back in, where it already is.
   */
  @Test
  void aDeployThatTimesOutBacksOff() {
    JointSimulation truth = enabled();
    JointMechanism pivot = sim.joints().of(PIVOT);
    truth.jam(PIVOT, true);

    sim.operator().setFaceDownButton(true);
    sim.operator().notifyNewData();
    Optional<MoveEnding> timedOut = Optional.empty();
    for (int loop = 0; loop < 250 && timedOut.isEmpty(); loop++) {
      sim.loop();
      timedOut = pivot.lastEnding().filter(ending -> ending.result() == MoveResult.TIMED_OUT);
    }
    sim.run(1);

    assertThat(timedOut).as("the deploy timed out").isPresent();
    ended(timedOut.orElseThrow()).named("pivot → out").timedOut().partWay();
    ended(pivot).named("pivot → in").arrived();
    assertThat(truth.position(PIVOT)).isCloseTo(Math.toRadians(1.5), within(0.03));
  }

  /**
   * The aimed shot: held, the shooter spins up and the robot turns to the hub, the left stick still
   * the driver's; the feed runs only once the robot is aimed and the shooter is at speed, and stops
   * when the trigger is let go.
   */
  @Test
  void theShotFeedsOnlyOnceAimedAndAtSpeed() {
    JointSimulation truth = enabled();
    Pose2d from = new Pose2d(3, 2.5, Rotation2d.ZERO);
    sim.placeAt(from);
    Translation2d hub = Rebuilt.HUB.on(RobotSim.FIELD, Alliance.BLUE).getTranslation();

    sim.driver().setRightTriggerAxis(1);
    sim.driver().notifyNewData();
    sim.run(0.3);
    double early = truth.velocity(FEEDER);
    sim.run(2.7);
    double feeding = truth.velocity(FEEDER);
    double shooting = truth.velocity(SHOOTER);
    Pose2d aimed = sim.world().truePose();
    sim.driver().setRightTriggerAxis(0);
    sim.driver().notifyNewData();
    sim.run(1);

    assertThat(early).as("not yet aimed, nor at speed").isCloseTo(0, within(1.0));
    assertThat(shooting).as("at speed").isCloseTo(3000 * RPM, within(0.03 * 3000 * RPM));
    assertThat(feeding).as("feeding").isCloseTo(1000 * RPM, within(10.0));
    assertThat(aimed.getRotation().getDegrees())
        .as("facing the hub")
        .isCloseTo(
            Math.toDegrees(Math.atan2(hub.getY() - aimed.getY(), hub.getX() - aimed.getX())),
            within(2.0));
    assertThat(truth.velocity(FEEDER)).as("let go, it stops").isCloseTo(0, within(1.0));
    assertThat(truth.velocity(INDEXER)).as("and so does the indexer").isCloseTo(0, within(1.0));
  }
}

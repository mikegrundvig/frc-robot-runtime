package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Driving from the driver's sticks, with and without the drivetrain holding a heading, and the
 * wheels locked in an X, in the simulator: wheels and sensors as configured, so the robot's pose is
 * the truth.
 */
@ExtendWith(RobotSim.class)
class DriveAssistSimTest {
  private final DriveSimHarness harness = new DriveSimHarness(ExampleRobot.SIM);
  private final Drive drive = harness.drive;

  /** What a stick at 0.4 asks of what one at 0.8 does: the deadband cut away, then squared. */
  private static final double SHAPED = Math.pow(0.3 / 0.7, 2);

  @AfterEach
  void tearDown() {
    harness.close();
  }

  /** The robot is exactly here, and the drive knows it. */
  private void startAt(Pose2d pose) {
    harness.simulation.place(pose);
    harness.loop();
    harness.localization.resetPose(pose);
    harness.loop();
  }

  private Pose2d truth() {
    return harness.simulation.truePose();
  }

  /** How far the robot's heading is from a target, the nearer way round, in degrees. */
  private double offBy(Rotation2d target) {
    return target.minus(truth().getRotation()).getDegrees();
  }

  private void run(double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      harness.loop();
    }
  }

  /** Schedules a command, enabled, and runs for a time. */
  private void drive(DriveAssist assist, double seconds) {
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(assist);
    run(seconds);
  }

  /** The stick on the field's axes, translation as the robot turns it for the driver. */
  private void sticks(double x, double y, double turn) {
    harness.sticks(x, y, turn);
  }

  /**
   * From rest, a snap to 90° settles inside 2° and never goes more than 2° past it, and arrives
   * within 0.5° of it: a computed turn isn't deadbanded, as a stick's is (a deadband would stop it
   * short). In the simulator: 1.4° over, inside 2° by 0.76 s, 0.17° off after 3 s.
   */
  @Test
  void aSnapSettlesOnItsHeadingWithoutOvershooting() {
    Rotation2d ninety = Rotation2d.fromDegrees(90);
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(drive.facing(Heading.field(ninety)));
    double peak = 0;
    double settledAt = Double.NaN;
    for (int i = 1; i <= RobotLoop.loops(3); i++) {
      harness.loop();
      peak = Math.max(peak, truth().getRotation().getDegrees());
      if (Math.abs(offBy(ninety)) > 2) {
        settledAt = Double.NaN;
      } else if (Double.isNaN(settledAt)) {
        settledAt = i * RobotLoop.PERIOD_SECONDS;
      }
    }

    assertThat(settledAt).as("inside 2° for good, from").isLessThan(1.0);
    assertThat(peak - 90).as("past the heading, at most").isLessThanOrEqualTo(2.0);
    assertThat(offBy(ninety)).as("after 3 s").isCloseTo(0, within(0.5));
  }

  /**
   * Full stick along the field while it faces 90°: it keeps the heading, inside 2°, the whole way.
   */
  @Test
  void itHoldsItsHeadingWhileTranslating() {
    Rotation2d ninety = Rotation2d.fromDegrees(90);
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(drive.facing(Heading.field(ninety)));
    run(1.5);
    sticks(1, 0, 0);
    double worst = 0;
    for (int i = 0; i < RobotLoop.loops(2); i++) {
      harness.loop();
      worst = Math.max(worst, Math.abs(offBy(ninety)));
    }

    assertThat(truth().getX()).as("driving along the field").isGreaterThan(3.0);
    assertThat(worst).isLessThanOrEqualTo(2.0);
  }

  /**
   * Strafing past the hub at speed, the bearing to it turns faster than the heading's own error
   * could drive: the target's rate is fed forward, and the error stays inside 3°.
   */
  @Test
  void itFacesAPlaceWhileStrafingPastIt() {
    Pose2d start = new Pose2d(2, -6, Rotation2d.ZERO);
    Rotation2d toHub =
        Heading.toward(Rebuilt.HUB)
            .resolve(start, RobotSim.FIELD, Optional.of(Alliance.BLUE))
            .orElseThrow();
    startAt(new Pose2d(start.getTranslation(), toHub));
    RobotSim.setEnabled(true);
    DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB)).within(Degrees.of(3));
    harness.scheduler.schedule(aim);
    sticks(0, 1, 0);
    double worstAtSpeed = 0;
    int atSpeed = 0;
    int aimedAtSpeed = 0;
    for (int i = 0; i < RobotLoop.loops(3.6); i++) {
      harness.loop();
      Rotation2d now =
          Heading.toward(Rebuilt.HUB)
              .resolve(truth(), RobotSim.FIELD, Optional.of(Alliance.BLUE))
              .orElseThrow();
      if (truth().getY() > -3.5) {
        worstAtSpeed = Math.max(worstAtSpeed, Math.abs(offBy(now)));
        atSpeed++;
        aimedAtSpeed += aim.aimed().getAsBoolean() ? 1 : 0;
      }
    }
    assertThat(truth().getY()).as("past the hub, which is at 4.03").isGreaterThan(5.5);
    assertThat(worstAtSpeed).isLessThanOrEqualTo(3.0);
    // Turning with the bearing, it's aimed all the same: the gate asks whether it keeps up.
    assertThat(aimedAtSpeed).as("loops aimed of " + atSpeed + " at speed").isEqualTo(atSpeed);
  }

  /**
   * Each alliance snaps to its own downfield and faces its own hub: the angle from the drivers'
   * forward, and the place as it is for the robot's alliance, whichever end of the field it's on.
   */
  @Test
  void eachAllianceFacesItsOwnDownfieldAndHub() {
    RobotSim.setEnabled(true);
    for (Alliance alliance : Alliance.values()) {
      harness.alliance(Optional.of(alliance));
      boolean red = alliance == Alliance.RED;
      Translation2d hub = Rebuilt.HUB.on(RobotSim.FIELD, alliance).getTranslation();
      startAt(new Pose2d(red ? 12 : 4, 1, Rotation2d.fromDegrees(90)));

      DriveAssist left = drive.facing(Heading.downfield(Degrees.of(20)));
      harness.scheduler.schedule(left);
      run(1.5);
      double downfield = truth().getRotation().getDegrees();
      harness.scheduler.cancel(left);

      DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB));
      harness.scheduler.schedule(aim);
      run(1.5);
      Translation2d to = hub.minus(truth().getTranslation());
      double toHub = Math.toDegrees(Math.atan2(to.getY(), to.getX()));
      harness.scheduler.cancel(aim);

      assertThat(Math.IEEEremainder(downfield - (red ? 200 : 20), 360))
          .as(alliance + " 20° left of downfield")
          .isCloseTo(0, within(1.0));
      assertThat(Math.IEEEremainder(truth().getRotation().getDegrees() - toHub, 360))
          .as(alliance + " toward its hub at " + hub)
          .isCloseTo(0, within(1.0));
    }
  }

  /**
   * With the alliance unknown there is nothing to face downfield or toward: the robot doesn't turn,
   * the translation still drives, and the gate says why it's closed.
   */
  @Test
  void withoutAnAllianceItDrivesButDoesNotTurn() {
    harness.alliance(Optional.empty());
    startAt(new Pose2d(3, 2, Rotation2d.fromDegrees(30)));
    DriveAssist aim = drive.facing(Heading.downfield(Degrees.of(0)));
    Gate aimed = aim.aimed();
    drive(aim, 0.1);
    sticks(1, 0, 0);
    run(1.0);

    assertThat(truth().getX()).as("driving").isGreaterThan(3.5);
    assertThat(truth().getRotation().getDegrees()).isCloseTo(30, within(0.5));
    assertThat(aimed.getAsBoolean()).isFalse();
    assertThat(aimed.why()).isEqualTo("aimed at downfield is false: alliance unknown");
    assertThat(drive.facing(Heading.toward(Rebuilt.HUB)).aimed().why())
        .isEqualTo("aimed at the Hub is false: alliance unknown");
  }

  /** On the place there is no bearing on it: it doesn't turn, and says so. */
  @Test
  void onTheirPlaceTheRobotDoesNotTurn() {
    Pose2d hub = Rebuilt.HUB.on(RobotSim.FIELD, Alliance.BLUE);
    startAt(new Pose2d(hub.getTranslation(), Rotation2d.fromDegrees(30)));
    DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB));
    Gate aimed = aim.aimed();
    drive(aim, 1.0);

    assertThat(truth().getRotation().getDegrees()).isCloseTo(30, within(0.5));
    assertThat(aimed.why()).isEqualTo("aimed at the Hub is false: on the Hub");
  }

  /**
   * The driver's turn stick takes the heading back while it's past its deadband; let go, and the
   * robot returns to the target.
   */
  @Test
  void theDriverTakesTheHeadingBackAndGivesItBack() {
    Rotation2d zero = Rotation2d.ZERO;
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(drive.facing(Heading.field(zero)));
    run(0.3);
    sticks(0, 0, 0.5);
    run(0.6);
    double turned = truth().getRotation().getDegrees();
    sticks(0, 0, 0.05); // inside the deadband: the heading is held again
    run(2.0);

    assertThat(turned).as("turned with the stick, counterclockwise").isGreaterThan(20);
    assertThat(offBy(zero)).as("back on its target").isCloseTo(0, within(0.5));
  }

  /**
   * The gate is a fact about the robot: closed while it turns, open once it's there, and the same
   * with the command never run. Closed, it says how far off.
   */
  @Test
  void theAimedGateOpensOnArrival() {
    DriveAssist snap = drive.facing(Heading.field(Rotation2d.fromDegrees(90)));
    Gate aimed = snap.aimed();
    RobotSim.setEnabled(true);
    harness.loop();
    String before = aimed.why();
    boolean closedBefore = !aimed.getAsBoolean();
    harness.scheduler.schedule(snap);
    run(0.3);
    boolean closedWhileTurning = !aimed.getAsBoolean();
    String turning = aimed.why();
    run(2.0);

    assertThat(closedBefore).isTrue();
    assertThat(before).matches("aimed at field 90° is false: 9\\d\\.\\d° off field 90°");
    assertThat(closedWhileTurning).isTrue();
    assertThat(turning).matches("aimed at field 90° is false: \\d+\\.\\d° off field 90°.*");
    assertThat(aimed.getAsBoolean()).isTrue();
    assertThat(aimed.blocking()).isEmpty();
    assertThat(aimed.name()).isEqualTo("aimed at field 90°");
  }

  @Test
  void theAimedGateIsAFactAboutTheRobotWhetherOrNotTheCommandRuns() {
    startAt(new Pose2d(0, 0, Rotation2d.fromDegrees(88.5)));
    Gate aimed = drive.facing(Heading.field(Rotation2d.fromDegrees(90))).aimed();
    Gate loose =
        drive.facing(Heading.field(Rotation2d.fromDegrees(90))).within(Degrees.of(10)).aimed();
    Gate tight =
        drive.facing(Heading.field(Rotation2d.fromDegrees(90))).within(Degrees.of(1)).aimed();
    run(0.1);

    assertThat(aimed.getAsBoolean()).as("1.5° off, within 2°").isTrue();
    assertThat(loose.getAsBoolean()).isTrue();
    assertThat(tight.getAsBoolean()).isFalse();
    assertThat(tight.why()).isEqualTo("aimed at field 90° is false: 1.5° off field 90°");
    assertThat(
            drive.getRegisteredScheduler().isRunning(drive.facing(Heading.field(Rotation2d.ZERO))))
        .isFalse();
  }

  /** A tolerance that can't be is refused as the robot starts, and once it runs, says why. */
  @Test
  void aToleranceThatCannotBeIsRefused() {
    DriveAssist assist = drive.facing(Heading.field(Rotation2d.ZERO));

    assertThatThrownBy(() -> assist.within(Degrees.of(0)))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("aim within a positive angle");
    harness.loop(); // the scheduler has run: the robot is running
    DriveAssist refused = assist.within(Degrees.of(-3));
    Gate aimed = refused.aimed();
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(refused);
    run(0.1);

    assertThat(harness.scheduler.isRunning(refused)).as("ended as it started").isFalse();
    assertThat(aimed.getAsBoolean()).isFalse();
    assertThat(aimed.why()).contains("aim within a positive angle, not -3.0°");
  }

  /**
   * The wheels turn into an X, each across its line to the center (either end forward), the robot
   * stays where it is while the sticks push, and the default command drives again after.
   */
  @Test
  void theWheelsLockInAnXAndTheDefaultCommandResumes() {
    drive.setDefaultCommand(drive.driven());
    Command lock = drive.lockWheels();
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(lock);
    sticks(1, 0, 0.3);
    run(1.5);
    Translation2d[] wheels = drive.moduleLocations();
    var angles = drive.moduleVelocities();

    for (int i = 0; i < 4; i++) {
      double across = Math.toDegrees(Math.atan2(wheels[i].getY(), wheels[i].getX())) + 90;
      assertThat(Math.IEEEremainder(angles[i].angle.getDegrees() - across, 180))
          .as("wheel " + i + " across its line to the center")
          .isCloseTo(0, within(2.0));
    }
    assertThat(truth().getTranslation().getNorm()).as("stayed put").isLessThan(0.02);
    assertThat(truth().getRotation().getDegrees()).isCloseTo(0, within(1.0));

    harness.scheduler.cancel(lock);
    run(1.0);

    assertThat(truth().getX()).as("driving again").isGreaterThan(0.5);
  }

  /**
   * The sticks' shaping is the drive's own: past a deadband that is cut away, then squared, so a
   * stick at 0.4 asks (0.3/0.7)² of what 0.8 does, translation and turn alike; a stick inside the
   * deadband stands still.
   */
  @Test
  void drivenShapesTheSticks() {
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(drive.driven());

    sticks(0.05, 0.05, 0.05);
    run(1.0);
    double idle = truth().getTranslation().getNorm() + Math.abs(truth().getRotation().getDegrees());
    sticks(0.4, 0, 0);
    run(2.0);
    double slow = drive.getRobotRelativeVelocities().vx;
    sticks(0.8, 0, 0);
    run(2.0);
    double fast = drive.getRobotRelativeVelocities().vx;
    sticks(0, 0, 0);
    run(2.0);
    sticks(0, 0, -0.4);
    run(2.0);
    double turningSlowly = drive.getRobotRelativeVelocities().omega;
    sticks(0, 0, -0.8);
    run(2.0);
    double turningFast = drive.getRobotRelativeVelocities().omega;

    assertThat(idle).as("inside the deadband").isLessThan(1e-3);
    assertThat(slow / fast).as("squared").isCloseTo(SHAPED, within(0.01));
    assertThat(turningFast).as("clockwise").isLessThan(-1);
    assertThat(turningSlowly / turningFast).as("squared").isCloseTo(SHAPED, within(0.01));
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;

/**
 * Follows one planned path with the drivetrain, for one run of a {@link DriveMove}: what to ask of
 * the drivetrain each loop, and whether the robot has come to the path's end.
 *
 * <p>A path says where the robot should be, and how fast it should be moving, at each moment after
 * the move starts. Each loop, the robot is asked for:
 *
 * <ul>
 *   <li>the velocity the path has at the end of this loop, since that is when the wheels will have
 *       answered;
 *   <li>plus a push back toward the path's place and heading, for however far the robot has strayed
 *       from them ({@code translationKp}, {@code headingKp});
 *   <li>plus a push toward the path's speed, for however much slower the robot is measured to move
 *       ({@link #RATE_DAMPING}).
 * </ul>
 *
 * <p>The wheels are also given the volts that the path's acceleration takes, so that the robot
 * keeps up with the path instead of trailing it until the pushes build up.
 */
final class PathFollower {
  /**
   * How much of the gap between the path's speed and turning rate and the robot's measured ones a
   * move adds to what it asks. Even given the volts each acceleration takes, the drivetrain's speed
   * loop trails a little. Damping it takes a quarter turn from 49 loops to 42 in the simulator.
   */
  static final double RATE_DAMPING = 0.5;

  private final Drive drive;
  private final HolonomicTrajectory path;

  /** Where the path ends. */
  private final Pose2d end;

  private final DriveTask.Arrival arrival;

  // The drivetrain's gains and limits as they were tuned when the move started.

  private final double translationKp;
  private final double headingKp;
  private final double topSpeedMetersPerSecond;
  private final double topTurnRateRadiansPerSecond;

  /**
   * @param path the path to follow, planned from where the robot was as the move started
   * @param arrival what the robot must reach to have arrived
   * @param control the drivetrain's gains and limits, as tuned now
   */
  PathFollower(
      Drive drive, HolonomicTrajectory path, DriveTask.Arrival arrival, DrivetrainControl control) {
    this.drive = drive;
    this.path = path;
    this.arrival = arrival;
    end = path.end().pose;
    translationKp = control.gains().translationKp();
    headingKp = control.gains().headingKp();
    topSpeedMetersPerSecond = control.limits().maxLinearSpeed().in(MetersPerSecond);
    topTurnRateRadiansPerSecond = control.limits().maxAngularSpeed().in(RadiansPerSecond);
  }

  /** How long the path takes, in seconds. */
  double pathSeconds() {
    return path.duration;
  }

  /**
   * Whether the robot is at the path's end as of this loop: the path's time is up, the robot is
   * within {@link DriveMove#PLACED_METERS} and {@link DriveMove#TURNED_RADIANS} of the last pose,
   * and it has nearly stopped. A turn is judged by its heading alone ({@link
   * DriveTask.Arrival#FACING_ITS_HEADING}).
   *
   * @param elapsedSeconds how long the move has run
   * @param pose where the robot is
   * @param measuredOnField how fast the robot is measured to move, on the field's axes
   */
  boolean isAtItsEnd(double elapsedSeconds, Pose2d pose, ChassisVelocities measuredOnField) {
    boolean pathIsOver = elapsedSeconds >= path.duration;
    if (!pathIsOver) {
      return false;
    }
    if (arrival == DriveTask.Arrival.AT_ITS_POSE) {
      double metersFromEnd = pose.getTranslation().getDistance(end.getTranslation());
      boolean atItsPlace = metersFromEnd <= DriveMove.PLACED_METERS;
      if (!atItsPlace) {
        return false;
      }
    }
    double radiansFromEnd = Math.abs(end.getRotation().minus(pose.getRotation()).getRadians());
    boolean facingItsHeading = radiansFromEnd <= DriveMove.TURNED_RADIANS;
    if (!facingItsHeading) {
      return false;
    }
    double speedMetersPerSecond = Math.hypot(measuredOnField.vx, measuredOnField.vy);
    boolean nearlyStoppedMoving = speedMetersPerSecond <= DriveMove.STILL_METERS_PER_SECOND;
    boolean nearlyStoppedTurning =
        Math.abs(measuredOnField.omega) <= DriveMove.STILL_RADIANS_PER_SECOND;
    return nearlyStoppedMoving && nearlyStoppedTurning;
  }

  /**
   * Drives the robot along the path for one loop.
   *
   * @param elapsedSeconds how long the move has run
   * @param pose where the robot is
   * @param measuredOnField how fast the robot is measured to move, on the field's axes
   */
  void drive(double elapsedSeconds, Pose2d pose, ChassisVelocities measuredOnField) {
    // What the wheels are told now takes effect over the loop, so the robot is steered at where
    // the path will be when the loop ends.
    HolonomicSample dueNextLoop = path.sampleAt(elapsedSeconds + RobotLoop.PERIOD_SECONDS);
    ChassisVelocities pathVelocityNow = path.sampleAt(elapsedSeconds).velocity;

    Translation2d velocity = velocityAsked(dueNextLoop, pose, measuredOnField);
    double turnRate = turnRateAsked(dueNextLoop, pose, measuredOnField);
    ChassisVelocities asked = new ChassisVelocities(velocity.getX(), velocity.getY(), turnRate);
    ChassisAccelerations pathAcceleration =
        accelerationOverTheLoop(pathVelocityNow, dueNextLoop.velocity);

    // The path is on the field's axes, and the drivetrain is driven on the robot's.
    Rotation2d heading = pose.getRotation();
    drive.runVelocity(asked.toRobotRelative(heading), pathAcceleration.toRobotRelative(heading));
  }

  /**
   * The velocity to ask of the robot, on the field's axes, in meters a second: the path's, plus the
   * pushes back toward the path's place and toward its speed, and no faster than the top speed.
   */
  private Translation2d velocityAsked(
      HolonomicSample due, Pose2d pose, ChassisVelocities measuredOnField) {
    Translation2d pathVelocity = new Translation2d(due.velocity.vx, due.velocity.vy);
    Translation2d measuredVelocity = new Translation2d(measuredOnField.vx, measuredOnField.vy);
    Translation2d metersOffThePath = due.pose.getTranslation().minus(pose.getTranslation());
    Translation2d slowerThanThePath = pathVelocity.minus(measuredVelocity);

    Translation2d asked =
        pathVelocity
            .plus(metersOffThePath.times(translationKp))
            .plus(slowerThanThePath.times(RATE_DAMPING));

    // Too fast, it keeps its direction and is scaled back to the top speed.
    double speedAsked = asked.getNorm();
    if (speedAsked > topSpeedMetersPerSecond) {
      return asked.times(topSpeedMetersPerSecond / speedAsked);
    }
    return asked;
  }

  /**
   * The turning rate to ask of the robot, in radians a second: the path's, plus the pushes back
   * toward the path's heading and toward its turning rate, and no faster than the top turning rate.
   */
  private double turnRateAsked(
      HolonomicSample due, Pose2d pose, ChassisVelocities measuredOnField) {
    double radiansOffThePath = due.pose.getRotation().minus(pose.getRotation()).getRadians();
    double slowerThanThePath = due.velocity.omega - measuredOnField.omega;
    double asked =
        due.velocity.omega + headingKp * radiansOffThePath + RATE_DAMPING * slowerThanThePath;
    return Math.clamp(asked, -topTurnRateRadiansPerSecond, topTurnRateRadiansPerSecond);
  }

  /**
   * The path's acceleration over the loop, on the field's axes: how much its velocity changes
   * between now and the end of the loop, divided by the loop's length.
   */
  private static ChassisAccelerations accelerationOverTheLoop(
      ChassisVelocities now, ChassisVelocities nextLoop) {
    return new ChassisAccelerations(
        (nextLoop.vx - now.vx) / RobotLoop.PERIOD_SECONDS,
        (nextLoop.vy - now.vy) / RobotLoop.PERIOD_SECONDS,
        (nextLoop.omega - now.omega) / RobotLoop.PERIOD_SECONDS);
  }
}

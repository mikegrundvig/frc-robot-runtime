package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.MathUtil;

/**
 * What the drivetrain tells its four modules in one loop. There are four things it can tell them:
 * to move the robot at a velocity ({@link #drive}), to lock the wheels ({@link #lockWheels}), to
 * point every wheel one way with the wheels held still ({@link #pointWheels}), and to stop ({@link
 * #stop}).
 *
 * <p>It logs what it asks of the wheels as {@code Drive/Setpoints}, and what the modules were told
 * in the end as {@code Drive/SetpointsOptimized}.
 */
final class DriveOutput {
  /** No wheel is told anything: what the log shows while the robot is disabled. */
  private static final SwerveModuleVelocity[] NO_SETPOINTS = {};

  private final Module[] modules;
  private final SwerveDriveKinematics kinematics;

  /** Where each wheel touches the floor, from the robot's center. */
  private final Translation2d[] moduleTranslations;

  private final double massKilograms;
  private final double yawInertiaKilogramSquareMeters;

  /** The wheels' squared distances from the robot's center, summed, in square meters. */
  private final double sumOfSquaredWheelDistances;

  /**
   * The two directions that lock each wheel: across its line to the robot's center, pointing one
   * way or the other. They are worked out once, since they depend only on where the wheels are.
   */
  private final double[] acrossRadians = new double[Module.COUNT];

  private final Rotation2d[] across = new Rotation2d[Module.COUNT];
  private final Rotation2d[] acrossOtherWay = new Rotation2d[Module.COUNT];

  /** The fastest any wheel is told to roll, in meters a second, as tuned. */
  private double topWheelSpeedMetersPerSecond;

  DriveOutput(Module[] modules, RobotDefinition robot, SwerveDriveKinematics kinematics) {
    this.modules = modules;
    this.kinematics = kinematics;
    moduleTranslations = robot.moduleTranslations();
    massKilograms = robot.body().mass().in(Kilograms);
    yawInertiaKilogramSquareMeters = robot.body().yawMomentOfInertia().in(KilogramSquareMeters);
    double sum = 0;
    for (Translation2d at : moduleTranslations) {
      sum += at.getSquaredNorm();
    }
    sumOfSquaredWheelDistances = sum;
    for (int i = 0; i < Module.COUNT; i++) {
      Translation2d at = moduleTranslations[i];
      double towardCenterLineRadians = Math.atan2(at.getY(), at.getX());
      acrossRadians[i] = towardCenterLineRadians + Math.PI / 2;
      across[i] = new Rotation2d(acrossRadians[i]);
      acrossOtherWay[i] = new Rotation2d(acrossRadians[i] + Math.PI);
    }
  }

  /** From now on no wheel is told to roll faster than this, in meters a second. */
  void limitWheelSpeed(double topWheelSpeedMetersPerSecond) {
    this.topWheelSpeedMetersPerSecond = topWheelSpeedMetersPerSecond;
  }

  /**
   * Moves the robot at a velocity, relative to itself, for one loop, while it speeds up or turns up
   * at an acceleration.
   *
   * <p>The velocity decides each wheel's speed and direction. The acceleration adds to each drive
   * motor's feedforward the volts that the wheel's share of the push takes, so that the robot
   * follows a profile and does not trail it. Plain driving passes no acceleration.
   *
   * @param robotRelative forward and left speeds in meters a second, and the turning rate in
   *     radians a second, counterclockwise
   * @param acceleration forward and left accelerations in meters a second squared, and the turning
   *     acceleration in radians a second squared
   */
  void drive(ChassisVelocities robotRelative, ChassisAccelerations acceleration) {
    // Discretization: a robot that turns while it drives would curve off its line during the loop,
    // because the wheels hold one direction for the whole 20 ms while the robot rotates under
    // them. This asks for the velocity that, held for one loop, ends where the velocity asked for
    // would have ended.
    ChassisVelocities heldForTheLoop = robotRelative.discretize(RobotLoop.PERIOD_SECONDS);

    // Desaturation: if any wheel is asked to roll faster than the top speed, every wheel is slowed
    // by the same share, so the robot moves in the direction asked, only slower.
    SwerveModuleVelocity[] setpoints =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toSwerveModuleVelocities(heldForTheLoop), topWheelSpeedMetersPerSecond);

    // The force each wheel takes for the acceleration. Speeding the robot up takes its mass times
    // the acceleration, shared equally between the wheels. Turning it up takes its yaw inertia
    // times the turning acceleration as a torque about the center. Each wheel pushes across its
    // line to the center, harder the further out it is, so a wheel at (x, y) pushes with
    // torque * (-y, x) / (the sum of every wheel's squared distance).
    double shareOfForceForwardNewtons = massKilograms * acceleration.ax / Module.COUNT;
    double shareOfForceLeftNewtons = massKilograms * acceleration.ay / Module.COUNT;
    double torqueNewtonMeters = yawInertiaKilogramSquareMeters * acceleration.alpha;

    SwerveModuleVelocity[] told = new SwerveModuleVelocity[Module.COUNT];
    for (int i = 0; i < Module.COUNT; i++) {
      Translation2d at = moduleTranslations[i];
      double forceForwardNewtons =
          shareOfForceForwardNewtons - torqueNewtonMeters * at.getY() / sumOfSquaredWheelDistances;
      double forceLeftNewtons =
          shareOfForceLeftNewtons + torqueNewtonMeters * at.getX() / sumOfSquaredWheelDistances;
      told[i] = modules[i].runSetpoint(setpoints[i], forceForwardNewtons, forceLeftNewtons);
    }
    Logger.recordOutput("Drive/Setpoints", setpoints);
    Logger.recordOutput("Drive/SetpointsOptimized", told);
  }

  /**
   * Locks the wheels for one loop: each wheel is turned across its line to the robot's center, and
   * its drive motor holds it still, so that the robot is hard to push.
   *
   * <p>A wheel locks just as well pointing either way along that direction, so each turns to
   * whichever is nearer.
   */
  void lockWheels() {
    for (int i = 0; i < Module.COUNT; i++) {
      double pointsNowRadians = modules[i].steerAngle().getRadians();
      double turnRadians = MathUtil.angleModulus(acrossRadians[i] - pointsNowRadians);
      boolean otherWayIsNearer = Math.abs(turnRadians) > Math.PI / 2;
      modules[i].holdSteer(otherWayIsNearer ? acrossOtherWay[i] : across[i]);
    }
  }

  /**
   * Points every wheel at one angle for one loop, relative to the robot, with the drive wheels held
   * still.
   */
  void pointWheels(Rotation2d angle) {
    for (Module module : modules) {
      module.holdSteer(angle);
    }
  }

  /** Stops every motor: what the drivetrain does each loop the robot is disabled. */
  void stop() {
    for (Module module : modules) {
      module.stop();
    }
    Logger.recordOutput("Drive/SetpointsOptimized", NO_SETPOINTS);
  }
}

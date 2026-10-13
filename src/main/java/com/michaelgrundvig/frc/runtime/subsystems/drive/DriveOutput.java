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
 * <p>While the robot is disabled it tells them only to stop, whatever it is asked ({@link
 * #mayBeDriven}). This is the one place the wheels are told anything, so no command, pit tool or
 * simulator control drives a disabled robot.
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
   * The two directions that lock each wheel: along its line to the robot's center, pointing at the
   * center or away from it. Together the four wheels make an X. A wheel rolls only the way it
   * points, so no push along the floor and no turn about the center can roll all four at once. The
   * directions are worked out once, since they depend only on where the wheels are.
   */
  private final double[] towardCenterRadians = new double[Module.COUNT];

  private final Rotation2d[] towardCenter = new Rotation2d[Module.COUNT];
  private final Rotation2d[] awayFromCenter = new Rotation2d[Module.COUNT];

  /** The fastest any wheel is told to roll, in meters a second, as tuned. */
  private double topWheelSpeedMetersPerSecond;

  /** Whether the robot is enabled this loop. It is taken as disabled until the first loop says. */
  private boolean enabled;

  /** What the log has of what the wheels are asked. */
  private enum Logged {
    NOTHING,
    SETPOINTS,
    NO_SETPOINTS
  }

  /**
   * What was last logged. A disabled robot's wheels are asked nothing loop after loop, and that is
   * logged once, as the robot is disabled: the log keeps a value until another replaces it, and
   * each write makes a new value and builds the key's full name.
   */
  private Logged logged = Logged.NOTHING;

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
      towardCenterRadians[i] = Math.atan2(at.getY(), at.getX());
      towardCenter[i] = new Rotation2d(towardCenterRadians[i]);
      awayFromCenter[i] = new Rotation2d(towardCenterRadians[i] + Math.PI);
    }
  }

  /** From now on no wheel is told to roll faster than this, in meters a second. */
  void limitWheelSpeed(double topWheelSpeedMetersPerSecond) {
    this.topWheelSpeedMetersPerSecond = topWheelSpeedMetersPerSecond;
  }

  /**
   * Starts a loop: from now until the next loop, the modules are driven only if the robot is
   * enabled. A disabled robot's motors are stopped here, before anything asks for them.
   */
  void newLoop(boolean enabled) {
    this.enabled = enabled;
    if (!enabled) {
      stop();
    }
  }

  /**
   * Whether the wheels may be driven as they are asked this loop. They may not while the robot is
   * disabled, as a joint may not ({@code JointMechanism.mayBeDriven}). What is asked then is
   * dropped: the motors were stopped as the loop began ({@link #newLoop}), and stay so.
   *
   * <p>On a robot the motor controllers go neutral by themselves when the Driver Station disables
   * them. Refusing here as well means the robot does not depend on that, the log shows no wheel
   * told anything, and the simulator, which has no Driver Station to cut its motors' power, does
   * the same as the robot.
   */
  private boolean mayBeDriven() {
    return enabled;
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
    if (!mayBeDriven()) {
      return;
    }
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
    logged = Logged.SETPOINTS;
    Logger.recordOutput("Drive/Setpoints", setpoints);
    Logger.recordOutput("Drive/SetpointsOptimized", told);
  }

  /**
   * Locks the wheels for one loop: each wheel is turned along its line to the robot's center, so
   * the four make an X, and its drive motor holds it still, so that the robot is hard to push or to
   * spin.
   *
   * <p>A wheel locks just as well pointing either way along that direction, so each turns to
   * whichever is nearer.
   */
  void lockWheels() {
    if (!mayBeDriven()) {
      return;
    }
    for (int i = 0; i < Module.COUNT; i++) {
      double pointsNowRadians = modules[i].steerAngle().getRadians();
      double turnRadians = MathUtil.angleModulus(towardCenterRadians[i] - pointsNowRadians);
      boolean otherWayIsNearer = Math.abs(turnRadians) > Math.PI / 2;
      modules[i].holdSteer(otherWayIsNearer ? awayFromCenter[i] : towardCenter[i]);
    }
  }

  /**
   * Points every wheel at one angle for one loop, relative to the robot, with the drive wheels held
   * still.
   */
  void pointWheels(Rotation2d angle) {
    if (!mayBeDriven()) {
      return;
    }
    for (Module module : modules) {
      module.holdSteer(angle);
    }
  }

  /**
   * Stops every motor, which leaves each to brake or coast as its neutral mode says: what the
   * drivetrain does each loop the robot is disabled, whatever it is asked.
   */
  void stop() {
    for (Module module : modules) {
      module.stop();
    }
    if (logged == Logged.NO_SETPOINTS) {
      return;
    }
    logged = Logged.NO_SETPOINTS;
    Logger.recordOutput("Drive/Setpoints", NO_SETPOINTS);
    Logger.recordOutput("Drive/SetpointsOptimized", NO_SETPOINTS);
  }
}

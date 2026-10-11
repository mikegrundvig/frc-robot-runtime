package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * One swerve module: a wheel that is driven and steered. It turns what the drivetrain asks of the
 * wheel, a speed along the floor and a direction, into what its motor controllers are told, and
 * turns what they read back into meters.
 *
 * <p>The hardware works in radians at the wheel ({@link ModuleIO}). The wheel's radius, which
 * changes as the tread wears, is applied here and nowhere else.
 */
public class Module {
  /**
   * How many modules a swerve drivetrain has. They are numbered from 0, in the order of {@link
   * #NAMES}.
   */
  static final int COUNT = 4;

  /** The modules' names, in the order the drivetrain numbers them, as the log's keys have them. */
  static final List<String> NAMES = List.of("FrontLeft", "FrontRight", "BackLeft", "BackRight");

  /**
   * A wheel asked to roll slower than this is asked to stand still, and keeps the direction it has.
   * The direction asked of a wheel that is barely moving is mostly noise, and steering toward it
   * would make a robot at rest twitch its wheels.
   */
  private static final double STEER_HOLD_SPEED_MPS = 0.01;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  /** Where the module's inputs are logged, {@code Drive/ModuleFrontLeft}: worked out once. */
  private final String inputsKey;

  private final double wheelRadiusMeters;

  /** The volts the drive motor needs to overcome friction, whichever way the wheel rolls. */
  private double driveKs;

  /** The volts the drive motor needs for each meter a second the wheel rolls at. */
  private double driveKv;

  /**
   * The volts the drive motors need for each newton the wheel pushes the robot with, on top of what
   * the wheel's speed takes. A force at the tread is a torque at the wheel (times the radius),
   * which is a smaller torque at the motors (divided by the gearing and what the gearing loses),
   * which is a current through each motor (divided by the motor's torque constant), which is a
   * voltage across its windings (times their resistance).
   */
  private final double voltsPerNewton;

  /**
   * @param index which module this is, from 0 to 3, in the order of {@link #NAMES}
   * @param type the module as the robot's definition has it: its wheel, gearing and motors
   */
  Module(ModuleIO io, int index, DrivetrainGeometry.ModuleType type) {
    this.io = io;
    this.inputsKey = "Drive/Module" + NAMES.get(index);
    this.wheelRadiusMeters = type.wheelRadius().in(Meters);
    Actuator.Motors motors = type.driveMotors();
    voltsPerNewton =
        wheelRadiusMeters
            * motors.motor().model().R
            / (type.driveGearing().ratio()
                * type.driveGearing().efficiency()
                * motors.count()
                * motors.motor().model().Kt);
  }

  /**
   * Takes new gains and limits. The drive's feedforward is worked out here each loop, so its gains
   * are kept here. Everything else goes to the motor controllers.
   */
  void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    driveKs = gains.driveKs();
    driveKv = gains.driveKv();
    io.configure(gains, limits);
  }

  /** Reads the hardware. In a replay there is none, and the log supplies the inputs next. */
  void readInputs() {
    io.updateInputs(inputs);
  }

  /** Logs what was read. In a replay, this is where the inputs are taken from the log instead. */
  void logInputs() {
    Logger.processInputs(inputsKey, inputs);
  }

  /**
   * Drives the wheel toward a speed and a direction, with nothing asked of it beyond keeping that
   * speed.
   *
   * @return what the module was told, which may be the wheel reversed and slowed ({@link
   *     #runSetpoint(SwerveModuleVelocity, double, double)})
   */
  SwerveModuleVelocity runSetpoint(SwerveModuleVelocity desired) {
    return runSetpoint(desired, 0, 0);
  }

  /**
   * Drives the wheel toward a speed and a direction, while the wheel also pushes the robot with a
   * force: its share of what an acceleration of the robot takes. The part of the force that lies
   * along the wheel is added to the drive motor's feedforward as volts, so that the wheel speeds up
   * with the robot and does not trail it.
   *
   * @param desired the wheel's speed along the floor and its direction, relative to the robot
   * @param forceForwardNewtons the force's part along the robot's forward axis
   * @param forceLeftNewtons the force's part along the robot's left axis
   * @return what the module was told: the wheel may be reversed rather than turned more than a
   *     quarter turn, and slowed while it still points the wrong way
   */
  SwerveModuleVelocity runSetpoint(
      SwerveModuleVelocity desired, double forceForwardNewtons, double forceLeftNewtons) {
    SwerveModuleVelocity target = reachable(desired);

    double metersPerSecond = target.velocity;
    double forceAlongWheelNewtons =
        forceForwardNewtons * target.angle.getCos() + forceLeftNewtons * target.angle.getSin();
    double feedforwardVolts =
        driveKs * Math.signum(metersPerSecond)
            + driveKv * metersPerSecond
            + voltsPerNewton * forceAlongWheelNewtons;
    io.setDriveVelocity(metersPerSecond / wheelRadiusMeters, feedforwardVolts);
    io.setSteerAngle(target.angle);
    return target;
  }

  /**
   * What the wheel is told for what is asked of it, from the direction it points now.
   *
   * <p>A wheel asked to stand still keeps its direction. Any other wheel turns at most a quarter
   * turn: past that, it is quicker to turn to the opposite direction and roll backward. While it
   * still points away from where it is headed, it is slowed by the cosine of the angle between, so
   * that it does not drag the robot sideways on the way.
   */
  private SwerveModuleVelocity reachable(SwerveModuleVelocity desired) {
    Rotation2d pointsNow = inputs.steerAngle;
    boolean askedToStandStill = Math.abs(desired.velocity) < STEER_HOLD_SPEED_MPS;
    if (askedToStandStill) {
      return new SwerveModuleVelocity(0.0, pointsNow);
    }
    return desired.optimize(pointsNow).cosineScale(pointsNow);
  }

  /**
   * Points the wheel at an angle, relative to the robot, with the drive wheel held still. Unlike
   * driving, it takes no shortcut by reversing the wheel, so the steering makes the whole turn it
   * is asked for: what a steering test needs, and what locking the wheels uses.
   */
  void holdSteer(Rotation2d angle) {
    io.setDriveVelocity(0, 0);
    io.setSteerAngle(angle);
  }

  /** Stops both motors. */
  void stop() {
    io.stop();
  }

  /** The steering motor's voltage, as of this loop. */
  double steerAppliedVolts() {
    return inputs.steerAppliedVolts;
  }

  /** The most current through the steering motor since the last loop, either way. */
  double steerCurrentAmps() {
    return inputs.steerStatorCurrentAmps;
  }

  /** The module's devices that didn't take their configuration at startup, or empty. */
  String unconfigured() {
    return inputs.unconfigured;
  }

  /** How far the wheel has rolled, in meters, and which way it points. */
  SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionRad * wheelRadiusMeters, inputs.steerAngle);
  }

  /** How fast the wheel rolls along the floor, in meters a second, and which way it points. */
  SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(
        inputs.driveVelocityRadPerSec * wheelRadiusMeters, inputs.steerAngle);
  }

  /** Which way the wheel points, as its module measures it. */
  Rotation2d steerAngle() {
    return inputs.steerAngle;
  }

  /** How far the wheel has turned, in radians. */
  double wheelRotationRadians() {
    return inputs.drivePositionRad;
  }

  /** Whether both of the module's motor controllers answer. */
  boolean isConnected() {
    return inputs.driveConnected && inputs.steerConnected;
  }

  /**
   * How many high-rate readings arrived this loop, for odometry. A reading is a time, a wheel
   * position and a steer angle, so only as many count as all three have.
   */
  int odometryReadings() {
    return Math.min(
        inputs.odometryTimestampsNanos.length,
        Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometrySteerAnglesRad.length));
  }

  /** When a high-rate reading was taken, in nanoseconds on the robot's clock. */
  long odometryTimestampNanos(int reading) {
    return inputs.odometryTimestampsNanos[reading];
  }

  /** How far the wheel had rolled, in meters, at a reading. */
  double odometryDistanceMeters(int reading) {
    return inputs.odometryDrivePositionsRad[reading] * wheelRadiusMeters;
  }

  /** Which way the wheel pointed, in radians, at a reading. */
  double odometryAngleRadians(int reading) {
    return inputs.odometrySteerAnglesRad[reading];
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.michaelgrundvig.frc.runtime.behavior.Debounce;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import java.util.Queue;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;

/**
 * The drivetrain's gyro on a CTRE Pigeon 2: yaw at 250 Hz for odometry, and the attitude and
 * acceleration once a loop (acceleration twice, since a collision's jolt is brief).
 *
 * <p>The Pigeon 2's axes are WPILib's (+x forward, +y left, +z up; yaw counterclockwise, pitch nose
 * down), so its angles pass through as they are. Its yaw, pitch, and roll follow the mount pose
 * saved on the Pigeon, set in Phoenix Tuner X (its mount calibration); its accelerometer doesn't
 * (CTRE's Pigeon 2.0 User's Guide, "Enclosure Orientated Signals": it reads along the case's own
 * axes). So this turns the accelerometer's axes into the robot's by the Pigeon's orientation as the
 * robot's definition gives it ({@code ImuMount.orientation}), which the accelerometer check in
 * docs/calibration.md confirms before localization trusts it. Its accelerometer reports up to ±2 g
 * on each axis.
 *
 * <p>The definition's orientation isn't written to the Pigeon: how Phoenix 6 takes a mount pose's
 * angles (the case's turn on the robot, or the turn that undoes it) isn't settled from its
 * documentation, so the two are set apart for now, and must agree.
 */
final class GyroIOPigeon2 implements GyroIO {
  /**
   * How often the Pigeon sends its accelerations: 100 Hz, twice a robot loop. A collision's jolt is
   * brief, and a reading once a loop could miss most of it.
   */
  private static final double ACCELERATION_HZ = 100;

  /**
   * How long the Pigeon may go without answering before it is called disconnected. One missed frame
   * is a glitch, not a lost device.
   *
   * <p>The half second is kept in whole nanoseconds ({@link Debounce}), not by WPILib's debouncer,
   * whose seconds in a double put the edge a loop early or late depending on what the clock reads.
   * So every driver, and the simulator, calls a device silent in the same loop.
   */
  private static final double SILENT_AFTER_SECONDS = 0.5;

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private final StatusSignal<LinearAcceleration> accelerationZ;

  /** How the Pigeon is turned on the robot: turns its case's axes into the robot's. */
  private final Rotation3d orientation;

  private final double radiansPerYawUnit;
  private final double yawScale;
  private final Queue<Long> odometryTimes;
  private final Queue<Double> odometryYaws;

  /** The last high-rate yaw that came, in radians. */
  private double lastYawRadians;

  private final Debounce connected = new Debounce(SILENT_AFTER_SECONDS, DebounceType.FALLING);

  /**
   * @param yawScale the gyro's measured scale: its readings times this are the true rotation
   * @param orientation how the Pigeon is turned on the robot, from the robot's definition: the
   *     rotation that takes its case's axes into the robot's
   */
  GyroIOPigeon2(
      int id, CANBus bus, double yawScale, Rotation3d orientation, PhoenixOdometryThread odometry) {
    this.yawScale = yawScale;
    this.orientation = orientation;
    Pigeon2 pigeon = new Pigeon2(id, bus);
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    accelerationX = pigeon.getAccelerationX();
    accelerationY = pigeon.getAccelerationY();
    accelerationZ = pigeon.getAccelerationZ();
    // The odometry thread records raw values; this turns the yaw's unit (degrees) into radians.
    radiansPerYawUnit = yaw.getValue().unit().toBaseUnits(1);

    // How often the Pigeon sends each signal: the yaw as fast as the odometry thread reads it, the
    // rest of what is read here once a robot loop, and the accelerations twice a loop. Then Phoenix
    // is told to stop every frame whose rate was not set here, to keep the CAN bus's load down.
    yaw.setUpdateFrequency(PhoenixOdometryThread.FREQUENCY_HZ);
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, yawVelocity, pitch, roll);
    BaseStatusSignal.setUpdateFrequencyForAll(
        ACCELERATION_HZ, accelerationX, accelerationY, accelerationZ);
    pigeon.optimizeBusUtilization();

    // The thread reads its own copy of the yaw, so it never shares one with this loop.
    odometryTimes = odometry.timestamps();
    odometryYaws = odometry.register(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean answered =
        BaseStatusSignal.refreshAll(
                yaw, yawVelocity, pitch, roll, accelerationX, accelerationY, accelerationZ)
            .isOK();
    inputs.connected = connected.calculate(answered);
    inputs.yaw = new Rotation2d(yawScale * yaw.getValue().in(Radians));
    inputs.yawVelocityRadPerSec = yawScale * yawVelocity.getValue().in(RadiansPerSecond);
    inputs.pitchRad = pitch.getValue().in(Radians);
    inputs.rollRad = roll.getValue().in(Radians);
    Translation3d acceleration =
        robotAxes(
            orientation,
            accelerationX.getValue().in(MetersPerSecondPerSecond),
            accelerationY.getValue().in(MetersPerSecondPerSecond),
            accelerationZ.getValue().in(MetersPerSecondPerSecond));
    inputs.accelerationXMetersPerSecSq = acceleration.getX();
    inputs.accelerationYMetersPerSecSq = acceleration.getY();

    drainOdometryYaws(inputs);
  }

  /**
   * Takes every high-rate yaw the odometry thread has recorded since the last loop, oldest first,
   * with the time each was measured. The drivetrain holds the thread's lock while it reads, so
   * these are in step with the modules' readings.
   */
  private void drainOdometryYaws(GyroIOInputs inputs) {
    int readings = Math.min(odometryTimes.size(), odometryYaws.size());
    long[] timesNanos = new long[readings];
    double[] yawsRadians = new double[readings];
    for (int reading = 0; reading < readings; reading++) {
      Long timeNanos = odometryTimes.poll();
      Double yaw = odometryYaws.poll();
      if (timeNanos == null || yaw == null) {
        break; // cannot happen while the lock is held
      }
      // The thread records NaN for a yaw that didn't come. The last yaw that did come stands in
      // for it, as the Pigeon's last frame would, and the Pigeon's connection says when it's lost.
      boolean yawCame = !yaw.isNaN();
      if (yawCame) {
        lastYawRadians = yawScale * yaw * radiansPerYawUnit;
      }
      timesNanos[reading] = timeNanos;
      yawsRadians[reading] = lastYawRadians;
    }
    inputs.odometryTimestampsNanos = timesNanos;
    inputs.odometryYawsRad = yawsRadians;
  }

  /** A vector the case reads, along the robot's axes. */
  static Translation3d robotAxes(Rotation3d orientation, double x, double y, double z) {
    return new Translation3d(x, y, z).rotateBy(orientation);
  }
}

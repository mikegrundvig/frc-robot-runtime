package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import java.util.Queue;
import org.wpilib.math.filter.Debouncer;
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
  private static final double ACCELERATION_HZ = 100;

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
  private double lastYaw;

  /** Disconnected once silent for half a second, not at one glitch. */
  private final Debouncer connected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

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
    boolean ok =
        BaseStatusSignal.refreshAll(
                yaw, yawVelocity, pitch, roll, accelerationX, accelerationY, accelerationZ)
            .isOK();
    inputs.connected = connected.calculate(ok);
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

    // The drivetrain holds the odometry thread's lock while reading, so these are in step with the
    // modules' readings.
    int readings = Math.min(odometryTimes.size(), odometryYaws.size());
    long[] times = new long[readings];
    double[] yaws = new double[readings];
    for (int k = 0; k < readings; k++) {
      Long time = odometryTimes.poll();
      Double reading = odometryYaws.poll();
      if (time == null || reading == null) {
        break; // cannot happen while the lock is held
      }
      // A yaw that didn't come reads no number: the last that did stands, as the Pigeon's last
      // frame would, and its connection says when it's lost.
      if (!reading.isNaN()) {
        lastYaw = yawScale * reading * radiansPerYawUnit;
      }
      times[k] = time;
      yaws[k] = lastYaw;
    }
    inputs.odometryTimestampsNanos = times;
    inputs.odometryYawsRad = yaws;
  }

  /** A vector the case reads, along the robot's axes. */
  static Translation3d robotAxes(Rotation3d orientation, double x, double y, double z) {
    return new Translation3d(x, y, z).rotateBy(orientation);
  }
}

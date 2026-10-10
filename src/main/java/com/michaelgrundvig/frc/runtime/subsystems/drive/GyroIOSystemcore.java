package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.michaelgrundvig.frc.localization.Timestamps;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.RobotController;

/**
 * Systemcore's onboard IMU as a gyro: the backup to the main gyro, and its cross-check. Read once a
 * loop (its fused yaw updates at 400 Hz, by the alpha spec); the drivetrain interpolates between
 * readings when it has to turn odometry with it.
 *
 * <p>WPILib reports no connection status for it, so it counts as connected; a failure shows as the
 * drivetrain's cross-check disagreeing with the gyro. Its accelerometer reads the board's axes, and
 * mounted flat, its pitch and roll are about them too; the board's orientation on the robot, from
 * the robot's definition ({@code ImuMount.orientation}), turns both into the robot's: a board
 * turned a quarter turn reads the robot's pitch as its roll. On an edge, WPILib gives the angles
 * for that mount, which are taken as the robot's. Unconfirmed on a real unit until checked against
 * another IMU: its axes, signs, scale, and drift.
 */
public final class GyroIOSystemcore implements GyroIO {
  private final OnboardIMU imu;
  private final boolean flat;
  private final Rotation3d orientation;
  private boolean read;
  private double lastYaw;
  private long lastTime;

  /**
   * @param orientation how the board is turned on the robot, from the robot's definition: the
   *     rotation that takes the board's axes into the robot's. How it's mounted, which sets the
   *     axis WPILib takes its yaw about, follows from it ({@link #mount}).
   */
  public GyroIOSystemcore(Rotation3d orientation) {
    OnboardIMU.MountOrientation mount = mount(orientation);
    imu = new OnboardIMU(mount);
    flat = mount == OnboardIMU.MountOrientation.FLAT;
    this.orientation = orientation;
  }

  /**
   * How Systemcore is mounted, which WPILib's IMU takes its yaw by, from how the board is turned on
   * the robot: flat when its face points up or down; on an edge, landscape when its long side lies
   * level, portrait when its short side does. Its long side is taken to be its x axis: unconfirmed
   * on a real unit, as its axes are.
   */
  static OnboardIMU.MountOrientation mount(Rotation3d orientation) {
    Translation3d face = new Translation3d(0, 0, 1).rotateBy(orientation);
    if (Math.abs(face.getZ()) > Math.sqrt(0.5)) {
      return OnboardIMU.MountOrientation.FLAT;
    }
    Translation3d longSide = new Translation3d(1, 0, 0).rotateBy(orientation);
    return Math.abs(longSide.getZ()) < Math.sqrt(0.5)
        ? OnboardIMU.MountOrientation.LANDSCAPE
        : OnboardIMU.MountOrientation.PORTRAIT;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    long now = RobotController.getTime();
    double yaw = imu.getYawRadians();
    inputs.connected = true;
    inputs.yaw = new Rotation2d(yaw);
    // From successive readings, which works however the board is mounted.
    inputs.yawVelocityRadPerSec =
        read && now > lastTime
            ? MathUtil.angleModulus(yaw - lastYaw) / Timestamps.toSeconds(now - lastTime)
            : 0;
    read = true;
    lastYaw = yaw;
    lastTime = now;
    double[] attitude =
        flat
            ? robotAttitude(orientation, imu.getAngleX(), imu.getAngleY())
            : new double[] {imu.getAngleX(), imu.getAngleY()};
    inputs.rollRad = attitude[0];
    inputs.pitchRad = attitude[1];
    Translation3d acceleration =
        robotAxes(orientation, imu.getAccelX(), imu.getAccelY(), imu.getAccelZ());
    inputs.accelerationXMetersPerSecSq = acceleration.getX();
    inputs.accelerationYMetersPerSecSq = acceleration.getY();
    inputs.odometryTimestampsNanos = new long[] {now};
    inputs.odometryYawsRad = new double[] {yaw};
  }

  /**
   * The robot's roll and pitch, from the board's: which way is up along the board's axes, turned
   * into the robot's. WPILib's attitude: a positive roll is the left side up, a positive pitch the
   * nose down.
   *
   * @return the robot's roll and pitch, in radians, in that order
   */
  static double[] robotAttitude(Rotation3d orientation, double boardRoll, double boardPitch) {
    Translation3d up =
        new Translation3d(
                -Math.sin(boardPitch),
                Math.sin(boardRoll) * Math.cos(boardPitch),
                Math.cos(boardRoll) * Math.cos(boardPitch))
            .rotateBy(orientation);
    return new double[] {
      Math.atan2(up.getY(), up.getZ()), Math.asin(Math.clamp(-up.getX(), -1.0, 1.0))
    };
  }

  /** What the board's accelerometer reads, along the robot's axes. */
  static Translation3d robotAxes(Rotation3d orientation, double x, double y, double z) {
    return new Translation3d(x, y, z).rotateBy(orientation);
  }
}

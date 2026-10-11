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
  /**
   * The robot's tilt.
   *
   * @param rollRadians how far it has rolled about its forward axis: positive is its left side up
   * @param pitchRadians how far it has pitched about its left axis: positive is its nose down
   */
  record Attitude(double rollRadians, double pitchRadians) {}

  private final OnboardIMU imu;

  /** Whether the board lies flat on the robot, its face up or down, and not on an edge. */
  private final boolean mountedFlat;

  /** How the board is turned on the robot: the rotation that takes its axes into the robot's. */
  private final Rotation3d orientation;

  /** Whether the IMU has been read before, so that there is a reading to take a yaw rate from. */
  private boolean hasReadBefore;

  private double lastYawRadians;
  private long lastReadNanos;

  /**
   * @param orientation how the board is turned on the robot, from the robot's definition: the
   *     rotation that takes the board's axes into the robot's. How it's mounted, which sets the
   *     axis WPILib takes its yaw about, follows from it ({@link #mount}).
   */
  public GyroIOSystemcore(Rotation3d orientation) {
    OnboardIMU.MountOrientation mount = mount(orientation);
    imu = new OnboardIMU(mount);
    mountedFlat = mount == OnboardIMU.MountOrientation.FLAT;
    this.orientation = orientation;
  }

  /**
   * How Systemcore is mounted, which WPILib's IMU takes its yaw by, from how the board is turned on
   * the robot: flat when its face points up or down; on an edge, landscape when its long side lies
   * level, portrait when its short side does. Its long side is taken to be its x axis: unconfirmed
   * on a real unit, as its axes are.
   */
  static OnboardIMU.MountOrientation mount(Rotation3d orientation) {
    // An axis whose vertical part is more than this lies within 45° of straight up or down.
    double nearerVerticalThanLevel = Math.sqrt(0.5);
    Translation3d face = new Translation3d(0, 0, 1).rotateBy(orientation);
    boolean facePointsUpOrDown = Math.abs(face.getZ()) > nearerVerticalThanLevel;
    if (facePointsUpOrDown) {
      return OnboardIMU.MountOrientation.FLAT;
    }
    Translation3d longSide = new Translation3d(1, 0, 0).rotateBy(orientation);
    boolean longSideLiesLevel = Math.abs(longSide.getZ()) < nearerVerticalThanLevel;
    if (longSideLiesLevel) {
      return OnboardIMU.MountOrientation.LANDSCAPE;
    }
    return OnboardIMU.MountOrientation.PORTRAIT;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    long now = RobotController.getTime();
    double yaw = imu.getYawRadians();
    inputs.connected = true;
    inputs.yaw = new Rotation2d(yaw);
    inputs.yawVelocityRadPerSec = yawRate(yaw, now);
    hasReadBefore = true;
    lastYawRadians = yaw;
    lastReadNanos = now;

    double boardRoll = imu.getAngleX();
    double boardPitch = imu.getAngleY();
    if (mountedFlat) {
      Attitude robots = robotAttitude(orientation, boardRoll, boardPitch);
      inputs.rollRad = robots.rollRadians();
      inputs.pitchRad = robots.pitchRadians();
    } else {
      // On an edge, WPILib already gives the angles for that mount.
      inputs.rollRad = boardRoll;
      inputs.pitchRad = boardPitch;
    }
    Translation3d acceleration =
        robotAxes(orientation, imu.getAccelX(), imu.getAccelY(), imu.getAccelZ());
    inputs.accelerationXMetersPerSecSq = acceleration.getX();
    inputs.accelerationYMetersPerSecSq = acceleration.getY();
    inputs.odometryTimestampsNanos = new long[] {now};
    inputs.odometryYawsRad = new double[] {yaw};
  }

  /**
   * How fast the yaw is turning, in radians a second, from this reading and the one before. Taking
   * it from successive yaws works however the board is mounted. The first reading has none before
   * it, and two readings at the same time say nothing, so both give zero.
   */
  private double yawRate(double yawRadians, long nowNanos) {
    boolean hasAnEarlierReading = hasReadBefore && nowNanos > lastReadNanos;
    if (!hasAnEarlierReading) {
      return 0;
    }
    double turnedRadians = MathUtil.angleModulus(yawRadians - lastYawRadians);
    return turnedRadians / Timestamps.toSeconds(nowNanos - lastReadNanos);
  }

  /**
   * The robot's roll and pitch, from the board's, for a board mounted flat. The board's roll and
   * pitch say which way is up along the board's own axes. That direction is turned into the robot's
   * axes, and the robot's roll and pitch are read back from it.
   */
  static Attitude robotAttitude(Rotation3d orientation, double boardRoll, double boardPitch) {
    Translation3d upAlongTheBoard =
        new Translation3d(
            -Math.sin(boardPitch),
            Math.sin(boardRoll) * Math.cos(boardPitch),
            Math.cos(boardRoll) * Math.cos(boardPitch));
    Translation3d up = upAlongTheBoard.rotateBy(orientation);
    double rollRadians = Math.atan2(up.getY(), up.getZ());
    double pitchRadians = Math.asin(Math.clamp(-up.getX(), -1.0, 1.0));
    return new Attitude(rollRadians, pitchRadians);
  }

  /** What the board's accelerometer reads, along the robot's axes. */
  static Translation3d robotAxes(Rotation3d orientation, double x, double y, double z) {
    return new Translation3d(x, y, z).rotateBy(orientation);
  }
}

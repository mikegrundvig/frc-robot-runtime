package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.michaelgrundvig.frc.runtime.behavior.Debounce;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixRestarts;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixStartup;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
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
 * down), so its angles pass through as they are.
 *
 * <p><b>Its mount.</b> As the robot starts, the Pigeon is sent its whole configuration, built from
 * the robot's definition: its mount pose is the definition's orientation ({@code
 * ImuMount.orientation}), and everything else is Phoenix's default. So a mount pose or a gyro trim
 * left on it from other code or from Phoenix Tuner X doesn't survive a restart of the robot code.
 * Its yaw, pitch, and roll follow that mount pose; its accelerometer doesn't (CTRE's Pigeon 2.0
 * User's Guide, "Enclosure Orientated Signals": it reads along the case's own axes). So this turns
 * the accelerometer's axes into the robot's by the same orientation, which the accelerometer check
 * in docs/calibration.md confirms before localization trusts it. Its accelerometer reports up to ±2
 * g on each axis.
 *
 * <p>Unconfirmed: that Phoenix takes a mount pose as the Pigeon's own turn on the robot, the
 * definition's orientation, and not the turn that undoes it. CTRE's documentation doesn't say, and
 * Phoenix's simulated Pigeon ignores its mount pose. For a Pigeon mounted level with its x axis
 * forward, the orientation is zero, and the two readings agree. The level check in
 * docs/calibration.md shows it for any other.
 *
 * <p><b>Restarts.</b> A Pigeon that loses power comes back counting its yaw from zero. Phoenix says
 * when it has sent the frame a device sends as it starts, and then the heading is held where it was
 * for that loop and goes on from there: the Pigeon's new count is taken up from its first reading
 * after the restart ({@link GyroIOInputs#restarted}). The robot's turn while the Pigeon was off,
 * and in that one loop, is not counted, and the cameras put it right.
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

  /** The heading as last read, high-rate or once a loop, in radians: what a restart holds. */
  private double lastYawRadians;

  private final Debounce connected = new Debounce(SILENT_AFTER_SECONDS, DebounceType.FALLING);

  /**
   * Whether the Pigeon restarted since the last loop, from the frame a device sends as it starts.
   */
  private final PhoenixRestarts restarts;

  /**
   * Whether the heading follows the Pigeon's count, or waits to take up its count after a restart.
   */
  private enum Count {
    /** The heading is the Pigeon's yaw, scaled, plus the offset taken up at its last restart. */
    FOLLOWED,
    /** The Pigeon restarted: the heading stands until its first reading of its new count. */
    TAKEN_UP_AT_ITS_NEXT_READING
  }

  private Count count = Count.FOLLOWED;

  /**
   * What is added to the Pigeon's yaw, scaled, to make the heading, in radians: zero until it first
   * restarts, and then whatever carries the heading on from where it stood.
   */
  private double offsetRadians;

  /**
   * @param tries how many times the Pigeon is sent its configuration as the robot starts ({@code
   *     PhoenixStartup#tries})
   * @param yawScale the gyro's measured scale: its readings times this are the true rotation
   * @param orientation how the Pigeon is turned on the robot, from the robot's definition: the
   *     rotation that takes its case's axes into the robot's
   */
  GyroIOPigeon2(
      int id,
      CANBus bus,
      int tries,
      double yawScale,
      Rotation3d orientation,
      PhoenixOdometryThread odometry) {
    this(id, bus, tries, yawScale, orientation, odometry, ParentDevice::getResetOccurredChecker);
  }

  /**
   * The same, with where the Pigeon's start frame is heard. For the tests: Phoenix's simulated
   * Pigeon never restarts, so a test says when it has.
   */
  GyroIOPigeon2(
      int id,
      CANBus bus,
      int tries,
      double yawScale,
      Rotation3d orientation,
      PhoenixOdometryThread odometry,
      Function<ParentDevice, BooleanSupplier> startFrames) {
    this.yawScale = yawScale;
    this.orientation = orientation;
    Pigeon2 pigeon = new Pigeon2(id, bus);
    Pigeon2Configuration settings = settings(orientation);
    PhoenixStartup.confirmed(
        tries, () -> pigeon.getConfigurator().apply(settings, PhoenixStartup.TIMEOUT_SECONDS));
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
    // Asked from here, once it is configured: the frame it sent as it and the robot started is not
    // a restart.
    restarts = new PhoenixRestarts(startFrames.apply(pigeon));
  }

  /**
   * The Pigeon's whole configuration: Phoenix's defaults, with its mount pose from the definition's
   * orientation, in degrees: yaw about the robot's z axis, pitch about its y axis, roll about its x
   * axis.
   */
  static Pigeon2Configuration settings(Rotation3d orientation) {
    Pigeon2Configuration settings = new Pigeon2Configuration();
    settings.MountPose.MountPoseYaw = Math.toDegrees(orientation.getZ());
    settings.MountPose.MountPosePitch = Math.toDegrees(orientation.getY());
    settings.MountPose.MountPoseRoll = Math.toDegrees(orientation.getX());
    return settings;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    boolean answered =
        BaseStatusSignal.refreshAll(
                yaw, yawVelocity, pitch, roll, accelerationX, accelerationY, accelerationZ)
            .isOK();
    boolean restarted = restarts.happened();
    if (restarted) {
      count = Count.TAKEN_UP_AT_ITS_NEXT_READING;
    }
    inputs.connected = connected.calculate(answered);
    inputs.restarted = restarted;
    drainOdometryYaws(inputs, restarted);
    boolean yawCame = yaw.getStatus().isOK();
    double scaledYaw = yawScale * yaw.getValue().in(Radians);
    boolean takesUpTheNewCount =
        count == Count.TAKEN_UP_AT_ITS_NEXT_READING && !restarted && yawCame;
    if (takesUpTheNewCount) {
      offsetRadians = lastYawRadians - scaledYaw;
      count = Count.FOLLOWED;
    }
    if (count == Count.FOLLOWED) {
      lastYawRadians = scaledYaw + offsetRadians;
    }
    inputs.yaw = new Rotation2d(lastYawRadians);
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
  }

  /**
   * Takes every high-rate yaw the odometry thread has recorded since the last loop, oldest first,
   * with the time each was measured. The drivetrain holds the thread's lock while it reads, so
   * these are in step with the modules' readings.
   *
   * <p>In the loop the Pigeon restarted, its readings may be of its old count or its new one, and
   * nothing says which: each stands at the heading as it was. After that, the first reading takes
   * up the new count, so the heading goes on from there.
   *
   * @param restarted whether the Pigeon restarted since the last loop
   */
  private void drainOdometryYaws(GyroIOInputs inputs, boolean restarted) {
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
      if (yawCame && !restarted) {
        lastYawRadians = followed(yawScale * yaw * radiansPerYawUnit);
      }
      timesNanos[reading] = timeNanos;
      yawsRadians[reading] = lastYawRadians;
    }
    inputs.odometryTimestampsNanos = timesNanos;
    inputs.odometryYawsRad = yawsRadians;
  }

  /**
   * The heading for a yaw the Pigeon read, scaled: its count plus the offset, taking up a new count
   * after a restart from where the heading stood.
   */
  private double followed(double scaledYaw) {
    if (count == Count.TAKEN_UP_AT_ITS_NEXT_READING) {
      offsetRadians = lastYawRadians - scaledYaw;
      count = Count.FOLLOWED;
    }
    return scaledYaw + offsetRadians;
  }

  /** A vector the case reads, along the robot's axes. */
  static Translation3d robotAxes(Rotation3d orientation, double x, double y, double z) {
    return new Translation3d(x, y, z).rotateBy(orientation);
  }
}

package com.michaelgrundvig.frc.simulation;

import java.util.Random;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;

/**
 * A simulated IMU: its gyro, and with it the robot's attitude and its accelerometer.
 *
 * <p><b>The gyro's model.</b> Its yaw reading is
 *
 * <pre>
 *   reading = offset + scale · (every turn the robot truly made) + drift rate · (time run) + noise
 * </pre>
 *
 * <p>The scale error, the steady drift and the noise are {@link SensorErrors.GyroErrors}. The noise
 * is drawn fresh for each reading and does not add up. The drift is a steady rate, not a random
 * walk. The offset is zero until the gyro's errors are changed or it is reset, and then it is
 * whatever keeps the reading from jumping, or sets it to zero.
 *
 * <p>Like a real gyro, it reads zero wherever the robot pointed when it started, and it follows the
 * robot when someone picks it up and turns it, since that is a turn the robot truly made ({@link
 * ChassisTruth#yawTurnedRadians}).
 *
 * <p><b>The rest of the IMU.</b> The robot's pitch and roll are read exactly as they truly are. Its
 * acceleration is read through its own accelerometer ({@link AccelerometerModel}), which feels a
 * hit's jolt on its own.
 *
 * <p><b>How it fails</b>, as a real one does, at any moment: disconnected (no readings at all, and
 * it feels no jolt), frozen (still reporting, the same yaw every time), erring differently from now
 * on, or reset to zero, as one that browns out and restarts does.
 *
 * <p>The robot program's IO adapter reports it.
 */
public final class GyroModel {
  private final ChassisTruth chassis;
  private final AccelerometerModel accelerometer;
  private final Random noise;

  /** How it erred as the robot was built: what undoing a change of its errors goes back to. */
  private final SensorErrors.GyroErrors built;

  /** How it errs now, and the same in the units its reading is worked out in. */
  private SensorErrors.GyroErrors errors;

  private double scale;
  private double driftRadiansPerSecond;
  private double noiseRadians;

  /**
   * Added to its reading so that the reading carries on from where it was when its errors change,
   * or starts again from zero when it resets. Zero until either happens.
   */
  private double offsetRadians;

  /** Its latest yaw reading: what it goes on reporting while frozen. */
  private double latestYawRadians;

  private boolean connected = true;
  private boolean frozen;

  /**
   * @param chassis the chassis it rides on, as the physics keeps it
   * @param errors how it errs, as the robot was built
   * @param accelerometer its own accelerometer
   * @param seed for its noise ({@link Seeds})
   */
  public GyroModel(
      ChassisTruth chassis,
      SensorErrors.GyroErrors errors,
      AccelerometerModel accelerometer,
      long seed) {
    this.chassis = chassis;
    this.accelerometer = accelerometer;
    built = errors;
    this.errors = errors;
    takeErrors(errors);
    noise = new Random(seed);
  }

  /** Unplugged: no readings until it's plugged back in. */
  public void setConnected(boolean connected) {
    this.connected = connected;
  }

  public boolean isConnected() {
    return connected;
  }

  /** Stuck: still reporting, the same yaw every time. */
  public void setFrozen(boolean frozen) {
    this.frozen = frozen;
  }

  public boolean isFrozen() {
    return frozen;
  }

  /**
   * Errs differently from now on: its reading carries on from where it is, then turns at the new
   * scale and drifts at the new rate.
   */
  public void setErrors(SensorErrors.GyroErrors errors) {
    double readingBefore = readingWithoutNoise();
    takeErrors(errors);
    this.errors = errors;
    double readingAfter = readingWithoutNoise();
    offsetRadians += readingBefore - readingAfter;
  }

  /** How it errs now. */
  public SensorErrors.GyroErrors errors() {
    return errors;
  }

  /** How it erred as the robot was built. */
  public SensorErrors.GyroErrors built() {
    return built;
  }

  /** Reads zero from now on wherever the robot points, as a gyro that restarts does. */
  public void reset() {
    offsetRadians -= readingWithoutNoise();
  }

  private void takeErrors(SensorErrors.GyroErrors errors) {
    scale = errors.scale();
    driftRadiansPerSecond = Math.toRadians(errors.driftDegreesPerMinute()) / 60;
    noiseRadians = Math.toRadians(errors.noiseDegrees());
  }

  /** What it reads before noise: the true turn at its scale, with its drift, from its offset. */
  private double readingWithoutNoise() {
    return offsetRadians
        + chassis.yawTurnedRadians() * scale
        + driftRadiansPerSecond * chassis.seconds();
  }

  /** After each physics step: its accelerometer feels it. */
  public void feel(double seconds) {
    accelerometer.step(seconds);
  }

  /** At the end of each loop. */
  public void endLoop(double seconds) {
    accelerometer.endLoop(seconds);
  }

  /** A hit's jolt, along the robot's axes: its next reading feels it. */
  public void jolt(Translation2d robotAxes) {
    accelerometer.jolt(robotAxes);
  }

  /**
   * Takes a yaw reading, as its odometry signal does: with its noise, unless it's frozen, when it
   * reads what it last did.
   *
   * @return whether it read at all: not while unplugged
   */
  public boolean sample() {
    if (!connected) {
      return false;
    }
    // The noise is drawn whether or not the reading is kept, so that freezing the gyro does not
    // change the numbers it draws once it thaws.
    double reading = readingWithoutNoise() + noiseRadians * noise.nextGaussian();
    if (!frozen) {
      latestYawRadians = reading;
    }
    return true;
  }

  /** Its latest yaw reading, in radians. */
  public double yawRadians() {
    return latestYawRadians;
  }

  /** How fast it reads the robot turning, at its scale, in rad/s. */
  public double yawRateRadiansPerSecond() {
    return chassis.yawRateRadiansPerSecond() * scale;
  }

  /** The robot's pitch, as it truly is. */
  public double pitchRadians() {
    return chassis.pitchRadians();
  }

  /** The robot's roll, as it truly is. */
  public double rollRadians() {
    return chassis.rollRadians();
  }

  /**
   * What its accelerometer reads now, once a loop, along the robot's forward and left axes;
   * unplugged, it reads nothing, and forgets any jolt it felt.
   *
   * @return the reading, or null while unplugged
   */
  public @Nullable Translation2d acceleration() {
    if (!connected) {
      accelerometer.missed();
      return null;
    }
    return accelerometer.read();
  }
}

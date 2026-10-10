package com.michaelgrundvig.frc.simulation;

import java.util.Random;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;

/**
 * A simulated IMU: the robot's true rotation, read with the scale error, drift, and noise of {@link
 * SensorErrors.GyroErrors}, and its attitude and acceleration. Like a real gyro, it reads zero
 * wherever the robot pointed when it started, and it follows the robot when someone picks it up and
 * turns it. The rest of the IMU reads the robot's attitude as it truly is, and its acceleration
 * through its own accelerometer ({@link AccelerometerModel}), which feels a hit's jolt on its own;
 * unplugged, it feels none. It can fail as a real one does, at any moment: disconnected (no
 * readings at all), frozen (still reporting, the same yaw every time), erring differently from now
 * on, or reset to zero, as one that browns out and restarts does.
 *
 * <p>It reads the chassis's truth ({@link ChassisTruth}): every turn the robot truly made, and how
 * long the world has run, for its drift. The robot program's IO adapter reports it.
 */
public final class GyroModel {
  private final ChassisTruth chassis;
  private final SensorErrors.GyroErrors built;
  private SensorErrors.GyroErrors errors;
  private double scale;
  private double driftRadiansPerSecond;
  private double noiseRadians;
  private final Random noise;
  private final AccelerometerModel accelerometer;

  /**
   * Added to its reading so the reading carries on from where it was when its errors change or it
   * resets: none until one does.
   */
  private double offsetRadians;

  private double latestYaw;
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
    double before = trueReading();
    takeErrors(errors);
    this.errors = errors;
    offsetRadians += before - trueReading();
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
    offsetRadians -= trueReading();
  }

  private void takeErrors(SensorErrors.GyroErrors errors) {
    scale = errors.scale();
    driftRadiansPerSecond = Math.toRadians(errors.driftDegreesPerMinute()) / 60;
    noiseRadians = Math.toRadians(errors.noiseDegrees());
  }

  /** What it reads before noise: the true turn at its scale, with its drift, from its offset. */
  private double trueReading() {
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
    double reading = trueReading() + noiseRadians * noise.nextGaussian();
    if (!frozen) {
      latestYaw = reading;
    }
    return true;
  }

  /** Its latest yaw reading, in radians. */
  public double yawRadians() {
    return latestYaw;
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

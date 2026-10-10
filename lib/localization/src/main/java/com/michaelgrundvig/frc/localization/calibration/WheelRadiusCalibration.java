package com.michaelgrundvig.frc.localization.calibration;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The wheels' radius, from spinning in place against the gyro (a top team's procedure). Turning in
 * place, every wheel rolls along a circle around the robot's center: when the robot turns θ, a
 * wheel d from the center rolls θ·d, and the wheel radius is that distance over the wheel's own
 * rotation. Tread wears, so the radius shrinks through a season: measure again before each event.
 *
 * <p>The gyro is the reference, so its scale error carries straight into the result (0.3% high
 * reads a 0.3% larger radius): calibrate the gyro's scale first. Spin slowly, so the wheels roll
 * instead of scrubbing.
 */
public final class WheelRadiusCalibration {
  private final double[] distances;
  private final double[] start;
  private final double[] latest;
  private double turned;
  private double lastYaw;
  private boolean started;

  /**
   * @param moduleLocations where each wheel touches the floor, from the robot's center
   */
  public WheelRadiusCalibration(Translation2d[] moduleLocations) {
    distances = new double[moduleLocations.length];
    for (int i = 0; i < moduleLocations.length; i++) {
      distances[i] = moduleLocations[i].getNorm();
      if (!(distances[i] > 0)) {
        throw new IllegalArgumentException("a wheel can't sit at the robot's center");
      }
    }
    start = new double[distances.length];
    latest = new double[distances.length];
  }

  /**
   * Where the spin starts: the gyro's yaw, and each wheel's rotation, in radians.
   *
   * @param gyroYawRadians any zero, wrapped or not
   */
  public void start(double gyroYawRadians, double[] wheelRotationsRadians) {
    requireEveryWheel(wheelRotationsRadians);
    System.arraycopy(wheelRotationsRadians, 0, start, 0, start.length);
    System.arraycopy(wheelRotationsRadians, 0, latest, 0, latest.length);
    lastYaw = gyroYawRadians;
    turned = 0;
    started = true;
  }

  /**
   * A reading during the spin, at least once a loop so the gyro's turn is followed across wraps.
   */
  public void update(double gyroYawRadians, double[] wheelRotationsRadians) {
    if (!started) {
      throw new IllegalStateException("start the spin first");
    }
    requireEveryWheel(wheelRotationsRadians);
    turned += MathUtil.angleModulus(gyroYawRadians - lastYaw);
    lastYaw = gyroYawRadians;
    System.arraycopy(wheelRotationsRadians, 0, latest, 0, latest.length);
  }

  /** How far the robot has turned since the start, by the gyro, either way. */
  public double turnedRadians() {
    return Math.abs(turned);
  }

  /**
   * The wheel radius the spin says, in meters, from all the wheels together: the distance they
   * rolled, by the gyro, over how far they turned. NaN before the wheels have turned.
   */
  public double radiusMeters() {
    double rolled = 0;
    double rotated = 0;
    for (int i = 0; i < distances.length; i++) {
      rolled += Math.abs(turned) * distances[i];
      rotated += Math.abs(latest[i] - start[i]);
    }
    return rotated > 0 ? rolled / rotated : Double.NaN;
  }

  /**
   * Each wheel's own radius, in meters, the same way: a wheel far from the rest is worn
   * differently, or slipping. NaN for a wheel that hasn't turned.
   */
  public double[] radiiMeters() {
    double[] radii = new double[distances.length];
    for (int i = 0; i < distances.length; i++) {
      double rotated = Math.abs(latest[i] - start[i]);
      radii[i] = rotated > 0 ? Math.abs(turned) * distances[i] / rotated : Double.NaN;
    }
    return radii;
  }

  private void requireEveryWheel(double[] wheelRotationsRadians) {
    if (wheelRotationsRadians.length != distances.length) {
      throw new IllegalArgumentException("one rotation for each wheel");
    }
  }
}

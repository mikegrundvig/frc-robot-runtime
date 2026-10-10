package com.michaelgrundvig.frc.localization;

import org.wpilib.math.util.MathUtil;

/**
 * A gyro's recent yaw readings, for its yaw at the moment of each wheel reading: a gyro read less
 * often than the wheels (an IMU read once a loop, while the wheels are read at 250 Hz) still turns
 * every odometry step. Between readings the yaw is interpolated the short way around; at a
 * reading's own time it's that reading exactly; past the newest it holds the newest.
 */
public final class YawTrack {
  private long[] times = new long[0];
  private double[] yaws = new double[0];
  private long previousTime = Long.MIN_VALUE;
  private double previousYaw;

  /**
   * This loop's readings, oldest first. The previous loop's newest reading is kept, so moments
   * before this loop's first reading interpolate from it.
   */
  public void update(long[] timestampsNanos, double[] yawsRadians) {
    if (timestampsNanos.length != yawsRadians.length) {
      throw new IllegalArgumentException("one time for each yaw");
    }
    if (times.length > 0) {
      previousTime = times[times.length - 1];
      previousYaw = yaws[yaws.length - 1];
    }
    times = timestampsNanos.clone();
    yaws = yawsRadians.clone();
  }

  /** Forgets every reading: the gyro was lost, so its old readings say nothing about now. */
  public void clear() {
    times = new long[0];
    yaws = new double[0];
    previousTime = Long.MIN_VALUE;
  }

  /** Whether there's a reading this loop. */
  public boolean hasReadings() {
    return times.length > 0;
  }

  /** The yaw at a moment, in radians. Call only when there's a reading this loop. */
  public double at(long timestampNanos) {
    if (times.length == 0) {
      throw new IllegalStateException("no readings this loop");
    }
    long beforeTime = previousTime;
    double beforeYaw = previousYaw;
    for (int k = 0; k < times.length; k++) {
      if (times[k] == timestampNanos) {
        return yaws[k];
      }
      if (times[k] > timestampNanos) {
        if (beforeTime == Long.MIN_VALUE || beforeTime >= times[k]) {
          return yaws[k]; // before any reading known: the earliest
        }
        double fraction = (double) (timestampNanos - beforeTime) / (times[k] - beforeTime);
        return beforeYaw + fraction * MathUtil.angleModulus(yaws[k] - beforeYaw);
      }
      beforeTime = times[k];
      beforeYaw = yaws[k];
    }
    return yaws[yaws.length - 1];
  }
}

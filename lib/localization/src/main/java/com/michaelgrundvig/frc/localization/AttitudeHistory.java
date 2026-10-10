package com.michaelgrundvig.frc.localization;

/**
 * The robot's recent pitch and roll, for looking up its attitude when a camera image was taken: on
 * a bump the attitude changes within a tenth of a second, about as long as an image takes to
 * arrive. Between readings it interpolates; outside them it holds the nearest.
 */
public final class AttitudeHistory {
  private final long[] time;
  private final double[] pitch;
  private final double[] roll;
  private int newest = -1;
  private int size;

  /** The robot's attitude at a moment, in radians, as {@link CaptureContext} takes it. */
  public record Attitude(double pitchRadians, double rollRadians) {
    public static final Attitude LEVEL = new Attitude(0, 0);
  }

  /**
   * @param capacity how many readings to keep: 25 is half a second at 50 Hz
   */
  public AttitudeHistory(int capacity) {
    if (capacity < 2) {
      throw new IllegalArgumentException("keep at least two readings: " + capacity);
    }
    time = new long[capacity];
    pitch = new double[capacity];
    roll = new double[capacity];
  }

  /** A reading, newer than the last; an older or equal one is ignored. */
  public void add(long timestampNanos, double pitchRadians, double rollRadians) {
    if (size > 0 && timestampNanos <= time[newest]) {
      return;
    }
    newest = (newest + 1) % time.length;
    time[newest] = timestampNanos;
    pitch[newest] = pitchRadians;
    roll[newest] = rollRadians;
    size = Math.min(size + 1, time.length);
  }

  /** The attitude at a moment: level before any reading. */
  public Attitude at(long timestampNanos) {
    if (size == 0) {
      return Attitude.LEVEL;
    }
    int after = newest;
    for (int k = 0; k < size; k++) {
      int slot = Math.floorMod(newest - k, time.length);
      if (time[slot] <= timestampNanos) {
        if (k == 0) {
          return new Attitude(pitch[slot], roll[slot]); // at or after the newest reading
        }
        double fraction = (double) (timestampNanos - time[slot]) / (time[after] - time[slot]);
        return new Attitude(
            pitch[slot] + fraction * (pitch[after] - pitch[slot]),
            roll[slot] + fraction * (roll[after] - roll[slot]));
      }
      after = slot;
    }
    return new Attitude(pitch[after], roll[after]); // before the oldest reading
  }
}

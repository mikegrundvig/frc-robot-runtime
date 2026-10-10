package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.robot.Joint;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The robot's recent joint positions, for looking up where a camera on a joint was when its image
 * was taken: a turret turns a tenth of a turn in the time an image takes to arrive. Between
 * readings it interpolates; outside them it holds the nearest; before any, each joint is at its
 * reference.
 */
public final class JointHistory {
  private final List<Joint> joints;
  private final double[] references;
  private final long[] time;
  private final double[][] positions;
  private int newest = -1;
  private int size;

  /**
   * @param references the joints kept, each at its reference: the robot's joints that move on their
   *     own ({@code RobotMath.references})
   * @param capacity how many readings to keep: 25 is half a second at 50 Hz
   */
  public JointHistory(Map<Joint, Double> references, int capacity) {
    if (capacity < 2) {
      throw new IllegalArgumentException("keep at least two readings: " + capacity);
    }
    joints = List.copyOf(references.keySet());
    this.references = new double[joints.size()];
    int j = 0;
    for (double reference : references.values()) {
      this.references[j++] = reference;
    }
    time = new long[capacity];
    positions = new double[capacity][joints.size()];
  }

  /**
   * A reading, newer than the last; an older or equal one is ignored. A joint it doesn't name is
   * where it was last, or at its reference.
   */
  public void add(long timestampNanos, Map<Joint, Double> read) {
    if (size > 0 && timestampNanos <= time[newest]) {
      return;
    }
    double[] last = size > 0 ? positions[newest] : references;
    int slot = (newest + 1) % time.length;
    for (int j = 0; j < joints.size(); j++) {
      Double position = read.get(joints.get(j));
      positions[slot][j] = position != null ? position : last[j];
    }
    newest = slot;
    time[newest] = timestampNanos;
    size = Math.min(size + 1, time.length);
  }

  /** Every kept joint's position at a moment. */
  public Map<Joint, Double> at(long timestampNanos) {
    if (size == 0) {
      return positions(references, references, 0);
    }
    int after = newest;
    for (int k = 0; k < size; k++) {
      int slot = Math.floorMod(newest - k, time.length);
      if (time[slot] <= timestampNanos) {
        if (k == 0) {
          return positions(positions[slot], positions[slot], 0); // at or after the newest
        }
        double fraction = (double) (timestampNanos - time[slot]) / (time[after] - time[slot]);
        return positions(positions[slot], positions[after], fraction);
      }
      after = slot;
    }
    return positions(positions[after], positions[after], 0); // before the oldest reading
  }

  private Map<Joint, Double> positions(double[] from, double[] to, double fraction) {
    Map<Joint, Double> at = new LinkedHashMap<>();
    for (int j = 0; j < joints.size(); j++) {
      at.put(joints.get(j), from[j] + fraction * (to[j] - from[j]));
    }
    return at;
  }
}

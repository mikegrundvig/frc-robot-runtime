package com.michaelgrundvig.frc.localization;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The pose estimator's history: a ring of odometry steps, each with the filter's state after it, so
 * a late measurement can be applied at the moment it was taken and the steps since replayed. Steps
 * are counted from the oldest kept (0) to the newest ({@link #size()} - 1).
 */
final class OdometryHistory {
  private final int capacity;
  private final long[] time;
  private final double[] dx;
  private final double[] dy;
  private final double[] residual;

  /** Each step's own added position variance for slip, tilt, and collisions: fixed on arrival. */
  private final double[] extra;

  private final YawSource[] turnedBy;
  private final double[] yawAfter;
  private final double[] xAfter;
  private final double[] yAfter;
  private final double[] offsetAfter;

  /** The filter's uncertainty after each step: {@link PoseFilter#uncertainty()} numbers a step. */
  private final double[] uncertaintyAfter;

  private final int uncertainty;

  private int oldest;
  private int size;

  /** How the robot moved lately: its velocity over the field, and its turn rate. */
  record Motion(Translation2d velocity, double yawRate) {}

  /**
   * @param capacity how many steps it keeps
   * @param uncertainty how many numbers the filter's uncertainty takes ({@link
   *     PoseFilter#uncertainty()})
   */
  OdometryHistory(int capacity, int uncertainty) {
    this.capacity = capacity;
    this.uncertainty = uncertainty;
    time = new long[capacity];
    dx = new double[capacity];
    dy = new double[capacity];
    residual = new double[capacity];
    extra = new double[capacity];
    turnedBy = new YawSource[capacity];
    yawAfter = new double[capacity];
    xAfter = new double[capacity];
    yAfter = new double[capacity];
    offsetAfter = new double[capacity];
    uncertaintyAfter = new double[capacity * uncertainty];
  }

  /** Starts over with a single step: where odometry starts. */
  void start(long timestampNanos, YawSource source, double yaw) {
    oldest = 0;
    size = 1;
    time[0] = timestampNanos;
    turnedBy[0] = source;
    yawAfter[0] = yaw;
  }

  /**
   * Adds the newest step, dropping the oldest when the ring is full.
   *
   * @param dxMeters the wheels' travel forward, as much of it as counts
   * @param dyMeters and to the left
   * @param residualMeters the wheels' disagreement
   * @param extraVariance the step's added position variance
   * @param source what turned this step: an IMU, or the wheels standing in
   * @param yaw the unwrapped yaw after the step
   * @return the new step's number
   */
  int add(
      long timestampNanos,
      double dxMeters,
      double dyMeters,
      double residualMeters,
      double extraVariance,
      YawSource source,
      double yaw) {
    if (size == capacity) {
      oldest = slot(1);
      size--;
    }
    int slot = slot(size);
    size++;
    time[slot] = timestampNanos;
    dx[slot] = dxMeters;
    dy[slot] = dyMeters;
    residual[slot] = residualMeters;
    extra[slot] = extraVariance;
    turnedBy[slot] = source;
    yawAfter[slot] = yaw;
    return size - 1;
  }

  int size() {
    return size;
  }

  long time(int step) {
    return time[slot(step)];
  }

  long newestTime() {
    return time(size - 1);
  }

  double dx(int step) {
    return dx[slot(step)];
  }

  double dy(int step) {
    return dy[slot(step)];
  }

  double residual(int step) {
    return residual[slot(step)];
  }

  double extra(int step) {
    return extra[slot(step)];
  }

  YawSource turnedBy(int step) {
    return turnedBy[slot(step)];
  }

  double yawAfter(int step) {
    return yawAfter[slot(step)];
  }

  /** The yaw a fraction of the way through a step. */
  double yawAt(int step, double fraction) {
    double before = yawAfter(step - 1);
    return before + fraction * (yawAfter(step) - before);
  }

  /** The last step taken before a moment, or the oldest. */
  int lastStepBefore(long timestampNanos) {
    int step = size - 1;
    while (step > 0 && time(step) >= timestampNanos) {
      step--;
    }
    return step;
  }

  /** Keeps the filter's state after a step. */
  void store(int step, PoseFilter filter) {
    int slot = slot(step);
    xAfter[slot] = filter.x();
    yAfter[slot] = filter.y();
    offsetAfter[slot] = filter.offset();
    filter.copyUncertaintyTo(uncertaintyAfter, slot * uncertainty);
  }

  /** Puts the filter back in its state after a step. */
  void restore(int step, PoseFilter filter) {
    int slot = slot(step);
    filter.restore(
        xAfter[slot], yAfter[slot], offsetAfter[slot], uncertaintyAfter, slot * uncertainty);
  }

  /** Drops every step but the newest, which becomes the history's start. */
  void keepOnlyNewest() {
    oldest = slot(size - 1);
    size = 1;
  }

  /** Drops steps at or before a moment, always keeping the newest. */
  void dropThrough(long cutoffNanos) {
    while (size > 1 && time(1) <= cutoffNanos) {
      oldest = slot(1);
      size--;
    }
  }

  /**
   * Where the robot was at a moment from the oldest step up to the newest, as the history has it,
   * turned by the current heading offset.
   */
  Pose2d poseAt(long timestampNanos, double offset) {
    int j = size - 1;
    while (j > 0 && time(j - 1) >= timestampNanos) {
      j--;
    }
    if (j == 0) {
      int slot = slot(0);
      return new Pose2d(xAfter[slot], yAfter[slot], new Rotation2d(yawAfter[slot] + offset));
    }
    int before = slot(j - 1);
    int after = slot(j);
    double fraction = (double) (timestampNanos - time[before]) / (time[after] - time[before]);
    return new Pose2d(
        xAfter[before] + fraction * (xAfter[after] - xAfter[before]),
        yAfter[before] + fraction * (yAfter[after] - yAfter[before]),
        new Rotation2d(yawAt(j, fraction) + offset));
  }

  /** How the robot moved over the steps within a window before the newest. */
  Motion motionOver(long windowNanos, double offset) {
    long now = newestTime();
    double moveX = 0;
    double moveY = 0;
    double turned = 0;
    long from = now;
    for (int j = size - 1; j > 0 && time(j) > now - windowNanos; j--) {
      int slot = slot(j);
      int previous = slot(j - 1);
      double heading = yawAfter[previous] + offset;
      moveX += Math.cos(heading) * dx[slot] - Math.sin(heading) * dy[slot];
      moveY += Math.sin(heading) * dx[slot] + Math.cos(heading) * dy[slot];
      turned += yawAfter[slot] - yawAfter[previous];
      from = time[previous];
    }
    double seconds = Timestamps.toSeconds(now - from);
    Translation2d velocity =
        seconds > 0 ? new Translation2d(moveX / seconds, moveY / seconds) : Translation2d.ZERO;
    return new Motion(velocity, seconds > 0 ? turned / seconds : 0);
  }

  private int slot(int step) {
    return (oldest + step) % capacity;
  }
}

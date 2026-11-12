package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import com.michaelgrundvig.frc.world.physical.Vector3;

/**
 * Where a shape is drawn: its own axes, turned and moved into the robot's frame. A shape is written
 * in its frame's coordinates (a camera's arrow along its frame's x, its optical axis), and the
 * frame places it on the robot.
 */
final class Frame {
  /** The robot's own frame: x forward, y left, z up, from the center of the frame on the floor. */
  static final Frame ROBOT = at(Vector3.ZERO);

  private final Vector3 origin;
  private final Vector3 xAxis;
  private final Vector3 yAxis;
  private final Vector3 zAxis;

  private Frame(Vector3 origin, Vector3 xAxis, Vector3 yAxis, Vector3 zAxis) {
    this.origin = origin;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.zAxis = zAxis;
  }

  /** The robot's axes, moved to a point. */
  static Frame at(Vector3 origin) {
    return new Frame(origin, new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1));
  }

  /**
   * Axes turned as WPILib's {@code Rotation3d(roll, pitch, yaw)} turns them, at a point: by the
   * roll about x, then the pitch about y, then the yaw about z, each about the robot's own axes. A
   * positive pitch turns x down, as WPILib's does.
   */
  static Frame turned(Vector3 origin, double rollRadians, double pitchRadians, double yawRadians) {
    double cr = StrictMath.cos(rollRadians);
    double sr = StrictMath.sin(rollRadians);
    double cp = StrictMath.cos(pitchRadians);
    double sp = StrictMath.sin(pitchRadians);
    double cy = StrictMath.cos(yawRadians);
    double sy = StrictMath.sin(yawRadians);
    // The columns of Rz(yaw) Ry(pitch) Rx(roll).
    return new Frame(
        origin,
        new Vector3(cy * cp, sy * cp, -sp),
        new Vector3(cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
        new Vector3(cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr));
  }

  /** A point given in this frame, in the robot's frame. */
  Vector3 place(Vector3 local) {
    return origin.plus(turn(local));
  }

  /** A direction given in this frame, in the robot's frame. */
  Vector3 turn(Vector3 local) {
    return xAxis.times(local.x()).plus(yAxis.times(local.y())).plus(zAxis.times(local.z()));
  }
}

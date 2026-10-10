package com.michaelgrundvig.frc.simulation;

import com.michaelgrundvig.frc.world.field.Terrain;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Something done to the physics' world from outside it ({@link Physics#apply}): the robot carried
 * somewhere, pushed, tilted by hand, the floor gone slick, a tread worn, the ground itself. Each is
 * a declared override of the kinematic physics, saying so: a push a speed the wheels skid under, a
 * slick floor a share of their rolling.
 */
public sealed interface Disturbance {
  /**
   * Carried and set down here, as the drive team does: its wheels don't turn, but the turn counts
   * as one the robot truly made, so its gyro feels it. The physics keeps the robot's velocity
   * through it, so a shove mid-drive carries on.
   */
  record Placed(Pose2d pose) implements Disturbance {}

  /**
   * Pushed toward this velocity across the field, as another robot pushing it does, getting going
   * and stopping at {@link KinematicPhysics#PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED}: the
   * wheels skid and never see it. Zero stops the push.
   *
   * @param fieldVelocity in m/s, in the field's frame
   */
  record Pushed(Translation2d fieldVelocity) implements Disturbance {}

  /**
   * Tilted by hand, whatever the ground under it says, as a wheel up on a game piece does: a
   * declared override, not physics. {@link Ground} sets it back down.
   *
   * @param pitchRadians nose up is negative
   * @param rollRadians the left side up is positive
   */
  record TiltedByHand(double pitchRadians, double rollRadians) implements Disturbance {}

  /**
   * The ground the robot drives on, from now on, and the robot set back down on it, undoing a tilt
   * by hand.
   */
  record Ground(Terrain terrain) implements Disturbance {}

  /**
   * Only this share of the wheels' rolling moves the robot, as on a slick floor or with wheels in
   * the air: a declared override, every wheel alike.
   *
   * @param share from 0 to 1: 1 is full grip
   */
  record Grip(double share) implements Disturbance {
    public Grip {
      if (!(share >= 0 && share <= 1)) {
        throw new IllegalArgumentException("a share of the wheels' rolling: " + share);
      }
    }
  }

  /**
   * One wheel's tread worn from now on: its true radius over the configured one, 0.97 for tread
   * worn 3%. The robot moves that much less than its encoder counts.
   *
   * @param wheel front left, front right, back left, back right: 0 to 3
   */
  record TreadWorn(int wheel, double scale) implements Disturbance {
    public TreadWorn {
      if (!(scale > 0)) {
        throw new IllegalArgumentException("a wheel's radius scale is positive: " + scale);
      }
    }
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.record.Holder;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import java.util.List;

/**
 * What a joint's motors were told in one loop. The joint keeps it into the next loop, where its
 * gates, its idle and its log ask what the joint was doing.
 *
 * <p>Usually that is a {@link Demand}: what a command asked or what the joint does idle, as the
 * joint's interlocks and limit switches left it. The model's {@link JointAsk} has no word for the
 * loops in which no demand drives the joint at all, so those are named here ({@link Undriven}).
 */
sealed interface Applied permits Applied.Undriven, Demand {
  /**
   * The loops in which no demand drives the joint: the robot is disabled, or the joint does not
   * know where it is and so cannot be sent anywhere.
   */
  enum Undriven implements Applied {
    /**
     * The motors were stopped, or, for a joint that has lost the encoder its loop closes on, held
     * where the motor's own count says the joint is.
     */
    NOTHING,
    /** Homing pushed the joint toward its hard stop or its switch. */
    HOMING;

    @Override
    public List<Holder> heldBy() {
      return List.of();
    }
  }

  /**
   * What held the joint short of what was asked, in the order it was applied. Empty when nothing
   * held it, and in a loop in which no demand drove it.
   */
  List<Holder> heldBy();

  /** Whether the joint was driven toward a position along its profile. */
  default boolean wasToAPosition() {
    return this instanceof Demand demand && demand.ask() instanceof JointAsk.ToPosition;
  }

  /** Whether the joint's motors were put in neutral because that is what was asked or allowed. */
  default boolean wasNeutral() {
    return this instanceof Demand demand && demand.ask() instanceof JointAsk.Neutral;
  }
}

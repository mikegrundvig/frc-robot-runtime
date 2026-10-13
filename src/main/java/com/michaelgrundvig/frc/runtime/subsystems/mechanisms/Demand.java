package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.record.Holder;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import java.util.ArrayList;
import java.util.List;

/**
 * What a joint is to do in one loop, part of the way through the stages that lie beneath the
 * joint's commands.
 *
 * <p>A demand starts as what a command asked, or as what the joint does when no command asks
 * anything ({@link Idle}). The joint's interlocks ({@link Interlocks}) and then its limit switches
 * ({@link LimitStops}) each look at it in turn. A stage with nothing to change hands on the very
 * same object. A stage that holds the joint short of what was asked hands on a new demand, which
 * says what the joint may do instead and adds itself to what holds the joint.
 *
 * <p>Once every stage has seen it, a demand is what the joint's motor controller is told, so it is
 * also one of the things a joint can have {@link Applied} in a loop.
 *
 * @param ask what the joint is to do: go to a position, run at an output, or go neutral
 * @param restartProfile whether the joint's motion profile must start again from where the joint
 *     is, and not carry on from where it had got to, because what drives the joint changed under it
 * @param heldBy what holds the joint short of what was asked, in the order the stages applied it.
 *     Empty when nothing holds the joint. The list never changes once the demand is made.
 */
record Demand(JointAsk ask, boolean restartProfile, List<Holder> heldBy) implements Applied {
  /** Neutral, with nothing holding the joint: what a joint that only runs at outputs does idle. */
  static final Demand NEUTRAL = of(JointAsk.NEUTRAL);

  /** Something asked, with nothing yet holding the joint short of it. */
  static Demand of(JointAsk ask) {
    return new Demand(ask, false, List.of());
  }

  /** The same demand, with the joint's profile starting again from where the joint is. */
  Demand restartingProfile() {
    return new Demand(ask, true, heldBy);
  }

  /**
   * This demand as something that holds the joint leaves it.
   *
   * @param allowed what the joint may do instead of what was asked
   * @param by what holds it: an interlock, a tripped limit switch, or its own lost position
   */
  Demand heldTo(JointAsk allowed, Holder by) {
    return new Demand(allowed, restartProfile, joined(heldBy, by));
  }

  /** What already holds a joint with one more thing added after it, as a list of its own. */
  static List<Holder> joined(List<Holder> heldBy, Holder more) {
    List<Holder> joined = new ArrayList<>(heldBy);
    joined.add(more);
    return List.copyOf(joined);
  }
}

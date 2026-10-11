package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.robot.JointAsk;
import java.util.OptionalDouble;

/**
 * What a joint does in a loop in which no command asks anything of it.
 *
 * <p>A joint that holds positions holds one. A joint that only runs at outputs, such as a roller,
 * goes neutral, unless the last thing it was asked for was a position, which it then holds.
 *
 * <p>Where a joint holds is the latest of these:
 *
 * <ul>
 *   <li>where it was first read, where it was when the robot was enabled, or where its homing found
 *       it ({@link #holdAt});
 *   <li>where its last move to a position arrived, or, if that move did not arrive, where the joint
 *       came to a stop ({@link #moveEnded});
 *   <li>where it was when it stopped running at an output and was left idle ({@link #demand}).
 * </ul>
 *
 * <p>A joint whose position reads as something that is not a number has nowhere to hold. Left idle
 * like that it is neutral, saying why, until it reads a number again, and then it holds there. A
 * joint that already holds somewhere goes on holding there whatever it reads.
 *
 * <p>The idle is the joint's own, not a command, so a joint that a command owns but no longer
 * drives does not sag. A default command replaces it, as it would in Commands v3.
 */
final class Idle {
  /** Whether the joint is one that holds positions: it has a range or named positions. */
  private final boolean holdsPositions;

  /**
   * Whether the joint holds a position when it is next left idle. A joint that holds positions
   * always does. Any other joint does only while the last thing a command asked of it was a
   * position.
   */
  private boolean holdsWhenIdle;

  /**
   * Where the joint holds when idle, in the joint's units. Empty until the joint has been read, and
   * while what was read is not a number: there is then nowhere to hold.
   */
  private OptionalDouble holdsAt = OptionalDouble.empty();

  /**
   * The demand that holds the joint there. It is built when where the joint holds changes and
   * handed out in every idle loop after that, so that a joint left idle allocates nothing. It is
   * handed out only while the joint has somewhere to hold.
   */
  private Demand hold = Demand.NEUTRAL;

  /**
   * What the joint does idle while it has nowhere to hold: neutral, saying that its position is not
   * a number. It is built once, so that a joint whose position stays that way allocates nothing.
   */
  private final Demand nowhereToHold;

  /**
   * @param label the joint's label, which names it in what holds it
   * @param holdsPositions whether the joint is one that holds positions
   */
  Idle(String label, boolean holdsPositions) {
    this.holdsPositions = holdsPositions;
    holdsWhenIdle = holdsPositions;
    nowhereToHold =
        Demand.NEUTRAL.heldTo(JointAsk.NEUTRAL, possessive(label) + " position isn't a number");
  }

  /** Where the joint holds when idle, or empty while there is nowhere to hold yet. */
  OptionalDouble at() {
    return holdsAt;
  }

  /**
   * From now on the joint holds at a position: where it was read, or where its homing found it. A
   * position that is not a number is nowhere to hold, and nothing is built for it, so it costs
   * nothing to be told so every loop.
   */
  void holdAt(double position) {
    if (Double.isNaN(position)) {
      holdsAt = OptionalDouble.empty();
      return;
    }
    holdsAt = OptionalDouble.of(position);
    hold = holdingAt(position);
  }

  /**
   * A move to a position ended, and the joint holds from now on.
   *
   * @param position where the move arrived. If it did not arrive, where the joint comes to a stop,
   *     so that a joint that jammed holds where it jammed and does not push on toward a position it
   *     could not reach.
   */
  void moveEnded(double position) {
    holdsWhenIdle = true;
    holdAt(position);
  }

  /** A command asked something of the joint this loop, which decides what it does idle next. */
  void commandAsked(JointAsk asked) {
    holdsWhenIdle = holdsPositions || asked instanceof JointAsk.ToPosition;
  }

  /**
   * What the joint does this loop, no command having asked anything of it.
   *
   * @param lastLoop what the joint's motors were told in the loop before
   * @param here where the joint is now, inside its limits, or not a number if that is what the
   *     joint reads
   */
  Demand demand(Applied lastLoop, double here) {
    if (!holdsWhenIdle) {
      return Demand.NEUTRAL;
    }
    if (lastLoop.wasToAPosition() && holdsAt.isPresent()) {
      return hold;
    }
    // The joint was running at an output, or was not driven at all, or had nowhere to hold. It
    // holds where it is now, and its profile starts from there.
    holdAt(here);
    if (holdsAt.isEmpty()) {
      return nowhereToHold;
    }
    return hold.restartingProfile();
  }

  private static Demand holdingAt(double position) {
    return Demand.of(new JointAsk.ToPosition(position, JointAsk.FULL_CRUISE));
  }
}

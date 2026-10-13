package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;

/**
 * Whose joint it is and what they have asked of it, as far as the joint's gates need to know: which
 * question a gate on "has the joint done what it was asked" is answering, and whether the joint has
 * been judged against it yet.
 *
 * <p><b>A question</b> is one state of what the joint is asked. It changes when:
 *
 * <ul>
 *   <li>a command takes the joint ({@link #takenBy}), which has not asked anything of it yet;
 *   <li>the command that has the joint ends ({@link #endedFor}), and the joint is nobody's;
 *   <li>the joint is asked something new ({@link #asked}), and not what it was asked the loop
 *       before going on.
 * </ul>
 *
 * <p><b>Judged</b> means the joint's own step at the end of a loop ({@link JointMechanism#write})
 * has taken the question's ask as what it tells its motors, so there is something of this question
 * to compare the joint's readings with. Until then, what the joint last told its motors belongs to
 * an earlier question, and a gate that compared the readings with it would be answering that one:
 * an idle, stopped shooter is "at its goal", and a command that took it a moment ago to spin it up
 * must not be told so.
 *
 * <p>So a change of question is unjudged from the moment it happens until the end of that loop's
 * commands. One case lasts longer: a joint a command has taken and has not yet asked anything of,
 * however many loops that takes. Once its holder has asked, the joint's idle is an answer to its
 * holder too: a pivot sent out holds there, at its goal, after the move that sent it is over.
 */
final class Tenure {
  /** What {@link #judged} holds before the joint has been judged for any question. */
  private static final long NEVER = -1;

  /**
   * The command that has the joint, or null while none has. It is null, and not an empty Optional,
   * because it is read and compared on every event the scheduler emits.
   */
  private @Nullable Command holder;

  /** Whether the command that has the joint has asked anything of it since taking it. */
  private boolean holderHasAsked;

  /** How many times what the joint is asked has changed. */
  private long question;

  /** The question the joint's latest request belongs to. */
  private long askedIn;

  /** The latest question the joint has been judged against. */
  private long judged = NEVER;

  /**
   * A command that requires the joint has its first turn. It has the joint from now, and has asked
   * nothing of it yet.
   *
   * @param command the command that has it, all the way out among those that require it: a
   *     parent's, when a parent that requires the joint forks the command that will drive it
   */
  void takenBy(Command command) {
    if (command == holder) {
      return;
    }
    holder = command;
    holderHasAsked = false;
    question++;
  }

  /** A command has ended. If it was the one that had the joint, the joint is nobody's. */
  void endedFor(Command command) {
    if (command != holder) {
      return;
    }
    holder = null;
    holderHasAsked = false;
    question++;
  }

  /** The command that has the joint, or null while none has. */
  @Nullable Command holder() {
    return holder;
  }

  /**
   * The joint is asked something this loop.
   *
   * @param goesOn whether it is what the joint was asked the loop before, going on, and so no new
   *     question ({@link JointRequests#ask})
   */
  void asked(boolean goesOn) {
    if (!goesOn) {
      question++;
    }
    holderHasAsked = holder != null;
    askedIn = question;
  }

  /**
   * The joint's step at the end of a loop has decided what its motors are told.
   *
   * @param aRequestWasTaken whether that was what it was asked this loop, and not what it does idle
   */
  void wrote(boolean aRequestWasTaken) {
    if (aRequestWasTaken) {
      // The request may be of a question that has since changed again, by its command ending in
      // the same loop. The joint is then judged for the request's question, and not for this one.
      judged = askedIn;
      return;
    }
    boolean idleIsItsAnswer = holder == null || holderHasAsked;
    if (idleIsItsAnswer) {
      judged = question;
    }
  }

  /** Which question the joint's gates are answering now: a count that only goes up. */
  long question() {
    return question;
  }

  /** Whether the joint has been judged against the question it is under now. */
  boolean isJudged() {
    return judged == question;
  }

  /**
   * A count that moves on whenever a gate's answer may no longer stand: with each new question, and
   * again when the joint is judged against it. It only goes up, by one for each: two for each
   * question, and one more once that question is judged. A gate worked out before either is worked
   * out again when it is next read ({@link com.michaelgrundvig.frc.runtime.behavior.Gate}).
   */
  long standing() {
    return 2 * question + (isJudged() ? 1 : 0);
  }
}

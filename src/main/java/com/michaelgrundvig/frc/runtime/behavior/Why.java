package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.record.Identity;
import com.michaelgrundvig.frc.world.record.MoveReason;
import com.michaelgrundvig.frc.world.record.MoveSentences;
import java.util.Optional;

/**
 * Why a move ended as it did, where how it ended does not say: one of a closed set of reasons, the
 * joint the reason is about if it is about one, and the text that only the robot's code can give.
 * The reason and the joint are logged as facts, and the text beside them.
 *
 * @param reason which of the reasons it is
 * @param joint the joint the reason is about, as its index in the definition's joints, or {@link
 *     Identity#NONE} for a reason that is about none
 * @param text what the reason says that the facts cannot: a gate's name and its false terms, a
 *     refusal's own words, a fault. Empty for a reason the facts say whole.
 */
public record Why(MoveReason reason, int joint, String text) {
  /** The robot was disabled, which cancels every command. */
  public static final Optional<Why> ROBOT_DISABLED =
      Optional.of(new Why(MoveReason.ROBOT_DISABLED, Identity.NONE, ""));

  /** A reason about no joint, said in text. */
  public static Why saying(MoveReason reason, String text) {
    return new Why(reason, Identity.NONE, text);
  }

  /**
   * What a reason reads as, where a message needs it apart from a move's sentence: the reason's own
   * words for the ones the facts say whole, and its text for the rest.
   */
  public String said() {
    return switch (reason) {
      case ROBOT_DISABLED -> MoveSentences.ROBOT_WAS_DISABLED;
      case NO_DRIVER -> MoveSentences.NO_DRIVER;
      default -> text;
    };
  }
}

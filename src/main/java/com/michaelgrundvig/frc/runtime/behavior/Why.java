package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.record.Identity;
import com.michaelgrundvig.frc.world.record.MoveReason;
import java.util.Optional;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.RobotMode;

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

  /**
   * The robot's mode the command was started in ended: WPILib's scheduler cancels a command that
   * was started in a mode, outside any other command, when that mode ends.
   */
  public static final Optional<Why> ROBOT_MODE_ENDED =
      Optional.of(new Why(MoveReason.ROBOT_MODE_ENDED, Identity.NONE, ""));

  /**
   * Why a move is canceled now: what its mechanism says (the robot was disabled this loop), or else
   * that the robot's mode the move was started in has ended. Any other cancel has no reason the
   * move knows of.
   *
   * @param mechanismSays why its mechanism says it is canceled, if it can say
   * @param startedIn the robot's mode as the move's run began
   */
  public static Optional<Why> canceledNow(Optional<Why> mechanismSays, RobotMode startedIn) {
    if (mechanismSays.isPresent()) {
      return mechanismSays;
    }
    boolean modeEnded = startedIn != RobotMode.UNKNOWN && RobotState.getRobotMode() != startedIn;
    if (modeEnded) {
      return ROBOT_MODE_ENDED;
    }
    return Optional.empty();
  }

  /** A reason about no joint, said in text. */
  public static Why saying(MoveReason reason, String text) {
    return new Why(reason, Identity.NONE, text);
  }
}

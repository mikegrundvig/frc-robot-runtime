package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.record.MoveSentences;
import org.wpilib.command3.Scheduler;

/**
 * What disabling the robot does to its commands: in the loop the robot goes from enabled to
 * disabled, every running command is canceled. A disabled robot then has nothing left over to carry
 * on with when it is enabled again: no move part way to an old target, and no command a button
 * started a minute ago.
 *
 * <p>Since 2027 alpha 8, WPILib's v3 scheduler also cancels, at a disable, every command that needs
 * a mechanism not controllable while disabled, and refuses to start one while the robot is
 * disabled. Every joint and the drivetrain are such mechanisms. The robot's loop still cancels
 * every command here first, in the loop of the disable, so each move says why it ended ({@link
 * Why#ROBOT_DISABLED}) and a command that needs no mechanism is canceled too ({@link
 * #cancelCommandsIfDisabled}).
 *
 * <p>Only the change from enabled to disabled cancels. While the robot is disabled the scheduler
 * starts no command on a joint or the drivetrain, a default command among them: the drive's starts
 * at the enable. A command that needs no mechanism still runs while disabled: a pit action that
 * runs only while disabled, the heading reset.
 *
 * <p>What the scheduler runs that is not a command is not canceled: its periodic tasks, which work
 * out each gate and clear the binding record, go on in every mode.
 */
public final class Disabling {
  /**
   * What a move that is canceled this way says, after how it ended: "pivot → out: CANCELED after
   * 0.20 s, pivot at 38.49°: the robot was disabled". A move records the reason as a fact ({@link
   * Why#ROBOT_DISABLED}), and these are the words its sentence says it in.
   */
  public static final String WHY = MoveSentences.ROBOT_WAS_DISABLED;

  private final Scheduler scheduler;

  /** Whether the robot was enabled in the last loop. A robot starts disabled. */
  private boolean wasEnabled;

  /**
   * @param scheduler the robot's command scheduler, whose commands are canceled
   */
  public Disabling(Scheduler scheduler) {
    this.scheduler = scheduler;
  }

  /**
   * Cancels every running command, and every one waiting to start, if the robot was disabled since
   * the last loop. Call it once a loop, after the mechanisms have been told whether the robot is
   * enabled, so that a move they cancel can say why, and before the scheduler runs.
   *
   * <p>The scheduler tells each command it is canceled ({@code onCancel}) before this returns, so
   * each move's outcome is written in this loop. What a command does as it is canceled is its
   * team's code, and may throw: call this where a command's throw is handled.
   *
   * @param enabled whether the robot is enabled this loop
   */
  public void cancelCommandsIfDisabled(boolean enabled) {
    boolean disabledSinceTheLastLoop = wasEnabled && !enabled;
    if (disabledSinceTheLastLoop) {
      scheduler.cancelAll();
    }
    // Noted only once the commands are canceled. If one throws as it is canceled, the scheduler
    // stops there, and the rest are canceled in the next loop instead of being left to run.
    wasEnabled = enabled;
  }
}

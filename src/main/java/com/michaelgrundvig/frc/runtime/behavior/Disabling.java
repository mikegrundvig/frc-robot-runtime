package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.record.MoveSentences;
import org.wpilib.command3.Scheduler;

/**
 * What disabling the robot does to its commands: in the loop the robot goes from enabled to
 * disabled, every running command is canceled. A disabled robot then has nothing left over to carry
 * on with when it is enabled again: no move part way to an old target, and no command a button
 * started a minute ago.
 *
 * <p>WPILib's older command framework did this by default, and its v3 scheduler does not: it runs
 * every command in every mode. So the robot's loop does it here, once, for every command, and a
 * team's commands need nothing ({@link #cancelCommandsIfDisabled}).
 *
 * <p>Only the change from enabled to disabled cancels. What starts while the robot is disabled is
 * left to run: a default command, which the scheduler starts again at once, a pit action that runs
 * only while disabled, the heading reset, or a command that code schedules itself. It moves
 * nothing, because no joint and no wheel is driven while the robot is disabled. Commands v3 has no
 * way for a command to say that it runs while disabled, and none is needed here: nothing has to
 * survive the cancel.
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

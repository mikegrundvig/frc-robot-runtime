package com.michaelgrundvig.frc.runtime.behavior;

import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Scheduler;

/**
 * What a team's code builds that can't work, refused by one rule wherever it's built: a move to
 * another joint's place, a share of the cruise past all of it, a gate on a place the joint hasn't.
 * Built as the robot starts, it throws, so the mistake stops the robot starting, and fails the test
 * that builds every robot, before anyone presses a button. Built once the robot runs, it's made all
 * the same, as a stand-in that says why: a move that ends {@link Outcome#REFUSED} as it starts, a
 * gate that holds false. A robot on the field is never stopped by one.
 */
public final class Refusals {
  private Refusals() {}

  /**
   * Whether the robot is starting: its command scheduler hasn't run yet, and no command's body is
   * running. Commands v3 says whether a command's body is running ({@code
   * Scheduler.currentCommand}), but outside one isn't only at startup: a team's {@code
   * autonomousInit}, a periodic method or an auto rebuilt as its chooser changes all run on the
   * field outside any command, where a throw would stop the robot. So startup is before the
   * scheduler's first run, which it says ({@code Scheduler.lastRuntimeMs}, -1 until then).
   */
  public static boolean starting(Scheduler scheduler) {
    return scheduler.lastRuntimeMs() < 0 && scheduler.currentCommand() == null;
  }

  /**
   * Why something can't work, as it's built: thrown as the robot starts ({@link #starting}), kept
   * once it runs for what was built to refuse with.
   *
   * @param why what can't work, and what to do; null for nothing
   * @return the reason, or null for none
   * @throws IllegalArgumentException with the reason, as the robot starts
   */
  public static @Nullable String asBuilt(Scheduler scheduler, @Nullable String why) {
    if (why != null && starting(scheduler)) {
      throw new IllegalArgumentException(why);
    }
    return why;
  }

  /**
   * A gate that can't be: thrown as the robot starts; built once it runs, a gate that holds false,
   * its term the reason, so a move waiting on it times out saying why ("pivot at clear is false:
   * pivot has no position clear at 0.4 m: it has in, run, out").
   *
   * @throws IllegalArgumentException with the reason, as the robot starts
   */
  public static Gate gate(Scheduler scheduler, String name, String why) {
    asBuilt(scheduler, why);
    return new Gate(
        scheduler,
        name,
        terms -> {
          terms.add(why);
          return false;
        });
  }
}

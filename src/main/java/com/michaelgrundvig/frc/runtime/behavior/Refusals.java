package com.michaelgrundvig.frc.runtime.behavior;

import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Scheduler;

/**
 * The one rule for what a team's code builds that cannot work: a move to another joint's place, a
 * share of the cruise speed above all of it, a gate on a place the joint does not have. Wherever
 * such a thing is built, it is refused the same way, and how depends on when.
 *
 * <ul>
 *   <li><b>As the robot starts</b>, building it throws. The mistake stops the robot from starting,
 *       and fails the test that builds every robot, before anyone presses a button.
 *   <li><b>Once the robot runs</b>, it is built all the same, as a stand-in that says why: a move
 *       that ends {@link Outcome#REFUSED} as it starts, or a gate that holds false. A robot on the
 *       field is never stopped by one.
 * </ul>
 */
public final class Refusals {
  private Refusals() {}

  /**
   * Whether the robot is starting: its command scheduler has not run yet, and no command's body is
   * running.
   *
   * <p>Commands v3 can say whether a command's body is running ({@code Scheduler.currentCommand}),
   * but being outside one does not mean the robot is starting. A team's {@code autonomousInit}, a
   * periodic method, or an auto rebuilt as its chooser changes all run on the field outside any
   * command, where a throw would stop the robot. So the test for starting is that the scheduler has
   * never run, which it reports ({@code Scheduler.lastRuntimeMs} is -1 until its first run).
   */
  public static boolean starting(Scheduler scheduler) {
    boolean hasNeverRun = scheduler.lastRuntimeMs() < 0;
    return hasNeverRun && scheduler.currentCommand() == null;
  }

  /**
   * Refuses something that cannot work, as it is built. As the robot starts ({@link #starting}) the
   * reason is thrown. Once the robot runs the reason is handed back, for the caller to build a
   * stand-in that refuses with it.
   *
   * @param why what cannot work, and what to do about it; empty when it can work
   * @return the same reason
   * @throws IllegalArgumentException with the reason, as the robot starts
   */
  public static Optional<String> whenBuilt(Scheduler scheduler, Optional<String> why) {
    if (why.isPresent() && starting(scheduler)) {
      throw new IllegalArgumentException(why.get());
    }
    return why;
  }

  /**
   * The same rule as {@link #whenBuilt}, for a caller that holds its reason as a string that may be
   * null.
   *
   * @param why what cannot work, and what to do about it; null when it can work
   * @return the same reason, or null for none
   * @throws IllegalArgumentException with the reason, as the robot starts
   */
  public static @Nullable String asBuilt(Scheduler scheduler, @Nullable String why) {
    return whenBuilt(scheduler, Optional.ofNullable(why)).orElse(null);
  }

  /**
   * A gate that cannot be. As the robot starts it is a throw. Once the robot runs it is a gate that
   * holds false with the reason as its one term, so a move that waits on it times out saying why
   * ("pivot at clear is false: pivot has no position clear at 0.4 m: it has in, run, out").
   *
   * @throws IllegalArgumentException with the reason, as the robot starts
   */
  public static Gate gate(Scheduler scheduler, String name, String why) {
    whenBuilt(scheduler, Optional.of(why));
    return new Gate(
        scheduler,
        name,
        terms -> {
          terms.add(why);
          return false;
        });
  }
}

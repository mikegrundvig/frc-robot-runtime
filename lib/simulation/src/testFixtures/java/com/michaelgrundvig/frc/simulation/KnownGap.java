package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.fail;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.Locale;
import org.junit.jupiter.api.Tag;

/**
 * Marks a truth-suite test that today's simulator fails: physics or geometry says one number, the
 * simulator gives another, and the reason is known (its value). The test asserts today's number, so
 * the build stays green while the gap is known, and fails loudly the day the gap closes or moves:
 * closed, it's promoted to a plain truth test; moved, someone finds out why. {@code
 * docs/simulator.md} lists them.
 */
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
@Tag("known-gap")
public @interface KnownGap {
  /** Why today's simulator fails it. */
  String value();

  /** The checks a known gap makes. */
  public final class Check {
    private Check() {}

    /**
     * A number that physics puts at {@code truth}, within {@code passLine} of it as a share, which
     * today's simulator puts at {@code today} instead. Fails if it passes its line now (the gap
     * closed: promote the test), or if it moved more than {@code todayTolerance} from today's
     * number (find out why, and record the new number if it's meant).
     */
    public static void stillOpen(
        String what,
        double measured,
        double truth,
        double passLine,
        double today,
        double todayTolerance) {
      if (Math.abs(measured - truth) <= passLine * Math.abs(truth)) {
        fail(
            format(
                "Known gap closed: %s is %.6g, within %.3g%% of physics' %.6g. Promote the test to a"
                    + " truth test, and take it off the known gaps in docs/simulator.md.",
                what, measured, 100 * passLine, truth));
      }
      if (!(Math.abs(measured - today) <= todayTolerance)) {
        fail(
            format(
                "Known gap moved: %s is %.6g, where it was %.6g (physics: %.6g). Find out what moved"
                    + " it; if that's meant, record the new number.",
                what, measured, today, truth));
      }
    }

    /**
     * Something physics says ({@code physicsHolds} is whether the simulator does it now) that
     * today's simulator doesn't, and what it does instead ({@code todayHolds}).
     */
    public static void stillOpen(
        String what, boolean physicsHolds, boolean todayHolds, String today) {
      if (physicsHolds) {
        fail(
            "Known gap closed: "
                + what
                + " now holds. Promote the test to a truth test, and take it off the known gaps"
                + " in docs/simulator.md.");
      }
      if (!todayHolds) {
        fail("Known gap moved: " + what + " still fails, but no longer as recorded: " + today);
      }
    }

    private static String format(String format, Object... values) {
      return String.format(Locale.ROOT, format, values);
    }
  }
}

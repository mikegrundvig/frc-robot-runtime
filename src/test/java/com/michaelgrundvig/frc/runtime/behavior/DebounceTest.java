package com.michaelgrundvig.frc.runtime.behavior;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.filter.Debouncer.DebounceType;

/**
 * The debounce on its own, on a clock the test steps: one script of a condition, loop by loop, run
 * with the clock starting at many different readings.
 */
class DebounceTest {
  private static final long LOOP_NANOS = 20_000_000;

  /**
   * How far the robot's clock is stepped before each run of a script that uses the robot's own
   * clock, in seconds: forty odd amounts from 14 ms to 21 s, and one of a quarter of an hour.
   */
  static final double[] CLOCK_STEPS = clockSteps();

  /** The condition on each loop of the script: held for seven loops, a one-loop dip, then gone. */
  static final String SCRIPT = "001111111010000000";

  /** What each kind of debounce of 0.1 s, five loops, answers on each loop of the script. */
  static final Map<DebounceType, String> ANSWERS =
      Map.of(
          DebounceType.RISING, "000000111000000000",
          DebounceType.FALLING, "111111111111111000",
          DebounceType.BOTH, "000000111111111000");

  private static double[] clockSteps() {
    double[] steps = new double[41];
    for (int i = 0; i < 40; i++) {
      steps[i] = 0.0137 * (i * i + 1);
    }
    steps[40] = 900.000001;
    return steps;
  }

  /** The clock readings a script starts at: none, odd ones, a match, a day, and thirty years. */
  private static final long[] STARTS = {
    0,
    1,
    19_999_999,
    13_700_001,
    150_000_000_000L,
    12_345_678_901_234L,
    86_400_000_000_000L,
    86_400_000_000_007L,
    946_080_000_000_000_003L
  };

  /**
   * A debounce time that is a whole number of loops ends on the same loop whatever the clock read
   * as it started. WPILib's debouncer, which subtracts seconds held in doubles, ended a loop early
   * or late depending on that reading.
   */
  @Test
  void aTimeOfWholeLoopsEndsOnTheSameLoopWhateverTheClockReads() {
    Map<String, String> answered = new LinkedHashMap<>();
    Map<String, String> expected = new LinkedHashMap<>();

    for (DebounceType type : DebounceType.values()) {
      for (long start : STARTS) {
        long[] now = {start};
        Debounce debounce = new Debounce(0.1, type, () -> now[0]);
        StringBuilder answers = new StringBuilder();
        for (char condition : SCRIPT.toCharArray()) {
          now[0] += LOOP_NANOS;
          answers.append(debounce.calculate(condition == '1') ? '1' : '0');
        }
        answered.put(type + " from " + start, answers.toString());
        expected.put(type + " from " + start, ANSWERS.get(type));
      }
    }

    assertThat(answered).containsExactlyEntriesOf(expected);
  }

  /**
   * Times that are not a whole number of loops, or no time at all: a time ends on the first loop at
   * or past it, no time changes nothing, and a time that is not a number never ends.
   */
  @Test
  void aTimeEndsOnTheFirstLoopAtOrPastIt() {
    Map<Double, String> rising = new LinkedHashMap<>();

    for (double seconds : new double[] {0, 0.02, 0.05, 0.1, 0.100000001, Double.NaN}) {
      long[] now = {7};
      Debounce debounce = new Debounce(seconds, DebounceType.RISING, () -> now[0]);
      StringBuilder said = new StringBuilder();
      for (char condition : SCRIPT.toCharArray()) {
        now[0] += LOOP_NANOS;
        said.append(debounce.calculate(condition == '1') ? '1' : '0');
      }
      rising.put(seconds, said.toString());
    }

    Map<Double, String> expected = new LinkedHashMap<>();
    expected.put(0.0, "001111111010000000");
    expected.put(0.02, "001111111010000000");
    expected.put(0.05, "000011111000000000");
    expected.put(0.1, "000000111000000000");
    expected.put(0.100000001, "000000011000000000");
    expected.put(Double.NaN, "000000000000000000");
    assertThat(rising).containsExactlyEntriesOf(expected);
  }
}

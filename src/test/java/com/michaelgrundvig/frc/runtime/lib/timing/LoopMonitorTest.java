package com.michaelgrundvig.frc.runtime.lib.timing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;

/** Loop timing, with the clock given by hand. */
class LoopMonitorTest {
  private static final long MS = 1_000_000L;

  private final LoopMonitor monitor = new LoopMonitor(0.020, 3);

  /** One loop: starts at a time, its sections take these milliseconds, in order. */
  private LoopMonitor.Report loop(long startMs, long... sectionMs) {
    long now = startMs * MS;
    monitor.start(now);
    for (int i = 0; i < sectionMs.length; i++) {
      now += sectionMs[i] * MS;
      monitor.mark("S" + i, now);
    }
    return monitor.finish(now + MS); // a millisecond of bookkeeping after the last section
  }

  @Test
  void eachSectionIsTimedFromTheOneBefore() {
    LoopMonitor.Report report = loop(0, 3, 5, 2);

    assertThat(report.sections())
        .extracting(LoopMonitor.Section::name)
        .containsExactly("S0", "S1", "S2");
    assertThat(report.sections().get(1).seconds()).isCloseTo(0.005, within(1e-12));
    assertThat(report.seconds()).isCloseTo(0.011, within(1e-12));
    assertThat(report.slowest()).isEqualTo(new LoopMonitor.Section("S1", 0.005));
    assertThat(report.overran()).isFalse();
    assertThat(report.describe()).isEqualTo("Worst 11 ms lately, none overran");
  }

  @Test
  void overrunsAreCountedWhileTheyAreRecent() {
    // Each loop starts on its time, or at once after one that overran, as the robot's timer runs.
    loop(0, 25);
    LoopMonitor.Report second = loop(26, 4);
    loop(46, 30);
    LoopMonitor.Report fourth = loop(77, 4); // the first has left the three recent loops
    loop(97, 4);
    LoopMonitor.Report sixth = loop(117, 4);

    assertThat(second.overrunsLately()).isEqualTo(1);
    assertThat(second.worstLatelySeconds()).isCloseTo(0.026, within(1e-12));
    assertThat(fourth.overrunsLately()).isEqualTo(1);
    assertThat(fourth.describe()).isEqualTo("Overran 1 time lately, worst 31 ms");
    assertThat(fourth.lateLately()).as("an overrun isn't also a late start").isZero();
    assertThat(sixth.overrunsLately()).isZero();
  }

  /**
   * Logging, dashboards, and garbage collection run between loops, where start-to-finish timing
   * can't see them: a loop that starts late shows the time they took.
   */
  @Test
  void aLoopThatStartsLateShowsTimeLostBetweenLoops() {
    loop(0, 4); // takes 5 ms
    LoopMonitor.Report late = loop(30, 4); // 10 ms after its time
    LoopMonitor.Report onTime = loop(50, 4);

    assertThat(late.overran()).as("its own code was quick").isFalse();
    assertThat(late.lateLately()).isEqualTo(1);
    assertThat(late.outsideSeconds())
        .as("30 ms since, 5 in the loop")
        .isCloseTo(0.025, within(1e-12));
    assertThat(late.describe())
        .isEqualTo(
            "Worst 5 ms lately, none overran; 1 started late (time lost between loops: logging,"
                + " garbage collection)");
    assertThat(onTime.outsideSeconds()).isZero();
    assertThat(onTime.lateLately()).as("still recent").isEqualTo(1);
  }

  @Test
  void aStartWithinTheTimersJitterIsOnTime() {
    loop(0, 4);
    LoopMonitor.Report report = loop(24, 4); // 4 ms late: under a quarter period

    assertThat(report.lateLately()).isZero();
  }

  @Test
  void aLoopMarkedWithoutAStartIsTimedFromTheLastLoop() {
    loop(0, 2);
    monitor.mark("Part", 10 * MS);

    LoopMonitor.Report report = monitor.finish(11 * MS);

    assertThat(report.sections()).containsExactly(new LoopMonitor.Section("Part", 0.007));
    assertThat(report.seconds()).isCloseTo(0.008, within(1e-12));
  }

  @Test
  void aFirstMarkWithNothingBeforeItTakesNoTime() {
    monitor.mark("Part", 5 * MS);

    LoopMonitor.Report report = monitor.finish(6 * MS);

    assertThat(report.sections()).containsExactly(new LoopMonitor.Section("Part", 0));
    assertThat(report.seconds()).isCloseTo(0.001, within(1e-12));
  }

  @Test
  void aLoopWithNoSectionsHasNoSlowest() {
    assertThat(monitor.finish(0).slowest()).isNull();
  }

  @Test
  void settingsMustMakeSense() {
    assertThatThrownBy(() -> new LoopMonitor(0, 10)).isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> new LoopMonitor(0.02, 0)).isInstanceOf(IllegalArgumentException.class);
  }
}

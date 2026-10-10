package com.michaelgrundvig.frc.runtime.lib.timing;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;

/**
 * Times each robot loop, section by section, and keeps count of the recent loops that overran their
 * period, or started late.
 *
 * <p>Each loop: {@link #start} when it begins, {@link #mark} as each section ends (a section is
 * timed from the previous mark, or from the start), and {@link #finish} when it's done, which
 * reports the loop. A mark without a start times from the previous loop's last mark, so a loop run
 * without its start (a test calling one part of it) still reports something sensible.
 *
 * <p>Timing from start to finish sees only the robot's own code. Work the robot program does
 * between loops (writing the log, publishing to dashboards, updating tunables) and pauses for
 * garbage collection show up only as the next loop starting late: a loop's start more than a
 * quarter period after its time. Both count.
 */
public final class LoopMonitor {
  /**
   * A loop that starts this much of a period later than its time (after the previous loop, or its
   * period, whichever was longer) started late: more than the timer's jitter.
   */
  private static final double LATE_FRACTION = 0.25;

  private final long periodNanos;
  private final long lateMarginNanos;
  private final long[] recentLoops;
  private final boolean[] recentLate;
  private final List<String> names = new ArrayList<>();
  private final List<Long> durations = new ArrayList<>();
  private int next;
  private int filled;
  private long loopStart = Long.MIN_VALUE;
  private long lastMark = Long.MIN_VALUE;
  private long previousStart = Long.MIN_VALUE;
  private long previousTook;
  private long outside;
  private boolean late;

  /** One section of a loop, and how long it took. */
  public record Section(String name, double seconds) {}

  /**
   * One loop, and the loops before it.
   *
   * @param seconds how long this loop took, start to finish
   * @param overran whether it took longer than the period
   * @param sections each section, in the order they ran
   * @param overrunsLately how many of the recent loops overran, this one included
   * @param worstLatelySeconds the longest of the recent loops
   * @param outsideSeconds how long the robot program spent between the previous loop's finish and
   *     this one's start, beyond waiting for its time: 0 when it started on time
   * @param lateLately how many of the recent loops started late, this one included
   */
  public record Report(
      double seconds,
      boolean overran,
      List<Section> sections,
      int overrunsLately,
      double worstLatelySeconds,
      double outsideSeconds,
      int lateLately) {
    /** Nothing timed yet. */
    public static final Report NONE = new Report(0, false, List.of(), 0, 0, 0, 0);

    /** The section that took longest this loop; null if the loop had none. */
    public @Nullable Section slowest() {
      Section slowest = null;
      for (Section section : sections) {
        if (slowest == null || section.seconds() > slowest.seconds()) {
          slowest = section;
        }
      }
      return slowest;
    }

    /**
     * One line for the pit: the slowest loop lately, to the millisecond, and how many overran. It
     * changes only when the loops do, not with every loop's own time, so a page showing it isn't
     * sent a new line every loop.
     */
    public String describe() {
      long worst = Math.round(worstLatelySeconds * 1000);
      String took =
          overrunsLately == 0
              ? String.format(Locale.ROOT, "Worst %d ms lately, none overran", worst)
              : String.format(
                  Locale.ROOT,
                  "Overran %d %s lately, worst %d ms",
                  overrunsLately,
                  overrunsLately == 1 ? "time" : "times",
                  worst);
      return lateLately == 0
          ? took
          : String.format(
              Locale.ROOT,
              "%s; %d started late (time lost between loops: logging, garbage collection)",
              took,
              lateLately);
    }
  }

  /**
   * @param periodSeconds the loop's period: a loop that takes longer overran
   * @param recentLoops how many loops "lately" covers: 500 is ten seconds at 50 Hz
   */
  public LoopMonitor(double periodSeconds, int recentLoops) {
    if (!(periodSeconds > 0) || Double.isInfinite(periodSeconds)) {
      throw new IllegalArgumentException("a loop's period: " + periodSeconds);
    }
    if (recentLoops < 1) {
      throw new IllegalArgumentException("at least one recent loop: " + recentLoops);
    }
    periodNanos = Math.round(periodSeconds * 1e9);
    lateMarginNanos = Math.round(periodSeconds * LATE_FRACTION * 1e9);
    this.recentLoops = new long[recentLoops];
    recentLate = new boolean[recentLoops];
  }

  /**
   * A loop begins. From the time since the previous one began: whether this one started late, and
   * how long the program spent outside the loops in between.
   */
  public void start(long nowNanos) {
    if (previousStart != Long.MIN_VALUE) {
      long cycle = nowNanos - previousStart;
      late = cycle - Math.max(previousTook, periodNanos) > lateMarginNanos;
      outside = late ? Math.max(0, cycle - previousTook) : 0;
    }
    previousStart = nowNanos;
    loopStart = nowNanos;
    lastMark = nowNanos;
    names.clear();
    durations.clear();
  }

  /** A section of the loop just ended. */
  public void mark(String section, long nowNanos) {
    if (lastMark == Long.MIN_VALUE) {
      lastMark = nowNanos;
      loopStart = nowNanos;
    }
    if (loopStart == Long.MIN_VALUE) {
      loopStart = lastMark;
    }
    names.add(section);
    durations.add(Math.max(0, nowNanos - lastMark));
    lastMark = nowNanos;
  }

  /** The loop is done: how it went, and how the recent loops went. */
  public Report finish(long nowNanos) {
    long start = loopStart == Long.MIN_VALUE ? nowNanos : loopStart;
    long took = Math.max(0, nowNanos - start);
    recentLoops[next] = took;
    recentLate[next] = late;
    next = (next + 1) % recentLoops.length;
    filled = Math.min(filled + 1, recentLoops.length);
    int overruns = 0;
    int lateLately = 0;
    long worst = 0;
    for (int i = 0; i < filled; i++) {
      overruns += recentLoops[i] > periodNanos ? 1 : 0;
      lateLately += recentLate[i] ? 1 : 0;
      worst = Math.max(worst, recentLoops[i]);
    }
    List<Section> sections = new ArrayList<>(names.size());
    for (int i = 0; i < names.size(); i++) {
      sections.add(new Section(names.get(i), durations.get(i) / 1e9));
    }
    names.clear();
    durations.clear();
    loopStart = Long.MIN_VALUE;
    lastMark = nowNanos;
    previousTook = took;
    Report report =
        new Report(
            took / 1e9,
            took > periodNanos,
            List.copyOf(sections),
            overruns,
            worst / 1e9,
            outside / 1e9,
            lateLately);
    late = false;
    outside = 0;
    return report;
  }
}

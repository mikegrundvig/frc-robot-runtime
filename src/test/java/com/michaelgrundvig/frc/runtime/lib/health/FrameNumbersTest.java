package com.michaelgrundvig.frc.runtime.lib.health;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.lib.health.FrameNumbers.Step;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/** A camera's frame numbers as they arrive over a network that reorders and repeats datagrams. */
class FrameNumbersTest {
  /** Frames in the order they arrive, each its number and when it was taken. */
  private record Arrivals(String name, long[][] frames, List<Step> steps, long highest) {}

  private static final long WRAP = 0xffff_ffffL;
  private static final long MS = 1_000_000L;

  /** A frame numbered so, taken at that many milliseconds. */
  private static long[] f(long number, long takenMs) {
    return new long[] {number, takenMs * MS};
  }

  @Test
  void eachNumberStandsAsItArrived() {
    int window = FrameNumbers.REORDER_FRAMES;
    List<Arrivals> rows =
        List.of(
            new Arrivals(
                "in order",
                new long[][] {f(0, 0), f(1, 16), f(2, 33)},
                List.of(Step.NEWEST, Step.NEWEST, Step.NEWEST),
                2),
            new Arrivals(
                "a gap is new frames, the missing ones lost",
                new long[][] {f(5, 0), f(9, 66)},
                List.of(Step.NEWEST, Step.NEWEST),
                9),
            new Arrivals(
                "a pair out of order: late, not a restart",
                new long[][] {f(10, 0), f(12, 33), f(11, 16)},
                List.of(Step.NEWEST, Step.NEWEST, Step.LATE),
                12),
            new Arrivals(
                "the same datagram twice, and a late one twice",
                new long[][] {f(20, 0), f(20, 0), f(22, 33), f(21, 16), f(21, 16)},
                List.of(Step.NEWEST, Step.SEEN_BEFORE, Step.NEWEST, Step.LATE, Step.SEEN_BEFORE),
                22),
            new Arrivals(
                "a restart: counted from 0 again, taken after",
                new long[][] {f(5000, 0), f(0, 900), f(1, 916)},
                List.of(Step.NEWEST, Step.RESTARTED, Step.NEWEST),
                1),
            new Arrivals(
                "a restart soon after its start: its numbers seen before, its frames taken after",
                new long[][] {f(40, 0), f(41, 16), f(3, 2000), f(4, 2016)},
                List.of(Step.NEWEST, Step.NEWEST, Step.RESTARTED, Step.NEWEST),
                4),
            new Arrivals(
                "the oldest a late frame may be",
                new long[][] {f(2000, 0), f(2000 - window + 1, -17_000)},
                List.of(Step.NEWEST, Step.LATE),
                2000),
            new Arrivals(
                "just past it, a restart whenever it was taken",
                new long[][] {f(2000, 0), f(2000 - window, -17_000)},
                List.of(Step.NEWEST, Step.RESTARTED),
                2000 - window),
            new Arrivals(
                "a number seen a window ago is new again once the window passed it",
                new long[][] {f(3, 0), f(3 + window, 17_000), f(4, 16)},
                List.of(Step.NEWEST, Step.NEWEST, Step.LATE),
                3 + window),
            new Arrivals(
                "the 32-bit number wraps: a restart",
                new long[][] {f(WRAP - 1, 0), f(WRAP, 16), f(0, 33)},
                List.of(Step.NEWEST, Step.NEWEST, Step.RESTARTED),
                0));
    for (Arrivals row : rows) {
      FrameNumbers numbers = new FrameNumbers();
      List<Step> steps = new ArrayList<>();
      for (long[] frame : row.frames()) {
        steps.add(numbers.take(frame[0], frame[1]));
      }
      assertThat(steps).as(row.name()).isEqualTo(row.steps());
      assertThat(numbers.highest()).as(row.name()).isEqualTo(row.highest());
    }
  }

  @Test
  void nothingIsHeardBeforeTheFirstFrame() {
    assertThat(new FrameNumbers().highest()).isEqualTo(-1);
  }
}

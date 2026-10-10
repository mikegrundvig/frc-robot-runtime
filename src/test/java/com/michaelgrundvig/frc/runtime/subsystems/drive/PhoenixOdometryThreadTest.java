package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.BaseStatusSignal;
import com.michaelgrundvig.frc.runtime.subsystems.drive.PhoenixOdometryThread.Batch;
import com.michaelgrundvig.frc.runtime.subsystems.drive.PhoenixOdometryThread.Reading;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

/**
 * The odometry thread's failure paths, fed batches a test scripts in place of Phoenix's signals: a
 * read that throws is reported and the thread reads on, and a signal that read badly or arrived
 * late is left out of the readings' time.
 */
class PhoenixOdometryThreadTest {
  private static final long SECOND = 1_000_000_000L;
  private static final long MILLISECOND = 1_000_000L;

  private final List<String> reported = new CopyOnWriteArrayList<>();

  /**
   * Hands out its steps in order, throwing one that's an exception; past them, says it's done and
   * waits to be interrupted.
   */
  private static final class Script implements PhoenixOdometryThread.Signals {
    private final Deque<Object> steps = new ArrayDeque<>();
    final CountDownLatch done = new CountDownLatch(1);

    Script(Object... steps) {
      this.steps.addAll(List.of(steps));
    }

    @Override
    public Batch read(BaseStatusSignal[] signals) throws InterruptedException {
      Object step = steps.poll();
      if (step == null) {
        done.countDown();
        new CountDownLatch(1).await(); // until the test stops the thread
        throw new InterruptedException();
      }
      if (step instanceof RuntimeException thrown) {
        throw thrown;
      }
      return (Batch) step;
    }
  }

  /** Runs the thread through the steps, then stops it: the readings' times it recorded. */
  private List<Long> run(Object... steps) throws InterruptedException {
    Script script = new Script(steps);
    PhoenixOdometryThread thread = new PhoenixOdometryThread(script, reported::add);
    Queue<Long> times = thread.timestamps();
    thread.start();
    try {
      assertThat(script.done.await(2, TimeUnit.SECONDS)).as("read every step").isTrue();
    } finally {
      thread.interrupt();
      thread.join(2000);
    }
    assertThat(thread.isAlive()).as("stopped").isFalse();
    return List.copyOf(times);
  }

  private static Batch batch(long nanos, Reading... readings) {
    return new Batch(nanos, List.of(readings));
  }

  /** A signal read without an error, this many milliseconds in transit. */
  private static Reading fresh(double milliseconds) {
    return new Reading(0, true, milliseconds / 1000);
  }

  @Test
  void aReadThatThrowsIsReportedAndTheThreadReadsOn() throws InterruptedException {
    List<Long> times =
        run(
            batch(SECOND, fresh(2)),
            new IllegalStateException("the bus went away"),
            batch(2 * SECOND, fresh(2)));

    assertThat(reported)
        .containsExactly(
            "Odometry thread: a reading failed: java.lang.IllegalStateException: the bus went away");
    assertThat(times).containsExactly(SECOND - 2 * MILLISECOND, 2 * SECOND - 2 * MILLISECOND);
  }

  /**
   * A reading's time is when it was read, less the signals' average time in transit over those that
   * arrived fresh. Beside two fresh signals (1 and 3 ms, so 2 ms), a third is left out: one that
   * read with an error, one past two periods in transit (its device stopped answering), one before
   * it was sent, and one whose time isn't valid. With none fresh, it's when it was read.
   */
  @Test
  void onlyFreshSignalsTimeTheReading() throws InterruptedException {
    List<Long> times =
        run(
            batch(SECOND, fresh(1), fresh(3)),
            batch(SECOND, fresh(1), fresh(3), new Reading(0, false, 0.005)),
            batch(SECOND, fresh(1), fresh(3), fresh(9)),
            batch(SECOND, fresh(1), fresh(3), fresh(-1)),
            batch(SECOND, fresh(1), fresh(3), new Reading(0, true, Double.NaN)),
            batch(SECOND, new Reading(0, false, 0.002), fresh(20)));

    long twoEarly = SECOND - 2 * MILLISECOND;
    assertThat(times).containsExactly(twoEarly, twoEarly, twoEarly, twoEarly, twoEarly, SECOND);
    assertThat(reported).isEmpty();
  }

  /** A batch of no signals (none registered yet) is no reading. */
  @Test
  void aBatchOfNothingIsNoReading() throws InterruptedException {
    assertThat(run(Batch.NONE, batch(SECOND, fresh(2)))).containsExactly(SECOND - 2 * MILLISECOND);
  }
}

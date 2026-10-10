package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Timestamp;
import com.michaelgrundvig.frc.localization.Timestamps;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.RobotController;

/**
 * Reads the drivetrain's odometry signals as fast as the devices publish them (250 Hz), on its own
 * thread, so odometry sees every turn of the wheels whatever the main loop is doing. It waits until
 * every signal has a new value, then records them together, stamped with when they were measured:
 * now, less the signals' average time in transit; a signal that didn't come reads no number. The
 * main loop drains the readings every loop through the IO layer, so they're logged, and a replay
 * sees exactly what the robot saw.
 *
 * <p>On Systemcore, waiting works on every CAN bus and Phoenix shares the robot's clock (Phoenix
 * 26.70). Register every signal before starting the thread.
 */
final class PhoenixOdometryThread extends Thread {
  /** How often the drivetrain's devices publish odometry signals, and this thread reads them. */
  static final double FREQUENCY_HZ = 250;

  /** Readings held for the main loop: four loops' worth, in case one runs late. */
  private static final int CAPACITY = 20;

  /** A signal longer than two periods in transit is stale: its device has stopped answering. */
  private static final double STALE = 2 / FREQUENCY_HZ;

  private static final long PERIOD_MILLIS = Math.round(1000 / FREQUENCY_HZ);

  /**
   * One signal's reading.
   *
   * @param ok whether it read without an error
   * @param transitSeconds how long it took to arrive; NaN when its time isn't valid
   */
  record Reading(double value, boolean ok, double transitSeconds) {
    /**
     * Whether it came: read without an error, and not past two periods in transit, as one whose
     * device has stopped answering is. A time that isn't valid says nothing either way.
     */
    boolean came() {
      return ok && !(transitSeconds > STALE);
    }
  }

  /**
   * One read of every signal, in the order they were registered.
   *
   * @param nanos when it was read, on the robot's clock
   */
  record Batch(long nanos, List<Reading> readings) {
    static final Batch NONE = new Batch(0, List.of());
  }

  /** Reads the signals once each has a new value: Phoenix's on the robot, a test's own. */
  @FunctionalInterface
  interface Signals {
    Batch read(BaseStatusSignal[] signals) throws InterruptedException;
  }

  private final ReentrantLock lock = new ReentrantLock();
  private final List<Queue<Double>> values = new ArrayList<>();
  private final List<Queue<Long>> timestamps = new ArrayList<>();
  private final Signals source;
  private final Consumer<String> report;
  private BaseStatusSignal[] signals = new BaseStatusSignal[0];

  PhoenixOdometryThread() {
    this(
        PhoenixOdometryThread::waitForAll,
        message -> DriverStationErrors.reportError(message, false));
  }

  /**
   * @param source reads the registered signals
   * @param report says what went wrong, as the Driver Station's errors do
   */
  PhoenixOdometryThread(Signals source, Consumer<String> report) {
    this.source = source;
    this.report = report;
    setName("PhoenixOdometry");
    setDaemon(true);
  }

  /** Reads a signal from now on; its readings arrive on the returned queue, oldest first. */
  Queue<Double> register(BaseStatusSignal signal) {
    lock.lock();
    try {
      BaseStatusSignal[] more = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, more, 0, signals.length);
      more[signals.length] = signal;
      signals = more;
      Queue<Double> queue = new ArrayBlockingQueue<>(CAPACITY);
      values.add(queue);
      return queue;
    } finally {
      lock.unlock();
    }
  }

  /**
   * A queue of every reading's time, in nanoseconds on the robot's clock: one for each device's IO,
   * each drained with that device's readings.
   */
  Queue<Long> timestamps() {
    lock.lock();
    try {
      Queue<Long> queue = new ArrayBlockingQueue<>(CAPACITY);
      timestamps.add(queue);
      return queue;
    } finally {
      lock.unlock();
    }
  }

  /**
   * Hold this while draining the readings, all devices together, so every device's readings stay in
   * step: the thread records only between drains.
   */
  Lock lock() {
    return lock;
  }

  /**
   * Reads until interrupted. Nothing a reading throws stops it: it's reported, and the thread reads
   * again a period later. (If it stopped, odometry would freeze with every device still reporting
   * connected; the drivetrain alerts when readings stop arriving.)
   */
  @Override
  public void run() {
    while (!isInterrupted()) {
      try {
        readOnce();
      } catch (InterruptedException e) {
        return;
      } catch (RuntimeException e) {
        report.accept("Odometry thread: a reading failed: " + e);
        if (!pause()) {
          return;
        }
      }
    }
  }

  private void readOnce() throws InterruptedException {
    BaseStatusSignal[] registered;
    lock.lock();
    try {
      registered = signals;
    } finally {
      lock.unlock();
    }
    record(source.read(registered));
  }

  /**
   * Phoenix's signals: waits until every one has a new value, or two periods pass (a device is
   * missing: its value stays as it was, and its IO reports it disconnected), and reads them.
   */
  private static Batch waitForAll(BaseStatusSignal[] signals) throws InterruptedException {
    if (signals.length == 0) {
      Thread.sleep(PERIOD_MILLIS);
      return Batch.NONE;
    }
    // A wait that fails at once would spin: then it waits a period.
    long before = System.nanoTime();
    StatusCode status = BaseStatusSignal.waitForAll(2 / FREQUENCY_HZ, signals);
    if (!status.isOK() && System.nanoTime() - before < PERIOD_MILLIS * 1_000_000L) {
      Thread.sleep(PERIOD_MILLIS);
    }
    // The time is read before the lock, which the main loop may be holding while it drains.
    long now = RobotController.getTime();
    List<Reading> readings = new ArrayList<>(signals.length);
    for (BaseStatusSignal signal : signals) {
      Timestamp stamp = signal.getTimestamp();
      readings.add(
          new Reading(
              signal.getValueAsDouble(),
              signal.getStatus().isOK(),
              stamp.isValid() ? stamp.getLatency() : Double.NaN));
    }
    return new Batch(now, readings);
  }

  /** Waits a period after a failed reading: false if interrupted meanwhile. */
  private static boolean pause() {
    try {
      Thread.sleep(PERIOD_MILLIS);
      return true;
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      return false;
    }
  }

  private void record(Batch batch) {
    List<Reading> readings = batch.readings();
    if (readings.isEmpty()) {
      return;
    }
    lock.lock();
    try {
      // The signals' average time in transit, over the ones that arrived fresh: a device that's
      // gone stale would otherwise skew every reading's time early.
      double latency = 0;
      int timed = 0;
      for (Reading reading : readings) {
        double transit = reading.transitSeconds();
        if (reading.ok() && transit >= 0 && transit <= STALE) {
          latency += transit;
          timed++;
        }
      }
      long measured = batch.nanos() - (timed == 0 ? 0 : Timestamps.fromSeconds(latency / timed));
      // Each registered signal's queue takes its reading, in the order they were registered: one
      // that didn't come reads no number, rather than the last its device sent. A wheel's is left
      // out of odometry from that reading on, not once its module has been silent long enough to
      // read disconnected.
      for (int i = 0; i < values.size(); i++) {
        Reading reading = readings.get(i);
        keepNewest(values.get(i), reading.came() ? reading.value() : Double.NaN);
      }
      for (Queue<Long> queue : timestamps) {
        keepNewest(queue, measured);
      }
    } finally {
      lock.unlock();
    }
  }

  /**
   * Adds a reading; when the main loop has fallen so far behind that the queue is full, the oldest
   * reading makes room. Every queue fills and drains together, so they stay in step.
   */
  static <T> void keepNewest(Queue<T> queue, T value) {
    if (!queue.offer(value)) {
      queue.poll();
      queue.offer(value);
    }
  }
}

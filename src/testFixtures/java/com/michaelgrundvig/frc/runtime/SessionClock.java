package com.michaelgrundvig.frc.runtime;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.hal.simulation.SimulatorJNI;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.RobotController;

/**
 * The clock a recorded session runs on ({@link SessionRecording}): the simulator's own, stopped,
 * and moved one loop at a time, as the simulator's tests move theirs ({@code RobotSim}). A session
 * is a whole robot program in a process of its own. Left on the computer's clock, how many loops
 * fit in a second of the session hung on how busy the computer was, and so did everything the
 * session times: an experiment, a deadman, when a camera's image falls due. On this clock a session
 * is the same on any computer, however busy, and takes only as long as the computer needs.
 *
 * <p>The clock starts at {@link #STARTS_AT_NANOS} whatever the computer's own says, and the program
 * starts up at that moment. Each loop runs with the clock standing still, a loop's time after the
 * one before: loop n runs n loops' time after the start. The session's robot moves the clock when a
 * loop is over ({@link #loopIsOver}), on the loop's own thread, and its loop never waits for its
 * time, as a replay's does not ({@code setUseTiming(false)}).
 */
final class SessionClock {
  /**
   * What the clock says as the program starts, in nanoseconds: one second. It is not zero, because
   * WPILib takes an alert raised at zero for one that was never raised.
   */
  static final long STARTS_AT_NANOS = 1_000_000_000L;

  /**
   * How long a session waits for anything it waits for before it ends the program: far longer than
   * any of it takes on the busiest computer, and short of the time a session is allowed.
   */
  private static final long PATIENCE_SECONDS = 60;

  /** How often something waited for is looked at again. */
  private static final long LOOK_AGAIN_MILLIS = 1;

  /**
   * The most records the log may have waiting before the clock waits for it. The log writes on a
   * thread of its own, and drops what it is handed while 500 wait. A session's loops come as fast
   * as the computer runs them, so on a busy computer that thread could fall behind.
   */
  private static final long MOST_RECORDS_WAITING = 100;

  /** How many records the robot's loop has handed the log: the startup's, then each loop's. */
  private long recordsHanded;

  /** How many of them the log has taken, counted on the log's thread. */
  private final AtomicLong recordsTaken = new AtomicLong();

  private SessionClock() {}

  /**
   * Stops the simulator's clock at {@link #STARTS_AT_NANOS}, before the robot program is made, so
   * that everything the program makes starts at the same moment in every session. Call it once
   * WPILib's simulation has started, and before AdvantageKit's log has.
   *
   * <p>The clock counts from wherever the computer's own stood, which is days of uptime on most. So
   * it is moved back, by exactly what it says while it stands still.
   */
  static SessionClock stopped() {
    SimHooks.pauseTiming();
    SimulatorJNI.stepTimingAsync(STARTS_AT_NANOS - RobotController.getMonotonicTime());
    SessionClock clock = new SessionClock();
    // AdvantageKit hands every record to this, on the log's own thread.
    Logger.addDataReceiver(table -> clock.recordsTaken.incrementAndGet());
    return clock;
  }

  /**
   * Moves the clock one loop's time on. Call it on the robot's loop's thread when a loop is over
   * and before the next is given its time, and once when the program has started up, so that the
   * first loop's time is not the startup's: the log tells its records apart by their times.
   *
   * <p>The record of what just ended is handed to the log after this. If the log has fallen far
   * behind, the clock waits here for it first, so that no record is ever dropped.
   */
  void loopIsOver() {
    await(
        "the log to take the records it was handed",
        () -> recordsHanded - recordsTaken.get() < MOST_RECORDS_WAITING);
    recordsHanded++;
    RobotSim.stepClock();
  }

  /**
   * Waits, on the computer's clock, for something another thread does, looking again every
   * millisecond. A session waits this way for what must have happened before it goes on, and never
   * for a length of time. If it has not happened in {@link #PATIENCE_SECONDS}, the program ends,
   * saying what it waited for.
   */
  static void await(String what, BooleanSupplier happened) {
    long giveUpAt = System.nanoTime() + TimeUnit.SECONDS.toNanos(PATIENCE_SECONDS);
    while (!happened.getAsBoolean()) {
      if (System.nanoTime() > giveUpAt) {
        giveUp(what);
      }
      try {
        Thread.sleep(LOOK_AGAIN_MILLIS);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        giveUp(what);
      }
    }
  }

  /** Ends the program at once, saying what never happened. */
  private static void giveUp(String what) {
    System.out.println(
        "The session waited " + PATIENCE_SECONDS + " s for " + what + ", and it never came");
    Runtime.getRuntime().halt(1);
  }
}

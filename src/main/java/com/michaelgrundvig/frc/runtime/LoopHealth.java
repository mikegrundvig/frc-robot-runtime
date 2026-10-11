package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.runtime.lib.timing.LoopMonitor;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import java.io.File;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.util.Alert;

/**
 * Whether the robot loop keeps up, and its log with it; and, on a robot or in a replay of one,
 * whether its commands run. {@link Robot} starts and finishes each loop here, and the robot's
 * container marks the sections of the loop on {@link #monitor()}.
 *
 * <p>It raises four alerts:
 *
 * <ul>
 *   <li><b>Overrunning.</b> Loops overran, which is time spent in the robot's own code, or started
 *       late, which is time lost between loops (writing the log, dashboards, garbage collection).
 *       The alert names where the time went.
 *   <li><b>Logging behind.</b> The log fell behind and dropped data. That log has gaps, so the
 *       alert stays raised for the rest of the session.
 *   <li><b>The log drive.</b> The drive the log goes to is missing, cannot be written, or is nearly
 *       full. It is checked once a second on a thread of its own, so that a drive which hangs never
 *       stalls the robot loop.
 *   <li><b>Commands threw</b> ({@link #threw}), on a robot or in a replay of one. Each place they
 *       threw from is listed once. The code has a bug, so the alert stays raised for the rest of
 *       the session.
 * </ul>
 */
final class LoopHealth implements AutoCloseable {
  /** How long "lately" covers, in seconds. */
  private static final double RECENT_SECONDS = 10;

  /** A log drive with less room than this is nearly full. It is about half an hour of logging. */
  static final long MIN_FREE_BYTES = 500L * 1024 * 1024;

  /**
   * How many loops in a row the commands may throw, on a robot or in a replay of one, before the
   * program ends. It is half a second.
   *
   * <p>A command that throws once costs one loop. Code that throws every loop (a binding's
   * condition, a default command, a scheduler's periodic task that threw and so is over) leaves
   * nothing running while the robot drives on with its last outputs. Then it is better that the
   * program ends, and restarts, than that it runs on wedged.
   */
  static final int FAILING_LOOPS = 25;

  private static final String ERRORS_KEY = "Errors/Commands";

  /**
   * A place that commands threw from.
   *
   * @param kind the exception's class
   * @param stack its stack
   */
  private record Site(Class<?> kind, List<StackTraceElement> stack) {}

  private final LoopMonitor monitor =
      new LoopMonitor(RobotLoop.PERIOD_SECONDS, RobotLoop.loops(RECENT_SECONDS));
  private final Alert overrunning =
      new Alert("Loop", "overrunning", "The loop is overrunning", Alert.Level.MEDIUM);
  private final Alert loggingBehind =
      new Alert(
          "Loop",
          "loggingBehind",
          "Logging fell behind and dropped data this session: this log has gaps",
          Alert.Level.HIGH);
  private final Alert logDrive = new Alert("Loop", "logDrive", "", Alert.Level.HIGH);
  private final Alert threw = new Alert("Loop", "threw", "", Alert.Level.HIGH);

  // Commands that threw.

  /** Each place commands threw from. Each is reported once. */
  private final Set<Site> reported = new HashSet<>();

  /** What the alert lists: each place commands threw from, once, in the order they first did. */
  private final List<String> threwFrom = new ArrayList<>();

  /** How many loops in a row the commands have thrown in, up to and including this one. */
  private int failingLoops;

  private boolean threwThisLoop;

  // The log drive.

  /** The thread that watches the log drive, once it is started. */
  private Optional<ScheduledExecutorService> driveWatch = Optional.empty();

  /** What is wrong with the log drive, as its watching thread last found, if anything is. */
  private volatile Optional<String> logDriveProblem = Optional.empty();

  // The loop's timing.

  private LoopMonitor.Report last = LoopMonitor.Report.NONE;

  /**
   * What the overrun alert adds about the last loop that overran: how long it took, and in which
   * section most of it went. Empty until a loop has overrun.
   */
  private String lastOverrun = "";

  /** The loop's timing, for marking the sections of a loop. */
  LoopMonitor monitor() {
    return monitor;
  }

  /** The last loop's timing, section by section. */
  LoopMonitor.Report last() {
    return last;
  }

  /** A loop begins. */
  void start(long nowNanos) {
    monitor.start(nowNanos);
  }

  /**
   * A loop is done: logs its timing section by section, and brings the alerts up to date.
   *
   * @param logDropped whether the log dropped data this loop, because AdvantageKit's queue to its
   *     writers was full
   */
  void finish(long nowNanos, boolean logDropped) {
    last = monitor.finish(nowNanos);
    logTiming();
    showOverruns();
    if (logDropped) {
      // Never lowered again: the log has gaps from here on.
      loggingBehind.set(true);
    }
    showLogDrive();
    if (!threwThisLoop) {
      failingLoops = 0;
    }
    threwThisLoop = false;
  }

  private void logTiming() {
    Logger.recordOutput("Loop/Milliseconds", last.seconds() * 1000);
    Logger.recordOutput("Loop/OverrunsLately", last.overrunsLately());
    Logger.recordOutput("Loop/WorstLatelyMilliseconds", last.worstLatelySeconds() * 1000);
    Logger.recordOutput("Loop/LateLately", last.lateLately());
    Logger.recordOutput("Loop/OutsideMilliseconds", last.outsideSeconds() * 1000);
    for (LoopMonitor.Section section : last.sections()) {
      Logger.recordOutput(
          "Loop/Sections/" + section.name() + "Milliseconds", section.seconds() * 1000);
    }
  }

  /** Raises the overrun alert while loops have overrun or started late lately, saying how. */
  private void showOverruns() {
    LoopMonitor.Section slowest = last.slowest();
    if (last.overran() && slowest != null) {
      lastOverrun =
          format(
              "; the last took %.0f ms, %.0f of them in %s",
              last.seconds() * 1000, slowest.seconds() * 1000, slowest.name());
    }
    boolean isInTrouble = last.overrunsLately() > 0 || last.lateLately() > 0;
    if (isInTrouble) {
      overrunning.setText(overrunText());
    }
    overrunning.set(isInTrouble);
  }

  /**
   * What the overrun alert says: how often the loop overran lately, how many loops started late
   * lately, or both with "; " between them.
   */
  private String overrunText() {
    boolean overran = last.overrunsLately() > 0;
    boolean startedLate = last.lateLately() > 0;
    if (overran && startedLate) {
      return overranText() + "; " + startedLateText();
    }
    if (overran) {
      return overranText();
    }
    if (startedLate) {
      return startedLateText();
    }
    return "";
  }

  private String overranText() {
    return format(
        "The loop overran %.0f ms %d %s in the last %.0f s%s",
        RobotLoop.PERIOD_SECONDS * 1000,
        last.overrunsLately(),
        last.overrunsLately() == 1 ? "time" : "times",
        RECENT_SECONDS,
        lastOverrun);
  }

  private String startedLateText() {
    return format(
        "%d %s started late in the last %.0f s: time lost between loops (writing the log,"
            + " dashboards, garbage collection)",
        last.lateLately(), last.lateLately() == 1 ? "loop" : "loops", RECENT_SECONDS);
  }

  /** Raises the log drive's alert while its watching thread finds something wrong with it. */
  private void showLogDrive() {
    Optional<String> problem = logDriveProblem;
    if (problem.isPresent() && !problem.get().equals(logDrive.getText())) {
      logDrive.setText(problem.get());
    }
    logDrive.set(problem.isPresent());
  }

  /**
   * The commands threw this loop, on a robot or in a replay of one ({@link Robot#loop}). What threw
   * may be a command, a binding's condition, or anything else the scheduler runs. The scheduler has
   * already ended a command that threw, and the command that started it, and the robot carries on
   * without them.
   *
   * <p>Every throw is logged ({@code Errors/Commands}, with the stack). Each place that commands
   * threw from is reported once, as a Driver Station error with the stack and in the alert.
   *
   * @throws RuntimeException the same exception, on the {@value #FAILING_LOOPS}th loop in a row
   *     whose commands threw. The program then ends, and restarts, which is better than running on
   *     with nothing running.
   */
  void threw(RuntimeException e) {
    threwThisLoop = true;
    failingLoops++;
    StackTraceElement[] frames = e.getStackTrace();
    StringWriter stack = new StringWriter();
    e.printStackTrace(new PrintWriter(stack));
    Logger.recordOutput(ERRORS_KEY, stack.toString());

    boolean isANewSite = reported.add(new Site(e.getClass(), List.of(frames)));
    if (isANewSite) {
      report(e, frames);
    }
    if (failingLoops >= FAILING_LOOPS) {
      DriverStationErrors.reportError(
          "Commands threw in " + FAILING_LOOPS + " loops in a row: the program ends, and restarts",
          frames);
      throw e;
    }
  }

  /** Reports a place commands threw from: to the Driver Station, and in the alert's list. */
  private void report(RuntimeException e, StackTraceElement[] frames) {
    DriverStationErrors.reportError(
        "Commands threw " + what(e) + ". The robot carries on without them", frames);
    String where = frames.length > 0 ? ", at " + frames[0] : "";
    threwFrom.add(what(e) + where);
    threw.setText(
        "Commands threw, and the robot carried on without them: "
            + String.join("; ", threwFrom)
            + ". Fix the code: the stack is in the Driver Station's log");
    threw.set(true);
  }

  /**
   * What was thrown, by its kind and what it says: "IllegalStateException: no reading yet". An
   * exception that says nothing is named by its kind alone.
   */
  private static String what(RuntimeException e) {
    String message = e.getMessage();
    String kind = e.getClass().getSimpleName();
    if (message == null || message.isBlank()) {
      return kind;
    }
    return kind + ": " + message;
  }

  /** One line for the pit: how long loops take, and whether they keep up. */
  String describe() {
    return last.describe();
  }

  /**
   * Watches the drive the log goes to: once now, and then every second on a thread of its own. A
   * USB drive that hangs would otherwise hang the robot loop. This is called on a real robot only.
   */
  void watchLogDrive(File drive) {
    logDriveProblem = logDriveProblem(drive, MIN_FREE_BYTES);
    ScheduledExecutorService watch =
        Executors.newSingleThreadScheduledExecutor(
            task -> {
              Thread thread = new Thread(task, "Log drive watch");
              thread.setDaemon(true);
              return thread;
            });
    watch.scheduleWithFixedDelay(
        () -> logDriveProblem = logDriveProblem(drive, MIN_FREE_BYTES), 1, 1, TimeUnit.SECONDS);
    driveWatch = Optional.of(watch);
  }

  /**
   * Why a session is not being logged, or soon will not be, if either is so: there is no drive
   * where the log goes, the drive cannot be written, or it is nearly full.
   */
  static Optional<String> logDriveProblem(File drive, long minimumFreeBytes) {
    if (!drive.isDirectory()) {
      return Optional.of("No USB drive at " + drive + ": this session isn't being logged");
    }
    if (!drive.canWrite()) {
      return Optional.of(
          "The USB drive at " + drive + " can't be written: this session isn't being logged");
    }
    long free = drive.getUsableSpace();
    if (free < minimumFreeBytes) {
      return Optional.of(
          format(
              "The USB drive at %s has only %d MB free: logging stops when it's full",
              drive, free >> 20));
    }
    return Optional.empty();
  }

  boolean isOverrunAlertActive() {
    return overrunning.get();
  }

  String overrunAlertText() {
    return overrunning.getText();
  }

  boolean isLoggingBehindAlertActive() {
    return loggingBehind.get();
  }

  boolean isThrewAlertActive() {
    return threw.get();
  }

  String threwAlertText() {
    return threw.getText();
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }

  @Override
  public void close() {
    if (driveWatch.isPresent()) {
      driveWatch.get().shutdownNow();
    }
    overrunning.close();
    loggingBehind.close();
    logDrive.close();
    threw.close();
  }
}

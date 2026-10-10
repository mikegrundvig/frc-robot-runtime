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
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.util.Alert;

/**
 * Whether the robot loop keeps up, and its log with it, and on a robot (or in a replay of one)
 * whether its commands run. {@link Robot} starts and finishes each loop here; the container marks
 * its sections on {@link #monitor()}.
 *
 * <p>It raises four alerts:
 *
 * <ul>
 *   <li>loops that overran, in the robot's own code, or started late, from time lost between loops
 *       (writing the log, dashboards, garbage collection), naming where the time went;
 *   <li>a log that fell behind and dropped data: that log has gaps, so the alert stays raised for
 *       the rest of the session;
 *   <li>a log drive that's missing, can't be written, or is nearly full, checked every second away
 *       from the robot loop, so a drive that hangs never stalls it;
 *   <li>on a robot, and in a replay of one, commands that threw ({@link #threw}), each place once:
 *       the code has a bug, so the alert stays raised for the rest of the session.
 * </ul>
 */
final class LoopHealth implements AutoCloseable {
  /** How long "lately" covers. */
  private static final double RECENT_SECONDS = 10;

  /** A log drive with less room than this is nearly full: about half an hour of logging. */
  static final long MIN_FREE_BYTES = 500L * 1024 * 1024;

  /**
   * How many loops in a row the commands may throw on a robot (or in a replay of one, which follows
   * it) before the program ends: half a second. A command that throws once costs a loop. Code that
   * throws every loop (a binding's condition, a default command, a sideload that threw and so is
   * over) runs nothing while the robot drives on its last outputs, so the program ends, and
   * restarts, rather than run wedged.
   */
  static final int FAILING_LOOPS = 25;

  private static final String ERRORS_KEY = "Errors/Commands";

  /** Where commands threw from: an exception's kind and its stack. */
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

  /** Each place commands threw from, reported once. */
  private final Set<Site> reported = new HashSet<>();

  /** What the alert lists: each place commands threw from, once, in the order they did. */
  private final List<String> threwFrom = new ArrayList<>();

  /** Loops in a row whose commands threw. */
  private int failingLoops;

  private boolean failedThisLoop;
  private @Nullable ScheduledExecutorService driveWatch;
  private volatile @Nullable String logDriveProblem;
  private LoopMonitor.Report last = LoopMonitor.Report.NONE;
  private String lastOverrun = "";

  /** The loop's timing, for marking the sections of it. */
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
   * The loop is done: logs its timing section by section, and updates the alerts.
   *
   * @param logDropped whether the log dropped data this loop (AdvantageKit's queue to its writers
   *     was full)
   */
  void finish(long nowNanos, boolean logDropped) {
    last = monitor.finish(nowNanos);
    Logger.recordOutput("Loop/Milliseconds", last.seconds() * 1000);
    Logger.recordOutput("Loop/OverrunsLately", last.overrunsLately());
    Logger.recordOutput("Loop/WorstLatelyMilliseconds", last.worstLatelySeconds() * 1000);
    Logger.recordOutput("Loop/LateLately", last.lateLately());
    Logger.recordOutput("Loop/OutsideMilliseconds", last.outsideSeconds() * 1000);
    for (LoopMonitor.Section section : last.sections()) {
      Logger.recordOutput(
          "Loop/Sections/" + section.name() + "Milliseconds", section.seconds() * 1000);
    }
    LoopMonitor.Section slowest = last.slowest();
    if (last.overran() && slowest != null) {
      lastOverrun =
          format(
              "; the last took %.0f ms, %.0f of them in %s",
              last.seconds() * 1000, slowest.seconds() * 1000, slowest.name());
    }
    boolean trouble = last.overrunsLately() > 0 || last.lateLately() > 0;
    if (trouble) {
      overrunning.setText(overrunText());
    }
    overrunning.set(trouble);
    if (logDropped) {
      loggingBehind.set(true); // stays raised: the log has gaps
    }
    String driveProblem = logDriveProblem;
    if (driveProblem != null && !driveProblem.equals(logDrive.getText())) {
      logDrive.setText(driveProblem);
    }
    logDrive.set(driveProblem != null);
    if (!failedThisLoop) {
      failingLoops = 0;
    }
    failedThisLoop = false;
  }

  /**
   * The commands threw this loop, on a robot or in a replay of one ({@link Robot#loop}): a command,
   * a binding's condition, anything the scheduler runs. The scheduler has already ended a command
   * that threw, and the command that started it, and the robot carries on without them. It's logged
   * every time ({@code Errors/Commands}, with the stack), and each place it threw from is reported
   * once: a Driver Station error with the stack, and the alert.
   *
   * @throws RuntimeException the same exception, on the {@value #FAILING_LOOPS}th loop in a row
   *     whose commands threw: the program ends, and restarts, rather than run with nothing running
   */
  void threw(RuntimeException e) {
    failedThisLoop = true;
    failingLoops++;
    StackTraceElement[] frames = e.getStackTrace();
    StringWriter stack = new StringWriter();
    e.printStackTrace(new PrintWriter(stack));
    Logger.recordOutput(ERRORS_KEY, stack.toString());
    if (reported.add(new Site(e.getClass(), List.of(frames)))) {
      DriverStationErrors.reportError(
          "Commands threw " + what(e) + ". The robot carries on without them", frames);
      threwFrom.add(what(e) + (frames.length > 0 ? ", at " + frames[0] : ""));
      threw.setText(
          "Commands threw, and the robot carried on without them: "
              + String.join("; ", threwFrom)
              + ". Fix the code: the stack is in the Driver Station's log");
      threw.set(true);
    }
    if (failingLoops >= FAILING_LOOPS) {
      DriverStationErrors.reportError(
          "Commands threw in " + FAILING_LOOPS + " loops in a row: the program ends, and restarts",
          frames);
      throw e;
    }
  }

  /** "IllegalStateException: no reading yet", or its kind alone when it says nothing. */
  private static String what(RuntimeException e) {
    String message = e.getMessage();
    String kind = e.getClass().getSimpleName();
    return message == null || message.isBlank() ? kind : kind + ": " + message;
  }

  private String overrunText() {
    String overran =
        last.overrunsLately() == 0
            ? ""
            : format(
                "The loop overran %.0f ms %d %s in the last %.0f s%s",
                RobotLoop.PERIOD_SECONDS * 1000,
                last.overrunsLately(),
                last.overrunsLately() == 1 ? "time" : "times",
                RECENT_SECONDS,
                lastOverrun);
    String late =
        last.lateLately() == 0
            ? ""
            : format(
                "%d %s started late in the last %.0f s: time lost between loops (writing the log,"
                    + " dashboards, garbage collection)",
                last.lateLately(), last.lateLately() == 1 ? "loop" : "loops", RECENT_SECONDS);
    return overran.isEmpty() ? late : late.isEmpty() ? overran : overran + "; " + late;
  }

  /** One line for the pit: how long loops take, and whether they keep up. */
  String describe() {
    return last.describe();
  }

  /**
   * Watches the drive the log goes to: now, then every second on a thread of its own, since a USB
   * drive that hangs would otherwise hang the robot loop. On the robot only.
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
    driveWatch = watch;
  }

  /**
   * Why a session isn't being logged, or soon won't be: no drive where the log goes, one that can't
   * be written, or one nearly full.
   *
   * @return null when the drive is there, writable, and has room
   */
  static @Nullable String logDriveProblem(File drive, long minimumFreeBytes) {
    if (!drive.isDirectory()) {
      return "No USB drive at " + drive + ": this session isn't being logged";
    }
    if (!drive.canWrite()) {
      return "The USB drive at " + drive + " can't be written: this session isn't being logged";
    }
    long free = drive.getUsableSpace();
    if (free < minimumFreeBytes) {
      return format(
          "The USB drive at %s has only %d MB free: logging stops when it's full",
          drive, free >> 20);
    }
    return null;
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
    ScheduledExecutorService watch = driveWatch;
    if (watch != null) {
      watch.shutdownNow();
    }
    overrunning.close();
    loggingBehind.close();
    logDrive.close();
    threw.close();
  }
}

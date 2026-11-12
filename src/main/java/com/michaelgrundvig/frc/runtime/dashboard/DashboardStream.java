package com.michaelgrundvig.frc.runtime.dashboard;

import com.michaelgrundvig.frc.runtime.lib.Said;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import java.util.function.Consumer;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.util.Alert;

/**
 * Streams the log to dashboards. In the pit every loop is streamed. On a robot's field only one
 * loop in {@link #FIELD_DIVISOR} is, because the radio's bandwidth is capped there, and the log on
 * the robot's drive keeps every loop anyway; the simulator has no radio, and streams every loop
 * ({@link OnTheField}).
 *
 * <p>Whether the field is attached is read from the Driver Station data in the log itself, so a
 * replay streams the same loops the robot did.
 *
 * <p>AdvantageKit calls every receiver in turn on one thread and catches nothing, so a throw here
 * would end that thread, and with it the log on the robot's drive. The stream catches its own: the
 * first loop that throws stops the stream, which says so once, and the log goes on.
 */
public final class DashboardStream implements LogDataReceiver {
  /** On the field, one loop in this many is streamed, which comes to 10 Hz. */
  static final int FIELD_DIVISOR = (int) Math.round(RobotLoop.FREQUENCY_HZ / 10);

  /** Whether it streams the loops, or stopped after one that threw. */
  private enum Streaming {
    STREAMING,
    STOPPED
  }

  /** How much of the log streams while the field is attached. */
  public enum OnTheField {
    /** One loop in {@link #FIELD_DIVISOR}: a robot's, whose radio's bandwidth is capped there. */
    THINNED,
    /** Every loop: the simulator's, which has no radio. */
    EVERY_LOOP
  }

  private final LogDataReceiver dashboards;
  private final OnTheField onTheField;
  private final Consumer<String> stopped;
  private Streaming state = Streaming.STREAMING;

  /** How many loops have been offered to the stream so far. */
  private long loops;

  /**
   * @param dashboards where the log streams to: AdvantageKit's NetworkTables publisher
   * @param onTheField how much of it streams while the field is attached
   */
  public DashboardStream(LogDataReceiver dashboards, OnTheField onTheField) {
    this(dashboards, onTheField, DashboardStream::alert);
  }

  /**
   * The same, saying through {@code stopped} that the stream stopped after a loop that threw.
   *
   * @param stopped told once, with what to show, when the stream stops
   */
  DashboardStream(LogDataReceiver dashboards, OnTheField onTheField, Consumer<String> stopped) {
    this.dashboards = dashboards;
    this.onTheField = onTheField;
    this.stopped = stopped;
  }

  /** Raises a robot alert that stays up for the rest of the session. */
  private static void alert(String text) {
    // Never closed: the alert stays up until the program ends, as the stream stays stopped.
    Alert alert = new Alert("Dashboards", "streamStopped", text, Alert.Level.HIGH);
    alert.set(true);
  }

  @Override
  public void start() {
    dashboards.start();
  }

  @Override
  public void end() {
    dashboards.end();
  }

  /**
   * Streams one loop, if it's this loop's turn, and never throws what the stream threw: the
   * dashboards stop at the loop before, and the log on the robot's drive goes on.
   */
  @Override
  public void putTable(LogTable table) throws InterruptedException {
    if (state == Streaming.STOPPED) {
      return;
    }
    try {
      boolean thinned =
          onTheField == OnTheField.THINNED && table.get("DriverStation/FMSAttached", false);
      boolean isThisLoopsTurn = loops % FIELD_DIVISOR == 0;
      if (!thinned || isThisLoopsTurn) {
        dashboards.putTable(table);
      }
      loops++;
    } catch (RuntimeException thrown) {
      stop(thrown);
    }
  }

  private void stop(RuntimeException thrown) {
    state = Streaming.STOPPED;
    String text =
        "The dashboards' stream stopped: they see no more of this session, and the log on the"
            + " robot's drive goes on. "
            + Said.of(thrown);
    try {
      // The stack, for whoever reads the Driver Station's console: the alert has room for a line.
      DriverStationErrors.reportError(text, thrown.getStackTrace());
      stopped.accept(text);
    } catch (RuntimeException ignored) {
      // Saying so failed too, and there is nowhere left to say it: the log still goes on.
    }
  }
}

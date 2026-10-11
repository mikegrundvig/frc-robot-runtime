package com.michaelgrundvig.frc.runtime.dashboard;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/**
 * Streams the log to dashboards. In the pit every loop is streamed. On the field only one loop in
 * {@link #FIELD_DIVISOR} is, because the radio's bandwidth is capped there, and the log on the
 * robot's drive keeps every loop anyway.
 *
 * <p>Whether the field is attached is read from the Driver Station data in the log itself, so a
 * replay streams the same loops the robot did.
 */
public final class DashboardStream implements LogDataReceiver {
  /** On the field, one loop in this many is streamed, which comes to 10 Hz. */
  static final int FIELD_DIVISOR = (int) Math.round(RobotLoop.FREQUENCY_HZ / 10);

  private final LogDataReceiver dashboards;

  /** How many loops have been offered to the stream so far. */
  private long loops;

  /**
   * @param dashboards where the log streams to: AdvantageKit's NetworkTables publisher
   */
  public DashboardStream(LogDataReceiver dashboards) {
    this.dashboards = dashboards;
  }

  @Override
  public void start() {
    dashboards.start();
  }

  @Override
  public void end() {
    dashboards.end();
  }

  @Override
  public void putTable(LogTable table) throws InterruptedException {
    boolean onTheField = table.get("DriverStation/FMSAttached", false);
    boolean isThisLoopsTurn = loops % FIELD_DIVISOR == 0;
    if (!onTheField || isThisLoopsTurn) {
      dashboards.putTable(table);
    }
    loops++;
  }
}

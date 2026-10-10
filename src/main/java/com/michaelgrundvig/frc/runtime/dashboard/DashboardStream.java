package com.michaelgrundvig.frc.runtime.dashboard;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/**
 * Streams the log to dashboards: every loop in the pit, and one loop in {@link #FIELD_DIVISOR} on
 * the field, where the radio's bandwidth is capped and the log on the robot's drive keeps
 * everything anyway. Whether the field is attached comes from the logged Driver Station data
 * itself, so a replay streams the same loops.
 */
public final class DashboardStream implements LogDataReceiver {
  /** On the field, stream one loop in this many: 10 Hz. */
  static final int FIELD_DIVISOR = (int) Math.round(RobotLoop.FREQUENCY_HZ / 10);

  private final LogDataReceiver dashboards;
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
    if (!onTheField || loops % FIELD_DIVISOR == 0) {
      dashboards.putTable(table);
    }
    loops++;
  }
}

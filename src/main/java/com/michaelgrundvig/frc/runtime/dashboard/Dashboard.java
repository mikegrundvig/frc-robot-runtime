package com.michaelgrundvig.frc.runtime.dashboard;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.wpilib.driverstation.MatchState;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.AlertDataJNI;

/**
 * What the drive team's dashboard shows: a few values under {@code Dashboard/}, which appear in
 * NetworkTables under {@code /Telemetry/Dashboard/}. They're published every loop from startup, so
 * no widget is ever blank just because nothing has happened yet.
 *
 * <p>The Elastic layout in {@code layouts/elastic-layout.json} reads them, and {@code
 * DashboardLayoutTest} fails if any of its widgets reads a value nothing publishes: rename one
 * here, and change the layout with it. They are the dashboard's own values, apart from the log, so
 * renaming a logged value never blanks the dashboard, and the dashboard keeps its full rate on the
 * field, where the log's stream is thinned ({@link DashboardStream}).
 */
public final class Dashboard {
  static final String MATCH_TIME = "Dashboard/MatchTime";
  static final String READY_TO_SCORE = "Dashboard/ReadyToScore";
  static final String SCORING = "Dashboard/Scoring";
  static final String STATION = "Dashboard/Station";
  static final String ON_START = "Dashboard/OnStart";
  static final String PLACEMENT = "Dashboard/Placement";
  static final String LOCALIZATION = "Dashboard/Localization";
  static final String CAMERAS = "Dashboard/Cameras";
  static final String BATTERY_VOLTS = "Dashboard/BatteryVolts";
  static final String LOOP = "Dashboard/Loop";

  /**
   * Every active alert, in the shape Elastic's Alerts widget reads: {@code errors}, {@code
   * warnings}, and {@code infos} (high, medium, and low), newest first.
   */
  static final String ALERTS = "Dashboard/Alerts";

  private Dashboard() {}

  /** Publishes every value: once a loop, after the robot has updated them. */
  public static void publish(RobotStatus robot) {
    Telemetry.log(MATCH_TIME, MatchState.getMatchTime());
    Telemetry.log(READY_TO_SCORE, robot.readyToScore());
    Telemetry.log(SCORING, robot.scoring());
    Telemetry.log(STATION, robot.station());
    Telemetry.log(ON_START, robot.onStart());
    Telemetry.log(PLACEMENT, robot.placement());
    Telemetry.log(LOCALIZATION, robot.localization());
    Telemetry.log(CAMERAS, robot.cameras());
    Telemetry.log(BATTERY_VOLTS, RobotController.getBatteryVoltage());
    Telemetry.log(LOOP, robot.loop());
    publishAlerts();
  }

  /**
   * Every active alert, by level: WPILib keeps them all, whoever raised them (this code, a library,
   * a vendor's), and they appear here the way Elastic's Alerts widget expects them.
   */
  private static void publishAlerts() {
    List<AlertDataJNI.AlertInfo> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.activeStartTime != 0) {
        active.add(alert);
      }
    }
    active.sort(
        Comparator.comparingLong((AlertDataJNI.AlertInfo a) -> a.activeStartTime).reversed());
    Telemetry.log(ALERTS + "/.type", "Alerts");
    Telemetry.log(ALERTS + "/errors", texts(active, AlertDataJNI.LEVEL_HIGH));
    Telemetry.log(ALERTS + "/warnings", texts(active, AlertDataJNI.LEVEL_MEDIUM));
    Telemetry.log(ALERTS + "/infos", texts(active, AlertDataJNI.LEVEL_LOW));
  }

  private static String[] texts(List<AlertDataJNI.AlertInfo> alerts, int level) {
    return alerts.stream()
        .filter(alert -> alert.level == level)
        .map(alert -> alert.text)
        .toArray(String[]::new);
  }
}

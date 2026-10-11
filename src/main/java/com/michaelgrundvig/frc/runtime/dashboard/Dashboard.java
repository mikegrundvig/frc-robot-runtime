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
 * NetworkTables under {@code /Telemetry/Dashboard/}. Every value is published every loop from
 * startup, so no widget is ever blank only because nothing has happened yet.
 *
 * <p>The Elastic layout in {@code layouts/elastic-layout.json} reads these values, and {@code
 * DashboardLayoutTest} fails if any of its widgets reads a value that nothing publishes. So a value
 * renamed here must be renamed in the layout too.
 *
 * <p>These are the dashboard's own values, kept apart from the log on purpose. Renaming a logged
 * value never blanks the dashboard. And the dashboard keeps its full rate on the field, where the
 * log's stream is thinned ({@link DashboardStream}).
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
   * warnings} and {@code infos}, which are the high, medium and low alerts, each newest first.
   */
  static final String ALERTS = "Dashboard/Alerts";

  private Dashboard() {}

  /** Publishes every value. Called once a loop, after the robot has updated what they show. */
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
   * Publishes every active alert, by level. WPILib keeps every alert, whoever raised it (this code,
   * a library, a vendor's library), so all of them appear here.
   */
  private static void publishAlerts() {
    List<AlertDataJNI.AlertInfo> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      // WPILib gives an alert that is not raised a start time of zero.
      boolean isRaised = alert.activeStartTime != 0;
      if (isRaised) {
        active.add(alert);
      }
    }
    Comparator<AlertDataJNI.AlertInfo> oldestFirst =
        Comparator.comparingLong(alert -> alert.activeStartTime);
    active.sort(oldestFirst.reversed());
    // The type tells Elastic to show these three lists in its Alerts widget.
    Telemetry.log(ALERTS + "/.type", "Alerts");
    Telemetry.log(ALERTS + "/errors", texts(active, AlertDataJNI.LEVEL_HIGH));
    Telemetry.log(ALERTS + "/warnings", texts(active, AlertDataJNI.LEVEL_MEDIUM));
    Telemetry.log(ALERTS + "/infos", texts(active, AlertDataJNI.LEVEL_LOW));
  }

  /** The texts of the alerts of one level, in the order given. */
  private static String[] texts(List<AlertDataJNI.AlertInfo> alerts, int level) {
    return alerts.stream()
        .filter(alert -> alert.level == level)
        .map(alert -> alert.text)
        .toArray(String[]::new);
  }
}

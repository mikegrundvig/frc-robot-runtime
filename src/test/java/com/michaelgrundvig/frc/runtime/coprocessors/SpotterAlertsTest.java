package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.spotter.manager.Alert;
import com.michaelgrundvig.frc.spotter.manager.Level;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.util.AlertDataJNI;

/**
 * Spotter's alerts through a session: the robot starts with no key while a board requires one, the
 * front board is missing, then connects with its CPU warm, which turns failing and then clears.
 * Each step is the list Spotter's manager would give, and the robot's alerts are read as a
 * dashboard reads them.
 */
class SpotterAlertsTest {
  private static final Alert NO_KEY =
      new Alert(
          "setup/key",
          Level.WARNING,
          "",
          "no Spotter key pair on this controller: boards that require signatures will refuse it");
  private static final Alert MISSING =
      new Alert("10.0.0.11/missing", Level.FAILING, "10.0.0.11", "10.0.0.11: no answer for 5 s");
  private static final String CPU = "vision-front/value/vision.health.cpuTemperature";

  private final AtomicReference<List<Alert>> spotter = new AtomicReference<>(List.of());
  private final SpotterAlerts alerts = new SpotterAlerts(spotter::get);

  @AfterEach
  void close() {
    alerts.close();
  }

  /** One row of the session: what Spotter says, and what the robot's alerts show then. */
  private record Step(String what, List<Alert> spotter, List<String> shown) {}

  @Test
  void eachOfSpottersAlertsIsTheRobotsWhileItStands() {
    List<Step> session =
        List.of(
            new Step("nothing yet", List.of(), List.of()),
            new Step(
                "no key, a board missing",
                List.of(MISSING, NO_KEY),
                List.of(
                    "HIGH 10.0.0.11/missing 10.0.0.11: no answer for 5 s",
                    "MEDIUM setup/key " + NO_KEY.text())),
            new Step(
                "the board connects warm",
                List.of(cpu(Level.WARNING, 81), NO_KEY),
                List.of(
                    "MEDIUM " + CPU + " vision-front: CPU temperature above 80 °C, 81 °C",
                    "MEDIUM setup/key " + NO_KEY.text())),
            new Step(
                "warmer, same level",
                List.of(cpu(Level.WARNING, 84), NO_KEY),
                List.of(
                    "MEDIUM " + CPU + " vision-front: CPU temperature above 80 °C, 84 °C",
                    "MEDIUM setup/key " + NO_KEY.text())),
            new Step(
                "failing: the same key at another level",
                List.of(cpu(Level.FAILING, 91), NO_KEY),
                List.of(
                    "HIGH " + CPU + " vision-front: CPU temperature above 80 °C, 91 °C",
                    "MEDIUM setup/key " + NO_KEY.text())),
            new Step("cooled", List.of(NO_KEY), List.of("MEDIUM setup/key " + NO_KEY.text())),
            new Step("the key in place", List.of(), List.of()));

    for (Step step : session) {
      spotter.set(step.spotter());
      alerts.periodic();
      alerts.periodic(); // the same list again changes nothing
      assertThat(shown()).as(step.what()).containsExactlyInAnyOrderElementsOf(step.shown());
    }
  }

  @Test
  void closingTakesThemAllDown() {
    spotter.set(List.of(MISSING, NO_KEY));
    alerts.periodic();
    alerts.close();
    assertThat(shown()).isEmpty();
  }

  private static Alert cpu(Level level, int celsius) {
    return new Alert(
        CPU,
        level,
        "vision-front",
        "vision-front: CPU temperature above 80 °C, " + celsius + " °C");
  }

  /** The robot's Coprocessors alerts a dashboard shows now: level, key and text. */
  private static List<String> shown() {
    List<String> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.group.equals("Coprocessors") && alert.activeStartTime != 0) {
        String level = "LOW";
        if (alert.level == AlertDataJNI.LEVEL_HIGH) {
          level = "HIGH";
        } else if (alert.level == AlertDataJNI.LEVEL_MEDIUM) {
          level = "MEDIUM";
        }
        active.add(level + " " + alert.id + " " + alert.text);
      }
    }
    return active;
  }
}

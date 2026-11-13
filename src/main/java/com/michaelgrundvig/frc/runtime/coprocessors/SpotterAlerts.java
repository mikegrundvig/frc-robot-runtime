package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.spotter.manager.Level;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.wpilib.util.Alert;

/**
 * Spotter's alerts as the robot's own, in the group {@code Coprocessors}: a board missing, on
 * another protocol, with packs that differ or problems in them, each value of its packs at warning
 * or failing, and the robot's own Spotter setup (its packs folder, its key). One WPILib alert per
 * Spotter alert's key, high for failing, medium for a warning; WPILib's alert has a fixed level, so
 * one whose level changes is replaced. Each text names its board first.
 */
final class SpotterAlerts implements AutoCloseable {
  private final Supplier<List<com.michaelgrundvig.frc.spotter.manager.Alert>> alerts;
  private final Map<String, Alert> shown = new HashMap<>();
  private List<com.michaelgrundvig.frc.spotter.manager.Alert> mirrored = List.of();

  /**
   * @param alerts Spotter's current alerts, the same list until one changes ({@code
   *     Manager.alerts})
   */
  SpotterAlerts(Supplier<List<com.michaelgrundvig.frc.spotter.manager.Alert>> alerts) {
    this.alerts = alerts;
  }

  /** Once each loop, after Spotter's manager has updated. */
  void periodic() {
    List<com.michaelgrundvig.frc.spotter.manager.Alert> now = alerts.get();
    if (now == mirrored) {
      return;
    }
    mirrored = now;
    Set<String> standing = new HashSet<>();
    for (com.michaelgrundvig.frc.spotter.manager.Alert each : now) {
      standing.add(each.key());
      Alert.Level level = each.level() == Level.FAILING ? Alert.Level.HIGH : Alert.Level.MEDIUM;
      Alert alert = shown.get(each.key());
      if (alert != null && alert.getLevel() != level) {
        alert.close();
        alert = null;
      }
      if (alert == null) {
        alert = new Alert("Coprocessors", each.key(), each.text(), level);
        shown.put(each.key(), alert);
      }
      alert.setText(each.text());
      alert.set(true);
    }
    shown
        .entrySet()
        .removeIf(entry -> {
          if (standing.contains(entry.getKey())) {
            return false;
          }
          entry.getValue().close();
          return true;
        });
  }

  @Override
  public void close() {
    shown.values().forEach(Alert::close);
    shown.clear();
  }
}

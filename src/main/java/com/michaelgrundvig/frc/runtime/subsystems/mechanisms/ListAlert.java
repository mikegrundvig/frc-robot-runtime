package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import java.util.List;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;
import org.wpilib.util.Alert;

/**
 * An alert that names a list, raised while the list isn't empty: the sensors lost, the limit
 * switches doubted, the motor controllers drawing nothing. Its text changes only when the list
 * does, and it's made the first time the list has anything in it, so a robot that never needs it
 * shows none.
 */
final class ListAlert implements AutoCloseable {
  private final String group;
  private final String name;
  private final Alert.Level level;
  private final Function<List<String>, String> says;

  /** The list as the alert says it now. */
  private List<String> said = List.of();

  private @Nullable Alert alert;

  /**
   * @param group the alerts' group it's shown in, "Sensors"
   * @param name its name in the group
   * @param says its text for a list that isn't empty
   */
  ListAlert(String group, String name, Alert.Level level, Function<List<String>, String> says) {
    this.group = group;
    this.name = name;
    this.level = level;
    this.says = says;
  }

  /** This loop's list: the alert says it, or falls while it's empty. */
  void set(List<String> items) {
    if (items.equals(said)) {
      return;
    }
    said = List.copyOf(items);
    Alert raised = alert;
    if (raised == null) {
      raised = new Alert(group, name, says.apply(said), level);
      alert = raised;
    } else if (!said.isEmpty()) {
      raised.setText(says.apply(said));
    }
    raised.set(!said.isEmpty());
  }

  @Override
  public void close() {
    Alert raised = alert;
    if (raised != null) {
      raised.close();
    }
  }
}

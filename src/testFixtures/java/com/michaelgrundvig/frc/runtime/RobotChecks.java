package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import java.util.List;
import java.util.Map;

/** Checks a team's robots run against as the runtime's own: what must hold of any robot. */
public final class RobotChecks {
  private RobotChecks() {}

  /** Every live tuning a robot has, from the container's one list of them. */
  public static List<LiveConfig<?>> liveConfigs(RobotConfig robot) {
    return RobotContainer.tunings(robot).all();
  }

  /**
   * A robot's configuration works with live tuning, within the limits its mechanisms declare.
   * Declaring a mechanism's tuning fails if a committed value is outside its limits; setting each
   * value to what it already is rebuilds the configuration through its checks, and changes nothing.
   */
  public static void committedValuesCanBeDialedIn(RobotConfig robot) {
    for (LiveConfig<?> live : liveConfigs(robot)) {
      assertThat(live.knobs()).as(robot.name() + " " + live.name()).isNotEmpty();
      for (LiveConfig.Knob knob : live.knobs()) {
        assertThat(live.set(Map.of(knob.path(), live.committedValue(knob.path()))))
            .as(robot.name() + " " + live.name() + " " + knob.path())
            .isTrue();
      }
      assertThat(live.changes()).as(robot.name() + " " + live.name()).isEmpty();
    }
  }
}

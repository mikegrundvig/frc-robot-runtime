package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Rule;
import java.util.ArrayList;
import java.util.List;

/**
 * A named situation for the simulated robot: a fault in its sensors, or something that happens to
 * it, such as a collision. It's a named set of the simulated world's faults ({@link Faults}). The
 * simulator starts in one with {@code ./gradlew run -Pscenario=<name>}, the Pitwall page applies
 * one to a running simulation, and a test builds its robot in one ({@code RobotSim}). {@link
 * Scenarios} lists them.
 *
 * <p>A scenario changes only the simulated world. The robot's mechanisms and localization are never
 * told: they have to find out from the sensors, as on a real robot.
 *
 * <p>A scenario has two parts, either of which may be empty. Its {@link #sensorFaults} change how
 * the robot's sensors err, and {@code ./gradlew run} applies them before the robot's first loop, so
 * the robot starts with them. Its {@link #events} are what happens to the robot: {@code ./gradlew
 * run} makes them happen once the robot is enabled, and a test when it chooses. The page applies
 * both at once.
 *
 * @param name how it's asked for: lowercase words joined by hyphens, such as {@code camera-low}
 * @param description what it does to the robot
 * @param sensorFaults how the robot's sensors err in it; {@link #NO_FAULTS} for none
 * @param events what happens to the robot when it starts; {@link #NOTHING} for nothing
 */
@ConfigRecord
public record Scenario(String name, String description, Settings sensorFaults, Settings events) {
  /** The sensors err as they otherwise would. */
  public static final Settings NO_FAULTS = world -> List.of();

  /** Nothing happens when the scenario starts. */
  public static final Settings NOTHING = world -> List.of();

  /** Some of the simulated world's faults, by name, with their values. */
  @FunctionalInterface
  public interface Settings {
    /**
     * @param world the simulated world, for a scenario that knocks its first camera, say
     * @return the faults, on that world's parts
     * @throws IllegalArgumentException if the world lacks a part the scenario needs
     */
    List<Setting> in(SimulatedWorld world);
  }

  public Scenario {
    Checks.on(Scenario.class)
        .field("name", name, lowercaseWords())
        .field("description", description, notBlank())
        .orThrow();
  }

  /** Starts describing a scenario, one named value at a time. */
  public static ScenarioBuilder.NameStage builder() {
    return ScenarioBuilder.builder();
  }

  /** These faults, whatever the world. */
  public static Settings faults(Setting... faults) {
    List<Setting> all = List.of(faults);
    return world -> all;
  }

  /** Its sensor faults, on this world's parts. */
  public List<Setting> sensorFaultsIn(SimulatedWorld world) {
    return sensorFaults.in(world);
  }

  /** What happens to the robot in it, on this world's parts. */
  public List<Setting> eventsIn(SimulatedWorld world) {
    return events.in(world);
  }

  /** Everything it does, sensor faults first: what the page applies to a running simulation. */
  public List<Setting> faultsIn(SimulatedWorld world) {
    List<Setting> all = new ArrayList<>(sensorFaultsIn(world));
    all.addAll(eventsIn(world));
    return all;
  }

  /** Lowercase letters and digits, in words joined by single hyphens: typed on a command line. */
  private static Rule<String> lowercaseWords() {
    // A rule answers null for a value it accepts, and otherwise says what is wrong with it.
    return value -> {
      if (value.matches("[a-z0-9]+(-[a-z0-9]+)*")) {
        return null;
      }
      return "\"" + value + "\" must be lowercase words joined by hyphens, like camera-low";
    };
  }
}

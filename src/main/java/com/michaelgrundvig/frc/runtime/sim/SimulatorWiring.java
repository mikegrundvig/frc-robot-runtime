package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.hardware.Simulation;
import com.michaelgrundvig.frc.runtime.hardware.World;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Mass;

/**
 * The simulated world, wired for the robot program: the one factory its top level calls, in the
 * simulator's mode, for the world its robot drives in and the scenario that world starts in. The
 * robot program sees the result only as a {@link Simulation} and its {@link World}.
 */
public final class SimulatorWiring implements Simulation {
  private final SensorErrors errors;
  private final Optional<Scenario> scenario;

  /** The world made, once it is. */
  private @Nullable SimulatedWorld world;

  private SimulatorWiring(SensorErrors errors, Optional<Scenario> scenario) {
    this.errors = errors;
    this.scenario = scenario;
  }

  /**
   * The simulator, for a robot whose wheels and sensors err like this, starting in the scenario
   * asked for.
   *
   * @param errors how the robot's wheels, sensors and camera mounts differ from its configuration
   * @param scenario the scenario's name, if one was asked for ({@link Scenarios})
   * @throws IllegalArgumentException naming every scenario there is, when none has that name. A
   *     scenario is asked for by name, so a wrong name stops the program.
   */
  public static SimulatorWiring start(SensorErrors errors, Optional<String> scenario) {
    return new SimulatorWiring(errors, scenario.map(Scenarios::named));
  }

  /** The simulator, for a robot whose wheels and sensors err like this, in no scenario. */
  public static SimulatorWiring of(SensorErrors errors) {
    return start(errors, Optional.empty());
  }

  @Override
  public Optional<ScenarioAsked> scenario() {
    return scenario.map(asked -> new ScenarioAsked(asked.name(), asked.description()));
  }

  @Override
  public World world(FieldModel field, RobotDefinition definition, Mass robotMass) {
    if (world != null) {
      throw new IllegalStateException("The simulator makes its world once");
    }
    SimulatedWorld made = new SimulatedWorld(field, definition, robotMass, errors);
    world = made;
    return made;
  }

  /**
   * The world made, for a test to shove, tilt, or fail its parts.
   *
   * @throws IllegalStateException if none is made yet
   */
  public SimulatedWorld made() {
    SimulatedWorld made = world;
    if (made == null) {
      throw new IllegalStateException("The simulator has made no world yet");
    }
    return made;
  }

  @Override
  public List<Setting> scenarioFaults() {
    SimulatedWorld made = made();
    return scenario.map(asked -> asked.sensorFaultsIn(made)).orElse(List.of());
  }

  @Override
  public List<Setting> scenarioEvents() {
    SimulatedWorld made = made();
    return scenario.map(asked -> asked.eventsIn(made)).orElse(List.of());
  }
}

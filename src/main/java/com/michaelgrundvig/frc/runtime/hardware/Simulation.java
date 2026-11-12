package com.michaelgrundvig.frc.runtime.hardware;

import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.List;
import java.util.Optional;
import org.wpilib.units.measure.Mass;

/**
 * The simulator, as the robot program starts it: the world it makes for the robot, and the scenario
 * that world starts in, if one was asked for. Only the simulator's mode has one. The robot program
 * gets it from the simulator's one factory ({@code Robot.simulation}) and sees the simulator
 * through this and the {@link World} it makes, never through the simulator's classes.
 */
public interface Simulation {
  /**
   * A scenario asked for by name.
   *
   * @param name how it was asked for
   * @param description what it does to the robot
   */
  record ScenarioAsked(String name, String description) {}

  /** The scenario asked for, if one was. */
  Optional<ScenarioAsked> scenario();

  /**
   * Makes the world the robot drives in, once, as the robot is built.
   *
   * @param field the field, with its tags, its terrain and its elements
   * @param definition what the robot is, which each part of the world is simulated from
   * @param robotMass the robot's mass, which the drivetrain's physics reads
   * @throws IllegalStateException if it already made one
   */
  World world(FieldModel field, RobotDefinition definition, Mass robotMass);

  /**
   * The scenario's sensor faults, on the world made: applied before the robot's first loop. None
   * without a scenario.
   *
   * @throws IllegalStateException if no world is made yet
   */
  List<Setting> scenarioFaults();

  /**
   * What happens to the robot in the scenario, on the world made: once the robot is enabled.
   * Nothing without a scenario.
   *
   * @throws IllegalStateException if no world is made yet
   */
  List<Setting> scenarioEvents();
}

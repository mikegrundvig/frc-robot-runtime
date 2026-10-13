package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * The names behind the numbers in the log. What a joint did is logged as facts, and a fact names a
 * joint, an interlock, a sensor, a configuration, a position or an output by where it is in one of
 * the robot definition's lists. A person reading the raw log has no definition to look in, so each
 * of those lists is logged once here, in its own order, under {@code Robot/Names}: entry 2 of
 * {@code Robot/Names/Interlocks} is the interlock a holder calls interlock 2.
 *
 * <p>The lists never change while the robot runs, so they are logged in the robot's first loop and
 * not again.
 */
final class RecordedNames {
  private final RobotDefinition definition;

  /** Whether the lists have been logged. */
  private boolean logged;

  /**
   * @param definition the robot, whose lists the log's facts point into
   */
  RecordedNames(RobotDefinition definition) {
    this.definition = definition;
  }

  /** Logs the lists, the first time it is called. */
  void logOnce() {
    if (logged) {
      return;
    }
    logged = true;
    Logger.recordOutput("Robot/Names/Joints", names(definition.joints(), Joint::label));
    Logger.recordOutput(
        "Robot/Names/Interlocks", names(definition.everyInterlock(), Interlock::toString));
    Logger.recordOutput("Robot/Names/Sensors", names(definition.everySensor(), Sensor::label));
    Logger.recordOutput(
        "Robot/Names/Configurations", names(definition.configurations(), Configuration::name));
    for (Joint joint : definition.joints()) {
      Logger.recordOutput(
          "Robot/Names/Positions/" + joint.label(), names(joint.positions(), Position::name));
      Logger.recordOutput(
          "Robot/Names/Outputs/" + joint.label(), names(joint.outputs(), Output::name));
    }
  }

  /** The name of each thing in a list, in the list's order. */
  private static <T> String[] names(List<T> things, Function<T, String> name) {
    String[] names = new String[things.size()];
    for (int index = 0; index < names.length; index++) {
      names[index] = name.apply(things.get(index));
    }
    return names;
  }
}

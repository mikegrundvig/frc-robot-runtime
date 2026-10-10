package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.runtime.behavior.Bindings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;

/**
 * What the robot says at startup it doubts or doesn't know, in one place, so nobody has to run a
 * test to read it: allowed, but said once. What the robot's configuration refuses never gets here;
 * the robot doesn't start.
 *
 * <ul>
 *   <li>Its definition's own ({@code RobotDefinition.findings}): a named speed at its motors' free
 *       speed, a kG nobody tuned, a link not weighed.
 *   <li>Localization's ({@link LocalizationService#doubts}): treads said to grip so hard its slip
 *       check can't tell a slide from a collision.
 *   <li>Each driven joint's motor controller settings ({@link ControllerSettings#findings}): a
 *       tuned gain far from what its weight needs, a gravity offset at Phoenix's limit, an encoder
 *       wrapping near its range.
 *   <li>The configurations it can't run, naming a joint the template doesn't drive yet.
 *   <li>Its gamepads' bindings ({@link Bindings#doubts}): two commands bound to one button the same
 *       way, which start together.
 * </ul>
 *
 * <p>The robot's are logged ({@code Robot/Doubts}) wherever the program runs, and on a robot said
 * on the Driver Station's console; what only the simulator reads, a link's mass, is logged only
 * when it simulates ({@code Robot/SimulatorNotes}), never on the robot's console.
 */
final class Doubts {
  private Doubts() {}

  /** Every doubt and note the robot says at startup: its definition's, then its bindings'. */
  static List<Finding> of(RobotDefinition definition, Bindings bindings) {
    List<Finding> found = new ArrayList<>(of(definition));
    found.addAll(bindings.doubts());
    return found;
  }

  /** Every doubt and note its definition gives, the definition's own first. */
  static List<Finding> of(RobotDefinition definition) {
    List<Finding> found = new ArrayList<>(definition.findings());
    found.addAll(LocalizationService.doubts(definition));
    for (Joint joint : Joints.driven(definition)) {
      found.addAll(ControllerSettings.findings(definition.math(), joint));
    }
    for (Configuration configuration : definition.configurations()) {
      String cantRun = Joints.cantRun(definition, configuration);
      if (cantRun != null) {
        found.add(Finding.doubt(configuration.name(), cantRun));
      }
    }
    return found.stream().filter(finding -> !finding.refused()).toList();
  }

  /**
   * The lines said for what one scope reads, in order: a doubt as it is, what's not known after
   * "not known: ", and what only the simulator reads after "simulator: ".
   */
  static List<String> lines(List<Finding> findings, Finding.Scope scope) {
    List<String> lines = new ArrayList<>();
    for (Finding finding : findings) {
      if (finding.scope() == scope && !finding.refused()) {
        lines.add(line(finding));
      }
    }
    return lines;
  }

  /** One finding as it's said. */
  private static String line(Finding finding) {
    if (finding.scope() == Finding.Scope.SIMULATOR) {
      return "simulator: " + finding.message();
    }
    return finding.level() == Finding.Level.NOTE
        ? "not known: " + finding.message()
        : finding.message();
  }

  /**
   * Says them, once, at startup: the robot's in the log, and on a robot on the Driver Station's
   * console; the simulator's in the log when it simulates.
   */
  static void say(RobotDefinition definition, List<Finding> findings, RunMode mode) {
    List<String> robot = lines(findings, Finding.Scope.ROBOT);
    Logger.recordOutput("Robot/Doubts", robot.toArray(String[]::new));
    if (mode == RunMode.SIM) {
      Logger.recordOutput(
          "Robot/SimulatorNotes", lines(findings, Finding.Scope.SIMULATOR).toArray(String[]::new));
    }
    if (mode == RunMode.REAL) {
      for (String line : robot) {
        DriverStationErrors.reportWarning(definition.name() + ": " + line, false);
      }
    }
  }
}

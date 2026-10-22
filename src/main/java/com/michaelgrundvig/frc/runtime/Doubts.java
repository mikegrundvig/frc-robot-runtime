package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.localization.DefaultLocalizer;
import com.michaelgrundvig.frc.runtime.behavior.Bindings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.units.measure.Mass;

/**
 * What the robot says at startup that it doubts or does not know. All of it is allowed, and all of
 * it is said once, in one place, so that nobody has to run a test to read it. What a robot's
 * configuration refuses never gets here, because such a robot does not start.
 *
 * <p>The doubts come from five places:
 *
 * <ul>
 *   <li>The robot's definition ({@code RobotDefinition.findings}): a named speed at its motors'
 *       free speed, a kG nobody tuned, a link that was never weighed.
 *   <li>Localization ({@link DefaultLocalizer#doubts}): treads said to grip so hard that the slip
 *       check cannot tell a slide from a collision.
 *   <li>Each driven joint's motor controller settings ({@link ControllerSettings#findings}): a
 *       tuned gain far from what the joint's weight needs, a gravity offset at Phoenix's limit, an
 *       encoder that wraps near the joint's range.
 *   <li>The configurations the robot cannot run, each naming a joint that the template does not
 *       drive yet.
 *   <li>The gamepads' bindings ({@link Bindings#doubts}): two commands bound to one button in the
 *       same way, which start together.
 * </ul>
 *
 * <p>What the robot itself reads is logged ({@code Robot/Doubts}) wherever the program runs, and on
 * a real robot it is also said on the Driver Station's console. What only the simulator reads, such
 * as a link's mass, is logged only when the program simulates ({@code Robot/SimulatorNotes}), and
 * never reaches a robot's console.
 */
final class Doubts {
  private Doubts() {}

  /**
   * Every doubt and note the robot says at startup: its definition's, then its bindings'.
   *
   * @param robotMass the robot's mass: as weighed, or until it is, the season's maximum, which a
   *     note then names ({@code SeasonLimits.robotMass})
   */
  static List<Finding> of(RobotDefinition definition, Mass robotMass, Bindings bindings) {
    List<Finding> found = new ArrayList<>(of(definition, robotMass));
    found.addAll(bindings.doubts());
    return found;
  }

  /**
   * Every doubt and note that comes from the robot's definition, in the order of the list above.
   */
  static List<Finding> of(RobotDefinition definition, Mass robotMass) {
    List<Finding> found = new ArrayList<>(definition.findings(robotMass));
    found.addAll(DefaultLocalizer.doubts(definition));
    for (Joint joint : Joints.driven(definition)) {
      found.addAll(ControllerSettings.findings(definition.math(), joint));
    }
    for (Configuration configuration : definition.configurations()) {
      Optional<String> cannotRun = Optional.ofNullable(Joints.cantRun(definition, configuration));
      if (cannotRun.isPresent()) {
        found.add(Finding.doubt(configuration.name(), cannotRun.get()));
      }
    }
    return found.stream().filter(finding -> !finding.refused()).toList();
  }

  /**
   * The lines said about what one reader reads, in order.
   *
   * @param scope the reader: the robot itself, or only the simulator
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

  /**
   * One finding as it is said. A doubt is said as it is. What is not known is said after "not
   * known: ", and what only the simulator reads after "simulator: ".
   */
  private static String line(Finding finding) {
    if (finding.scope() == Finding.Scope.SIMULATOR) {
      return "simulator: " + finding.message();
    }
    if (finding.level() == Finding.Level.NOTE) {
      return "not known: " + finding.message();
    }
    return finding.message();
  }

  /**
   * Says the findings, once, at startup. The robot's go in the log, and on a real robot to the
   * Driver Station's console as well. The simulator's go in the log when the program simulates.
   */
  static void say(RobotDefinition definition, List<Finding> findings, RunMode mode) {
    List<String> robots = lines(findings, Finding.Scope.ROBOT);
    Logger.recordOutput("Robot/Doubts", robots.toArray(String[]::new));
    if (mode == RunMode.SIM) {
      List<String> simulators = lines(findings, Finding.Scope.SIMULATOR);
      Logger.recordOutput("Robot/SimulatorNotes", simulators.toArray(String[]::new));
    }
    if (mode == RunMode.REAL) {
      for (String line : robots) {
        DriverStationErrors.reportWarning(definition.name() + ": " + line, false);
      }
    }
  }
}

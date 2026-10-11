package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.wpilib.command3.Scheduler;

/**
 * What the robot program asks of a team's code. It is the shape of a team's {@code Controls}
 * constructor, so {@code Controls::new} is one. The program calls it once, after the mechanisms are
 * built, with everything that behavior reaches. A test makes its own controls the same way, and
 * gets back what it made.
 *
 * @param <T> what is made: a team's controls
 */
@FunctionalInterface
public interface ControlsMaker<T> {
  /** Controls that bind nothing, for a robot whose behavior a test supplies itself. */
  ControlsMaker<String> NONE =
      (scheduler, robot, joints, drive, driver, operator, autos) -> "no controls";

  /**
   * Builds the robot's behavior and binds it to the gamepads.
   *
   * @param scheduler the command scheduler, for gates of your own
   * @param robot the robot this runs on, its definition
   * @param joints every driven joint's mechanism, by its joint
   * @param drive the drivetrain, with its moves, assisted driving and wheel lock
   * @param driver the driver's gamepad, port 0
   * @param operator the operator's gamepad, port 1
   * @param autos the robot's autos: declare each with {@code autos.add(name, start, factory)}
   */
  T make(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos);
}

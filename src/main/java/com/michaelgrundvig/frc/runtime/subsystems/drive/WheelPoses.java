package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Where each wheel is on the robot and which way it points, for the log: the center of each wheel
 * in the robot's frame, turned to the angle its module measures. AdvantageScope turns the schematic
 * robot's wheels by these (its components 0 to 3, generated into ascope_assets/ from the robot's
 * configuration).
 */
final class WheelPoses {
  private final Module[] modules;

  /** Where each wheel touches the floor, from the robot's center. */
  private final Translation2d[] moduleTranslations;

  /** Each wheel's radius, as built: its center is this far above the floor. */
  private final double[] wheelRadiusMeters = new double[Module.COUNT];

  /** The wheels' poses as last worked out, handed out again loop after loop. */
  private final Pose3d[] poses = new Pose3d[Module.COUNT];

  /** Whether each wheel has a pose worked out yet. None has before the first loop. */
  private final boolean[] hasPose = new boolean[Module.COUNT];

  /** The steer angle each wheel's pose was worked out for, in radians. */
  private final double[] posedAtRadians = new double[Module.COUNT];

  WheelPoses(Module[] modules, RobotDefinition robot) {
    this.modules = modules;
    moduleTranslations = robot.moduleTranslations();
    for (int i = 0; i < Module.COUNT; i++) {
      wheelRadiusMeters[i] = robot.drivetrain().module(i).wheelRadius().in(Meters);
    }
  }

  /**
   * Each wheel's pose as of this loop: front left, front right, back left, back right. A wheel that
   * has not turned since its pose was last worked out keeps that pose, so in most loops nothing new
   * is made.
   */
  Pose3d[] now() {
    for (int i = 0; i < Module.COUNT; i++) {
      double steerRadians = modules[i].steerAngle().getRadians();
      boolean alreadyPosedThere = hasPose[i] && posedAtRadians[i] == steerRadians;
      if (alreadyPosedThere) {
        continue;
      }
      Translation2d at = moduleTranslations[i];
      poses[i] =
          new Pose3d(
              at.getX(), at.getY(), wheelRadiusMeters[i], new Rotation3d(0, 0, steerRadians));
      posedAtRadians[i] = steerRadians;
      hasPose[i] = true;
    }
    return poses;
  }
}

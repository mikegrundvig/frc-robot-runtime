package com.michaelgrundvig.frc.runtime.examples;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.localization.VisionConfigBuilder;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.BodyBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;

/**
 * Measured values written where they're set, each with when and how it was measured, as
 * docs/values.md shows them (DocExamplesTest): the template's robot's, with these changed.
 */
final class MeasuredValues {
  private MeasuredValues() {}

  static final DrivetrainGeometry.ModuleType MODULES =
      DrivetrainGeometryModuleTypeBuilder.builder(
              ExampleRobot.SIM.definition().drivetrain().corners().get(0).module())
          .wheelRadius(Inches.of(1.974)) // measured YYYY-MM-DD
          .build();

  static final Body BODY =
      BodyBuilder.builder(ExampleRobot.SIM.definition().body())
          .weight(
              Body.Weight.weighed(
                  Pounds.of(131.2))) // measured YYYY-MM-DD: two scales, bumpers and battery on
          .build();

  private static final VisionConfig VISION =
      VisionConfigBuilder.builder(VisionConfig.DEFAULTS)
          .linearStdDevBaseline(
              0.012) // characterized YYYY-MM-DD: camera noise, our camera at 1280 x 800
          .distanceExponent(2.1) // characterized YYYY-MM-DD: camera noise
          .build();

  /** The vision configuration with the camera's measured noise. */
  static VisionConfig vision() {
    return VISION;
  }
}

package com.michaelgrundvig.frc.runtime.examples;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import org.wpilib.math.geometry.Translation3d;

/**
 * A robot whose hood the template doesn't drive yet: two linear servos push it up, one each side.
 * Its block says everything about it but how it's driven, and {@link Hood}, a subsystem of the
 * team's own, drives it. docs/mechanisms.md, "When the template can't drive it", is this code
 * (DocExamplesTest).
 */
final class ServoHoodRobot {
  private ServoHoodRobot() {}

  static final Position HOOD_DOWN = Position.degrees("down", 0);
  static final Position HOOD_LOB = Position.degrees("lob", 30);

  /**
   * A WCP-0408, an Actuonix L16-R: 168 mm eye to eye closed, 100 mm of stroke, 50 N at most and 32
   * mm/s with nothing on it (WCP's guide, its 35:1 table).
   */
  static final Actuator.Servo WCP_0408 =
      Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();

  static final Joint.Revolute HOOD =
      Joint.pivot("hood", Part.CHASSIS, new Translation3d(-0.2, 0, 0.55), Joint.PITCH_UP)
          .range(Range.degrees(0, 40))
          .rest(HOOD_DOWN)
          .positions(HOOD_DOWN, HOOD_LOB)
          // Each servo's body on the chassis, from the hood's axle, and its rod's end on the hood.
          .servo(
              "hood left",
              WCP_0408,
              new Translation3d(0.054, 0.1, -0.233),
              new Translation3d(0.054, 0.1, 0))
          .servo(
              "hood right",
              WCP_0408,
              new Translation3d(0.054, -0.1, -0.233),
              new Translation3d(0.054, -0.1, 0)); // no .control(...): Hood drives it

  /** The template's robot, with this hood. */
  static final RobotConfig ROBOT =
      RobotConfigBuilder.builder(ExampleRobot.SIM)
          .definition(
              RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                  .name("servo hood")
                  .joints(List.of(HOOD))
                  .build())
          .build();
}

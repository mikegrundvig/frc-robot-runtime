package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Motor;
import org.wpilib.math.geometry.Translation3d;

/**
 * Valid pieces of a robot for the writer's tests, independent of any team's robots: the model's own
 * test robot, as its tests build it.
 */
final class TestRobot {
  private TestRobot() {}

  /** A 28 x 24 in frame with 3.5 in bumpers (a 35 x 31 in outline), 30 in tall. */
  static Body body() {
    return Body.frame(Inches.of(28), Inches.of(24), Inches.of(30))
        .bumpers(Inches.of(3.5), Inches.of(2.5), Inches.of(5.75))
        .weight(Kilograms.of(60))
        .inertia(
            Body.Turning.tensor(
                Inertia.builder().ixx(6).iyy(6).izz(6).ixy(0).ixz(0).iyz(0).build()))
        .cg(new Translation3d(0, 0, 0.2));
  }

  /** Kraken X60 drive at 6.75:1 on 2 in radius wheels, inset 2.625 in from the frame's edges. */
  static Drivetrain drivetrain() {
    Drivetrain.Module modules =
        Drivetrain.Module.builder()
            .driveMotor(Motor.KRAKEN_X60)
            .steerMotor(Motor.KRAKEN_X44)
            .wheelRadius(Inches.of(2))
            .driveGearing(Gearing.of(6.75))
            .steerGearing(Gearing.of(12.8))
            .couplingRatio(0)
            .wheelCoefficientOfFriction(1.2)
            .driveInertia(KilogramSquareMeters.of(0.001))
            .steerInertia(KilogramSquareMeters.of(0.004))
            .build();
    return Drivetrain.inset(modules, body(), Inches.of(2.625));
  }

  /** A level, forward-facing 1280 x 800 camera with an 80 degree field of view, here. */
  static CameraConfig camera(String name, Translation3d position) {
    CameraConfig.ImageSensor sensor =
        CameraConfig.ImageSensor.of(1280, 800, Hertz.of(30))
            .fieldOfView(Degrees.of(80), CameraConfig.Across.DIAGONAL)
            .latency(Milliseconds.of(30))
            .sightRange(Meters.of(7));
    return CameraConfig.of(name, sensor)
        .at(position.getMeasureX(), position.getMeasureY(), position.getMeasureZ())
        .facing(Degrees.of(0), Degrees.of(0))
        .roll(Degrees.of(0))
        .noiseScale(1.0);
  }
}

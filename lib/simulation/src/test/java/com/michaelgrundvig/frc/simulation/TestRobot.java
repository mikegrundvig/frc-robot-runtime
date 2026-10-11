package com.michaelgrundvig.frc.simulation;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Motor;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;

/**
 * A robot for the simulation's own tests, independent of any team's: a 27.5 in square frame of 68
 * kg on four Kraken X60 swerve modules at 6.75:1 on 2 in wheels, and motor controllers that drive
 * it the simplest way ({@link Controllers}).
 */
final class TestRobot {
  static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(27.5))
          .frameWidth(Inches.of(27.5))
          .height(Inches.of(30))
          .bumperEnds(Inches.of(3.5))
          .bumperSides(Inches.of(3.5))
          .bumperBottom(Inches.of(2.5))
          .bumperTop(Inches.of(5.75))
          .weight(Body.Weight.weighed(Kilograms.of(68)))
          .inertia(
              Body.Turning.tensor(
                  Inertia.builder().ixx(6.7).iyy(6.7).izz(6.7).ixy(0).ixz(0).iyz(0).build()))
          .cg(new Translation3d(0, 0, 0.15))
          .build();

  static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.inset(
          DrivetrainGeometry.ModuleType.builder()
              .driveMotor(Motor.KRAKEN_X60)
              .steerMotor(Motor.KRAKEN_X60)
              .wheelRadius(Inches.of(2))
              .driveGearing(Gearing.builder().ratio(6.75).efficiency(1.0).build())
              .steerGearing(Gearing.builder().ratio(150.0 / 7).efficiency(1.0).build())
              .couplingRatio(0)
              .wheelCoefficientOfFriction(1.2)
              .driveInertia(KilogramSquareMeters.of(0.001))
              .steerInertia(KilogramSquareMeters.of(0.004))
              .build(),
          BODY,
          Inches.of(2.625));

  static final double RADIUS = DRIVETRAIN.module(0).wheelRadius().in(Meters);
  static final DCMotor MOTOR = DRIVETRAIN.module(0).driveMotor().model();
  static final double GEARING = DRIVETRAIN.module(0).driveGearing().ratio();

  private TestRobot() {}

  /** Today's physics for the robot, its treads as configured. */
  static KinematicPhysics physics() {
    return new KinematicPhysics(BODY, DRIVETRAIN, List.of(1.0, 1.0, 1.0, 1.0));
  }

  /** Where each wheel sits, from the robot's center. */
  static Translation2d[] wheels() {
    return DRIVETRAIN.moduleTranslations();
  }

  /**
   * Motor controllers for the tests: every drive motor asked for one voltage, within the battery's
   * (12 V unless sagged), under one stator current limit, which the physics holds; each wheel
   * steered where told, at once.
   */
  static final class Controllers implements Actuators {
    double volts;
    double limitAmps = 30;
    double batteryVolts = 12;
    final double[] steer = new double[4];

    /** Every wheel steered this way. */
    Controllers steer(double radians) {
      java.util.Arrays.fill(steer, radians);
      return this;
    }

    @Override
    public void actuate(double seconds, Physics physics, WheelDrive[] drives) {
      double asked = Math.clamp(volts, -batteryVolts, batteryVolts);
      for (int i = 0; i < drives.length; i++) {
        drives[i].drive(asked, limitAmps, batteryVolts);
        drives[i].steer(steer[i]);
      }
    }
  }
}

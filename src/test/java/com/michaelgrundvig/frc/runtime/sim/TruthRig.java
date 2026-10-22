package com.michaelgrundvig.frc.runtime.sim;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryCornerBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.ImusBuilder;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Mass;

/**
 * The template's simulated drivetrain on its own, its motor controllers commanded directly, for the
 * truth suite: what the simulator does, measured against what physics says it should. It runs whole
 * loops, as the robot does, each five of the simulator's steps.
 */
final class TruthRig {
  /** A fifth of a loop: the simulator's step, and its odometry's rate. */
  static final double STEP_SECONDS = RobotLoop.PERIOD_SECONDS / DriveSimulation.STEPS_PER_LOOP;

  final Body body;

  /** The robot's mass, as the program on the season's field takes it. */
  final Mass robotMass;

  final DrivetrainGeometry drivetrain;
  final DriveSimulation simulation;
  final ModuleIO[] modules;
  final Translation2d[] wheels;
  private final DrivetrainControl control = ExampleRobot.SIM.drivetrain();
  private long nanos;

  /** The template's robot, its sensors exact. */
  TruthRig() {
    this(ExampleRobot.SIM.definition().drivetrain().wheelCoefficientOfFriction());
  }

  /** The template's robot on a tread with this grip, its sensors exact. */
  TruthRig(double frictionCoefficient) {
    this(ExampleRobot.SIM.definition().body(), frictionCoefficient, SensorErrors.NONE);
  }

  TruthRig(Body body, double frictionCoefficient, SensorErrors errors) {
    this(body, frictionCoefficient, errors, ExampleRobot.SIM.definition().imus().gyro());
  }

  /** As above, its gyro mounted here. */
  TruthRig(Body body, double frictionCoefficient, SensorErrors errors, ImuMount gyro) {
    this.body = body;
    this.robotMass = Rebuilt.GAME.limits().robotMass(body);
    DrivetrainGeometry template = ExampleRobot.SIM.definition().drivetrain();
    List<DrivetrainGeometry.Corner> corners = new ArrayList<>();
    for (DrivetrainGeometry.Corner corner : template.corners()) {
      corners.add(
          DrivetrainGeometryCornerBuilder.builder(corner)
              .module(
                  DrivetrainGeometryModuleTypeBuilder.builder(corner.module())
                      .wheelCoefficientOfFriction(frictionCoefficient)
                      .build())
              .build());
    }
    drivetrain = DrivetrainGeometryBuilder.builder(template).corners(corners).build();
    simulation =
        new DriveSimulation(
            body,
            robotMass,
            drivetrain,
            errors,
            ImusBuilder.builder(ExampleRobot.SIM.definition().imus()).gyro(gyro).build());
    modules = simulation.modules();
    wheels = drivetrain.moduleTranslations();
    limitDrives(control.limits().driveStatorCurrent().in(Amps));
  }

  /** Every drive motor's stator current limit, from now on. */
  void limitDrives(double amps) {
    DrivetrainControl.Limits limits =
        DrivetrainControlLimitsBuilder.builder(control.limits())
            .driveStatorCurrent(Amps.of(amps))
            .build();
    for (ModuleIO module : modules) {
      module.configure(control.gains(), limits);
    }
  }

  /** Every drive motor asked for this voltage, open loop. */
  void driveVolts(double volts) {
    for (int i = 0; i < modules.length; i++) {
      simulation.moduleSim(i).setDriveVoltage(volts);
    }
  }

  /** Every module steered to this angle, in the robot's frame. */
  void steer(Rotation2d angle) {
    for (ModuleIO module : modules) {
      module.setSteerAngle(angle);
    }
  }

  /**
   * Each module steered across its own arm from the center: spinning the robot counterclockwise.
   */
  void steerTangent() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setSteerAngle(wheels[i].getAngle().orElseThrow().plus(Rotation2d.CCW_90DEG));
    }
  }

  /** Runs whole loops for about this long. */
  void run(double seconds) {
    loops(RobotLoop.loops(seconds));
  }

  /** Runs this many loops, each five of the simulator's steps. */
  void loops(int count) {
    for (int i = 0; i < count; i++) {
      nanos += Math.round(RobotLoop.PERIOD_SECONDS * 1e9);
      simulation.step(RobotLoop.PERIOD_SECONDS, nanos);
    }
  }

  /** The robot's velocity on the field over the last step, in m/s. */
  Translation2d velocity() {
    return simulation.pointVelocity(Translation2d.ZERO);
  }

  double mass() {
    return robotMass.in(Kilograms);
  }

  double wheelRadius() {
    return drivetrain.module(0).wheelRadius().in(Meters);
  }

  double wheelInertia() {
    return drivetrain.module(0).driveInertiaOrDefault().in(KilogramSquareMeters);
  }

  double gearing() {
    return drivetrain.module(0).driveGearing().ratio();
  }

  DCMotor driveMotor() {
    return drivetrain.module(0).driveMotor().model();
  }

  /** A drive motor's push on the floor at this current, through its gearing, in newtons. */
  double force(double amps) {
    return driveMotor().Kt * amps * gearing() / wheelRadius();
  }

  /** The robot's mass with each wheel's own moving parts as mass at its tread, in kilograms. */
  double effectiveMass() {
    return mass() + 4 * wheelInertia() / (wheelRadius() * wheelRadius());
  }

  /** How far each module sits from the robot's center, in meters (all alike on a square robot). */
  double arm() {
    return wheels[0].getNorm();
  }
}

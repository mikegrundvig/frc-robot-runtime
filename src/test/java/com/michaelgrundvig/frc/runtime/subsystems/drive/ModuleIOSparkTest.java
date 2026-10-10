package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;

/**
 * A MAXSwerve module on REV's simulated SPARKs: what it reads, in the template's units, and what it
 * tells the SPARKs, in theirs.
 */
@ExtendWith(RobotSim.class)
class ModuleIOSparkTest {
  /** REV's MAXSwerve: a Vortex driving through the template's gearing, a NEO 550 steering. */
  private static final DrivetrainGeometry.ModuleType MAX_SWERVE =
      DrivetrainGeometryModuleTypeBuilder.builder(
              ExampleRobot.SIM.definition().drivetrain().corners().get(0).module())
          .driveMotor(Motor.NEO_VORTEX)
          .steerMotor(Motor.NEO_550)
          .couplingRatio(0)
          .build();

  private static ModuleIOSpark module(int drive, int steer, double offsetTurns) {
    return new ModuleIOSpark(
        SwerveWiring.ModuleWiring.builder()
            .driveId(drive)
            .steerId(steer)
            .encoderOffset(Rotations.of(offsetTurns))
            .driveInverted(false)
            .steerInverted(true)
            .encoderInverted(true)
            .build(),
        CANPort.CAN_S0,
        MechanismWiring.Controller.SPARK_FLEX,
        MechanismWiring.Controller.SPARK_MAX,
        MAX_SWERVE);
  }

  /**
   * The wheel turns the drive's rotor turns through its gearing, and the module points where its
   * absolute encoder reads, with its offset: 0.3 of a turn read, less 0.1, is 0.2 of a turn.
   */
  @Test
  void itReadsTheWheelThroughItsGearingAndTheAngleWithItsOffset() {
    ModuleIOSpark io = module(11, 12, -0.1);
    double gearing = MAX_SWERVE.driveGearing().ratio();
    SparkSim drive = new SparkSim(io.drive(), DCMotor.getNeoVortex(1));
    drive.setPosition(2 * gearing);
    // REV's simulation hands a position on to its encoder as it steps.
    drive.iterate(0, 12, 0.02);
    new SparkAbsoluteEncoderSim((SparkMax) io.steer()).setPosition(0.3);
    ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();

    io.updateInputs(inputs);

    assertThat(inputs.drivePositionRad)
        .as("two turns of the wheel")
        .isCloseTo(4 * Math.PI, within(1e-4));
    assertThat(inputs.steerAngle.getRotations()).isCloseTo(0.2, within(1e-6));
    assertThat(inputs.odometryTimestampsNanos).hasSize(1);
    assertThat(inputs.odometrySteerAnglesRad[0]).isCloseTo(0.4 * Math.PI, within(1e-5));
  }

  /**
   * An angle goes to the steering SPARK in its encoder's own reading, the offset taken back off,
   * from 0 to 1; a wheel speed to the drive SPARK in rotor RPM, with the robot's feedforward.
   */
  @Test
  void itAsksInTheSparksUnits() {
    ModuleIO io = module(13, 14, 0.25);
    ModuleIOSpark spark = (ModuleIOSpark) io;
    SparkSim steer = new SparkSim(spark.steer(), DCMotor.getNeo550(1));
    SparkSim drive = new SparkSim(spark.drive(), DCMotor.getNeoVortex(1));
    steer.enable();
    drive.enable();

    io.setSteerAngle(Rotation2d.fromRotations(0.1));
    io.setDriveVelocity(2 * Math.PI, 1.5);
    steer.iterate(0, 12, 0.02);
    drive.iterate(0, 12, 0.02);

    assertThat(steer.getSetpoint())
        .as("0.1 less 0.25, round to 0.85")
        .isCloseTo(0.85, within(1e-6));
    assertThat(drive.getSetpoint())
        .as("a turn of the wheel a second")
        .isCloseTo(MAX_SWERVE.driveGearing().ratio() * 60, within(1e-3));
  }

  /** The template's gains reach the SPARKs' loops as shares of 12 V, in their units. */
  @Test
  void itsGainsReachTheLoopsInTheirUnits() {
    ModuleIOSpark io = module(15, 16, 0);
    DrivetrainControl control = ExampleRobot.SIM.drivetrain();

    io.configure(control.gains(), control.limits());

    double wheelRadius = MAX_SWERVE.wheelRadius().in(org.wpilib.units.Units.Meters);
    double gearing = MAX_SWERVE.driveGearing().ratio();
    double driveKp = control.gains().driveKp() * 2 * Math.PI * wheelRadius / gearing / 60 / 12;
    double steerKp = control.gains().steerKp() * 2 * Math.PI / 12;
    assertThat(((SparkFlex) io.drive()).configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .isCloseTo(driveKp, within(1e-6 * driveKp + 1e-9));
    assertThat(((SparkMax) io.steer()).configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .isCloseTo(steerKp, within(1e-6 * steerKp));
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import com.revrobotics.spark.SparkSim;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import org.wpilib.hardware.bus.CANPort;

/**
 * The module conformance suite's bench for the REV module ({@link ModuleIOSpark}), against REV's
 * own simulated SPARKs: a SPARK Flex driving through a NEO Vortex and a SPARK MAX steering a NEO
 * 550, its absolute encoder on the steering SPARK's data port. As on the joints' REV bench, the
 * simulation runs when it is told to, every power-on is on SPARKs at CAN IDs not yet used, and the
 * steering SPARK's one simulated position is what its encoder reads.
 */
public final class SparkModuleBench implements ModuleBench {
  /** The battery, in volts. */
  private static final double BATTERY_VOLTS = 12;

  private static final double SECONDS_PER_MINUTE = 60;

  /** The first CAN ID not yet used on the bench's port, for the life of the JVM. */
  private static final AtomicInteger NEXT_ID = new AtomicInteger(1);

  /** REV's MAXSwerve: a Vortex driving through the template's gearing, a NEO 550 steering. */
  private static final DrivetrainGeometry.ModuleType MODULE =
      DrivetrainGeometryModuleTypeBuilder.builder(
              ExampleRobot.SIM.definition().drivetrain().corners().get(0).module())
          .driveMotor(Motor.NEO_VORTEX)
          .steerMotor(Motor.NEO_550)
          .couplingRatio(0)
          .build();

  @Override
  public String name() {
    return "a SPARK module";
  }

  @Override
  public double wheelRadiusMeters() {
    return MODULE.wheelRadius().in(Meters);
  }

  @Override
  public Rig powerOn(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    return new SparkRig(gains, limits, NEXT_ID.getAndAdd(2));
  }

  @Override
  public Optional<String> lacks(Ability ability) {
    return Optional.empty();
  }

  @Override
  public Optional<String> knownDifference(ModuleClause clause) {
    return Optional.empty();
  }

  /** One REV module, powered on. */
  private static final class SparkRig implements Rig {
    private final ModuleIOSpark io;
    private final SparkSim drive;
    private final SparkSim steer;
    private final double driveGearing = MODULE.driveGearing().ratio();
    private double wheelRotations;
    private double wheelRotationsPerSecond;
    private double moduleRotations;

    SparkRig(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits, int firstId) {
      RobotSim.setEnabled(true);
      SwerveWiring.ModuleWiring wiring =
          SwerveWiring.ModuleWiring.builder()
              .driveId(firstId)
              .steerId(firstId + 1)
              .encoderOffset(Rotations.of(0))
              .driveInverted(false)
              .steerInverted(false)
              .encoderInverted(false)
              .build();
      io =
          new ModuleIOSpark(
              wiring,
              CANPort.CAN_S4,
              MechanismWiring.Controller.SPARK_FLEX,
              MechanismWiring.Controller.SPARK_MAX,
              MODULE);
      io.configure(gains, limits);
      drive = new SparkSim(io.drive(), MODULE.driveMotor().model());
      steer = new SparkSim(io.steer(), MODULE.steerMotor().model());
      step();
    }

    @Override
    public ModuleIO io() {
      return io;
    }

    /**
     * Runs both simulated SPARKs for a loop, the wheel ending it where the suite has it. The
     * steering SPARK's simulated position is its encoder's reading, a share of a turn from 0 to 1.
     */
    @Override
    public void step() {
      double rpm = wheelRotationsPerSecond * driveGearing * SECONDS_PER_MINUTE;
      double turnsThisLoop = rpm / SECONDS_PER_MINUTE * RobotLoop.PERIOD_SECONDS;
      drive.setPosition(wheelRotations * driveGearing - turnsThisLoop);
      drive.iterate(rpm, BATTERY_VOLTS, RobotLoop.PERIOD_SECONDS);
      double encoderReads = moduleRotations - Math.floor(moduleRotations);
      steer.setPosition(encoderReads);
      steer.getAbsoluteEncoderSim().setPosition(encoderReads);
      steer.iterate(0, BATTERY_VOLTS, RobotLoop.PERIOD_SECONDS);
      RobotSim.stepClock();
    }

    @Override
    public void wheelIs(double radians, double radiansPerSecond) {
      wheelRotations = radians / (2 * Math.PI);
      wheelRotationsPerSecond = radiansPerSecond / (2 * Math.PI);
    }

    @Override
    public void moduleIs(double radians) {
      moduleRotations = radians / (2 * Math.PI);
    }

    @Override
    public double driveVolts() {
      return drive.getAppliedOutput() * drive.getBusVoltage();
    }

    @Override
    public double steerVolts() {
      return steer.getAppliedOutput() * steer.getBusVoltage();
    }

    /** Leaves both SPARKs stopped. They stay open: REV's simulation cannot open one twice. */
    @Override
    public void close() {
      io.stop();
    }
  }
}

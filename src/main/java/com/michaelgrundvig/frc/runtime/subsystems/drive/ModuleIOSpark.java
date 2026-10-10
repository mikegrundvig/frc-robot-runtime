package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.util.Signal;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.RobotController;

/**
 * One swerve module on REV's SPARKs, as a MAXSwerve is: a SPARK MAX or SPARK Flex driving the
 * wheel, another turning it, and the module's absolute encoder on the steering SPARK's data port,
 * which its loop closes on, wrapping, so the shorter way round is taken.
 *
 * <p>Gains come in the template's units, as the TalonFX module's do, and go to the SPARKs' loops as
 * a share of 12 V: the drive's kP per rotor RPM of error, the steering's per turn of the module.
 * The robot's feedforward goes with the drive's speed as volts; the steering's damping is worked
 * out on the robot, from its measured speed, and sent as volts too. A SPARK limits its motor's
 * current, not the battery's, so the drive's supply limits aren't applied.
 *
 * <p>Odometry is read once a loop, not at the TalonFX modules' 250 Hz: each loop gives one reading.
 * The absolute encoder's offset and direction are applied here, not on the SPARK, so they mean what
 * the wiring says ({@link SwerveWiring.ModuleWiring}) whatever REV's own conventions.
 */
final class ModuleIOSpark implements ModuleIO {
  private static final double TWO_PI = 2 * Math.PI;

  /** At startup, configuring a SPARK is tried this many times. */
  private static final int TRIES = 5;

  private static final int SIGNAL_MS = 20;

  private final SparkBase drive;
  private final SparkBase steer;
  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder steerEncoder;
  private final SparkClosedLoopController driveLoop;
  private final SparkClosedLoopController steerLoop;
  private final MechanismWiring.Controller driveController;
  private final MechanismWiring.Controller steerController;
  private final double driveGearing;
  private final double couplingRatio;
  private final double wheelRadiusMeters;

  /** Added to the absolute encoder's reading, in turns, so it reads zero pointing ahead. */
  private final double encoderOffset;

  private boolean configured;

  /** The steering's damping, volts per radian per second of its turning. */
  private double steerKd;

  /** The SPARKs that didn't confirm their configuration at startup. */
  private final List<String> unconfigured = new ArrayList<>();

  private final Debouncer driveConnected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer steerConnected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  /** The module's turning speed as last read, in rotations per second, for the coupling. */
  private double steerRotationsPerSecond;

  /**
   * @param wiring the module's SPARKs and its encoder's offset and directions
   * @param port the CAN port the SPARKs are on
   * @param driveController the drive's SPARK, a MAX or a Flex
   * @param steerController the steering's
   * @param module the module's gearing and wheel
   */
  ModuleIOSpark(
      SwerveWiring.ModuleWiring wiring,
      CANPort port,
      MechanismWiring.Controller driveController,
      MechanismWiring.Controller steerController,
      DrivetrainGeometry.ModuleType module) {
    this.driveController = driveController;
    this.steerController = steerController;
    drive = spark(driveController, port, wiring.driveId());
    steer = spark(steerController, port, wiring.steerId());
    driveGearing = module.driveGearing().ratio();
    couplingRatio = module.couplingRatio();
    wheelRadiusMeters = module.wheelRadius().in(org.wpilib.units.Units.Meters);
    encoderOffset = wiring.encoderOffset().in(Rotations);

    SparkBaseConfig driveConfig = blank(driveController);
    driveConfig
        .inverted(wiring.driveInverted())
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .voltageCompensation(Motor.NOMINAL_VOLTS);
    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    driveConfig
        .signals
        .primaryEncoderPositionPeriodMs(SIGNAL_MS)
        .primaryEncoderVelocityPeriodMs(SIGNAL_MS)
        .appliedOutputPeriodMs(SIGNAL_MS)
        .outputCurrentPeriodMs(SIGNAL_MS)
        .busVoltagePeriodMs(SIGNAL_MS);
    SparkBaseConfig steerConfig = blank(steerController);
    steerConfig
        .inverted(wiring.steerInverted())
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .voltageCompensation(Motor.NOMINAL_VOLTS);
    if (steerController == MechanismWiring.Controller.SPARK_MAX) {
      steerConfig.absoluteEncoder.setSparkMaxDataPortConfig();
    }
    steerConfig.absoluteEncoder.inverted(wiring.encoderInverted());
    steerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true);
    steerConfig
        .signals
        .absoluteEncoderPositionPeriodMs(SIGNAL_MS)
        .absoluteEncoderVelocityPeriodMs(SIGNAL_MS)
        .appliedOutputPeriodMs(SIGNAL_MS)
        .outputCurrentPeriodMs(SIGNAL_MS);
    // Saved on the SPARKs, so a brownout doesn't lose them; the gains come in configure().
    if (!applied(
        () ->
            drive.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))) {
      unconfigured.add("drive motor " + wiring.driveId());
    }
    if (!applied(
        () ->
            steer.configure(
                steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))) {
      unconfigured.add("steering motor " + wiring.steerId());
    }
    driveEncoder = drive.getEncoder();
    steerEncoder = steer.getAbsoluteEncoder();
    driveLoop = drive.getClosedLoopController();
    steerLoop = steer.getClosedLoopController();
  }

  private static SparkBase spark(MechanismWiring.Controller controller, CANPort port, int id) {
    return controller == MechanismWiring.Controller.SPARK_FLEX
        ? new SparkFlex(port, id, SparkLowLevel.MotorType.kBrushless)
        : new SparkMax(port, id, SparkLowLevel.MotorType.kBrushless);
  }

  /** An empty configuration for a SPARK: only what's set on it is applied. */
  private static SparkBaseConfig blank(MechanismWiring.Controller controller) {
    return controller == MechanismWiring.Controller.SPARK_FLEX
        ? new SparkFlexConfig()
        : new SparkMaxConfig();
  }

  private static boolean applied(Supplier<REVLibError> attempt) {
    for (int i = 0; i < TRIES; i++) {
      if (attempt.get() == REVLibError.kOk) {
        return true;
      }
    }
    return false;
  }

  /** The steering SPARK: for the tests, which read what it was told through REV's simulation. */
  SparkBase steer() {
    return steer;
  }

  /** The drive SPARK, likewise. */
  SparkBase drive() {
    return drive;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    Signal<Double> rotor = driveEncoder.getPosition();
    Signal<Double> rpm = driveEncoder.getVelocity();
    Signal<Double> angle = steerEncoder.getPosition();
    Signal<Double> turning = steerEncoder.getVelocity();
    double moduleRotations = angle.get(0.0) + encoderOffset;
    // REV's absolute encoder reads its speed in turns a minute.
    steerRotationsPerSecond = turning.get(0.0) / 60;

    inputs.driveConnected = driveConnected.calculate(rotor.isValid() && rpm.isValid());
    double wheelRotations =
        ModuleIOTalonFX.wheelRotations(
            rotor.get(0.0) / driveGearing, moduleRotations, couplingRatio, driveGearing);
    inputs.drivePositionRad = TWO_PI * wheelRotations;
    inputs.driveVelocityRadPerSec =
        TWO_PI
            * ModuleIOTalonFX.wheelRotations(
                rpm.get(0.0) / 60 / driveGearing,
                steerRotationsPerSecond,
                couplingRatio,
                driveGearing);
    inputs.driveAppliedVolts = drive.getAppliedOutput().get(0.0) * drive.getBusVoltage().get(0.0);
    double driveAmps = Math.abs(drive.getOutputCurrent().get(0.0));
    inputs.driveStatorCurrentAmps = driveAmps;
    // A SPARK doesn't measure what it draws from the battery: its share of the motor's current.
    inputs.driveSupplyCurrentAmps = Math.abs(drive.getAppliedOutput().get(0.0)) * driveAmps;
    inputs.driveTorqueLimited = false;

    inputs.steerConnected = steerConnected.calculate(angle.isValid());
    inputs.steerAngle = Rotation2d.fromRotations(moduleRotations);
    inputs.steerVelocityRadPerSec = TWO_PI * steerRotationsPerSecond;
    inputs.steerAppliedVolts = steer.getAppliedOutput().get(0.0) * steer.getBusVoltage().get(0.0);
    inputs.steerStatorCurrentAmps = Math.abs(steer.getOutputCurrent().get(0.0));

    // One reading a loop: SPARKs aren't read at the TalonFX modules' 250 Hz.
    inputs.odometryTimestampsNanos = new long[] {RobotController.getTime()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometrySteerAnglesRad = new double[] {TWO_PI * moduleRotations};
    inputs.unconfigured = String.join(", ", unconfigured);
  }

  /**
   * Gains and limits, at startup and whenever tuning changes them: saved at startup, sent without
   * waiting after, so the loop never blocks.
   */
  @Override
  public void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    // Volts per wheel turn a second, per rotor RPM, as a share of 12 V.
    double driveKp = gains.driveKp() * TWO_PI * wheelRadiusMeters / driveGearing / 60;
    // Volts per turn of the module, read by the absolute encoder on its shaft, as a share.
    double steerKp = gains.steerKp() * TWO_PI;
    steerKd = gains.steerKd();
    SparkBaseConfig driveConfig = blank(driveController);
    driveConfig.smartCurrentLimit((int) Math.round(limits.driveStatorCurrent().in(Amps)));
    driveConfig.closedLoop.p(driveKp / Motor.NOMINAL_VOLTS, ClosedLoopSlot.kSlot0);
    SparkBaseConfig steerConfig = blank(steerController);
    steerConfig.smartCurrentLimit((int) Math.round(limits.steerStatorCurrent().in(Amps)));
    steerConfig.closedLoop.p(steerKp / Motor.NOMINAL_VOLTS, ClosedLoopSlot.kSlot0);
    if (!configured) {
      if (!applied(
          () ->
              drive.configure(
                  driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters))) {
        unconfigured.add("drive motor's gains");
      }
      if (!applied(
          () ->
              steer.configure(
                  steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters))) {
        unconfigured.add("steering motor's gains");
      }
      configured = true;
      return;
    }
    drive.configureAsync(
        driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    steer.configureAsync(
        steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
    // The drive's reading includes the coupling, so aim for the wheel's speed plus it.
    double wheel = wheelRadPerSec / TWO_PI + couplingRatio * steerRotationsPerSecond / driveGearing;
    driveLoop.setSetpoint(
        wheel * driveGearing * 60,
        SparkLowLevel.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setSteerAngle(Rotation2d angle) {
    // In the encoder's own reading, its offset taken back off, from 0 to 1.
    double reading = angle.getRotations() - encoderOffset;
    reading -= Math.floor(reading);
    steerLoop.setSetpoint(
        reading,
        SparkLowLevel.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        -steerKd * TWO_PI * steerRotationsPerSecond,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    drive.stopMotor();
    steer.stopMotor();
  }
}

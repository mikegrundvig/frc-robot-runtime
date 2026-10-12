package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.behavior.Debounce;
import com.michaelgrundvig.frc.runtime.lib.rev.RevRestarts;
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
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.RobotController;

/**
 * One swerve module on REV's SPARKs, as a MAXSwerve is: a SPARK MAX or SPARK Flex driving the
 * wheel, another turning the module, and the module's absolute encoder on the steering SPARK's data
 * port. The steering's loop runs on that encoder and wraps, so it turns the short way round to an
 * angle.
 *
 * <p><b>Units.</b> A SPARK reads its motor in the motor's rotations, and speeds in rotations a
 * minute (RPM), with no conversion on the device. So the gearing is applied here, each way.
 *
 * <p><b>Gains</b> come in the template's units, as the TalonFX module's do, and go to the SPARKs'
 * loops as a share of 12 V: the drive's kP per RPM of its motor, the steering's per turn of the
 * module. The robot's feedforward goes with the drive's speed, as volts. The steering's damping
 * (kD) is worked out on the robot, from the module's measured turning speed, and sent as volts too.
 *
 * <p><b>Current.</b> A SPARK limits the current through its motor, not what it draws from the
 * battery, so the drive's supply limits are not applied.
 *
 * <p><b>Odometry</b> is read once a loop, not at the TalonFX modules' 250 Hz: each loop gives one
 * reading.
 *
 * <p><b>Restarts.</b> A SPARK that loses power and comes back says so in its sticky warnings
 * ({@link RevRestarts}). It comes back with the settings saved on it: what the wiring decides, and
 * the gains and limits it was first sent. Whatever was tuned since is not saved, since it belongs
 * to the session, so a restarted SPARK is sent it again ({@link #sendTuningAgainAfterARestart}).
 * The steering needs telling nothing else, because its angle is its absolute encoder's. The drive
 * comes back counting its motor from 0, and nothing here corrects for that.
 *
 * <p><b>A frame that does not come</b> leaves the module's last reading standing, so the wheel
 * reads as not having moved for that loop and not as back at 0 ({@link #takeTheFramesThatCame}).
 *
 * <p><b>The absolute encoder's offset</b> is applied here, not on the SPARK, so it means what the
 * wiring says ({@link SwerveWiring.ModuleWiring}) whatever REV's own conventions. Its direction is
 * set on the SPARK, because the SPARK's loop runs on the reading.
 */
final class ModuleIOSpark implements ModuleIO {
  private static final double TWO_PI = 2 * Math.PI;

  /** A SPARK's speeds are in rotations a minute. */
  private static final double SECONDS_PER_MINUTE = 60;

  /** At startup, configuring a SPARK is tried this many times. */
  private static final int TRIES = 5;

  /**
   * How often a SPARK sends each signal this class reads, in milliseconds: 20 ms, the robot loop's
   * period. Sending faster would load the CAN bus for readings nothing uses.
   */
  private static final int SIGNAL_MS = 20;

  /**
   * How long a SPARK may go without answering before it is called disconnected. One missed frame is
   * a glitch, not a lost device.
   *
   * <p>The half second is kept in whole nanoseconds ({@link Debounce}), not by WPILib's debouncer,
   * whose seconds in a double put the edge a loop early or late depending on what the clock reads.
   * So every driver, and the simulator, calls a device silent in the same loop.
   */
  private static final double SILENT_AFTER_SECONDS = 0.5;

  /**
   * Each SPARK is asked whether it restarted once in this many loops, ten times a second. Its
   * warning stays set until it is cleared, so no restart is missed, and one is acted on within a
   * tenth of a second. Asking every loop would cost more than it is worth: REV makes about 64 bytes
   * of objects for each SPARK each time it is asked (measured on REVLib 2027.0.0-alpha-8).
   */
  static final int RESTART_LOOPS = 5;

  private final SparkBase drive;
  private final SparkBase steer;
  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder steerEncoder;
  private final SparkClosedLoopController driveLoop;
  private final SparkClosedLoopController steerLoop;
  private final MechanismWiring.Controller driveController;
  private final MechanismWiring.Controller steerController;

  /** Drive motor rotations for one rotation of the wheel. */
  private final double driveGearing;

  /** Drive motor rotations for one turn of the module, with the wheel held still. */
  private final double couplingRatio;

  private final double wheelRadiusMeters;

  /** Added to the absolute encoder's reading, in turns, so it reads zero pointing ahead. */
  private final double encoderOffset;

  /**
   * Whether the SPARKs have been sent their gains and limits yet. The first time, at startup, they
   * are saved on the SPARKs and waited for. After that they are sent without waiting.
   */
  private boolean configured;

  /**
   * The gains and limits each SPARK was last sent ({@link #configure}), kept to send again to one
   * that restarts. Until the first are sent each is a blank configuration, which changes nothing on
   * a SPARK.
   */
  private SparkBaseConfig driveTuning;

  private SparkBaseConfig steerTuning;

  /**
   * Whether each SPARK restarted since it was last asked. A restarted SPARK falls back to what was
   * saved on it, so it is sent the session's tuning again.
   */
  private final RevRestarts driveRestarts = new RevRestarts();

  private final RevRestarts steerRestarts = new RevRestarts();

  /** How many loops the module has been read in, to ask its SPARKs about restarts every few. */
  private int loops;

  /** The steering's damping, volts per radian per second of its turning. */
  private double steerKd;

  /** The SPARKs that didn't confirm their configuration at startup. */
  private final List<String> unconfigured = new ArrayList<>();

  /**
   * The same SPARKs as one line of text, such as "drive motor 3". Kept beside the list so the loop
   * hands the inputs a string without building one.
   */
  private String unconfiguredNames = "";

  private final Debounce driveConnected = new Debounce(SILENT_AFTER_SECONDS, DebounceType.FALLING);
  private final Debounce steerConnected = new Debounce(SILENT_AFTER_SECONDS, DebounceType.FALLING);

  /**
   * The last reading of each frame that came: the drive motor's turns and its turns a second, the
   * steering encoder's reading in turns, before its offset, and the module's turning speed in
   * rotations per second, which the coupling is worked out with too. A frame that does not come
   * leaves its reading here as it was. Each is 0 until its first frame, as REV reads one.
   */
  private double motorRotations;

  private double motorRotationsPerSecond;
  private double encoderRotations;
  private double steerRotationsPerSecond;

  /** Whether this loop's frames of the drive's motor came, both of them. */
  private boolean driveFramesCame;

  /** Whether this loop's frame of the steering encoder's angle came. */
  private boolean steerFrameCame;

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
    wheelRadiusMeters = module.wheelRadius().in(Meters);
    encoderOffset = wiring.encoderOffset().in(Rotations);

    // What the wiring decides is saved on the SPARKs now. The gains and limits come in configure().
    SparkBaseConfig driveSettings = driveSettings(wiring);
    SparkBaseConfig steerSettings = steerSettings(wiring);
    if (!applied(() -> resetAndSave(drive, driveSettings))) {
      noteUnconfigured("drive motor " + wiring.driveId());
    }
    if (!applied(() -> resetAndSave(steer, steerSettings))) {
      noteUnconfigured("steering motor " + wiring.steerId());
    }
    // Each SPARK's "has reset" warning is cleared now, so that what set it before the robot started
    // is not taken for a restart.
    drive.clearFaults();
    steer.clearFaults();
    driveTuning = blank(driveController);
    steerTuning = blank(steerController);
    driveEncoder = drive.getEncoder();
    steerEncoder = steer.getAbsoluteEncoder();
    driveLoop = drive.getClosedLoopController();
    steerLoop = steer.getClosedLoopController();
  }

  /**
   * The drive SPARK's settings that the wiring decides: its direction, braking when idle, and its
   * loop running on its motor's own encoder. It sends what this class reads once a robot loop: its
   * motor's position and speed, its output, its current, and the battery's voltage.
   */
  private SparkBaseConfig driveSettings(SwerveWiring.ModuleWiring wiring) {
    SparkBaseConfig settings = blank(driveController);
    settings
        .inverted(wiring.driveInverted())
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .voltageCompensation(Motor.NOMINAL_VOLTS);
    settings.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    settings
        .signals
        .primaryEncoderPositionPeriodMs(SIGNAL_MS)
        .primaryEncoderVelocityPeriodMs(SIGNAL_MS)
        .appliedOutputPeriodMs(SIGNAL_MS)
        .outputCurrentPeriodMs(SIGNAL_MS)
        .busVoltagePeriodMs(SIGNAL_MS);
    return settings;
  }

  /**
   * The steering SPARK's settings that the wiring decides: its direction, braking when idle, and
   * its loop running on the absolute encoder on its data port, wrapping so it takes the short way
   * round. It sends what this class reads once a robot loop: the encoder's angle and turning speed,
   * its output and its current.
   */
  private SparkBaseConfig steerSettings(SwerveWiring.ModuleWiring wiring) {
    SparkBaseConfig settings = blank(steerController);
    settings
        .inverted(wiring.steerInverted())
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .voltageCompensation(Motor.NOMINAL_VOLTS);
    if (steerController == MechanismWiring.Controller.SPARK_MAX) {
      // A SPARK MAX's data port serves several kinds of sensor, and has to be told this one.
      settings.absoluteEncoder.setSparkMaxDataPortConfig();
    }
    settings.absoluteEncoder.inverted(wiring.encoderInverted());
    settings
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true);
    settings
        .signals
        .absoluteEncoderPositionPeriodMs(SIGNAL_MS)
        .absoluteEncoderVelocityPeriodMs(SIGNAL_MS)
        .appliedOutputPeriodMs(SIGNAL_MS)
        .outputCurrentPeriodMs(SIGNAL_MS);
    return settings;
  }

  /**
   * Sends a SPARK its settings after resetting it to REV's defaults, so nothing left on it from
   * other code survives, and saves the result on it, so a brownout doesn't lose it.
   */
  private static REVLibError resetAndSave(SparkBase spark, SparkBaseConfig settings) {
    return spark.configure(
        settings, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Sends a SPARK more settings on top of what it has, and saves the result on it. */
  private static REVLibError addAndSave(SparkBase spark, SparkBaseConfig settings) {
    return spark.configure(
        settings, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sends a SPARK settings while the robot runs: on top of what it has, without waiting for it to
   * confirm, and without saving, since tuning belongs to the session.
   */
  private static void sendWhileRunning(SparkBase spark, SparkBaseConfig settings) {
    spark.configureAsync(
        settings, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** A SPARK of the kind the wiring names, a Flex or a MAX, driving a brushless motor. */
  private static SparkBase spark(MechanismWiring.Controller controller, CANPort port, int id) {
    if (controller == MechanismWiring.Controller.SPARK_FLEX) {
      return new SparkFlex(port, id, SparkLowLevel.MotorType.kBrushless);
    }
    return new SparkMax(port, id, SparkLowLevel.MotorType.kBrushless);
  }

  /** An empty configuration for a SPARK: only what's set on it is applied. */
  private static SparkBaseConfig blank(MechanismWiring.Controller controller) {
    if (controller == MechanismWiring.Controller.SPARK_FLEX) {
      return new SparkFlexConfig();
    }
    return new SparkMaxConfig();
  }

  /**
   * Whether a SPARK took a configuration. At startup, where waiting is fine, it is tried a few
   * times before giving up.
   */
  private static boolean applied(Supplier<REVLibError> attempt) {
    for (int i = 0; i < TRIES; i++) {
      if (attempt.get() == REVLibError.kOk) {
        return true;
      }
    }
    return false;
  }

  /** Notes something a SPARK didn't confirm at startup. */
  private void noteUnconfigured(String what) {
    unconfigured.add(what);
    unconfiguredNames = String.join(", ", unconfigured);
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
    // The frames are handed straight to a small step that takes numbers out of them, and nothing
    // else is handed them. Measured: handing the four frames on to everything below instead cost
    // 160 bytes more a loop, since the JIT then had to keep each frame as an object.
    sendTuningAgainAfterARestart();
    takeTheFramesThatCame(
        driveEncoder.getPosition(),
        driveEncoder.getVelocity(),
        steerEncoder.getPosition(),
        steerEncoder.getVelocity());
    fill(inputs);
  }

  /**
   * The same, from frames a test hands it: one that did not come, which REV's simulation never
   * leaves out by itself.
   */
  void updateInputs(
      ModuleIOInputs inputs,
      Signal<Double> motorTurns,
      Signal<Double> motorRpm,
      Signal<Double> encoderTurns,
      Signal<Double> encoderRpm) {
    sendTuningAgainAfterARestart();
    takeTheFramesThatCame(motorTurns, motorRpm, encoderTurns, encoderRpm);
    fill(inputs);
  }

  /**
   * Sends each SPARK that restarted since the last loop the session's tuning again. A restarted
   * SPARK has fallen back to what was saved on it at startup: the gains and limits of the first
   * {@link #configure}, without whatever was tuned since. Each is asked once in {@link
   * #RESTART_LOOPS} loops.
   */
  private void sendTuningAgainAfterARestart() {
    boolean asksThisLoop = loops % RESTART_LOOPS == 0;
    loops++;
    if (!asksThisLoop) {
      return;
    }
    if (restarted(drive, driveRestarts)) {
      sendWhileRunning(drive, driveTuning);
    }
    if (restarted(steer, steerRestarts)) {
      sendWhileRunning(steer, steerTuning);
    }
  }

  /**
   * Whether a SPARK restarted since the last loop, from its sticky "has reset" warning. Its faults
   * are cleared when it did, so that its next restart can show.
   */
  private static boolean restarted(SparkBase spark, RevRestarts restarts) {
    if (!restarts.happened(spark.getStickyWarnings())) {
      return false;
    }
    spark.clearFaults();
    return true;
  }

  /**
   * Takes each frame that came this loop as the module's reading, and notes which SPARKs answered.
   *
   * <p>A frame that did not come reads as REV's default, 0, which is also what a wheel that has not
   * turned and an encoder at its zero read. Taken as a reading, it would put the wheel back at 0
   * and the module at its encoder's offset for one loop, and odometry would take the wheel to have
   * rolled all the way back. So a missing frame is never taken as a reading: the last reading of
   * that frame stands, as a joint's SPARK keeps its last ({@code JointIOSpark}), and the wheel
   * reads as not having moved this loop. After half a second of missing frames the SPARK counts as
   * disconnected, and the drivetrain leaves its wheel out of odometry.
   *
   * @param motorTurns the drive motor's position frame, in the motor's turns
   * @param motorRpm the drive motor's speed frame, in the motor's turns a minute
   * @param encoderTurns the steering encoder's position frame, in turns from 0 to 1
   * @param encoderRpm the steering encoder's speed frame, in turns a minute
   */
  private void takeTheFramesThatCame(
      Signal<Double> motorTurns,
      Signal<Double> motorRpm,
      Signal<Double> encoderTurns,
      Signal<Double> encoderRpm) {
    if (motorTurns.isValid()) {
      motorRotations = motorTurns.get();
    }
    if (motorRpm.isValid()) {
      motorRotationsPerSecond = motorRpm.get() / SECONDS_PER_MINUTE;
    }
    if (encoderTurns.isValid()) {
      encoderRotations = encoderTurns.get();
    }
    if (encoderRpm.isValid()) {
      steerRotationsPerSecond = encoderRpm.get() / SECONDS_PER_MINUTE;
    }
    driveFramesCame = motorTurns.isValid() && motorRpm.isValid();
    steerFrameCame = encoderTurns.isValid();
  }

  /** Fills the inputs from the module's readings as they stand after this loop's frames. */
  private void fill(ModuleIOInputs inputs) {
    double moduleRotations = encoderRotations + encoderOffset;

    inputs.driveConnected = driveConnected.calculate(driveFramesCame);
    double wheelRotations =
        ModuleIOTalonFX.wheelRotations(
            motorRotations / driveGearing, moduleRotations, couplingRatio, driveGearing);
    inputs.drivePositionRad = TWO_PI * wheelRotations;
    inputs.driveVelocityRadPerSec =
        TWO_PI
            * ModuleIOTalonFX.wheelRotations(
                motorRotationsPerSecond / driveGearing,
                steerRotationsPerSecond,
                couplingRatio,
                driveGearing);
    // A SPARK reports its output as a share of the battery's voltage.
    double driveShare = drive.getAppliedOutput().get(0.0);
    inputs.driveAppliedVolts = driveShare * drive.getBusVoltage().get(0.0);
    double driveAmps = Math.abs(drive.getOutputCurrent().get(0.0));
    inputs.driveStatorCurrentAmps = driveAmps;
    // A SPARK measures the current through its motor, not what it draws from the battery. The
    // battery's share is estimated as the motor's current times the share of the battery's voltage
    // across the motor.
    inputs.driveSupplyCurrentAmps = Math.abs(driveShare) * driveAmps;
    // Whether the SPARK's current limit is holding the motor back is not read.
    inputs.driveTorqueLimited = false;

    inputs.steerConnected = steerConnected.calculate(steerFrameCame);
    inputs.steerAngle = Rotation2d.fromRotations(moduleRotations);
    inputs.steerVelocityRadPerSec = TWO_PI * steerRotationsPerSecond;
    inputs.steerAppliedVolts = steer.getAppliedOutput().get(0.0) * steer.getBusVoltage().get(0.0);
    inputs.steerStatorCurrentAmps = Math.abs(steer.getOutputCurrent().get(0.0));

    // One reading a loop: SPARKs aren't read at the TalonFX modules' 250 Hz.
    inputs.odometryTimestampsNanos = new long[] {RobotController.getTime()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometrySteerAnglesRad = new double[] {TWO_PI * moduleRotations};
    inputs.unconfigured = unconfiguredNames;
  }

  /**
   * Gains and limits, at startup and whenever tuning changes them: saved at startup, sent without
   * waiting after, so the loop never blocks.
   */
  @Override
  public void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    steerKd = gains.steerKd();
    driveTuning = blank(driveController);
    driveTuning.smartCurrentLimit(wholeAmps(limits.driveStatorCurrent().in(Amps)));
    driveTuning.closedLoop.p(driveKpAsAShare(gains), ClosedLoopSlot.kSlot0);
    steerTuning = blank(steerController);
    steerTuning.smartCurrentLimit(wholeAmps(limits.steerStatorCurrent().in(Amps)));
    steerTuning.closedLoop.p(steerKpAsAShare(gains), ClosedLoopSlot.kSlot0);
    if (!configured) {
      saveAtStartup();
      configured = true;
      return;
    }
    sendWhileRunning(drive, driveTuning);
    sendWhileRunning(steer, steerTuning);
  }

  /**
   * The drive's kP as its SPARK's loop takes it: a share of 12 V per RPM of error of its motor. The
   * template's is in volts per meter per second of the robot's speed. One wheel rotation a second
   * is 2π times the wheel's radius in meters a second, and it is the gearing times 60 RPM of the
   * motor.
   */
  private double driveKpAsAShare(DrivetrainControl.Gains gains) {
    double voltsPerMotorRpm =
        gains.driveKp() * TWO_PI * wheelRadiusMeters / driveGearing / SECONDS_PER_MINUTE;
    return voltsPerMotorRpm / Motor.NOMINAL_VOLTS;
  }

  /**
   * The steering's kP as its SPARK's loop takes it: a share of 12 V per turn of error of the
   * module, which is what the absolute encoder on its shaft reads. The template's is in volts per
   * radian, and one turn is 2π radians.
   */
  private static double steerKpAsAShare(DrivetrainControl.Gains gains) {
    double voltsPerModuleTurn = gains.steerKp() * TWO_PI;
    return voltsPerModuleTurn / Motor.NOMINAL_VOLTS;
  }

  /** A current limit as a SPARK takes it: whole amps. */
  private static int wholeAmps(double amps) {
    return (int) Math.round(amps);
  }

  /**
   * Saves the gains and limits on the SPARKs, on top of what the constructor saved, and waits for
   * each to confirm. One that doesn't is noted, for the drivetrain's alert.
   */
  private void saveAtStartup() {
    if (!applied(() -> addAndSave(drive, driveTuning))) {
      noteUnconfigured("drive motor's gains");
    }
    if (!applied(() -> addAndSave(steer, steerTuning))) {
      noteUnconfigured("steering motor's gains");
    }
  }

  @Override
  public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
    // The drive's own reading of its speed includes what the module's turning adds through the
    // bevel gear. So the speed it is told to hold is the wheel's, plus that share.
    double wheelRotationsPerSecond =
        wheelRadPerSec / TWO_PI + couplingRatio * steerRotationsPerSecond / driveGearing;
    // The SPARK's loop runs on its motor's speed, in turns a minute.
    double motorRpm = wheelRotationsPerSecond * driveGearing * SECONDS_PER_MINUTE;
    driveLoop.setSetpoint(
        motorRpm,
        SparkLowLevel.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setSteerAngle(Rotation2d angle) {
    // The SPARK's loop runs on the encoder's own reading, which is the module's angle with the
    // offset taken back off, as a share of a turn from 0 to 1.
    double encoderTurns = angle.getRotations() - encoderOffset;
    double encoderReading = encoderTurns - Math.floor(encoderTurns);
    // The steering's damping: volts against the module's measured turning speed.
    double dampingVolts = -steerKd * TWO_PI * steerRotationsPerSecond;
    steerLoop.setSetpoint(
        encoderReading,
        SparkLowLevel.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        dampingVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    drive.stopMotor();
    steer.stopMotor();
  }
}

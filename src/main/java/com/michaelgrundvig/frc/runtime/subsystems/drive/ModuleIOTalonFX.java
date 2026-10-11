package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.function.Supplier;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * One swerve module on CTRE hardware: a TalonFX driving the wheel, a TalonFX turning the module,
 * and a CANcoder reading the module's angle. The drivetrain's configuration gives the gearing,
 * gains and limits. The robot's wiring gives the CAN IDs, the directions and the encoder's offset.
 *
 * <p><b>No Phoenix Pro license is needed.</b> The steering motor closes its loop on the CANcoder as
 * a remote sensor, and no request uses FOC.
 *
 * <p><b>Units.</b> The gearing is configured on the motor controllers, so the drive reports in
 * rotations of the wheel and the steering in rotations of the module. This class turns those into
 * radians.
 *
 * <p><b>Coupling.</b> In an MK4-style module, turning the module also turns the wheel a little,
 * through the bevel gear, with the robot standing still. The drive's encoder counts that as travel.
 * So that share is taken out of every drive reading ({@link #wheelRotations}), and added to the
 * speed the drive is asked to hold, so the wheel keeps its speed over the floor while the module
 * steers ({@link #setDriveVelocity}).
 */
final class ModuleIOTalonFX implements ModuleIO {
  private static final double TWO_PI = 2 * Math.PI;

  /** At startup each device is sent its settings up to this many times, until it confirms them. */
  private static final int TRIES = 5;

  /** How long each of those tries waits for the device to confirm. Startup can afford the wait. */
  private static final double STARTUP_TIMEOUT_SECONDS = 0.25;

  /**
   * A timeout of zero sends a setting and returns without waiting for the device to confirm it.
   * Everything sent while the robot's loop runs uses this, because a loop that waits on a device
   * stutters.
   */
  private static final double NO_WAIT_SECONDS = 0;

  /**
   * How long a device may go without answering before it is called disconnected. One missed frame
   * is a glitch, not a lost device.
   */
  private static final double SILENT_AFTER_SECONDS = 0.5;

  private final TalonFX drive;
  private final TalonFX steer;

  /** Drive motor rotations for one rotation of the wheel. */
  private final double driveGearing;

  /** Drive motor rotations for one turn of the module, with the wheel held still. */
  private final double couplingRatio;

  private final double wheelRadiusMeters;

  // Each motor's whole configuration. The constructor fills in what the wiring and the gearing
  // decide. The gains and current limits are filled in by configure(), which then sends them.
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration steerConfig = new TalonFXConfiguration();

  /**
   * Whether the motor controllers have been sent their whole configuration yet. The first time, at
   * startup, it is sent whole and waited for. After that, only the gains and limits are sent.
   */
  private boolean configured;

  /** The devices that didn't confirm their configuration at startup, for the drivetrain's alert. */
  private final List<String> unconfigured = new ArrayList<>();

  /**
   * The same devices as one line of text, such as "drive motor 3, CANcoder 9". Kept beside the list
   * so the loop hands the inputs a string without building one.
   */
  private String unconfiguredNames = "";

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveVolts;
  private final StatusSignal<Current> driveStatorCurrent;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Boolean> driveTorqueLimited;
  private final StatusSignal<Angle> steerPosition;
  private final StatusSignal<AngularVelocity> steerVelocity;
  private final StatusSignal<Voltage> steerVolts;
  private final StatusSignal<Current> steerStatorCurrent;
  private final StatusSignal<Angle> encoderAbsolute;

  // The odometry thread's readings since the last loop, oldest first. One time per reading, and
  // for each reading the drive's position, the module's angle and the steering's current.
  private final Queue<Long> odometryTimes;
  private final Queue<Double> odometryDriveRotations;
  private final Queue<Double> odometrySteerRotations;
  private final Queue<Double> odometrySteerCurrent;

  private final Debouncer driveConnected =
      new Debouncer(SILENT_AFTER_SECONDS, Debouncer.DebounceType.FALLING);
  private final Debouncer steerConnected =
      new Debouncer(SILENT_AFTER_SECONDS, Debouncer.DebounceType.FALLING);

  // The requests sent to the motors. Each is made once and reused, so asking allocates nothing.
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final NeutralOut neutral = new NeutralOut();

  /** The module's turning speed as last read, in rotations per second, for the coupling. */
  private double steerRotationsPerSecond;

  /**
   * @param wiring the module's devices
   * @param bus the bus they're on
   * @param hardware the modules' gearing and wheel
   * @param couplingRatio drive motor rotations per module turn, with the wheel held still
   * @param odometry the thread that reads the drive's and steering's positions at 250 Hz
   */
  ModuleIOTalonFX(
      SwerveWiring.ModuleWiring wiring,
      CANBus bus,
      DrivetrainGeometry.ModuleType hardware,
      double couplingRatio,
      PhoenixOdometryThread odometry) {
    drive = new TalonFX(wiring.driveId(), bus);
    steer = new TalonFX(wiring.steerId(), bus);
    int encoderId = wiring.encoderId().orElseThrow();
    CANcoder encoder = new CANcoder(encoderId, bus);
    driveGearing = hardware.driveGearing().ratio();
    this.couplingRatio = couplingRatio;
    wheelRadiusMeters = hardware.wheelRadius().in(Meters);

    // The CANcoder is configured here. The motors' configurations are only filled in: configure()
    // sends them, once it has added the gains and limits.
    CANcoderConfiguration encoderConfig = encoderSettings(wiring);
    if (!tryUntilOk(
        () -> encoder.getConfigurator().apply(encoderConfig, STARTUP_TIMEOUT_SECONDS))) {
      noteUnconfigured("CANcoder " + encoderId);
    }
    fillInDriveSettings(wiring);
    fillInSteerSettings(wiring, encoderId, hardware.steerGearing().ratio());

    drivePosition = drive.getPosition();
    driveVelocity = drive.getVelocity();
    driveVolts = drive.getMotorVoltage();
    driveStatorCurrent = drive.getStatorCurrent();
    driveSupplyCurrent = drive.getSupplyCurrent();
    driveTorqueLimited = drive.getFault_StatorCurrLimit();
    steerPosition = steer.getPosition();
    steerVelocity = steer.getVelocity();
    steerVolts = steer.getMotorVoltage();
    steerStatorCurrent = steer.getStatorCurrent();
    encoderAbsolute = encoder.getAbsolutePosition();
    setSignalRates(encoder);

    // The thread reads its own copies of the signals, so it never shares one with this loop.
    odometryTimes = odometry.timestamps();
    odometryDriveRotations = odometry.register(drivePosition.clone());
    odometrySteerRotations = odometry.register(steerPosition.clone());
    odometrySteerCurrent = odometry.register(steerStatorCurrent.clone());
  }

  /**
   * The CANcoder's settings: its offset and direction from the wiring, so that it reads zero with
   * the wheel pointing straight ahead, and a wrap point of half a turn, so that it reads from half
   * a turn one way to half a turn the other.
   */
  private static CANcoderConfiguration encoderSettings(SwerveWiring.ModuleWiring wiring) {
    CANcoderConfiguration settings = new CANcoderConfiguration();
    settings.MagnetSensor.MagnetOffset = wiring.encoderOffset().in(Rotations);
    settings.MagnetSensor.SensorDirection =
        wiring.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    settings.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    return settings;
  }

  /**
   * Fills in the drive motor's settings that the wiring and gearing decide. With the gearing as its
   * sensor-to-mechanism ratio, the motor reports and is asked in rotations of the wheel. Both
   * current limits are switched on here; their values, and the supply limit's step-down, come in
   * {@link #configure}.
   */
  private void fillInDriveSettings(SwerveWiring.ModuleWiring wiring) {
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = direction(wiring.driveInverted());
    driveConfig.Feedback.SensorToMechanismRatio = driveGearing;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  /**
   * Fills in the steering motor's settings that the wiring and gearing decide. Its loop runs on the
   * CANcoder, read over CAN as a remote sensor, so its position is the module's angle in rotations.
   * Its loop wraps, so it turns the short way round to an angle.
   *
   * <p>The steering has no supply current limit. The current a motor draws from the battery is
   * never more than the current through it, so the stator limit already caps it. Phoenix turns a
   * supply limit on unless told not to (70 A, stepping down to 40 A after a second), so it is
   * turned off here, where it shows. With it off, the step-down can't happen either: it is part of
   * the limit.
   *
   * @param steerGearing steering motor rotations for one turn of the module, which is also one turn
   *     of the CANcoder
   */
  private void fillInSteerSettings(
      SwerveWiring.ModuleWiring wiring, int encoderId, double steerGearing) {
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted = direction(wiring.steerInverted());
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.FeedbackRemoteSensorID = encoderId;
    steerConfig.Feedback.RotorToSensorRatio = steerGearing;
    steerConfig.Feedback.SensorToMechanismRatio = 1;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
  }

  /**
   * Sets how often the devices send each signal, to keep the CAN bus's load down. There are three
   * rates:
   *
   * <ul>
   *   <li><b>250 Hz</b> ({@link PhoenixOdometryThread#FREQUENCY_HZ}) for what the odometry thread
   *       reads: the drive's position and the module's angle, so odometry sees every turn of the
   *       wheel, and the steering's current, whose peaks last only a few milliseconds.
   *   <li><b>100 Hz</b> ({@link PhoenixDrivetrain#ENCODER_FREQUENCY_HZ}) for the CANcoder's
   *       position and velocity, which the steering motor's loop runs on. This rate is set
   *       explicitly because signals that share a CAN frame go at the fastest rate any of them asks
   *       for. Left at the default, they could go no faster than the absolute position's once a
   *       loop.
   *   <li><b>Once a robot loop</b> ({@link RobotLoop#FREQUENCY_HZ}) for everything else this class
   *       reads: reading faster than the loop gains nothing.
   * </ul>
   *
   * <p>Then Phoenix is told to stop every frame of the motor controllers whose rate was not set
   * here.
   */
  private void setSignalRates(CANcoder encoder) {
    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixOdometryThread.FREQUENCY_HZ, drivePosition, steerPosition, steerStatorCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixDrivetrain.ENCODER_FREQUENCY_HZ, encoder.getPosition(), encoder.getVelocity());
    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotLoop.FREQUENCY_HZ,
        driveVelocity,
        driveVolts,
        driveStatorCurrent,
        driveSupplyCurrent,
        driveTorqueLimited,
        steerVelocity,
        steerVolts,
        encoderAbsolute);
    ParentDevice.optimizeBusUtilizationForAll(drive, steer);
  }

  /**
   * Which way a motor turns for a positive output, as Phoenix says it: counterclockwise, unless the
   * wiring says the motor is inverted.
   */
  private static InvertedValue direction(boolean inverted) {
    if (inverted) {
      return InvertedValue.Clockwise_Positive;
    }
    return InvertedValue.CounterClockwise_Positive;
  }

  /** Notes a device that didn't confirm its configuration at startup. */
  private void noteUnconfigured(String device) {
    unconfigured.add(device);
    unconfiguredNames = String.join(", ", unconfigured);
  }

  /** Configuring at startup, where waiting is fine: tries a few times before giving up. */
  static boolean tryUntilOk(Supplier<StatusCode> attempt) {
    for (int i = 0; i < TRIES; i++) {
      if (attempt.get().isOK()) {
        return true;
      }
    }
    return false;
  }

  /**
   * The wheel's rotations from the drive's reading, less what the module's turning put there. Works
   * the same for positions (rotations) and velocities (rotations per second).
   */
  static double wheelRotations(
      double driveReading, double moduleRotations, double couplingRatio, double driveGearing) {
    return driveReading - couplingRatio * moduleRotations / driveGearing;
  }

  /**
   * The most current, either way, among a loop's readings and the latest: the steering's peak,
   * which lasts a few milliseconds as a turn starts and stops, so a reading once a loop would
   * almost always miss it. A reading that didn't come is recorded as no number, and is passed over.
   * Empties the queue of readings.
   */
  static double mostAmps(Queue<Double> readings, double latest) {
    double most = Math.abs(latest);
    while (true) {
      Double amps = readings.poll();
      if (amps == null) {
        return most;
      }
      if (!amps.isNaN()) {
        most = Math.max(most, Math.abs(amps));
      }
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // The signals are listed at each call, not kept in an array made once. Measured on Phoenix
    // 26.70, refreshing an array kept in a field allocated about twice as much a loop as this.
    boolean driveAnswered =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveVolts,
                driveStatorCurrent,
                driveSupplyCurrent,
                driveTorqueLimited)
            .isOK();
    boolean steerAnswered =
        BaseStatusSignal.refreshAll(
                steerPosition, steerVelocity, steerVolts, steerStatorCurrent, encoderAbsolute)
            .isOK();
    steerRotationsPerSecond = steerVelocity.getValueAsDouble();

    inputs.driveConnected = driveConnected.calculate(driveAnswered);
    inputs.drivePositionRad =
        TWO_PI
            * wheelRotations(
                drivePosition.getValueAsDouble(),
                steerPosition.getValueAsDouble(),
                couplingRatio,
                driveGearing);
    inputs.driveVelocityRadPerSec =
        TWO_PI
            * wheelRotations(
                driveVelocity.getValueAsDouble(),
                steerRotationsPerSecond,
                couplingRatio,
                driveGearing);
    inputs.driveAppliedVolts = driveVolts.getValueAsDouble();
    // Phoenix signs both currents: negative while the motor brakes and sends current back. The
    // inputs hold sizes, as every driver's do.
    inputs.driveStatorCurrentAmps = Math.abs(driveStatorCurrent.getValueAsDouble());
    inputs.driveSupplyCurrentAmps = Math.abs(driveSupplyCurrent.getValueAsDouble());
    inputs.driveTorqueLimited = Boolean.TRUE.equals(driveTorqueLimited.getValue());

    inputs.steerConnected = steerConnected.calculate(steerAnswered);
    inputs.steerAngle = Rotation2d.fromRotations(encoderAbsolute.getValueAsDouble());
    inputs.steerVelocityRadPerSec = TWO_PI * steerRotationsPerSecond;
    inputs.steerAppliedVolts = steerVolts.getValueAsDouble();
    inputs.steerStatorCurrentAmps =
        mostAmps(odometrySteerCurrent, steerStatorCurrent.getValueAsDouble());

    drainOdometryReadings(inputs);
    inputs.unconfigured = unconfiguredNames;
  }

  /**
   * Takes every high-rate reading the odometry thread has recorded since the last loop, oldest
   * first: the time each was measured, and the wheel's position and the module's angle then. The
   * drivetrain holds the thread's lock while it reads, so every module's readings are in step.
   */
  private void drainOdometryReadings(ModuleIOInputs inputs) {
    int readings =
        Math.min(
            odometryTimes.size(),
            Math.min(odometryDriveRotations.size(), odometrySteerRotations.size()));
    long[] timesNanos = new long[readings];
    double[] drivePositionsRad = new double[readings];
    double[] steerAnglesRad = new double[readings];
    for (int reading = 0; reading < readings; reading++) {
      Long timeNanos = odometryTimes.poll();
      Double driveRotations = odometryDriveRotations.poll();
      Double moduleRotations = odometrySteerRotations.poll();
      if (timeNanos == null || driveRotations == null || moduleRotations == null) {
        break; // cannot happen while the lock is held; the arrays keep what was read
      }
      timesNanos[reading] = timeNanos;
      steerAnglesRad[reading] = TWO_PI * moduleRotations;
      drivePositionsRad[reading] =
          TWO_PI * wheelRotations(driveRotations, moduleRotations, couplingRatio, driveGearing);
    }
    inputs.odometryTimestampsNanos = timesNanos;
    inputs.odometryDrivePositionsRad = drivePositionsRad;
    inputs.odometrySteerAnglesRad = steerAnglesRad;
  }

  /**
   * Gains and limits, at startup and whenever tuning changes them. At startup it waits for each
   * device to confirm; while running it sends them without waiting, so the loop never blocks: a
   * robot loop that waits on a device stutters.
   */
  @Override
  public void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    fillInGains(gains);
    fillInCurrentLimits(limits);
    if (!configured) {
      configureAtStartup();
      configured = true;
      return;
    }
    retuneWhileRunning();
  }

  /**
   * Converts the template's gains to the devices' units. The drive's kP is given in volts per meter
   * per second of the robot's speed, and Phoenix wants volts per wheel rotation per second: one
   * rotation is 2π times the wheel's radius in meters. The steering's kP and kD are given per
   * radian, and Phoenix wants them per rotation: one rotation is 2π radians.
   */
  private void fillInGains(DrivetrainControl.Gains gains) {
    driveConfig.Slot0.kP = gains.driveKp() * TWO_PI * wheelRadiusMeters;
    steerConfig.Slot0.kP = gains.steerKp() * TWO_PI;
    steerConfig.Slot0.kD = gains.steerKd() * TWO_PI;
  }

  /**
   * Sets the motors' current limits, in amps.
   *
   * <p>The stator limit caps the current through a motor, and so its torque.
   *
   * <p>The drive's supply limit caps what the motor draws from the battery, through its breaker.
   * Phoenix lets the motor draw up to {@code SupplyCurrentLimit}. Once the motor has been held at
   * that limit for {@code SupplyCurrentLowerTime} seconds, Phoenix lowers the limit to {@code
   * SupplyCurrentLowerLimit} until the motor draws less than that again. Phoenix does this even
   * when the code doesn't ask: without the last two lines below, its defaults (40 A after 1 s) come
   * back unseen, and the robot loses its push a second into a hard shove with nothing here to say
   * why (PhoenixIOTest fails if they go). So all three come from the configuration. The step-down
   * itself is worth keeping: a 40 A breaker carries 40 A for as long as you like, but can trip
   * after 3.9 s at 60 A. {@code DrivetrainControl.Limits} says more, and docs/values.md says how
   * the values were chosen.
   */
  private void fillInCurrentLimits(DrivetrainControl.Limits limits) {
    driveConfig.CurrentLimits.StatorCurrentLimit = limits.driveStatorCurrent().in(Amps);
    steerConfig.CurrentLimits.StatorCurrentLimit = limits.steerStatorCurrent().in(Amps);
    driveConfig.CurrentLimits.SupplyCurrentLimit = limits.driveSupplyCurrent().in(Amps);
    driveConfig.CurrentLimits.SupplyCurrentLowerLimit = limits.driveSupplyLowerCurrent().in(Amps);
    driveConfig.CurrentLimits.SupplyCurrentLowerTime = limits.driveSupplyLowerTime().in(Seconds);
  }

  /**
   * Sends each motor controller its whole configuration and waits for it to confirm. One that
   * doesn't is noted, for the drivetrain's alert.
   */
  private void configureAtStartup() {
    if (!tryUntilOk(() -> drive.getConfigurator().apply(driveConfig, STARTUP_TIMEOUT_SECONDS))) {
      noteUnconfigured("drive motor " + drive.getDeviceID());
    }
    if (!tryUntilOk(() -> steer.getConfigurator().apply(steerConfig, STARTUP_TIMEOUT_SECONDS))) {
      noteUnconfigured("steering motor " + steer.getDeviceID());
    }
  }

  /**
   * Sends the gains and the current limits, which are all that tuning changes, without waiting for
   * the motor controllers to confirm.
   */
  private void retuneWhileRunning() {
    drive.getConfigurator().apply(driveConfig.Slot0, NO_WAIT_SECONDS);
    drive.getConfigurator().apply(driveConfig.CurrentLimits, NO_WAIT_SECONDS);
    steer.getConfigurator().apply(steerConfig.Slot0, NO_WAIT_SECONDS);
    steer.getConfigurator().apply(steerConfig.CurrentLimits, NO_WAIT_SECONDS);
  }

  @Override
  public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
    // The drive's own reading of its speed includes what the module's turning adds through the
    // bevel gear. So the speed it is told to hold is the wheel's, plus that share.
    double wheelRotationsPerSecond = wheelRadPerSec / TWO_PI;
    double addedByTurning = couplingRatio * steerRotationsPerSecond / driveGearing;
    drive.setControl(
        velocityRequest
            .withVelocity(wheelRotationsPerSecond + addedByTurning)
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setSteerAngle(Rotation2d angle) {
    steer.setControl(positionRequest.withPosition(angle.getRotations()));
  }

  @Override
  public void stop() {
    drive.setControl(neutral);
    steer.setControl(neutral);
  }
}

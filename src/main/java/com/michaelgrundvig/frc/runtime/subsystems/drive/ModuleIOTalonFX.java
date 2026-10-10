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
 * and a CANcoder reading its angle. The drivetrain's configuration gives the gearing, gains, and
 * limits; the robot's wiring gives the CAN IDs, directions, and the encoder's offset.
 *
 * <p>Runs on Phoenix 6 without a Pro license: the steering motor closes its loop on the CANcoder as
 * a remote sensor, and no request uses FOC. The motor controllers report in wheel and module
 * rotations (the gearing is configured on them). Turning an MK4-style module turns its wheel
 * through the bevel gear, which the drive encoder would count as travel: that share is taken out of
 * every reading, and added to the drive's velocity target so the wheel holds its speed while
 * steering.
 */
final class ModuleIOTalonFX implements ModuleIO {
  private static final double TWO_PI = 2 * Math.PI;

  /** At startup, configuring a device is tried this many times, waiting this long each time. */
  private static final int TRIES = 5;

  private static final double STARTUP_TIMEOUT_SECONDS = 0.25;

  private final TalonFX drive;
  private final TalonFX steer;
  private final double driveGearing;
  private final double couplingRatio;
  private final double wheelRadiusMeters;
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration steerConfig = new TalonFXConfiguration();
  private boolean configured;

  /** The devices that didn't confirm their configuration at startup, for the drivetrain's alert. */
  private final List<String> unconfigured = new ArrayList<>();

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

  private final Queue<Long> odometryTimes;
  private final Queue<Double> odometryDriveRotations;
  private final Queue<Double> odometrySteerRotations;
  private final Queue<Double> odometrySteerCurrent;

  // A device counts as disconnected once it has been silent for half a second, not at one glitch.
  private final Debouncer driveConnected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer steerConnected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

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
    CANcoder encoder = new CANcoder(wiring.encoderId().orElseThrow(), bus);
    driveGearing = hardware.driveGearing().ratio();
    this.couplingRatio = couplingRatio;
    wheelRadiusMeters = hardware.wheelRadius().in(Meters);

    // The encoder reads zero with the wheel straight ahead, and half a turn either way.
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = wiring.encoderOffset().in(Rotations);
    encoderConfig.MagnetSensor.SensorDirection =
        wiring.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    if (!tryUntilOk(
        () -> encoder.getConfigurator().apply(encoderConfig, STARTUP_TIMEOUT_SECONDS))) {
      unconfigured.add("CANcoder " + wiring.encoderId().orElseThrow());
    }

    // The drive reports and controls wheel rotations. Gains and limits come in configure().
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = direction(wiring.driveInverted());
    driveConfig.Feedback.SensorToMechanismRatio = driveGearing;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true; // and its step-down: configure()

    // The steering closes its loop on the CANcoder, in module rotations, the short way around.
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted = direction(wiring.steerInverted());
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.FeedbackRemoteSensorID = wiring.encoderId().orElseThrow();
    steerConfig.Feedback.RotorToSensorRatio = hardware.steerGearing().ratio();
    steerConfig.Feedback.SensorToMechanismRatio = 1;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // No supply limit for the steering: the current a motor draws from the battery is never more
    // than the current through it, so the stator limit already caps it. Phoenix turns a supply
    // limit on unless told not to (70 A, stepping down to 40 A after a second), so it's turned off
    // here, where it shows. With it off, the step-down can't happen either: it's part of the limit.
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

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

    // Odometry's signals as fast as the thread reads them, with the steering's current (its peaks
    // last a few milliseconds), the rest once a loop, and nothing else from the motor controllers.
    // The CANcoder's position and velocity, which the steering's loop runs on, go at their own
    // rate, set explicitly: signals that share a frame go at the fastest rate any of them asks for,
    // and left at the default, they could go no faster than the absolute position's once a loop.
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

    // The thread reads its own copies of the signals, so it never shares one with this loop.
    odometryTimes = odometry.timestamps();
    odometryDriveRotations = odometry.register(drivePosition.clone());
    odometrySteerRotations = odometry.register(steerPosition.clone());
    odometrySteerCurrent = odometry.register(steerStatorCurrent.clone());
  }

  private static InvertedValue direction(boolean clockwise) {
    return clockwise ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
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
   * almost always miss it. A reading that didn't come, no number, is passed over. Takes the
   * readings.
   */
  static double mostAmps(Queue<Double> readings, double latest) {
    double most = Math.abs(latest);
    for (Double amps = readings.poll(); amps != null; amps = readings.poll()) {
      if (!amps.isNaN()) {
        most = Math.max(most, Math.abs(amps));
      }
    }
    return most;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    boolean driveOk =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveVolts,
                driveStatorCurrent,
                driveSupplyCurrent,
                driveTorqueLimited)
            .isOK();
    boolean steerOk =
        BaseStatusSignal.refreshAll(
                steerPosition, steerVelocity, steerVolts, steerStatorCurrent, encoderAbsolute)
            .isOK();
    steerRotationsPerSecond = steerVelocity.getValueAsDouble();

    inputs.driveConnected = driveConnected.calculate(driveOk);
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
    inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueLimited = Boolean.TRUE.equals(driveTorqueLimited.getValue());

    inputs.steerConnected = steerConnected.calculate(steerOk);
    inputs.steerAngle = Rotation2d.fromRotations(encoderAbsolute.getValueAsDouble());
    inputs.steerVelocityRadPerSec = TWO_PI * steerRotationsPerSecond;
    inputs.steerAppliedVolts = steerVolts.getValueAsDouble();
    inputs.steerStatorCurrentAmps =
        mostAmps(odometrySteerCurrent, steerStatorCurrent.getValueAsDouble());

    // The drivetrain holds the odometry thread's lock while reading, so these are in step.
    int readings =
        Math.min(
            odometryTimes.size(),
            Math.min(odometryDriveRotations.size(), odometrySteerRotations.size()));
    long[] times = new long[readings];
    double[] drivePositions = new double[readings];
    double[] steerAngles = new double[readings];
    for (int k = 0; k < readings; k++) {
      Long time = odometryTimes.poll();
      Double driveRotations = odometryDriveRotations.poll();
      Double moduleRotations = odometrySteerRotations.poll();
      if (time == null || driveRotations == null || moduleRotations == null) {
        break; // cannot happen while the lock is held; the arrays keep what was read
      }
      times[k] = time;
      steerAngles[k] = TWO_PI * moduleRotations;
      drivePositions[k] =
          TWO_PI * wheelRotations(driveRotations, moduleRotations, couplingRatio, driveGearing);
    }
    inputs.odometryTimestampsNanos = times;
    inputs.odometryDrivePositionsRad = drivePositions;
    inputs.odometrySteerAnglesRad = steerAngles;
    inputs.unconfigured = String.join(", ", unconfigured);
  }

  /**
   * Gains and limits, at startup and whenever tuning changes them. At startup it waits for each
   * device to confirm; while running it sends them without waiting, so the loop never blocks: a
   * robot loop that waits on a device stutters.
   */
  @Override
  public void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    // The template's gains are per meter per second and per radian; the devices' per rotation.
    driveConfig.Slot0.kP = gains.driveKp() * TWO_PI * wheelRadiusMeters;
    driveConfig.CurrentLimits.StatorCurrentLimit = limits.driveStatorCurrent().in(Amps);
    // The supply limit, and its step-down. Supply current is what the motor draws from the battery,
    // through its breaker. Phoenix lets the motor draw up to SupplyCurrentLimit, but once it has
    // been held at that limit for SupplyCurrentLowerTime, it lowers the limit to
    // SupplyCurrentLowerLimit until the motor draws less than that again. Phoenix does this even
    // when the code doesn't ask: delete the last two lines below, and its defaults (40 A after 1 s)
    // come back unseen, and the robot loses its push a second into a hard shove with nothing here
    // to say why (PhoenixIOTest fails if they go). So all three come from the configuration. The
    // step-down itself is worth keeping: a 40 A breaker carries 40 A for as long as you like, but
    // can trip after 3.9 s at 60 A. DrivetrainControl.Limits says more; docs/values.md, how the
    // values were chosen.
    driveConfig.CurrentLimits.SupplyCurrentLimit = limits.driveSupplyCurrent().in(Amps);
    driveConfig.CurrentLimits.SupplyCurrentLowerLimit = limits.driveSupplyLowerCurrent().in(Amps);
    driveConfig.CurrentLimits.SupplyCurrentLowerTime = limits.driveSupplyLowerTime().in(Seconds);
    steerConfig.Slot0.kP = gains.steerKp() * TWO_PI;
    steerConfig.Slot0.kD = gains.steerKd() * TWO_PI;
    steerConfig.CurrentLimits.StatorCurrentLimit = limits.steerStatorCurrent().in(Amps);
    if (!configured) {
      if (!tryUntilOk(() -> drive.getConfigurator().apply(driveConfig, STARTUP_TIMEOUT_SECONDS))) {
        unconfigured.add("drive motor " + drive.getDeviceID());
      }
      if (!tryUntilOk(() -> steer.getConfigurator().apply(steerConfig, STARTUP_TIMEOUT_SECONDS))) {
        unconfigured.add("steering motor " + steer.getDeviceID());
      }
      configured = true;
      return;
    }
    drive.getConfigurator().apply(driveConfig.Slot0, 0);
    drive.getConfigurator().apply(driveConfig.CurrentLimits, 0);
    steer.getConfigurator().apply(steerConfig.Slot0, 0);
    steer.getConfigurator().apply(steerConfig.CurrentLimits, 0);
  }

  @Override
  public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
    // The drive's reading includes the coupling, so aim for the wheel's speed plus it.
    double target =
        wheelRadPerSec / TWO_PI + couplingRatio * steerRotationsPerSecond / driveGearing;
    drive.setControl(velocityRequest.withVelocity(target).withFeedForward(feedforwardVolts));
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

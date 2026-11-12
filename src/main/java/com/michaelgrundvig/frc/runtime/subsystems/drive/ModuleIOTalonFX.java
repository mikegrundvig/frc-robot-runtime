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
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.runtime.behavior.Debounce;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixRestarts;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixStartup;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.math.filter.Debouncer.DebounceType;
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
 * <p><b>No Phoenix Pro license is needed</b> unless the wiring says {@code .foc()}. The steering
 * motor closes its loop on the CANcoder as a remote sensor, and a request uses FOC only where its
 * motor's controller says {@code .foc()}: the drive's speed, the steering's angle, or both.
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
 *
 * <p><b>Settings that weren't taken.</b> Each device is sent its whole configuration as the robot
 * starts. One that doesn't confirm is named in the inputs ({@link ModuleIOInputs#unconfigured}),
 * and the drivetrain holds still while any is. Once that device answers, it is sent its whole
 * configuration again, on a thread of its own, until it confirms ({@link DeviceSettings}).
 *
 * <p><b>Restarts.</b> A TalonFX that loses power and comes back keeps its settings, and counts its
 * rotor from zero. Phoenix says when the drive's has sent the frame it sends as it starts, and the
 * inputs say so for that one loop ({@link ModuleIOInputs#driveRestarted}), so the drivetrain does
 * not take the wheel's count starting again for travel.
 *
 * <p><b>The CANcoder's magnet.</b> What the CANcoder says of its magnet, and its faults, are read
 * ten times a second ({@link #HEALTH_LOOPS}). Reading them costs no CAN traffic: a CANcoder sends
 * them whether or not anything reads them, since its frames are not trimmed here.
 */
final class ModuleIOTalonFX implements ModuleIO {
  private static final double TWO_PI = 2 * Math.PI;

  /**
   * A timeout of zero sends a setting and returns without waiting for the device to confirm it.
   * Everything sent while the robot's loop runs uses this, because a loop that waits on a device
   * stutters.
   */
  private static final double NO_WAIT_SECONDS = 0;

  /**
   * How long a device may go without answering before it is called disconnected. One missed frame
   * is a glitch, not a lost device.
   *
   * <p>The half second is kept in whole nanoseconds ({@link Debounce}), not by WPILib's debouncer,
   * whose seconds in a double put the edge a loop early or late depending on what the clock reads.
   * So every driver, and the simulator, calls a device silent in the same loop.
   */
  private static final double SILENT_AFTER_SECONDS = 0.5;

  /**
   * The CANcoder's magnet and faults are read once in this many loops, ten times a second: a magnet
   * that has slipped stays slipped, and a few readings a second show it.
   */
  static final int HEALTH_LOOPS = 5;

  /** Each of the module's devices, as the tests stand in for sending one its settings. */
  enum Device {
    DRIVE,
    STEER,
    ENCODER
  }

  private final TalonFX drive;
  private final TalonFX steer;
  private final CANcoder encoder;

  /** SwerveDrive motor rotations for one rotation of the wheel. */
  private final double driveGearing;

  /** SwerveDrive motor rotations for one turn of the module, with the wheel held still. */
  private final double couplingRatio;

  private final double wheelRadiusMeters;

  /** Where the module hears its devices and sends them their settings ({@link Frames}). */
  private final Frames frames;

  // Each device's whole configuration. The constructor fills in what the wiring and the gearing
  // decide. The motors' gains and current limits are filled in by configure(), which then sends
  // them.
  private final CANcoderConfiguration encoderConfig;
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration steerConfig = new TalonFXConfiguration();

  /** Whether each device has taken its whole configuration, and sending it again until it has. */
  private final DeviceSettings driveSettings;

  private final DeviceSettings steerSettings;
  private final DeviceSettings encoderSettings;

  /** How many times each device is sent its settings at startup. */
  private final int startupTries;

  /**
   * The devices that haven't taken their whole configuration, as one line of text, such as "drive
   * motor 3, CANcoder 9". Worked out again only when one takes it, so the loop hands the inputs a
   * string without building one.
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
  private final StatusSignal<MagnetHealthValue> encoderMagnet;
  private final StatusSignal<Integer> encoderFaults;

  // The odometry thread's readings since the last loop, oldest first. One time per reading, and
  // for each reading the drive's position, the module's angle and the steering's current.
  private final Queue<Long> odometryTimes;
  private final Queue<Double> odometryDriveRotations;
  private final Queue<Double> odometrySteerRotations;
  private final Queue<Double> odometrySteerCurrent;

  /**
   * Whether the drive's controller restarted since the last loop. A restarted TalonFX counts its
   * rotor from zero, so the wheel's position starts again there.
   */
  private final PhoenixRestarts driveRestarts;

  private final Debounce driveConnected = new Debounce(SILENT_AFTER_SECONDS, DebounceType.FALLING);
  private final Debounce steerConnected = new Debounce(SILENT_AFTER_SECONDS, DebounceType.FALLING);

  // The requests sent to the motors. Each is made once and reused, so asking allocates nothing.
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final NeutralOut neutral = new NeutralOut();

  /** The module's turning speed as last read, in rotations per second, for the coupling. */
  private double steerRotationsPerSecond;

  /** What the CANcoder last said of its magnet, and its faults: read every few loops. */
  private MagnetHealth magnetHealth = MagnetHealth.NOT_SAID;

  private int faults;

  /** How many loops the module has been read in, to read the CANcoder's health every few. */
  private int loops;

  /**
   * Where the module hears its devices, and sends them their settings. On a robot this is Phoenix,
   * as here.
   *
   * <p>CTRE's simulated devices never restart, and always take what they are sent. So the tests of
   * what the module does when its drive restarts, or when a device doesn't take its settings, put
   * their own in this one's place, as the joints' tests do ({@code JointIOTalon.Frames}).
   */
  static class Frames {
    /** Whether the drive has sent the frame it sends as it starts, since this was last asked. */
    BooleanSupplier driveStartFrames(ParentDevice drive) {
      return drive.getResetOccurredChecker();
    }

    /**
     * Sends one of the module's devices its whole configuration, and says whether it confirmed.
     *
     * @param send the send, which waits for the device's answer
     */
    StatusCode send(Device device, Supplier<StatusCode> send) {
      return send.get();
    }
  }

  /**
   * @param wiring the module's devices
   * @param bus the bus they're on
   * @param hardware the modules' gearing and wheel
   * @param couplingRatio drive motor rotations per module turn, with the wheel held still
   * @param odometry the thread that reads the drive's and steering's positions at 250 Hz
   * @param startup how many times each device is tried at startup, and where a send while the robot
   *     runs waits for its answer
   */
  ModuleIOTalonFX(
      SwerveWiring.ModuleWiring wiring,
      CANBus bus,
      Drivetrain.Module hardware,
      double couplingRatio,
      PhoenixOdometryThread odometry,
      ModuleStartup startup) {
    this(wiring, bus, hardware, couplingRatio, odometry, startup, new Frames());
  }

  /**
   * The same, with where the module hears its devices and sends them their settings. For the tests,
   * which restart the drive there, or have a device not take its settings ({@link Frames}).
   */
  ModuleIOTalonFX(
      SwerveWiring.ModuleWiring wiring,
      CANBus bus,
      Drivetrain.Module hardware,
      double couplingRatio,
      PhoenixOdometryThread odometry,
      ModuleStartup startup,
      Frames frames) {
    this.frames = frames;
    startupTries = startup.tries();
    MechanismWiring.Cancoder cancoder = cancoderOf(wiring);
    int driveId = wiring.drive().id();
    int steerId = wiring.steering().id();
    int encoderId = cancoder.id();
    drive = new TalonFX(driveId, bus);
    steer = new TalonFX(steerId, bus);
    encoder = new CANcoder(encoderId, bus);
    driveGearing = hardware.driveGearing().ratio();
    this.couplingRatio = couplingRatio;
    wheelRadiusMeters = hardware.wheelRadius().in(Meters);
    driveSettings = new DeviceSettings("drive motor " + driveId, startup.sender());
    steerSettings = new DeviceSettings("steering motor " + steerId, startup.sender());
    encoderSettings = new DeviceSettings("CANcoder " + encoderId, startup.sender());

    // The CANcoder is configured here. The motors' configurations are only filled in: configure()
    // sends them, once it has added the gains and limits.
    encoderConfig = encoderSettings(cancoder);
    encoderSettings.sentAtStartup(confirmedAtStartup(Device.ENCODER));
    noteWhatIsUnconfigured();
    fillInDriveSettings(wiring);
    fillInSteerSettings(wiring, encoderId, hardware.steerGearing().ratio());
    // FOC is asked for on every request to a motor whose wiring says .foc(), as a joint's are.
    velocityRequest.withEnableFOC(saysFoc(wiring.drive()));
    positionRequest.withEnableFOC(saysFoc(wiring.steering()));

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
    encoderMagnet = encoder.getMagnetHealth();
    encoderFaults = encoder.getFaultField();
    setDriveSignalRates();
    setSteerSignalRates();
    setEncoderSignalRates();
    ParentDevice.optimizeBusUtilizationForAll(drive, steer);

    // The thread reads its own copies of the signals, so it never shares one with this loop.
    odometryTimes = odometry.timestamps();
    odometryDriveRotations = odometry.register(drivePosition.clone());
    odometrySteerRotations = odometry.register(steerPosition.clone());
    odometrySteerCurrent = odometry.register(steerStatorCurrent.clone());
    // Asked from here, once the devices are configured: the frame the drive sent as it and the
    // robot started is not a restart.
    driveRestarts = new PhoenixRestarts(frames.driveStartFrames(drive));
  }

  /**
   * The module's CANcoder. A TalonFX steers by one, so the robot's definition refuses a module of
   * TalonFXs without one before this is built.
   */
  /** What the drive motor was last told. For the tests, which read its requests back. */
  ControlRequest driveAsked() {
    return drive.getAppliedControl();
  }

  /** What the steering motor was last told. For the tests, which read its requests back. */
  ControlRequest steerAsked() {
    return steer.getAppliedControl();
  }

  /** Whether a motor's controller is wired to drive it with FOC, which needs Phoenix Pro. */
  private static boolean saysFoc(MechanismWiring.Device device) {
    return device.commutation() == MechanismWiring.Commutation.FOC;
  }

  private static MechanismWiring.Cancoder cancoderOf(SwerveWiring.ModuleWiring wiring) {
    if (wiring.encoder() instanceof MechanismWiring.Cancoder cancoder) {
      return cancoder;
    }
    throw new IllegalArgumentException(
        "a TalonFX steers by a CANcoder, and this module's encoder is on a data port");
  }

  /**
   * The CANcoder's settings: its offset and direction from the wiring, so that it reads zero with
   * the wheel pointing straight ahead, and a wrap point of half a turn, so that it reads from half
   * a turn one way to half a turn the other.
   */
  private static CANcoderConfiguration encoderSettings(MechanismWiring.Cancoder cancoder) {
    CANcoderConfiguration settings = new CANcoderConfiguration();
    settings.MagnetSensor.MagnetOffset = cancoder.offset().in(Rotations);
    settings.MagnetSensor.SensorDirection =
        cancoder.turnsClockwise()
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
    driveConfig.MotorOutput.Inverted = direction(wiring.drive());
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
    steerConfig.MotorOutput.Inverted = direction(wiring.steering());
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.FeedbackRemoteSensorID = encoderId;
    steerConfig.Feedback.RotorToSensorRatio = steerGearing;
    steerConfig.Feedback.SensorToMechanismRatio = 1;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
  }

  /**
   * How often the devices send each signal, to keep the CAN bus's load down. There are three rates:
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
   * here. Each device's rates are set again when it takes its settings after startup: one that
   * wasn't on the bus then never heard them.
   */
  private void setDriveSignalRates() {
    BaseStatusSignal.setUpdateFrequencyForAll(PhoenixOdometryThread.FREQUENCY_HZ, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotLoop.FREQUENCY_HZ,
        driveVelocity,
        driveVolts,
        driveStatorCurrent,
        driveSupplyCurrent,
        driveTorqueLimited);
  }

  private void setSteerSignalRates() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixOdometryThread.FREQUENCY_HZ, steerPosition, steerStatorCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, steerVelocity, steerVolts);
  }

  private void setEncoderSignalRates() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        PhoenixDrivetrain.ENCODER_FREQUENCY_HZ, encoder.getPosition(), encoder.getVelocity());
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, encoderAbsolute);
  }

  /**
   * Which way a motor turns for a positive output, as Phoenix says it: counterclockwise, seen
   * facing its shaft, unless its wiring says {@code .clockwise()}.
   */
  private static InvertedValue direction(MechanismWiring.Device motor) {
    if (motor.turnsClockwise()) {
      return InvertedValue.Clockwise_Positive;
    }
    return InvertedValue.CounterClockwise_Positive;
  }

  /**
   * One whole send of a device's configuration as it stands now, which waits for the device to
   * confirm. A motor's configuration is copied here, on the loop, so tuning that changes it while
   * the send waits on the sender can't change what is sent.
   */
  private Supplier<StatusCode> wholeConfiguration(Device device) {
    return switch (device) {
      case DRIVE -> {
        TalonFXConfiguration whole = driveConfig.clone();
        yield () -> drive.getConfigurator().apply(whole, PhoenixStartup.TIMEOUT_SECONDS);
      }
      case STEER -> {
        TalonFXConfiguration whole = steerConfig.clone();
        yield () -> steer.getConfigurator().apply(whole, PhoenixStartup.TIMEOUT_SECONDS);
      }
      case ENCODER ->
          () -> encoder.getConfigurator().apply(encoderConfig, PhoenixStartup.TIMEOUT_SECONDS);
    };
  }

  /** Sends a device its whole configuration at startup, as many times as startup tries. */
  private boolean confirmedAtStartup(Device device) {
    return PhoenixStartup.confirmed(
        startupTries, () -> frames.send(device, wholeConfiguration(device)));
  }

  /** Works out the line of devices that haven't taken their whole configuration. */
  private void noteWhatIsUnconfigured() {
    List<String> missing = new ArrayList<>();
    for (DeviceSettings device : List.of(driveSettings, steerSettings, encoderSettings)) {
      if (device.missing()) {
        missing.add(device.name());
      }
    }
    unconfiguredNames = String.join(", ", missing);
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

  /**
   * What a CANcoder's word on its magnet means to the robot. A reading Phoenix calls invalid says
   * nothing about the magnet.
   */
  static MagnetHealth magnetHealth(MagnetHealthValue said) {
    return switch (said) {
      case Magnet_Green -> MagnetHealth.GREEN;
      case Magnet_Orange -> MagnetHealth.ORANGE;
      case Magnet_Red -> MagnetHealth.RED;
      case Magnet_Invalid -> MagnetHealth.NOT_SAID;
    };
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
        BaseStatusSignal.refreshAll(steerPosition, steerVelocity, steerVolts, steerStatorCurrent)
            .isOK();
    boolean encoderAnswered = BaseStatusSignal.refreshAll(encoderAbsolute).isOK();
    steerRotationsPerSecond = steerVelocity.getValueAsDouble();
    sendSettingsAgainOnceAnswered(driveAnswered, steerAnswered, encoderAnswered);
    readEncoderHealthEveryFewLoops();

    inputs.driveConnected = driveConnected.calculate(driveAnswered);
    inputs.driveRestarted = driveRestarts.happened();
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

    inputs.steerConnected = steerConnected.calculate(steerAnswered && encoderAnswered);
    // The CANcoder reads from half a turn one way to half a turn the other, with its offset.
    double moduleRotations = encoderAbsolute.getValueAsDouble();
    inputs.steerAngle = Rotation2d.fromRotations(moduleRotations);
    inputs.steerAngleRotations = moduleRotations;
    inputs.steerVelocityRadPerSec = TWO_PI * steerRotationsPerSecond;
    inputs.steerAppliedVolts = steerVolts.getValueAsDouble();
    inputs.steerStatorCurrentAmps =
        mostAmps(odometrySteerCurrent, steerStatorCurrent.getValueAsDouble());
    inputs.encoderMagnet = magnetHealth;
    inputs.encoderFaults = faults;

    drainOdometryReadings(inputs);
    inputs.unconfigured = unconfiguredNames;
  }

  /**
   * Sends each device that hasn't taken its settings its whole configuration again, once it
   * answers, and takes the answers that have come. A motor that takes them is sent the gains and
   * limits as they are now too, in case the pit tuned them while the send waited.
   */
  private void sendSettingsAgainOnceAnswered(
      boolean driveAnswered, boolean steerAnswered, boolean encoderAnswered) {
    if (driveSettings.wantsSendingAgain(driveAnswered)) {
      driveSettings.sendAgain(sentWithItsRates(Device.DRIVE));
    }
    if (steerSettings.wantsSendingAgain(steerAnswered)) {
      steerSettings.sendAgain(sentWithItsRates(Device.STEER));
    }
    if (encoderSettings.wantsSendingAgain(encoderAnswered)) {
      encoderSettings.sendAgain(sentWithItsRates(Device.ENCODER));
    }
    boolean driveTookIt = driveSettings.answerCame();
    boolean steerTookIt = steerSettings.answerCame();
    boolean encoderTookIt = encoderSettings.answerCame();
    if (driveTookIt) {
      retune(drive, driveConfig);
    }
    if (steerTookIt) {
      retune(steer, steerConfig);
    }
    if (driveTookIt || steerTookIt || encoderTookIt) {
      noteWhatIsUnconfigured();
    }
  }

  /**
   * A send for the sender: a device's whole configuration, as it stands now, and once the device
   * confirms it, the rates of the signals read from it. A device that wasn't on the bus as the
   * robot started never heard them.
   *
   * @return the send, which says whether the device confirmed its configuration
   */
  private BooleanSupplier sentWithItsRates(Device device) {
    Supplier<StatusCode> whole = wholeConfiguration(device);
    return () -> {
      if (!frames.send(device, whole).isOK()) {
        return false;
      }
      switch (device) {
        case DRIVE -> {
          setDriveSignalRates();
          drive.optimizeBusUtilization();
        }
        case STEER -> {
          setSteerSignalRates();
          steer.optimizeBusUtilization();
        }
        case ENCODER -> setEncoderSignalRates();
      }
      return true;
    };
  }

  /** Reads what the CANcoder says of its magnet, and its faults, once in {@link #HEALTH_LOOPS}. */
  private void readEncoderHealthEveryFewLoops() {
    boolean readsHealthThisLoop = loops % HEALTH_LOOPS == 0;
    loops++;
    if (!readsHealthThisLoop) {
      return;
    }
    // A reading that didn't come leaves the last one standing.
    if (encoderMagnet.refresh().getStatus().isOK()) {
      magnetHealth = magnetHealth(encoderMagnet.getValue());
    }
    if (encoderFaults.refresh().getStatus().isOK()) {
      faults = encoderFaults.getValue();
    }
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
  public void configure(DrivetrainControl.SwerveGains gains, DrivetrainControl.Limits limits) {
    fillInGains(gains);
    fillInCurrentLimits(limits);
    if (driveSettings.state() == DeviceSettings.State.NOT_YET_SENT) {
      configureAtStartup();
      return;
    }
    retune(drive, driveConfig);
    retune(steer, steerConfig);
  }

  /**
   * Converts the template's gains to the devices' units. The drive's kP is given in volts per meter
   * per second of the robot's speed, and Phoenix wants volts per wheel rotation per second: one
   * rotation is 2π times the wheel's radius in meters. The steering's kP and kD are given per
   * radian, and Phoenix wants them per rotation: one rotation is 2π radians.
   */
  private void fillInGains(DrivetrainControl.SwerveGains gains) {
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
   * doesn't is noted, for the drivetrain's alert, and sent it again once it answers.
   */
  private void configureAtStartup() {
    driveSettings.sentAtStartup(confirmedAtStartup(Device.DRIVE));
    steerSettings.sentAtStartup(confirmedAtStartup(Device.STEER));
    noteWhatIsUnconfigured();
  }

  /**
   * Sends a motor the gains and the current limits, which are all that tuning changes, without
   * waiting for it to confirm.
   */
  private static void retune(TalonFX motor, TalonFXConfiguration config) {
    motor.getConfigurator().apply(config.Slot0, NO_WAIT_SECONDS);
    motor.getConfigurator().apply(config.CurrentLimits, NO_WAIT_SECONDS);
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

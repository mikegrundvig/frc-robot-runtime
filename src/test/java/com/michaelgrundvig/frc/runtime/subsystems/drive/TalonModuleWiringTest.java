package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.TestWiring;
import com.michaelgrundvig.frc.runtime.Waits;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiringModuleWiringBuilder;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation2d;

/**
 * What the CTRE module writes to its devices, read back from Phoenix's own simulated ones: for each
 * of a table of wirings, every setting the wiring decides, and what the module then reads. Then the
 * drivetrain's four modules, each reading its own devices; and a device that doesn't take its
 * settings.
 *
 * <p>Phoenix's simulated devices outlive a test, so every row here uses the same three CAN IDs and
 * is sent its whole configuration afresh, as a device out of its box would be. The IDs are ones no
 * other test in the runtime uses (24 to 40): Phoenix's simulated devices are all on one bus.
 */
@ExtendWith(RobotSim.class)
class TalonModuleWiringTest {
  private static final CANBus BUS = new CANBus("wiring");

  /** The same bus, as the robot's wiring names it. */
  private static final CanBus WIRING_BUS = CanBus.canivore("wiring");

  private static final Drivetrain GEOMETRY = ExampleRobot.SIM.definition().drivetrain();
  private static final Drivetrain.Module MODULE = GEOMETRY.module(0);
  private static final DrivetrainControl DRIVETRAIN = ExampleRobot.SIM.drivetrain();
  private static final int DRIVE_ID = 37;
  private static final int STEER_ID = 38;
  private static final int ENCODER_ID = 39;

  /**
   * How near the wheel's reading must be, in radians. Phoenix's simulated position reads a little
   * short (0.0016 rad in two turns); a wrong direction or gearing is turns out.
   */
  private static final double WHEEL_RADIANS = 0.01;

  /** How long a read of a device's settings waits for them, in seconds. */
  private static final double SETTINGS_WAIT_SECONDS = 1;

  /** Each device tried as on a bus that is seen, and a send while running answered at once. */
  private static final ModuleStartup STARTUP = new ModuleStartup(5, Runnable::run);

  private final List<PhoenixOdometryThread> threads = new ArrayList<>();

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  @AfterEach
  void stopTheThreads() throws InterruptedException {
    List<PhoenixOdometryThread> stillRunning = new ArrayList<>();
    for (PhoenixOdometryThread thread : threads) {
      thread.interrupt();
      thread.join(2000);
      if (thread.isAlive()) {
        stillRunning.add(thread);
      }
    }
    new TalonFX(DRIVE_ID, BUS).setControl(new NeutralOut());
    new TalonFX(STEER_ID, BUS).setControl(new NeutralOut());
    // A thread still running would read the next test's devices: that fails this one.
    assertThat(stillRunning)
        .as("odometry threads still running 2 s after they were stopped")
        .isEmpty();
  }

  /**
   * One wiring of the module: its CANcoder's offset, in rotations, and which way each device is
   * mounted.
   */
  private record Wiring(
      double offset, boolean driveInverted, boolean steerInverted, boolean encoderInverted) {
    SwerveWiring.ModuleWiring of(int drive, int steer, int encoder) {
      return of(WIRING_BUS, drive, steer, encoder);
    }

    /** The module at these IDs on a bus, each inverted device written {@code .clockwise()}. */
    SwerveWiring.ModuleWiring of(CanBus bus, int drive, int steer, int encoder) {
      MechanismWiring.Device driveMotor = MechanismWiring.Controller.TALON_FX.can(bus, drive);
      MechanismWiring.Device steerMotor = MechanismWiring.Controller.TALON_FX.can(bus, steer);
      MechanismWiring.Cancoder cancoder =
          MechanismWiring.CANCODER.can(bus, encoder).offset(Rotations.of(offset));
      return SwerveWiringModuleWiringBuilder.builder()
          .drive(driveInverted ? driveMotor.clockwise() : driveMotor)
          .steering(steerInverted ? steerMotor.clockwise() : steerMotor)
          .encoder(encoderInverted ? cancoder.clockwise() : cancoder)
          .build();
    }

    @Override
    public String toString() {
      return "offset "
          + offset
          + (driveInverted ? ", drive inverted" : "")
          + (steerInverted ? ", steering inverted" : "")
          + (encoderInverted ? ", CANcoder inverted" : "");
    }
  }

  /**
   * Offsets of 0.1, 0.4 and -0.3 of a turn, and each of the three directions both ways, in rows
   * where no two signs move together.
   */
  private static final List<Wiring> WIRINGS =
      List.of(
          new Wiring(0.1, false, false, false),
          new Wiring(0.4, true, false, false),
          new Wiring(-0.3, false, true, false),
          new Wiring(0.1, false, false, true),
          new Wiring(0.4, true, true, true),
          new Wiring(-0.3, true, false, true));

  private PhoenixOdometryThread odometry() {
    PhoenixOdometryThread thread = new PhoenixOdometryThread();
    threads.add(thread);
    return thread;
  }

  /** Reads some of a device's settings back from it, asking until it answers. */
  private static void readBack(Supplier<StatusCode> read) throws InterruptedException {
    Waits.until("the device's settings read back", () -> {}, () -> read.get().isOK());
  }

  /**
   * Every setting the wiring decides is what each device holds: the CANcoder's offset, direction
   * and wrap point; each motor's direction and brake; the drive's gearing; and the steering's loop
   * on the CANcoder, by its ID, through its gearing, wrapping. Then the module reads what that
   * means: the CANcoder's magnet a quarter turn round reads that quarter turn, its way, with the
   * offset, and the drive's rotor two wheel turns round reads two turns of the wheel, its way.
   */
  @Test
  void eachDeviceHoldsWhatTheWiringSaysAndTheModuleReadsItSo() throws InterruptedException {
    TalonFX drive = new TalonFX(DRIVE_ID, BUS);
    TalonFX steer = new TalonFX(STEER_ID, BUS);
    CANcoder encoder = new CANcoder(ENCODER_ID, BUS);
    double driveGearing = MODULE.driveGearing().ratio();
    for (Wiring row : WIRINGS) {
      drive.getSimState().setRawRotorPosition(0);
      encoder.getSimState().setRawPosition(0);
      encoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Green);
      // No coupling, so the wheel's reading is its drive's alone.
      ModuleIOTalonFX io =
          new ModuleIOTalonFX(
              row.of(DRIVE_ID, STEER_ID, ENCODER_ID), BUS, MODULE, 0, odometry(), STARTUP);
      io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());

      MagnetSensorConfigs magnet = new MagnetSensorConfigs();
      readBack(() -> encoder.getConfigurator().refresh(magnet, SETTINGS_WAIT_SECONDS));
      assertThat(magnet.MagnetOffset)
          .as("%s: the offset", row)
          .isCloseTo(row.offset(), within(1e-3));
      assertThat(magnet.SensorDirection)
          .as("%s: the CANcoder's direction", row)
          .isEqualTo(
              row.encoderInverted()
                  ? SensorDirectionValue.Clockwise_Positive
                  : SensorDirectionValue.CounterClockwise_Positive);
      assertThat(magnet.AbsoluteSensorDiscontinuityPoint)
          .as("%s: wraps at half a turn", row)
          .isEqualTo(0.5);

      MotorOutputConfigs driveOutput = new MotorOutputConfigs();
      FeedbackConfigs driveFeedback = new FeedbackConfigs();
      readBack(() -> drive.getConfigurator().refresh(driveOutput, SETTINGS_WAIT_SECONDS));
      readBack(() -> drive.getConfigurator().refresh(driveFeedback, SETTINGS_WAIT_SECONDS));
      assertThat(driveOutput.Inverted)
          .as("%s: the drive's direction", row)
          .isEqualTo(direction(row.driveInverted()));
      assertThat(driveOutput.NeutralMode).isEqualTo(NeutralModeValue.Brake);
      assertThat(driveFeedback.SensorToMechanismRatio)
          .as("%s: the drive's gearing", row)
          .isCloseTo(driveGearing, within(1e-4));
      assertThat(driveFeedback.FeedbackSensorSource)
          .isEqualTo(FeedbackSensorSourceValue.RotorSensor);

      MotorOutputConfigs steerOutput = new MotorOutputConfigs();
      FeedbackConfigs steerFeedback = new FeedbackConfigs();
      ClosedLoopGeneralConfigs steerLoop = new ClosedLoopGeneralConfigs();
      readBack(() -> steer.getConfigurator().refresh(steerOutput, SETTINGS_WAIT_SECONDS));
      readBack(() -> steer.getConfigurator().refresh(steerFeedback, SETTINGS_WAIT_SECONDS));
      readBack(() -> steer.getConfigurator().refresh(steerLoop, SETTINGS_WAIT_SECONDS));
      assertThat(steerOutput.Inverted)
          .as("%s: the steering's direction", row)
          .isEqualTo(direction(row.steerInverted()));
      assertThat(steerOutput.NeutralMode).isEqualTo(NeutralModeValue.Brake);
      assertThat(steerFeedback.FeedbackSensorSource)
          .isEqualTo(FeedbackSensorSourceValue.RemoteCANcoder);
      assertThat(steerFeedback.FeedbackRemoteSensorID)
          .as("%s: its CANcoder", row)
          .isEqualTo(ENCODER_ID);
      assertThat(steerFeedback.RotorToSensorRatio)
          .as("%s: the steering's gearing", row)
          .isCloseTo(MODULE.steerGearing().ratio(), within(1e-4));
      assertThat(steerFeedback.SensorToMechanismRatio).isEqualTo(1);
      assertThat(steerLoop.ContinuousWrap).as("%s: it turns the short way", row).isTrue();

      // A quarter turn of the magnet, counterclockwise as the CANcoder's own face has it, and two
      // turns of the wheel at the drive's rotor, each the way its device is mounted.
      encoder.getSimState().setRawPosition(0.25);
      drive.getSimState().setRawRotorPosition(2 * driveGearing);
      double angle = wrapped((row.encoderInverted() ? -0.25 : 0.25) + row.offset());
      double wheel = 2 * Math.PI * (row.driveInverted() ? -2 : 2);
      ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
      Waits.until(
          () ->
              String.format(
                  "%s: reads %s of a turn and %s rad, wants %s and %s",
                  row, inputs.steerAngleRotations, inputs.drivePositionRad, angle, wheel),
          () -> io.updateInputs(inputs),
          () ->
              Math.abs(inputs.steerAngleRotations - angle) < 1e-3
                  && Math.abs(inputs.drivePositionRad - wheel) < WHEEL_RADIANS);
      assertThat(inputs.steerAngle.getRotations()).isCloseTo(angle, within(1e-3));
      assertThat(inputs.unconfigured).as("%s: every device took its settings", row).isEmpty();
    }
  }

  /**
   * The drive's speed and the steering's angle each ask for FOC when that motor's controller says
   * {@code .foc()}, and not when it doesn't, in every pairing of the two.
   */
  @Test
  void eachMotorsRequestsAskForFocOnlyWhenItsControllerSaysSo() {
    record Row(boolean driveFoc, boolean steerFoc) {}
    List<Row> rows =
        List.of(
            new Row(false, false), new Row(true, false), new Row(false, true), new Row(true, true));
    for (Row row : rows) {
      SwerveWiring.ModuleWiring plain = WIRINGS.get(0).of(DRIVE_ID, STEER_ID, ENCODER_ID);
      SwerveWiring.ModuleWiring wiring =
          SwerveWiringModuleWiringBuilder.builder(plain)
              .drive(row.driveFoc() ? plain.drive().foc() : plain.drive())
              .steering(row.steerFoc() ? plain.steering().foc() : plain.steering())
              .build();
      ModuleIOTalonFX io = new ModuleIOTalonFX(wiring, BUS, MODULE, 0, odometry(), STARTUP);

      io.setDriveVelocity(10, 0.5);
      io.setSteerAngle(Rotation2d.fromDegrees(30));

      assertThat(((VelocityVoltage) io.driveAsked()).EnableFOC)
          .as("%s: the drive's speed", row)
          .isEqualTo(row.driveFoc());
      assertThat(((PositionVoltage) io.steerAsked()).EnableFOC)
          .as("%s: the steering's angle", row)
          .isEqualTo(row.steerFoc());
      io.stop();
    }
  }

  /** A turn from half a turn one way to half a turn the other. */
  private static double wrapped(double rotations) {
    return rotations - Math.floor(rotations + 0.5);
  }

  private static InvertedValue direction(boolean inverted) {
    return inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  }

  /**
   * The drivetrain's four modules, each at its own IDs, each read from its own devices: front left,
   * front right, back left, back right, as the wiring lists them. Each drive is a different number
   * of wheel turns round and each CANcoder a different angle, so a module that read another's
   * devices would read another's numbers. The gyro reads through the wiring's scale.
   */
  @Test
  void eachOfTheFourModulesReadsItsOwnDevicesAndTheGyroItsScale() throws InterruptedException {
    CanBus fourModules = CanBus.canivore("four-modules");
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>();
    for (int i = 0; i < Module.COUNT; i++) {
      modules.add(
          new Wiring(0, false, false, false).of(fourModules, 24 + 3 * i, 25 + 3 * i, 26 + 3 * i));
    }
    SwerveWiring wiring =
        TestWiring.wired(
                GEOMETRY, SwerveWiring.PIGEON_2.can(fourModules, 36).yawScale(1.05), modules)
            .wiring()
            .orElseThrow();
    CANBus bus = new CANBus("four-modules");
    double driveGearing = MODULE.driveGearing().ratio();
    for (int i = 0; i < Module.COUNT; i++) {
      new TalonFX(24 + 3 * i, bus).getSimState().setRawRotorPosition(0);
      new CANcoder(26 + 3 * i, bus).getSimState().setRawPosition(0);
    }

    try (PhoenixDrivetrain drivetrain = new PhoenixDrivetrain(wiring, GEOMETRY)) {
      for (int i = 0; i < Module.COUNT; i++) {
        new TalonFX(24 + 3 * i, bus).getSimState().setRawRotorPosition((i + 1) * driveGearing);
        new CANcoder(26 + 3 * i, bus).getSimState().setRawPosition(0.05 * (i + 1));
      }
      new Pigeon2(36, bus).getSimState().setRawYaw(80);

      // The drivetrain sends each module its gains and limits, and so its whole configuration.
      ModuleIO[] ios = drivetrain.modules();
      for (ModuleIO io : ios) {
        io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());
      }
      for (int i = 0; i < Module.COUNT; i++) {
        ModuleIO io = ios[i];
        int module = i;
        ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
        double angle = 0.05 * (i + 1);
        // The coupling takes the module's turn out of the wheel's reading.
        double wheel =
            2
                * Math.PI
                * ModuleIOTalonFX.wheelRotations(
                    i + 1, angle, MODULE.couplingRatio(), driveGearing);
        Waits.until(
            () ->
                String.format(
                    "%s reads %s of a turn and %s rad, wants its own %s and %s",
                    Module.NAMES.get(module),
                    inputs.steerAngleRotations,
                    inputs.drivePositionRad,
                    angle,
                    wheel),
            () -> readUnderTheLock(drivetrain, () -> io.updateInputs(inputs)),
            () ->
                Math.abs(inputs.steerAngleRotations - angle) < 1e-3
                    && Math.abs(inputs.drivePositionRad - wheel) < WHEEL_RADIANS);
      }
      GyroIOInputsAutoLogged gyro = new GyroIOInputsAutoLogged();
      Waits.until(
          () ->
              String.format(
                  Locale.ROOT, "80° through a scale of 1.05, read %s", gyro.yaw.getDegrees()),
          () -> readUnderTheLock(drivetrain, () -> drivetrain.gyro().updateInputs(gyro)),
          () -> Math.abs(gyro.yaw.getDegrees() - 84) < 0.05);
    }
  }

  private static void readUnderTheLock(PhoenixDrivetrain drivetrain, Runnable read) {
    drivetrain.odometryLock().lock();
    try {
      read.run();
    } finally {
      drivetrain.odometryLock().unlock();
    }
  }

  /**
   * Where the module sends its settings, with a hand on it: a device listed as not answering
   * doesn't confirm, as one off the bus or at a wrong CAN ID wouldn't. Every send is counted.
   */
  private static final class Refusing extends ModuleIOTalonFX.Frames {
    private final Set<ModuleIOTalonFX.Device> refusing =
        EnumSet.noneOf(ModuleIOTalonFX.Device.class);
    private final AtomicInteger sends = new AtomicInteger();

    @Override
    StatusCode send(ModuleIOTalonFX.Device device, Supplier<StatusCode> send) {
      sends.incrementAndGet();
      if (refusing.contains(device)) {
        return StatusCode.EcuIsNotPresent;
      }
      return send.get();
    }
  }

  /** A device that doesn't take its settings, and what the module and the drivetrain say of it. */
  private record Missing(ModuleIOTalonFX.Device device, String named) {}

  /**
   * A device that never confirms its settings at startup is named in the inputs, as each of the
   * three is. Once it answers and takes them, it is sent its whole configuration again and leaves
   * the list: here the steering, its configuration read back from it.
   */
  @Test
  void aDeviceThatDoesntTakeItsSettingsIsNamedUntilItDoes() throws InterruptedException {
    List<Missing> rows =
        List.of(
            new Missing(ModuleIOTalonFX.Device.DRIVE, "drive motor " + DRIVE_ID),
            new Missing(ModuleIOTalonFX.Device.STEER, "steering motor " + STEER_ID),
            new Missing(ModuleIOTalonFX.Device.ENCODER, "CANcoder " + ENCODER_ID));
    for (Missing row : rows) {
      Refusing frames = new Refusing();
      frames.refusing.add(row.device());
      ModuleIOTalonFX io =
          new ModuleIOTalonFX(
              new Wiring(0.1, false, true, false).of(DRIVE_ID, STEER_ID, ENCODER_ID),
              BUS,
              MODULE,
              0,
              odometry(),
              STARTUP,
              frames);
      io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());
      ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
      io.updateInputs(inputs);

      assertThat(inputs.unconfigured).as("%s refused", row.device()).isEqualTo(row.named());

      frames.refusing.clear();
      Waits.until(
          () -> String.format(Locale.ROOT, "%s answers and takes it", row.device()),
          () -> io.updateInputs(inputs),
          () -> inputs.unconfigured.isEmpty());
    }
    MotorOutputConfigs steerOutput = new MotorOutputConfigs();
    readBack(
        () ->
            new TalonFX(STEER_ID, BUS)
                .getConfigurator()
                .refresh(steerOutput, SETTINGS_WAIT_SECONDS));
    assertThat(steerOutput.Inverted)
        .as("the steering's direction, sent again")
        .isEqualTo(InvertedValue.Clockwise_Positive);
  }

  /**
   * On a CANivore that isn't seen as the robot starts, each device is sent its settings once, not
   * five times: no device on it could answer.
   */
  @Test
  void onABusThatIsntSeenEachDeviceIsSentItsSettingsOnce() {
    Refusing frames = new Refusing();
    frames.refusing.addAll(EnumSet.allOf(ModuleIOTalonFX.Device.class));
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(
            new Wiring(0, false, false, false).of(DRIVE_ID, STEER_ID, ENCODER_ID),
            BUS,
            MODULE,
            0,
            odometry(),
            new ModuleStartup(1, Runnable::run),
            frames);
    io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());

    assertThat(frames.sends).as("one send for each of the three").hasValue(3);
  }
}

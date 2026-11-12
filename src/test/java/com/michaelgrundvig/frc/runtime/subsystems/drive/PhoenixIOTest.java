package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotation;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.TestWiring;
import com.michaelgrundvig.frc.runtime.Waits;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.ToDoubleFunction;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;

/**
 * The CTRE IO against Phoenix's own simulated devices: set what a sensor reads, and check what the
 * robot code gets. Conversions, the coupling, and the odometry thread; the control loops belong to
 * the devices.
 */
@ExtendWith(RobotSim.class)
class PhoenixIOTest {
  private static final CANBus BUS = new CANBus("drivetrain");
  private static final Drivetrain GEOMETRY = ExampleRobot.SIM.definition().drivetrain();
  private static final DrivetrainControl DRIVETRAIN = ExampleRobot.SIM.drivetrain();
  private static final double COUPLING = 3.5;

  /** Each device tried as on a bus that is seen, and a send while running answered at once. */
  private static final ModuleStartup STARTUP = new ModuleStartup(5, Runnable::run);

  private final PhoenixOdometryThread odometry = new PhoenixOdometryThread();

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false); // as PhoenixDrivetrain does on the robot
  }

  @AfterEach
  void tearDown() throws InterruptedException {
    odometry.interrupt();
    odometry.join(2000);
    // Stopped before the next test's devices start, or this test fails.
    assertThat(odometry.isAlive()).as("the odometry thread, 2 s after it was stopped").isFalse();
  }

  private static SwerveWiring.ModuleWiring wiring(int drive, int steer, int encoder) {
    return TestWiring.talons(CanBus.SYSTEMCORE, drive, steer, encoder);
  }

  @Test
  void theWheelTurnsByItsGearingLessWhatTurningTheModulePutThere() throws InterruptedException {
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(wiring(1, 2, 3), BUS, GEOMETRY.module(0), COUPLING, odometry, STARTUP);
    io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());
    odometry.start();
    double gearing = GEOMETRY.module(0).driveGearing().ratio();

    new TalonFX(1, BUS).getSimState().setRawRotorPosition(2 * gearing); // two wheel turns' worth
    new CANcoder(3, BUS).getSimState().setRawPosition(0.25); // the module turned a quarter
    // The steering motor passes the CANcoder's angle on a moment after the CANcoder reports it, so
    // the readings settle over a few frames.
    double expected = 2 * Math.PI * (2 - COUPLING * 0.25 / gearing);
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    Waits.until(
        "the readings settle where the gearing and coupling put them",
        () -> io.updateInputs(inputs),
        () -> {
          int last = inputs.odometryDrivePositionsRad.length - 1;
          return last >= 0
              && Math.abs(inputs.drivePositionRad - expected) < 0.01
              && Math.abs(inputs.odometryDrivePositionsRad[last] - expected) < 0.01
              && Math.abs(inputs.odometrySteerAnglesRad[last] - Math.PI / 2) < 0.01;
        });
    assertThat(inputs.steerAngle.getDegrees()).isCloseTo(90, within(0.5));
    assertThat(inputs.odometryTimestampsNanos).hasSameSizeAs(inputs.odometryDrivePositionsRad);
    assertThat(inputs.unconfigured).as("every device took its configuration").isEmpty();
  }

  /**
   * What Phoenix does with a motor's supply current when the code doesn't say: a supply limit is on
   * (70 A), and it steps down to 40 A once the motor has been held at the limit for 1 s. The
   * comments in {@link ModuleIOTalonFX} and {@code DrivetrainControl.Limits} say so; if a Phoenix
   * update changes these defaults, change those comments.
   */
  @Test
  void phoenixStepsTheSupplyLimitDownTo40AmpsAfterASecondUnlessToldOtherwise() {
    CurrentLimitsConfigs defaults = new CurrentLimitsConfigs();

    assertThat(defaults.SupplyCurrentLimitEnable).isTrue();
    assertThat(defaults.SupplyCurrentLimit).isEqualTo(70);
    assertThat(defaults.SupplyCurrentLowerLimit).isEqualTo(40);
    assertThat(defaults.SupplyCurrentLowerTime).isEqualTo(1.0);
  }

  /**
   * The drive motors' supply limit, and its step-down, come from the configuration: at startup, and
   * each time tuning changes them. The values differ from Phoenix's defaults (40 A after 1 s), so
   * this fails if the lines that set them are removed and the defaults silently return.
   */
  @Test
  void theDriveMotorsSupplyLimitStepsDownAsConfiguredAtStartupAndWhenTuned()
      throws InterruptedException {
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(
            wiring(11, 12, 13), BUS, GEOMETRY.module(0), COUPLING, odometry, STARTUP);
    TalonFX drive = new TalonFX(11, BUS);

    io.configure(DRIVETRAIN.gains(), stepDown(55, 35, 0.5));
    assertSupplyLimit(drive, 55, 35, 0.5);

    io.configure(DRIVETRAIN.gains(), stepDown(50, 30, 0.8)); // tuned on the running robot
    assertSupplyLimit(drive, 50, 30, 0.8);

    io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits()); // back to the template's
    DrivetrainControl.Limits template = DRIVETRAIN.limits();
    assertSupplyLimit(
        drive,
        template.driveSupplyCurrent().in(Amps),
        template.driveSupplyLowerCurrent().in(Amps),
        template.driveSupplyLowerTime().in(Seconds));
  }

  /**
   * Tuner X's stock steering gains, typed as Tuner X shows them, per turn of the module, reach the
   * steering motor's slot as the same numbers: the steering's TalonFX runs on its CANcoder, a turn
   * of the module a turn of its sensor, so its slot is per turn of the module too. Held in volts
   * per radian between, a gain typed with its unit comes back out as it went in; typed as a plain
   * number, 100 would be 100 per radian, 2π times too hot.
   */
  @Test
  void tunerXsStockSteeringGainsReachTheSteeringMotorAsTunerXShowsThem()
      throws InterruptedException {
    record Row(String gain, ToDoubleFunction<Slot0Configs> read, double tunerX) {}
    List<Row> rows =
        List.of(new Row("kP", slot -> slot.kP, 100), new Row("kD", slot -> slot.kD, 0.5));
    DrivetrainControl typed =
        DrivetrainControl.defaults()
            .withSteerKp(Volts.per(Rotation).of(100))
            .withSteerKd(Volts.per(RotationsPerSecond).of(0.5));
    DrivetrainControl seeded = GEOMETRY.control(typed).control().orElseThrow();
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(
            wiring(17, 18, 19), BUS, GEOMETRY.module(0), COUPLING, odometry, STARTUP);
    TalonFX steer = new TalonFX(18, BUS);
    Slot0Configs slot = new Slot0Configs();

    io.configure(seeded.gains(), seeded.limits());

    assertThat(steer.getConfigurator().refresh(slot, 1.0).isOK()).as("read back").isTrue();
    for (Row row : rows) {
      assertThat(row.read().applyAsDouble(slot))
          .as("the steering's slot %s", row.gain())
          .isCloseTo(row.tunerX(), within(1e-9));
    }
  }

  /**
   * The steering has no supply limit, so no step-down: its stator limit caps what it draws from the
   * battery. Phoenix would turn one on by default, so this fails if the line turning it off goes.
   */
  @Test
  void theSteeringHasNoSupplyLimitAndSoNoStepDown() throws InterruptedException {
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(
            wiring(14, 15, 16), BUS, GEOMETRY.module(0), COUPLING, odometry, STARTUP);
    TalonFX steer = new TalonFX(15, BUS);
    CurrentLimitsConfigs read = new CurrentLimitsConfigs();

    io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());
    assertThat(steer.getConfigurator().refresh(read, 1.0).isOK()).as("read back").isTrue();
    assertThat(read.SupplyCurrentLimitEnable).as("at startup").isFalse();
    assertThat(read.StatorCurrentLimitEnable).isTrue();

    io.configure(
        DRIVETRAIN.gains(),
        DrivetrainControlLimitsBuilder.builder(DRIVETRAIN.limits())
            .steerStatorCurrent(Amps.of(35))
            .build());
    Waits.until(
        "the tuned limit reached the steering",
        () -> steer.getConfigurator().refresh(read, 1.0),
        () -> Math.abs(read.StatorCurrentLimit - 35) < 0.01);
    assertThat(read.SupplyCurrentLimitEnable).as("after tuning").isFalse();
  }

  /** The template's limits, with the drive's supply limit and its step-down changed. */
  private static DrivetrainControl.Limits stepDown(
      double amps, double lowerAmps, double lowerSeconds) {
    return DrivetrainControlLimitsBuilder.builder(DRIVETRAIN.limits())
        .driveSupplyCurrent(Amps.of(amps))
        .driveSupplyLowerCurrent(Amps.of(lowerAmps))
        .driveSupplyLowerTime(Seconds.of(lowerSeconds))
        .build();
  }

  /**
   * Waits for the motor to report this supply limit and step-down (a change while running is sent
   * without waiting), then checks the limit is on. The motor keeps the time to about 0.02 s: 0.75 s
   * reads back as 0.76.
   */
  private static void assertSupplyLimit(
      TalonFX motor, double amps, double lowerAmps, double lowerSeconds)
      throws InterruptedException {
    CurrentLimitsConfigs read = new CurrentLimitsConfigs();
    Waits.until(
        () ->
            "wants "
                + amps
                + " A, then "
                + lowerAmps
                + " A after "
                + lowerSeconds
                + " s; the drive motor has "
                + read.SupplyCurrentLimit
                + " A, "
                + read.SupplyCurrentLowerLimit
                + " A, "
                + read.SupplyCurrentLowerTime
                + " s",
        () -> motor.getConfigurator().refresh(read, 1.0),
        () ->
            Math.abs(read.SupplyCurrentLimit - amps) < 0.01
                && Math.abs(read.SupplyCurrentLowerLimit - lowerAmps) < 0.01
                && Math.abs(read.SupplyCurrentLowerTime - lowerSeconds) < 0.01);
    assertThat(read.SupplyCurrentLimitEnable).as("the supply limit is on").isTrue();
  }

  /**
   * A module's drive motor that stops answering: its next reading doesn't come, past two periods in
   * transit, and reads no number, so odometry leaves the wheel out of that reading, while the
   * module still reads connected, its silence not yet half a second long. Read again, it's a number
   * again. Until 2026-10-08 the reading was the last the motor sent, its wheel standing still, for
   * the half second before its module read disconnected.
   */
  @Test
  void aReadingThatDidntComeIsNoNumberBeforeItsModuleReadsDisconnected()
      throws InterruptedException {
    CountDownLatch read = new CountDownLatch(1);
    Deque<PhoenixOdometryThread.Batch> batches =
        new ArrayDeque<>(
            List.of(odometryBatch(1, true), odometryBatch(2, false), odometryBatch(3, true)));
    PhoenixOdometryThread scripted =
        new PhoenixOdometryThread(
            signals -> {
              PhoenixOdometryThread.Batch next = batches.poll();
              if (next == null) {
                read.countDown();
                new CountDownLatch(1).await(); // until the test stops the thread
                throw new InterruptedException();
              }
              return next;
            },
            message -> {});
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(
            wiring(21, 22, 23), BUS, GEOMETRY.module(0), COUPLING, scripted, STARTUP);
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    scripted.start();
    try {
      assertThat(read.await(Waits.LIMIT.toSeconds(), TimeUnit.SECONDS))
          .as("waited 30 s for every batch read")
          .isTrue();
      scripted.lock().lock();
      try {
        io.updateInputs(inputs);
      } finally {
        scripted.lock().unlock();
      }
    } finally {
      scripted.interrupt();
      scripted.join(2000);
    }
    assertThat(scripted.isAlive()).as("the scripted thread, 2 s after it was stopped").isFalse();

    assertThat(inputs.driveConnected).as("not silent half a second yet").isTrue();
    assertThat(inputs.odometryDrivePositionsRad).hasSize(3);
    assertThat(inputs.odometryDrivePositionsRad[0]).isFinite();
    assertThat(inputs.odometryDrivePositionsRad[1]).isNaN();
    assertThat(inputs.odometryDrivePositionsRad[2]).isFinite();
    assertThat(inputs.odometrySteerAnglesRad).containsExactly(0, 0, 0);
    assertThat(inputs.steerStatorCurrentAmps).as("its readings' peak").isGreaterThanOrEqualTo(1);
  }

  /**
   * One read of a module's odometry signals, in the order it registers them (its drive's position,
   * its steering's, its steering's current), the drive at so many rotations: fresh, or its device
   * silent, its reading 20 ms in transit.
   */
  private static PhoenixOdometryThread.Batch odometryBatch(double driveRotations, boolean came) {
    return new PhoenixOdometryThread.Batch(
        (long) (driveRotations * 1e9),
        List.of(
            new PhoenixOdometryThread.Reading(driveRotations, true, came ? 0.002 : 0.02),
            new PhoenixOdometryThread.Reading(0, true, 0.002),
            new PhoenixOdometryThread.Reading(1, true, 0.002)));
  }

  /** A main loop that fell behind gets the newest readings, not stale ones. */
  @Test
  void aFullQueueMakesRoomByDroppingTheOldestReading() {
    Queue<Integer> queue = new ArrayBlockingQueue<>(3);
    for (int reading = 1; reading <= 5; reading++) {
      PhoenixOdometryThread.keepNewest(queue, reading);
    }

    assertThat(queue).containsExactly(3, 4, 5);
  }

  /** The steering's current is the loop's peak, either way, from every reading since the last. */
  @Test
  void theSteeringsCurrentIsTheMostAmongTheLoopsReadings() {
    Queue<Double> readings = new ArrayDeque<>(List.of(3.0, -40.0, 12.0));

    assertThat(ModuleIOTalonFX.mostAmps(readings, 5)).isEqualTo(40);
    assertThat(readings).as("taken").isEmpty();
    assertThat(ModuleIOTalonFX.mostAmps(readings, -2)).as("no readings: the latest").isEqualTo(2);
    assertThat(ModuleIOTalonFX.mostAmps(new ArrayDeque<>(List.of(3.0, Double.NaN)), 2))
        .as("one that didn't come passed over")
        .isEqualTo(3);
  }

  @Test
  void theCouplingIsTheWheelTurnsTheModulesTurnPutsOnTheDriveEncoder() {
    // A quarter turn of the module with the wheel still turns the drive motor 3.5 / 4 times.
    assertThat(ModuleIOTalonFX.wheelRotations(3.5 * 0.25 / 6.75, 0.25, 3.5, 6.75))
        .isCloseTo(0, within(1e-12));
    assertThat(ModuleIOTalonFX.wheelRotations(2, 0, 3.5, 6.75)).isEqualTo(2);
  }

  @Test
  void theGyroPassesItsAnglesThroughInWpilibsConvention() throws InterruptedException {
    GyroIOPigeon2 io = new GyroIOPigeon2(20, BUS, 5, 1.0, Rotation3d.ZERO, odometry);
    odometry.start();

    var pigeon = new Pigeon2(20, BUS).getSimState();
    pigeon.setRawYaw(90);
    pigeon.setPitch(-5); // nose up
    pigeon.setRoll(3);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
    Waits.until(
        "the simulated gyro answered",
        () -> io.updateInputs(inputs),
        () ->
            Math.abs(inputs.yaw.getDegrees() - 90) < 0.1
                && Math.abs(Math.toDegrees(inputs.rollRad) - 3) < 0.1
                && inputs.odometryYawsRad.length > 0);
    assertThat(Math.toDegrees(inputs.pitchRad)).isCloseTo(-5, within(0.1));
    assertThat(inputs.odometryYawsRad[inputs.odometryYawsRad.length - 1])
        .isCloseTo(Math.PI / 2, within(0.002));
  }

  /**
   * As the robot starts, the Pigeon is sent its whole configuration: its mount pose is the
   * definition's orientation, in degrees, and whatever else was on it (a mount pose or a gyro trim
   * from Tuner X or other code) is Phoenix's default again.
   */
  @Test
  void thePigeonTakesItsMountFromTheDefinitionAndNothingLeftOnIt() throws InterruptedException {
    Pigeon2 pigeon = new Pigeon2(20, BUS);
    // Rows: the orientation, and the mount pose it is sent as: yaw, pitch, roll.
    List<Mount> rows =
        List.of(
            new Mount(Rotation3d.ZERO, 0, 0, 0),
            new Mount(new Rotation3d(0, 0, Math.PI / 2), 90, 0, 0),
            new Mount(new Rotation3d(Math.PI, 0, 0), 0, 0, 180),
            new Mount(new Rotation3d(0, Math.toRadians(-10), 0), 0, -10, 0));
    for (Mount row : rows) {
      Pigeon2Configuration stale = new Pigeon2Configuration();
      stale.MountPose.MountPoseRoll = 7;
      stale.GyroTrim.GyroScalarZ = 3;
      assertThat(pigeon.getConfigurator().apply(stale, 1.0).isOK()).as("left on it").isTrue();

      new GyroIOPigeon2(20, BUS, 5, 1.0, row.orientation(), odometry);
      Pigeon2Configuration read = new Pigeon2Configuration();
      Waits.until(
          row + ": the Pigeon's configuration read back",
          () -> {},
          () -> pigeon.getConfigurator().refresh(read, 1.0).isOK());
      assertThat(read.MountPose.MountPoseYaw).as("%s yaw", row).isCloseTo(row.yaw(), within(0.01));
      assertThat(read.MountPose.MountPosePitch)
          .as("%s pitch", row)
          .isCloseTo(row.pitch(), within(0.01));
      assertThat(Math.abs(read.MountPose.MountPoseRoll))
          .as("%s roll", row)
          .isCloseTo(row.roll(), within(0.01));
      assertThat(read.GyroTrim.GyroScalarZ).as("%s: no trim left", row).isZero();
    }
  }

  /** An orientation, and the Pigeon's mount pose for it, in degrees. */
  private record Mount(Rotation3d orientation, double yaw, double pitch, double roll) {}

  /**
   * A Pigeon that restarts counts its yaw from zero again. The loop it says so, the heading holds
   * where it was, every high-rate reading too; from its next reading on, the heading goes on from
   * there: at 30° when it restarted, its new count's 10° reads 40°, never 0° or 10°.
   */
  @Test
  void aPigeonThatRestartsIsTakenUpWhereTheHeadingStood() throws InterruptedException {
    boolean[] startedAgain = {false};
    GyroIOPigeon2 io =
        new GyroIOPigeon2(
            20,
            BUS,
            5,
            1.0,
            Rotation3d.ZERO,
            odometry,
            pigeon -> {
              BooleanSupplier phoenix = pigeon.getResetOccurredChecker();
              return () -> {
                boolean restarted = startedAgain[0];
                startedAgain[0] = false;
                return phoenix.getAsBoolean() || restarted;
              };
            });
    odometry.start();
    var pigeon = new Pigeon2(20, BUS).getSimState();
    pigeon.setRawYaw(30);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
    Waits.until(
        "the yaw read at 30°",
        () -> readUnderTheLock(io, inputs),
        () -> Math.abs(inputs.yaw.getDegrees() - 30) < 0.1);

    // It restarts, and counts from zero: the robot hears it say so once its new count has come.
    // Phoenix's simulated Pigeon takes a frame or two to send a yaw it is set to.
    pigeon.setRawYaw(0);
    StatusSignal<Angle> yaw = new Pigeon2(20, BUS).getYaw();
    Waits.until(
        "the Pigeon's yaw counting from 0 again",
        yaw::refresh,
        () -> Math.abs(yaw.getValueAsDouble()) < 0.1);
    Thread.sleep(50);
    startedAgain[0] = true;
    readUnderTheLock(io, inputs);
    boolean saidRestarted = inputs.restarted;
    double heldDegrees = inputs.yaw.getDegrees();
    double[] heldYaws = inputs.odometryYawsRad.clone();
    readUnderTheLock(io, inputs);
    boolean saidOnce = !inputs.restarted;
    pigeon.setRawYaw(10);
    Waits.until(
        () -> "10° of its new count, on from 30°: read " + inputs.yaw.getDegrees(),
        () -> readUnderTheLock(io, inputs),
        () ->
            Math.abs(inputs.yaw.getDegrees() - 40) < 0.1
                && inputs.odometryYawsRad.length > 0
                && Math.abs(
                        Math.toDegrees(inputs.odometryYawsRad[inputs.odometryYawsRad.length - 1])
                            - 40)
                    < 0.1);
    assertThat(saidRestarted).as("the loop it restarted").isTrue();
    assertThat(saidOnce).as("in one loop only").isTrue();
    assertThat(heldDegrees).as("held").isCloseTo(30, within(0.1));
    for (double held : heldYaws) {
      assertThat(Math.toDegrees(held)).as("each reading held").isCloseTo(30, within(0.1));
    }
  }

  private void readUnderTheLock(GyroIOPigeon2 io, GyroIOInputsAutoLogged inputs) {
    odometry.lock().lock();
    try {
      io.updateInputs(inputs);
    } finally {
      odometry.lock().unlock();
    }
  }

  /**
   * The accelerometer reads along the Pigeon's case, whatever its mount pose: turned a quarter turn
   * left on the robot, it feels the robot speed up forward on its -y axis; mounted logo down, it
   * feels a push left on its -y axis and the floor holding it up on its -z axis. Turned by the
   * orientation the robot's definition gives it, both read along the robot's axes again.
   */
  @Test
  void theAccelerometerIsTurnedFromThePigeonsCaseToTheRobotsAxes() {
    Rotation3d turnedLeft = new Rotation3d(0, 0, Math.PI / 2);
    Rotation3d logoDown = new Rotation3d(Math.PI, 0, 0);

    Translation3d forward = GyroIOPigeon2.robotAxes(turnedLeft, 0, -1, 0);
    Translation3d left = GyroIOPigeon2.robotAxes(logoDown, 0, -1, -9.81);

    assertThat(forward.getDistance(new Translation3d(1, 0, 0))).isLessThan(1e-12);
    assertThat(left.getDistance(new Translation3d(0, 1, 9.81))).isLessThan(1e-12);
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The CTRE IO against Phoenix's own simulated devices: set what a sensor reads, and check what the
 * robot code gets. Conversions, the coupling, and the odometry thread; the control loops belong to
 * the devices.
 */
@ExtendWith(RobotSim.class)
class PhoenixIOTest {
  private static final CANBus BUS = new CANBus("drivetrain");
  private static final DrivetrainGeometry GEOMETRY = ExampleRobot.SIM.definition().drivetrain();
  private static final DrivetrainControl DRIVETRAIN = ExampleRobot.SIM.drivetrain();
  private static final double COUPLING = 3.5;

  private final PhoenixOdometryThread odometry = new PhoenixOdometryThread();

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false); // as PhoenixDrivetrain does on the robot
  }

  @AfterEach
  void tearDown() throws InterruptedException {
    odometry.interrupt();
    odometry.join(2000); // stopped before the next test's devices start
  }

  private static SwerveWiring.ModuleWiring wiring(int drive, int steer, int encoder) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(drive)
        .steerId(steer)
        .encoderOffset(Rotations.of(0))
        .driveInverted(false)
        .steerInverted(false)
        .encoderInverted(false)
        .encoderId(encoder)
        .build();
  }

  /** Waits up to two seconds, reading again every few milliseconds, for something to be true. */
  private static boolean eventually(Runnable read, BooleanSupplier done)
      throws InterruptedException {
    for (int i = 0; i < 200; i++) {
      read.run();
      if (done.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10);
    }
    return false;
  }

  @Test
  void theWheelTurnsByItsGearingLessWhatTurningTheModulePutThere() throws InterruptedException {
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(wiring(1, 2, 3), BUS, GEOMETRY.module(0), COUPLING, odometry);
    io.configure(DRIVETRAIN.gains(), DRIVETRAIN.limits());
    odometry.start();
    double gearing = GEOMETRY.module(0).driveGearing().ratio();

    new TalonFX(1, BUS).getSimState().setRawRotorPosition(2 * gearing); // two wheel turns' worth
    new CANcoder(3, BUS).getSimState().setRawPosition(0.25); // the module turned a quarter
    // The steering motor passes the CANcoder's angle on a moment after the CANcoder reports it, so
    // the readings settle over a few frames.
    double expected = 2 * Math.PI * (2 - COUPLING * 0.25 / gearing);
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    boolean settled =
        eventually(
            () -> io.updateInputs(inputs),
            () -> {
              int last = inputs.odometryDrivePositionsRad.length - 1;
              return last >= 0
                  && Math.abs(inputs.drivePositionRad - expected) < 0.01
                  && Math.abs(inputs.odometryDrivePositionsRad[last] - expected) < 0.01
                  && Math.abs(inputs.odometrySteerAnglesRad[last] - Math.PI / 2) < 0.01;
            });

    assertThat(settled).as("the readings settle where the gearing and coupling put them").isTrue();
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
        new ModuleIOTalonFX(wiring(11, 12, 13), BUS, GEOMETRY.module(0), COUPLING, odometry);
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
   * The steering has no supply limit, so no step-down: its stator limit caps what it draws from the
   * battery. Phoenix would turn one on by default, so this fails if the line turning it off goes.
   */
  @Test
  void theSteeringHasNoSupplyLimitAndSoNoStepDown() throws InterruptedException {
    ModuleIOTalonFX io =
        new ModuleIOTalonFX(wiring(14, 15, 16), BUS, GEOMETRY.module(0), COUPLING, odometry);
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
    boolean applied =
        eventually(
            () -> steer.getConfigurator().refresh(read, 1.0),
            () -> Math.abs(read.StatorCurrentLimit - 35) < 0.01);
    assertThat(applied).as("the tuned limit reached the steering").isTrue();
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
    boolean reported =
        eventually(
            () -> motor.getConfigurator().refresh(read, 1.0),
            () ->
                Math.abs(read.SupplyCurrentLimit - amps) < 0.01
                    && Math.abs(read.SupplyCurrentLowerLimit - lowerAmps) < 0.01
                    && Math.abs(read.SupplyCurrentLowerTime - lowerSeconds) < 0.01);
    assertThat(reported)
        .as(
            "wants %s A, then %s A after %s s; the drive motor has %s A, %s A, %s s",
            amps,
            lowerAmps,
            lowerSeconds,
            read.SupplyCurrentLimit,
            read.SupplyCurrentLowerLimit,
            read.SupplyCurrentLowerTime)
        .isTrue();
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
        new ModuleIOTalonFX(wiring(21, 22, 23), BUS, GEOMETRY.module(0), COUPLING, scripted);
    ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    scripted.start();
    try {
      assertThat(read.await(2, TimeUnit.SECONDS)).as("read every batch").isTrue();
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

  @Test
  void configuringAtStartupTriesAFewTimesThenSaysItFailed() {
    int[] attempts = {0};

    boolean failed =
        ModuleIOTalonFX.tryUntilOk(
            () -> {
              attempts[0]++;
              return StatusCode.EcuIsNotPresent;
            });
    boolean ok = ModuleIOTalonFX.tryUntilOk(() -> StatusCode.OK);

    assertThat(failed).isFalse();
    assertThat(attempts[0]).isEqualTo(5);
    assertThat(ok).isTrue();
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
    GyroIOPigeon2 io = new GyroIOPigeon2(20, BUS, 1.0, Rotation3d.ZERO, odometry);
    odometry.start();

    var pigeon = new Pigeon2(20, BUS).getSimState();
    pigeon.setRawYaw(90);
    pigeon.setPitch(-5); // nose up
    pigeon.setRoll(3);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
    boolean read =
        eventually(
            () -> io.updateInputs(inputs),
            () ->
                Math.abs(inputs.yaw.getDegrees() - 90) < 0.1
                    && Math.abs(Math.toDegrees(inputs.rollRad) - 3) < 0.1
                    && inputs.odometryYawsRad.length > 0);

    assertThat(read).as("the simulated gyro answered").isTrue();
    assertThat(Math.toDegrees(inputs.pitchRad)).isCloseTo(-5, within(0.1));
    assertThat(inputs.odometryYawsRad[inputs.odometryYawsRad.length - 1])
        .isCloseTo(Math.PI / 2, within(0.002));
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

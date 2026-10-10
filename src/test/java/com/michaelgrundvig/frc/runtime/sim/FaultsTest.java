package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.simulation.FaultCatalog.BACKUP_IMU;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.GYRO;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO.ModuleIOInputs;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.KinematicPhysics;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.system.RobotController;

/**
 * The simulated world's live faults, as the page applies them: each changes what the robot's code
 * reads from its sensors (or where the robot really is), and undoing it puts that back. The robot's
 * code reads only its IO's inputs, so these tests read the same. The cameras' faults are in {@code
 * CameraFaultsSimTest}.
 */
@ExtendWith(RobotSim.class)
class FaultsTest {
  /** Where the robot stands: on blue's side, clear of everything. */
  private static final Pose2d STANDING = new Pose2d(2.0, 4.0, Rotation2d.ZERO);

  private final List<SimulatedWorld> worlds = new ArrayList<>();
  private final List<String> said = new ArrayList<>();
  private Components components = new Components(said::add);

  @AfterEach
  void tearDown() {
    worlds.forEach(SimulatedWorld::close);
  }

  /** A world with sensors exactly as configured, its parts declared, the robot standing. */
  private SimulatedWorld world() {
    return world(SensorErrors.NONE);
  }

  /** A world with sensors that err like this, its parts declared, the robot standing. */
  private SimulatedWorld world(SensorErrors errors) {
    SimulatedWorld built =
        new SimulatedWorld(RobotSim.FIELD, RobotSim.WITHOUT_CAMERAS.definition(), errors);
    worlds.add(built);
    components = new Components(said::add);
    built.addTo(components);
    built.place(STANDING);
    step(built);
    return built;
  }

  private static void step(SimulatedWorld world) {
    RobotSim.stepClock();
    world.step(RobotController.getTime());
  }

  private void apply(String component, String fault, Map<String, Double> values) {
    assertThat(components.apply(Setting.of(component, fault, values))).isNull();
  }

  private void apply(String component, String fault) {
    apply(component, fault, Map.of());
  }

  private void undo(String component, String fault) {
    assertThat(components.undo(component, fault)).isNull();
  }

  private static GyroIO imu(SimulatedWorld world, String name) {
    return name.equals(GYRO) ? world.drivetrain().gyro() : world.drivetrain().backupGyro();
  }

  /** Turns the robot in place without its wheels: its IMUs feel it. */
  private static void turn(SimulatedWorld world, double degrees) {
    world.drivetrain().shove(new Transform2d(0, 0, Rotation2d.fromDegrees(degrees)));
    step(world);
  }

  @Test
  void asBuiltNothingIsOn() {
    world();

    assertThat(components.on()).isEmpty();
    assertThat(components.components())
        .extracting(c -> c.name())
        .containsExactly(
            GYRO,
            BACKUP_IMU,
            FaultCatalog.module(0),
            FaultCatalog.module(1),
            FaultCatalog.module(2),
            FaultCatalog.module(3),
            ROBOT,
            FaultCatalog.FIELD);
  }

  /**
   * A tag moved: the cameras see it there from now on, and moving another puts the first back.
   * Undone, every tag is where the field was built, a little off its layout if it was built so.
   */
  @Test
  void aTagMovedIsSeenThereUntilPutBack() {
    Translation3d built = new Translation3d(0.01, 0, 0);
    SimulatedWorld world = world(SensorErrors.NONE.withTagErrors(Map.of(26, built)));
    Pose3d layout = RobotSim.FIELD.tag(26).orElseThrow();
    Pose3d other = RobotSim.FIELD.tag(25).orElseThrow();

    apply(
        FaultCatalog.FIELD,
        FaultCatalog.TAG_MOVED,
        Map.of(FaultCatalog.TAG, 26.0, "X", 4.0, "Y", 0.0, "Up", -2.0));
    Pose3d moved = world.vision().tagPose(26).orElseThrow();
    apply(
        FaultCatalog.FIELD,
        FaultCatalog.TAG_MOVED,
        Map.of(FaultCatalog.TAG, 25.0, "X", 0.0, "Y", 4.0, "Up", 0.0));
    Pose3d first = world.vision().tagPose(26).orElseThrow();
    Pose3d second = world.vision().tagPose(25).orElseThrow();
    undo(FaultCatalog.FIELD, FaultCatalog.TAG_MOVED);

    Translation3d expected = built.plus(new Translation3d(0.1016, 0, -0.0508));
    assertThat(moved.getTranslation().minus(layout.getTranslation()).getDistance(expected))
        .isLessThan(1e-9);
    assertThat(first.getTranslation().minus(layout.getTranslation()).getDistance(built))
        .as("put back where it was built")
        .isLessThan(1e-9);
    assertThat(second.getY() - other.getY()).isCloseTo(0.1016, within(1e-9));
    assertThat(world.vision().tagPose(25).orElseThrow()).isEqualTo(other);
    assertThat(components.on()).isEmpty();
  }

  @Test
  void aTagTheFieldDoesntHaveIsRefused() {
    world();

    assertThat(
            components.apply(
                Setting.of(
                    FaultCatalog.FIELD,
                    FaultCatalog.TAG_MOVED,
                    Map.of(FaultCatalog.TAG, 26.5, "X", 4.0))))
        .contains("whole number");
    assertThat(components.on()).isEmpty();
  }

  @ParameterizedTest
  @ValueSource(strings = {GYRO, BACKUP_IMU})
  void anUnpluggedImuReadsNothingUntilPluggedBackIn(String name) {
    SimulatedWorld world = world();
    read(imu(world, name)); // what it read before

    apply(name, FaultCatalog.UNPLUGGED);
    step(world);
    GyroIO.GyroIOInputs unplugged = read(imu(world, name));
    undo(name, FaultCatalog.UNPLUGGED);
    step(world);

    assertThat(unplugged.connected).isFalse();
    assertThat(unplugged.odometryYawsRad).isEmpty();
    assertThat(read(imu(world, name)).connected).isTrue();
    assertThat(components.on()).isEmpty();
  }

  @ParameterizedTest
  @ValueSource(strings = {GYRO, BACKUP_IMU})
  void aFrozenImuMissesTurnsUntilUndone(String name) {
    SimulatedWorld world = world();

    apply(name, FaultCatalog.FROZEN);
    turn(world, 30);
    double frozen = read(imu(world, name)).yaw.getDegrees();
    undo(name, FaultCatalog.FROZEN);
    turn(world, 10);

    assertThat(frozen).isCloseTo(0, within(1e-9));
    assertThat(read(imu(world, name)).yaw.getDegrees()).isCloseTo(40, within(1e-9));
  }

  /** A scale error changes the turns it reads from then on, without a jump; undoing it too. */
  @ParameterizedTest
  @ValueSource(strings = {GYRO, BACKUP_IMU})
  void anImuErringReadsTurnsByItsNewScaleFromNowOn(String name) {
    SimulatedWorld world = world();
    turn(world, 20);

    apply(name, FaultCatalog.ERRORS, Map.of(FaultCatalog.SCALE, 1.5));
    step(world);
    double atTheChange = read(imu(world, name)).yaw.getDegrees();
    turn(world, 30);
    double erring = read(imu(world, name)).yaw.getDegrees();
    undo(name, FaultCatalog.ERRORS);
    turn(world, 30);

    assertThat(atTheChange).as("no jump").isCloseTo(20, within(1e-9));
    assertThat(erring).isCloseTo(20 + 45, within(1e-9));
    assertThat(read(imu(world, name)).yaw.getDegrees()).isCloseTo(20 + 45 + 30, within(1e-9));
  }

  @ParameterizedTest
  @ValueSource(strings = {GYRO, BACKUP_IMU})
  void aResetImuReadsZeroWhereverTheRobotPoints(String name) {
    SimulatedWorld world = world();
    turn(world, 30);

    apply(name, FaultCatalog.RESET);
    step(world);
    double reset = read(imu(world, name)).yaw.getDegrees();
    turn(world, 10);

    assertThat(reset).isCloseTo(0, within(1e-9));
    assertThat(read(imu(world, name)).yaw.getDegrees()).isCloseTo(10, within(1e-9));
    assertThat(components.undo(name, FaultCatalog.RESET))
        .as("an event isn't undone")
        .contains("happens once, and nothing undoes it");
  }

  /**
   * A stale module's readings stop while its wheel turns and steers, and catch up once it's undone.
   */
  @Test
  void aStaleModuleStopsCountingUntilItCatchesUp() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(0);
    drive(world, 3.0, 0.2);

    apply(FaultCatalog.module(0), FaultCatalog.STALE);
    ModuleIOInputs before = read(module);
    module.configure(ExampleRobot.SIM.drivetrain().gains(), ExampleRobot.SIM.drivetrain().limits());
    module.setSteerAngle(Rotation2d.fromDegrees(30));
    drive(world, 3.0, 0.5);
    ModuleIOInputs stale = read(module);
    drive(world, 3.0, 0.1);
    double truth = world.drivetrain().physics().wheel(0).rotationRadians();
    undo(FaultCatalog.module(0), FaultCatalog.STALE);
    step(world);
    ModuleIOInputs caughtUp = read(module);

    assertThat(stale.drivePositionRad).isEqualTo(before.drivePositionRad);
    assertThat(stale.odometryDrivePositionsRad).containsOnly(before.drivePositionRad);
    assertThat(stale.steerAngle).isEqualTo(before.steerAngle);
    assertThat(read(world.drivetrainHardware().modules().get(1)).drivePositionRad)
        .as("the others count on")
        .isGreaterThan(before.drivePositionRad + 1);
    assertThat(caughtUp.drivePositionRad).as("caught up").isGreaterThan(truth);
    assertThat(caughtUp.steerAngle.getDegrees())
        .as("steered while stale")
        .isCloseTo(30, within(1.0));
  }

  /** Stale while it steers, a module reads the steering's speed it last heard, and holds it. */
  @Test
  void aStaleModuleHoldsTheSteeringSpeedItLastHeard() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(0);
    module.configure(ExampleRobot.SIM.drivetrain().gains(), ExampleRobot.SIM.drivetrain().limits());
    module.setSteerAngle(Rotation2d.fromDegrees(90));
    step(world);
    step(world); // turning

    apply(FaultCatalog.module(0), FaultCatalog.STALE);
    double heard = read(module).steerVelocityRadPerSec;
    drive(world, 0, 0.5); // the module settles at 90°, unheard
    double held = read(module).steerVelocityRadPerSec;

    assertThat(heard).as("turning when it went stale").isNotZero();
    assertThat(held).isEqualTo(heard);
  }

  /**
   * Undone, a fault puts a season-worn robot back as it was built, its errors and all: not perfect.
   */
  @Test
  void undoingPutsASeasonWornRobotBackAsBuilt() {
    SimulatedWorld world = world(SensorErrors.TYPICAL);
    apply(
        GYRO,
        FaultCatalog.ERRORS,
        Map.of(FaultCatalog.SCALE, 1.2, FaultCatalog.DRIFT, 0.0, FaultCatalog.NOISE, 0.0));
    apply(FaultCatalog.module(0), FaultCatalog.ZEROED_OFF, Map.of(FaultCatalog.OFFSET, 10.0));
    apply(FaultCatalog.module(1), FaultCatalog.WORN, Map.of(FaultCatalog.SCALE, 0.9));
    assertThat(components.on()).hasSize(3);

    assertThat(components.undoAll()).isNull();

    DriveSimulation drivetrain = world.drivetrain();
    assertThat(drivetrain.gyroSim().errors()).isEqualTo(SensorErrors.TYPICAL.gyro());
    assertThat(Math.toDegrees(drivetrain.moduleSim(0).steerOffsetRadians()))
        .isCloseTo(SensorErrors.TYPICAL.steerOffsetDegrees().get(0), within(1e-9));
    assertThat(drivetrain.treadScale(1)).isEqualTo(SensorErrors.TYPICAL.wheelRadiusScale().get(1));
    assertThat(components.on()).isEmpty();
  }

  /** Applied with the values the part was built with, a lasting fault changes nothing: refused. */
  @Test
  void aFaultAsBuiltChangesNothing() {
    world(SensorErrors.TYPICAL);

    assertThat(components.apply(Setting.of(FaultCatalog.module(2), FaultCatalog.ZEROED_OFF)))
        .isEqualTo("Back left module: Zeroed off changes nothing: as built");
    assertThat(components.on()).isEmpty();
  }

  /** A zero knocked off: the module reads that far from where it points, until undone. */
  @Test
  void aZeroKnockedOffShiftsWhatTheModuleReads() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(2);
    double before = read(module).steerAngle.getDegrees();

    apply(FaultCatalog.module(2), FaultCatalog.ZEROED_OFF, Map.of(FaultCatalog.OFFSET, 10.0));
    step(world);
    double off = read(module).steerAngle.getDegrees();
    undo(FaultCatalog.module(2), FaultCatalog.ZEROED_OFF);
    step(world);

    assertThat(off - before).isCloseTo(-10, within(1e-6));
    assertThat(read(module).steerAngle.getDegrees()).isCloseTo(before, within(1e-6));
  }

  /** Worn tread: the robot goes less far than its encoders count, until it's new again. */
  @Test
  void wornTreadMovesTheRobotLessThanItsEncodersCount() {
    double fresh = drive(world(), 3.0, 1.0);
    SimulatedWorld worn = world();
    for (int i = 0; i < 4; i++) {
      apply(FaultCatalog.module(i), FaultCatalog.WORN, Map.of(FaultCatalog.SCALE, 0.9));
    }

    double wornDistance = drive(worn, 3.0, 1.0);
    for (int i = 0; i < 4; i++) {
      undo(FaultCatalog.module(i), FaultCatalog.WORN);
    }

    // Within a thousandth: the robot follows its wheels no faster than its grip lets it.
    assertThat(wornDistance / fresh).isCloseTo(0.9, within(1e-3));
    assertThat(worn.drivetrain().treadScale(0)).isEqualTo(1.0);
    assertThat(components.on()).isEmpty();
  }

  @Test
  void aWheelSlipCountsTurnsTheFloorNeverSaw() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(3);
    double before = read(module).drivePositionRad;

    apply(FaultCatalog.module(3), FaultCatalog.SLIPS, Map.of(FaultCatalog.TURNS, 1.0));
    step(world);

    assertThat(read(module).drivePositionRad - before).isCloseTo(2 * Math.PI, within(1e-9));
    assertThat(world.truePose()).isEqualTo(STANDING);
  }

  /** A wheel spinning past the floor: its encoder counts it every loop, until it grips again. */
  @Test
  void aSpinningWheelCountsTravelTheFloorNeverSawUntilItGrips() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(2);
    double radius = ExampleRobot.SIM.definition().drivetrain().module(0).wheelRadius().in(Meters);
    double before = read(module).drivePositionRad;

    apply(FaultCatalog.module(2), FaultCatalog.SPINNING, Map.of(FaultCatalog.SPEED, 0.5));
    for (int i = 0; i < 10; i++) {
      step(world);
    }
    ModuleIOInputs spinning = read(module);
    undo(FaultCatalog.module(2), FaultCatalog.SPINNING);
    step(world);
    step(world);

    assertThat((spinning.drivePositionRad - before) * radius)
        .as("a fifth of a second at 0.5 m/s")
        .isCloseTo(0.1, within(1e-9));
    assertThat(spinning.driveVelocityRadPerSec * radius).isCloseTo(0.5, within(1e-9));
    assertThat(read(module).drivePositionRad).isCloseTo(spinning.drivePositionRad, within(1e-9));
    assertThat(world.truePose()).isEqualTo(STANDING);
    assertThat(components.on()).isEmpty();
  }

  /**
   * Pushed sideways: the robot slides, getting going at 3 m/s², while its wheels see nothing and
   * its IMU feels the push; stopped, it slows as it got going.
   */
  @Test
  void aPushSlidesTheRobotUnseenByItsWheelsAndFeltByItsImu() {
    SimulatedWorld world = world();
    ModuleIO module = world.drivetrainHardware().modules().get(0);
    double before = read(module).drivePositionRad;

    apply(ROBOT, FaultCatalog.PUSHED, Map.of(FaultCatalog.FORWARD, 0.0, FaultCatalog.LEFT, 1.0));
    step(world);
    GyroIO.GyroIOInputs starting = read(world.drivetrain().gyro());
    for (int i = 0; i < 49; i++) {
      step(world);
    }
    double slid = world.truePose().getY() - STANDING.getY();
    undo(ROBOT, FaultCatalog.PUSHED);
    step(world);
    GyroIO.GyroIOInputs stopping = read(world.drivetrain().gyro());
    for (int i = 0; i < 20; i++) {
      step(world);
    }

    assertThat(starting.accelerationYMetersPerSecSq)
        .isCloseTo(KinematicPhysics.PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED, within(1e-6));
    assertThat(stopping.accelerationYMetersPerSecSq)
        .isCloseTo(-KinematicPhysics.PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED, within(1e-6));
    // A second's push: a third of a second getting going, the rest at 1 m/s.
    assertThat(slid).isCloseTo(1.0 - 1.0 / (2 * 3), within(0.01));
    assertThat(world.truePose().getX()).isCloseTo(STANDING.getX(), within(1e-9));
    assertThat(read(module).drivePositionRad).as("the wheels never turned").isEqualTo(before);
    assertThat(read(world.drivetrain().gyro()).accelerationYMetersPerSecSq)
        .as("stopped")
        .isCloseTo(0, within(1e-9));
    assertThat(components.on()).isEmpty();
  }

  /** Tilted, the IMU feels the floor hold it up along its tipped axes, as a real one does. */
  @Test
  void aTiltedImuFeelsGravitysShare() {
    SimulatedWorld world = world();

    apply(ROBOT, FaultCatalog.TILTED, Map.of(FaultCatalog.PITCH, -10.0, FaultCatalog.ROLL, 5.0));
    step(world);
    GyroIO.GyroIOInputs tilted = read(world.drivetrain().gyro());

    // Nose up (a negative pitch): forward. Left side up (a positive roll): left.
    assertThat(tilted.accelerationXMetersPerSecSq)
        .isCloseTo(ImuReadings.STANDARD_GRAVITY * Math.sin(Math.toRadians(10)), within(1e-9));
    assertThat(tilted.accelerationYMetersPerSecSq)
        .isCloseTo(
            ImuReadings.STANDARD_GRAVITY
                * Math.sin(Math.toRadians(5))
                * Math.cos(Math.toRadians(10)),
            within(1e-9));
  }

  @Test
  void aShoveMovesTheRobotInItsOwnFrameAndItsGyroFeelsTheTurn() {
    SimulatedWorld world = world();

    apply(
        ROBOT,
        FaultCatalog.SHOVED,
        Map.of(FaultCatalog.FORWARD, 1.0, FaultCatalog.LEFT, 0.0, "Turn", 90.0));
    step(world);

    assertThat(world.truePose().getX()).isCloseTo(3.0, within(1e-9));
    assertThat(read(world.drivetrain().gyro()).yaw.getDegrees()).isCloseTo(90, within(1e-9));
  }

  @Test
  void aHitJoltsTheImu() {
    SimulatedWorld world = world();

    apply(ROBOT, FaultCatalog.HIT, Map.of(FaultCatalog.FORWARD, 0.5, FaultCatalog.LEFT, 0.0));
    step(world);

    assertThat(read(world.drivetrain().gyro()).accelerationXMetersPerSecSq)
        .isGreaterThan(DriveSimulation.HIT_JOLT_METERS_PER_SECOND_SQUARED / 2);
  }

  /** Tilted by hand, its IMU reads the tilt; undone, it's back on the (flat) ground. */
  @Test
  void aTiltIsReadByTheImuUntilTheRobotIsSetBackDown() {
    SimulatedWorld world = world();

    apply(ROBOT, FaultCatalog.TILTED, Map.of(FaultCatalog.PITCH, -5.0, FaultCatalog.ROLL, 3.0));
    step(world);
    GyroIO.GyroIOInputs tilted = read(world.drivetrain().gyro());
    undo(ROBOT, FaultCatalog.TILTED);
    step(world);

    assertThat(Math.toDegrees(tilted.pitchRad)).isCloseTo(-5, within(1e-9));
    assertThat(Math.toDegrees(tilted.rollRad)).isCloseTo(3, within(1e-9));
    assertThat(read(world.drivetrain().gyro()).pitchRad).isCloseTo(0, within(1e-9));
  }

  @Test
  void slippingMovesTheRobotLessUntilItGripsAgain() {
    double gripping = drive(world(), 3.0, 1.0);
    SimulatedWorld world = world();

    apply(ROBOT, FaultCatalog.SLIPPING, Map.of(FaultCatalog.GRIP, 0.5));
    double slipping = drive(world, 3.0, 1.0);
    undo(ROBOT, FaultCatalog.SLIPPING);

    // Within a thousandth: the robot follows its wheels no faster than its grip lets it.
    assertThat(slipping / gripping).isCloseTo(0.5, within(1e-3));
    assertThat(world.drivetrain().grip()).isEqualTo(1);
  }

  @Test
  void placedPutsTheRobotWhereAskedAndItsGyroFeelsTheTurn() {
    SimulatedWorld world = world();

    apply(ROBOT, FaultCatalog.PLACED, Map.of("X", 5.0, "Y", 2.0, "Heading", -90.0));
    step(world);

    assertThat(world.truePose().getTranslation().getDistance(new Translation2d(5, 2)))
        .isCloseTo(0, within(1e-9));
    assertThat(read(world.drivetrain().gyro()).yaw.getDegrees()).isCloseTo(-90, within(1e-9));
  }

  /** Undoing everything: the robot as built, its sensors reading as they did. */
  @Test
  void undoingEveryFaultPutsTheRobotBackAsBuilt() {
    SimulatedWorld world = world();
    apply(GYRO, FaultCatalog.UNPLUGGED);
    apply(BACKUP_IMU, FaultCatalog.ERRORS, Map.of(FaultCatalog.SCALE, 1.2));
    apply(FaultCatalog.module(1), FaultCatalog.STALE);
    apply(ROBOT, FaultCatalog.SLIPPING, Map.of(FaultCatalog.GRIP, 0.3));
    assertThat(components.on()).hasSize(4);

    assertThat(components.undoAll()).isNull();
    step(world);

    assertThat(components.on()).isEmpty();
    assertThat(read(world.drivetrain().gyro()).connected).isTrue();
    assertThat(world.drivetrain().backupGyroSim().errors())
        .isEqualTo(SensorErrors.NONE.backupGyro());
    assertThat(world.drivetrain().grip()).isEqualTo(1);
  }

  /** What's done is said, for the console and the log. */
  @Test
  void eachFaultAppliedOrUndoneIsSaid() {
    world();

    apply(GYRO, FaultCatalog.FROZEN);
    undo(GYRO, FaultCatalog.FROZEN);
    apply(ROBOT, FaultCatalog.HIT, Map.of(FaultCatalog.LEFT, 0.6));

    assertThat(said)
        .containsExactly(
            "Gyro: Frozen, on",
            "Gyro: Frozen, undone: Reports the robot's turns again.",
            "Robot: Hit: Forward 0 m, Left 0.6 m");
  }

  /** Drives every wheel at a voltage for a time; how far the robot went from where it stood. */
  private static double drive(SimulatedWorld world, double volts, double seconds) {
    for (int i = 0; i < 4; i++) {
      world.drivetrain().moduleSim(i).setDriveVoltage(volts);
    }
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      step(world);
    }
    return world.truePose().getTranslation().getDistance(STANDING.getTranslation());
  }

  private static ModuleIOInputs read(ModuleIO module) {
    ModuleIOInputs inputs = new ModuleIOInputs();
    module.updateInputs(inputs);
    return inputs;
  }

  private static GyroIO.GyroIOInputs read(GyroIO gyro) {
    GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();
    gyro.updateInputs(inputs);
    return inputs;
  }
}

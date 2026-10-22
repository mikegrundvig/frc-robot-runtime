package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO.ModuleIOInputs;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;

/**
 * The simulator's scenarios: each is found by its name, a misspelled name lists them all, each is
 * made of the simulated world's faults, and each does to the simulated world what it says. What the
 * robot's code makes of each is for the whole-robot tests ({@code ScenarioSimTest}).
 */
@ExtendWith(RobotSim.class)
class ScenariosTest {
  private static final List<CameraConfig> CAMERAS = ExampleRobot.SIM.definition().cameras();

  /** Where the robot stands for the events: on blue's side, clear of everything. */
  private static final Pose2d STANDING = new Pose2d(2.0, 4.0, Rotation2d.ZERO);

  private final List<SimulatedWorld> worlds = new ArrayList<>();

  /** The parts of the world built last, as Pitwall's simulator controls have them. */
  private Components components = new Components(said -> {});

  @AfterEach
  void tearDown() {
    worlds.forEach(SimulatedWorld::close);
  }

  /** The simulator's world on 2026's field, without cameras, sensors exact, the robot standing. */
  private SimulatedWorld world() {
    return world(List.of(), SensorErrors.NONE);
  }

  /** A world, its parts declared to a fresh set of simulator controls, the robot standing. */
  private SimulatedWorld world(List<CameraConfig> cameras, SensorErrors errors) {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition()).cameras(cameras).build();
    SimulatedWorld built =
        new SimulatedWorld(RobotSim.FIELD, robot, RobotSim.massOf(robot), errors);
    worlds.add(built);
    components = new Components(said -> {});
    built.addTo(components);
    built.place(STANDING);
    step(built);
    return built;
  }

  private static void step(SimulatedWorld world) {
    RobotSim.stepClock();
    world.step(RobotController.getTime());
  }

  /** Applies all of a scenario's faults to the world built last, as the page does. */
  private void start(String scenario, SimulatedWorld world) {
    assertThat(apply(Scenarios.named(scenario).faultsIn(world))).as(scenario).isNull();
  }

  /** Applies faults to the world built last, as a scenario does: why any were refused, or null. */
  private @Nullable String apply(List<Setting> faults) {
    return components.apply(faults);
  }

  static Stream<String> names() {
    return Scenarios.ALL.stream().map(Scenario::name);
  }

  @Test
  void everyScenarioHasItsOwnName() {
    List<String> names = names().toList();

    assertThat(names).doesNotHaveDuplicates();
    for (String name : names) {
      assertThat(Scenarios.named(name).name()).isEqualTo(name);
    }
  }

  /**
   * Every scenario is made of faults the world declares, with values they take, so the simulator's
   * controls offer every one.
   */
  @Test
  void everyScenarioIsMadeOfTheWorldsFaults() {
    SimulatedWorld world = world(CAMERAS.subList(0, 1), SensorErrors.TYPICAL);

    Scenarios.addTo(components, world); // refuses a fault the world doesn't declare

    assertThat(components.scenarioView()).containsOnlyKeys(names().toArray(String[]::new));
  }

  /** A robot without cameras isn't offered a scenario that knocks one. */
  @Test
  void aRobotWithoutCamerasHasNoCameraScenarios() {
    SimulatedWorld world = world();

    Scenarios.addTo(components, world);

    assertThat(components.scenarioView().keySet())
        .doesNotContain(
            "camera-low", "camera-turned", "camera-late", "camera-clock-off", "camera-frozen")
        .contains("collision", "gyro-unplugged");
  }

  /**
   * Every scenario does something: it turns a lasting fault on, or what happens in it changes where
   * the robot is or what its gyros and wheels read, a step later, from a world it didn't happen in.
   */
  @ParameterizedTest
  @MethodSource("names")
  void everyScenarioDoesSomething(String name) {
    Scenario scenario = Scenarios.named(name);
    List<CameraConfig> cameras = name.startsWith("camera") ? CAMERAS.subList(0, 1) : List.of();
    SimulatedWorld untouched = world(cameras, SensorErrors.TYPICAL);
    SimulatedWorld started = world(cameras, SensorErrors.TYPICAL);

    assertThat(apply(scenario.faultsIn(started))).isNull();

    boolean faults = !components.on().isEmpty();
    boolean happened = !afterATurn(started).equals(afterATurn(untouched));
    assertThat(faults || happened).isTrue();
  }

  /**
   * A step after the robot is turned in place (so a frozen gyro shows): where it is, its tilt, and
   * what its gyros and wheels read.
   */
  private static List<Object> afterATurn(SimulatedWorld world) {
    world.drivetrain().shove(new Transform2d(0, 0, Rotation2d.fromDegrees(30)));
    step(world);
    GyroIO.GyroIOInputs gyro = read(world.drivetrain().gyro());
    GyroIO.GyroIOInputs backup = read(world.drivetrain().backupGyro());
    List<Object> seen =
        new ArrayList<>(
            List.of(
                world.truePose(),
                world.drivetrain().pitchRadians(),
                world.drivetrain().rollRadians(),
                gyro.connected,
                gyro.yaw,
                backup.connected,
                backup.yaw));
    for (ModuleIO module : world.drivetrainHardware().modules()) {
      seen.add(read(module).drivePositionRad);
    }
    return seen;
  }

  @Test
  void aMisspelledNameListsEveryScenario() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Scenarios.named("colision"))
        .withMessageStartingWith("No simulator scenario is named \"colision\". The scenarios: ")
        .withMessageContaining(
            Scenarios.ALL.stream().map(Scenario::name).collect(Collectors.joining(", ")));
  }

  @Test
  void theListingGivesEachNameAndWhatItDoes() {
    String listing = Scenarios.listing();

    assertThat(listing.lines()).hasSize(Scenarios.ALL.size());
    assertThat(listing)
        .contains("collision        Another robot hits it")
        .contains("camera-low       The robot's first camera knocked 4 in low");
    assertThat(Scenarios.line(Scenarios.named("shove")))
        .startsWith("shove            Pushed 0.6 m")
        .endsWith(System.lineSeparator());
  }

  @Test
  void aNameIsLowercaseWordsJoinedByHyphensAndADescriptionSaysSomething() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Scenario.builder()
                    .name("Camera Low")
                    .description(" ")
                    .sensorFaults(Scenario.NO_FAULTS)
                    .events(Scenario.NOTHING)
                    .build())
        .withMessageContaining(
            "Scenario.name: \"Camera Low\" must be lowercase words joined by hyphens")
        .withMessageContaining("Scenario.description: must not be blank");
  }

  @Test
  void aCameraKnockedLowIsTheFirstCameraAlone() {
    SimulatedWorld world = world(CAMERAS.subList(0, 2), SensorErrors.TYPICAL);

    start("camera-low", world);

    assertThat(Scenarios.named("camera-low").sensorFaultsIn(world))
        .containsExactly(
            Setting.of(
                FaultCatalog.camera(CAMERAS.get(0).name()),
                FaultCatalog.MOVED,
                Map.of(FaultCatalog.FORWARD, 0.0, FaultCatalog.LEFT, 0.0, FaultCatalog.UP, -4.0)));
    assertThat(world.camera(0).isMoved()).isTrue();
    assertThat(world.camera(1).isMoved()).isFalse();
    assertThat(world.drivetrain().gyroSim().errors()).isEqualTo(SensorErrors.TYPICAL.gyro());
  }

  @Test
  void aCameraTurnedIsTwoDegreesOfYaw() {
    SimulatedWorld world = world(CAMERAS.subList(0, 1), SensorErrors.TYPICAL);

    assertThat(Scenarios.named("camera-turned").sensorFaultsIn(world).get(0).values())
        .containsEntry(FaultCatalog.YAW, 2.0)
        .containsEntry(FaultCatalog.ROLL, 0.0)
        .containsEntry(FaultCatalog.PITCH, 0.0);
  }

  @Test
  void aCameraScenarioNeedsACamera() {
    SimulatedWorld world = world();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> Scenarios.named("camera-low").faultsIn(world))
        .withMessage("this scenario knocks a camera, and the robot has none");
  }

  @Test
  void theGyroScaleIsTheGyrosAlone() {
    SimulatedWorld world = world(List.of(), SensorErrors.TYPICAL);

    start("gyro-scale", world);

    SensorErrors.GyroErrors gyro = world.drivetrain().gyroSim().errors();
    assertThat(gyro.scale()).isEqualTo(1.15);
    assertThat(gyro.driftDegreesPerMinute())
        .isEqualTo(SensorErrors.TYPICAL.gyro().driftDegreesPerMinute());
    assertThat(world.drivetrain().backupGyroSim().errors())
        .isEqualTo(SensorErrors.TYPICAL.backupGyro());
  }

  @Test
  void wornTreadAndAModuleZeroedOffChangeTheWheels() {
    SimulatedWorld worn = world(List.of(), SensorErrors.TYPICAL);
    start("worn-tread", worn);
    for (int i = 0; i < 4; i++) {
      assertThat(worn.drivetrain().treadScale(i)).isEqualTo(0.97);
    }

    SimulatedWorld zeroedOff = world(List.of(), SensorErrors.TYPICAL);
    start("module-zero-off", zeroedOff);
    assertThat(Math.toDegrees(zeroedOff.drivetrain().moduleSim(0).steerOffsetRadians()))
        .isCloseTo(5.0, within(1e-9));
    assertThat(Math.toDegrees(zeroedOff.drivetrain().moduleSim(1).steerOffsetRadians()))
        .isCloseTo(SensorErrors.TYPICAL.steerOffsetDegrees().get(1), within(1e-9));
  }

  /** Exact sensors on a season-worn robot: every wheel and IMU as {@link SensorErrors#NONE}. */
  @Test
  void exactSensorsAreExactlyAsConfigured() {
    SimulatedWorld world = world(List.of(), SensorErrors.TYPICAL);

    start("exact-sensors", world);

    DriveSimulation drivetrain = world.drivetrain();
    for (int i = 0; i < 4; i++) {
      assertThat(drivetrain.treadScale(i)).isEqualTo(1.0);
      assertThat(drivetrain.moduleSim(i).steerOffsetRadians()).isZero();
    }
    assertThat(drivetrain.gyroSim().errors()).isEqualTo(SensorErrors.GyroErrors.EXACT);
    assertThat(drivetrain.backupGyroSim().errors()).isEqualTo(SensorErrors.GyroErrors.EXACT);
  }

  @Test
  void aCollisionMovesTheRobotAndJoltsItsImu() {
    SimulatedWorld world = world();

    start("collision", world);

    assertThat(world.truePose().getY()).isCloseTo(4.6, within(1e-9));
    assertThat(read(world.drivetrain().gyro()).accelerationYMetersPerSecSq)
        .isCloseTo(DriveSimulation.HIT_JOLT_METERS_PER_SECOND_SQUARED, within(1e-9));
    assertThat(read(world.drivetrain().backupGyro()).accelerationYMetersPerSecSq)
        .as("the backup IMU feels it too")
        .isCloseTo(DriveSimulation.HIT_JOLT_METERS_PER_SECOND_SQUARED, within(1e-9));
  }

  @Test
  void aShoveMovesTheRobotUnfelt() {
    SimulatedWorld world = world();

    start("shove", world);

    assertThat(world.truePose().getY()).isCloseTo(4.6, within(1e-9));
    GyroIO.GyroIOInputs felt = read(world.drivetrain().gyro());
    assertThat(Math.hypot(felt.accelerationXMetersPerSecSq, felt.accelerationYMetersPerSecSq))
        .isCloseTo(0, within(1e-9));
  }

  @Test
  void aSlippingWheelTurnsWithoutMovingTheRobot() {
    SimulatedWorld world = world();
    ModuleIO[] modules = world.drivetrainHardware().modules().toArray(ModuleIO[]::new);
    double before = read(modules[0]).drivePositionRad;

    start("wheel-slip", world);
    step(world);

    assertThat(read(modules[0]).drivePositionRad - before).isCloseTo(Math.PI, within(1e-9));
    assertThat(read(modules[1]).drivePositionRad).isCloseTo(0, within(1e-9));
    assertThat(world.truePose()).isEqualTo(STANDING);
  }

  /** Every wheel pushed the same, the robot goes three quarters as far with a corner up. */
  @Test
  void aLiftedWheelTiltsTheRobotAndItsWheelsMoveItLess() {
    double flat = distanceDriven(world());
    SimulatedWorld world = world();

    start("wheel-lifted", world);
    double lifted = distanceDriven(world);

    assertThat(Math.toDegrees(world.drivetrain().pitchRadians()))
        .as("nose up")
        .isCloseTo(-5, within(1e-9));
    assertThat(Math.toDegrees(world.drivetrain().rollRadians()))
        .as("left side up")
        .isCloseTo(5, within(1e-9));
    // Within a thousandth: the robot follows its wheels no faster than its grip lets it.
    assertThat(lifted / flat).isCloseTo(0.75, within(1e-3));
  }

  /** How far the robot goes with every drive motor at 3 V for a second. */
  private static double distanceDriven(SimulatedWorld world) {
    for (int i = 0; i < 4; i++) {
      world.drivetrain().moduleSim(i).setDriveVoltage(3.0);
    }
    for (int i = 0; i < RobotLoop.loops(1.0); i++) {
      step(world);
    }
    return world.truePose().getTranslation().getDistance(STANDING.getTranslation());
  }

  @Test
  void theGyroUnpluggedLeavesTheBackup() {
    SimulatedWorld world = world();

    start("gyro-unplugged", world);
    step(world);

    assertThat(read(world.drivetrain().gyro()).connected).isFalse();
    assertThat(read(world.drivetrain().backupGyro()).connected).isTrue();
  }

  @Test
  void bothImusUnpluggedLeaveNeither() {
    SimulatedWorld world = world();

    start("imus-unplugged", world);
    step(world);

    assertThat(read(world.drivetrain().gyro()).connected).isFalse();
    assertThat(read(world.drivetrain().backupGyro()).connected).isFalse();
  }

  @Test
  void aFrozenGyroMissesATurn() {
    SimulatedWorld world = world();
    double before = read(world.drivetrain().gyro()).yaw.getDegrees();

    start("gyro-frozen", world);
    world.place(new Pose2d(STANDING.getTranslation(), Rotation2d.fromDegrees(30)));
    step(world);

    GyroIO.GyroIOInputs gyro = read(world.drivetrain().gyro());
    assertThat(gyro.connected).isTrue();
    assertThat(gyro.yaw.getDegrees()).isEqualTo(before);
    assertThat(read(world.drivetrain().backupGyro()).yaw.getDegrees()).isCloseTo(30, within(1e-9));
  }

  /** From either side, the robot ends up clear of the nearest bump's ramp, facing its crest. */
  @Test
  void aBumpCrossingSetsTheRobotBeforeTheNearestBumpFacingIt() {
    for (double x : new double[] {3.0, 6.0}) { // before blue's left bump, and past it
      SimulatedWorld world = world();
      world.place(new Pose2d(x, 5.5, Rotation2d.fromDegrees(70)));

      start("bump-crossing", world);

      Bump bump = Rebuilt.LEFT_BUMP;
      Translation2d spot = world.truePose().getTranslation();
      double offset = bump.acrossOffset(spot);
      assertThat(Math.abs(offset))
          .as("from x = " + x)
          .isCloseTo(bump.depthMeters() / 2 + Faults.BUMP_CLEARANCE_METERS, within(1e-9));
      assertThat(Math.signum(offset))
          .isEqualTo(Math.signum(bump.acrossOffset(new Translation2d(x, 5.5))));
      assertThat(bump.alongOffset(spot)).isCloseTo(bump.length() / 2, within(1e-9));
      Translation2d ahead = spot.plus(new Translation2d(0.1, world.truePose().getRotation()));
      assertThat(Math.abs(bump.acrossOffset(ahead)))
          .as("facing the crest")
          .isLessThan(Math.abs(offset));
    }
  }

  @Test
  void aBumpCrossingNeedsABump() {
    SimulatedWorld world = world();
    world.drivetrain().setTerrain(Terrain.FLAT); // the field's bumps gone since it was declared

    assertThatIllegalArgumentException()
        .isThrownBy(() -> Scenarios.named("bump-crossing").faultsIn(world))
        .withMessage("this scenario carries the robot to a bump, and the field has none");
    assertThat(apply(List.of(Setting.of(FaultCatalog.ROBOT, FaultCatalog.TO_A_BUMP))))
        .isEqualTo(
            "Robot: Carried to a bump failed: carrying the robot to a bump needs a field with"
                + " bumps, and this one has none");
  }

  /** A field without bumps isn't offered the bump scenario, nor carrying the robot to a bump. */
  @Test
  void aFieldWithoutBumpsHasNoBumpScenario() {
    SimulatedWorld world = world();
    world.drivetrain().setTerrain(Terrain.FLAT);
    Components flat = new Components(said -> {});
    world.addTo(flat);

    Scenarios.addTo(flat, world);

    assertThat(flat.scenarioView()).doesNotContainKey("bump-crossing").containsKey("collision");
    assertThat(flat.whyNot(Setting.of(FaultCatalog.ROBOT, FaultCatalog.TO_A_BUMP)))
        .startsWith("Robot has no fault named Carried to a bump");
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

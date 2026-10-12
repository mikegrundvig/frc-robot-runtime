package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.PoseEstimator;
import com.michaelgrundvig.frc.localization.PoseReset;
import com.michaelgrundvig.frc.localization.SwerveKinematicsModel;
import com.michaelgrundvig.frc.localization.TagAllowlist;
import com.michaelgrundvig.frc.localization.TagDetections;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.localization.VisionResult;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Disabling;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveHardware;
import com.michaelgrundvig.frc.runtime.subsystems.localization.Localization;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;

/**
 * Drives the drivetrain in simulation the way the robot runs it: the simulated Driver Station, the
 * command scheduler, and the joystick drive command, on simulated modules and IMUs, with
 * localization taking what they read, and the simulator's physics stepped before each loop reads
 * the sensors.
 */
public final class DriveSimHarness implements AutoCloseable {
  public final Scheduler scheduler = Scheduler.createIndependentScheduler();

  /** Cancels every command in the loop the robot is disabled, as the robot's loop does. */
  private final Disabling disabling = new Disabling(scheduler);

  public final DriveSimulation simulation;
  public final PoseEstimator estimator;
  public final Localization localization;

  /**
   * Localization itself, behind {@link #localization}: what a test asks beyond the robot's view.
   */
  public final LocalizationService service;

  public final Drive drive;

  /** Every camera measurement's fate, in order. */
  public final List<FrameVerdict> frames = new ArrayList<>();

  /** What localization made of each image the tag cameras sent, at once, in order. */
  public final List<VisionResult> images = new ArrayList<>();

  private @Nullable FakeCamera camera;
  private List<FakeTagCamera> tagCameras = List.of();
  private @Nullable TagAllowlist onlyTags;
  private @Nullable Runnable eachLoop;

  /** The driver's sticks, as the robot hands them to the drive: on the field's axes already. */
  private Translation2d stick = Translation2d.ZERO;

  private double turnStick;

  /** The alliance the drive is told: blue unless a test says another, or none. */
  private Optional<Alliance> alliance = Optional.of(Alliance.BLUE);

  /**
   * An estimator that starts with the robot exactly here, and accepts any point as on the field. It
   * keeps apart the simulator's field's tags.
   */
  public static PoseEstimator estimatorAt(Pose2d start) {
    return estimatorAt(start, EstimatorConfig.DEFAULTS);
  }

  public static PoseEstimator estimatorAt(Pose2d start, EstimatorConfig config) {
    return new PoseEstimator(
        config,
        point -> true,
        RobotSim.FIELD.tags().keySet(),
        new PoseReset(0, start, PoseCovariance.fromStdDevs(0.001, 0.001, 0.001), "test start"));
  }

  /** Localization for a robot, on flat ground, starting with its estimator. */
  public static Localization localizationFor(RobotConfig robot, PoseEstimator estimator) {
    return localizationFor(robot, estimator, Terrain.FLAT);
  }

  /**
   * Localization for a robot on this terrain, starting with its estimator. Its images are solved
   * against the simulator's field's tags.
   */
  public static Localization localizationFor(
      RobotConfig robot, PoseEstimator estimator, Terrain terrain) {
    return new Localization(
        new LocalizationService(
            estimator,
            new SwerveKinematicsModel(robot.definition().moduleTranslations()),
            RobotSim.FIELD,
            terrain,
            robot.definition(),
            RobotLoop.PERIOD_SECONDS));
  }

  /** Whether the harness hands the drive the driver's sticks as it builds it. */
  private enum Sticks {
    HANDED_OVER,
    NOT_YET
  }

  /** Wheels and sensors exactly as configured: odometry is the truth. */
  public DriveSimHarness(RobotConfig robot) {
    this(robot, SensorErrors.NONE);
  }

  /**
   * The same, with a drive that has not been handed the driver's sticks, the field or the alliance:
   * a robot program part-way through building itself. {@link #takeDriver} hands them over.
   */
  public static DriveSimHarness withoutDriver(RobotConfig robot) {
    return new DriveSimHarness(
        robot, SensorErrors.NONE, EstimatorConfig.DEFAULTS, Terrain.FLAT, Sticks.NOT_YET);
  }

  public DriveSimHarness(RobotConfig robot, SensorErrors errors) {
    this(robot, errors, EstimatorConfig.DEFAULTS);
  }

  public DriveSimHarness(RobotConfig robot, SensorErrors errors, EstimatorConfig config) {
    this(robot, errors, config, Terrain.FLAT);
  }

  /** On this terrain: the simulator rides over it, and localization knows it. */
  public DriveSimHarness(
      RobotConfig robot, SensorErrors errors, EstimatorConfig config, Terrain terrain) {
    this(robot, errors, config, terrain, Sticks.HANDED_OVER);
  }

  private DriveSimHarness(
      RobotConfig robot,
      SensorErrors errors,
      EstimatorConfig config,
      Terrain terrain,
      Sticks sticks) {
    estimator = estimatorAt(Pose2d.ZERO, config);
    localization = localizationFor(robot, estimator, terrain);
    service = localization.service();
    RobotDefinition definition = robot.definition();
    simulation =
        new DriveSimulation(definition.body(), definition.drivetrain(), errors, definition.imus());
    simulation.setTerrain(terrain);
    drive =
        new Drive(
            "Drive",
            DriveHardware.builder()
                .modules(List.of(simulation.modules()))
                .gyro(simulation.gyro())
                .backupImu(simulation.backupGyro())
                .odometryLock(new ReentrantLock())
                .build(),
            definition,
            robot::drivetrain,
            scheduler,
            localization::pose);
    if (sticks == Sticks.HANDED_OVER) {
      takeDriver();
    }
  }

  /**
   * Hands the drive the driver's sticks, the field and the alliance, as the robot does once while
   * it builds itself. Only a harness made {@link #withoutDriver} needs to call it.
   */
  public void takeDriver() {
    drive.takeDriver(() -> stick, () -> turnStick, () -> RobotSim.FIELD, () -> alliance);
  }

  /**
   * From now on the drive's sticks read as these (the translation stick on the field's axes, as the
   * robot turns it for the driver, and the turn stick counterclockwise positive).
   */
  public void sticks(double x, double y, double turn) {
    stick = new Translation2d(x, y);
    turnStick = turn;
  }

  /** From now on the drive is told this alliance, or none yet. */
  public void alliance(Optional<Alliance> alliance) {
    this.alliance = alliance;
  }

  /** From now on, a camera's results reach the estimator every loop. */
  public void watchWith(FakeCamera camera) {
    this.camera = camera;
  }

  /**
   * From now on, these cameras' raw images reach localization every loop, solved there as the robot
   * was when each was taken.
   */
  public void watchWith(List<FakeTagCamera> cameras) {
    tagCameras = List.copyOf(cameras);
  }

  /** From now on, the tag cameras' images are solved from these tags alone; null for every tag. */
  public void trustOnly(@Nullable TagAllowlist tags) {
    onlyTags = tags;
  }

  /** From now on, no camera's results reach the estimator: blinded, until watching again. */
  public void stopWatching() {
    camera = null;
    tagCameras = List.of();
  }

  /** Something to do at the end of every loop, such as feeding another estimator to compare. */
  public void eachLoop(Runnable action) {
    eachLoop = action;
  }

  /**
   * One robot loop: the physics moves, the drivetrain reads its sensors, every command is canceled
   * if the robot was just disabled, localization takes the readings, the camera's results arrive,
   * the estimate takes those, then commands run.
   */
  public void loop() {
    RobotSim.stepClock();
    long now = RobotController.getTime();
    simulation.step(RobotLoop.PERIOD_SECONDS, now);
    boolean enabled = RobotState.isEnabled();
    drive.periodic(enabled);
    disabling.cancelCommandsIfDisabled(enabled);
    localization.periodic(drive.readings(), enabled);
    FakeCamera watching = camera;
    if (watching != null) {
      for (VisionMeasurement measurement : watching.poll(now)) {
        service.camera(measurement);
      }
    }
    for (FakeTagCamera tagCamera : tagCameras) {
      for (TagDetections image : tagCamera.poll(now)) {
        images.add(service.camera(image, onlyTags));
      }
    }
    frames.addAll(localization.update().frames());
    scheduler.run();
    Runnable action = eachLoop;
    if (action != null) {
      action.run();
    }
  }

  /** Hold the sticks (as {@link #sticks} takes them) for {@code seconds}, enabled. */
  public void drive(double seconds, double x, double y, double omega) {
    RobotSim.setEnabled(true);
    sticks(x, y, omega);
    Command driving = drive.driven();
    scheduler.schedule(driving);
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      loop();
    }
    scheduler.cancel(driving);
    sticks(0, 0, 0);
  }

  /** Run a command for {@code seconds}, enabled, the way the robot loop runs it. */
  public void run(Command command, double seconds) {
    RobotSim.setEnabled(true);
    scheduler.schedule(command);
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      loop();
    }
  }

  /** Sit disabled for {@code seconds}. */
  public void idle(double seconds) {
    RobotSim.setEnabled(false);
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      loop();
    }
  }

  /** How far the estimate is from where the robot really is, in meters. */
  public double positionError() {
    return localization.pose().getTranslation().getDistance(simulation.truePose().getTranslation());
  }

  /** How far the estimate's heading is turned from the truth, in degrees. */
  public double headingErrorDegrees() {
    Pose2d truth = simulation.truePose();
    return localization.pose().getRotation().minus(truth.getRotation()).getDegrees();
  }

  @Override
  public void close() {
    drive.close();
    localization.close();
  }
}

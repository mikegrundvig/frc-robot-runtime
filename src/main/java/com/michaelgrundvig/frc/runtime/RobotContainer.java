package com.michaelgrundvig.frc.runtime;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.SwerveKinematicsModel;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Profile;
import com.michaelgrundvig.frc.pitwall.experiment.Profiles;
import com.michaelgrundvig.frc.runtime.autos.AutoPit;
import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Bindings;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.commands.Calibration;
import com.michaelgrundvig.frc.runtime.dashboard.Dashboard;
import com.michaelgrundvig.frc.runtime.dashboard.RobotStatus;
import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.hardware.RobotHardware;
import com.michaelgrundvig.frc.runtime.lib.timing.LoopMonitor;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimulation;
import com.michaelgrundvig.frc.runtime.sim.Scenarios;
import com.michaelgrundvig.frc.runtime.sim.SimulatedHardware;
import com.michaelgrundvig.frc.runtime.sim.SimulatedWorld;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.localization.Localization;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointExperiments;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import com.michaelgrundvig.frc.runtime.venues.Venues;
import com.michaelgrundvig.frc.runtime.views.world.WorldView;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Placement;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.time.Duration;
import java.time.LocalDate;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.system.RobotController;
import org.wpilib.tunable.TunableRegistry;

/**
 * The composition root: builds every mechanism on the hardware for the mode the program runs in
 * ({@link Hardware}), runs them each loop, and hands the team's {@code Controls} what behavior
 * reaches: the joints, the drivetrain, the sensors, the driver's and operator's gamepads, and the
 * list its autos are declared in. It runs the auto the pit chose: when autonomous becomes enabled
 * it schedules the auto's command, made fresh, and when autonomous ends it cancels it. The
 * template's, so a team's code goes in {@code Controls} and {@code Robots}, and the template's
 * updates here never collide with it. This is the one file that shows how the whole robot fits
 * together, so keep it readable.
 */
public final class RobotContainer implements AutoCloseable {
  private static final int DRIVER_PORT = 0;
  private static final int OPERATOR_PORT = 1;

  /** How well a drive team places the robot on its start: within about 5 cm and 2 degrees. */
  static final PoseCovariance PLACED = PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2));

  /** How straight a driver lines the robot up by eye for a heading reset: about 5 degrees. */
  static final double HEADING_BY_EYE_RADIANS = Math.toRadians(5);

  private final Scheduler scheduler;

  /**
   * What the pit can tune on a robot: each mechanism's configuration, with the limits it may be
   * tuned within. The one list of them: the container publishes these, and the tests check every
   * one (its committed values within its limits, a meaning for each value, and a row in the values
   * register, docs/values.md). A mechanism with tuning adds it here.
   *
   * @param drivetrain the drivetrain's gains and limits ({@link Drive#tuning})
   * @param vision how far the cameras are trusted ({@link Vision#tuning})
   * @param joints each driven joint's gains and current limits, by its label ({@link
   *     JointMechanism#tuning})
   */
  record Tunings(
      LiveConfig<DrivetrainControl> drivetrain,
      LiveConfig<VisionConfig> vision,
      Map<String, LiveConfig<MechanismControl>> joints) {
    /** Every one. */
    List<LiveConfig<?>> all() {
      List<LiveConfig<?>> all = new ArrayList<>(List.of(drivetrain, vision));
      all.addAll(joints.values());
      return List.copyOf(all);
    }
  }

  /** Each mechanism's tuning, starting from a robot's committed configuration. */
  static Tunings tunings(RobotConfig robot) {
    Map<String, LiveConfig<MechanismControl>> joints = new LinkedHashMap<>();
    for (Joint joint : Joints.driven(robot.definition())) {
      joints.put(joint.label(), JointMechanism.tuning(joint, ControllerSettings.control(joint)));
    }
    // In the definition's order, which is the order Pitwall's Tune lists them in.
    return new Tunings(
        Drive.tuning(robot.drivetrain(), robot.definition().drivetrain().freeSpeed()),
        Vision.tuning(robot.vision()),
        Collections.unmodifiableMap(joints));
  }

  /** What the pit can tune, as published: every one is removed again as the robot closes. */
  private final Tunings tunings;

  /** The field as this match sets it: the alliance, the start, the placement, the zones. */
  private final MatchField matchField;

  private final Venues venues;

  /** The autos the team declares, and the one the pit chose: the template runs it. */
  private final Autos autos;

  /** The simulator's world, which knows where the robot really is. Null unless simulating. */
  private final @Nullable SimulatedWorld world;

  private final Localization localization;

  /** What the estimator did with the last loop's camera measurements, for the World view. */
  private List<FrameVerdict> frames = List.of();

  /** What the robot is, as configured, for the World view. */
  private final RobotDefinition robotDefinition;

  private final Drive drive;

  /** Every driven joint's mechanism, and the joints without wiring on a robot. */
  private final Joints joints;

  /** Whether a camera rides on a joint, so localization needs the joints' positions. */
  private final boolean camerasOnJoints;

  private final Vision vision;
  private final Calibration calibration;

  /** Every command bound to the drive team's gamepads, as the bindings were made. */
  private final Bindings bindings;

  private final RecordingGamepad driver;
  private final RecordingGamepad operator;

  /** What the robot reads and drives: its devices, the simulator's, or none in a replay. */
  private final Hardware hardware;

  /** Every loop's time, section by section: the robot marks its sections, this marks its own. */
  private final LoopHealth loopHealth;

  private final LoopMonitor loop;

  private boolean wasAutonomous;

  /**
   * Built by {@link Robot#assemble}, for the robot and for tests alike.
   *
   * @param scheduler runs every command: the controls', and the Pitwall page's actions
   * @param robot the robot: its frame, wiring, and cameras
   * @param field the field the robot plays on, with its official layout
   * @param venues the venue the pit selected, which takes the field's place while disabled
   * @param controls what the robot does and the drive team's buttons for it: the team's own, made
   *     once the mechanisms are
   * @param simulatedErrors in the simulator, how the robot's wheels and sensors differ from its
   *     configuration ({@link SensorErrors#TYPICAL} makes its estimated pose drift from the truth
   *     as a real robot's does)
   * @param loopHealth the loop's timing, which the robot starts and finishes and this marks the
   *     sections of; closed with this
   * @param watched the hardware for the mode, handed through a test's own on its way to the
   *     mechanisms, to record what they're told ({@code RobotSim}); as it is otherwise
   */
  RobotContainer(
      RunMode mode,
      Scheduler scheduler,
      RobotConfig robot,
      FieldModel field,
      Venues venues,
      ControlsMaker<?> controls,
      SensorErrors simulatedErrors,
      LoopHealth loopHealth,
      UnaryOperator<Hardware> watched) {
    // What the pit can tune, published where dashboards and the Pitwall page find it; the
    // mechanisms read it every loop.
    Tunings tunings = tunings(robot);
    this.tunings = tunings;
    TunableRegistry.publish("Drive/config", tunings.drivetrain());
    TunableRegistry.publish("Vision/config", tunings.vision());
    tunings
        .joints()
        .forEach((label, tuning) -> TunableRegistry.publish("Joints/" + label + "/config", tuning));
    RobotDefinition definition = robot.definition();
    Supplier<DrivetrainControl> drivetrain = tunings.drivetrain()::get;
    this.scheduler = scheduler;
    this.loopHealth = loopHealth;
    loop = loopHealth.monitor();
    List<CameraConfig> cameras = definition.cameras();
    robotDefinition = definition;
    autos = new Autos(scheduler);
    matchField = new MatchField(field, mode == RunMode.SIM, autos);
    this.venues = venues;
    bindings = new Bindings(scheduler);
    driver = bindings.gamepad("driver", DRIVER_PORT);
    operator = bindings.gamepad("operator", OPERATOR_PORT);

    // What the robot reads and drives: the simulator's, the robot's own devices, or none (a replay
    // reads the log).
    world = mode == RunMode.SIM ? new SimulatedWorld(field, definition, simulatedErrors) : null;
    hardware = watched.apply(hardware(mode, definition, world));

    // Until the robot is placed it could be anywhere: localization starts it in the middle of the
    // field, as unsure as the field is big, so the first camera measurement wins. The simulated
    // robot stands there too.
    Pose2d middle = new Pose2d(field.length() / 2, field.width() / 2, Rotation2d.ZERO);
    localization =
        new Localization(
            new LocalizationService(
                field,
                definition,
                new SwerveKinematicsModel(definition.moduleTranslations()),
                robot.estimator(),
                RobotLoop.PERIOD_SECONDS));
    drive =
        new Drive("Drive", hardware.drive(), definition, drivetrain, scheduler, localization::pose);
    // The driver's sticks and the field, so the drive's own commands (driven, facing) read them:
    // the team's Controls builds assisted driving without holding them.
    drive.takeDriver(
        this::leftStick, () -> -driver.getRightX(), matchField::field, matchField::alliance);
    if (world != null) {
      world.place(middle);
    }

    // Every driven joint, a v3 mechanism the template builds from the definition, the joint's block
    // saying how it's driven and wired, each as the pit tunes it.
    camerasOnJoints = definition.cameras().stream().anyMatch(camera -> camera.joint().isPresent());
    joints =
        new Joints(
            definition,
            hardware,
            joint -> Objects.requireNonNull(tunings.joints().get(joint.label()))::get,
            scheduler);
    // Cameras: PhotonVision on the robot, its simulation in the simulator, the log in a replay.
    vision =
        new Vision(
            cameras,
            hardware.cameras().toArray(CameraIO[]::new),
            localization.service(),
            field,
            tunings.vision()::get,
            matchField::alliance);
    calibration =
        new Calibration(
            drive,
            vision,
            localization,
            cameras,
            robot.drivetrainWiring(),
            definition.drivetrain(),
            LocalDate::now);

    // Driver: left stick translates (field-relative, from the driver's end of the field), right
    // stick turns: the drive's default command, driven(). The team's Controls binds the rest of
    // driving to the same sticks: drive.facing(...) holds a heading, drive.lockWheels() an X. Back,
    // with the robot facing straight downfield, resets its heading: the fallback
    // when the heading has gone wrong. Enabled or not. The template's own, so they're there
    // whatever the team's Controls binds, and a second command on back is a doubt at startup.
    drive.setDefaultCommand(drive.driven());
    driver
        .back()
        .onTrue(Command.noRequirements(coroutine -> faceDownfield()).named("Reset heading"));

    // What the robot does, and the drive team's buttons for the rest: the team's own class. Its
    // constructor builds the commands, binds them and declares its autos, so nothing else needs to
    // hold it.
    controls.make(scheduler, definition, joints, drive, driver, operator, autos);

    // What it doubts or doesn't know, said once its bindings are made: its definition's, and two
    // commands bound to one button the same way.
    Doubts.say(definition, Doubts.of(definition, bindings), mode);
  }

  /**
   * The driver says the robot faces straight downfield, away from the driver's own wall: its
   * heading becomes the alliance's forward, as sure as lining up by eye allows. Its position stays.
   * If the driver was wrong, the cameras' headings will agree against it and the estimate turns
   * back to them.
   */
  void faceDownfield() {
    localization.resetHeading(matchField.driverForward(), HEADING_BY_EYE_RADIANS);
  }

  /**
   * The driver's left stick as a direction on the field. Gamepad Y is negative when pushed forward,
   * so negate it for forward = away from the driver, then turn it the way the driver faces: a red
   * driver's forward is the field's -x.
   */
  private Translation2d leftStick() {
    return new Translation2d(-driver.getLeftY(), -driver.getLeftX())
        .rotateBy(matchField.driverForward());
  }

  /**
   * One loop of every mechanism, before the command scheduler runs: the drivetrain reads its
   * sensors, localization takes them, the cameras theirs, and the field's checks follow.
   */
  public void periodic() {
    boolean enabled = RobotState.isEnabled();
    // The venue first: a selection the pit made takes effect at the start of a loop, while
    // disabled.
    venues.update(!enabled, RobotState.isFMSAttached()).ifPresent(this::venue);
    // The auto the pit chose likewise, before the start it names is read.
    autos.update(!enabled);
    // Where the drive team put the robot, if they did this loop: on the chosen start. With cameras,
    // the tags will say instead.
    Optional<Pose2d> placed = matchField.read(enabled);
    SimulatedWorld sim = world;
    if (sim != null) {
      // In the simulator the drive team really moves it. Then the world moves on and the robot
      // reads its sensors, as on a robot between loops: its gyro feels the turn this loop.
      placed.ifPresent(sim::place);
      sim.step(RobotController.getTime());
      loop.mark("Simulation", System.nanoTime());
    }
    boolean autonomous = RobotState.isAutonomousEnabled();
    if (autonomous && !wasAutonomous) {
      vision.autonomousStarted(RobotController.getTime());
      // The chosen auto, made fresh, runs from here; the command scheduler runs it this loop.
      autos.autonomousStarted();
    } else if (!autonomous && wasAutonomous) {
      autos.autonomousEnded();
    }
    wasAutonomous = autonomous;

    drive.periodic(enabled);
    loop.mark("Drive", System.nanoTime());
    long read = RobotController.getTime();
    joints.read(read, enabled);
    if (camerasOnJoints) {
      // Where each camera on a joint is, for the images it takes.
      localization.service().joints(read, joints.positions());
    }
    loop.mark("Joints", System.nanoTime());
    localization.periodic(drive.readings(), enabled);
    // Told where it is after odometry, so a turn made in placing it isn't counted twice.
    String placedOn = matchField.placedOn();
    placed.ifPresent(pose -> localization.place(pose, PLACED, placedOn));
    vision.periodic(RobotController.getTime());
    loop.mark("Vision", System.nanoTime());
    frames = localization.update().frames();
    vision.record(frames);
    loop.mark("Localization", System.nanoTime());
    Pose2d pose = localization.pose();
    if (sim != null) {
      sim.log(pose);
    }
    matchField.update(
        pose, drive.getRobotRelativeVelocities(), sim != null ? sim.truePose() : null);
    loop.mark("Field", System.nanoTime());
    Dashboard.publish(status());
    loop.mark("Dashboard", System.nanoTime());
  }

  /**
   * What happens after the command scheduler, each loop: each joint does what its commands asked of
   * it this loop, or its idle.
   */
  public void afterCommands() {
    joints.write();
  }

  /**
   * What the robot reads and drives, for the mode the program runs in: the simulated world's, the
   * devices its definition's wiring names on a robot, or nothing in a replay, which reads the log.
   *
   * @param world the simulated world, in the simulator
   */
  static Hardware hardware(
      RunMode mode, RobotDefinition definition, @Nullable SimulatedWorld world) {
    return switch (mode) {
      case SIM -> new SimulatedHardware(Objects.requireNonNull(world));
      case REAL -> new RobotHardware(definition);
      case REPLAY -> Hardware.replay(definition);
    };
  }

  /** One line for the pit: how long loops take, and whether they keep up. */
  String loopStatus() {
    return loopHealth.describe();
  }

  /**
   * Plays on a venue from now on: localization solves images against its tags, the cameras' mount
   * checks judge against them, and places derived from tags move with them.
   */
  private void venue(FieldModel venue) {
    matchField.venue(venue);
    localization.service().venue(venue);
    vision.venue(venue);
  }

  /** The field as this match sets it: the alliance, the start, the placement, the zones. */
  MatchField matchField() {
    return matchField;
  }

  /** The robot's autos: the team's declared in its {@code Controls}, and the one chosen. */
  Autos autos() {
    return autos;
  }

  /** The robot's state, as the drive team's dashboard shows it, as of this loop. */
  RobotStatus status() {
    return RobotStatus.builder()
        .readyToScore(localization.readyToScore())
        .scoring(localization.scoring())
        .station(matchField.allianceStation())
        .onStart(matchField.placement().status() == Placement.Status.READY)
        .placement(matchField.placement().describe())
        .localization(localization.describe())
        .cameras(vision.summary())
        .loop(loopStatus())
        .build();
  }

  /** One line for the pit: what the pose rests on, and how sure it is. */
  String localizationStatus() {
    return localization.describe();
  }

  /** Whether the estimate is good enough to score from: bind scoring and aiming to it. */
  boolean readyToScore() {
    return localization.readyToScore();
  }

  /** One line for the pit and the drive team: ready to score, or why not. */
  String scoring() {
    return localization.scoring();
  }

  /** One line for the pit: the cameras, and how their measurements fare. */
  String cameras() {
    return vision.summary();
  }

  Vision vision() {
    return vision;
  }

  Localization localization() {
    return localization;
  }

  /**
   * What became of each camera image this loop, as localization decided it: for tests that check
   * what it made of them.
   */
  List<FrameVerdict> frames() {
    return frames;
  }

  /** The localization estimate, as of the last loop. */
  LocalizationEstimate estimate() {
    return localization.estimate();
  }

  /** The calibration procedures, for the Pitwall page. */
  Calibration calibration() {
    return calibration;
  }

  /** A pit check for the Pitwall page: drive forward slowly for two seconds. */
  Command driveCheck() {
    return drive.driveForward(0.5, 2.0);
  }

  /** The most often a status line that follows the sensors changes on the Pitwall page. */
  static final Duration STATUS_INTERVAL = Duration.ofSeconds(1);

  /** The drive check's name on the Pitwall page. */
  static final String DRIVE_CHECK = "Drive forward slowly (2 s)";

  /** The steering's experiment on the Pitwall page, and its motion. */
  static final String STEER_STEP = "Steer to 90° and back";

  static final Profile STEER_STEP_PROFILE =
      Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0.8));

  /**
   * Puts this robot's pit tools on its Pitwall page: the drive check and the calibrations as
   * actions, experiments for tuning the steering and each joint, and one status line each for the
   * loop, the alliance, the placement, localization, scoring, the last calibration, and the
   * cameras. In the simulator, the simulated world's parts that can fail, and the scenarios, for
   * the page to apply.
   */
  void addTo(Pitwall pitwall) {
    SimulatedWorld sim = world;
    if (sim != null) {
      pitwall
          .simulator()
          .ifPresent(
              simulator -> {
                sim.addTo(simulator.components());
                Scenarios.addTo(simulator.components(), sim);
              });
    }
    pitwall.devices().add(AutoPit.device(autos));
    pitwall.actions().add(DRIVE_CHECK, Actions.command(() -> scheduler, this::driveCheck));
    calibration.addTo(pitwall.actions(), scheduler);
    pitwall
        .experiments()
        .add(
            new Experiment(
                STEER_STEP, drive.steeringRig(tunings.drivetrain()), STEER_STEP_PROFILE));
    // Each joint's, from its places and speeds: nothing to write per joint.
    JointExperiments.of(joints, tunings.joints()).forEach(pitwall.experiments()::add);
    // The World view: the field, the robot where localization believes it is, its cameras' sight
    // lines with their verdicts, and in the simulator, the truth.
    pitwall
        .scenes()
        .add(
            WorldView.scene(
                matchField::field,
                robotDefinition,
                worldReadings(),
                sim != null ? () -> sim.drivetrain().truePose3d() : null,
                camera -> Optional.of(Calibration.cameraMountAction(camera))));
    // Each change goes to every open page: the lines that follow sensors change at most once a
    // second, and are written to change only when something worth reading does. Grouped (see
    // Status's javadoc) so the page shows the robot, the field, and the joints apart.
    pitwall
        .status()
        .add("Robot/Loop", this::loopStatus, STATUS_INTERVAL)
        .add("Field/Alliance", matchField::allianceStation)
        .add("Field/Venue", venues::active)
        .add("Field/Auto", autos::describe)
        .add("Field/Placement", () -> matchField.placement().describe(), STATUS_INTERVAL)
        .add("Field/Localization", this::localizationStatus, STATUS_INTERVAL)
        .add("Field/Scoring", this::scoring)
        .add("Field/Calibration", calibration::lastResult)
        .add("Field/Cameras", this::cameras, STATUS_INTERVAL)
        // Joints already raise these as alerts; the page shows the same two checks.
        .add("Mechanisms/Switches trusted", () -> joints.switchDoubts().isEmpty())
        .add("Mechanisms/Power channels", () -> joints.deadChannels().isEmpty());
  }

  Drive drive() {
    return drive;
  }

  /** What the robot is, as configured: the definition its {@code Controls} is given. */
  RobotDefinition definition() {
    return robotDefinition;
  }

  /** Every driven joint's mechanism, for behavior code to look up by its joint. */
  Joints joints() {
    return joints;
  }

  /** Every command bound to the drive team's gamepads, as the bindings were made. */
  Bindings bindings() {
    return bindings;
  }

  /** The driver's gamepad, port 0. */
  RecordingGamepad driver() {
    return driver;
  }

  /** The operator's gamepad, port 1. */
  RecordingGamepad operator() {
    return operator;
  }

  /** What the World view reads each loop while a page shows it: localization's and the wheels'. */
  private WorldView.Readings worldReadings() {
    return new WorldView.Readings() {
      @Override
      public LocalizationEstimate estimate() {
        return localization.estimate();
      }

      @Override
      public double pitchRadians() {
        return localization.pitchRadians();
      }

      @Override
      public double rollRadians() {
        return localization.rollRadians();
      }

      @Override
      public SwerveModuleVelocity[] modules() {
        return drive.moduleVelocities();
      }

      @Override
      public List<FrameVerdict> frames() {
        return frames;
      }
    };
  }

  /** The drivetrain's configuration as the pit tunes it. */
  LiveConfig<DrivetrainControl> drivetrainTuning() {
    return tunings.drivetrain();
  }

  /** The simulator's world, or null outside the simulator. */
  @Nullable SimulatedWorld world() {
    return world;
  }

  /** The simulator's drivetrain physics, or null outside the simulator. */
  @Nullable DriveSimulation simulation() {
    SimulatedWorld sim = world;
    return sim != null ? sim.drivetrain() : null;
  }

  /** Releases what holds global resources (alerts), so a test can build another robot. */
  @Override
  public void close() {
    tunings.all().forEach(TunableRegistry::remove);
    drive.close();
    localization.close();
    joints.close();
    matchField.close();
    autos.close();
    venues.close();
    loopHealth.close();
    vision.close();
    if (world != null) {
      world.close();
    }
    hardware.close();
  }
}

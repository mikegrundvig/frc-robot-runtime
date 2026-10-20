package com.michaelgrundvig.frc.runtime;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Profile;
import com.michaelgrundvig.frc.pitwall.experiment.Profiles;
import com.michaelgrundvig.frc.pitwall.robot.RobotSession;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.world.SimulatedParts;
import com.michaelgrundvig.frc.pitwall.world.WorldScene;
import com.michaelgrundvig.frc.runtime.autos.AutoPit;
import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Bindings;
import com.michaelgrundvig.frc.runtime.behavior.Disabling;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.commands.Calibration;
import com.michaelgrundvig.frc.runtime.dashboard.Dashboard;
import com.michaelgrundvig.frc.runtime.dashboard.RobotStatus;
import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.hardware.RobotHardware;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.runtime.lib.timing.LoopMonitor;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimulation;
import com.michaelgrundvig.frc.runtime.sim.Faults;
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
import java.util.function.UnaryOperator;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;
import org.wpilib.tunable.TunableRegistry;

/**
 * Everything the robot is made of, built and run in one place. This class builds every mechanism on
 * the hardware for the mode the program runs in ({@link Hardware}), runs them all each loop, and
 * hands the team's {@code Controls} what behavior reaches: the joints, the drivetrain, the driver's
 * and the operator's gamepads, and the list its autos are declared in.
 *
 * <p>It is the template's file, not a team's. A team's code goes in {@code Controls} and {@code
 * Robots}, so the template's updates here never collide with it. It is also the one file that shows
 * how the whole robot fits together, and it is written to be read from the top:
 *
 * <ul>
 *   <li>the constructor builds the robot, in an order that matters and is explained there;
 *   <li>{@link #periodic()} is one loop of every mechanism before the commands run, as a list of
 *       steps, and {@link #afterCommands()} is what follows them;
 *   <li>{@link #addTo(Pitwall)} puts the robot's pit tools on its Pitwall page.
 * </ul>
 */
public final class RobotContainer implements AutoCloseable {
  private static final int DRIVER_PORT = 0;
  private static final int OPERATOR_PORT = 1;

  /** How well a drive team places the robot on its start: within about 5 cm and 2 degrees. */
  static final PoseCovariance PLACED = PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2));

  /** How straight a driver lines the robot up by eye for a heading reset: about 5 degrees. */
  static final double HEADING_BY_EYE_RADIANS = Math.toRadians(5);

  /** The most often a status line that follows the sensors changes on the Pitwall page. */
  static final Duration STATUS_INTERVAL = Duration.ofSeconds(1);

  /** The drive check's name on the Pitwall page. */
  static final String DRIVE_CHECK = "Drive forward slowly (2 s)";

  /** The name of the steering's experiment on the Pitwall page. */
  static final String STEER_STEP = "Steer to 90° and back";

  /** The motion of the steering's experiment. */
  static final Profile STEER_STEP_PROFILE =
      Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0.8));

  /**
   * What the pit can tune on a robot: each mechanism's configuration, with the limits it may be
   * tuned within. This is the one list of them. The container publishes these, and the tests check
   * every one: that its committed values are within its limits, that each value has a meaning, and
   * that each has a row in the values register (docs/values.md). A mechanism that gains tuning adds
   * it here.
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
    // The joints are kept in the definition's order, which is the order Pitwall's Tune view lists
    // them in.
    Map<String, LiveConfig<MechanismControl>> joints = new LinkedHashMap<>();
    for (Joint joint : Joints.driven(robot.definition())) {
      joints.put(joint.label(), JointMechanism.tuning(joint, ControllerSettings.control(joint)));
    }
    return new Tunings(
        Drive.tuning(robot.drivetrain(), robot.definition().drivetrain().freeSpeed()),
        Vision.tuning(robot.vision()),
        Collections.unmodifiableMap(joints));
  }

  private final Scheduler scheduler;

  /** What the robot is, as configured. */
  private final RobotDefinition definition;

  /** What the pit can tune, as published. Every one is removed again as the robot closes. */
  private final Tunings tunings;

  /** What the robot reads and drives: its own devices, the simulator's, or none in a replay. */
  private final Hardware hardware;

  /** The simulator's world, which knows where the robot really is. Only the simulator has one. */
  private final Optional<SimulatedWorld> world;

  // The field and the match.

  /** The field as this match sets it: the alliance, the start, the placement, the zones. */
  private final MatchField matchField;

  /** The venue the pit selected, which takes the official field's place. */
  private final Venues venues;

  /** The autos the team declares, and the one the pit chose, which this class runs. */
  private final Autos autos;

  /** Whether autonomous was enabled in the last loop, to notice it starting and ending. */
  private boolean wasAutonomous;

  /** Cancels every command in the loop the robot is disabled. */
  private final Disabling disabling;

  // The mechanisms.

  private final Drive drive;

  /** Every driven joint's mechanism, and the joints without wiring on a robot. */
  private final Joints joints;

  /** Whether any camera rides on a joint. If one does, localization needs the joints' positions. */
  private final boolean camerasOnJoints;

  private final Vision vision;
  private final Localization localization;

  /** What the estimator did with the last loop's camera frames. */
  private List<FrameVerdict> frames = List.of();

  private final Calibration calibration;

  // The drive team's controls.

  /** Every command bound to the drive team's gamepads, as the bindings were made. */
  private final Bindings bindings;

  private final RecordingGamepad driver;
  private final RecordingGamepad operator;

  // The loop's timing.

  /** Whether the loop keeps up. The robot starts and finishes each loop on it. */
  private final LoopHealth loopHealth;

  /** Where this class marks the end of each section of a loop, so each section is timed. */
  private final LoopMonitor loop;

  /**
   * Builds the robot. {@link Robot#assemble} calls this, for the robot and for tests alike.
   *
   * <p>The order of the steps matters in three ways. Each part is handed the parts it reads, so
   * those come first. Each part raises its alerts and starts its logs as it is made, so the order
   * here is the order they appear in. And the command scheduler runs its periodic tasks in the
   * order they were added: the bindings add theirs before any gate that the team's controls make.
   *
   * @param runsOn the computer the program runs on: a laptop driving real devices opens only those
   *     on its CANivore
   * @param scheduler runs every command: the controls', and the Pitwall page's actions
   * @param robot the robot: its frame, wiring, and cameras
   * @param field the field the robot plays on, with its official layout
   * @param venues the venue the pit selected, which takes the field's place while disabled
   * @param controls what the robot does and the drive team's buttons for it. It is the team's own,
   *     and is made once the mechanisms are.
   * @param simulatedErrors in the simulator, how the robot's wheels and sensors differ from its
   *     configuration ({@link SensorErrors#TYPICAL} makes its estimated pose drift from the truth
   *     as a real robot's does)
   * @param loopHealth the loop's timing. The robot starts and finishes each loop on it, and this
   *     class marks the sections. It is closed with this class.
   * @param watched a test's wrapper around the hardware, through which the hardware passes on its
   *     way to the mechanisms, so that the test can record what they are told ({@code RobotSim}).
   *     Outside such a test it hands the hardware on as it is.
   */
  RobotContainer(
      RunMode mode,
      RunsOn runsOn,
      Scheduler scheduler,
      RobotConfig robot,
      FieldModel field,
      Venues venues,
      ControlsMaker<?> controls,
      SensorErrors simulatedErrors,
      LoopHealth loopHealth,
      UnaryOperator<Hardware> watched) {
    this.scheduler = scheduler;
    this.venues = venues;
    this.loopHealth = loopHealth;
    loop = loopHealth.monitor();
    definition = robot.definition();

    // What the pit can tune. The mechanisms read these every loop, so a tuned value takes effect
    // at once.
    tunings = published(tunings(robot));

    // The match, and the drive team's gamepads.
    autos = new Autos(scheduler);
    disabling = new Disabling(scheduler);
    matchField = new MatchField(field, mode, autos);
    bindings = new Bindings(scheduler, RobotState::isEnabled);
    driver = bindings.gamepad("driver", DRIVER_PORT);
    operator = bindings.gamepad("operator", OPERATOR_PORT);

    // What the robot reads and drives.
    world = simulatedWorld(mode, field, simulatedErrors);
    hardware = watched.apply(hardware(mode, runsOn, definition, world));

    // Localization comes before the drivetrain, which asks it where the robot is.
    localization = new Localization(localizationService(field, robot));
    drive =
        new Drive(
            "Drive",
            hardware.drive(),
            definition,
            tunings.drivetrain()::get,
            scheduler,
            localization::pose);
    // The drivetrain is handed the driver's sticks and the field, so that its own commands (driven,
    // facing) read them. The team's Controls can then build assisted driving without holding them.
    drive.takeDriver(
        this::leftStick, () -> -driver.getRightX(), matchField::field, matchField::alliance);
    // Until the robot is placed it could be anywhere. Localization starts it in the middle of the
    // field and as unsure as the field is big, so that the first camera measurement wins. The
    // simulated robot is stood in the same spot.
    if (world.isPresent()) {
      world.get().place(middleOf(field));
    }

    // Every driven joint. Each is a v3 mechanism that the template builds from the joint's block
    // in the definition, which says how the joint is driven and wired, and each is driven as the
    // pit tunes it.
    camerasOnJoints = definition.cameras().stream().anyMatch(camera -> camera.joint().isPresent());
    joints =
        new Joints(
            definition,
            hardware,
            joint -> Objects.requireNonNull(tunings.joints().get(joint.label()))::get,
            scheduler);

    // The cameras: PhotonVision on a robot, its simulation in the simulator, the log in a replay.
    List<CameraConfig> cameras = definition.cameras();
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

    // The template's own two bindings, which are there whatever the team's Controls binds. The
    // sticks drive: this is the drivetrain's default command. And the driver's back button resets
    // the heading. They are made before the team's, so a second command on back is a doubt that
    // the robot says at startup.
    //
    // The heading reset is the one binding that starts while the robot is disabled too. A drive
    // team lines the robot up and presses back before the match, and the reset moves nothing. The
    // back button is asked for by its index because that hands back the gamepad's own kind of
    // trigger, which has that way to bind. It is the same trigger driver.back() is.
    drive.setDefaultCommand(drive.driven());
    driver
        .button(Gamepad.Button.BACK.value)
        .onTrueAlsoWhileDisabled(
            Command.noRequirements(coroutine -> faceDownfield()).named("Reset heading"));

    // What the robot does, and the drive team's buttons for it: the team's own class. Its
    // constructor builds the commands, binds them and declares its autos, so nothing here needs to
    // keep it.
    controls.make(scheduler, definition, joints, drive, driver, operator, autos);

    // What the robot doubts or does not know is said last, once the team's bindings exist, because
    // two commands bound to one button in the same way are among the doubts.
    Doubts.say(definition, Doubts.of(definition, bindings), mode);
  }

  // ---- Building the robot ----

  /** Publishes what the pit can tune, where dashboards and the Pitwall page find it. */
  private static Tunings published(Tunings tunings) {
    TunableRegistry.publish("Drive/config", tunings.drivetrain());
    TunableRegistry.publish("Vision/config", tunings.vision());
    tunings
        .joints()
        .forEach((label, tuning) -> TunableRegistry.publish("Joints/" + label + "/config", tuning));
    return tunings;
  }

  /** The simulated world, which only the simulator has. */
  private Optional<SimulatedWorld> simulatedWorld(
      RunMode mode, FieldModel field, SensorErrors simulatedErrors) {
    if (mode != RunMode.SIM) {
      return Optional.empty();
    }
    return Optional.of(new SimulatedWorld(field, definition, simulatedErrors));
  }

  /**
   * What the robot reads and drives, for the mode the program runs in. In the simulator it is the
   * simulated world's. On a robot it is the devices that the definition's wiring names, and on a
   * laptop driving real devices those of them it reaches, on its CANivore. In a replay it is
   * nothing, because a replay reads the log.
   *
   * @param runsOn the computer the program runs on
   * @param world the simulated world, in the simulator
   */
  static Hardware hardware(
      RunMode mode, RunsOn runsOn, RobotDefinition definition, Optional<SimulatedWorld> world) {
    return switch (mode) {
      case SIM -> new SimulatedHardware(world.orElseThrow());
      case REAL -> new RobotHardware(definition, runsOn);
      case REPLAY -> Hardware.replay(definition);
    };
  }

  /** Localization with the localizer the robot chose, or the default one. */
  private LocalizationService localizationService(FieldModel field, RobotConfig robot) {
    return new LocalizationService(
        field, definition, robot.localizerOn(field), RobotLoop.PERIOD_SECONDS);
  }

  private static Pose2d middleOf(FieldModel field) {
    return new Pose2d(field.length() / 2, field.width() / 2, Rotation2d.ZERO);
  }

  // ---- The driver's sticks ----

  /**
   * The driver's left stick as a direction on the field. Pushed forward, it points away from the
   * driver. A gamepad's Y axis is negative when the stick is pushed forward, so it is negated, and
   * the result is then turned to the way the driver faces: a red driver's forward is the field's
   * -x.
   */
  private Translation2d leftStick() {
    return new Translation2d(-driver.getLeftY(), -driver.getLeftX())
        .rotateBy(matchField.driverForward());
  }

  /**
   * Resets the heading. By pressing back, the driver says that the robot faces straight downfield,
   * away from the driver's own wall. The robot's heading becomes its alliance's forward, as sure as
   * lining up by eye allows, and its position stays. This is the fallback for a heading that has
   * gone wrong, and it works whether or not the robot is enabled. If the driver was wrong, the
   * cameras' headings will agree against the new one, and the estimate turns back to them.
   */
  void faceDownfield() {
    localization.resetHeading(matchField.driverForward(), HEADING_BY_EYE_RADIANS);
  }

  // ---- Each loop ----

  /**
   * One loop of every mechanism, before the command scheduler runs. The steps are in the order they
   * must happen in:
   *
   * <ol>
   *   <li>The pit's choices are taken, and the match is read. Both can move the robot's start, so
   *       they come before anything reads where the robot is.
   *   <li>In the simulator, the world moves on, so that the sensors read this loop's world.
   *   <li>The sensors are read: the drivetrain's, then the joints'.
   *   <li>Localization works out where the robot is from what the sensors and the cameras read.
   *   <li>The field's checks judge where that is, and the dashboard shows the result.
   * </ol>
   *
   * <p>The commands run after all of this ({@link Robot#loop}), so they decide on this loop's
   * readings and this loop's pose.
   */
  public void periodic() {
    boolean enabled = RobotState.isEnabled();
    takeThePitsChoices(enabled);
    Optional<Pose2d> placed = matchField.read(enabled);
    stepTheSimulatedWorld(placed);
    startOrEndTheAuto();
    readTheSensors(enabled);
    localize(placed, enabled);
    judgeTheField();
    Dashboard.publish(status());
    loop.mark("Dashboard", System.nanoTime());
  }

  /**
   * Cancels every running command in the loop the robot goes from enabled to disabled ({@link
   * Disabling}). Disabled means disabled: nothing a command was doing is left to carry on when the
   * robot is enabled again. This is the one place it happens, so a team's commands need nothing.
   *
   * <p>The robot's loop calls this between {@link #periodic()} and the command scheduler. That is
   * after the auto is ended, so that an auto cut off by a disable is still recorded as cut off as
   * autonomous ended. And it is after the sensors are read, because that is when the drivetrain and
   * the joints are told the robot is disabled, and a move they cancel says so.
   */
  public void cancelTheCommandsIfDisabled() {
    disabling.cancelCommandsIfDisabled(RobotState.isEnabled());
  }

  /**
   * What happens after the command scheduler, each loop: each joint does what its commands asked of
   * it this loop, or what it does idle. Because this comes after the commands, what a command asks
   * is applied in the same loop.
   */
  public void afterCommands() {
    joints.write();
  }

  /**
   * Takes what the pit chose since the last loop: a venue and an auto. Both take effect only at the
   * start of a loop, and only while the robot is disabled. The auto is taken here, before the match
   * is read, because the chosen auto names the robot's start.
   */
  private void takeThePitsChoices(boolean enabled) {
    Optional<FieldModel> newVenue = venues.update(!enabled, RobotState.isFMSAttached());
    if (newVenue.isPresent()) {
      playOn(newVenue.get());
    }
    autos.update(!enabled);
  }

  /**
   * Plays on a venue from now on. Localization solves camera frames against the venue's tags, the
   * cameras' mount checks judge against them, and the places derived from tags move with them.
   */
  private void playOn(FieldModel venue) {
    matchField.venue(venue);
    localization.service().venue(venue);
    vision.venue(venue);
  }

  /**
   * In the simulator, moves the world on by one loop. First the drive team really moves the robot,
   * if it placed it this loop. Then the world steps, before the robot reads its sensors, as the
   * real world moves between a robot's loops: the gyro feels in this loop the turn made in this
   * loop.
   *
   * @param placed where the drive team put the robot this loop, if it did
   */
  private void stepTheSimulatedWorld(Optional<Pose2d> placed) {
    if (world.isEmpty()) {
      return;
    }
    SimulatedWorld simulated = world.get();
    if (placed.isPresent()) {
      simulated.place(placed.get());
    }
    simulated.step(RobotController.getTime());
    loop.mark("Simulation", System.nanoTime());
  }

  /**
   * Starts the chosen auto in the loop autonomous becomes enabled, and ends it in the loop
   * autonomous stops. The auto's command is made fresh as it starts, and the command scheduler runs
   * it later in this same loop.
   */
  private void startOrEndTheAuto() {
    boolean autonomous = RobotState.isAutonomousEnabled();
    if (autonomous && !wasAutonomous) {
      vision.autonomousStarted(RobotController.getTime());
      autos.autonomousStarted();
    } else if (!autonomous && wasAutonomous) {
      autos.autonomousEnded();
    }
    wasAutonomous = autonomous;
  }

  /** Reads the drivetrain's sensors, then the joints'. */
  private void readTheSensors(boolean enabled) {
    drive.periodic(enabled);
    loop.mark("Drive", System.nanoTime());

    long readAt = RobotController.getTime();
    joints.read(readAt, enabled);
    if (camerasOnJoints) {
      // Localization needs to know where each camera on a joint was, for the frames it took.
      localization.service().joints(readAt, joints.positions());
    }
    loop.mark("Joints", System.nanoTime());
  }

  /**
   * Works out where the robot is. Odometry goes first. Only then is the robot told where the drive
   * team placed it, so that a turn made in placing it is not counted a second time as a turn the
   * wheels made. The cameras' frames are read next, and the estimator then weighs all of it.
   *
   * @param placed where the drive team put the robot this loop, if it did. It is on its chosen
   *     start. On a robot with cameras, the tags will soon say where it is instead.
   */
  private void localize(Optional<Pose2d> placed, boolean enabled) {
    localization.periodic(drive.readings(), enabled);
    if (placed.isPresent()) {
      localization.place(placed.get(), PLACED, matchField.placedOn());
    }
    vision.periodic(RobotController.getTime());
    loop.mark("Vision", System.nanoTime());

    frames = localization.update().frames();
    vision.record(frames);
    loop.mark("Localization", System.nanoTime());
  }

  /**
   * Judges where the robot is on the field: its placement on its start, the zones it is in, and
   * whether it is on the field at all. In the simulator, the world also logs how far the estimate
   * is from the truth.
   */
  private void judgeTheField() {
    Pose2d pose = localization.pose();
    Optional<Pose2d> truth = Optional.empty();
    if (world.isPresent()) {
      world.get().log(pose);
      truth = Optional.of(world.get().truePose());
    }
    matchField.update(pose, drive.getRobotRelativeVelocities(), truth);
    loop.mark("Field", System.nanoTime());
  }

  // ---- What the pit and the dashboard read ----

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

  /** One line for the pit: how long loops take, and whether they keep up. */
  String loopStatus() {
    return loopHealth.describe();
  }

  /** One line for the pit: what the pose rests on, and how sure it is. */
  String localizationStatus() {
    return localization.describe();
  }

  /** Whether the estimate is good enough to score from. Scoring and aiming are bound to this. */
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

  /** A pit check for the Pitwall page: drive forward slowly for two seconds. */
  Command driveCheck() {
    return drive.driveForward(0.5, 2.0);
  }

  // ---- The Pitwall page ----

  /**
   * Puts this robot's pit tools on its Pitwall page. The order here is the order the page lists
   * them in.
   */
  void addTo(Pitwall pitwall) {
    addTheSimulatorsControls(pitwall);
    pitwall.devices().add(AutoPit.device(autos));
    pitwall.actions().add(DRIVE_CHECK, Actions.command(() -> scheduler, this::driveCheck));
    calibration.addTo(pitwall.actions(), scheduler);
    addTheExperiments(pitwall);
    addTheWorldView(pitwall);
    addTheStatusLines(pitwall);
  }

  /**
   * In the simulator, gives the page the simulated world's parts that can fail, and the scenarios,
   * for the page to apply.
   */
  private void addTheSimulatorsControls(Pitwall pitwall) {
    if (world.isEmpty() || pitwall.simulator().isEmpty()) {
      return;
    }
    Components components = pitwall.simulator().get().components();
    world.get().addTo(components);
    Scenarios.addTo(components, world.get());
  }

  /**
   * The experiments for tuning: one for the steering, and one for each joint. A joint's experiment
   * is made from its places and speeds, so there is nothing to write for each joint.
   */
  private void addTheExperiments(Pitwall pitwall) {
    pitwall
        .experiments()
        .add(
            new Experiment(
                STEER_STEP, drive.steeringRig(tunings.drivetrain()), STEER_STEP_PROFILE));
    JointExperiments.of(joints, tunings.joints()).forEach(pitwall.experiments()::add);
  }

  /**
   * The World view: the field, the robot where localization believed it was, its cameras' sight
   * lines with what became of each image, and, in the simulator, where the robot really was.
   * Pitwall draws it from the robot's record, loop by loop, so there is one only when Pitwall has
   * the robot's session of its log.
   */
  private void addTheWorldView(Pitwall pitwall) {
    Optional<RobotSession> session = pitwall.session();
    if (session.isEmpty()) {
      return;
    }
    // Only a simulator's record says where the robot really is.
    // And only a simulated robot has parts that can be made to fail where they are clicked.
    WorldScene.Truth truth = WorldScene.Truth.NOT_KNOWN;
    SimulatedParts simulated = SimulatedParts.NONE;
    if (world.isPresent()) {
      truth = WorldScene.Truth.IN_THE_RECORD;
      simulated = Faults.parts();
    }
    WorldScene scene =
        WorldScene.of(
            definition,
            matchField::field,
            truth,
            Vision.sightingWords(),
            camera -> Optional.of(Calibration.cameraMountAction(camera)),
            simulated);
    pitwall.scenes().add(scene.scene(session.get()));
  }

  /**
   * One status line each for the loop, the alliance, the venue, the auto, the placement,
   * localization, scoring, the last calibration, the cameras, and two checks of the mechanisms.
   *
   * <p>Each change of a line goes to every open page. The lines that follow the sensors change at
   * most once a second, and are worded so that they change only when something worth reading does.
   * The names are grouped (see {@code Status}'s Javadoc), so that the page shows the robot, the
   * field and the mechanisms apart.
   */
  private void addTheStatusLines(Pitwall pitwall) {
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
        // The joints already raise these two as alerts. The page shows the same two checks.
        .add("Mechanisms/Switches trusted", () -> joints.switchDoubts().isEmpty())
        .add("Mechanisms/Power channels", () -> joints.deadChannels().isEmpty());
  }

  // ---- The parts, for tests and for the robot program ----

  /** The field as this match sets it: the alliance, the start, the placement, the zones. */
  MatchField matchField() {
    return matchField;
  }

  /** The robot's autos: the ones its {@code Controls} declares, and the one chosen. */
  Autos autos() {
    return autos;
  }

  Vision vision() {
    return vision;
  }

  Localization localization() {
    return localization;
  }

  /**
   * What became of each camera frame this loop, as localization decided it. Tests check what it
   * made of them.
   */
  List<FrameVerdict> frames() {
    return frames;
  }

  /**
   * The default localizer's estimate, as of the last loop: for the tests of that localizer.
   *
   * @throws IllegalStateException if the robot chose another localizer
   */
  LocalizationEstimate estimate() {
    return localization.estimate();
  }

  /** The calibration procedures, for the Pitwall page. */
  Calibration calibration() {
    return calibration;
  }

  Drive drive() {
    return drive;
  }

  /** What the robot is, as configured: the definition its {@code Controls} is given. */
  RobotDefinition definition() {
    return definition;
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

  /** The drivetrain's configuration as the pit tunes it. */
  LiveConfig<DrivetrainControl> drivetrainTuning() {
    return tunings.drivetrain();
  }

  /** The simulator's world. Only the simulator has one. */
  Optional<SimulatedWorld> world() {
    return world;
  }

  /** The simulator's drivetrain physics. Only the simulator has any. */
  Optional<DriveSimulation> simulation() {
    return world.map(SimulatedWorld::drivetrain);
  }

  /** Releases what holds global resources, such as alerts, so that a test can build another. */
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
    if (world.isPresent()) {
      world.get().close();
    }
    hardware.close();
  }
}

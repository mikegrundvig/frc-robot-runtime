package com.michaelgrundvig.frc.runtime;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Profile;
import com.michaelgrundvig.frc.pitwall.experiment.Profiles;
import com.michaelgrundvig.frc.pitwall.robot.RobotSession;
import com.michaelgrundvig.frc.pitwall.world.SimulatedParts;
import com.michaelgrundvig.frc.pitwall.world.WorldScene;
import com.michaelgrundvig.frc.runtime.autos.AutoPit;
import com.michaelgrundvig.frc.runtime.commands.Calibration;
import com.michaelgrundvig.frc.runtime.coprocessors.BoardPit;
import com.michaelgrundvig.frc.runtime.hardware.World;
import com.michaelgrundvig.frc.runtime.subsystems.drive.SwerveDrive;
import com.michaelgrundvig.frc.runtime.subsystems.localization.Localization;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointExperiments;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraPit;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import java.time.Duration;
import java.util.Optional;
import org.wpilib.command3.Command;

/**
 * The robot's pit tools on its Pitwall page: its cards, its actions, its experiments, its World
 * view and its status lines, each asked of the part that owns it ({@link RobotContainer}'s). {@link
 * Robot#assemble} puts them on the page once the robot is built.
 */
final class RobotPitwall {
  /** The most often a status line that follows the sensors changes on the Pitwall page. */
  static final Duration STATUS_INTERVAL = Duration.ofSeconds(1);

  /** The drive check's name on the Pitwall page. */
  static final String DRIVE_CHECK = "Drive forward slowly (2 s)";

  /** The name of the steering's experiment on the Pitwall page. */
  static final String STEER_STEP = "Steer to 90° and back";

  /** The motion of the steering's experiment. */
  static final Profile STEER_STEP_PROFILE =
      Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0.8));

  private RobotPitwall() {}

  /**
   * Puts a robot's pit tools on its Pitwall page. The order here is the order the page lists them
   * in.
   *
   * @param robot the robot, built
   * @param mode what the program runs as, which says where the cameras' frames come from
   * @param config the robot's configuration, which lists its coprocessors: the only places the page
   *     loads a camera's live view from
   */
  static void addTo(Pitwall pitwall, RobotContainer robot, RunMode mode, RobotConfig config) {
    addTheSimulatorsControls(pitwall, robot.world());
    pitwall.devices().add(AutoPit.device(robot.autos()));
    CameraPit.devices(robot.vision(), cameraFrames(mode), robot.tunings().cameras())
        .forEach(pitwall.devices()::add);
    CameraPit.pictureOrigins(config.coprocessors()).forEach(pitwall.devices()::picturesFrom);
    BoardPit.devices(robot.coprocessorHealth()).forEach(pitwall.devices()::add);
    pitwall
        .actions()
        .add(DRIVE_CHECK, Actions.command(robot::scheduler, () -> driveCheck(robot.drive())));
    robot.calibration().addTo(pitwall.actions(), robot.scheduler());
    addTheExperiments(pitwall, robot);
    addTheWorldView(pitwall, robot);
    addTheStatusLines(pitwall, robot);
  }

  /** Where the cameras' frames come from in a mode: the coprocessors only on a robot. */
  static CameraPit.Frames cameraFrames(RunMode mode) {
    return switch (mode) {
      case REAL -> CameraPit.Frames.FROM_THE_COPROCESSORS;
      case SIM -> CameraPit.Frames.FROM_THE_SIMULATOR;
      case REPLAY -> CameraPit.Frames.FROM_THE_LOG;
    };
  }

  /** A pit check for the Pitwall page: drive forward slowly for two seconds. */
  static Command driveCheck(SwerveDrive drive) {
    return drive.driveForward(0.5, 2.0);
  }

  /**
   * In the simulator, gives the page the simulated world's parts that can fail, and the scenarios,
   * for the page to apply.
   */
  private static void addTheSimulatorsControls(Pitwall pitwall, Optional<World> world) {
    if (world.isEmpty() || pitwall.simulator().isEmpty()) {
      return;
    }
    world.get().addControlsTo(pitwall.simulator().get().components());
  }

  /**
   * The experiments for tuning: one for the steering, and one for each joint. A joint's experiment
   * is made from its places and speeds, so there is nothing to write for each joint.
   */
  private static void addTheExperiments(Pitwall pitwall, RobotContainer robot) {
    pitwall
        .experiments()
        .add(
            new Experiment(
                STEER_STEP,
                robot.drive().steeringRig(robot.tunings().drivetrain()),
                STEER_STEP_PROFILE));
    JointExperiments.of(robot.joints(), robot.tunings().joints())
        .forEach(pitwall.experiments()::add);
  }

  /**
   * The World view: the field, the robot where localization believed it was, its cameras' sight
   * lines with what became of each image, and, in the simulator, where the robot really was.
   * Pitwall draws it from the robot's record, loop by loop, so there is one only when Pitwall has
   * the robot's session of its log.
   */
  private static void addTheWorldView(Pitwall pitwall, RobotContainer robot) {
    Optional<RobotSession> session = pitwall.session();
    if (session.isEmpty()) {
      return;
    }
    // Only a simulator's record says where the robot really is.
    // And only a simulated robot has parts that can be made to fail where they are clicked.
    WorldScene.Truth truth = WorldScene.Truth.NOT_KNOWN;
    SimulatedParts simulated = SimulatedParts.NONE;
    Optional<World> world = robot.world();
    if (world.isPresent()) {
      truth = WorldScene.Truth.IN_THE_RECORD;
      simulated = world.get().parts();
    }
    MatchField matchField = robot.matchField();
    WorldScene scene =
        WorldScene.of(
            robot.definition(),
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
  private static void addTheStatusLines(Pitwall pitwall, RobotContainer robot) {
    MatchField matchField = robot.matchField();
    Localization localization = robot.localization();
    Joints joints = robot.joints();
    pitwall
        .status()
        .add("Robot/Loop", robot.loopHealth()::describe, STATUS_INTERVAL)
        .add("Field/Alliance", matchField::allianceStation)
        .add("Field/Venue", robot.venues()::active)
        .add("Field/Auto", robot.autos()::describe)
        .add("Field/Placement", () -> matchField.placement().describe(), STATUS_INTERVAL)
        .add("Field/Localization", localization::describe, STATUS_INTERVAL)
        .add("Field/Scoring", localization::scoring)
        .add("Field/Calibration", robot.calibration()::lastResult)
        .add("Field/Cameras", robot.vision()::summary, STATUS_INTERVAL)
        // The joints already raise these two as alerts. The page shows the same two checks.
        .add("Mechanisms/Switches trusted", () -> joints.switchDoubts().isEmpty())
        .add("Mechanisms/Power channels", () -> joints.deadChannels().isEmpty());
  }
}

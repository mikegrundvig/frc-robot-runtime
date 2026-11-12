// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.AdvantageKitLog;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.robot.RobotSession;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import com.michaelgrundvig.frc.runtime.coprocessors.VisorConfigs;
import com.michaelgrundvig.frc.runtime.dashboard.DashboardStream;
import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.hardware.PhoenixBuses;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.runtime.hardware.Simulation;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.SimulatorWiring;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import com.michaelgrundvig.frc.runtime.venues.VenueIO;
import com.michaelgrundvig.frc.runtime.venues.VenueIODisk;
import com.michaelgrundvig.frc.runtime.venues.VenuePit;
import com.michaelgrundvig.frc.runtime.venues.VenueStore;
import com.michaelgrundvig.frc.runtime.venues.Venues;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.game.Game;
import java.lang.management.ManagementFactory;
import java.nio.file.Path;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.system.Filesystem;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Mass;
import org.wpilib.util.Alert;

/**
 * The robot program, as WPILib runs it. As it starts, it sets up the log, starts Pitwall, and puts
 * the robot together ({@link #assemble}). After that, every loop is {@link #loop}, in every mode:
 * the command scheduler runs between the mechanisms' sensors and their outputs.
 *
 * <p>The parts of this that need neither WPILib's robot base nor a log are static, so that tests
 * build the robot and run the loop with the very same code ({@code RobotSim}).
 */
public class Robot extends LoggedRobot {
  /** Set by {@code ./gradlew run}: how the simulated robot starts, as a {@link SimStart}'s name. */
  static final String START_PROPERTY = "frc.sim.start";

  /** Set by {@code ./gradlew run -Pscenario=<name>}: the simulator scenario to start in. */
  static final String SCENARIO_PROPERTY = "frc.sim.scenario";

  /** The drive a robot's log goes to: its USB drive. */
  static final String LOG_DRIVE = "/U";

  /** How {@code ./gradlew run} starts the simulated robot. */
  enum SimStart {
    /**
     * Ready to drive, and disabled. This is the default, without WPILib's simulator GUI. The robot
     * is in teleop on the blue alliance's station 2 with a Driver Station attached. Pitwall's Sim
     * view enables it with one press, right beside where the keyboard driving is switched on.
     */
    READY,
    /**
     * As {@link #READY}, and enabled. This is the start with the GUI ({@code -Pgui}), because the
     * GUI's keyboard drives the robot at once, and the GUI is on screen to disable it.
     */
    ENABLED,
    /**
     * As a real robot starts, before its Driver Station connects: disabled, with no station and no
     * mode ({@code -PstartDisabled}). Tests and replays never set the property, so they start this
     * way too.
     */
    DISABLED;

    /**
     * The start that a property's value names, whatever its case. A missing value, or one that
     * names no start, is {@link #DISABLED}.
     */
    static SimStart named(Optional<String> name) {
      for (SimStart start : values()) {
        if (name.isPresent() && start.name().equalsIgnoreCase(name.get())) {
          return start;
        }
      }
      return DISABLED;
    }
  }

  /** What the loop does when a section of it throws ({@link LoopSection}). */
  enum AfterAThrow {
    /**
     * The loop carries on, and the throw is reported. A robot does this, so that one bug does not
     * end a match. A replay of a robot does too, because it follows the robot's path.
     */
    CARRIES_ON,
    /**
     * The throw ends the program at once. The simulator does this, and so do the tests that run it,
     * so that a bug fails there.
     */
    ENDS_THE_PROGRAM
  }

  /** What the program runs as: on a robot, in the simulator, or replaying a log. */
  private final RunMode mode;

  /**
   * The computer it runs on: a robot's Systemcore, or a laptop. A laptop in {@link RunMode#REAL}
   * drives real devices through CTRE's hardware-attached simulation ({@link HardwareAttached}).
   */
  private final RunsOn runsOn;

  /** The folder the log is written to, on a robot and in the simulator ({@link #logFolder}). */
  private final Path logFolder;

  /** Whether the loop keeps up, and its log with it. */
  private final LoopHealth loopHealth = new LoopHealth();

  /** A debug deploy ({@code deploy -PdebugMode}) waits for a debugger every time it starts. */
  private final Alert debugDeploy =
      new Alert(
          "Robot",
          "debugDeploy",
          "A debug deploy: after any restart the robot waits for a debugger. Deploy without"
              + " -PdebugMode before a match",
          Alert.Level.HIGH);

  /**
   * Pitwall, the robot's own web page: its status, its actions, and live tuning of its
   * configuration.
   */
  private final Pitwall pitwall;

  /** Every mechanism, and the controls that drive them. */
  private final RobotContainer container;

  /**
   * In the simulator, what happens to the robot in the scenario that was asked for (a collision, a
   * slip). The events wait here until the robot is first enabled, and then happen once.
   */
  private Optional<List<Setting>> pendingEvents;

  /**
   * Starts the robot program. {@link RobotProgram#start} hands it what the team supplies.
   *
   * <p>The log is set up and started first, so that everything built afterwards is logged from its
   * first value. Pitwall is started before the robot is put together, because the mechanisms read
   * their configuration through it.
   *
   * <p>Where the page is served, Pitwall is handed the robot's session: the robot's definition, and
   * the last few minutes of the log, which the log hands it loop by loop. Its Robot view draws the
   * robot's joints from that.
   */
  public Robot(RobotProgram program) {
    this(program, RunMode.current(), RunsOn.current(), logFolder(RunsOn.current()));
  }

  /**
   * Starts the robot program as it runs in a mode, on a computer, with its log in a folder. A test
   * starts a robot's program this way, on a laptop, as it runs on a robot.
   *
   * @param logFolder the folder the log is written to ({@link #logFolder}); a replay writes its log
   *     beside the one it replays, wherever that is
   */
  Robot(RobotProgram program, RunMode mode, RunsOn runsOn, Path logFolder) {
    super(RobotLoop.PERIOD_SECONDS);
    this.mode = mode;
    this.runsOn = runsOn;
    this.logFolder = logFolder;
    recordWhichCodeThisIs(program.build());
    // Which controller this is, in the log and on the console: a team with several units tells them
    // apart by it.
    String serial = RobotController.getSerialNumber();
    Logger.recordMetadata("ControllerSerial", serial);
    System.out.println("Controller serial: " + serial);
    Optional<Simulation> simulation =
        simulation(mode, Optional.ofNullable(System.getProperty(SCENARIO_PROPERTY)));
    Optional<Simulation.ScenarioAsked> scenario = simulation.flatMap(Simulation::scenario);
    if (scenario.isPresent()) {
      // Every log of a scenario names it.
      Logger.recordMetadata("SimScenario", scenario.get().name());
      System.out.println(
          "Simulator: scenario " + scenario.get().name() + ": " + scenario.get().description());
    }
    RobotConfig robot = program.robot().get();
    boolean hardwareAttached = HardwareAttached.applies(mode, runsOn);
    if (hardwareAttached) {
      // Before anything else is opened: real motors are about to be, from a laptop.
      HardwareAttached.banner(robot.definition()).forEach(System.out::println);
      Logger.recordMetadata("HardwareAttached", "real devices driven from a laptop");
      // The team number Systemcore would give it, before the coprocessors are told the robot's
      // address by it.
      OptionalInt team =
          HardwareAttached.team(
              Optional.ofNullable(System.getProperty(HardwareAttached.TEAM_PROPERTY)));
      team.ifPresent(RoboRioSim::setTeamNumber);
      HardwareAttached.coprocessorLines(robot.coprocessors().size(), team)
          .forEach(line -> System.out.println("Hardware-attached: " + line));
    }
    Optional<RobotSession> session = sessionFor(mode, robot);
    chooseWhereTheLogGoes(session);
    Logger.start();
    debugDeploy.set(
        mode == RunMode.REAL
            && runsOn == RunsOn.SYSTEMCORE
            && startedForDebugger(ManagementFactory.getRuntimeMXBean().getInputArguments()));

    if (session.isPresent()) {
      pitwall = Pitwall.start(robot.name(), session.get(), HardwareAttached.pitwallKind(mode));
    } else {
      pitwall = Pitwall.start(robot.name());
    }
    container =
        assemble(
            mode,
            runsOn,
            Scheduler.getDefault(),
            robot,
            program.game(),
            program.controls(),
            pitwall,
            simulation,
            loopHealth,
            venueStore(mode, runsOn),
            UnaryOperator.identity());
    if (hardwareAttached) {
      // Once the devices are opened: whether the laptop sees each CANivore they're on.
      HardwareAttached.canivores(robot.definition()).stream()
          .map(PhoenixBuses::status)
          .forEach(line -> System.out.println("Hardware-attached: " + line));
    }
    // The scenario's sensor faults are applied now, before the first loop, through the simulator's
    // controls, so the Pitwall page shows them and can undo them. Its events wait until the robot
    // is enabled.
    pendingEvents = startScenario(simulation, simulatorsControls());
  }

  // ---- Starting ----

  /** Records in the log's metadata exactly which code produced the log. */
  private static void recordWhichCodeThisIs(BuildInfo build) {
    Logger.recordMetadata("ProjectName", build.projectName());
    Logger.recordMetadata("BuildDate", build.buildDate());
    Logger.recordMetadata("GitSHA", build.gitSha());
    Logger.recordMetadata("GitDate", build.gitDate());
    Logger.recordMetadata("GitBranch", build.gitBranch());
    Logger.recordMetadata("GitDirty", build.changes().text());
  }

  /**
   * The robot's session for Pitwall's page, wherever the page is served: on a robot and in the
   * simulator. A replay serves no page, so it keeps no session.
   */
  static Optional<RobotSession> sessionFor(RunMode mode, RobotConfig robot) {
    if (mode == RunMode.REPLAY) {
      return Optional.empty();
    }
    return Optional.of(
        RobotSession.of(
            robot.definition(),
            RobotLoop.PERIOD_SECONDS,
            RobotSession.REMEMBERED_SECONDS,
            Vision.sightingWords()));
  }

  /**
   * Chooses where the log is written and streamed, and in a replay where it is read from.
   *
   * @param session Pitwall's session of the robot, which takes every loop's record beside the log's
   *     file and its stream; empty in a replay
   */
  private void chooseWhereTheLogGoes(Optional<RobotSession> session) {
    switch (mode) {
      case REAL -> {
        // On a robot the log goes to the USB drive, and is streamed to dashboards. On the field the
        // stream is thinned, because the radio's bandwidth is capped and the drive keeps
        // everything anyway. On a laptop driving real devices, it goes to logs/, as in the
        // simulator, and there's no USB drive to watch.
        logReceivers(mode, logFolder).forEach(Logger::addDataReceiver);
        if (runsOn == RunsOn.SYSTEMCORE) {
          Path drive = Objects.requireNonNull(logFolder.getParent(), "a log folder on a drive");
          loopHealth.watchLogDrive(drive.toFile());
        }
      }
      case SIM -> {
        // In the simulator the log is streamed to AdvantageScope, and every session is also kept
        // as a file in logs/.
        logReceivers(mode, logFolder).forEach(Logger::addDataReceiver);
      }
      case REPLAY -> {
        // A replay runs a log through this code as fast as it can, and writes what this code makes
        // of it to a log beside the original, with "_sim" in its name, to compare.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }
    // Pitwall's Robot view is drawn from the log itself: its session takes every loop's record.
    session.ifPresent(taking -> Logger.addDataReceiver(AdvantageKitLog.receiver(taking)));
  }

  /**
   * The folder the log is written to on a computer: on a robot, a logs folder on its USB drive
   * ({@value #LOG_DRIVE}); on a laptop, in the simulator or driving real devices, the program's own
   * logs/. Those are the folders AdvantageKit's writer picks for itself on each.
   */
  static Path logFolder(RunsOn runsOn) {
    if (runsOn == RunsOn.SYSTEMCORE) {
      return Path.of(LOG_DRIVE, "logs");
    }
    return Path.of("logs");
  }

  /**
   * What the log is written to and streamed to, on a robot and in the simulator: a file in the log
   * folder, and NetworkTables for dashboards, guarded so a throw stops the stream and not the log,
   * and on a robot thinned on the field ({@link DashboardStream}). A replay has its own, beside the
   * log it replays.
   */
  static List<LogDataReceiver> logReceivers(RunMode mode, Path logFolder) {
    WPILOGWriter file = new WPILOGWriter(logFolder.toString());
    DashboardStream.OnTheField onTheField = DashboardStream.OnTheField.EVERY_LOOP;
    if (mode == RunMode.REAL) {
      onTheField = DashboardStream.OnTheField.THINNED;
    }
    return List.of(file, new DashboardStream(new NT4Publisher(), onTheField));
  }

  /** Whether the program was started for a debugger to attach (JDWP), as a debug deploy is. */
  static boolean startedForDebugger(List<String> jvmArguments) {
    return jvmArguments.stream().anyMatch(argument -> argument.contains("jdwp"));
  }

  /**
   * Where venues are kept. On a robot that is the robot's disk. On a laptop, in the simulator or
   * driving real devices, it is the program's own folder, {@code build/venues/}. A replay has none,
   * because it takes the venue from the log. Tests have none either: they use the official layout
   * unless they say otherwise.
   */
  static Optional<VenueStore> venueStore(RunMode mode, RunsOn runsOn) {
    if (mode == RunMode.REPLAY) {
      return Optional.empty();
    }
    Path committed = HardwareAttached.deployDirectory().resolve("venues");
    return Optional.of(new VenueStore(venueFolder(runsOn), committed));
  }

  /**
   * Where the pit's venues are kept on this computer: on a robot, its disk; on a laptop, the
   * program's own folder, {@code build/venues/}, whether it simulates or drives real devices.
   */
  static Path venueFolder(RunsOn runsOn) {
    if (runsOn == RunsOn.SYSTEMCORE) {
      return VenueStore.ROBOT_FOLDER;
    }
    return Filesystem.getOperatingDirectory().toPath().resolve("build/venues");
  }

  /**
   * Puts the robot together: every mechanism, on the field it plays on, with its pit tools and its
   * status on the Pitwall page. The mechanisms read their configuration through Pitwall, so a tuned
   * value takes effect at once. Tests build the robot here too ({@code RobotSim}), so they test the
   * robot that the robot program runs.
   *
   * @param runsOn the computer the program runs on: a laptop driving real devices opens only those
   *     on its CANivore
   * @param game this season's field, venues and limits: a robot not yet weighed is taken at the
   *     most the limits allow, the one place its mass is settled
   * @param controls what the robot does, made once its mechanisms are built
   * @param simulation in the simulator, the simulator, which makes the world the robot drives in
   *     ({@link #simulation})
   * @param venueStore where venues are kept, if this program keeps any. With one, the pit can
   *     upload and choose venues on the Pitwall page.
   * @param watched a test's wrapper around the hardware, to record what the mechanisms tell it. On
   *     a robot it hands the hardware on as it is.
   */
  static RobotContainer assemble(
      RunMode mode,
      RunsOn runsOn,
      Scheduler scheduler,
      RobotConfig robot,
      Game game,
      ControlsMaker<?> controls,
      Pitwall pitwall,
      Optional<Simulation> simulation,
      LoopHealth loopHealth,
      Optional<VenueStore> venueStore,
      UnaryOperator<Hardware> watched) {
    // A camera on Visor that Visor couldn't run on the field stops the program here, in Visor's
    // words, before anything is opened or any board is configured.
    FieldModel officialField = game.officialField();
    VisorConfigs.requireRunnable(robot.definition().cameras(), officialField);
    Optional<VenueIODisk> disk = venueStore.map(VenueIODisk::new);
    // Without a store nothing is ever selected, so the robot stays on the official layout.
    VenueIO selection = disk.isPresent() ? disk.get() : new VenueIO() {};
    Venues venues = new Venues(game, selection);
    Mass robotMass = game.limits().robotMass(robot.definition().body());
    RobotContainer container =
        new RobotContainer(
            mode,
            runsOn,
            scheduler,
            robot,
            officialField,
            robotMass,
            venues,
            controls,
            simulation,
            loopHealth,
            watched);
    RobotPitwall.addTo(pitwall, container, mode, robot);
    if (venueStore.isPresent() && disk.isPresent()) {
      pitwall.devices().add(VenuePit.device(venueStore.get(), disk.get(), venues, game));
    }
    return container;
  }

  // ---- Simulator scenarios ----

  /**
   * The simulator, in its mode: the robot program's one call into the simulator, which wires the
   * simulated world behind the {@link Simulation} it hands back. Only a physics simulation has one:
   * a replay runs what its log recorded, and a robot has no simulator. The architecture test allows
   * the simulator's classes here and nowhere else in the robot program.
   *
   * @param scenario the simulator scenario's name, if one was asked for
   * @throws IllegalArgumentException listing every scenario, when none has this name. A scenario is
   *     asked for by name, so a wrong name stops the program.
   */
  static Optional<Simulation> simulation(RunMode mode, Optional<String> scenario) {
    if (mode != RunMode.SIM) {
      return Optional.empty();
    }
    // In the simulator, the sensors err as a season-worn robot's do.
    return Optional.of(SimulatorWiring.start(SensorErrors.TYPICAL, scenario));
  }

  /**
   * Starts the scenario that was asked for, if one was: applies its sensor faults now, through the
   * simulator's controls.
   *
   * @param simulation the simulator, which only the simulator's mode has, with the world it made
   * @param controls the simulator's controls' parts that can fail, where Pitwall has them
   * @return what happens to the robot in the scenario, which is to happen once the robot is
   *     enabled; empty when no scenario was asked for
   * @throws IllegalStateException when a scenario was asked for and there are no simulator's
   *     controls to apply it through, or one of its faults cannot be applied. The scenario was
   *     asked for by name, so the program stops and does not run without it.
   */
  static Optional<List<Setting>> startScenario(
      Optional<Simulation> simulation, Optional<Components> controls) {
    if (simulation.isEmpty()) {
      return Optional.empty();
    }
    Optional<Simulation.ScenarioAsked> asked = simulation.get().scenario();
    if (asked.isEmpty()) {
      return Optional.empty();
    }
    if (controls.isEmpty()) {
      throw new IllegalStateException(
          "Simulator scenario "
              + asked.get().name()
              + ": it needs Pitwall's simulator controls, and this program has none");
    }
    apply(simulation.get().scenarioFaults(), controls.get());
    return Optional.of(simulation.get().scenarioEvents());
  }

  /**
   * Makes a scenario's events happen in the simulated world, if the robot is enabled.
   *
   * @param pending the events that wait to happen, if any still do
   * @param controls the simulator's controls, which only the simulator has
   * @return the events that still wait: the same ones, or none once they have happened
   */
  static Optional<List<Setting>> startWhenEnabled(
      Optional<List<Setting>> pending, Optional<Components> controls, boolean enabled) {
    if (pending.isEmpty() || controls.isEmpty() || !enabled) {
      return pending;
    }
    apply(pending.get(), controls.get());
    return Optional.empty();
  }

  /**
   * Applies a scenario's settings through the simulator's controls. A setting that changes nothing,
   * because the part is already that way as built, is passed over.
   *
   * @throws IllegalStateException saying why, when one cannot be applied. The scenario was asked
   *     for by name, so the program stops and does not run without it.
   */
  static void apply(List<Setting> settings, Components controls) {
    Optional<String> refused = Optional.ofNullable(controls.apply(settings));
    if (refused.isPresent()) {
      throw new IllegalStateException("Simulator scenario: " + refused.get());
    }
  }

  /** The simulator's controls' parts that can fail, where Pitwall has a simulator. */
  private Optional<Components> simulatorsControls() {
    return pitwall.simulator().map(Simulator::components);
  }

  // ---- For tests that drive the robot program ----

  /** What the program is running as. */
  RunMode mode() {
    return mode;
  }

  /** The mechanisms. */
  RobotContainer container() {
    return container;
  }

  /** Pitwall. */
  Pitwall pitwall() {
    return pitwall;
  }

  /** Whether the loop keeps up, and its log with it. */
  LoopHealth loopHealth() {
    return loopHealth;
  }

  // ---- Every loop ----

  @Override
  public void robotPeriodic() {
    loop(pitwall, container, Scheduler.getDefault(), loopHealth, afterAThrow(mode));
  }

  /** What the loop does when a section of it throws, for the mode the program runs in. */
  static AfterAThrow afterAThrow(RunMode mode) {
    if (mode == RunMode.SIM) {
      return AfterAThrow.ENDS_THE_PROGRAM;
    }
    return AfterAThrow.CARRIES_ON;
  }

  /**
   * One loop, in every mode. Tests run the robot with this too ({@code RobotSim}). Each step is
   * timed, and the order is what makes a loop act on its own readings:
   *
   * <ol>
   *   <li>The Pitwall page's requests are taken, so that what the pit asked is in force for the
   *       whole loop.
   *   <li>Every mechanism reads its sensors, and the robot works out where it is ({@link
   *       RobotContainer#periodic}).
   *   <li>The command scheduler runs. Its bindings and commands decide on this loop's readings. In
   *       the loop the robot is disabled, every running command is canceled first ({@link
   *       RobotContainer#cancelTheCommandsIfDisabled}), so the scheduler runs nothing left over
   *       from before.
   *   <li>The joints are told what the commands asked ({@link RobotContainer#afterCommands}), so a
   *       goal set in this loop is applied in this loop.
   * </ol>
   *
   * <p><b>When the loop throws.</b> Each section of the loop runs under a guard of its own ({@link
   * LoopSection}), and anything in one may throw: a command, a binding's condition, a sensor's
   * reading, a joint's output, the page, the dashboard. In the simulator, and in a test, a throw
   * ends the program at once, so a bug fails there. On a robot, and in a replay of one, the throw
   * is reported ({@link LoopHealth#threw}) and the sections after it still run:
   *
   * <ul>
   *   <li>A section that reads sensors leaves this loop's readings part new and part old. The
   *       commands don't run this loop, the drivetrain is told to stand still, and each joint does
   *       what it does idle. The commands run again in the next loop.
   *   <li>A command that throws has been ended by v3's scheduler, with the command that started it,
   *       and the joints still do what the other commands asked.
   *   <li>A joint whose output throws keeps what it was last told, and the other joints are told
   *       theirs.
   *   <li>The page, the coprocessors, the venues, the match, the field's checks and the dashboard
   *       serve the pit and the drive team. The robot drives the same without them.
   * </ul>
   *
   * <p>Only when the loop fails in {@value LoopHealth#FAILING_LOOPS} loops in a row, from the
   * sensors, the commands or the outputs, does the program end, and restart.
   *
   * @param afterAThrow what to do when a section throws ({@link #afterAThrow})
   */
  static void loop(
      Pitwall pitwall,
      RobotContainer container,
      Scheduler scheduler,
      LoopHealth loopHealth,
      AfterAThrow afterAThrow) {
    loopHealth.start(System.nanoTime(), afterAThrow);
    try {
      pitwall.periodic();
    } catch (RuntimeException e) {
      loopHealth.threw(LoopSection.PITWALL, e);
    }
    loopHealth.monitor().mark("Pitwall", System.nanoTime());
    container.periodic();
    runTheCommands(container, scheduler, loopHealth);
    loopHealth.monitor().mark("Commands", System.nanoTime());
    container.afterCommands();
    loopHealth.monitor().mark("Outputs", System.nanoTime());
    loopHealth.finish(System.nanoTime(), Logger.getReceiverQueueFault());
  }

  /**
   * Runs the command scheduler, after canceling every command if the robot was disabled since the
   * last loop. Canceling a command runs the team's own code for it, which may throw as any command
   * may, so it is under the same guard as the scheduler.
   *
   * <p>In a loop whose sensors threw, the commands are canceled at a disable all the same, because
   * disabled means disabled, but the scheduler does not run: its commands would decide on readings
   * that are part this loop's and part the last one's.
   */
  private static void runTheCommands(
      RobotContainer container, Scheduler scheduler, LoopHealth loopHealth) {
    try {
      container.cancelTheCommandsIfDisabled();
      if (loopHealth.thisLoop() == LoopHealth.ThisLoop.ON_UNTRUSTED_READINGS) {
        return;
      }
      scheduler.run();
    } catch (RuntimeException e) {
      loopHealth.threw(LoopSection.COMMANDS, e);
    }
  }

  // ---- The simulator ----

  /**
   * Starts the simulated robot the way {@code ./gradlew run} asked ({@link SimStart}). A robot
   * never enables itself before anyone is watching it, which is why the default start is disabled.
   */
  @Override
  public void simulationInit() {
    startAsAsked(mode, SimStart.named(Optional.ofNullable(System.getProperty(START_PROPERTY))));
  }

  /**
   * Readies or enables the simulated robot. It does so only in a physics simulation, and only when
   * asked.
   */
  static void startAsAsked(RunMode mode, SimStart start) {
    if (mode != RunMode.SIM || start == SimStart.DISABLED) {
      return;
    }
    if (MatchState.getAlliance().isEmpty()) {
      // Nothing has chosen a station yet. Blue 2 stands the robot on the center start, and blue's
      // forward is the field view's right: the easiest first drive.
      DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    }
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(start == SimStart.ENABLED);
    DriverStationSim.notifyNewData();
  }

  /**
   * Makes the scenario's events (a collision, a slip) happen on the first loop the robot is
   * enabled, so that the fault finds the robot where the drive team put it. That relies on two
   * things: WPILib runs this after {@link #robotPeriodic}, and the robot's container has put the
   * robot on its start in that loop at the latest.
   */
  @Override
  public void simulationPeriodic() {
    pendingEvents = startWhenEnabled(pendingEvents, simulatorsControls(), RobotState.isEnabled());
  }

  // WPILib calls one of these each loop for the mode the robot is in. This program does everything
  // in robotPeriodic, in every mode, so they are empty.

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void utilityPeriodic() {}
}

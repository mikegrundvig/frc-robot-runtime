// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import com.michaelgrundvig.frc.runtime.dashboard.DashboardStream;
import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.Scenario;
import com.michaelgrundvig.frc.runtime.sim.Scenarios;
import com.michaelgrundvig.frc.runtime.sim.SimulatedWorld;
import com.michaelgrundvig.frc.runtime.venues.VenueIO;
import com.michaelgrundvig.frc.runtime.venues.VenueIODisk;
import com.michaelgrundvig.frc.runtime.venues.VenuePit;
import com.michaelgrundvig.frc.runtime.venues.VenueStore;
import com.michaelgrundvig.frc.runtime.venues.Venues;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.game.Game;
import com.michaelgrundvig.frc.world.robot.Body;
import java.io.File;
import java.lang.management.ManagementFactory;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.UnaryOperator;
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
import org.wpilib.system.Filesystem;
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

  /** Where the robot's log goes: AdvantageKit writes to a logs folder on this drive. */
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

  /** What the loop does when the commands throw. */
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
  private final RunMode mode = RunMode.current();

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
   */
  public Robot(RobotProgram program) {
    super(RobotLoop.PERIOD_SECONDS);
    recordWhichCodeThisIs(program.build());
    Optional<Scenario> scenario =
        scenarioAskedFor(mode, Optional.ofNullable(System.getProperty(SCENARIO_PROPERTY)));
    if (scenario.isPresent()) {
      // Every log of a scenario names it.
      Logger.recordMetadata("SimScenario", scenario.get().name());
      System.out.println(
          "Simulator: scenario " + scenario.get().name() + ": " + scenario.get().description());
    }
    chooseWhereTheLogGoes();
    Logger.start();
    debugDeploy.set(
        mode == RunMode.REAL
            && startedForDebugger(ManagementFactory.getRuntimeMXBean().getInputArguments()));

    RobotConfig robot = program.robot().get();
    pitwall = Pitwall.start(robot.name());
    container =
        assemble(
            mode,
            Scheduler.getDefault(),
            robot,
            program.season(),
            program.controls(),
            pitwall,
            // In the simulator, the sensors err as a season-worn robot's do.
            SensorErrors.TYPICAL,
            loopHealth,
            venueStore(mode),
            UnaryOperator.identity());
    // The scenario's sensor faults are applied now, before the first loop, through the simulator's
    // controls, so the Pitwall page shows them and can undo them. Its events wait until the robot
    // is enabled.
    pendingEvents = startScenario(scenario, container.world(), simulatorsControls());
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

  /** Chooses where the log is written and streamed, and in a replay where it is read from. */
  private void chooseWhereTheLogGoes() {
    switch (mode) {
      case REAL -> {
        // On a robot the log goes to the USB drive, and is streamed to dashboards. On the field the
        // stream is thinned, because the radio's bandwidth is capped and the drive keeps
        // everything anyway.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new DashboardStream(new NT4Publisher()));
        loopHealth.watchLogDrive(new File(LOG_DRIVE));
      }
      case SIM -> {
        // In the simulator the log is streamed to AdvantageScope, and every session is also kept
        // as a file in logs/.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
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
  }

  /** Whether the program was started for a debugger to attach (JDWP), as a debug deploy is. */
  static boolean startedForDebugger(List<String> jvmArguments) {
    return jvmArguments.stream().anyMatch(argument -> argument.contains("jdwp"));
  }

  /**
   * Where venues are kept. On a robot that is the robot's disk. In the simulator it is the
   * simulator's own folder, {@code build/venues/}. A replay has none, because it takes the venue
   * from the log. Tests have none either: they use the official layout unless they say otherwise.
   */
  static Optional<VenueStore> venueStore(RunMode mode) {
    if (mode == RunMode.REPLAY) {
      return Optional.empty();
    }
    Path stored = VenueStore.ROBOT_FOLDER;
    if (mode == RunMode.SIM) {
      stored = Filesystem.getOperatingDirectory().toPath().resolve("build/venues");
    }
    Path committed = Filesystem.getDeployDirectory().toPath().resolve("venues");
    return Optional.of(new VenueStore(stored, committed));
  }

  /**
   * Puts the robot together: every mechanism, on the field it plays on, with its pit tools and its
   * status on the Pitwall page. The mechanisms read their configuration through Pitwall, so a tuned
   * value takes effect at once. Tests build the robot here too ({@code RobotSim}), so they test the
   * robot that the robot program runs.
   *
   * @param season this season's field and venues, for a robot of this robot's body
   * @param controls what the robot does, made once its mechanisms are built
   * @param simulatedErrors in the simulator, how the robot's wheels and sensors differ from its
   *     configuration
   * @param venueStore where venues are kept, if this program keeps any. With one, the pit can
   *     upload and choose venues on the Pitwall page.
   * @param watched a test's wrapper around the hardware, to record what the mechanisms tell it. On
   *     a robot it hands the hardware on as it is.
   */
  static RobotContainer assemble(
      RunMode mode,
      Scheduler scheduler,
      RobotConfig robot,
      Function<Body, Game> season,
      ControlsMaker<?> controls,
      Pitwall pitwall,
      SensorErrors simulatedErrors,
      LoopHealth loopHealth,
      Optional<VenueStore> venueStore,
      UnaryOperator<Hardware> watched) {
    Game game = season.apply(robot.definition().body());
    Optional<VenueIODisk> disk = venueStore.map(VenueIODisk::new);
    // Without a store nothing is ever selected, so the robot stays on the official layout.
    VenueIO selection = disk.isPresent() ? disk.get() : new VenueIO() {};
    Venues venues = new Venues(game, selection);
    RobotContainer container =
        new RobotContainer(
            mode,
            scheduler,
            robot,
            game.officialField(),
            venues,
            controls,
            simulatedErrors,
            loopHealth,
            watched);
    container.addTo(pitwall);
    if (venueStore.isPresent() && disk.isPresent()) {
      pitwall.devices().add(VenuePit.device(venueStore.get(), disk.get(), venues, game));
    }
    return container;
  }

  // ---- Simulator scenarios ----

  /**
   * The simulator scenario that was asked for. Only a physics simulation takes one: a replay runs
   * what its log recorded, and a robot has no simulator.
   *
   * @param name the scenario's name, if one was asked for
   * @throws IllegalArgumentException listing every scenario, when none has this name. A scenario is
   *     asked for by name, so a wrong name stops the program.
   */
  static Optional<Scenario> scenarioAskedFor(RunMode mode, Optional<String> name) {
    if (mode != RunMode.SIM || name.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(Scenarios.named(name.get()));
  }

  /**
   * Starts the scenario that was asked for, if one was: applies its sensor faults now, through the
   * simulator's controls.
   *
   * @param world the simulated world, which only the simulator has
   * @param controls the simulator's controls' parts that can fail, where Pitwall has them
   * @return what happens to the robot in the scenario, which is to happen once the robot is
   *     enabled; empty when no scenario was asked for
   * @throws IllegalStateException when a scenario was asked for and there is no simulated world, or
   *     no simulator's controls to apply it through, or one of its faults cannot be applied. The
   *     scenario was asked for by name, so the program stops and does not run without it.
   */
  static Optional<List<Setting>> startScenario(
      Optional<Scenario> asked, Optional<SimulatedWorld> world, Optional<Components> controls) {
    if (asked.isEmpty()) {
      return Optional.empty();
    }
    Scenario scenario = asked.get();
    if (world.isEmpty() || controls.isEmpty()) {
      String missing = world.isEmpty() ? "no simulated world" : "no simulator controls";
      throw new IllegalStateException(
          "Simulator scenario "
              + scenario.name()
              + ": it needs the simulated world and Pitwall's simulator controls, and this program"
              + " has "
              + missing);
    }
    apply(scenario.sensorFaultsIn(world.get()), controls.get());
    return Optional.of(scenario.eventsIn(world.get()));
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

  // ---- Every loop ----

  @Override
  public void robotPeriodic() {
    loop(pitwall, container, Scheduler.getDefault(), loopHealth, afterAThrow(mode));
  }

  /** What the loop does when the commands throw, for the mode the program runs in. */
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
   *   <li>The command scheduler runs. Its bindings and commands decide on this loop's readings.
   *   <li>The joints are told what the commands asked ({@link RobotContainer#afterCommands}), so a
   *       goal set in this loop is applied in this loop.
   * </ol>
   *
   * <p><b>When the commands throw.</b> A command, a binding's condition, or anything else the
   * scheduler runs may throw. In the simulator, and in a test, that ends the program at once, so a
   * bug fails there. On a robot, and in a replay of one, the loop carries on. By then v3's
   * scheduler has already ended the command that threw and the command that started it, and the
   * joints still do what the other commands asked. The throw is reported ({@link
   * LoopHealth#threw}). Only when the commands throw in {@value LoopHealth#FAILING_LOOPS} loops in
   * a row does the program end.
   *
   * @param afterAThrow what to do when the commands throw ({@link #afterAThrow})
   */
  static void loop(
      Pitwall pitwall,
      RobotContainer container,
      Scheduler scheduler,
      LoopHealth loopHealth,
      AfterAThrow afterAThrow) {
    loopHealth.start(System.nanoTime());
    pitwall.periodic();
    loopHealth.monitor().mark("Pitwall", System.nanoTime());
    container.periodic();
    runTheCommands(scheduler, loopHealth, afterAThrow);
    loopHealth.monitor().mark("Commands", System.nanoTime());
    container.afterCommands();
    loopHealth.monitor().mark("Outputs", System.nanoTime());
    loopHealth.finish(System.nanoTime(), Logger.getReceiverQueueFault());
  }

  private static void runTheCommands(
      Scheduler scheduler, LoopHealth loopHealth, AfterAThrow afterAThrow) {
    try {
      scheduler.run();
    } catch (RuntimeException e) {
      if (afterAThrow == AfterAThrow.ENDS_THE_PROGRAM) {
        throw e;
      }
      loopHealth.threw(e);
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

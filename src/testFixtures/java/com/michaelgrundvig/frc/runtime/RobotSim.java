package com.michaelgrundvig.frc.runtime;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Bindings;
import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.lib.timing.LoopMonitor;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimulation;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.runtime.sim.Scenario;
import com.michaelgrundvig.frc.runtime.sim.SimulatedWorld;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Placement;
import com.michaelgrundvig.frc.world.game.Game;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.extension.AfterEachCallback;
import org.junit.jupiter.api.extension.BeforeEachCallback;
import org.junit.jupiter.api.extension.ExtensionContext;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * The simulator, for tests: every test that touches WPILib's simulation (its clock, the Driver
 * Station, a simulated device) runs with this, and a test of the whole robot builds the robot here.
 *
 * <pre>{@code
 * @ExtendWith(RobotSim.class)                        // a mechanism, or any simulated hardware
 * @RegisterExtension final RobotSim sim = new RobotSim();   // the whole robot
 * }</pre>
 *
 * <p>Each test starts the same way, whatever ran before it: the clock stopped (it moves only when
 * the test steps it), the Driver Station attached and disabled on the blue alliance's station 2,
 * and fresh dashboards (WPILib's mock telemetry and tunable backends). Afterwards everything is
 * closed and reset, so nothing leaks into the next test. The HAL is shared by every test in a run:
 * change the Driver Station through this class, or call {@code notifyNewData()}, or the change
 * doesn't arrive.
 *
 * <p>The whole robot is built the way {@link Robot} builds it ({@link Robot#assemble}), Pitwall and
 * all, and {@link #loop()} runs the loop {@link Robot} runs, then what WPILib's loop does after it.
 * Unless a test builds another first, it is the simulator's robot ({@link ExampleRobot#SIM}) with
 * sensors exactly as configured, so poses come out exact.
 */
public final class RobotSim implements BeforeEachCallback, AfterEachCallback {
  /** The field the simulator's robot plays on. */
  public static final FieldModel FIELD = Rebuilt.field(ExampleRobot.SIM.definition().body());

  /** The simulator's robot without its cameras: odometry alone. */
  public static final RobotConfig WITHOUT_CAMERAS =
      RobotConfigBuilder.builder(ExampleRobot.SIM)
          .definition(
              RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                  .cameras(List.of())
                  .build())
          .build();

  private @Nullable MockTelemetryBackend telemetry;
  private @Nullable Scheduler scheduler;
  private @Nullable GamepadSim driver;
  private @Nullable GamepadSim operator;
  private @Nullable Pitwall pitwall;
  private @Nullable RobotContainer container;
  private @Nullable LoopHealth loopHealth;

  @Override
  public void beforeEach(ExtensionContext context) {
    assertTrue(HAL.initialize(), "simulated HAL failed to initialize");
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    setEnabled(false);
    telemetry = new MockTelemetryBackend();
    TelemetryRegistry.registerBackend("", telemetry);
    TunableRegistry.registerBackend("", new MockTunableBackend());
    scheduler = Scheduler.createIndependentScheduler();
    // Made after the Driver Station's data is reset, which clears a gamepad's axes.
    driver = new GamepadSim(0);
    operator = new GamepadSim(1);
  }

  @Override
  public void afterEach(ExtensionContext context) {
    closeRobot();
    JointsSimHarness.closeAll();
    TelemetryRegistry.reset();
    // What the robot published, every loop of it, let go: JUnit may hold a finished test's
    // instance, this with it, until its class is done.
    telemetry = null;
    TunableRegistry.reset();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  /** Builds the simulator's robot with sensors that err as a real robot's do, or not at all. */
  public RobotContainer build(SensorErrors errors) {
    return build(RunMode.SIM, ExampleRobot.SIM, errors);
  }

  /**
   * Builds the simulator's robot in a scenario ({@link
   * com.michaelgrundvig.frc.runtime.sim.Scenarios}), as {@code ./gradlew run -Pscenario=<name>}
   * does: the scenario's sensor faults on sensors that err as a season-worn robot's do, applied
   * through the simulator's controls before the first loop. What happens to the robot in it waits
   * until the test starts it ({@link #start}).
   */
  public RobotContainer build(Scenario scenario) {
    RobotContainer built = build(ExampleRobot.SIM, SensorErrors.TYPICAL);
    Robot.apply(scenario.sensorFaultsIn(world()), components());
    return built;
  }

  /** Builds a robot in the simulator. */
  public RobotContainer build(RobotConfig robot, SensorErrors errors) {
    return build(RunMode.SIM, robot, errors);
  }

  /**
   * Builds a robot, in place of any built before, as {@link Robot} does: in {@code REAL} mode it
   * talks to the hardware its configuration wires (CTRE's simulated devices, in a test), and in
   * {@code REPLAY} to nothing.
   *
   * @param errors in the simulator, how its wheels and sensors differ from its configuration
   */
  public RobotContainer build(RunMode mode, RobotConfig robot, SensorErrors errors) {
    return build(mode, robot, errors, UnaryOperator.identity());
  }

  /**
   * Builds a robot in the simulator, its hardware handed through a test's own on its way to the
   * robot's mechanisms: to record what each part is told, as {@code Monkey} does.
   */
  public RobotContainer build(
      RobotConfig robot, SensorErrors errors, UnaryOperator<Hardware> watched) {
    return build(RunMode.SIM, robot, errors, watched);
  }

  /**
   * Builds a robot in the simulator as a team's {@code Main} hands it to the runtime: with its
   * season's field and venues and its controls ({@code Rebuilt::game}, {@code Controls::new}).
   */
  public RobotContainer build(
      RobotConfig robot,
      Function<Body, Game> season,
      ControlsMaker<?> controls,
      SensorErrors errors) {
    return build(RunMode.SIM, robot, season, controls, errors, UnaryOperator.identity());
  }

  private RobotContainer build(
      RunMode mode, RobotConfig robot, SensorErrors errors, UnaryOperator<Hardware> watched) {
    return build(mode, robot, Rebuilt::game, ControlsMaker.NONE, errors, watched);
  }

  private RobotContainer build(
      RunMode mode,
      RobotConfig robot,
      Function<Body, Game> season,
      ControlsMaker<?> controls,
      SensorErrors errors,
      UnaryOperator<Hardware> watched) {
    closeRobot();
    LoopHealth health = new LoopHealth();
    Pitwall page = Pitwall.start(robot.name(), 0);
    pitwall = page;
    loopHealth = health;
    container =
        Robot.assemble(
            mode,
            scheduler(),
            robot,
            season,
            controls,
            page,
            errors,
            health,
            Optional.empty(),
            watched);
    return container;
  }

  /** The robot, built as the simulator's robot on first use unless the test built another. */
  public RobotContainer robot() {
    RobotContainer built = container;
    return built != null ? built : build(RunMode.SIM, ExampleRobot.SIM, SensorErrors.NONE);
  }

  /** The robot's Pitwall: its page's status, actions, and tuning. */
  public Pitwall pitwall() {
    robot();
    return Objects.requireNonNull(pitwall);
  }

  /** The simulator's physics: where the robot really is. */
  public DriveSimulation simulation() {
    return robot()
        .simulation()
        .orElseThrow(() -> new IllegalStateException("only the simulator has physics"));
  }

  /** The simulated world: the robot's physics and its cameras, which scenarios act on. */
  public SimulatedWorld world() {
    return robot()
        .world()
        .orElseThrow(() -> new IllegalStateException("only the simulator has a world"));
  }

  /**
   * Starts a scenario now: what happens to the robot in it (a collision, a slip) happens in the
   * simulated world. {@code ./gradlew run} starts it on the first loop the robot is enabled.
   */
  public void start(Scenario scenario) {
    Robot.apply(scenario.eventsIn(world()), components());
  }

  /**
   * The simulated world's parts that can fail, and the scenarios, as the simulator's controls on
   * the Pitwall page have them: apply a fault here as the page does.
   */
  public Components components() {
    return pitwall().simulator().orElseThrow().components();
  }

  /**
   * The drive team lifts the robot and sets it down at a pose, and tells it where: the simulated
   * robot is there, and so is the robot's estimate, as the placement check reads it. Between loops.
   */
  public void placeAt(Pose2d pose) {
    RobotContainer built = robot();
    world().place(pose);
    built.localization().place(pose, RobotContainer.PLACED, "test");
  }

  /** The pre-match placement check, as of the last loop: what the drive team reads. */
  public Placement placement() {
    return robot().matchField().placement();
  }

  /** The robot's autos: the team's, and the one the pit chose. */
  public Autos autos() {
    return robot().autos();
  }

  /** The robot's driven joints, for a team's behavior to reach through its parts. */
  public Joints joints() {
    return robot().joints();
  }

  /** The robot's drivetrain. */
  public Drive drive() {
    return robot().drive();
  }

  /** Where the robot thinks it is, as of the last loop. */
  public LocalizationEstimate estimate() {
    return robot().estimate();
  }

  /** The robot's command scheduler. */
  public Scheduler scheduler() {
    return Objects.requireNonNull(scheduler, "the test hasn't started");
  }

  /** The driver's gamepad, port 0. */
  public GamepadSim driver() {
    return Objects.requireNonNull(driver, "the test hasn't started");
  }

  /** The operator's gamepad, port 1. */
  public GamepadSim operator() {
    return Objects.requireNonNull(operator, "the test hasn't started");
  }

  /**
   * A team's controls on the robot, made as the robot makes its own {@code Controls}, with its
   * definition, joints, drivetrain, both gamepads and autos: {@code
   * sim.controls(BetaControls::new)}. Its bindings join the robot's record ({@link #bindings}), and
   * the autos it declares are the robot's ({@link RobotContainer#autos}).
   */
  public <T> T controls(ControlsMaker<T> controls) {
    RobotContainer robot = robot();
    return controls.make(
        scheduler(),
        robot.definition(),
        robot.joints(),
        robot.drive(),
        robot.driver(),
        robot.operator(),
        robot.autos());
  }

  /** Every command bound to the robot's gamepads, as the bindings were made. */
  public Bindings bindings() {
    return robot().bindings();
  }

  /** What the robot published, as a dashboard sees it. */
  public MockTelemetryBackend telemetry() {
    return Objects.requireNonNull(telemetry, "the test hasn't started");
  }

  /**
   * One robot loop, as WPILib's runs it: the clock moves a loop's time, WPILib reads the Driver
   * Station's newest data (which is how the simulator's controls on the Pitwall page reach the
   * robot, a loop after they're asked), then the robot runs {@link Robot#loop}, then WPILib's
   * tunables update. What the robot's commands throw, it throws, so a bug fails its test.
   */
  public void loop() {
    loop(Robot.AfterAThrow.ENDS_THE_PROGRAM);
  }

  /**
   * One robot loop as on a robot: what the robot's commands throw is reported, and the robot
   * carries on without them ({@link Robot#loop}).
   */
  void loopOnRobot() {
    loop(Robot.AfterAThrow.CARRIES_ON);
  }

  private void loop(Robot.AfterAThrow afterAThrow) {
    RobotContainer robot = robot();
    stepClock();
    DriverStationBackend.refreshData();
    Robot.loop(pitwall(), robot, scheduler(), Objects.requireNonNull(loopHealth), afterAThrow);
    TunableRegistry.update();
  }

  /** The loop's timing, as the robot program's loop monitor has it. */
  public LoopMonitor.Report lastLoop() {
    return Objects.requireNonNull(loopHealth).last();
  }

  /** {@code count} robot loops. */
  public void loops(int count) {
    for (int i = 0; i < count; i++) {
      loop();
    }
  }

  /** Robot loops for {@code seconds} of the robot's time. */
  public void run(double seconds) {
    loops(RobotLoop.loops(seconds));
  }

  /** Enables the robot in teleop, or disables it. */
  public static void setEnabled(boolean enabled) {
    enable(RobotMode.TELEOPERATED, enabled);
  }

  /** Puts the robot in a mode (autonomous, teleop, utility), enabled or not. */
  public static void enable(RobotMode mode, boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /** The Driver Station's alliance and station, as the drive team or the field sets it. */
  public static void station(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
  }

  /** Moves the simulated clock one robot loop, for a test that runs its own loop. */
  public static void stepClock() {
    SimHooks.stepTiming(RobotLoop.PERIOD_SECONDS);
  }

  private void closeRobot() {
    RobotContainer built = container;
    if (built != null) {
      built.close();
    }
    Pitwall page = pitwall;
    if (page != null) {
      page.close();
    }
    container = null;
    pitwall = null;
    loopHealth = null;
    scheduler = Scheduler.createIndependentScheduler();
  }
}

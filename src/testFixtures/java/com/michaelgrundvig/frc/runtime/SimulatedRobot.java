package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.VisionDecision;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.Scenario;
import com.michaelgrundvig.frc.runtime.sim.SimulatedWorld;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveHardware;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
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
 * The whole robot in the simulator, for a study: built the way {@link Robot} builds it and run loop
 * by loop the way {@link Robot} runs it, as the tests' {@code RobotSim} does, but outside a test.
 * The simulator's clock moves only when a loop runs, so a run takes as long as the computer needs
 * and repeats exactly.
 *
 * <p>WPILib's simulation is one per program (its clock, the Driver Station), so one robot runs at a
 * time: close it before building the next. Closing puts everything back as it was found, but the
 * clock, which stays paused: resumed, it would run on the wall clock until the next robot paused
 * it, and each run would start at a moment that hung on how long the last took.
 */
public final class SimulatedRobot implements AutoCloseable {
  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final GamepadSim driver;
  private final Pitwall pitwall;
  private final LoopHealth loopHealth = new LoopHealth();
  private final RobotContainer container;

  /**
   * Builds the robot, disabled on the blue alliance's station 2, as the drive team leaves it.
   *
   * @param errors how its wheels, sensors, and camera mounts differ from its configuration
   */
  public SimulatedRobot(RobotConfig robot, SensorErrors errors) {
    this(robot, errors, (name, camera) -> camera);
  }

  /**
   * The same, each camera's IO handed through {@code watched} on its way to the robot's code, by
   * the camera's name: a study reads there what each camera sent, or changes it, as a camera that
   * misreads a tag does.
   */
  public SimulatedRobot(
      RobotConfig robot, SensorErrors errors, BiFunction<String, CameraIO, CameraIO> watched) {
    if (!HAL.initialize()) {
      throw new IllegalStateException("WPILib's simulation failed to start");
    }
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    // Dashboards that keep what they're sent to themselves.
    TelemetryRegistry.registerBackend("", new MockTelemetryBackend());
    TunableRegistry.registerBackend("", new MockTunableBackend());
    setEnabled(false);
    // Made after the Driver Station's data is reset, which clears a gamepad's axes.
    driver = new GamepadSim(0);
    pitwall = Pitwall.start(robot.name(), 0);
    container =
        Robot.assemble(
            RunMode.SIM,
            RunsOn.DESKTOP,
            scheduler,
            robot,
            Rebuilt.GAME,
            ControlsMaker.NONE,
            pitwall,
            errors,
            loopHealth,
            Optional.empty(),
            hardware -> watchingCameras(hardware, robot, watched));
  }

  /** The robot's hardware as it is, each camera's IO handed through {@code watched}. */
  private static Hardware watchingCameras(
      Hardware hardware, RobotConfig robot, BiFunction<String, CameraIO, CameraIO> watched) {
    List<CameraIO> cameras = new ArrayList<>();
    List<CameraIO> real = hardware.cameras();
    for (int i = 0; i < real.size(); i++) {
      cameras.add(watched.apply(robot.definition().cameras().get(i).name(), real.get(i)));
    }
    return new Hardware() {
      @Override
      public DriveHardware drive() {
        return hardware.drive();
      }

      @Override
      public JointIO joint(Joint joint) {
        return hardware.joint(joint);
      }

      @Override
      public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
        return hardware.absoluteEncoder(joint);
      }

      @Override
      public SensorIO sensor(Sensor.Placed sensor) {
        return hardware.sensor(sensor);
      }

      @Override
      public PdhIO pdh() {
        return hardware.pdh();
      }

      @Override
      public List<CameraIO> cameras() {
        return List.copyOf(cameras);
      }

      @Override
      public void close() {
        hardware.close();
      }
    };
  }

  /** One robot loop: the clock moves a loop's time, the robot runs its loop, tunables update. */
  public void loop() {
    SimHooks.stepTiming(RobotLoop.PERIOD_SECONDS);
    // A bug ends a study at once.
    Robot.loop(pitwall, container, scheduler, loopHealth, Robot.AfterAThrow.ENDS_THE_PROGRAM);
    TunableRegistry.update();
  }

  /** Enables the robot in teleop, or disables it. */
  public void setEnabled(boolean enabled) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  /**
   * Holds the driver's sticks: forward and left as the driver sees them (blue's forward is the
   * field's +x), and turning counterclockwise, each from -1 to 1.
   */
  public void sticks(double forward, double left, double turn) {
    driver.setLeftY(-forward);
    driver.setLeftX(-left);
    driver.setRightX(-turn);
    driver.notifyNewData();
  }

  /**
   * Applies a scenario's sensor faults through the simulator's controls, as {@code ./gradlew run
   * -Pscenario=<name>} does before the robot's first loop: a camera knocked, a gyro erring.
   *
   * @throws IllegalStateException if one of them can't be applied, saying why
   */
  public void applyFaults(Scenario scenario) {
    Robot.apply(scenario.sensorFaultsIn(world()), components());
  }

  /**
   * Makes what happens in a scenario happen, now, through the simulator's controls: a collision, a
   * push.
   *
   * @throws IllegalStateException if it can't, saying why
   */
  public void start(Scenario scenario) {
    Robot.apply(scenario.eventsIn(world()), components());
  }

  /**
   * Applies faults through the simulator's controls, now: a camera unplugged, the robot shoved.
   *
   * @throws IllegalStateException if one of them can't be applied, saying why
   */
  public void apply(Setting... faults) {
    Robot.apply(List.of(faults), components());
  }

  /**
   * Undoes a fault that lasts, through the simulator's controls: a camera plugged back in.
   *
   * @throws IllegalStateException if it can't be undone, saying why
   */
  public void undo(String component, String fault) {
    String refused = components().undo(component, fault);
    if (refused != null) {
      throw new IllegalStateException("Simulator: " + refused);
    }
  }

  /** The names of the simulated world's cameras, in the robot's order. */
  public List<String> cameraNames() {
    return world().cameraNames();
  }

  /**
   * Tells localization the drive team placed the robot here, as sure as a placement on its start
   * is, wherever the robot really stands: a robot placed wrong believes this.
   */
  public void believePlaced(Pose2d pose) {
    container.localization().place(pose, RobotContainer.PLACED, "placed by the study");
  }

  /** What the drivetrain's sensors read in the last loop, as localization was given it. */
  public DrivetrainReadings readings() {
    return container.drive().readings();
  }

  /**
   * What the robot's localizer decided about each camera measurement it was given in the last loop.
   * Empty for a localizer that does not report its decisions.
   */
  public List<VisionDecision> visionDecisions() {
    return container.localization().service().localizer().decisions().orElse(List.of());
  }

  /**
   * Every camera measurement the robot's localizer was given in the last loop, in the order it was
   * given them: for a study that gives another localizer the same.
   */
  public List<VisionMeasurement> visionMeasurements() {
    return container.localization().service().given();
  }

  /** Where the robot's localizer has the robot, as of the last loop. */
  public Pose2d pose() {
    return container.localization().pose();
  }

  private SimulatedWorld world() {
    return container
        .world()
        .orElseThrow(() -> new IllegalStateException("only the simulator has a world"));
  }

  /**
   * The simulated world's parts that can fail, as the simulator's controls on Pitwall have them.
   */
  private Components components() {
    return pitwall.simulator().orElseThrow().components();
  }

  /** Starts the gyro-scale procedure, as the Pitwall page does. */
  public Command startGyroScaleProcedure() {
    Command procedure = container.calibration().gyroScale();
    scheduler.schedule(procedure);
    return procedure;
  }

  /** Whether a command is still running. */
  public boolean isRunning(Command command) {
    return scheduler.isScheduledOrRunning(command);
  }

  /** What the last calibration procedure reported. */
  public String lastCalibration() {
    return container.calibration().lastResult();
  }

  /** Where the robot really is. */
  public Pose2d truePose() {
    return container.simulation().orElseThrow().truePose();
  }

  /** How much less a camera is trusted for its mount, in meters ({@code MountCheck}). */
  public double mountDoubtMeters(String camera) {
    return container.vision().mountDoubtMeters(camera);
  }

  /** How far, on average, a camera's latest tags landed off; NaN until judged. */
  public double mountOffsetMeters(String camera) {
    return container.vision().mountOffsetMeters(camera);
  }

  /**
   * What the default localizer believes, as of the last loop.
   *
   * @throws IllegalStateException if the robot chose another localizer
   */
  public LocalizationEstimate estimate() {
    return container.estimate();
  }

  @Override
  public void close() {
    container.close();
    pitwall.close();
    TelemetryRegistry.reset();
    TunableRegistry.reset();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }
}

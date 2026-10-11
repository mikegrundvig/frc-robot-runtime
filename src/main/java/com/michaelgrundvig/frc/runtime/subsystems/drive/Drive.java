package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.experiment.Rig;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.path.Keyframe;
import com.michaelgrundvig.frc.world.path.Trajectories;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.LinearVelocity;

/**
 * The swerve drivetrain: four {@link Module}s and a gyro, as a Commands v3 {@link Mechanism}.
 *
 * <p>What it does for a team is its commands, each pre-built:
 *
 * <ul>
 *   <li>{@link #driven()} drives from the driver's sticks, and is the drive's default command;
 *   <li>{@link #facing} drives from the translation stick while the robot holds a heading by itself
 *       ({@link DriveAssist});
 *   <li>{@link #lockWheels()} makes the robot hard to push;
 *   <li>{@link #runUntil}, {@link #turnTo}, {@link #goTo(Pose2d)} and {@link #follow} are moves
 *       that end on their own, each with an outcome ({@link DriveMove}).
 * </ul>
 *
 * <p>The drivetrain does not know where the robot is. Each loop it reports what its sensors read
 * ({@link #readings()}) to localization, which owns the robot's pose and hands it back ({@link
 * #getPose()}).
 *
 * <p><b>Each loop</b>, {@link #periodic(boolean)} runs first, before localization and the command
 * scheduler: it takes any new tuning, reads the hardware, and stops the motors while the robot is
 * disabled. Then the command that owns the drivetrain runs, and tells the wheels what to do through
 * {@link #runVelocity(ChassisVelocities)}, which it must call every loop it drives.
 *
 * <p><b>Inside</b>, each part of that is a class of its own:
 *
 * <ul>
 *   <li>{@link DriveReadings}: what the drivetrain reads each loop, and what follows from it.
 *   <li>{@link DriveOutput}: what the modules are told, from a velocity asked of the robot.
 *   <li>{@link Sticks}: how the driver's sticks are shaped, and {@link Driver}, what the robot
 *       hands over for the drivetrain to be driven with.
 *   <li>{@link DriveAlerts}: what is wrong with the hardware, told to the drive team.
 *   <li>{@link SteeringRig}: the steering's test rig for the pit.
 *   <li>{@link WheelPoses}: the wheels as the log draws them.
 * </ul>
 *
 * <p>The drivetrain's configuration is read every loop, so a value tuned while the robot runs takes
 * effect at once: new gains and current limits go to the motor controllers in the loop they change.
 *
 * <p>Hardware comes in through the IO layer ({@link DriveHardware}): {@link PhoenixDrivetrain} on a
 * CTRE robot, the simulator's in the simulator ({@code com.michaelgrundvig.frc.runtime.sim}), the
 * log in a replay. A stand-in until AdvantageKit's 2027 swerve template ships: see
 * docs/stand-ins.md.
 */
@CheckReturnValue
public class Drive implements Mechanism, AutoCloseable {
  /** What a refusal says when the drive hasn't been handed the driver's sticks. */
  static final String NO_DRIVER =
      "the drivetrain has no driver's sticks: the robot hands them over as it builds the drive,"
          + " drive.takeDriver(...)";

  /** What a move to a place says when the drive hasn't been handed the field and the alliance. */
  static final String NO_FIELD =
      "the drivetrain has no field or alliance to find a place on: the robot hands them over as it"
          + " builds the drive, drive.takeDriver(...)";

  /** No acceleration: what plain driving at a velocity passes. */
  private static final ChassisAccelerations NO_ACCELERATION = new ChassisAccelerations();

  private final RobotDefinition robot;
  private final Scheduler scheduler;

  /** How the drivetrain is driven, as the pit tunes it, read each loop. */
  private final Supplier<DrivetrainControl> config;

  /** Where the robot is, from localization. */
  private final Supplier<Pose2d> pose;

  private final Module[] modules = new Module[Module.COUNT];
  private final SwerveDriveKinematics kinematics;

  /** What it reads each loop. */
  private final DriveReadings readings;

  /** What it tells its modules each loop. */
  private final DriveOutput output;

  private final DriveAlerts alerts;
  private final SteeringRig steeringRig;
  private final WheelPoses wheelPoses;

  /** The driver's sticks, the field and the alliance, once the robot has handed them over. */
  private Optional<Driver> driver = Optional.empty();

  // What the drivetrain is tuned to now.

  /** The configuration the motor controllers have, to notice when the pit changes it. */
  private DrivetrainControl applied;

  /** The speed full stick asks, and the fastest any wheel is told to roll. */
  private double topSpeedMetersPerSecond;

  /** The turning rate full stick asks. */
  private double topTurnRateRadiansPerSecond;

  /** How far ahead field-relative driving looks when it turns a velocity to the robot's axes. */
  private double skewLookaheadSeconds;

  /**
   * How fast the robot can speed up and spin up, and the tuning that was worked out for. The answer
   * depends only on the robot and its tuning, and working it out builds the robot's mass and its
   * motors' models afresh, so it is worked out again only when the tuning has changed.
   */
  private DrivetrainControl.Accelerations accelerations;

  private DrivetrainControl accelerationsAsTuned;

  /**
   * Why the template can't drive a drivetrain wired this way, or null when it can. So far its
   * modules are TalonFXs both driving and steering, or SPARKs doing both ({@link
   * PhoenixDrivetrain}). What the wiring itself can't do, such as a controller that doesn't drive
   * its motor, the robot's definition refuses before this is asked.
   */
  public static @Nullable String whyNotDriven(SwerveWiring wiring) {
    MechanismWiring.Controller drive = wiring.driveController();
    MechanismWiring.Controller steer = wiring.steerController();
    boolean bothTalons =
        drive == MechanismWiring.Controller.TALON_FX
            && steer == MechanismWiring.Controller.TALON_FX;
    boolean bothSparks = isSpark(drive) && isSpark(steer);
    if (bothTalons || bothSparks) {
      return null;
    }
    return "the drivetrain drives on "
        + drive.shown()
        + " and steers on "
        + steer.shown()
        + ", and the template's modules are TalonFXs or SPARKs, doing both, so far: put both on"
        + " TalonFXs, or both on SPARKs";
  }

  private static boolean isSpark(MechanismWiring.Controller controller) {
    return controller == MechanismWiring.Controller.SPARK_MAX
        || controller == MechanismWiring.Controller.SPARK_FLEX;
  }

  /**
   * The drivetrain's configuration as the pit tunes it: its gains and limits, each within the range
   * that's safe to dial in on a running robot (a range narrower than what's valid). Publish it, and
   * give the drivetrain its {@link LiveConfig#get}.
   *
   * @param committed its gains and limits, as its block in {@code Robots.java} commits them
   * @param freeSpeed the fastest its motors and gearing can turn its wheels, which its top speed is
   *     never tuned past: velocity control needs room under it to correct with
   */
  public static LiveConfig<DrivetrainControl> tuning(
      DrivetrainControl committed, LinearVelocity freeSpeed) {
    return LiveConfig.of("drivetrain", committed)
        .number("gains.driveKs", Limits.between(0, 1)) // volts
        .number("gains.driveKv", Limits.between(0, 5)) // volts per meter per second
        .number("gains.driveKp", Limits.between(0, 20))
        .number("gains.steerKp", Limits.between(0, 100))
        .number("gains.steerKd", Limits.between(0, 5))
        .measure("gains.skewLookahead", Seconds.of(0), Seconds.of(0.1))
        .measure("limits.driveStatorCurrent", Amps.of(10), Amps.of(120))
        .measure("limits.driveSupplyCurrent", Amps.of(10), Amps.of(80))
        // Never above a 40 A breaker's rating, and stepping down well before the fastest one can
        // trip at twice its rating (1.5 s at 80 A, the supply limit's top): docs/values.md has the
        // times.
        .measure("limits.driveSupplyLowerCurrent", Amps.of(10), Amps.of(40))
        .measure("limits.driveSupplyLowerTime", Seconds.of(0.1), Seconds.of(1))
        .measure("limits.steerStatorCurrent", Amps.of(5), Amps.of(80))
        .measure("limits.maxLinearSpeed", MetersPerSecond.of(0.5), freeSpeed)
        .measure("limits.maxAngularSpeed", RotationsPerSecond.of(0.1), RotationsPerSecond.of(3));
  }

  /**
   * @param name alert group name: "Drive" on the robot, another name for a second instance
   * @param hardware the modules, the gyro, and the backup IMU
   * @param robot the robot's definition: its frame and its drivetrain's geometry place the wheels
   * @param config how the drivetrain is driven, read every loop so tuned values take effect
   * @param pose where the robot is, from localization: field-relative driving turns by its heading
   */
  public Drive(
      String name,
      DriveHardware hardware,
      RobotDefinition robot,
      Supplier<DrivetrainControl> config,
      Scheduler scheduler,
      Supplier<Pose2d> pose) {
    this.config = config;
    this.robot = robot;
    this.scheduler = scheduler;
    this.pose = pose;
    for (int i = 0; i < Module.COUNT; i++) {
      modules[i] = new Module(hardware.modules().get(i), i, robot.drivetrain().module(i));
    }
    Translation2d[] moduleTranslations = robot.moduleTranslations();
    kinematics = new SwerveDriveKinematics(moduleTranslations);
    readings = new DriveReadings(modules, hardware, moduleTranslations, kinematics);
    output = new DriveOutput(modules, robot, kinematics);
    alerts = new DriveAlerts(name);
    steeringRig = new SteeringRig(modules, output);
    wheelPoses = new WheelPoses(modules, robot);
    applied = config.get();
    apply(applied);
    accelerationsAsTuned = applied;
    accelerations = DrivetrainControl.accelerations(robot, applied);
    // A move plans its path as it starts, on the robot's loop. Planning one here, as the robot
    // starts, has the planner's code loaded and compiled before the first move needs it.
    Trajectories.warmUp(RobotLoop.PERIOD_SECONDS);
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  // ---- Each loop ----

  /**
   * Starts the drivetrain's loop: takes any new tuning, reads the sensors, and stops the motors
   * while the robot is disabled. Call it once per loop, before localization and the commands, so
   * that both judge this loop's readings.
   *
   * @param enabled whether the robot is enabled
   */
  public void periodic(boolean enabled) {
    takeTuningFromThePit();
    readings.read();
    if (!enabled) {
      output.stop();
    }
    alerts.check(modules, readings);
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleVelocities", readings.moduleVelocitiesAsRead());
    Logger.recordOutput("Drive/ModulePoses", modulePoses());
  }

  /** Takes the drivetrain's configuration from the pit, if it was tuned since the last loop. */
  private void takeTuningFromThePit() {
    DrivetrainControl tuned = config.get();
    if (tuned.equals(applied)) {
      return;
    }
    apply(tuned);
  }

  /** Sends gains and limits to the modules, and keeps the limits that driving itself uses. */
  private void apply(DrivetrainControl control) {
    for (Module module : modules) {
      module.configure(control.gains(), control.limits());
    }
    topSpeedMetersPerSecond = control.limits().maxLinearSpeed().in(MetersPerSecond);
    topTurnRateRadiansPerSecond = control.limits().maxAngularSpeed().in(RadiansPerSecond);
    skewLookaheadSeconds = control.gains().skewLookahead().in(Seconds);
    output.limitWheelSpeed(topSpeedMetersPerSecond);
    applied = control;
  }

  /** What the sensors read this loop: the wheels at every high-rate reading, and the IMUs. */
  public DrivetrainReadings readings() {
    return readings.forLocalization();
  }

  /**
   * Each wheel's center in the robot's frame, turned to the angle its module measures: front left,
   * front right, back left, back right ({@link WheelPoses}).
   */
  Pose3d[] modulePoses() {
    return wheelPoses.now();
  }

  // ---- The moves ----

  /**
   * Drives at robot-relative velocities until a condition holds, which it ends on: {@code
   * drive.runUntil(new ChassisVelocities(1.5, 0, 0), bumped)}. ARRIVED, or TIMED_OUT after 3 s
   * unless {@link DriveMove#within} says otherwise, saying why a gate never opened.
   */
  public DriveMove runUntil(ChassisVelocities robotRelative, BooleanSupplier condition) {
    return DriveMove.runUntil(this, robotRelative, condition);
  }

  /**
   * Turns where it stands to a field heading, the nearer way round, profiled under the robot's own
   * turning acceleration: ARRIVED inside 2°, nearly still, for 40 ms.
   */
  public DriveMove turnTo(Rotation2d heading) {
    return DriveMove.turnTo(this, heading);
  }

  /**
   * Goes to a field pose in a straight line, turning on the way, profiled under the robot's own
   * accelerations: ARRIVED within 2 cm and 2°, nearly still, for 40 ms.
   */
  public DriveMove goTo(Pose2d pose) {
    return DriveMove.goTo(this, pose);
  }

  /**
   * As {@link #goTo(Pose2d)}, to a {@link Place} written once for blue: it is found for the robot's
   * alliance when the move starts, never when it's built, so one move is right on both. Started
   * with the alliance unknown, it ends REFUSED, saying so.
   *
   * @throws IllegalArgumentException as the robot starts, if it has not been handed the field and
   *     the alliance ({@link #takeDriver}); once it runs, a move that ends REFUSED as it starts
   */
  public DriveMove goTo(Place place) {
    return DriveMove.goTo(this, place);
  }

  /**
   * Drives a smooth path from where it is through keyframes, stopping at the last, under the
   * robot's own accelerations, turning to each keyframe's heading after it passes the one before:
   * ARRIVED at the last within 2 cm and 2°, nearly still, for 40 ms. {@link DriveMove#passed} opens
   * as it passes a named one.
   *
   * {@snippet class=com.michaelgrundvig.frc.runtime.examples.PickUp region=path}
   *
   * <p>A keyframe at a {@link Place} ({@code Keyframe.at(place)}) is found for the robot's alliance
   * when the path is planned, as the move starts: one path is right on both, and with the alliance
   * unknown it ends REFUSED.
   *
   * @throws IllegalArgumentException with no keyframes, or two of one name, as the robot starts;
   *     once it runs, such a path ends REFUSED as it starts, saying why
   */
  public DriveMove follow(Keyframe... keyframes) {
    return DriveMove.follow(this, List.of(keyframes));
  }

  /** How it's driven now, as tuned. */
  DrivetrainControl control() {
    return config.get();
  }

  /** How fast it can speed up and spin up, by the robot's model and its current limit now. */
  DrivetrainControl.Accelerations accelerations() {
    DrivetrainControl tuned = config.get();
    if (!tuned.equals(accelerationsAsTuned)) {
      accelerations = DrivetrainControl.accelerations(robot, tuned);
      accelerationsAsTuned = tuned;
    }
    return accelerations;
  }

  /**
   * Why a command or a gate on the drivetrain cannot be, as it is built ({@link Refusals#asBuilt}).
   * As the robot starts, the reason is thrown, so the mistake stops the robot starting. Once the
   * robot runs, the reason is handed back, for the command to end refused with or the gate to hold
   * false with.
   *
   * @param why why it cannot be, or empty when nothing is wrong with it
   * @throws IllegalArgumentException with the reason, as the robot starts
   */
  Optional<String> refusedAsBuilt(Optional<String> why) {
    return Refusals.asBuilt(scheduler, why);
  }

  // ---- Driving at a velocity ----

  /**
   * Drives at robot-relative velocities (m/s, m/s, rad/s) for one loop. Call it every loop while
   * driving.
   */
  public void runVelocity(ChassisVelocities robotRelative) {
    output.drive(robotRelative, NO_ACCELERATION);
  }

  /**
   * Drives at robot-relative velocities for one loop, while speeding up or turning up at
   * robot-relative accelerations (m/s², m/s², rad/s²), as a profile asks. Each wheel's drive motor
   * is given the volts its share of that push takes, from the model, so the robot follows the
   * profile rather than trailing it ({@link DriveOutput#drive}). Call it every loop while driving.
   */
  public void runVelocity(ChassisVelocities robotRelative, ChassisAccelerations acceleration) {
    output.drive(robotRelative, acceleration);
  }

  /**
   * Drives field-relative, for one loop, at a velocity and a turning rate that are already shaped:
   * meters a second on the field's axes, and radians a second counterclockwise.
   */
  void driveField(Translation2d velocityMetersPerSecond, double turnRateRadiansPerSecond) {
    ChassisVelocities fieldRelative =
        new ChassisVelocities(
            velocityMetersPerSecond.getX(),
            velocityMetersPerSecond.getY(),
            turnRateRadiansPerSecond);
    // Skew compensation: the wheels take a moment to reach the directions they are told, and the
    // robot has turned further by then. So the field's velocity is turned to the robot's axes by
    // the heading the robot is about to have, not the one it has now. Without this a robot that
    // spins while it drives veers off its line.
    Rotation2d turnedDuringLookahead = new Rotation2d(fieldRelative.omega * skewLookaheadSeconds);
    Rotation2d headingSoon = getPose().getRotation().plus(turnedDuringLookahead);
    runVelocity(fieldRelative.toRobotRelative(headingSoon));
  }

  // ---- Driving with the sticks ----

  /**
   * Hands the drivetrain the driver's sticks, the field and the alliance, once, as the robot builds
   * it. Every command that drives with the sticks reads them ({@link #driven}, {@link #facing}), so
   * a team's {@code Controls} builds assisted driving without holding the sticks itself; a move to
   * a {@link Place} finds it on the field for the alliance, when the move starts.
   *
   * @param translation the translation stick, turned to the field's axes: x along the field, y
   *     across it, both -1 to 1, and forward for the driver's alliance already the way they face (a
   *     red driver's forward is the field's -x)
   * @param turn the turn stick, -1 to 1, counterclockwise positive
   * @param field the field the robot plays on, read each loop (a venue can replace it)
   * @param alliance the robot's alliance, read each loop: empty until the Driver Station or the
   *     field says
   */
  public void takeDriver(
      Supplier<Translation2d> translation,
      DoubleSupplier turn,
      Supplier<FieldModel> field,
      Supplier<Optional<Alliance>> alliance) {
    driver = Optional.of(new Driver(translation, turn, field, alliance));
  }

  /** The driver's sticks, the field and the alliance, or empty before they are handed over. */
  Optional<Driver> driver() {
    return driver;
  }

  /**
   * Drives from the driver's sticks until interrupted: field-relative, the translation stick
   * squared past a deadband for fine control near center, the turn stick the same ({@link Sticks}).
   * The drive's default command.
   *
   * @throws IllegalArgumentException as the robot starts, if it has not been handed the driver's
   *     sticks ({@link #takeDriver}); once it runs, a command that ends as it starts
   */
  public Command driven() {
    String name = "Drive with the sticks";
    if (driver.isEmpty()) {
      Optional<String> refused = refusedAsBuilt(Optional.of(NO_DRIVER));
      String outcome = name + ": REFUSED: " + refused.orElse(NO_DRIVER);
      return run(coroutine -> Logger.recordOutput("Drive/Outcome", outcome)).named(name);
    }
    Driver sticks = driver.get();
    return run(coroutine -> {
          while (true) {
            driveField(
                velocityAskedBy(sticks.translation().get()),
                turnRateAskedBy(sticks.turn().getAsDouble()));
            coroutine.yield();
          }
        })
        .named(name);
  }

  /**
   * The field-relative velocity a translation stick asks, in meters a second: shaped ({@link
   * Sticks#velocityAsked}), and scaled to the top speed as tuned.
   */
  Translation2d velocityAskedBy(Translation2d stick) {
    return Sticks.velocityAsked(stick, topSpeedMetersPerSecond);
  }

  /**
   * The turning rate a turn stick asks, in radians a second, counterclockwise: shaped ({@link
   * Sticks#turnRateAsked}), and scaled to the top turning rate as tuned.
   */
  double turnRateAskedBy(double stick) {
    return Sticks.turnRateAsked(stick, topTurnRateRadiansPerSecond);
  }

  /**
   * Drives from the driver's translation stick as {@link #driven} does, the drivetrain holding the
   * robot's heading on a target: a field heading, an angle from the drivers' forward, or a place
   * ({@link Heading}). The driver takes the heading back while the turn stick is past its deadband.
   * Runs until interrupted; {@link DriveAssist#aimed} says when it's there.
   *
   * <p>With the target unknown (the alliance isn't, or the robot is on the place), the robot
   * doesn't turn, and the translation still drives.
   *
   * {@snippet class=com.michaelgrundvig.frc.runtime.examples.Aiming region=facing}
   */
  public DriveAssist facing(Heading heading) {
    return DriveAssist.facing(this, heading);
  }

  /**
   * Turns every wheel into an X, each along its line to the robot's center, with the drive motors
   * held at zero speed, until interrupted: the robot resists being pushed or spun. Each wheel turns
   * whichever end it points the nearer way.
   */
  public Command lockWheels() {
    return run(coroutine -> {
          while (true) {
            output.lockWheels();
            coroutine.yield();
          }
        })
        .named("Lock wheels");
  }

  // ---- Places, for a move that goes to one ----

  /**
   * Where a place is for the robot's alliance, as of now: the move that asks is starting.
   *
   * @throws IllegalArgumentException saying why it can't be found: the alliance isn't known, or the
   *     drive was never handed the field
   */
  Pose2d found(Place place) {
    if (driver.isEmpty()) {
      throw new IllegalArgumentException(NO_FIELD);
    }
    return driver.get().found(place);
  }

  /**
   * A path's keyframes, each at a pose: those at a place found for the robot's alliance now, the
   * rest as they are ({@link Driver#found(List)}).
   *
   * @throws IllegalArgumentException as {@link #found(Place)} does
   */
  List<Keyframe> found(List<Keyframe> keyframes) {
    if (driver.isPresent()) {
      return driver.get().found(keyframes);
    }
    // With no field to find a place on, only a path with no keyframe at a place can be followed.
    for (Keyframe keyframe : keyframes) {
      if (keyframe.place() != null) {
        throw new IllegalArgumentException(NO_FIELD);
      }
    }
    return keyframes;
  }

  // ---- What it measures ----

  /** Where the robot is, as localization has it. */
  public Pose2d getPose() {
    return pose.get();
  }

  /**
   * Measured robot-relative velocities as of this loop's read, from the modules that are connected:
   * a disconnected one reads the speed it last sent, however the robot moves now. With fewer than
   * two connected, nothing better than all four ({@link DriveReadings#robotRelativeVelocities}).
   */
  public ChassisVelocities getRobotRelativeVelocities() {
    return readings.robotRelativeVelocities();
  }

  /** The modules' positions, as WPILib's odometry takes them. */
  public SwerveModulePosition[] modulePositions() {
    return readings.modulePositions();
  }

  /** Each module's measured wheel speed and angle, in the order of {@link #moduleLocations}. */
  public SwerveModuleVelocity[] moduleVelocities() {
    return readings.moduleVelocities();
  }

  /** The gyro's yaw, as of this loop: for calibrating against, and WPILib's odometry. */
  public Rotation2d gyroYaw() {
    return readings.gyroYaw();
  }

  /** How far each wheel has turned, in radians: front left, front right, back left, back right. */
  public double[] wheelRotationsRadians() {
    return readings.wheelRotationsRadians();
  }

  /** Where each wheel touches the floor, from the robot's center. */
  public Translation2d[] moduleLocations() {
    return kinematics.getModules();
  }

  // ---- For the pit ----

  /**
   * Drives straight ahead, robot-relative, at a steady speed for a while, then stops: a pit check
   * that every wheel turns the right way.
   */
  public Command driveForward(double metersPerSecond, double seconds) {
    int loops = RobotLoop.loops(seconds);
    return run(coroutine -> {
          for (int i = 0; i < loops; i++) {
            runVelocity(new ChassisVelocities(metersPerSecond, 0, 0));
            coroutine.yield();
          }
          runVelocity(new ChassisVelocities(0, 0, 0));
        })
        .named("Drive forward");
  }

  /**
   * Points every wheel at one angle, relative to the robot, with the drive wheels held still: the
   * steering's test motion. Call it every loop the test runs.
   *
   * @param radians the angle, counterclockwise from the robot's forward
   */
  public void holdSteerAngle(double radians) {
    steeringRig.hold(radians);
  }

  /**
   * The steering as a Pitwall rig, so the page can tune it by experiment: its gains ({@code
   * steerKp} and {@code steerKd}, tuned where they always are), every wheel pointed at one angle as
   * the setpoint, and for each wheel its angle at every high-rate reading (what's scored), and once
   * a loop its steering motor's voltage and the most current it drew since the last. Everything it
   * records is a logged input, so a replay records the same ({@link SteeringRig}).
   *
   * @param tuning the drivetrain's configuration as the pit tunes it
   */
  public Rig steeringRig(LiveConfig<DrivetrainControl> tuning) {
    return steeringRig.rig(tuning, this);
  }

  // ---- Its alerts ----

  boolean isModuleUnconfiguredAlertActive(int module) {
    return alerts.isModuleUnconfiguredActive(module);
  }

  boolean isOdometryStoppedAlertActive() {
    return alerts.isOdometryStoppedActive();
  }

  /** Releases its alerts, so a test can build another drivetrain. */
  @Override
  public void close() {
    alerts.close();
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.OdometrySample;
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
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
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
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.util.Alert;

/**
 * The swerve drivetrain: four {@link Module}s and a gyro. It drives, and each loop it reports what
 * its sensors read ({@link #readings()}) for localization, which owns where the robot is.
 *
 * <p>Call {@link #periodic(boolean)} once per loop, before localization and the command scheduler.
 * Driving commands call {@link #runVelocity(ChassisVelocities)} every loop they are active.
 *
 * <p>The drivetrain's configuration is read every loop, so values tuned while the robot runs take
 * effect at once: new gains and current limits go to the motor controllers the loop they change.
 *
 * <p>Hardware comes in through the IO layer ({@link DriveHardware}): {@link PhoenixDrivetrain} on a
 * CTRE robot, the simulator's in the simulator ({@code com.michaelgrundvig.frc.runtime.sim}), the
 * log in a replay. A stand-in until AdvantageKit's 2027 swerve template ships: see
 * docs/stand-ins.md.
 */
@CheckReturnValue
public class Drive implements Mechanism, AutoCloseable {
  static final double STICK_DEADBAND = 0.1;

  /**
   * Wheels that report connected but send no readings this long have lost what reads them: on a
   * robot, the thread that records their high-rate readings.
   */
  private static final double STOPPED_SECONDS = 0.1;

  /** The wheels by name, as a steering test records them. */
  private static final List<String> WHEELS =
      List.of("Front left", "Front right", "Back left", "Back right");

  /** What a refusal says when the drive hasn't been handed the driver's sticks. */
  static final String NO_DRIVER =
      "the drivetrain has no driver's sticks: the robot hands them over as it builds the drive,"
          + " drive.takeDriver(...)";

  /** What a move to a place says when the drive hasn't been handed the field and the alliance. */
  static final String NO_FIELD =
      "the drivetrain has no field or alliance to find a place on: the robot hands them over as it"
          + " builds the drive, drive.takeDriver(...)";

  /**
   * Why a move to a place won't start before the Driver Station or the field names the alliance.
   */
  static final String ALLIANCE_UNKNOWN = "alliance unknown";

  /** No acceleration: plain velocity driving. */
  private static final ChassisAccelerations NO_ACCELERATION = new ChassisAccelerations();

  /**
   * The robot's mass and yaw inertia, and its wheels' squared distances from the center, summed.
   */
  private final double massKilograms;

  private final double yawInertia;
  private final double spread;

  private final RobotDefinition robot;
  private final Supplier<DrivetrainControl> config;
  private final Scheduler scheduler;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroIO backupImuIO;
  private final GyroIOInputsAutoLogged backupInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final SwerveDriveKinematics kinematics;

  /** Where each wheel touches the floor, from the robot's center. */
  private final Translation2d[] moduleTranslations;

  /** Each wheel's radius, as built: its center is this far above the floor. */
  private final double[] wheelRadiusMeters;

  /** The wheels' poses as last logged, reused loop after loop. */
  private final Pose3d[] modulePoses = new Pose3d[4];

  /** The steer angles the logged poses were made for: NaN before the first. */
  private final double[] posedAngles = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

  private final Supplier<Pose2d> pose;
  private final Lock odometryLock;

  /**
   * The driver's sticks, and the field and alliance the robot plays on, handed over by the robot
   * ({@link #takeDriver}): the sticks drive it, and the field and alliance find a place for it.
   *
   * @param translation the driver's translation stick, turned to the field's axes
   * @param turn the driver's turn stick, counterclockwise positive
   * @param field the field the robot plays on
   * @param alliance the robot's alliance, once known
   */
  record Driver(
      Supplier<Translation2d> translation,
      DoubleSupplier turn,
      Supplier<FieldModel> field,
      Supplier<Optional<Alliance>> alliance) {}

  /** The driver's sticks and the field, once the robot has handed them over. */
  private @Nullable Driver driver;

  /** The configuration the motor controllers have, and the limits taken from it. */
  private DrivetrainControl applied;

  private double maxLinearSpeedMps;
  private double maxAngularSpeedRadPerSec;
  private double skewLookaheadSec;

  /** What the sensors read this loop, for localization. */
  private DrivetrainReadings readings;

  /** The kinematics of each set of connected modules short of all four, made as each comes. */
  private final Map<Integer, SwerveDriveKinematics> reportingKinematics = new HashMap<>();

  private final Alert[] moduleDisconnectedAlerts = new Alert[4];
  private final Alert[] moduleUnconfiguredAlerts = new Alert[4];
  private final Alert odometryStopped;

  /** Loops in a row that the wheels reported connected but sent no readings. */
  private int loopsWithoutReadings;

  /** The steer angle last held for a steering test, in radians. */
  private double heldSteerRadians;

  /**
   * Why the template can't drive a drivetrain wired this way, or null when it can: its modules are
   * TalonFXs driving and steering, or SPARKs doing both ({@link PhoenixDrivetrain}), so far. What
   * the wiring itself can't do, a controller that doesn't drive its motor, its definition refuses.
   */
  public static @Nullable String whyNotDriven(SwerveWiring wiring) {
    MechanismWiring.Controller drive = wiring.driveController();
    MechanismWiring.Controller steer = wiring.steerController();
    boolean talons =
        drive == MechanismWiring.Controller.TALON_FX
            && steer == MechanismWiring.Controller.TALON_FX;
    boolean sparks = spark(drive) && spark(steer);
    return talons || sparks
        ? null
        : "the drivetrain drives on "
            + drive.shown()
            + " and steers on "
            + steer.shown()
            + ", and the template's modules are TalonFXs or SPARKs, doing both, so far: put both on"
            + " TalonFXs, or both on SPARKs";
  }

  private static boolean spark(MechanismWiring.Controller controller) {
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
    this.gyroIO = hardware.gyro();
    this.backupImuIO = hardware.backupImu();
    this.pose = pose;
    this.odometryLock = hardware.odometryLock();
    DrivetrainControl initial = config.get();
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(hardware.modules().get(i), i, robot.drivetrain().module(i));
    }
    applied = initial;
    apply(initial);
    moduleTranslations = robot.moduleTranslations();
    massKilograms = robot.body().mass().in(Kilograms);
    yawInertia = robot.body().yawMomentOfInertia().in(KilogramSquareMeters);
    double sum = 0;
    for (Translation2d at : moduleTranslations) {
      sum += at.getSquaredNorm();
    }
    spread = sum;
    wheelRadiusMeters = new double[4];
    for (int i = 0; i < 4; i++) {
      wheelRadiusMeters[i] = robot.drivetrain().module(i).wheelRadius().in(Meters);
    }
    kinematics = new SwerveDriveKinematics(moduleTranslations);
    readings =
        DrivetrainReadings.builder()
            .wheels(List.of())
            .gyro(ImuReadings.NONE)
            .backupImu(ImuReadings.NONE)
            .velocities(new ChassisVelocities())
            .build();

    odometryStopped =
        new Alert(
            name,
            "odometryStopped",
            "Odometry stopped: the wheels report connected, but no readings have arrived for "
                + STOPPED_SECONDS
                + " s. Restart the robot code.",
            Alert.Level.HIGH);
    for (int i = 0; i < 4; i++) {
      String module = Module.NAMES.get(i);
      moduleDisconnectedAlerts[i] =
          new Alert(
              name,
              "module" + module + "Disconnected",
              module
                  + " swerve module disconnected: driving on the other three, odometry from them."
                  + " Check its motor controllers' CAN wiring and power",
              Alert.Level.HIGH);
      moduleUnconfiguredAlerts[i] =
          new Alert(
              name,
              "module" + module + "Unconfigured",
              module + " swerve module not configured",
              Alert.Level.HIGH);
    }
    Trajectories.warmUp(RobotLoop.PERIOD_SECONDS);
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  /**
   * Reads the sensors, and stops the outputs while disabled. Once per loop, before localization and
   * the commands.
   *
   * @param enabled whether the robot is enabled
   */
  public void periodic(boolean enabled) {
    DrivetrainControl now = config.get();
    if (!now.equals(applied)) {
      apply(now);
    }
    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      backupImuIO.updateInputs(backupInputs);
      for (Module module : modules) {
        module.readInputs();
      }
    } finally {
      odometryLock.unlock();
    }
    Logger.processInputs("Drive/Gyro", gyroInputs);
    Logger.processInputs("Drive/BackupGyro", backupInputs);
    for (Module module : modules) {
      module.logInputs();
    }
    if (!enabled) {
      for (Module module : modules) {
        module.stop();
      }
      Logger.recordOutput("Drive/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    List<OdometrySample> wheels = wheelReadings();
    boolean connected = true;
    for (Module module : modules) {
      connected &= module.isConnected();
    }
    loopsWithoutReadings = connected && wheels.isEmpty() ? loopsWithoutReadings + 1 : 0;
    odometryStopped.set(loopsWithoutReadings > RobotLoop.loops(STOPPED_SECONDS));
    readings =
        DrivetrainReadings.builder()
            .wheels(wheels)
            .gyro(imuReadings(gyroInputs))
            .backupImu(imuReadings(backupInputs))
            .velocities(getRobotRelativeVelocities())
            .build();

    for (int i = 0; i < 4; i++) {
      moduleDisconnectedAlerts[i].set(!modules[i].isConnected());
      String unconfigured = modules[i].unconfigured();
      if (!unconfigured.isEmpty() && !moduleUnconfiguredAlerts[i].get()) {
        moduleUnconfiguredAlerts[i].setText(
            Module.NAMES.get(i)
                + " swerve module: "
                + unconfigured
                + " didn't take its configuration and runs on defaults (no current limits):"
                + " restart the robot code once it's connected");
      }
      moduleUnconfiguredAlerts[i].set(!unconfigured.isEmpty());
    }
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleVelocities", moduleVelocities());
    Logger.recordOutput("Drive/ModulePoses", modulePoses());
  }

  /**
   * Each wheel's center in the robot's frame, turned to the angle its module measures: front left,
   * front right, back left, back right. AdvantageScope turns the schematic robot's wheels by these
   * (its components 0 to 3, generated into ascope_assets/ from the robot's configuration).
   */
  Pose3d[] modulePoses() {
    for (int i = 0; i < 4; i++) {
      double angle = modules[i].steerAngle().getRadians();
      // A wheel that hasn't turned keeps its pose: nothing new to make, most loops.
      if (angle != posedAngles[i]) {
        Translation2d at = moduleTranslations[i];
        modulePoses[i] =
            new Pose3d(at.getX(), at.getY(), wheelRadiusMeters[i], new Rotation3d(0, 0, angle));
        posedAngles[i] = angle;
      }
    }
    return modulePoses;
  }

  /** What the sensors read this loop: the wheels at every high-rate reading, and the IMUs. */
  public DrivetrainReadings readings() {
    return readings;
  }

  /**
   * Every high-rate reading of the wheels since the last loop, oldest first. A module that's
   * disconnected reads only what it last sent, as if its wheel stood still: it reads no number
   * instead, as does one whose reading isn't a number, and odometry fits the motion from the rest
   * (driving on three). A reading no wheel reports in, or taken before the clock started, is left
   * out: the next good one covers the motion.
   */
  private List<OdometrySample> wheelReadings() {
    int count = Integer.MAX_VALUE;
    for (Module module : modules) {
      count = Math.min(count, module.odometryReadings());
    }
    List<OdometrySample> wheels = new ArrayList<>(count);
    for (int k = 0; k < count; k++) {
      double[] distances = new double[4];
      double[] angles = new double[4];
      boolean any = false;
      for (int i = 0; i < 4; i++) {
        boolean connected = modules[i].isConnected();
        distances[i] = connected ? modules[i].odometryDistanceMeters(k) : Double.NaN;
        angles[i] = connected ? modules[i].odometryAngleRadians(k) : Double.NaN;
        any |= Double.isFinite(distances[i]) && Double.isFinite(angles[i]);
      }
      long stamp = modules[0].odometryTimestampNanos(k);
      if (any && stamp >= 0) {
        wheels.add(new OdometrySample(stamp, distances, angles));
      }
    }
    return wheels;
  }

  /** One IMU's readings this loop, as localization takes them. */
  private static ImuReadings imuReadings(GyroIOInputsAutoLogged imu) {
    int count = Math.min(imu.odometryTimestampsNanos.length, imu.odometryYawsRad.length);
    List<Long> times = new ArrayList<>(count);
    List<Double> yaws = new ArrayList<>(count);
    for (int i = 0; i < count; i++) {
      times.add(imu.odometryTimestampsNanos[i]);
      yaws.add(imu.odometryYawsRad[i]);
    }
    return ImuReadings.builder()
        .reporting(imu.connected)
        .yawRadians(imu.yaw.getRadians())
        .yawRateRadiansPerSecond(imu.yawVelocityRadPerSec)
        .pitchRadians(imu.pitchRad)
        .rollRadians(imu.rollRad)
        .forwardAccelerationMetersPerSecSq(imu.accelerationXMetersPerSecSq)
        .leftAccelerationMetersPerSecSq(imu.accelerationYMetersPerSecSq)
        .yawTimesNanos(times)
        .yawsRadians(yaws)
        .build();
  }

  /** Sends new gains and limits to the modules, and keeps the limits driving uses. */
  private void apply(DrivetrainControl drivetrain) {
    for (Module module : modules) {
      module.configure(drivetrain.gains(), drivetrain.limits());
    }
    maxLinearSpeedMps = drivetrain.limits().maxLinearSpeed().in(MetersPerSecond);
    maxAngularSpeedRadPerSec = drivetrain.limits().maxAngularSpeed().in(RadiansPerSecond);
    skewLookaheadSec = drivetrain.gains().skewLookahead().in(Seconds);
    applied = drivetrain;
  }

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
    return DrivetrainControl.accelerations(robot, config.get());
  }

  /** Drive at robot-relative velocities (m/s, m/s, rad/s). Call every loop while driving. */
  public void runVelocity(ChassisVelocities robotRelative) {
    runVelocity(robotRelative, NO_ACCELERATION);
  }

  /**
   * Drive at robot-relative velocities while speeding up or turning up at robot-relative
   * accelerations (m/s², m/s², rad/s²), as a profile asks: each wheel pushes its share of the
   * robot's mass times the acceleration, and of its yaw inertia times the turning one, spread by
   * its distance from the center, and its motors are given the volts that takes, from the model, so
   * the robot follows the profile rather than trailing it. Call every loop while driving.
   */
  public void runVelocity(ChassisVelocities robotRelative, ChassisAccelerations acceleration) {
    // Correct for the robot rotating during the loop's step, which otherwise makes it drift
    // sideways while spinning and driving at the same time.
    ChassisVelocities discrete = robotRelative.discretize(RobotLoop.PERIOD_SECONDS);
    SwerveModuleVelocity[] setpoints =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toSwerveModuleVelocities(discrete), maxLinearSpeedMps);
    SwerveModuleVelocity[] sent = new SwerveModuleVelocity[4];
    int count = modules.length;
    for (int i = 0; i < count; i++) {
      Translation2d at = moduleTranslations[i];
      Translation2d force =
          new Translation2d(
              massKilograms * acceleration.ax / count
                  - yawInertia * acceleration.alpha * at.getY() / spread,
              massKilograms * acceleration.ay / count
                  + yawInertia * acceleration.alpha * at.getX() / spread);
      sent[i] = modules[i].runSetpoint(setpoints[i], force);
    }
    Logger.recordOutput("Drive/Setpoints", setpoints);
    Logger.recordOutput("Drive/SetpointsOptimized", sent);
  }

  /** Where the robot is, as localization has it. */
  public Pose2d getPose() {
    return pose.get();
  }

  /**
   * Measured robot-relative velocities, from the modules that are connected: a disconnected one
   * reads the speed it last sent, however the robot moves now. With fewer than two connected,
   * nothing better than all four.
   */
  public ChassisVelocities getRobotRelativeVelocities() {
    SwerveModuleVelocity[] measured = moduleVelocities();
    int connected = 0;
    for (int i = 0; i < 4; i++) {
      connected |= modules[i].isConnected() ? 1 << i : 0;
    }
    if (connected == 0b1111 || Integer.bitCount(connected) < 2) {
      return kinematics.toChassisVelocities(measured);
    }
    SwerveModuleVelocity[] reporting = new SwerveModuleVelocity[Integer.bitCount(connected)];
    int k = 0;
    for (int i = 0; i < 4; i++) {
      if ((connected & (1 << i)) != 0) {
        reporting[k++] = measured[i];
      }
    }
    return reportingKinematics
        .computeIfAbsent(connected, this::kinematicsOf)
        .toChassisVelocities(reporting);
  }

  /** WPILib's kinematics for the modules a mask's bits name. */
  private SwerveDriveKinematics kinematicsOf(int mask) {
    List<Translation2d> at = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      if ((mask & (1 << i)) != 0) {
        at.add(moduleTranslations[i]);
      }
    }
    return new SwerveDriveKinematics(at.toArray(Translation2d[]::new));
  }

  /** The modules' positions, as WPILib's odometry takes them. */
  public SwerveModulePosition[] modulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Each module's measured wheel speed and angle, in the order of {@code moduleTranslations}. */
  public SwerveModuleVelocity[] moduleVelocities() {
    SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      velocities[i] = modules[i].getVelocity();
    }
    return velocities;
  }

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
    driver = new Driver(translation, turn, field, alliance);
  }

  /**
   * Drives from the driver's sticks until interrupted: field-relative, the translation stick
   * squared past a deadband for fine control near center, the turn stick the same. The drive's
   * default command.
   *
   * @throws IllegalArgumentException as the robot starts, if it has not been handed the driver's
   *     sticks ({@link #takeDriver}); once it runs, a command that ends as it starts
   */
  public Command driven() {
    Driver sticks = driver;
    String refused = Refusals.asBuilt(scheduler, sticks == null ? NO_DRIVER : null);
    return run(coroutine -> {
          if (sticks == null) {
            Logger.recordOutput("Drive/Outcome", "Drive with the sticks: REFUSED: " + refused);
            return;
          }
          while (true) {
            driveField(
                translationFrom(sticks.translation().get()), turnFrom(sticks.turn().getAsDouble()));
            coroutine.yield();
          }
        })
        .named("Drive with the sticks");
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
   * Turns every wheel into an X, each perpendicular to its line to the robot's center, with the
   * drive motors held at zero speed, until interrupted: the robot resists being pushed. Each wheel
   * turns whichever end it points the nearer way.
   */
  public Command lockWheels() {
    return run(coroutine -> {
          while (true) {
            for (int i = 0; i < 4; i++) {
              Translation2d at = moduleTranslations[i];
              double across = Math.atan2(at.getY(), at.getX()) + Math.PI / 2;
              double turn = MathUtil.angleModulus(across - modules[i].steerAngle().getRadians());
              modules[i].holdSteer(
                  new Rotation2d(Math.abs(turn) > Math.PI / 2 ? across + Math.PI : across));
            }
            coroutine.yield();
          }
        })
        .named("Lock wheels");
  }

  /** The driver, once handed over, or null before. */
  @Nullable Driver driver() {
    return driver;
  }

  /**
   * A place as it is for the robot's alliance, read now: the move that asks is starting.
   *
   * @throws IllegalArgumentException saying why when it can't be found: the alliance isn't known,
   *     or the drive was never handed the field
   */
  Pose2d found(Place place) {
    Driver handed = driver;
    if (handed == null) {
      throw new IllegalArgumentException(NO_FIELD);
    }
    Alliance ours =
        handed.alliance().get().orElseThrow(() -> new IllegalArgumentException(ALLIANCE_UNKNOWN));
    return place.on(handed.field().get(), ours);
  }

  /**
   * Each keyframe at a pose: those at a place found for the robot's alliance now (named as they
   * were, or for the place), the rest as they are.
   *
   * @throws IllegalArgumentException as {@link #found(Place)} does
   */
  List<Keyframe> found(List<Keyframe> keyframes) {
    List<Keyframe> fixed = new ArrayList<>();
    for (Keyframe keyframe : keyframes) {
      Place place = keyframe.place();
      if (place == null) {
        fixed.add(keyframe);
      } else {
        String name = keyframe.name() != null ? keyframe.name() : place.name();
        fixed.add(Keyframe.at(found(place)).named(name));
      }
    }
    return fixed;
  }

  /**
   * A translation stick as the field-relative velocity it asks, in meters a second: past a
   * deadband, squared for fine control near center, and scaled to the top speed.
   */
  Translation2d translationFrom(Translation2d stick) {
    double linear = MathUtil.applyDeadband(stick.getNorm(), STICK_DEADBAND);
    linear = Math.min(1.0, linear * linear);
    return new Translation2d(
        linear * maxLinearSpeedMps, new Rotation2d(Math.atan2(stick.getY(), stick.getX())));
  }

  /**
   * A turn stick as the turning rate it asks, in radians a second, counterclockwise: past a
   * deadband, squared with its sign, and scaled to the top turning rate.
   */
  double turnFrom(double stick) {
    double turn = MathUtil.applyDeadband(stick, STICK_DEADBAND);
    return Math.copySign(turn * turn, turn) * maxAngularSpeedRadPerSec;
  }

  /**
   * Drives field-relative, for one loop, at a velocity and a turning rate already shaped: m/s on
   * the field's axes, and rad/s counterclockwise.
   */
  void driveField(Translation2d velocity, double omega) {
    ChassisVelocities fieldRelative =
        new ChassisVelocities(velocity.getX(), velocity.getY(), omega);
    // Skew compensation: by the time the wheels reach their new angles the robot has already
    // rotated, so convert using the heading it is about to have.
    Rotation2d futureHeading =
        getPose().getRotation().plus(new Rotation2d(fieldRelative.omega * skewLookaheadSec));
    runVelocity(fieldRelative.toRobotRelative(futureHeading));
  }

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
    heldSteerRadians = radians;
    Rotation2d angle = new Rotation2d(radians);
    for (Module module : modules) {
      module.holdSteer(angle);
    }
  }

  /**
   * The steering as a Pitwall rig, so the page can tune it by experiment: its gains ({@code
   * steerKp} and {@code steerKd}, tuned where they always are), every wheel pointed at one angle as
   * the setpoint, and for each wheel its angle at every high-rate reading (what's scored), and once
   * a loop its steering motor's voltage and the most current it drew since the last. Everything it
   * records is a logged input, so a replay records the same.
   *
   * @param tuning the drivetrain's configuration as the pit tunes it
   */
  public Rig steeringRig(LiveConfig<DrivetrainControl> tuning) {
    Rig.Builder rig =
        Rig.builder()
            .name("Steering")
            .gains(tuning, "gains.steerKp", "gains.steerKd")
            .setpoint(Radians, this::holdSteerAngle)
            .limits(Degrees.of(-180), Degrees.of(180));
    for (int i = 0; i < 4; i++) {
      Module module = modules[i];
      rig.measurement(
          WHEELS.get(i),
          Degrees,
          (now, series) -> {
            for (int k = 0; k < module.odometryReadings(); k++) {
              // The angle nearest the one held, so a wheel near ±180° doesn't jump a turn.
              double angle =
                  heldSteerRadians
                      + MathUtil.angleModulus(module.odometryAngleRadians(k) - heldSteerRadians);
              series.add(module.odometryTimestampNanos(k), Math.toDegrees(angle));
            }
          });
    }
    for (int i = 0; i < 4; i++) {
      Module module = modules[i];
      rig.signal(WHEELS.get(i) + " voltage", Volts, module::steerAppliedVolts);
      rig.signal(WHEELS.get(i) + " current", Amps, module::steerCurrentAmps);
    }
    return rig.requires(this).build();
  }

  /** The gyro's yaw, as of this loop: for calibrating against, and WPILib's odometry. */
  public Rotation2d gyroYaw() {
    return gyroInputs.yaw;
  }

  /** How far each wheel has turned, in radians: front left, front right, back left, back right. */
  public double[] wheelRotationsRadians() {
    double[] rotations = new double[4];
    for (int i = 0; i < 4; i++) {
      rotations[i] = modules[i].wheelRotationRadians();
    }
    return rotations;
  }

  /** Where each wheel touches the floor, from the robot's center. */
  public Translation2d[] moduleLocations() {
    return kinematics.getModules();
  }

  boolean isModuleUnconfiguredAlertActive(int module) {
    return moduleUnconfiguredAlerts[module].get();
  }

  boolean isOdometryStoppedAlertActive() {
    return odometryStopped.get();
  }

  @Override
  public void close() {
    for (Alert alert : moduleDisconnectedAlerts) {
      alert.close();
    }
    for (Alert alert : moduleUnconfiguredAlerts) {
      alert.close();
    }
    odometryStopped.close();
  }
}

package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.simulation.AccelerometerModel;
import com.michaelgrundvig.frc.simulation.Actuators;
import com.michaelgrundvig.frc.simulation.ChassisTruth;
import com.michaelgrundvig.frc.simulation.Disturbance;
import com.michaelgrundvig.frc.simulation.GyroModel;
import com.michaelgrundvig.frc.simulation.KinematicPhysics;
import com.michaelgrundvig.frc.simulation.Physics;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Imus;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The simulated drivetrain: the {@link Physics} that knows where the robot really is, the modules'
 * motor controllers that drive it, and the sensors that report it with {@link SensorErrors}, as
 * real sensors would, so the pose the robot code works out can drift from where the robot really
 * is, and localization is tested against that gap instead of a perfect world. This class steps them
 * together and takes the sensors' readings; the physics itself is today's {@link KinematicPhysics}.
 *
 * <p>Call {@link #step} once per loop, before the robot code reads its sensors. It runs the physics
 * at 250 Hz and takes the modules' and gyro's readings at each of those steps, as a robot's
 * high-rate odometry does. The robot doesn't slip on its own past what {@link KinematicPhysics}
 * allows (see docs/stand-ins.md), but faults can be made: {@link #spin} slips a wheel (and a module
 * can keep spinning faster than the floor), {@link #shove} moves the robot the way a push does, at
 * once, and {@link #push} as one does over time, {@link #hit} the way a collision does, and {@link
 * #tilt} with {@link #grip} lifts it onto a bump.
 */
public final class DriveSimulation {
  /** Physics steps, and odometry readings, per robot loop: 250 Hz at a 50 Hz loop. */
  static final int STEPS_PER_LOOP = 5;

  /** A swerve drivetrain's four modules: front left, front right, back left, back right. */
  private static final int MODULE_COUNT = 4;

  private static final double NANOS_PER_SECOND = 1e9;

  /** A hard hit's jolt, as an IMU feels it: 4 g. */
  static final double HIT_JOLT_METERS_PER_SECOND_SQUARED = 4 * ImuReadings.STANDARD_GRAVITY;

  private final Physics physics;
  private final ModuleIOSim[] modules = new ModuleIOSim[MODULE_COUNT];
  private final GyroIOSim gyro;
  private final GyroIOSim backupGyro;

  /** The modules, as the physics' actuators: each drives its own wheel. */
  private final Actuators actuators;

  // What's been done to the physics, as the faults ask: each wheel's true radius over the
  // configured one (now, and as the robot was built), the ground, a push, a tilt, and the grip.
  private final double[] treadScale = new double[MODULE_COUNT];
  private final double[] builtTreadScale = new double[MODULE_COUNT];
  private Terrain terrain = Terrain.FLAT;
  private boolean pushed;
  private boolean tiltedByHand;
  private double grip = 1;

  /**
   * On flat ground until given its terrain ({@link #setTerrain}), its IMUs at the robot's center.
   *
   * @param body the robot's frame, which places the wheels, and its mass, which they carry
   * @param drivetrain the drivetrain as defined: the code's belief about its modules
   * @param errors how the real wheels and sensors differ from the configuration
   */
  public DriveSimulation(Body body, DrivetrainGeometry drivetrain, SensorErrors errors) {
    this(body, drivetrain, errors, Imus.UNCHECKED);
  }

  /**
   * On flat ground until given its terrain ({@link #setTerrain}).
   *
   * @param body the robot's frame, which places the wheels, and its mass, which they carry
   * @param drivetrain the drivetrain as defined: the code's belief about its modules
   * @param errors how the real wheels and sensors differ from the configuration
   * @param imus where the robot's definition mounts its IMUs: they really sit off that, and are
   *     turned off it, by their accelerometers' errors. A robot without a backup IMU still has one
   *     simulated, at the gyro's mount, which the world never hands its code
   */
  public DriveSimulation(Body body, DrivetrainGeometry drivetrain, SensorErrors errors, Imus imus) {
    physics = new KinematicPhysics(body, drivetrain, errors.wheelRadiusScale());
    for (int i = 0; i < MODULE_COUNT; i++) {
      // Each module's CANcoder sends on its own clock: here a quarter period after the last, to the
      // steering loop's millisecond (0, 2, 5 and 7 ms into its cycle, at 100 Hz).
      double encoderPhase = i / (double) MODULE_COUNT;
      modules[i] =
          new ModuleIOSim(
              drivetrain.module(i),
              physics.wheel(i),
              Math.toRadians(errors.steerOffsetDegrees().get(i)),
              encoderPhase);
      treadScale[i] = errors.wheelRadiusScale().get(i);
      builtTreadScale[i] = treadScale[i];
    }
    actuators =
        (seconds, world, drives) -> {
          for (int i = 0; i < modules.length; i++) {
            modules[i].actuate(seconds, drives[i]);
          }
        };
    // Each IMU's noise comes from its own stream of the world's seed (Seeds). A robot without a
    // backup IMU still has one simulated, at the gyro's mount.
    long seed = errors.seed();
    gyro =
        imu(
            errors.gyro(),
            errors.accelerometer(),
            imus.gyro(),
            Seeds.gyro(seed),
            Seeds.gyroAccelerometer(seed));
    backupGyro =
        imu(
            errors.backupGyro(),
            errors.backupAccelerometer(),
            imus.backup().orElse(imus.gyro()),
            Seeds.backupGyro(seed),
            Seeds.backupAccelerometer(seed));
  }

  /** One simulated IMU on this robot's chassis: a gyro with its own accelerometer. */
  private GyroIOSim imu(
      SensorErrors.GyroErrors gyroErrors,
      SensorErrors.AccelErrors accelerometerErrors,
      ImuMount mount,
      long gyroSeed,
      long accelerometerSeed) {
    ChassisTruth chassis = physics.chassis();
    AccelerometerModel accelerometer =
        new AccelerometerModel(chassis, accelerometerErrors, mount, accelerometerSeed);
    return new GyroIOSim(new GyroModel(chassis, gyroErrors, accelerometer, gyroSeed));
  }

  /** The modules' motor controllers and encoders, for the drivetrain. */
  public ModuleIO[] modules() {
    return new ModuleIO[] {modules[0], modules[1], modules[2], modules[3]};
  }

  /** The robot's gyro. */
  public GyroIO gyro() {
    return gyro;
  }

  /** The backup IMU (Systemcore's own), read once a loop. */
  public GyroIO backupGyro() {
    return backupGyro;
  }

  /**
   * The ground the robot drives on: it rides over the bumps, tilted by the ground under its wheels,
   * and its wheels' travel moves it at the slope's cosine. Undoes a tilt made by hand.
   */
  public void setTerrain(Terrain terrain) {
    this.terrain = terrain;
    tiltedByHand = false;
    physics.apply(new Disturbance.Ground(terrain));
  }

  /** The ground the robot drives on. */
  public Terrain terrain() {
    return terrain;
  }

  /** Unplugs the gyro, or plugs it back in. */
  public void disconnectGyro(boolean disconnected) {
    gyro.setConnected(!disconnected);
  }

  /** Freezes the gyro: it keeps reporting, the same yaw every time. */
  public void freezeGyro(boolean frozen) {
    gyro.setFrozen(frozen);
  }

  /** Unplugs the backup IMU, or plugs it back in. */
  public void disconnectBackupGyro(boolean disconnected) {
    backupGyro.setConnected(!disconnected);
  }

  /**
   * Wears one wheel's tread from now on: its true radius over the configured one, 0.97 for tread
   * worn 3%. The robot moves that much less than its encoder counts.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public void wearTread(int module, double scale) {
    physics.apply(new Disturbance.TreadWorn(module, scale));
    treadScale[module] = scale;
  }

  /** A wheel's true radius over the configured one. */
  double treadScale(int module) {
    return treadScale[module];
  }

  /** A wheel's true radius over the configured one, as the robot was built. */
  double builtTreadScale(int module) {
    return builtTreadScale[module];
  }

  /** The simulated gyro, for its faults. */
  GyroIOSim gyroSim() {
    return gyro;
  }

  /** The simulated backup IMU, for its faults. */
  GyroIOSim backupGyroSim() {
    return backupGyro;
  }

  /**
   * A simulated module, for its faults.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  ModuleIOSim moduleSim(int module) {
    return modules[module];
  }

  /** The physics: where the robot, and each wheel, truly is. */
  Physics physics() {
    return physics;
  }

  /**
   * Runs the physics through one loop, and takes the sensors' readings along the way.
   *
   * @param seconds the loop's length
   * @param nowNanos the robot clock at the loop's end: the last reading's time
   */
  public void step(double seconds, long nowNanos) {
    if (!(seconds > 0)) {
      throw new IllegalArgumentException("a step takes time: " + seconds);
    }
    double stepSeconds = seconds / STEPS_PER_LOOP;
    long stepNanos = Math.round(stepSeconds * NANOS_PER_SECOND);
    for (int step = 1; step <= STEPS_PER_LOOP; step++) {
      physics.step(stepSeconds, actuators);
      gyro.feel(stepSeconds);
      backupGyro.feel(stepSeconds);

      // The readings are stamped back from the loop's end, a step apart, so the last one is now.
      int stepsStillToCome = STEPS_PER_LOOP - step;
      long readingTime = nowNanos - stepsStillToCome * stepNanos;
      for (ModuleIOSim module : modules) {
        module.sample(readingTime);
      }
      gyro.sample(readingTime);
      // The backup IMU is read once a loop, not at odometry's rate.
      boolean isTheLoopsLastStep = stepsStillToCome == 0;
      if (isTheLoopsLastStep) {
        backupGyro.sample(readingTime);
      }
    }
    gyro.endLoop(seconds);
    backupGyro.endLoop(seconds);
  }

  /** Where the robot really is. */
  public Pose2d truePose() {
    return physics.chassis().pose();
  }

  /** Where the robot really is, raised and tilted as it is: what its cameras see from. */
  public Pose3d truePose3d() {
    return physics.chassis().pose3d();
  }

  /**
   * Puts the robot somewhere, as the drive team does: its wheels don't turn, but its gyro feels any
   * turn, as a real one would.
   */
  public void place(Pose2d pose) {
    physics.apply(new Disturbance.Placed(pose));
  }

  /**
   * Moves the robot without its wheels turning, as another robot's push or a collision does:
   * odometry can't see it, and only the gyro sees any turn. A shove is in the robot's own frame.
   */
  public void shove(Transform2d shove) {
    place(truePose().plus(shove));
  }

  /**
   * Pushes the robot at this velocity, in its own frame as it faces now, as another robot pushing
   * it does: its wheels skid, never seeing it, while its IMU feels the push get going, and stop, at
   * {@link KinematicPhysics#PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED}. Zero stops it.
   */
  public void push(Translation2d robotRelativeVelocity) {
    Translation2d fieldVelocity = robotRelativeVelocity.rotateBy(truePose().getRotation());
    physics.apply(new Disturbance.Pushed(fieldVelocity));
    pushed = fieldVelocity.getNorm() > 0;
  }

  /** Whether a push is on. Once it's stopped, the robot still slows to its wheels' pace. */
  boolean isPushed() {
    return pushed;
  }

  /**
   * Moves the robot as a collision does ({@link #shove}), with the jolt a hit gives: its IMU feels
   * {@link #HIT_JOLT_METERS_PER_SECOND_SQUARED} the way it was knocked. The jolt is the IMUs' own,
   * a declared override: today's physics moves the robot at once, which no accelerometer could
   * read.
   */
  public void hit(Transform2d shove) {
    Rotation2d direction = shove.getTranslation().getAngle().orElse(Rotation2d.ZERO);
    Translation2d jolt = new Translation2d(HIT_JOLT_METERS_PER_SECOND_SQUARED, direction);
    gyro.jolt(jolt);
    backupGyro.jolt(jolt);
    shove(shove);
  }

  /**
   * Tilts the robot by hand, as a wheel up on a game piece does, whatever the terrain under it
   * says: its IMU reads the attitude. Level again with zeros; back on the terrain with {@link
   * #setTerrain}.
   */
  public void tilt(double pitchRadians, double rollRadians) {
    physics.apply(new Disturbance.TiltedByHand(pitchRadians, rollRadians));
    tiltedByHand = true;
  }

  /** Whether it's tilted by hand, whatever the ground under it says. */
  boolean isTiltedByHand() {
    return tiltedByHand;
  }

  /** How much of its wheels' rolling moves the robot: 1 with full grip. */
  double grip() {
    return grip;
  }

  /**
   * How much of its wheels' rolling moves the robot: 1 on carpet, less with wheels in the air on a
   * bump or spinning on a slick spot. The encoders count all of it.
   */
  public void grip(double fraction) {
    physics.apply(new Disturbance.Grip(fraction));
    grip = fraction;
  }

  /**
   * Spins one wheel without the robot moving: a slip, which its encoder counts and the floor never
   * sees. Odometry catches it as the wheels disagreeing.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public void spin(int module, double radians) {
    modules[module].spin(radians);
  }

  /**
   * From now on one wheel spins this much faster than the floor moves it, in meters per second: a
   * wheel losing its grip, which its encoder counts and the floor never sees. Zero grips again.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public void spinAt(int module, double metersPerSecond) {
    modules[module].spinAt(metersPerSecond);
  }

  /** Every true turn since the start, driving or placed: what a gyro integrates. */
  double yawTurnedRadians() {
    return physics.chassis().yawTurnedRadians();
  }

  double yawRateRadiansPerSecond() {
    return physics.chassis().yawRateRadiansPerSecond();
  }

  double pitchRadians() {
    return physics.chassis().pitchRadians();
  }

  double rollRadians() {
    return physics.chassis().rollRadians();
  }

  /**
   * How fast a point fixed on the robot moves across the field, after the last physics step: the
   * robot's velocity, and its turn swinging the point around its center.
   *
   * @param place where the point sits, from the robot's center
   */
  Translation2d pointVelocity(Translation2d place) {
    return physics.chassis().pointVelocity(place);
  }
}

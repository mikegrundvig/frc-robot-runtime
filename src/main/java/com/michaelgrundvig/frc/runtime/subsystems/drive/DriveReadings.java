package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.OdometrySample;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * What the drivetrain reads in one loop: its four modules, its gyro and its backup IMU, and what
 * follows directly from them, such as how fast the robot is measured to move.
 *
 * <p>Every reading is an AdvantageKit input and is logged as it is read, so a replay of a log hands
 * the drivetrain exactly what it read on the robot, and everything worked out from the readings
 * comes out the same.
 *
 * <p>The wheels and the gyro are also read faster than the loop runs, so that odometry sees every
 * turn of a wheel. Those high-rate readings arrive here in batches, one batch a loop, and are
 * handed on to localization ({@link #forLocalization}), which owns where the robot is.
 */
final class DriveReadings {
  private final Module[] modules;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroIO backupImuIO;
  private final GyroIOInputsAutoLogged backupInputs = new GyroIOInputsAutoLogged();

  /**
   * Held while the hardware is read. On a robot, a background thread records the devices' high-rate
   * readings, and it records only between reads, so that every device hands over the same number of
   * readings, taken at the same times.
   */
  private final Lock odometryLock;

  /** Where each wheel touches the floor, from the robot's center. */
  private final Translation2d[] moduleTranslations;

  /** The kinematics of all four modules. */
  private final SwerveDriveKinematics kinematics;

  /**
   * The kinematics of each set of connected modules short of all four, by the set ({@link
   * #connectedModules}). Each is made the first time its set is seen, which is when a module first
   * drops out.
   */
  private final Map<Integer, SwerveDriveKinematics> kinematicsOfConnected = new HashMap<>();

  /** What was read this loop, as localization takes it. */
  private DrivetrainReadings forLocalization =
      DrivetrainReadings.builder()
          .wheels(List.of())
          .gyro(ImuReadings.NONE)
          .backupImu(ImuReadings.NONE)
          .velocities(new ChassisVelocities())
          .build();

  DriveReadings(
      Module[] modules,
      DriveHardware hardware,
      Translation2d[] moduleTranslations,
      SwerveDriveKinematics kinematics) {
    this.modules = modules;
    this.gyroIO = hardware.gyro();
    this.backupImuIO = hardware.backupImu();
    this.odometryLock = hardware.odometryLock();
    this.moduleTranslations = moduleTranslations;
    this.kinematics = kinematics;
  }

  /** Reads the hardware for a new loop, logs what was read, and works out what follows from it. */
  void read() {
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
    forLocalization =
        DrivetrainReadings.builder()
            .wheels(wheelReadings())
            .gyro(imuReadings(gyroInputs))
            .backupImu(imuReadings(backupInputs))
            .velocities(robotRelativeVelocities())
            .build();
  }

  /** What was read this loop: the wheels at every high-rate reading, and the IMUs. */
  DrivetrainReadings forLocalization() {
    return forLocalization;
  }

  // ---- The wheels' high-rate readings ----

  /**
   * Every high-rate reading of the wheels since the last loop, oldest first.
   *
   * <p>A module that is disconnected reports only what it last sent, as if its wheel stood still.
   * Its wheel is left out of the reading instead, and odometry fits the robot's motion from the
   * wheels that remain. A wheel is left out by giving it NaN for its distance and its angle, which
   * is how an {@link OdometrySample} says a wheel has no reading.
   *
   * <p>A reading that no wheel reports in is dropped, and so is one taken before the robot's clock
   * started. The next good reading covers the motion they would have.
   */
  private List<OdometrySample> wheelReadings() {
    int readings = readingsEveryModuleHas();
    List<OdometrySample> wheels = new ArrayList<>(readings);
    for (int reading = 0; reading < readings; reading++) {
      double[] distancesMeters = new double[Module.COUNT];
      double[] anglesRadians = new double[Module.COUNT];
      boolean anyWheelReports = false;
      for (int i = 0; i < Module.COUNT; i++) {
        if (modules[i].isConnected()) {
          distancesMeters[i] = modules[i].odometryDistanceMeters(reading);
          anglesRadians[i] = modules[i].odometryAngleRadians(reading);
        } else {
          distancesMeters[i] = Double.NaN;
          anglesRadians[i] = Double.NaN;
        }
        anyWheelReports |= Double.isFinite(distancesMeters[i]) && Double.isFinite(anglesRadians[i]);
      }
      // Every module's readings are taken together, so the first module's time is the time of all.
      long takenNanos = modules[0].odometryTimestampNanos(reading);
      boolean takenAfterTheClockStarted = takenNanos >= 0;
      if (anyWheelReports && takenAfterTheClockStarted) {
        wheels.add(new OdometrySample(takenNanos, distancesMeters, anglesRadians));
      }
    }
    return wheels;
  }

  /** How many high-rate readings every module has this loop: the fewest that any one has. */
  private int readingsEveryModuleHas() {
    int fewest = Integer.MAX_VALUE;
    for (Module module : modules) {
      fewest = Math.min(fewest, module.odometryReadings());
    }
    return fewest;
  }

  /** Whether any high-rate reading of the wheels arrived this loop. */
  boolean wheelReadingsArrived() {
    return !forLocalization.wheels().isEmpty();
  }

  /** One IMU's readings this loop, as localization takes them. */
  private static ImuReadings imuReadings(GyroIOInputsAutoLogged imu) {
    int yaws = Math.min(imu.odometryTimestampsNanos.length, imu.odometryYawsRad.length);
    List<Long> yawTimesNanos = new ArrayList<>(yaws);
    List<Double> yawsRadians = new ArrayList<>(yaws);
    for (int i = 0; i < yaws; i++) {
      yawTimesNanos.add(imu.odometryTimestampsNanos[i]);
      yawsRadians.add(imu.odometryYawsRad[i]);
    }
    return ImuReadings.builder()
        .reporting(imu.connected)
        .yawRadians(imu.yaw.getRadians())
        .yawRateRadiansPerSecond(imu.yawVelocityRadPerSec)
        .pitchRadians(imu.pitchRad)
        .rollRadians(imu.rollRad)
        .forwardAccelerationMetersPerSecSq(imu.accelerationXMetersPerSecSq)
        .leftAccelerationMetersPerSecSq(imu.accelerationYMetersPerSecSq)
        .yawTimesNanos(yawTimesNanos)
        .yawsRadians(yawsRadians)
        .build();
  }

  // ---- How the robot is measured to move ----

  /**
   * The robot's measured velocities, relative to itself, from the modules that are connected.
   *
   * <p>A disconnected module reports the speed it last sent, however the robot moves now, so it is
   * left out and the velocities are fitted to the rest. Two modules are the fewest that say how the
   * robot turns. With fewer than two connected there is nothing better to go on than all four as
   * they read, so all four are used.
   */
  ChassisVelocities robotRelativeVelocities() {
    SwerveModuleVelocity[] measured = moduleVelocities();
    int connected = connectedModules();
    int connectedCount = Integer.bitCount(connected);
    boolean allConnected = connectedCount == Module.COUNT;
    boolean tooFewToFitFrom = connectedCount < 2;
    if (allConnected || tooFewToFitFrom) {
      return kinematics.toChassisVelocities(measured);
    }
    SwerveModuleVelocity[] ofConnected = new SwerveModuleVelocity[connectedCount];
    int next = 0;
    for (int i = 0; i < Module.COUNT; i++) {
      if (isIn(connected, i)) {
        ofConnected[next++] = measured[i];
      }
    }
    return kinematicsOfConnected
        .computeIfAbsent(connected, this::kinematicsOf)
        .toChassisVelocities(ofConnected);
  }

  /**
   * The modules that are connected this loop, as a set of module numbers: bit {@code i} of the
   * result is set when module {@code i} is connected. A set in one number can be the key the
   * kinematics for that set are kept under.
   */
  private int connectedModules() {
    int set = 0;
    for (int i = 0; i < Module.COUNT; i++) {
      if (modules[i].isConnected()) {
        set |= 1 << i;
      }
    }
    return set;
  }

  /** Whether a module is in a set of modules ({@link #connectedModules}). */
  private static boolean isIn(int set, int module) {
    return (set & (1 << module)) != 0;
  }

  /** WPILib's kinematics for the modules in a set, in the order the drivetrain numbers them. */
  private SwerveDriveKinematics kinematicsOf(int set) {
    List<Translation2d> at = new ArrayList<>();
    for (int i = 0; i < Module.COUNT; i++) {
      if (isIn(set, i)) {
        at.add(moduleTranslations[i]);
      }
    }
    return new SwerveDriveKinematics(at.toArray(Translation2d[]::new));
  }

  /** Whether every module is connected. */
  boolean allModulesConnected() {
    for (Module module : modules) {
      if (!module.isConnected()) {
        return false;
      }
    }
    return true;
  }

  /** Each module's measured wheel speed and direction, in the order the drivetrain numbers them. */
  SwerveModuleVelocity[] moduleVelocities() {
    SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[Module.COUNT];
    for (int i = 0; i < Module.COUNT; i++) {
      velocities[i] = modules[i].getVelocity();
    }
    return velocities;
  }

  /** How far each wheel has rolled and which way it points, as WPILib's odometry takes them. */
  SwerveModulePosition[] modulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[Module.COUNT];
    for (int i = 0; i < Module.COUNT; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** How far each wheel has turned, in radians, in the order the drivetrain numbers them. */
  double[] wheelRotationsRadians() {
    double[] rotations = new double[Module.COUNT];
    for (int i = 0; i < Module.COUNT; i++) {
      rotations[i] = modules[i].wheelRotationRadians();
    }
    return rotations;
  }

  /** The gyro's yaw, as of this loop. */
  Rotation2d gyroYaw() {
    return gyroInputs.yaw;
  }
}

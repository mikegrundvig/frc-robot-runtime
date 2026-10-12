package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.OdometrySample;
import com.michaelgrundvig.frc.localization.YawSamples;
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

  /**
   * The wheels' distances and angles at one high-rate reading. The two arrays are filled for each
   * reading in turn, and the {@link OdometrySample} made from them copies them, so these two serve
   * every reading of every loop.
   */
  private final double[] distancesMeters = new double[Module.COUNT];

  private final double[] anglesRadians = new double[Module.COUNT];

  /** The loop's high-rate readings as they are gathered. Localization is handed a copy. */
  private final List<OdometrySample> wheelReadingsGathered = new ArrayList<>();

  /**
   * Each module's measured wheel speed and direction as of this loop's read. This array is for the
   * drivetrain's own reading, by the fit below and by the log. WPILib's module velocities can be
   * written to, so anyone else is given their own ({@link #moduleVelocities}).
   */
  private SwerveModuleVelocity[] moduleVelocitiesAsRead;

  /**
   * How the robot is measured to move as of this loop's read, relative to itself. It is fitted to
   * the wheels once, as they are read, because the fit makes a dozen objects and nothing it depends
   * on changes until the next read. It is kept as plain numbers because WPILib's {@link
   * ChassisVelocities} can be written to, so each asker is given its own ({@link
   * #robotRelativeVelocities}).
   */
  private double forwardMetersPerSecond;

  private double leftMetersPerSecond;
  private double turnRateRadiansPerSecond;

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
    moduleVelocitiesAsRead = moduleVelocities();
    keepAsMeasured(fittedToTheWheels());
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
    moduleVelocitiesAsRead = moduleVelocities();
    ChassisVelocities measured = fittedToTheWheels();
    keepAsMeasured(measured);
    forLocalization =
        DrivetrainReadings.builder()
            .wheels(wheelReadings())
            .gyro(imuReadings(gyroInputs))
            .backupImu(imuReadings(backupInputs))
            .velocities(measured)
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
   * <p>A module whose drive controller restarted since the last loop is left out the same way, for
   * this one loop ({@link #countsForOdometry}).
   *
   * <p>A reading that no wheel reports in is dropped, and so is one taken before the robot's clock
   * started. The next good reading covers the motion they would have.
   */
  private List<OdometrySample> wheelReadings() {
    int readings = readingsEveryModuleHas();
    List<OdometrySample> wheels = wheelReadingsGathered;
    wheels.clear();
    for (int reading = 0; reading < readings; reading++) {
      boolean anyWheelReports = false;
      for (int i = 0; i < Module.COUNT; i++) {
        if (countsForOdometry(modules[i])) {
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

  /**
   * Whether a module's wheel counts for odometry this loop: its motor controllers answer, and its
   * drive controller did not restart since the last loop.
   *
   * <p>A restarted drive controller counts its wheel from zero again, somewhere among this loop's
   * readings ({@code ModuleIO}'s M16). The step across that would read as the wheel rolling all the
   * way back to where the robot started. Odometry only ever takes the difference of two readings
   * that both have the wheel, so leaving the wheel out of this whole loop is enough: no step then
   * spans the two counts, and the wheel is back, measured from its new count, with the next loop's
   * readings.
   */
  private static boolean countsForOdometry(Module module) {
    return module.isConnected() && !module.driveRestarted();
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

  /**
   * One IMU's readings this loop, as localization takes them. Its high-rate yaws are handed over as
   * the very arrays the IMU's hardware layer filled, with no copy: every read of an IMU makes new
   * arrays and never writes to the ones before ({@link GyroIO.GyroIOInputs}).
   */
  private static ImuReadings imuReadings(GyroIOInputsAutoLogged imu) {
    // A yaw counts only if it has a time, so only as many count as both arrays have.
    int yaws = Math.min(imu.odometryTimestampsNanos.length, imu.odometryYawsRad.length);
    return ImuReadings.builder()
        .reporting(imu.connected)
        .yawRadians(imu.yaw.getRadians())
        .yawRateRadiansPerSecond(imu.yawVelocityRadPerSec)
        .pitchRadians(imu.pitchRad)
        .rollRadians(imu.rollRad)
        .forwardAccelerationMetersPerSecSq(imu.accelerationXMetersPerSecSq)
        .leftAccelerationMetersPerSecSq(imu.accelerationYMetersPerSecSq)
        .yaws(YawSamples.handedOver(imu.odometryTimestampsNanos, imu.odometryYawsRad, yaws))
        .build();
  }

  // ---- How the robot is measured to move ----

  /**
   * The robot's measured velocities, relative to itself, as of this loop's read: the caller's own
   * to keep or to change.
   */
  ChassisVelocities robotRelativeVelocities() {
    return new ChassisVelocities(
        forwardMetersPerSecond, leftMetersPerSecond, turnRateRadiansPerSecond);
  }

  private void keepAsMeasured(ChassisVelocities measured) {
    forwardMetersPerSecond = measured.vx;
    leftMetersPerSecond = measured.vy;
    turnRateRadiansPerSecond = measured.omega;
  }

  /**
   * The robot's velocities, relative to itself, fitted to what the connected modules measure.
   *
   * <p>A disconnected module reports the speed it last sent, however the robot moves now, so it is
   * left out and the velocities are fitted to the rest. Two modules are the fewest that say how the
   * robot turns. With fewer than two connected there is nothing better to go on than all four as
   * they read, so all four are used.
   */
  private ChassisVelocities fittedToTheWheels() {
    SwerveModuleVelocity[] measured = moduleVelocitiesAsRead;
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

  /**
   * Each module's measured wheel speed and direction as of this loop's read, for the log, which
   * only reads them.
   */
  SwerveModuleVelocity[] moduleVelocitiesAsRead() {
    return moduleVelocitiesAsRead;
  }

  /**
   * Each module's measured wheel speed and direction, in the order the drivetrain numbers them: the
   * caller's own to keep or to change.
   */
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

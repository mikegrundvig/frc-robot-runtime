package com.michaelgrundvig.frc.runtime.commands;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.calibration.CameraMountCalibration;
import com.michaelgrundvig.frc.localization.calibration.GyroScaleCalibration;
import com.michaelgrundvig.frc.localization.calibration.HeadingAverage;
import com.michaelgrundvig.frc.localization.calibration.LevelCheck;
import com.michaelgrundvig.frc.localization.calibration.WheelRadiusCalibration;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.localization.Localization;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.time.LocalDate;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The calibration procedures, as actions for the Pitwall page. Each one drives or watches the
 * robot, works out a number that the robot's configuration should have, and says where that number
 * goes, with the date. Nothing here changes the configuration: the numbers are written into {@code
 * Robots} by hand, and the robot is redeployed.
 *
 * <p>The written procedures are in {@code docs/calibration.md}, and the math is in {@code
 * com.michaelgrundvig.frc.localization.calibration}. This class is the driving and the watching,
 * and the wording of each result.
 */
public final class Calibration {
  // The wheel radius: the robot spins slowly in place, and its wheels' travel is compared with the
  // gyro's turn.

  /** How fast the robot spins for the wheel radius. Slow, so the wheels roll and do not scrub. */
  private static final double WHEEL_SPIN_RADIANS_PER_SECOND = 0.5;

  /** How quickly it comes up to that speed, in radians per second each second. */
  private static final double WHEEL_SPIN_RAMP = 0.25;

  /** How fast it turns before counting, to point the wheels around the circle. */
  private static final double WHEEL_ALIGN_RADIANS_PER_SECOND = 0.1;

  /** How long it turns that way before counting. */
  private static final double WHEEL_ALIGN_SECONDS = 0.5;

  /** How many turns it counts. */
  private static final double WHEEL_SPIN_TURNS = 2;

  // The gyro's scale: the robot spins whole turns, and the gyro's turn is compared with the
  // cameras' heading before and after.

  /** How fast the robot spins for the gyro's scale. */
  private static final double GYRO_SPIN_RADIANS_PER_SECOND = 2;

  /** How quickly it speeds up and slows down, in radians per second each second. */
  private static final double GYRO_SPIN_RAMP = 2;

  /** The slowest it turns as it slows for the last turn, so that it does not stall short of it. */
  private static final double GYRO_SPIN_MIN_RADIANS_PER_SECOND = 0.2;

  /** After spinning, how long the robot settles before the gyro's last reading counts. */
  private static final double SETTLE_SECONDS = 0.5;

  /** How many whole turns it spins: as many as the cameras' reference can still place. */
  private static final int GYRO_SPIN_TURNS = 5;

  /** How long the robot stands still while the cameras measure its heading. */
  private static final double STILL_SECONDS = 1.5;

  /** The fewest heading measurements that make a reference. */
  private static final int MIN_HEADINGS = 5;

  // The level check.

  private static final double LEVEL_SECONDS = 2;
  private static final double LEVEL_TOLERANCE_RADIANS = Math.toRadians(1);

  // A camera's mount: the camera watches tags while the other cameras hold the robot's pose.

  /** The longest a camera's mount is watched for. */
  private static final double MOUNT_SECONDS = 20;

  /** The sightings that end the watching early. */
  private static final int MOUNT_SIGHTINGS = 600;

  /** The fewest sightings that can place a camera. */
  private static final int MIN_MOUNT_SIGHTINGS = 30;

  /** A camera's mount is sure when its position is known to within 1 cm. */
  private static final double MOUNT_SURE_METERS = 0.01;

  /** A camera's mount is sure when its angles are known to within half a degree. */
  private static final double MOUNT_SURE_DEGREES = 0.5;

  private final Drive drive;
  private final Vision vision;
  private final Localization localization;
  private final List<CameraConfig> cameras;
  private final Optional<SwerveWiring> wiring;
  private final DrivetrainGeometry drivetrain;
  private final Supplier<LocalDate> today;
  private String lastResult = "None run yet";

  /**
   * @param cameras the robot's cameras, whose mounts can be calibrated
   * @param wiring the drivetrain's wiring, which holds the gyro's scale as configured. Empty for a
   *     robot that only runs in simulation.
   * @param drivetrain the drivetrain as defined, whose wheel radius the measurement replaces
   * @param today the date, which each result is noted with
   */
  public Calibration(
      Drive drive,
      Vision vision,
      Localization localization,
      List<CameraConfig> cameras,
      Optional<SwerveWiring> wiring,
      DrivetrainGeometry drivetrain,
      Supplier<LocalDate> today) {
    this.drive = drive;
    this.vision = vision;
    this.localization = localization;
    this.cameras = List.copyOf(cameras);
    this.wiring = wiring;
    this.drivetrain = drivetrain;
    this.today = today;
  }

  /** The last procedure's result, for the Pitwall page. */
  public String lastResult() {
    return lastResult;
  }

  /**
   * Adds every procedure to the Pitwall page's actions. Each is named with what it asks of the
   * drive team: how long it takes, and where the robot needs to be.
   *
   * @param scheduler the robot's own scheduler, which runs the procedures
   */
  public void addTo(Actions actions, Scheduler scheduler) {
    double wheelSpinSeconds =
        WHEEL_SPIN_TURNS * 2 * Math.PI / WHEEL_SPIN_RADIANS_PER_SECOND
            + WHEEL_SPIN_RADIANS_PER_SECOND / WHEEL_SPIN_RAMP;
    // The time is said to the next five seconds up.
    double wheelSpinSecondsSaid = 5 * Math.ceil(wheelSpinSeconds / 5);
    actions
        .add(
            format("Calibrate the gyro's scale (%d turns where cameras see tags)", GYRO_SPIN_TURNS),
            Actions.command(() -> scheduler, this::gyroScale))
        .add(
            format(
                "Calibrate the wheel radius (%.0f slow turns in place, about %.0f s)",
                WHEEL_SPIN_TURNS, wheelSpinSecondsSaid),
            Actions.command(() -> scheduler, this::wheelRadius))
        .add(
            format("Check the IMU reads level (%.0f s, on a level floor)", LEVEL_SECONDS),
            Actions.command(() -> scheduler, this::level));
    for (CameraConfig camera : cameras) {
      // A camera on a joint is not calibrated here: where it is depends on where its joint is.
      if (camera.joint().isEmpty()) {
        actions.add(
            cameraMountAction(camera.name()),
            Actions.command(() -> scheduler, () -> cameraMount(camera.name())));
      }
    }
  }

  /** The name of the Pitwall action that calibrates a camera's mount, from the camera's name. */
  public static String cameraMountAction(String camera) {
    return format(
        "Calibrate the %s camera's mount (%.0f s, stop at several spots in view of tags)",
        camera, MOUNT_SECONDS);
  }

  // ---- The wheel radius ----

  /**
   * Measures the wheels' radius, by spinning slowly in place for two turns and comparing how far
   * the wheels rolled with how far the gyro says the robot turned. Calibrate the gyro's scale
   * first, because its error carries straight into the radius.
   */
  public Command wheelRadius() {
    return drive
        .run(this::measureWheelRadius)
        .whenCanceled(() -> report("Wheel radius: canceled before it finished"))
        .named("Calibrate wheel radius");
  }

  private void measureWheelRadius(Coroutine coroutine) {
    // A slow half second of turning points the wheels around the circle before anything is counted.
    spinFor(coroutine, WHEEL_ALIGN_RADIANS_PER_SECOND, WHEEL_ALIGN_SECONDS);
    WheelRadiusCalibration spin = new WheelRadiusCalibration(drive.moduleLocations());
    spin.start(drive.gyroYaw().getRadians(), drive.wheelRotationsRadians());
    double rate = WHEEL_ALIGN_RADIANS_PER_SECOND;
    while (spin.turnedRadians() < WHEEL_SPIN_TURNS * 2 * Math.PI) {
      double rampedUp = rate + WHEEL_SPIN_RAMP * RobotLoop.PERIOD_SECONDS;
      rate = Math.min(WHEEL_SPIN_RADIANS_PER_SECOND, rampedUp);
      drive.runVelocity(new ChassisVelocities(0, 0, rate));
      coroutine.yield();
      spin.update(drive.gyroYaw().getRadians(), drive.wheelRotationsRadians());
    }
    stop(coroutine);
    report(wheelRadiusResult(spin));
  }

  private String wheelRadiusResult(WheelRadiusCalibration spin) {
    double configured = drivetrain.module(0).wheelRadius().in(Inches);
    double measured = Meters.of(spin.radiusMeters()).in(Inches);
    double[] wheels = spin.radiiMeters();
    return format(
        "Wheel radius %.3f in by spinning in place (configured %.3f); each wheel %.3f,"
            + " %.3f, %.3f, %.3f. In Robots: .wheelRadius(Inches.of(%.3f)) //"
            + " measured %s",
        measured,
        configured,
        Meters.of(wheels[0]).in(Inches),
        Meters.of(wheels[1]).in(Inches),
        Meters.of(wheels[2]).in(Inches),
        Meters.of(wheels[3]).in(Inches),
        measured,
        today.get());
  }

  // ---- The gyro's scale ----

  /**
   * Measures the gyro's scale, by spinning five whole turns in place where the cameras see several
   * tags. The heading the cameras measure before and after is the reference the gyro's turn is
   * compared with.
   */
  public Command gyroScale() {
    return drive
        .run(this::measureGyroScale)
        .whenCanceled(
            () -> {
              vision.stopCollectingHeadings();
              report("Gyro scale: canceled before it finished");
            })
        .named("Calibrate gyro scale");
  }

  private void measureGyroScale(Coroutine coroutine) {
    HeadingAverage before = headingsWhileStill(coroutine);
    if (before.count() < MIN_HEADINGS) {
      report(noReference(before));
      return;
    }
    GyroScaleCalibration spin = new GyroScaleCalibration();
    spin.start(drive.gyroYaw().getRadians(), before.average());
    spinWholeTurns(coroutine, spin);
    stop(coroutine);
    // The robot settles, with the gyro still followed, before the cameras measure again.
    followTheGyroFor(coroutine, spin, SETTLE_SECONDS);
    HeadingAverage after = new HeadingAverage();
    vision.collectHeadings(after);
    followTheGyroFor(coroutine, spin, STILL_SECONDS);
    vision.stopCollectingHeadings();
    if (after.count() < MIN_HEADINGS) {
      report(noReference(after));
      return;
    }
    spin.finish(drive.gyroYaw().getRadians(), after.average());
    report(gyroScaleResult(spin));
  }

  /**
   * Spins the robot through the gyro's whole turns: speeding up, holding its speed, and slowing in
   * time to stop near the end of the last turn.
   */
  private void spinWholeTurns(Coroutine coroutine, GyroScaleCalibration spin) {
    double target = GYRO_SPIN_TURNS * 2 * Math.PI;
    double rate = 0;
    while (spin.gyroTurnedRadians() < target) {
      double left = target - spin.gyroTurnedRadians();
      // The fastest it can turn now and still stop within what is left, slowing at the ramp.
      double ableToStop = Math.sqrt(2 * GYRO_SPIN_RAMP * left);
      double rampedUp = rate + GYRO_SPIN_RAMP * RobotLoop.PERIOD_SECONDS;
      double wanted = Math.min(Math.min(GYRO_SPIN_RADIANS_PER_SECOND, ableToStop), rampedUp);
      rate = Math.max(GYRO_SPIN_MIN_RADIANS_PER_SECOND, wanted);
      drive.runVelocity(new ChassisVelocities(0, 0, rate));
      coroutine.yield();
      spin.update(drive.gyroYaw().getRadians());
    }
  }

  /** Waits, telling the calibration each loop what the gyro reads, so that it misses no turn. */
  private void followTheGyroFor(Coroutine coroutine, GyroScaleCalibration spin, double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      coroutine.yield();
      spin.update(drive.gyroYaw().getRadians());
    }
  }

  private String gyroScaleResult(GyroScaleCalibration spin) {
    double configured = wiring.map(SwerveWiring::gyroYawScale).orElse(1.0);
    double scale = configured * spin.scale();
    String readsHighOrLow = spin.scale() < 1 ? "high" : "low";
    return format(
        "Gyro scale %.5f: it reads %.2f%% %s over %d turns (%.1f° off). In the"
            + " robot's wiring: .gyroYawScale(%.5f) // measured %s",
        scale,
        Math.abs(1 / spin.scale() - 1) * 100,
        readsHighOrLow,
        GYRO_SPIN_TURNS,
        Math.toDegrees(spin.gyroTurnedRadians() - spin.trueTurnedRadians()),
        scale,
        today.get());
  }

  /** Stops the robot, and collects the cameras' headings while it stands still. */
  private HeadingAverage headingsWhileStill(Coroutine coroutine) {
    HeadingAverage headings = new HeadingAverage();
    stop(coroutine);
    vision.collectHeadings(headings);
    for (int i = 0; i < RobotLoop.loops(STILL_SECONDS); i++) {
      coroutine.yield();
    }
    vision.stopCollectingHeadings();
    return headings;
  }

  /** What is said when the cameras measured the heading too few times to make a reference. */
  private static String noReference(HeadingAverage headings) {
    return format(
        "Gyro scale: the cameras measured the heading %d times standing still; it needs %d. Face"
            + " where they see two or more tags.",
        headings.count(), MIN_HEADINGS);
  }

  // ---- The level check ----

  /**
   * Checks whether the IMU reads level, with the robot standing still on a level floor for two
   * seconds.
   */
  public Command level() {
    return Command.noRequirements(
            coroutine -> {
              LevelCheck level = new LevelCheck();
              for (int i = 0; i < RobotLoop.loops(LEVEL_SECONDS); i++) {
                level.add(localization.pitchRadians(), localization.rollRadians());
                coroutine.yield();
              }
              report(level.describe(LEVEL_TOLERANCE_RADIANS) + ", " + today.get());
            })
        .whenCanceled(() -> report("Level check: canceled before it finished"))
        .named("Check the IMU reads level");
  }

  // ---- A camera's mount ----

  /**
   * Finds where a camera really is, from the tags it sees while the other cameras hold the robot's
   * pose. The drive team stops at several spots in view of tags, nearer and farther, within twenty
   * seconds: an image counts only once the robot has stood still for half a second.
   *
   * @param name the name of a camera on the chassis
   * @throws IllegalArgumentException if no camera on the chassis has this name
   */
  public Command cameraMount(String name) {
    CameraConfig camera =
        cameras.stream()
            .filter(each -> each.name().equals(name) && each.joint().isEmpty())
            .findFirst()
            .orElseThrow(
                () -> new IllegalArgumentException("no camera on the chassis named " + name));
    return Command.noRequirements(coroutine -> measureCameraMount(coroutine, camera))
        // A canceled command never resumes, so nothing after the point it was canceled at runs,
        // not even a finally block. Without this, the camera would stay out of localization until
        // the robot restarts.
        .whenCanceled(
            () -> {
              vision.stopCalibratingMount();
              report(name + ": canceled before it finished");
            })
        .named("Calibrate camera mount: " + name);
  }

  private void measureCameraMount(Coroutine coroutine, CameraConfig camera) {
    Optional<String> cannot = vision.whyNoMountIsCalibrated();
    if (cannot.isPresent()) {
      report(camera.name() + ": not calibrated: " + cannot.get());
      return;
    }
    CameraMountCalibration mount = new CameraMountCalibration(camera.robotToCamera());
    vision.calibrateMount(camera.name(), mount);
    // Watches until the time is up, or until there are sightings enough, whichever comes first.
    for (int i = 0; i < RobotLoop.loops(MOUNT_SECONDS); i++) {
      if (mount.sightings() >= MOUNT_SIGHTINGS) {
        break;
      }
      coroutine.yield();
    }
    vision.stopCalibratingMount();

    Optional<CameraMountCalibration.Result> solved = Optional.empty();
    if (mount.sightings() >= MIN_MOUNT_SIGHTINGS) {
      solved = mount.solve();
    }
    if (solved.isPresent()) {
      report(cameraMountResult(camera, solved.get()));
    } else {
      report(
          format(
              "%s: too few sightings (%d) to place it. Stop for two seconds at each of several"
                  + " spots where it sees tags, nearer and farther, and where the other cameras"
                  + " see tags too: an image counts only while the robot stands still.",
              camera.name(), mount.sightings()));
    }
  }

  private String cameraMountResult(CameraConfig camera, CameraMountCalibration.Result solved) {
    Transform3d mount = solved.robotToCamera();
    Rotation3d turn = mount.getRotation();
    // How sure the solve is of each of the mount's six numbers: x, y, z, then the three angles.
    double[] sure = solved.stdDevs();
    double placeSure = Math.max(sure[0], Math.max(sure[1], sure[2]));
    double angleSure = Math.max(sure[3], Math.max(sure[4], sure[5]));
    boolean isUnsure =
        placeSure > MOUNT_SURE_METERS || angleSure > Math.toRadians(MOUNT_SURE_DEGREES);
    String unsure = "";
    if (isUnsure) {
      unsure = " Not sure of it: drive nearer and farther from the tags, and run it again.";
    }
    return format(
        "%s: tags landed %.1f cm off, %.1f cm with the mount found (%d sightings; to within %.2f"
            + " in and %.2f°).%s In Robots, the %s camera's mount: .forward(Inches.of(%.2f))"
            + ".left(Inches.of(%.2f)).up(Inches.of(%.2f)).roll(Degrees.of(%.1f))"
            + ".tiltUp(Degrees.of(%.1f)).yaw(Degrees.of(%.1f)) // measured %s",
        camera.name(),
        solved.residualBeforeMeters() * 100,
        solved.residualAfterMeters() * 100,
        solved.sightings(),
        Meters.of(placeSure).in(Inches),
        Math.toDegrees(angleSure),
        unsure,
        camera.name(),
        Meters.of(mount.getX()).in(Inches),
        Meters.of(mount.getY()).in(Inches),
        Meters.of(mount.getZ()).in(Inches),
        Math.toDegrees(turn.getX()),
        // The configuration's tilt is positive upward, and a rotation about y is positive downward.
        -Math.toDegrees(turn.getY()),
        Math.toDegrees(turn.getZ()),
        today.get());
  }

  // ---- Shared by the procedures ----

  /** Keeps a procedure's result for the Pitwall page, and logs it. */
  private void report(String result) {
    lastResult = result;
    Logger.recordOutput("Calibration/Result", result);
  }

  /** Turns in place at a steady rate for a time. */
  private void spinFor(Coroutine coroutine, double radiansPerSecond, double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      drive.runVelocity(new ChassisVelocities(0, 0, radiansPerSecond));
      coroutine.yield();
    }
  }

  /** Tells the drivetrain to stop, and waits a loop for it to have been told. */
  private void stop(Coroutine coroutine) {
    drive.runVelocity(new ChassisVelocities(0, 0, 0));
    coroutine.wait(Seconds.of(RobotLoop.PERIOD_SECONDS));
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}

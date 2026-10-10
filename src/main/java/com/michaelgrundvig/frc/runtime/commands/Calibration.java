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
 * The calibration procedures, as actions for the Pitwall page: each drives or watches the robot,
 * works out a number its configuration should have, and says where it goes, with the date. The
 * written procedures are in {@code docs/calibration.md}; the math is in {@code
 * com.michaelgrundvig.frc.localization.calibration}. Nothing is changed on its own: the numbers go
 * into {@code Robots}, and the robot is redeployed.
 */
public final class Calibration {
  /**
   * Spinning for the wheel radius: slowly, so the wheels roll rather than scrub (a top team's way).
   */
  private static final double WHEEL_SPIN_RADIANS_PER_SECOND = 0.5;

  private static final double WHEEL_SPIN_RAMP = 0.25;

  /** Before counting, a slow half second of turning points the wheels around the circle. */
  private static final double WHEEL_ALIGN_RADIANS_PER_SECOND = 0.1;

  private static final double WHEEL_ALIGN_SECONDS = 0.5;
  private static final double WHEEL_SPIN_TURNS = 2;

  /** Spinning for the gyro's scale: whole turns, as many as the reference can still place. */
  private static final double GYRO_SPIN_RADIANS_PER_SECOND = 2;

  private static final double GYRO_SPIN_RAMP = 2;

  /** Slowing for the last turn, never slower than this, so the robot doesn't stall short of it. */
  private static final double GYRO_SPIN_MIN_RADIANS_PER_SECOND = 0.2;

  /** After spinning, how long the robot settles before the gyro's last reading counts. */
  private static final double SETTLE_SECONDS = 0.5;

  private static final int GYRO_SPIN_TURNS = 5;
  private static final double STILL_SECONDS = 1.5;
  private static final int MIN_HEADINGS = 5;

  private static final double LEVEL_SECONDS = 2;
  private static final double LEVEL_TOLERANCE_RADIANS = Math.toRadians(1);

  /** A camera's mount: this long, or this many sightings, whichever comes first. */
  private static final double MOUNT_SECONDS = 20;

  private static final int MOUNT_SIGHTINGS = 600;
  private static final int MIN_MOUNT_SIGHTINGS = 30;

  /** A camera's mount is sure when its position is known within 1 cm and its angles 0.5°. */
  private static final double MOUNT_SURE_METERS = 0.01;

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
   * @param wiring the drivetrain's wiring, with the gyro's scale as configured; empty for a robot
   *     that only runs in simulation
   * @param drivetrain the drivetrain as defined, whose wheel radius the measurement replaces
   * @param today the date, for each result's note
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
   * Adds every procedure to the Pitwall page's actions, each named with what it asks of the drive
   * team: how long it takes, and where the robot needs to be.
   *
   * @param scheduler runs the procedures: the robot's own
   */
  public void addTo(Actions actions, Scheduler scheduler) {
    double wheelSpinSeconds =
        WHEEL_SPIN_TURNS * 2 * Math.PI / WHEEL_SPIN_RADIANS_PER_SECOND
            + WHEEL_SPIN_RADIANS_PER_SECOND / WHEEL_SPIN_RAMP;
    actions
        .add(
            format("Calibrate the gyro's scale (%d turns where cameras see tags)", GYRO_SPIN_TURNS),
            Actions.command(() -> scheduler, this::gyroScale))
        .add(
            format(
                "Calibrate the wheel radius (%.0f slow turns in place, about %.0f s)",
                WHEEL_SPIN_TURNS, 5 * Math.ceil(wheelSpinSeconds / 5)),
            Actions.command(() -> scheduler, this::wheelRadius))
        .add(
            format("Check the IMU reads level (%.0f s, on a level floor)", LEVEL_SECONDS),
            Actions.command(() -> scheduler, this::level));
    // A camera on a joint isn't calibrated here: where it is is its joint's.
    for (CameraConfig camera : cameras) {
      if (camera.joint().isEmpty()) {
        actions.add(
            cameraMountAction(camera.name()),
            Actions.command(() -> scheduler, () -> cameraMount(camera.name())));
      }
    }
  }

  /** The Pitwall action that calibrates a camera's mount, by the camera's name. */
  public static String cameraMountAction(String camera) {
    return format(
        "Calibrate the %s camera's mount (%.0f s, drive slowly past tags)", camera, MOUNT_SECONDS);
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }

  private void report(String result) {
    lastResult = result;
    Logger.recordOutput("Calibration/Result", result);
  }

  /**
   * The wheels' radius, by spinning slowly in place twice against the gyro. Calibrate the gyro's
   * scale first: its error carries straight into the radius.
   */
  public Command wheelRadius() {
    return drive
        .run(
            coroutine -> {
              // Point the wheels around the circle before counting.
              spinFor(coroutine, WHEEL_ALIGN_RADIANS_PER_SECOND, WHEEL_ALIGN_SECONDS);
              WheelRadiusCalibration spin = new WheelRadiusCalibration(drive.moduleLocations());
              spin.start(drive.gyroYaw().getRadians(), drive.wheelRotationsRadians());
              double rate = WHEEL_ALIGN_RADIANS_PER_SECOND;
              while (spin.turnedRadians() < WHEEL_SPIN_TURNS * 2 * Math.PI) {
                rate =
                    Math.min(
                        WHEEL_SPIN_RADIANS_PER_SECOND,
                        rate + WHEEL_SPIN_RAMP * RobotLoop.PERIOD_SECONDS);
                drive.runVelocity(new ChassisVelocities(0, 0, rate));
                coroutine.yield();
                spin.update(drive.gyroYaw().getRadians(), drive.wheelRotationsRadians());
              }
              stop(coroutine);
              double configured = drivetrain.module(0).wheelRadius().in(Inches);
              double measured = Meters.of(spin.radiusMeters()).in(Inches);
              double[] wheels = spin.radiiMeters();
              report(
                  String.format(
                      Locale.ROOT,
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
                      today.get()));
            })
        .whenCanceled(() -> report("Wheel radius: canceled before it finished"))
        .named("Calibrate wheel radius");
  }

  /**
   * The gyro's scale, by five whole turns in place where the cameras see several tags: their
   * heading before and after is the reference.
   */
  public Command gyroScale() {
    return drive
        .run(
            coroutine -> {
              HeadingAverage before = headingsWhileStill(coroutine);
              if (before.count() < MIN_HEADINGS) {
                report(noReference(before));
                return;
              }
              GyroScaleCalibration spin = new GyroScaleCalibration();
              spin.start(drive.gyroYaw().getRadians(), before.average());
              double target = GYRO_SPIN_TURNS * 2 * Math.PI;
              double rate = 0;
              while (spin.gyroTurnedRadians() < target) {
                // Slow down in time to stop near the last whole turn.
                double left = target - spin.gyroTurnedRadians();
                double stopping = Math.sqrt(2 * GYRO_SPIN_RAMP * left);
                rate =
                    Math.max(
                        GYRO_SPIN_MIN_RADIANS_PER_SECOND,
                        Math.min(
                            Math.min(GYRO_SPIN_RADIANS_PER_SECOND, stopping),
                            rate + GYRO_SPIN_RAMP * RobotLoop.PERIOD_SECONDS));
                drive.runVelocity(new ChassisVelocities(0, 0, rate));
                coroutine.yield();
                spin.update(drive.gyroYaw().getRadians());
              }
              stop(coroutine);
              for (int i = 0; i < RobotLoop.loops(SETTLE_SECONDS); i++) {
                coroutine.yield();
                spin.update(drive.gyroYaw().getRadians());
              }
              HeadingAverage after = new HeadingAverage();
              vision.collectHeadings(after);
              for (int i = 0; i < RobotLoop.loops(STILL_SECONDS); i++) {
                coroutine.yield();
                spin.update(drive.gyroYaw().getRadians());
              }
              vision.stopCollectingHeadings();
              if (after.count() < MIN_HEADINGS) {
                report(noReference(after));
                return;
              }
              spin.finish(drive.gyroYaw().getRadians(), after.average());
              double configured = wiring.map(SwerveWiring::gyroYawScale).orElse(1.0);
              double scale = configured * spin.scale();
              report(
                  String.format(
                      Locale.ROOT,
                      "Gyro scale %.5f: it reads %.2f%% %s over %d turns (%.1f° off). In the"
                          + " robot's wiring: .gyroYawScale(%.5f) // measured %s",
                      scale,
                      Math.abs(1 / spin.scale() - 1) * 100,
                      spin.scale() < 1 ? "high" : "low",
                      GYRO_SPIN_TURNS,
                      Math.toDegrees(spin.gyroTurnedRadians() - spin.trueTurnedRadians()),
                      scale,
                      today.get()));
            })
        .whenCanceled(
            () -> {
              vision.stopCollectingHeadings();
              report("Gyro scale: canceled before it finished");
            })
        .named("Calibrate gyro scale");
  }

  /** Whether the IMU reads level: the robot standing still on a level floor for two seconds. */
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

  /**
   * Where a camera really is, from the tags it sees while the other cameras hold the robot's pose:
   * drive slowly past tags, nearer and farther, for twenty seconds.
   */
  public Command cameraMount(String name) {
    CameraConfig camera =
        cameras.stream()
            .filter(each -> each.name().equals(name) && each.joint().isEmpty())
            .findFirst()
            .orElseThrow(
                () -> new IllegalArgumentException("no camera on the chassis named " + name));
    return Command.noRequirements(
            coroutine -> {
              CameraMountCalibration mount = new CameraMountCalibration(camera.robotToCamera());
              vision.calibrateMount(name, mount);
              for (int i = 0; i < RobotLoop.loops(MOUNT_SECONDS); i++) {
                if (mount.sightings() >= MOUNT_SIGHTINGS) {
                  break;
                }
                coroutine.yield();
              }
              vision.stopCalibratingMount();
              Optional<CameraMountCalibration.Result> result =
                  mount.sightings() >= MIN_MOUNT_SIGHTINGS ? mount.solve() : Optional.empty();
              report(
                  result
                      .map(solved -> describe(camera, solved))
                      .orElse(
                          String.format(
                              Locale.ROOT,
                              "%s: too few sightings (%d) to place it. Drive slowly past tags,"
                                  + " nearer and farther, where the other cameras see tags too.",
                              name,
                              mount.sightings())));
            })
        // A canceled command never resumes, so nothing after the loop runs (not even a finally):
        // without this, the camera would stay out of localization until the robot restarts.
        .whenCanceled(
            () -> {
              vision.stopCalibratingMount();
              report(name + ": canceled before it finished");
            })
        .named("Calibrate camera mount: " + name);
  }

  private String describe(CameraConfig camera, CameraMountCalibration.Result solved) {
    Transform3d mount = solved.robotToCamera();
    Rotation3d turn = mount.getRotation();
    double[] sure = solved.stdDevs();
    double placeSure = Math.max(sure[0], Math.max(sure[1], sure[2]));
    double angleSure = Math.max(sure[3], Math.max(sure[4], sure[5]));
    String unsure =
        placeSure > MOUNT_SURE_METERS || angleSure > Math.toRadians(MOUNT_SURE_DEGREES)
            ? " Not sure of it: drive nearer and farther from the tags, and run it again."
            : "";
    return String.format(
        Locale.ROOT,
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
        -Math.toDegrees(turn.getY()),
        Math.toDegrees(turn.getZ()),
        today.get());
  }

  /** Stands still, collecting the cameras' headings. */
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

  private static String noReference(HeadingAverage headings) {
    return String.format(
        Locale.ROOT,
        "Gyro scale: the cameras measured the heading %d times standing still; it needs %d. Face"
            + " where they see two or more tags.",
        headings.count(),
        MIN_HEADINGS);
  }

  private void spinFor(Coroutine coroutine, double radiansPerSecond, double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      drive.runVelocity(new ChassisVelocities(0, 0, radiansPerSecond));
      coroutine.yield();
    }
  }

  private void stop(Coroutine coroutine) {
    drive.runVelocity(new ChassisVelocities(0, 0, 0));
    coroutine.wait(Seconds.of(RobotLoop.PERIOD_SECONDS));
  }
}

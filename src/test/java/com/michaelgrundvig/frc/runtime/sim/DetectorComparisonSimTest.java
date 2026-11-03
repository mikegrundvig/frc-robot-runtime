package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.localization.TagSolver;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.DetectionModel;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.system.RobotController;

/**
 * Detectors compared on the same drive: a lap of 2026's field with the template robot's four
 * cameras, the same images, the same corner noise, each detector deciding which tags it finds by
 * their size in the image ({@link DetectionModel}, measured on rendered images). Each tag found is
 * solved as the pipeline solves one, its position turned by the robot's heading, and checked
 * against where the robot was.
 *
 * <p>It measures range only: which tags each detector finds. Blur, which the measurements left out,
 * and speed, which decides how many images a coprocessor can take, aren't in it. The pipeline's 6 m
 * cutoff (mean tag distance) is shown both applied and lifted.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class DetectorComparisonSimTest {
  /** Once round the field, through the middle of each half, the robot facing the way it drives. */
  private static final double LAP_SECONDS = 20;

  private static Pose2d onTheLap(double seconds) {
    double angle = 2 * Math.PI * seconds / LAP_SECONDS;
    double x = 8.27 + 5.5 * Math.cos(angle);
    double y = 4.03 + 2.6 * Math.sin(angle);
    double heading = Math.atan2(2.6 * Math.cos(angle), -5.5 * Math.sin(angle));
    return new Pose2d(x, y, new Rotation2d(heading));
  }

  /** The pipeline's current cutoff: an image whose tags average farther is refused. */
  private static final double CUTOFF_METRES = 6;

  /** One tag found and solved: how far it was, how far off its fix, and when. */
  private record Fix(double distanceMetres, double errorMetres, double seconds, boolean inside6m) {}

  /** What one detector made of the lap. */
  private record Lap(String detector, int images, int imagesWithTags, List<Fix> fixes) {
    double perSecond(boolean onlyInside6m) {
      return fixes.stream().filter(f -> f.inside6m() || !onlyInside6m).count() / LAP_SECONDS;
    }

    double farthest() {
      return fixes.stream().mapToDouble(Fix::distanceMetres).max().orElse(0);
    }

    /** The longest the robot went without a fix, start and end of the lap included. */
    double longestGap(boolean onlyInside6m) {
      List<Double> times =
          fixes.stream()
              .filter(f -> f.inside6m() || !onlyInside6m)
              .map(Fix::seconds)
              .sorted()
              .toList();
      double gap = 0;
      double last = 0;
      for (double time : times) {
        gap = Math.max(gap, time - last);
        last = time;
      }
      return Math.max(gap, LAP_SECONDS - last);
    }

    /** The fixes' errors from tags in a band of distance: median, 95th percentile, and how many. */
    String errors(double fromMetres, double toMetres) {
      List<Double> errors =
          fixes.stream()
              .filter(f -> f.distanceMetres() >= fromMetres && f.distanceMetres() < toMetres)
              .map(Fix::errorMetres)
              .sorted()
              .toList();
      if (errors.isEmpty()) {
        return "%-21s".formatted("none");
      }
      return String.format(
          Locale.ROOT,
          "%4.0f / %4.0f mm (%4d)",
          1000 * errors.get(errors.size() / 2),
          1000 * errors.get(errors.size() * 95 / 100),
          errors.size());
    }
  }

  private static Lap drive(
      String name, CameraConfig.Software software, @Nullable DetectionModel model) {
    int images = 0;
    int imagesWithTags = 0;
    List<Fix> fixes = new ArrayList<>();
    try (SimulatedWorld world =
        new SimulatedWorld(
            RobotSim.FIELD,
            ExampleRobot.SIM.definition(),
            RobotSim.massOf(ExampleRobot.SIM.definition()),
            SensorErrors.NONE,
            software,
            model)) {
      world.place(onTheLap(0));
      CameraIO[] cameras = world.cameras();
      long setOff = RobotController.getTime();
      // Where the robot truly was, tilted on the field's bumps as it crossed them, by time since it
      // set off: an image is checked against the robot at its stamp.
      TimeInterpolatableBuffer<Pose3d> truths = TimeInterpolatableBuffer.createBuffer(2.0);
      for (int i = 0; i < RobotLoop.loops(LAP_SECONDS); i++) {
        RobotSim.stepClock();
        world.place(onTheLap((RobotController.getTime() - setOff) / 1e9));
        world.step(RobotController.getTime());
        truths.addSample(
            (RobotController.getTime() - setOff) / 1e9, world.drivetrain().truePose3d());
        for (int c = 0; c < cameras.length; c++) {
          CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
          cameras[c].updateInputs(inputs);
          Transform3d mount = ExampleRobot.SIM.definition().cameras().get(c).robotToCamera();
          int next = 0;
          for (CameraIO.Frame frame : inputs.frames) {
            images++;
            List<CameraIO.SeenTag> tags =
                List.of(inputs.tags).subList(next, next + frame.tagCount());
            next += frame.tagCount();
            if (tags.isEmpty()) {
              continue;
            }
            imagesWithTags++;
            double seconds = (frame.timestampNanos() - setOff) / 1e9;
            Pose3d truth = truths.getSample(seconds).orElseThrow();
            double meanDistance =
                tags.stream()
                    .mapToDouble(tag -> tag.bestCameraToTag().getTranslation().getNorm())
                    .average()
                    .orElseThrow();
            for (CameraIO.SeenTag tag : tags) {
              Pose3d tagPose = RobotSim.FIELD.tag(tag.id()).orElseThrow();
              // With the robot's attitude, as the pipeline takes it from the gyro: heading, pitch
              // and roll.
              TagSolver.PositionFix fix =
                  TagSolver.positionFromTag(
                      tagPose, mount.plus(tag.bestCameraToTag()), truth.getRotation());
              fixes.add(
                  new Fix(
                      tag.bestCameraToTag().getTranslation().getNorm(),
                      fix.position().getDistance(truth.toPose2d().getTranslation()),
                      seconds,
                      meanDistance <= CUTOFF_METRES));
            }
          }
        }
      }
    }
    return new Lap(name, images, imagesWithTags, fixes);
  }

  @Test
  void napaFindsFartherTagsThanWpilibsDetector() {
    Map<String, Lap> laps = new LinkedHashMap<>();
    laps.put(
        "simulator",
        drive("Simulator's limits (7 m, 100 px²)", CameraConfig.Software.PHOTONVISION, null));
    for (DetectionModel model :
        List.of(
            DetectionModel.WPILIB_DECIMATE_2,
            DetectionModel.WPILIB_DECIMATE_1,
            DetectionModel.NAPA)) {
      laps.put(model.name(), drive(model.name(), CameraConfig.Software.VISOR, model));
    }

    System.out.printf(
        Locale.ROOT,
        "%-34s %8s %8s %9s %9s %10s %10s%n",
        "one %.0f s lap of the field".formatted(LAP_SECONDS),
        "w/ tags",
        "fixes/s",
        "≤6m /s",
        "farthest",
        "worst gap",
        "gap ≤6m");
    for (Lap lap : laps.values()) {
      System.out.printf(
          Locale.ROOT,
          "%-34s %7.0f%% %8.0f %9.0f %8.1fm %9.2fs %9.2fs%n",
          lap.detector(),
          100.0 * lap.imagesWithTags() / lap.images(),
          lap.perSecond(false),
          lap.perSecond(true),
          lap.farthest(),
          lap.longestGap(false),
          lap.longestGap(true));
    }
    System.out.printf(
        Locale.ROOT,
        "%-34s %-22s %-22s %-22s %-22s%n",
        "fix error, median / 95% (fixes)",
        "0-3 m",
        "3-6 m",
        "6-9 m",
        "9 m and on");
    for (Lap lap : laps.values()) {
      System.out.printf(
          Locale.ROOT,
          "%-34s %-22s %-22s %-22s %-22s%n",
          lap.detector(),
          lap.errors(0, 3),
          lap.errors(3, 6),
          lap.errors(6, 9),
          lap.errors(9, 100));
    }

    Lap decimate2 = Objects.requireNonNull(laps.get("WPILib decimate 2"));
    Lap decimate1 = Objects.requireNonNull(laps.get("WPILib decimate 1"));
    Lap napa = Objects.requireNonNull(laps.get("NAPA"));
    assertThat(napa.perSecond(false)).isGreaterThan(decimate1.perSecond(false));
    assertThat(decimate1.perSecond(false)).isGreaterThan(decimate2.perSecond(false));
    assertThat(napa.farthest()).isGreaterThan(decimate1.farthest());
    assertThat(napa.longestGap(false)).isLessThanOrEqualTo(decimate2.longestGap(false));
  }
}

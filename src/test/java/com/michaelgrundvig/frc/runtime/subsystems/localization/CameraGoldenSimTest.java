package com.michaelgrundvig.frc.runtime.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.TagAllowlist;
import com.michaelgrundvig.frc.localization.Verdict;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.localization.VisionConfigBuilder;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.localization.VisionResult;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.GoldenFile;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.runtime.sim.FakeTagCamera;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.RobotController;

/**
 * Localization's camera path, recorded: the template's four cameras send raw tag images through a
 * seeded session (standing, driving, a task trusting only the hub's tags, a knocked camera, a tilt,
 * a spin), and localization solves each as the robot was when it was taken. Every loop's estimate,
 * every image's solve, and every measurement's verdict must match {@value #GOLDEN}.
 *
 * <p>{@link EstimatorGoldenSimTest} guards the estimator, fed measurements already solved; this
 * guards everything before it on the camera path: the pipeline, each image's capture (the heading,
 * the attitude and the turning then), a camera's mount doubt, and a task's tags. Record it again as
 * that test says.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class CameraGoldenSimTest {
  static final String GOLDEN = "camera-golden.csv";

  /** In front of the blue hub's near face (tags 25 and 26), facing it. */
  private static final Pose2d START = new Pose2d(2.5, 4.1, Rotation2d.fromDegrees(10));

  private static final String HEADER =
      "L,loop,x,y,heading,mode,accepted,rejected / I,camera,taken,solved,outcome,x,y,heading,xx,yy,"
          + "headingVariance / F,camera,taken,verdict,solver,innovation,chi2,threshold,tags";

  @Test
  void theCameraPathBehavesAsRecorded() throws IOException {
    GoldenFile.check(getClass(), GOLDEN, HEADER, session().rows());
  }

  /** The session goes where it's meant to: every part of the camera path has its say. */
  @Test
  void theSessionReachesEveryPartOfTheCameraPath() {
    Session session = session();

    assertThat(session.images())
        .extracting(VisionResult::refused)
        .as("spinning, and a task's tags")
        .contains(Verdict.TURNING, Verdict.UNTRUSTED_TAGS);
    assertThat(session.images())
        .filteredOn(image -> image.measurement() != null)
        .extracting(VisionResult::outcome)
        .anyMatch(outcome -> outcome.endsWith("with heading"))
        .anyMatch(outcome -> outcome.startsWith("heading-constrained"));
    assertThat(session.frames())
        .extracting(FrameVerdict::verdict)
        .as("the update says what became of every image, refused or weighed")
        .contains(Verdict.ACCEPTED, Verdict.TURNING, Verdict.UNTRUSTED_TAGS);
  }

  /** What the session did: a row per loop, image and measurement, and the records behind them. */
  private record Session(List<String> rows, List<VisionResult> images, List<FrameVerdict> frames) {}

  private static Session session() {
    List<String> rows = new ArrayList<>();
    try (DriveSimHarness robot = new DriveSimHarness(ExampleRobot.SIM, SensorErrors.TYPICAL)) {
      robot.simulation.place(START);
      robot.localization.resetPose(START);
      List<CameraConfig> mounted = ExampleRobot.SIM.definition().cameras();
      List<FakeTagCamera> cameras = new ArrayList<>();
      for (int i = 0; i < mounted.size(); i++) {
        cameras.add(
            new FakeTagCamera(robot.simulation, mounted.get(i), RobotSim.FIELD, 0.01, 0.04, 20, i));
      }
      robot.watchWith(cameras);
      // A pit that turned the turning limit down, so a spin refuses images.
      robot.service.vision(
          VisionConfigBuilder.builder(VisionConfig.DEFAULTS).maxYawRateRadiansPerSecond(2).build());
      // Times from the session's start: the simulator's clock starts wherever the tests before
      // left it.
      long start = RobotController.getTime();
      int[] loop = {0};
      int[] seen = {0, 0};
      robot.eachLoop(
          () -> {
            rows.add(loopRow(++loop[0], robot.estimator.estimate()));
            for (; seen[0] < robot.images.size(); seen[0]++) {
              rows.add(imageRow(robot.images.get(seen[0]), start));
            }
            for (; seen[1] < robot.frames.size(); seen[1]++) {
              FrameVerdict frame = robot.frames.get(seen[1]);
              // An image refused before it became a measurement has its row above.
              frame.measured().ifPresent(weighed -> rows.add(frameRow(frame, weighed, start)));
            }
          });

      robot.idle(0.5); // standing: the hub's tags give the heading too
      robot.drive(1.0, -0.3, 0.2, 0); // driving: a moving robot's images count for less
      robot.trustOnly(TagAllowlist.of("Hub", List.of(25, 26))); // a task's tags alone
      robot.drive(0.8, 0.3, -0.2, 0);
      robot.trustOnly(null);
      robot.service.mountDoubt("front-left", 0.08, 3.0); // a knocked camera, widened
      robot.simulation.tilt(Math.toRadians(-8), Math.toRadians(3)); // solved as tilted
      robot.drive(0.6, 0.2, 0, 0);
      robot.simulation.tilt(0, 0);
      robot.drive(0.6, 0, 0, 1.0); // spinning: images refused for turning
      robot.idle(0.5);
      return new Session(rows, List.copyOf(robot.images), List.copyOf(robot.frames));
    }
  }

  private static String loopRow(int loop, LocalizationEstimate estimate) {
    return String.format(
        Locale.ROOT,
        "L,%d,%.9f,%.9f,%.9f,%s,%d,%d",
        loop,
        estimate.pose().getX(),
        estimate.pose().getY(),
        estimate.pose().getRotation().getRadians(),
        estimate.mode(),
        estimate.health().accepted(),
        estimate.health().rejected());
  }

  /** An image as solved: how, or why not, in the log's words, and the measurement it made. */
  private static String imageRow(VisionResult image, long start) {
    VisionMeasurement measurement = image.measurement();
    Verdict refused = image.refused();
    String solved = measurement != null ? measurement.solver().name() : String.valueOf(refused);
    // The outcome's own commas would split it into columns.
    String outcome = image.outcome().replace(',', ';');
    String head =
        String.format(
            Locale.ROOT,
            "I,%s,%d,%s,%s",
            image.detections().camera(),
            image.detections().timestampNanos() - start,
            solved,
            outcome);
    if (measurement == null) {
      return head;
    }
    Pose2d pose = measurement.pose();
    PoseCovariance spread = measurement.covariance();
    return head
        + String.format(
            Locale.ROOT,
            ",%.9f,%.9f,%.9f,%.9f,%.9f,%.9f",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians(),
            spread.xx(),
            spread.yy(),
            spread.heading());
  }

  /** A measurement as the estimator weighed it. */
  private static String frameRow(FrameVerdict frame, FrameVerdict.Measured measured, long start) {
    return String.format(
        Locale.ROOT,
        "F,%s,%d,%s,%s,%.9f,%.9f,%.9f,%s",
        frame.camera(),
        frame.captureNanos() - start,
        frame.verdict(),
        measured.solver(),
        measured.innovationMeters(),
        measured.chiSquared(),
        frame.threshold(),
        frame.tags().stream()
            .map(tag -> String.valueOf(tag.tag()))
            .collect(Collectors.joining(" ")));
  }
}

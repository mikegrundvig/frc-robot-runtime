package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static com.michaelgrundvig.frc.simulation.KnownGap.Check.stillOpen;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.simulation.Consistency;
import com.michaelgrundvig.frc.simulation.KnownGap;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.world.field.Tag;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.RobotController;

/**
 * The truth suite's cameras: what the simulated cameras see from a known pose, against a pinhole
 * camera written here, and when they see it, against their configuration. With the noise off
 * ({@link VisionSim.Noise#NONE}), each tag's corners land where the pinhole puts them, each tag's
 * own solve is where the tag truly is, and a tag is seen just inside the image's edge and not just
 * outside it. The poses are surveyed by hand, level and tilted as on a ramp, where the template's
 * cameras see tags on the simulator's field.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class CameraTruthSimTest {
  /** An AprilTag's black square, 36h11 as FRC prints it: 6.5 in. */
  private static final double TAG_SIZE = Units.inchesToMeters(6.5);

  /** Where the robot stands: level, turned, and tilted on a ramp. */
  private static final List<Pose3d> SURVEYED =
      List.of(
          new Pose3d(2.0, 4.0, 0, Rotation3d.ZERO),
          new Pose3d(3.0, 2.5, 0, new Rotation3d(0, 0, Math.toRadians(-30))),
          new Pose3d(
              2.5,
              5.5,
              0.12,
              new Rotation3d(Math.toRadians(4), Math.toRadians(-12), Math.toRadians(20))));

  private static final List<CameraConfig> CAMERAS = ExampleRobot.SIM.definition().cameras();

  /**
   * Every tag corner every camera sees lands within 0.01 px of the pinhole's: the camera's pose
   * from the robot's and its mount, the tag's corners from its pose and size, projected through
   * intrinsics from the configured resolution and diagonal field of view. The field of view spans
   * the image from its first pixel's center to its last's, around a principal point at its middle,
   * as PhotonVision lays out a camera given only those.
   */
  @Test
  void exactCornersLandWhereAPinholeCameraPutsThem() {
    int corners = 0;
    for (Pose3d robot : SURVEYED) {
      for (Seen seen : render(robot)) {
        Pinhole pinhole = new Pinhole(seen.config(), robot);
        double[][] expected = pinhole.corners(seen.tag());
        for (TargetCorner corner : seen.target().getDetectedCorners()) {
          double nearest = Double.POSITIVE_INFINITY;
          for (double[] point : expected) {
            nearest = Math.min(nearest, Math.hypot(corner.x - point[0], corner.y - point[1]));
          }
          assertThat(nearest)
              .as("tag %d from %s, %s", seen.target().getFiducialId(), robot, seen.config().name())
              .isLessThan(0.01);
          corners++;
        }
      }
    }
    assertThat(corners).as("corners seen").isGreaterThan(40);
  }

  /** Each tag's own solve, from its four exact corners, is where the tag is: 1 mm and 0.05°. */
  @Test
  void eachTagsSolveIsWhereTheTagIs() {
    int tags = 0;
    for (Pose3d robot : SURVEYED) {
      for (Seen seen : render(robot)) {
        Pose3d camera = robot.transformBy(seen.config().robotToCamera());
        Transform3d truth = new Transform3d(camera, seen.tag());
        Transform3d solved = seen.target().getBestCameraToTarget();
        String what = "tag " + seen.target().getFiducialId() + " from " + seen.config().name();

        assertThat(solved.getTranslation().getDistance(truth.getTranslation()))
            .as(what)
            .isLessThan(0.001);
        double turned =
            Math.toDegrees(solved.getRotation().rotateBy(truth.getRotation().inverse()).getAngle());
        assertThat(Math.min(turned, 360 - turned)).as(what).isLessThan(0.05);
        tags++;
      }
    }
    assertThat(tags).as("tags seen").isGreaterThan(10);
  }

  /**
   * The robot turned clockwise until a tag's outermost corner reaches the image's edge, where the
   * pinhole says, the camera sees the tag a hundredth of a degree short of it and not a hundredth
   * past it: every tag every camera sees from the first pose, each its own way out.
   */
  @Test
  void aTagIsSeenJustInsideTheImageAndNotJustOutside() {
    Pose3d robot = SURVEYED.get(0);
    double step = Math.toRadians(0.01);
    List<Seen> all = render(robot);
    for (Seen seen : all) {
      int id = seen.target().getFiducialId();
      double edge = turnToTheEdge(seen.config(), robot, seen.tag());
      String what = "tag " + id + " from " + seen.config().name();

      assertThat(sees(seen.config(), turned(robot, edge + step), id))
          .as("%s, just inside", what)
          .isTrue();
      assertThat(sees(seen.config(), turned(robot, edge - step), id))
          .as("%s, just outside", what)
          .isFalse();
    }
    assertThat(all).isNotEmpty();
  }

  /**
   * Over 20 s, a camera's images come at its configured frame rate, within 1%. Today PhotonVision's
   * simulation holds a frame back behind any image whose latency runs past the frame time (a third
   * of them at 30 ± 5 ms against 33 ms), so a camera configured at 30 Hz sends about 29.3.
   */
  @Test
  @KnownGap("PhotonVision's simulation delays the next frame behind an image's latency")
  void imagesComeAtTheConfiguredFrameRate() {
    CameraConfig config = CAMERAS.get(0);
    // The first image is left out: PhotonVision times a camera's first frame from when it was
    // made, on a clock the simulator's own may since have passed.
    List<PhotonPipelineResult> all = run(config, SURVEYED.get(0), 21);
    List<PhotonPipelineResult> results = all.subList(1, all.size());
    double first = results.get(0).metadata.captureTimestampNanos / 1e9;
    double last = results.get(results.size() - 1).metadata.captureTimestampNanos / 1e9;
    double rate = (results.size() - 1) / (last - first);

    double configured = config.sensor().frameRate().in(Hertz);
    stillOpen("frames a second", rate, configured, 0.01, 29.33, 0.15);
  }

  /**
   * Each image arrives its latency after it was taken: over 20 s, a mean within 0.5 ms of the
   * configured latency, and a spread within 10% of the configured one ({@link
   * VisionSim#LATENCY_STD_DEV_MILLIS}). Consistency, not truth: {@link VisionSim} stamps each image
   * the latency PhotonVision drew for it before it arrives, so the stamps' gap is that draw, and
   * this checks only that the draws are as configured. That the stamp is when the image was taken
   * is {@code CameraTimingSimTest}'s check, against where each image shows the robot.
   */
  @Test
  @Consistency("the stamps' gap is VisionSim's own subtraction of the latency PhotonVision drew")
  void imagesArriveTheirConfiguredLatencyAfterTheyWereTaken() {
    CameraConfig config = CAMERAS.get(0);
    List<PhotonPipelineResult> results = run(config, SURVEYED.get(0), 20);
    double sum = 0;
    double sumSquares = 0;
    for (PhotonPipelineResult result : results) {
      double millis =
          (result.metadata.publishTimestampNanos - result.metadata.captureTimestampNanos) / 1e6;
      sum += millis;
      sumSquares += millis * millis;
    }
    int n = results.size();
    double mean = sum / n;
    double spread = Math.sqrt(sumSquares / n - mean * mean);

    assertThat(n).isGreaterThan(500);
    assertThat(mean).isCloseTo(config.sensor().latency().in(Milliseconds), within(0.5));
    assertThat(spread)
        .isCloseTo(
            VisionSim.LATENCY_STD_DEV_MILLIS, within(0.1 * VisionSim.LATENCY_STD_DEV_MILLIS));
  }

  /**
   * A camera's images are its own, and stamped the same whatever ran before it. A test run builds
   * one robot after another in one process, where they share NetworkTables, and a camera made with
   * the name of an earlier one must not find that one's last image waiting for it. Such an image
   * would be the new camera's first, stamped on the earlier robot's clock, which is a different
   * time ago on every run: between tests the simulator's clock runs on the computer's.
   */
  @Test
  void aCamerasFirstImageIsNotAnEarlierCamerasLast() {
    CameraConfig config = CAMERAS.get(0);
    List<Long> first = stampsSinceItWasMade(config);
    // Some time passes between one robot and the next, and not a whole number of loops.
    SimHooks.stepTiming(0.7031);
    List<Long> second = stampsSinceItWasMade(config);

    assertThat(second).as("the same images, as long after the camera was made").isEqualTo(first);
  }

  /**
   * When each image a new camera sends over 0.2 s was taken and when it arrived, in nanoseconds
   * since the camera was made: an image's two times, then the next image's.
   */
  private static List<Long> stampsSinceItWasMade(CameraConfig config) {
    // The clock stands still until it is stepped, so this is when the camera is made.
    long made = RobotController.getMonotonicTime();
    List<Long> stamps = new ArrayList<>();
    for (PhotonPipelineResult image : run(config, SURVEYED.get(0), 0.2)) {
      assertThat(image.metadata.publishTimestampNanos)
          .as("an image arrives after its camera was made")
          .isGreaterThanOrEqualTo(made);
      stamps.add(image.metadata.captureTimestampNanos - made);
      stamps.add(image.metadata.publishTimestampNanos - made);
    }
    return stamps;
  }

  /** A tag one camera saw, as it truly sits, and what the camera made of it. */
  private record Seen(CameraConfig config, Pose3d tag, PhotonTrackedTarget target) {}

  /** What every camera sees from here, noise off, over half a second: each tag once. */
  private static List<Seen> render(Pose3d robot) {
    List<Seen> seen = new ArrayList<>();
    Map<Integer, Tag> tags = RobotSim.FIELD.tags();
    for (CameraConfig config : CAMERAS) {
      List<PhotonPipelineResult> results = run(config, robot, 0.5, VisionSim.Noise.NONE);
      for (PhotonPipelineResult result : results.subList(results.size() - 1, results.size())) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          seen.add(
              new Seen(
                  config, Objects.requireNonNull(tags.get(target.getFiducialId())).pose(), target));
        }
      }
    }
    return seen;
  }

  /** Whether this camera sees this tag from here, noise off. */
  private static boolean sees(CameraConfig config, Pose3d robot, int id) {
    List<PhotonPipelineResult> results = run(config, robot, 0.2, VisionSim.Noise.NONE);
    return results.get(results.size() - 1).getTargets().stream()
        .anyMatch(target -> target.getFiducialId() == id);
  }

  private static List<PhotonPipelineResult> run(CameraConfig config, Pose3d robot, double seconds) {
    return run(config, robot, seconds, VisionSim.Noise.TYPICAL);
  }

  /** Every image one camera sends over this long, the robot standing here. */
  private static List<PhotonPipelineResult> run(
      CameraConfig config, Pose3d robot, double seconds, VisionSim.Noise noise) {
    List<PhotonPipelineResult> results = new ArrayList<>();
    try (VisionSim vision =
        new VisionSim(RobotSim.FIELD, Map.of(), camera -> Seeds.camera(1, camera), noise)) {
      PhotonCamera camera = ((CameraIOPhotonVision) vision.camera(config)).camera();
      for (int loop = 0; loop < Math.round(seconds / 0.02); loop++) {
        RobotSim.stepClock();
        vision.update(robot);
        results.addAll(camera.getAllUnreadResults());
      }
      camera.close();
    }
    assertThat(results).as("images from %s", config.name()).isNotEmpty();
    return results;
  }

  private static Pose3d turned(Pose3d robot, double yaw) {
    return new Pose3d(
        robot.getTranslation(), robot.getRotation().rotateBy(new Rotation3d(0, 0, yaw)));
  }

  /**
   * How far to turn the robot about its center (clockwise: negative) to bring this tag's outermost
   * corner to the image's edge, by the pinhole: found by halving between where every corner is in
   * the image and a quarter turn, where the tag is out of it.
   */
  private static double turnToTheEdge(CameraConfig config, Pose3d robot, Pose3d tag) {
    double inside = 0;
    double outside = -Math.PI / 2;
    for (int i = 0; i < 60; i++) {
      double yaw = (inside + outside) / 2;
      if (new Pinhole(config, turned(robot, yaw)).inTheImage(tag)) {
        inside = yaw;
      } else {
        outside = yaw;
      }
    }
    return (inside + outside) / 2;
  }

  /** A perfect pinhole camera, as configured, mounted on the robot where it stands. */
  private static final class Pinhole {
    private final Pose3d camera;
    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;

    private final double width;
    private final double height;

    Pinhole(CameraConfig config, Pose3d robot) {
      camera = robot.transformBy(config.robotToCamera());
      CameraConfig.Sensor sensor = config.sensor();
      width = sensor.widthPixels();
      height = sensor.heightPixels();
      double halfDiagonal = Math.tan(sensor.diagonalFieldOfView().in(Degrees) * Math.PI / 360);
      double diagonal = Math.hypot(width, height);
      cx = (width - 1) / 2;
      cy = (height - 1) / 2;
      fx = cx / (halfDiagonal * width / diagonal);
      fy = cy / (halfDiagonal * height / diagonal);
    }

    /**
     * Whether all a tag's corners are in front of the lens and in the image: between 0 and its
     * width and height in pixels, as PhotonVision's simulation counts it.
     */
    boolean inTheImage(Pose3d tag) {
      for (double[] corner : corners(tag)) {
        if (!(corner[2] > 0
            && corner[0] >= 0
            && corner[0] <= width
            && corner[1] >= 0
            && corner[1] <= height)) {
          return false;
        }
      }
      return true;
    }

    /**
     * A tag's four corners in the image, in pixels, x right and y down, each with how far in front
     * of the lens it is, in meters.
     */
    double[][] corners(Pose3d tag) {
      double[][] corners = new double[4][];
      int i = 0;
      for (double left : new double[] {-TAG_SIZE / 2, TAG_SIZE / 2}) {
        for (double up : new double[] {-TAG_SIZE / 2, TAG_SIZE / 2}) {
          Translation3d onTheField =
              tag.transformBy(new Transform3d(new Translation3d(0, left, up), Rotation3d.ZERO))
                  .getTranslation();
          // In the camera's frame: x out of the lens, y left, z up.
          Translation3d seen =
              new Pose3d(onTheField, Rotation3d.ZERO).relativeTo(camera).getTranslation();
          corners[i++] =
              new double[] {
                cx - fx * seen.getY() / seen.getX(),
                cy - fy * seen.getZ() / seen.getX(),
                seen.getX()
              };
        }
      }
      return corners;
    }
  }
}

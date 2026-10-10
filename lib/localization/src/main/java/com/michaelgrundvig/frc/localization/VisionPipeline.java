package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Turns one camera image's AprilTags into a measurement for the estimator, or says why not.
 *
 * <p>How it solves, in order of preference:
 *
 * <ol>
 *   <li><b>Heading-constrained</b>, whenever the estimate is sure of the heading: each tag fixes
 *       only the position, using the robot's heading from the gyro. Of the two poses a single tag
 *       allows, the one whose heading agrees with the gyro is used, which removes single-tag
 *       ambiguity, and each tag's known height checks the view. The idea behind Limelight's
 *       MegaTag2 and PhotonVision's constrained solve. Such a position leans on the gyro's heading,
 *       so the measurement says how far its tags were ({@link VisionMeasurement#lever}): the
 *       estimator carries the heading's doubt in it.
 *   <li><b>Multi-tag</b>, the vision software's solution from every tag at once, when the heading
 *       isn't known yet (a robot not yet placed).
 *   <li><b>Single tag</b>, the least ambiguous one, only if its ambiguity is low.
 * </ol>
 *
 * <p>Uncertainty grows with the tags' distance (squared, by default), shrinks with their count, and
 * grows with the robot's speed and turning (blur). A camera whose mount is in doubt (its tags land
 * off the same way every time: {@link MountCheck}) adds that doubt, as independent errors add. A
 * result may correct heading only from a multi-tag solution of several tags, with the robot barely
 * turning and the solution fitting well; otherwise the gyro keeps heading to itself. Everything
 * thrown out says why, as a {@link Verdict}.
 *
 * <p>A task may trust only some tags ({@link TagAllowlist}): the image is solved from those alone,
 * and the vision software's multi-tag solution only if it used no others.
 *
 * <p>Pure logic: the field's layout, the camera's mount, and the robot at the capture time all come
 * in as arguments.
 */
public final class VisionPipeline {
  private final FieldModel field;

  public VisionPipeline(FieldModel field) {
    this.field = field;
  }

  /**
   * How far to trust one camera's image beyond its configuration, for the moment.
   *
   * @param mountDoubtMeters how far the camera's measurements may be off from where its mount moved
   *     since it was calibrated ({@link MountCheck#doubtMeters}): zero for a camera that sits where
   *     its configuration says
   * @param onlyTags the tags the current task trusts; null for every tag
   */
  public record Trust(double mountDoubtMeters, @Nullable TagAllowlist onlyTags) {
    /** A camera where its configuration says, and every tag. */
    public static final Trust FULL = new Trust(0, null);

    public Trust {
      if (!(mountDoubtMeters >= 0) || Double.isInfinite(mountDoubtMeters)) {
        throw new IllegalArgumentException("a mount's doubt is zero or more: " + mountDoubtMeters);
      }
    }
  }

  /** One image, and what it's solved with: its camera's mount on the robot as it was taken. */
  private record Image(
      TagDetections seen,
      CameraConfig camera,
      Transform3d mount,
      CaptureContext robot,
      VisionConfig config,
      double mountDoubtMeters) {
    /**
     * How much a result's uncertainty grows from the one-tag-at-one-meter baseline: with distance,
     * fewer tags, the camera's own noise, and the robot's motion.
     */
    double scale(int tags, double distance) {
      double motion =
          1
              + config.speedStdDevFactor() * robot.speedMetersPerSecond()
              + config.yawRateStdDevFactor() * robot.recentYawRateRadiansPerSecond();
      return Math.pow(distance, config.distanceExponent()) / tags * camera.noiseScale() * motion;
    }
  }

  /** What to make of one image from a camera on the chassis, where its configuration mounts it. */
  public VisionResult process(
      TagDetections seen,
      CameraConfig camera,
      CaptureContext robot,
      VisionConfig config,
      Trust trust) {
    return process(seen, camera, camera.robotToCamera(), robot, config, trust);
  }

  /**
   * What to make of one image, its camera where it was on the robot as the image was taken: on a
   * joint, where the joint had carried it.
   *
   * @param mount the camera's pose in the robot's frame as the image was taken
   */
  public VisionResult process(
      TagDetections seen,
      CameraConfig camera,
      Transform3d mount,
      CaptureContext robot,
      VisionConfig config,
      Trust trust) {
    TagAllowlist onlyTags = trust.onlyTags();
    if (onlyTags == null) {
      return solve(new Image(seen, camera, mount, robot, config, trust.mountDoubtMeters()));
    }
    TagDetections trusted = trusted(seen, onlyTags);
    if (trusted.tags().isEmpty() && !seen.tags().isEmpty()) {
      return new VisionResult(
          seen,
          null,
          Verdict.UNTRUSTED_TAGS,
          Verdict.UNTRUSTED_TAGS.words() + " (" + onlyTags.describe() + ")",
          Double.NaN,
          Double.NaN,
          Double.NaN,
          Double.NaN);
    }
    return solve(new Image(trusted, camera, mount, robot, config, trust.mountDoubtMeters()));
  }

  /**
   * An image as a task sees it: only the tags it trusts, and the vision software's multi-tag
   * solution only if that used no others.
   */
  private static TagDetections trusted(TagDetections seen, TagAllowlist onlyTags) {
    List<TagDetections.Tag> tags =
        seen.tags().stream().filter(tag -> onlyTags.allows(tag.id())).toList();
    TagDetections.MultiTag multiTag = seen.multiTag();
    boolean allTrusted = multiTag != null && multiTag.tagIds().stream().allMatch(onlyTags::allows);
    return new TagDetections(
        seen.timestampNanos(), seen.camera(), tags, allTrusted ? multiTag : null);
  }

  private VisionResult solve(Image image) {
    TagDetections seen = image.seen();
    CaptureContext robot = image.robot();
    VisionConfig config = image.config();
    List<TagDetections.Tag> known = new ArrayList<>();
    double distance = 0;
    for (TagDetections.Tag tag : seen.tags()) {
      if (field.tag(tag.id()).isPresent()) {
        known.add(tag);
        distance += tag.bestCameraToTag().getTranslation().getNorm();
      }
    }
    if (known.isEmpty()) {
      return rejected(seen, Verdict.UNKNOWN_TAGS, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }
    distance /= known.size();
    if (!Double.isFinite(distance)) {
      return rejected(
          seen, Verdict.TAG_NOT_A_NUMBER, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }
    if (robot.recentYawRateRadiansPerSecond() > config.maxYawRateRadiansPerSecond()) {
      return rejected(
          seen,
          Verdict.TURNING,
          robot.recentYawRateRadiansPerSecond(),
          config.maxYawRateRadiansPerSecond(),
          distance,
          Double.NaN);
    }
    if (distance > config.maxDistanceMeters()) {
      return rejected(
          seen, Verdict.TOO_FAR, distance, config.maxDistanceMeters(), distance, Double.NaN);
    }
    Rotation2d heading = robot.heading();
    if (heading != null && robot.headingStdDevRadians() <= config.maxHeadingStdDevToConstrain()) {
      return headingConstrained(image, known, heading, distance);
    }
    TagDetections.MultiTag multiTag = seen.multiTag();
    if (multiTag != null) {
      Pose3d pose =
          TagSolver.robotFromCamera(
              new Pose3d(
                  multiTag.fieldToCamera().getTranslation(),
                  multiTag.fieldToCamera().getRotation()),
              image.mount());
      return fullPose(image, pose, known, distance, VisionMeasurement.Solver.MULTI_TAG);
    }
    TagDetections.Tag clearest = known.get(0);
    for (TagDetections.Tag tag : known) {
      if (tag.ambiguity() < clearest.ambiguity()) {
        clearest = tag;
      }
    }
    if (clearest.ambiguity() > config.maxAmbiguity()) {
      return rejected(
          seen,
          Verdict.AMBIGUOUS,
          clearest.ambiguity(),
          config.maxAmbiguity(),
          distance,
          Double.NaN);
    }
    Pose3d pose =
        TagSolver.robotFromTag(
            field.tag(clearest.id()).orElseThrow(), clearest.bestCameraToTag(), image.mount());
    return fullPose(
        image,
        pose,
        List.of(clearest),
        clearest.bestCameraToTag().getTranslation().getNorm(),
        VisionMeasurement.Solver.SINGLE_TAG);
  }

  private VisionResult headingConstrained(
      Image image, List<TagDetections.Tag> known, Rotation2d heading, double distance) {
    TagDetections seen = image.seen();
    CameraConfig camera = image.camera();
    CaptureContext robot = image.robot();
    VisionConfig config = image.config();
    Transform3d robotToCamera = image.mount();
    Rotation3d attitude =
        new Rotation3d(robot.rollRadians(), robot.pitchRadians(), heading.getRadians());
    double sumX = 0;
    double sumY = 0;
    double sumToTagX = 0;
    double sumToTagY = 0;
    double sumWeights = 0;
    double worstHeight = 0;
    List<Double> weights = new ArrayList<>(known.size());
    for (TagDetections.Tag tag : known) {
      Pose3d fieldToTag = field.tag(tag.id()).orElseThrow();
      Transform3d cameraToTag = agreeingWithTheGyro(tag, fieldToTag, robotToCamera, heading);
      TagSolver.PositionFix fix =
          TagSolver.positionFromTag(fieldToTag, robotToCamera.plus(cameraToTag), attitude);
      double tagDistance = cameraToTag.getTranslation().getNorm();
      double weight = 1 / Math.max(tagDistance * tagDistance, 1e-6);
      sumX += weight * fix.position().getX();
      sumY += weight * fix.position().getY();
      // What the heading turned to place the robot: from where it puts the robot, to the tag.
      sumToTagX += weight * (fieldToTag.getX() - fix.position().getX());
      sumToTagY += weight * (fieldToTag.getY() - fix.position().getY());
      sumWeights += weight;
      weights.add(weight);
      if (Math.abs(fix.heightErrorMeters()) > Math.abs(worstHeight)) {
        worstHeight = fix.heightErrorMeters();
      }
    }
    if (!(Math.abs(worstHeight) <= config.maxHeightErrorMeters())) {
      return rejected(
          seen,
          Verdict.TAG_HEIGHT,
          worstHeight,
          config.maxHeightErrorMeters(),
          distance,
          worstHeight);
    }
    Translation2d position = new Translation2d(sumX / sumWeights, sumY / sumWeights);
    if (!Double.isFinite(position.getX()) || !Double.isFinite(position.getY())) {
      return rejected(
          seen, Verdict.SOLVE_NOT_A_NUMBER, Double.NaN, Double.NaN, distance, worstHeight);
    }
    double spread =
        widened(
            config.linearStdDevBaseline()
                * config.headingConstrainedFactor()
                * image.scale(known.size(), distance),
            image.mountDoubtMeters());

    // Heading, too, only under strict conditions; otherwise the gyro keeps it to itself.
    Rotation2d measuredHeading = heading;
    double headingStdDev = Double.POSITIVE_INFINITY;
    TagDetections.MultiTag multiTag = seen.multiTag();
    if (multiTag != null
        && multiTag.tagIds().size() >= config.minTagsForHeading()
        && robot.recentYawRateRadiansPerSecond() <= config.maxYawRateForHeading()
        && multiTag.reprojectionErrorPixels() <= config.maxReprojectionErrorForHeadingPixels()) {
      Pose3d full =
          TagSolver.robotFromCamera(
              new Pose3d(
                  multiTag.fieldToCamera().getTranslation(),
                  multiTag.fieldToCamera().getRotation()),
              robotToCamera);
      Rotation2d solved = full.toPose2d().getRotation();
      if (Double.isFinite(solved.getRadians())) {
        measuredHeading = solved;
        headingStdDev =
            widened(
                config.angularStdDevBaseline() * image.scale(known.size(), distance),
                image.mountDoubtMeters() / distance);
      }
    }
    VisionMeasurement measurement =
        new VisionMeasurement(
            seen.timestampNanos(),
            new Pose2d(position, measuredHeading),
            PoseCovariance.fromStdDevs(spread, spread, headingStdDev),
            new VisionMeasurement.Lever(
                heading, new Translation2d(sumToTagX / sumWeights, sumToTagY / sumWeights)),
            camera.name(),
            ids(known),
            weights,
            VisionMeasurement.Solver.HEADING_CONSTRAINED);
    return new VisionResult(
        seen,
        measurement,
        null,
        format(
            "heading-constrained: %d %s at %.1f m%s",
            known.size(),
            known.size() == 1 ? "tag" : "tags",
            distance,
            Double.isFinite(headingStdDev) ? ", with heading" : ""),
        distance,
        worstHeight,
        Double.NaN,
        Double.NaN);
  }

  /** Of a tag's two possible poses, the one whose robot heading is closer to the gyro's. */
  private static Transform3d agreeingWithTheGyro(
      TagDetections.Tag tag, Pose3d fieldToTag, Transform3d robotToCamera, Rotation2d heading) {
    Transform3d best = tag.bestCameraToTag();
    Transform3d alternate = tag.altCameraToTag();
    if (alternate.getTranslation().getNorm() < 1e-6) {
      return best;
    }
    double bestMiss =
        Math.abs(
            TagSolver.robotFromTag(fieldToTag, best, robotToCamera)
                .toPose2d()
                .getRotation()
                .minus(heading)
                .getRadians());
    double alternateMiss =
        Math.abs(
            TagSolver.robotFromTag(fieldToTag, alternate, robotToCamera)
                .toPose2d()
                .getRotation()
                .minus(heading)
                .getRadians());
    return alternateMiss < bestMiss ? alternate : best;
  }

  private VisionResult fullPose(
      Image image,
      Pose3d pose,
      List<TagDetections.Tag> used,
      double distance,
      VisionMeasurement.Solver solver) {
    TagDetections seen = image.seen();
    VisionConfig config = image.config();
    double height = pose.getZ();
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getZ())) {
      return rejected(seen, Verdict.SOLVE_NOT_A_NUMBER, Double.NaN, Double.NaN, distance, height);
    }
    if (!(Math.abs(height) <= config.maxHeightErrorMeters())) {
      return rejected(
          seen, Verdict.OFF_THE_FLOOR, height, config.maxHeightErrorMeters(), distance, height);
    }
    double scale = image.scale(used.size(), distance);
    double linear = widened(config.linearStdDevBaseline() * scale, image.mountDoubtMeters());
    VisionMeasurement measurement =
        new VisionMeasurement(
            seen.timestampNanos(),
            pose.toPose2d(),
            PoseCovariance.fromStdDevs(
                linear,
                linear,
                widened(
                    config.angularStdDevBaseline() * scale, image.mountDoubtMeters() / distance)),
            VisionMeasurement.Lever.NONE,
            image.camera().name(),
            ids(used),
            solver);
    return new VisionResult(
        seen,
        measurement,
        null,
        format(
            "%s: %d %s at %.1f m",
            solver == VisionMeasurement.Solver.MULTI_TAG ? "multi-tag" : "single tag",
            used.size(),
            used.size() == 1 ? "tag" : "tags",
            distance),
        distance,
        height,
        Double.NaN,
        Double.NaN);
  }

  /**
   * A spread with the camera's mount doubt added, as independent errors add: in squares. For
   * heading, a doubt in meters is a turn of the doubt over the tags' distance: a camera turned by
   * an angle puts tags that far off at that distance.
   *
   * <p>Only widened, never shut off: a heading error in the estimate puts every camera's tags off
   * as if each were turned on its mount, and from one camera the two look the same. A camera in
   * doubt that couldn't correct heading at all would keep the estimate from ever turning back.
   *
   * <p>A wider spread also loosens the estimator's gate for the measurement, which weighs how far
   * it lands from the estimate against both their spreads: a doubted camera's measurements are
   * rejected less often, and count for less when they're accepted.
   */
  private static double widened(double spread, double doubt) {
    return doubt > 0 ? Math.hypot(spread, doubt) : spread;
  }

  private static List<Integer> ids(List<TagDetections.Tag> tags) {
    return tags.stream().map(TagDetections.Tag::id).toList();
  }

  /**
   * An image thrown out, and why: the verdict, with its number and the limit it was held to in its
   * words for the log.
   */
  private static VisionResult rejected(
      TagDetections seen,
      Verdict why,
      double value,
      double threshold,
      double distance,
      double heightError) {
    return new VisionResult(
        seen, null, why, why.describe(value, threshold), distance, heightError, threshold, value);
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}

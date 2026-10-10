package com.michaelgrundvig.frc.runtime.views.world;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.TagVerdict;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Shapes;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

/**
 * Each camera's sight lines to the tags its measurement used: solid where localization accepted the
 * tag's measurement, dashed where it refused it. A click on one says why, from the tag's verdict
 * itself ({@link TagVerdict}), and offers the camera's calibration. A line stays a moment after its
 * measurement, so it doesn't flicker; the robot keeps what a click says for two seconds after it's
 * gone, and the page's Hold keeps it drawn.
 */
final class SightLines implements Layer {
  /** How long a line stays after its camera's last measurement of its tag. */
  private static final long HOLD_NANOS = 300_000_000L;

  /**
   * One camera's sight line to one tag, with one verdict, made once: its id, how it's drawn, and
   * what a click on it says, from its latest measurement. A click reads that on a web server
   * thread, so it's records the loop hands over whole.
   */
  private final class Line {
    final String id;
    final String camera;
    final String cameraFrame;
    final int tag;
    final String tagFrame;
    final Style style;

    /** What a click says: the same supplier while the line stands, so Pitwall keeps it as is. */
    final Supplier<Inspection> inspection = () -> inspection(this);

    /** The latest image whose measurement went through this line's tag with its verdict. */
    volatile @Nullable FrameVerdict frame;

    Line(CameraConfig camera, int tag, boolean accepted) {
      id = "ray." + camera.id() + "." + tag + (accepted ? ".ok" : ".bad");
      this.camera = camera.name();
      cameraFrame = WorldView.cameraFrame(camera.id());
      this.tag = tag;
      tagFrame = WorldView.tagFrame(tag);
      style = accepted ? Style.OK : Style.BAD.dashed();
    }
  }

  /**
   * One camera's sight to one tag: its two lines, which is drawn, and since its last measurement.
   */
  private static final class Sight {
    final Line accepted;
    final Line rejected;
    @Nullable Line drawn;
    long decidedNanos;

    Sight(Line accepted, Line rejected) {
      this.accepted = accepted;
      this.rejected = rejected;
    }
  }

  private final Supplier<FieldModel> field;

  /** The cameras, by name: what their measurements say. */
  private final Map<String, CameraConfig> cameras;

  private final WorldView.Readings readings;
  private final Function<String, Optional<String>> cameraAction;

  /** Each camera's sights, by tag; null for a tag the field doesn't have. Made as first seen. */
  private final Map<String, Map<Integer, @Nullable Sight>> sights = new HashMap<>();

  /** Every sight, in the order first seen: the order the lines are drawn in. */
  private final List<Sight> seen = new ArrayList<>();

  private List<FrameVerdict> lastFrames = List.of();

  SightLines(
      Supplier<FieldModel> field,
      List<CameraConfig> cameras,
      WorldView.Readings readings,
      Function<String, Optional<String>> cameraAction) {
    this.field = field;
    this.cameras =
        cameras.stream()
            .collect(Collectors.toUnmodifiableMap(CameraConfig::name, camera -> camera));
    this.readings = readings;
    this.cameraAction = cameraAction;
  }

  @Override
  public String id() {
    return "sight-lines";
  }

  @Override
  public String title() {
    return "Sight lines (dashed: rejected)";
  }

  /** Its lines run between the robot's cameras and the field's tags: it adds no frames. */
  @Override
  public void describe(SceneBuilder scene) {}

  /**
   * Takes the loop's verdicts into the lines they're drawn as, and draws the lines again only when
   * which there are changes. Every loop while the World is shown, so nothing here is made but a
   * sight seen for the first time.
   */
  @Override
  public void update(SceneState state) {
    long now = state.nowNanos();
    boolean changed = false;
    List<FrameVerdict> frames = readings.frames();
    if (frames != lastFrames) {
      lastFrames = frames;
      for (int i = 0; i < frames.size(); i++) {
        FrameVerdict frame = frames.get(i);
        CameraConfig camera = cameras.get(frame.camera());
        // Lines are drawn to the tags a measurement rested on: an image refused before it became
        // one has none.
        if (camera == null || frame.measured().isEmpty()) {
          continue;
        }
        List<TagVerdict> tags = frame.tags();
        for (int t = 0; t < tags.size(); t++) {
          TagVerdict verdict = tags.get(t);
          Sight sight = sight(camera, verdict.tag());
          if (sight == null) {
            continue;
          }
          Line line = verdict.accepted() ? sight.accepted : sight.rejected;
          line.frame = frame;
          changed |= sight.drawn != line;
          sight.drawn = line;
          sight.decidedNanos = now;
        }
      }
    }
    for (int i = 0; i < seen.size(); i++) {
      Sight sight = seen.get(i);
      if (sight.drawn != null && now - sight.decidedNanos > HOLD_NANOS) {
        sight.drawn = null;
        changed = true;
      }
    }
    if (!changed) {
      return;
    }
    Shapes shapes = state.shapes();
    for (Sight sight : seen) {
      Line line = sight.drawn;
      if (line != null) {
        shapes
            .line(line.id, SceneBuilder.ROOT)
            .between(line.cameraFrame, line.tagFrame)
            .style(line.style)
            .inspect(line.inspection);
      }
    }
  }

  /** A camera's sight to a tag, made the first time it's seen; null for a tag not on the field. */
  private @Nullable Sight sight(CameraConfig camera, Integer tag) {
    Map<Integer, @Nullable Sight> byTag =
        sights.computeIfAbsent(camera.name(), name -> new HashMap<>());
    if (!byTag.containsKey(tag)) {
      Sight sight =
          field.get().tag(tag).isPresent()
              ? new Sight(new Line(camera, tag, true), new Line(camera, tag, false))
              : null;
      byTag.put(tag, sight);
      if (sight != null) {
        seen.add(sight);
      }
    }
    return byTag.get(tag);
  }

  /**
   * What a click on a line says, from its image's verdicts alone (a record the loop no longer
   * touches): the tag's verdict and its numbers, called on a web server thread.
   */
  private Inspection inspection(Line line) {
    FrameVerdict frame = line.frame;
    TagVerdict verdict = frame == null ? null : verdictOn(frame, line.tag);
    FrameVerdict.Measured measurement = frame == null ? null : frame.measured().orElse(null);
    if (frame == null || verdict == null || measurement == null) {
      return Inspection.of(line.camera + " → tag " + line.tag).detail("Verdict", "not measured");
    }
    Pose2d measured = measurement.pose();
    Optional<Pose3d> tag = field.get().tag(line.tag);
    Inspection said =
        Inspection.of(line.camera + " → tag " + line.tag)
            .detail("Verdict", verdict.accepted() ? "accepted" : "rejected: " + verdict.reason())
            .detail(
                "Innovation",
                Double.isFinite(verdict.innovationMeters())
                    ? WorldView.offset(verdict.innovationMeters())
                        + " from the estimate, before this measurement moved it"
                    : "not compared")
            .detail(
                "χ²",
                Double.isFinite(verdict.chiSquared())
                    ? WorldView.chiSquared(verdict.chiSquared())
                    : "not compared")
            .detail(
                "Distance",
                tag.isPresent()
                    ? WorldView.meters(
                            measured
                                .getTranslation()
                                .getDistance(tag.get().toPose2d().getTranslation()))
                        + " from the measured pose to the tag"
                    : "the field has no such tag")
            .detail(
                "Measured pose",
                String.format(
                    Locale.ROOT,
                    "x %.2f m, y %.2f m, heading %.1f°",
                    measured.getX(),
                    measured.getY(),
                    measured.getRotation().getDegrees()))
            .detail(
                "Solved",
                measurement.solver().name().toLowerCase(Locale.ROOT).replace('_', ' ')
                    + " from tags "
                    + frame.tags().stream()
                        .map(seen -> String.valueOf(seen.tag()))
                        .collect(Collectors.joining(", ")));
    Optional<String> calibrate = cameraAction.apply(line.camera);
    return calibrate.isPresent() ? said.action(calibrate.get()) : said;
  }

  /** An image's verdict on one of its tags; null if the image didn't use it. */
  private static @Nullable TagVerdict verdictOn(FrameVerdict frame, int tag) {
    for (TagVerdict verdict : frame.tags()) {
      if (verdict.tag() == tag) {
        return verdict;
      }
    }
    return null;
  }
}

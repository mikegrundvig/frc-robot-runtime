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
 * Each camera's sight lines to the tags its measurement used. A line is solid where localization
 * accepted the tag's measurement, and dashed where it refused it. A click on one says why, from the
 * tag's verdict itself ({@link TagVerdict}), and offers the camera's calibration.
 *
 * <p>A line stays a moment after its measurement, so that it doesn't flicker between images. The
 * robot keeps what a click says for two seconds after the line is gone, and the page's Hold keeps
 * it drawn.
 *
 * <p>{@link #update} runs every loop while the World is shown, so it makes nothing new but a sight
 * seen for the first time. That is why a few things here are kept as plain fields that can be null
 * where an {@code Optional} would be made again every loop.
 */
final class SightLines implements Layer {
  /** How long a line stays after its camera's last measurement of its tag. */
  private static final long HOLD_NANOS = 300_000_000L;

  /** Which of a sight's two lines is drawn. */
  private enum Drawn {
    NEITHER,
    ACCEPTED,
    REJECTED
  }

  /**
   * One camera's sight line to one tag, with one verdict. It is made once, with its id, how it's
   * drawn, and what a click on it says, which comes from its latest measurement.
   */
  private final class Line {
    final String id;
    final String camera;
    final String cameraFrame;
    final int tag;
    final String tagFrame;
    final Style style;

    /**
     * What a click says. It is the same supplier for as long as the line stands, so Pitwall keeps
     * it as it is.
     */
    final Supplier<Inspection> inspection = () -> inspection(this);

    /**
     * The latest image whose measurement went through this line's tag with this line's verdict, or
     * null before there is one.
     *
     * <p>A click reads it on a web server thread, so the loop hands the record over whole, through
     * a volatile field. It is a plain reference because the loop sets it for every verdict of every
     * loop.
     */
    volatile @Nullable FrameVerdict frame;

    Line(CameraConfig camera, int tag, Drawn verdict) {
      boolean accepted = verdict == Drawn.ACCEPTED;
      String ending = ".bad";
      if (accepted) {
        ending = ".ok";
      }
      id = "ray." + camera.id() + "." + tag + ending;
      this.camera = camera.name();
      cameraFrame = WorldView.cameraFrame(camera.id());
      this.tag = tag;
      tagFrame = WorldView.tagFrame(tag);
      if (accepted) {
        style = Style.OK;
      } else {
        style = Style.BAD.dashed();
      }
    }
  }

  /** One camera's sight to one tag: its two lines, which of them is drawn, and since when. */
  private static final class Sight {
    final Line accepted;
    final Line rejected;
    Drawn drawn = Drawn.NEITHER;

    /** When the camera's latest measurement of the tag was decided. */
    long decidedNanos;

    Sight(Line accepted, Line rejected) {
      this.accepted = accepted;
      this.rejected = rejected;
    }

    /** The line for a verdict. Ask only for one of the two that are drawn. */
    Line line(Drawn verdict) {
      if (verdict == Drawn.ACCEPTED) {
        return accepted;
      }
      return rejected;
    }
  }

  private final Supplier<FieldModel> field;

  /** The cameras, by name, which is how their measurements name them. */
  private final Map<String, CameraConfig> cameras;

  private final WorldView.Readings readings;
  private final Function<String, Optional<String>> cameraAction;

  /**
   * Each camera's sights, by tag, made as each is first seen. A tag the field doesn't have has no
   * sight, and that is kept too (as an empty), so the field is asked about a tag only once.
   */
  private final Map<String, Map<Integer, Optional<Sight>>> sights = new HashMap<>();

  /** Every sight, in the order first seen, which is the order the lines are drawn in. */
  private final List<Sight> seen = new ArrayList<>();

  /** The verdicts taken last. The loop hands over a new list each time it has new ones. */
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

  /** Its lines run between the robot's cameras and the field's tags, so it adds no frames. */
  @Override
  public void describe(SceneBuilder scene) {}

  /**
   * Takes the loop's verdicts into the lines they're drawn as, lets go of lines held long enough,
   * and draws the lines again only when which ones there are has changed.
   */
  @Override
  public void update(SceneState state) {
    long now = state.nowNanos();
    boolean changed = takeNewVerdicts(now);
    changed |= dropLinesHeldLongEnough(now);
    if (changed) {
      // Asking the state for its shapes starts this layer's shapes over, so it is asked only now.
      draw(state.shapes());
    }
  }

  /**
   * Takes the loop's verdicts, if they are new, into the sights they belong to.
   *
   * @return whether any sight's drawn line changed
   */
  private boolean takeNewVerdicts(long now) {
    List<FrameVerdict> frames = readings.frames();
    if (frames == lastFrames) {
      return false;
    }
    lastFrames = frames;
    boolean changed = false;
    for (int i = 0; i < frames.size(); i++) {
      FrameVerdict frame = frames.get(i);
      // A camera the robot's configuration doesn't have can't be drawn from.
      CameraConfig camera = cameras.get(frame.camera());
      // Lines are drawn to the tags a measurement rested on. An image that was refused before it
      // became a measurement has none.
      if (camera == null || frame.measured().isEmpty()) {
        continue;
      }
      List<TagVerdict> tags = frame.tags();
      for (int t = 0; t < tags.size(); t++) {
        changed |= take(tags.get(t), frame, camera, now);
      }
    }
    return changed;
  }

  /**
   * Takes one tag's verdict into its sight.
   *
   * @return whether the sight's drawn line changed
   */
  private boolean take(TagVerdict verdict, FrameVerdict frame, CameraConfig camera, long now) {
    Optional<Sight> onTheField = sight(camera, verdict.tag());
    if (onTheField.isEmpty()) {
      return false;
    }
    Sight sight = onTheField.get();
    Drawn as = Drawn.REJECTED;
    if (verdict.accepted()) {
      as = Drawn.ACCEPTED;
    }
    sight.line(as).frame = frame;
    boolean changed = sight.drawn != as;
    sight.drawn = as;
    sight.decidedNanos = now;
    return changed;
  }

  /**
   * Stops drawing each line whose camera hasn't measured its tag for a moment.
   *
   * @return whether any line was dropped
   */
  private boolean dropLinesHeldLongEnough(long now) {
    boolean dropped = false;
    for (int i = 0; i < seen.size(); i++) {
      Sight sight = seen.get(i);
      if (sight.drawn != Drawn.NEITHER && now - sight.decidedNanos > HOLD_NANOS) {
        sight.drawn = Drawn.NEITHER;
        dropped = true;
      }
    }
    return dropped;
  }

  private void draw(Shapes shapes) {
    for (Sight sight : seen) {
      if (sight.drawn == Drawn.NEITHER) {
        continue;
      }
      Line line = sight.line(sight.drawn);
      shapes
          .line(line.id, SceneBuilder.ROOT)
          .between(line.cameraFrame, line.tagFrame)
          .style(line.style)
          .inspect(line.inspection);
    }
  }

  /**
   * A camera's sight to a tag, made the first time the pair is seen, or empty for a tag that isn't
   * on the field.
   */
  private Optional<Sight> sight(CameraConfig camera, Integer tag) {
    Map<Integer, Optional<Sight>> byTag =
        sights.computeIfAbsent(camera.name(), name -> new HashMap<>());
    Optional<Sight> known = byTag.get(tag);
    if (known != null) {
      return known;
    }
    Optional<Sight> made = Optional.empty();
    if (field.get().tag(tag).isPresent()) {
      Sight sight =
          new Sight(new Line(camera, tag, Drawn.ACCEPTED), new Line(camera, tag, Drawn.REJECTED));
      seen.add(sight);
      made = Optional.of(sight);
    }
    byTag.put(tag, made);
    return made;
  }

  // ---- What a click says ----

  /**
   * What a click on a line says: the tag's verdict and its numbers. It is called on a web server
   * thread, and reads only its image's verdicts, a record the loop no longer touches.
   */
  private Inspection inspection(Line line) {
    FrameVerdict frame = line.frame;
    if (frame == null) {
      return notMeasured(line);
    }
    Optional<TagVerdict> found = verdictOn(frame, line.tag);
    Optional<FrameVerdict.Measured> measurement = frame.measured();
    if (found.isEmpty() || measurement.isEmpty()) {
      return notMeasured(line);
    }
    TagVerdict verdict = found.get();
    Pose2d measured = measurement.get().pose();
    // Pitwall's own Javadoc shows the first lines and the last line of this as its example of an
    // inspection (Inspection), and the build checks that they are here as it shows them.
    Inspection said =
        Inspection.of(line.camera + " → tag " + line.tag)
            .detail("Verdict", verdict.accepted() ? "accepted" : "rejected: " + verdict.reason())
            .detail("Innovation", innovation(verdict))
            .detail("χ²", chiSquared(verdict))
            .detail("Distance", distanceToTheTag(measured, line.tag))
            .detail(
                "Measured pose",
                String.format(
                    Locale.ROOT,
                    "x %.2f m, y %.2f m, heading %.1f°",
                    measured.getX(),
                    measured.getY(),
                    measured.getRotation().getDegrees()))
            .detail("Solved", howItWasSolved(frame, measurement.get()));
    Optional<String> calibrate = cameraAction.apply(line.camera);
    return calibrate.isPresent() ? said.action(calibrate.get()) : said;
  }

  private static Inspection notMeasured(Line line) {
    return Inspection.of(line.camera + " → tag " + line.tag).detail("Verdict", "not measured");
  }

  /**
   * How far the measurement was from the estimate. A verdict carries a number that isn't one for a
   * measurement that was refused before it was compared.
   */
  private static String innovation(TagVerdict verdict) {
    if (!Double.isFinite(verdict.innovationMeters())) {
      return "not compared";
    }
    return WorldView.offset(verdict.innovationMeters())
        + " from the estimate, before this measurement moved it";
  }

  /** The measurement's χ² against the estimate, which likewise isn't a number if not compared. */
  private static String chiSquared(TagVerdict verdict) {
    if (!Double.isFinite(verdict.chiSquared())) {
      return "not compared";
    }
    return WorldView.chiSquared(verdict.chiSquared());
  }

  private String distanceToTheTag(Pose2d measured, int tag) {
    Optional<Pose3d> onTheField = field.get().tag(tag);
    if (onTheField.isEmpty()) {
      return "the field has no such tag";
    }
    double meters =
        measured.getTranslation().getDistance(onTheField.get().toPose2d().getTranslation());
    return WorldView.meters(meters) + " from the measured pose to the tag";
  }

  /** How the image was solved, and from which tags: "multi tag from tags 25, 26". */
  private static String howItWasSolved(FrameVerdict frame, FrameVerdict.Measured measurement) {
    String solver = measurement.solver().name().toLowerCase(Locale.ROOT).replace('_', ' ');
    String tags =
        frame.tags().stream()
            .map(seen -> String.valueOf(seen.tag()))
            .collect(Collectors.joining(", "));
    return solver + " from tags " + tags;
  }

  /** An image's verdict on one of its tags, or empty if the image didn't use the tag. */
  private static Optional<TagVerdict> verdictOn(FrameVerdict frame, int tag) {
    for (TagVerdict verdict : frame.tags()) {
      if (verdict.tag() == tag) {
        return Optional.of(verdict);
      }
    }
    return Optional.empty();
  }
}

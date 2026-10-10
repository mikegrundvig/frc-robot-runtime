package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;

/**
 * A simulated camera's live faults, as the page applies them, through PhotonVision's simulation:
 * each changes what the robot's code reads from the camera, and undoing it puts that back. The
 * robot stands on blue's side, and the camera that sees the most tags from there is the one failed.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class CameraFaultsSimTest {
  private static final Pose2d STANDING = new Pose2d(2.0, 4.0, Rotation2d.ZERO);

  private @Nullable SimulatedWorld built;

  private final Components components = new Components(said -> {});

  /** The camera that sees the most tags from where the robot stands, and its name on the page. */
  private CameraIO camera = new CameraIO() {};

  private String name = "";

  @BeforeEach
  void standSeeingTags() {
    SimulatedWorld world =
        new SimulatedWorld(RobotSim.FIELD, ExampleRobot.SIM.definition(), SensorErrors.NONE);
    built = world;
    world.addTo(components);
    world.place(STANDING);
    CameraIO[] cameras = world.cameras();
    int[] seen = new int[cameras.length];
    for (int loop = 0; loop < RobotLoop.loops(1.0); loop++) {
      step();
      for (int i = 0; i < cameras.length; i++) {
        seen[i] += read(cameras[i]).tags.length;
      }
    }
    int best = 0;
    for (int i = 1; i < cameras.length; i++) {
      best = seen[i] > seen[best] ? i : best;
    }
    assertThat(seen[best]).as("tags seen from where the robot stands").isPositive();
    camera = cameras[best];
    CameraConfig config = ExampleRobot.SIM.definition().cameras().get(best);
    name = FaultCatalog.camera(config.name());
  }

  @AfterEach
  void close() {
    world().close();
  }

  private SimulatedWorld world() {
    return Objects.requireNonNull(built);
  }

  @Test
  void anUnpluggedCameraSendsNothingUntilPluggedBackIn() {
    apply(FaultCatalog.UNPLUGGED, Map.of());
    List<CameraIO.CameraIOInputs> unplugged = loops(10);
    undo(FaultCatalog.UNPLUGGED);
    List<CameraIO.CameraIOInputs> plugged = loops(10);

    assertThat(unplugged).allSatisfy(inputs -> assertThat(inputs.connected).isFalse());
    assertThat(unplugged).allSatisfy(inputs -> assertThat(inputs.frames).isEmpty());
    assertThat(plugged.get(9).connected).isTrue();
    assertThat(frames(plugged)).isPositive();
  }

  /** Frozen, it sends its last image's tags as new, however the robot moves; then sees again. */
  @Test
  void aFrozenCameraSendsTheSameTagsAsNewUntilUndone() {
    apply(FaultCatalog.FROZEN, Map.of());
    world().place(new Pose2d(2.5, 3.6, Rotation2d.fromDegrees(10))); // where it sees otherwise
    List<CameraIO.CameraIOInputs> frozen = loops(10);
    undo(FaultCatalog.FROZEN);
    List<CameraIO.CameraIOInputs> unfrozen = loops(10);

    // Every image the same tags, the last image's before it froze, each stamped as new.
    List<CameraIO.SeenTag> frozenTags = tags(frozen);
    List<CameraIO.SeenTag> lastImage = frozenTags.stream().distinct().toList();
    assertThat(frames(frozen)).isGreaterThan(1);
    assertThat(lastImage).isNotEmpty();
    assertThat(frozenTags).hasSize(lastImage.size() * frames(frozen));
    long[] times =
        frozen.stream()
            .flatMap(inputs -> Arrays.stream(inputs.frames))
            .mapToLong(CameraIO.Frame::timestampNanos)
            .toArray();
    assertThat(times).as("stamped as new").isSorted().doesNotHaveDuplicates();
    assertThat(tags(unfrozen)).isNotEmpty().doesNotContainAnyElementsOf(lastImage);
  }

  /** Knocked down on its mount, it sees its tags from lower down, until it's put back. */
  @Test
  void aCameraKnockedOnItsMountSeesTagsFromThere() {
    List<CameraIO.CameraIOInputs> seen = loops(15);
    int tag = mostCentered(seen);
    Translation2d before = meanCorner(seen, tag);

    apply(
        FaultCatalog.MOVED,
        Map.of(FaultCatalog.FORWARD, 0.0, FaultCatalog.LEFT, 0.0, FaultCatalog.UP, -6.0));
    loops(5); // images taken before the knock arrive first
    Translation2d knocked = meanCorner(loops(15), tag);
    undo(FaultCatalog.MOVED);
    loops(5);
    Translation2d after = meanCorner(loops(15), tag);

    assertThat(knocked.getDistance(before)).as("pixels the tag moved").isGreaterThan(10);
    assertThat(after.getDistance(before)).as("pixels, back").isLessThan(2);
  }

  /** Turned on its mount, it sees its tags turned, until it's turned back. */
  @Test
  void aCameraTurnedOnItsMountSeesTagsTurned() {
    List<CameraIO.CameraIOInputs> seen = loops(15);
    int tag = mostCentered(seen);
    Translation2d before = meanCorner(seen, tag);

    apply(
        FaultCatalog.TURNED,
        Map.of(FaultCatalog.ROLL, 0.0, FaultCatalog.PITCH, 0.0, FaultCatalog.YAW, 2.0));
    loops(5);
    Translation2d turned = meanCorner(loops(15), tag);
    undo(FaultCatalog.TURNED);
    loops(5);
    Translation2d after = meanCorner(loops(15), tag);

    // 2 degrees, of the image's 68 across its 1280 pixels: about 38 pixels.
    assertThat(turned.getDistance(before)).as("pixels the tag moved").isBetween(30.0, 45.0);
    assertThat(after.getDistance(before)).as("pixels, back").isLessThan(2);
  }

  /** Late, its images arrive that much later than they were taken, until it's back to usual. */
  @Test
  void aLateCameraSendsItsImagesLater() {
    double usual = meanLatencySeconds(30);

    apply(FaultCatalog.LATE, Map.of(FaultCatalog.EXTRA, 200.0));
    loops(15); // the images already on their way arrive first
    double late = meanLatencySeconds(30);
    undo(FaultCatalog.LATE);
    loops(15);
    double after = meanLatencySeconds(30);

    assertThat(late - usual).isBetween(0.15, 0.25);
    assertThat(after - usual).isBetween(-0.03, 0.03);
  }

  private void apply(String fault, Map<String, Double> values) {
    assertThat(components.apply(Setting.of(name, fault, values))).isNull();
  }

  private void undo(String fault) {
    assertThat(components.undo(name, fault)).isNull();
  }

  private void step() {
    RobotSim.stepClock();
    world().step(RobotController.getTime());
  }

  /** Runs loops, reading the camera each, as the robot's code does. */
  private List<CameraIO.CameraIOInputs> loops(int count) {
    List<CameraIO.CameraIOInputs> read = new ArrayList<>();
    for (int i = 0; i < count; i++) {
      step();
      read.add(read(camera));
    }
    return read;
  }

  /** How late its images arrived, on average, over these loops: from taken to read. */
  private double meanLatencySeconds(int count) {
    double sum = 0;
    int images = 0;
    for (int i = 0; i < count; i++) {
      step();
      long now = RobotController.getTime();
      for (CameraIO.Frame frame : read(camera).frames) {
        sum += (now - frame.timestampNanos()) / 1e9;
        images++;
      }
    }
    assertThat(images).as("images arrived").isPositive();
    return sum / images;
  }

  /** Where a tag's first corner is in the image, on average, in pixels. */
  private static Translation2d meanCorner(List<CameraIO.CameraIOInputs> read, int id) {
    List<CameraIO.SeenTag> tags = tags(read).stream().filter(tag -> tag.id() == id).toList();
    assertThat(tags).as("tag " + id + " seen").isNotEmpty();
    Translation2d sum = Translation2d.ZERO;
    for (CameraIO.SeenTag tag : tags) {
      sum = sum.plus(new Translation2d(tag.corner0X(), tag.corner0Y()));
    }
    return sum.div(tags.size());
  }

  /** The tag seen nearest the middle of the image, which a small knock keeps in view. */
  private static int mostCentered(List<CameraIO.CameraIOInputs> read) {
    Translation2d middle = new Translation2d(640, 400);
    return tags(read).stream()
        .min(
            java.util.Comparator.comparingDouble(
                tag -> new Translation2d(tag.corner0X(), tag.corner0Y()).getDistance(middle)))
        .orElseThrow()
        .id();
  }

  private static List<CameraIO.SeenTag> tags(List<CameraIO.CameraIOInputs> read) {
    return read.stream().flatMap(inputs -> Arrays.stream(inputs.tags)).toList();
  }

  private static int frames(List<CameraIO.CameraIOInputs> read) {
    return read.stream().mapToInt(inputs -> inputs.frames.length).sum();
  }

  private static CameraIO.CameraIOInputs read(CameraIO camera) {
    CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
    camera.updateInputs(inputs);
    return inputs;
  }
}

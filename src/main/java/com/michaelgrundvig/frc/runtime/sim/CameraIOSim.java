package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.VisionSim;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A simulated camera as the robot's code reads it, which can fail the ways a real one does while
 * the simulation runs: unplugged (no results, and it says it's disconnected), frozen (its last
 * image's tags again and again, stamped as new), knocked or turned on its mount, later than its
 * configuration says, or with its clock off the robot's (its images stamped wrong). The robot's
 * code still believes its configuration, and has to find out from what the camera sees.
 *
 * <p>Its mount moves from where it was mounted as the world was built ({@link SensorErrors}), and
 * goes back there when the fault is undone.
 */
final class CameraIOSim implements CameraIO {
  private static final Frame[] NO_FRAMES = {};
  private static final SeenTag[] NO_TAGS = {};

  private final String name;
  private final CameraIO camera;
  private final VisionSim vision;
  private final Transform3d builtMountError;
  private boolean unplugged;
  private boolean frozen;
  private Translation3d moved = Translation3d.ZERO;
  private Rotation3d turned = Rotation3d.ZERO;
  private double extraLatencyMillis;
  private double clockOffsetMillis;

  /** The last image seen before it froze, and its tags. */
  private @Nullable Frame lastFrame;

  private SeenTag[] lastTags = NO_TAGS;

  /**
   * @param camera what PhotonVision's simulation gives the robot for it
   * @param vision PhotonVision's simulation, which mounts it
   * @param builtMountError how it was mounted off its configuration as the world was built
   */
  CameraIOSim(String name, CameraIO camera, VisionSim vision, Transform3d builtMountError) {
    this.name = name;
    this.camera = camera;
    this.vision = vision;
    this.builtMountError = builtMountError;
  }

  /** Its name, as configured. */
  String name() {
    return name;
  }

  /** Unplugs it, or plugs it back in. */
  void unplug(boolean unplugged) {
    this.unplugged = unplugged;
  }

  boolean isUnplugged() {
    return unplugged;
  }

  /** Freezes it on its last image, or lets it see again. */
  void freeze(boolean frozen) {
    this.frozen = frozen;
  }

  boolean isFrozen() {
    return frozen;
  }

  /**
   * Knocks it this far from where it was mounted, in the robot's frame; zero puts it back. Images
   * taken from now on see from there.
   */
  void move(Translation3d by) {
    moved = by;
    remount();
  }

  boolean isMoved() {
    return !moved.equals(Translation3d.ZERO);
  }

  /** Turns it this far from how it was mounted, in the robot's frame; zero puts it back. */
  void turn(Rotation3d by) {
    turned = by;
    remount();
  }

  boolean isTurned() {
    return !turned.equals(Rotation3d.ZERO);
  }

  /** Makes it this much later than its configuration says; zero for its usual latency. */
  void delay(double extraMillis) {
    extraLatencyMillis = extraMillis;
    vision.addLatency(name, extraMillis);
  }

  boolean isLate() {
    return extraLatencyMillis != 0;
  }

  /**
   * Sets its clock this far off the robot's: each image stamped this much later than it was taken
   * (earlier, if negative), arriving when it would have. Zero for a clock that agrees.
   */
  void offsetClock(double millis) {
    clockOffsetMillis = millis;
    vision.offsetClock(name, millis);
  }

  boolean hasClockOffset() {
    return clockOffsetMillis != 0;
  }

  private void remount() {
    vision.mount(
        name,
        new Transform3d(
            builtMountError.getTranslation().plus(moved),
            builtMountError.getRotation().rotateBy(turned)));
  }

  /**
   * Reads what the camera sent, as a real camera's IO does, then fails it as it's been failed. The
   * camera is read either way, so an unplugged one's results don't wait to arrive all at once.
   */
  @Override
  public void updateInputs(CameraIOInputs inputs) {
    camera.updateInputs(inputs);
    if (unplugged) {
      inputs.connected = false;
      inputs.frames = NO_FRAMES;
      inputs.tags = NO_TAGS;
      inputs.skippedFrames = 0;
      return;
    }
    if (frozen) {
      refreeze(inputs);
      return;
    }
    int count = inputs.frames.length;
    if (count > 0) {
      Frame last = inputs.frames[count - 1];
      lastFrame = last;
      lastTags =
          Arrays.copyOfRange(inputs.tags, inputs.tags.length - last.tagCount(), inputs.tags.length);
    }
  }

  /** Each new image's time, with the last image's tags before it froze. */
  private void refreeze(CameraIOInputs inputs) {
    Frame last = lastFrame;
    if (last == null) {
      inputs.frames = NO_FRAMES; // frozen before it saw anything: nothing to repeat
      inputs.tags = NO_TAGS;
      return;
    }
    Frame[] frames = new Frame[inputs.frames.length];
    List<SeenTag> tags = new ArrayList<>();
    for (int i = 0; i < frames.length; i++) {
      frames[i] =
          new Frame(
              inputs.frames[i].timestampNanos(),
              last.tagCount(),
              last.hasMultiTag(),
              last.multiTagFieldToCamera(),
              last.multiTagReprojectionErrorPixels());
      tags.addAll(List.of(lastTags));
    }
    inputs.frames = frames;
    inputs.tags = tags.toArray(SeenTag[]::new);
  }

  @Override
  public void close() {
    camera.close();
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.visor.CameraToTag;
import com.michaelgrundvig.frc.visor.Candidate;
import com.michaelgrundvig.frc.visor.VisorFrame;
import com.michaelgrundvig.frc.visor.VisorReceiver;
import com.michaelgrundvig.frc.visor.VisorVersion;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Every camera's frames from Visor, the AprilTag vision program on the coprocessors, sorted by
 * camera for each one's {@link CameraIOVisor}. One receiver serves them all: the coprocessors send
 * every camera's frames to the same port, each frame naming its camera.
 *
 * <p>Each loop, the first camera to read takes every frame that arrived and sorts them; each camera
 * then takes its own.
 */
public final class VisorFeed implements AutoCloseable {
  /** PhotonVision's order of a tag's corners, from Visor's (OpenCV's): reversed. */
  private static final int[] PHOTONVISION_ORDER = {3, 2, 1, 0};

  private final VisorReceiver receiver;
  private final VisorFrame frame = new VisorFrame();
  private final CameraToTag pose = new CameraToTag();
  private final Map<Integer, Pending> pending = new HashMap<>();
  private long otherVersionSeen;

  /** One camera's frames, waiting for it to read them. */
  private static final class Pending {
    final List<CameraIO.Frame> frames = new ArrayList<>();
    final List<CameraIO.SeenTag> tags = new ArrayList<>();
    int skipped;

    /** When its last frame arrived, on the robot's clock: a frame's own stamp may mean nothing. */
    long lastHeardNanos = Long.MIN_VALUE;
  }

  private VisorFeed(VisorReceiver receiver) {
    this.receiver = receiver;
  }

  /** Frames from the coprocessors, on Visor's port. */
  public static VisorFeed listen(int port) {
    return new VisorFeed(VisorReceiver.listen(port));
  }

  /** Frames a simulation gives it ({@link #offer}), through the same decoding real ones take. */
  public static VisorFeed inMemory() {
    return new VisorFeed(VisorReceiver.inMemory());
  }

  /** Takes a frame as if a coprocessor had sent it. For a feed {@link #inMemory}. */
  public void offer(VisorFrame sent) {
    receiver.offer(sent);
  }

  /**
   * Moves one camera's frames since its last read into its inputs, after sorting whatever arrived.
   *
   * @param nowNanos the robot's clock, for whether the camera is still sending
   */
  void read(int camera, CameraIO.CameraIOInputs inputs, long nowNanos) {
    sortWhatArrived(nowNanos);
    Pending mine = pending.computeIfAbsent(camera, index -> new Pending());
    inputs.frames = mine.frames.toArray(CameraIO.Frame[]::new);
    inputs.tags = mine.tags.toArray(CameraIO.SeenTag[]::new);
    inputs.skippedFrames = mine.skipped;
    // A camera is connected while its frames keep coming: Visor sends one for every image. Judged
    // by when they arrive, so a coprocessor whose clock isn't synced yet still counts as sending.
    inputs.connected =
        mine.lastHeardNanos != Long.MIN_VALUE
            && nowNanos - mine.lastHeardNanos < STILL_SENDING_NANOS;
    inputs.error = versionError();
    mine.frames.clear();
    mine.tags.clear();
    mine.skipped = 0;
  }

  /** How long without a frame before a camera counts as disconnected. */
  private static final long STILL_SENDING_NANOS = 500_000_000L;

  private void sortWhatArrived(long nowNanos) {
    while (receiver.poll(frame)) {
      Pending into = pending.computeIfAbsent(frame.camera(), index -> new Pending());
      into.lastHeardNanos = nowNanos;
      // A frame from a clock not yet synced to the robot's has a capture time that means nothing.
      if (!frame.clockSynced() || frame.captureNanos() < 0) {
        into.skipped++;
        continue;
      }
      int solved = 0;
      for (int tag = 0; tag < frame.tagCount(); tag++) {
        // A tag Visor couldn't solve (no lens calibration) has no pose to use.
        if (frame.posesSolved(tag)) {
          into.tags.add(seenTag(tag));
          solved++;
        }
      }
      into.frames.add(new CameraIO.Frame(frame.captureNanos(), solved, false, Transform3d.ZERO, 0));
    }
  }

  private CameraIO.SeenTag seenTag(int tag) {
    double[] corners = new double[8];
    for (int i = 0; i < 4; i++) {
      corners[2 * i] = frame.cornerX(tag, PHOTONVISION_ORDER[i]);
      corners[2 * i + 1] = frame.cornerY(tag, PHOTONVISION_ORDER[i]);
    }
    return new CameraIO.SeenTag(
        frame.tagId(tag),
        corners[0],
        corners[1],
        corners[2],
        corners[3],
        corners[4],
        corners[5],
        corners[6],
        corners[7],
        transform(tag, Candidate.BEST),
        transform(tag, Candidate.OTHER),
        frame.ambiguity(tag),
        false);
  }

  private Transform3d transform(int tag, Candidate candidate) {
    frame.cameraToTag(tag, candidate, pose);
    return new Transform3d(
        new Translation3d(pose.x(), pose.y(), pose.z()),
        new Rotation3d(new Quaternion(pose.qw(), pose.qx(), pose.qy(), pose.qz())));
  }

  /** Why frames were refused, while a coprocessor runs another Visor than this robot's. */
  private String versionError() {
    long otherVersion = receiver.otherVersion();
    if (otherVersion == otherVersionSeen) {
      return "";
    }
    otherVersionSeen = otherVersion;
    return "a coprocessor runs Visor "
        + receiver.lastOtherVersionId()
        + ", the robot "
        + VisorVersion.ID
        + ": deploy to it again";
  }

  @Override
  public void close() {
    receiver.close();
  }
}

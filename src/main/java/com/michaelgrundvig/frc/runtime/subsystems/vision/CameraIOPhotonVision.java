package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.localization.Timestamps;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.wpilib.math.geometry.Transform3d;

/**
 * A camera running PhotonVision, on a coprocessor or in PhotonVision's simulation. It reads every
 * result since the last loop, with each tag's corners and both of its candidate poses, stamped with
 * its capture time on the robot's clock.
 */
public class CameraIOPhotonVision implements CameraIO {
  /** A tag has four corners, and each is an x and a y in the image. */
  private static final int CORNERS = 4;

  private final PhotonCamera camera;

  /**
   * @param name the camera's name in PhotonVision, and in the robot's configuration
   */
  public CameraIOPhotonVision(String name) {
    camera = new PhotonCamera(name);
  }

  /** The PhotonVision camera, for a simulation to attach to. */
  PhotonCamera camera() {
    return camera;
  }

  /**
   * Reads every result since the last loop.
   *
   * <p>PhotonLib throws on purpose when the coprocessor runs a different PhotonVision version than
   * the robot's library, and again every few seconds after. {@link Vision}'s camera catches that,
   * and the camera reads as unreadable, with PhotonLib's reason, while the robot drives on its
   * other sensors.
   */
  @Override
  public void updateInputs(CameraIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<Frame> frames = new ArrayList<>();
    List<SeenTag> tags = new ArrayList<>();
    int skipped = 0;
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      double capturedSeconds = result.getTimestampSeconds();
      // PhotonLib's placeholder metadata says -1, which is no capture time to place the image at.
      boolean hasCaptureTime = Double.isFinite(capturedSeconds) && capturedSeconds >= 0;
      if (!hasCaptureTime) {
        skipped++;
        continue;
      }
      Optional<MultiTargetPNPResult> multiTag = result.getMultiTagResult();
      Set<Integer> usedByMultiTag = tagsUsedBy(multiTag);
      List<PhotonTrackedTarget> targets = result.getTargets();
      frames.add(frame(Timestamps.fromSeconds(capturedSeconds), targets.size(), multiTag));
      for (PhotonTrackedTarget target : targets) {
        tags.add(seenTag(target, usedByMultiTag.contains(target.getFiducialId())));
      }
    }
    inputs.frames = frames.toArray(Frame[]::new);
    inputs.tags = tags.toArray(SeenTag[]::new);
    inputs.skippedFrames = skipped;
    inputs.error = "";
  }

  /** The IDs of the tags a multi-tag solution used, or none without a solution. */
  private static Set<Integer> tagsUsedBy(Optional<MultiTargetPNPResult> multiTag) {
    Set<Integer> used = new HashSet<>();
    if (multiTag.isPresent()) {
      for (short id : multiTag.get().fiducialIDsUsed) {
        used.add((int) id);
      }
    }
    return used;
  }

  /**
   * One image's result, as it is logged. A log's record holds plain values, so an image without a
   * multi-tag solution says so with its flag, and carries a zero transform and a zero error in the
   * solution's place.
   */
  private static Frame frame(
      long capturedNanos, int tagCount, Optional<MultiTargetPNPResult> multiTag) {
    if (multiTag.isEmpty()) {
      return new Frame(capturedNanos, tagCount, false, Transform3d.ZERO, 0.0);
    }
    return new Frame(
        capturedNanos,
        tagCount,
        true,
        multiTag.get().estimatedPose.best,
        multiTag.get().estimatedPose.bestReprojErr);
  }

  /** One tag in an image, as it is logged. */
  private static SeenTag seenTag(PhotonTrackedTarget target, boolean inMultiTag) {
    // Each corner's x and then its y, corner after corner. A corner PhotonVision didn't send is
    // left at zero.
    double[] corners = new double[2 * CORNERS];
    List<TargetCorner> detected = target.getDetectedCorners();
    for (int i = 0; i < Math.min(CORNERS, detected.size()); i++) {
      corners[2 * i] = detected.get(i).x;
      corners[2 * i + 1] = detected.get(i).y;
    }
    return new SeenTag(
        target.getFiducialId(),
        corners[0],
        corners[1],
        corners[2],
        corners[3],
        corners[4],
        corners[5],
        corners[6],
        corners[7],
        target.getBestCameraToTarget(),
        target.getAlternateCameraToTarget(),
        target.getPoseAmbiguity(),
        inMultiTag);
  }

  @Override
  public void close() {
    camera.close();
  }
}

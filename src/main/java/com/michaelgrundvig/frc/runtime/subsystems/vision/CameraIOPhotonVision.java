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
 * A camera running PhotonVision, on a coprocessor or in PhotonVision's simulation: every result
 * since the last loop, with each tag's corners and both of its candidate poses, stamped with its
 * capture time on the robot's clock.
 */
public class CameraIOPhotonVision implements CameraIO {
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
   * Reads every result since the last loop. PhotonLib throws on purpose when the coprocessor runs a
   * different PhotonVision version than the robot's library, and again every few seconds after;
   * {@link Vision} catches that, and the camera reads as unreadable, with PhotonLib's reason, while
   * the robot drives on its other sensors.
   */
  @Override
  public void updateInputs(CameraIOInputs inputs) {
    inputs.connected = camera.isConnected();
    List<Frame> frames = new ArrayList<>();
    List<SeenTag> tags = new ArrayList<>();
    int skipped = 0;
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      double capturedSeconds = result.getTimestampSeconds();
      if (!Double.isFinite(capturedSeconds) || capturedSeconds < 0) {
        skipped++; // PhotonLib's placeholder metadata says -1: no capture time to place it at
        continue;
      }
      Optional<MultiTargetPNPResult> multiTag = result.getMultiTagResult();
      Set<Integer> usedByMultiTag = new HashSet<>();
      multiTag.ifPresent(
          solved -> solved.fiducialIDsUsed.forEach(id -> usedByMultiTag.add((int) id)));
      List<PhotonTrackedTarget> targets = result.getTargets();
      frames.add(
          new Frame(
              Timestamps.fromSeconds(capturedSeconds),
              targets.size(),
              multiTag.isPresent(),
              multiTag.map(solved -> solved.estimatedPose.best).orElse(Transform3d.ZERO),
              multiTag.map(solved -> solved.estimatedPose.bestReprojErr).orElse(0.0)));
      for (PhotonTrackedTarget target : targets) {
        double[] corners = new double[8];
        List<TargetCorner> detected = target.getDetectedCorners();
        for (int i = 0; i < Math.min(4, detected.size()); i++) {
          corners[2 * i] = detected.get(i).x;
          corners[2 * i + 1] = detected.get(i).y;
        }
        tags.add(
            new SeenTag(
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
                usedByMultiTag.contains(target.getFiducialId())));
      }
    }
    inputs.frames = frames.toArray(Frame[]::new);
    inputs.tags = tags.toArray(SeenTag[]::new);
    inputs.skippedFrames = skipped;
    inputs.error = "";
  }

  @Override
  public void close() {
    camera.close();
  }
}

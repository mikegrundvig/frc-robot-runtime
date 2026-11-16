package com.michaelgrundvig.frc.runtime.coprocessors;

import static com.michaelgrundvig.frc.world.config.Rules.between;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.CameraType;
import org.wpilib.units.measure.Time;

/**
 * What the pit tunes of one camera on Visor, live from Pitwall: its exposure and its gain, as the
 * numbers Visor sets. It starts at what the camera runs at now, its type's or its venue's ({@link
 * CameraSettings}), and a value tuned away from that runs until it's kept in the venue or the robot
 * restarts ({@link VisorTuning}).
 *
 * @param exposure how long each image is exposed, its automatic exposure off: from 100 µs, one of
 *     V4L2's steps, to one frame's time, past which the frame rate drops
 * @param gain the camera's {@code gain} control, in the camera's own units, within the range its
 *     type gives, or UVC's whole range where the type gives none; Visor holds it to the camera's
 *     own range. On a camera left at its own gain, it starts at the gain the camera reports, once
 *     its frames arrive
 */
@ConfigRecord
public record CameraTuning(Time exposure, int gain) {
  public CameraTuning {
    Checks.on(CameraTuning.class)
        .field(
            "exposure",
            exposure,
            between(CameraType.Exposure.Fixed.SHORTEST, CameraType.Exposure.Fixed.LONGEST))
        .field("gain", gain, between(0, CameraType.Gain.Level.MOST))
        .orThrow();
  }

  /** A camera's tuning, at this exposure and gain. */
  static CameraTuning of(Time exposure, int gain) {
    return CameraTuningBuilder.builder().exposure(exposure).gain(gain).build();
  }
}

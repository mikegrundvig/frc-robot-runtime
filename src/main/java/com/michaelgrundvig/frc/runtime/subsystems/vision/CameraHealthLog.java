package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.wpilib.units.Units.Hertz;

import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import org.littletonrobotics.junction.Logger;

/**
 * One camera's health, judged each loop from what it read ({@link CameraHealth}) and logged under
 * {@code Vision/<camera>/Health}: its frame rate, the frames its vision program dropped and the
 * network lost, and its detection time, over the last 10 s, with a flag for each warning. It's
 * judged from the camera's inputs alone, so a replay logs the same.
 */
final class CameraHealthLog {
  private final CameraHealth health;

  // Each value's name in the log, made once.
  private final String framesPerSecondKey;
  private final String droppedKey;
  private final String lostKey;
  private final String detectMeanKey;
  private final String detectWorstKey;
  private final String restartsKey;
  private final String droppingKey;
  private final String losingKey;
  private final String slowKey;

  CameraHealthLog(CameraConfig config) {
    health = new CameraHealth(1 / config.sensor().frameRate().in(Hertz));
    String key = "Vision/" + config.name() + "/Health/";
    framesPerSecondKey = key + "FramesPerSecond";
    droppedKey = key + "DroppedLately";
    lostKey = key + "LostLately";
    detectMeanKey = key + "DetectMeanMs";
    detectWorstKey = key + "DetectWorstMs";
    restartsKey = key + "VisorRestarts";
    droppingKey = key + "DroppingFrames";
    losingKey = key + "LosingFrames";
    slowKey = key + "DetectionSlow";
  }

  /** Judges this loop's inputs, and logs the camera's health. */
  void take(CameraIO.CameraIOInputs inputs, long nowNanos) {
    // Every frame that arrived: those handed on, and those left out for want of a capture time.
    int arrived = inputs.frames.length + inputs.skippedFrames;
    judge(inputs, arrived, nowNanos);
    Logger.recordOutput(framesPerSecondKey, health.framesPerSecond());
    // A camera that doesn't number its frames has no dropped or lost count, and none is logged as
    // if
    // it were one.
    if (health.numbers() == CameraHealth.Numbers.REPORTED) {
      Logger.recordOutput(droppedKey, health.droppedLately());
      Logger.recordOutput(lostKey, health.lostLately());
    }
    Logger.recordOutput(detectMeanKey, health.detectMeanMs());
    Logger.recordOutput(detectWorstKey, health.detectWorstMs());
    Logger.recordOutput(restartsKey, health.restarts());
    Logger.recordOutput(droppingKey, health.warns(CameraHealth.Warning.DROPPED_FRAMES));
    Logger.recordOutput(losingKey, health.warns(CameraHealth.Warning.LOST_FRAMES));
    Logger.recordOutput(slowKey, health.warns(CameraHealth.Warning.SLOW_DETECTION));
  }

  /**
   * Hands this loop's frames to the health check. The inputs say "no number" with -1, because an
   * AdvantageKit input is a plain value a log can hold; this is the one place that -1 is read, and
   * it becomes which of the check's updates fits.
   */
  private void judge(CameraIO.CameraIOInputs inputs, int arrived, long nowNanos) {
    boolean numbered = inputs.sequence >= 0 && inputs.dropped >= 0;
    if (!numbered) {
      health.update(nowNanos, arrived, inputs.detectMs);
      return;
    }
    health.update(nowNanos, arrived, inputs.sequence, inputs.dropped, inputs.detectMs);
  }

  CameraHealth health() {
    return health;
  }
}

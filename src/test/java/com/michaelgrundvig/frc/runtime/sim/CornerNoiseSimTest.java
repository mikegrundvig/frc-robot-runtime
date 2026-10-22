package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.VisionSim;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.RobotController;

/**
 * How far the simulated cameras' tag corners land from a perfect camera's is the world's {@link
 * SensorErrors#cornerNoise()}: the same seed gives the same corners, a world left as it was gives
 * the corners it always did, and a wider noise moves them.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class CornerNoiseSimTest {
  private static final Pose2d STANDING = new Pose2d(2.0, 4.0, Rotation2d.ZERO);

  @Test
  void theDefaultIsTheSimulatorsOwnNoise() {
    assertThat(SensorErrors.CornerNoise.TYPICAL.meanPixels())
        .isEqualTo(VisionSim.CORNER_NOISE_MEAN_PIXELS);
    assertThat(SensorErrors.CornerNoise.TYPICAL.stdDevPixels())
        .isEqualTo(VisionSim.CORNER_NOISE_STD_DEV_PIXELS);
    assertThat(SensorErrors.NONE.cornerNoise()).isEqualTo(SensorErrors.CornerNoise.TYPICAL);
    assertThat(SensorErrors.TYPICAL.cornerNoise()).isEqualTo(SensorErrors.CornerNoise.TYPICAL);
  }

  @Test
  void aWiderNoiseMovesTheCornersAndTheSameNoiseRepeatsThem() {
    List<Double> usual = corners(SensorErrors.NONE);
    List<Double> again =
        corners(SensorErrors.NONE.withCornerNoise(SensorErrors.CornerNoise.TYPICAL));
    List<Double> wide =
        corners(
            SensorErrors.NONE.withCornerNoise(
                SensorErrors.CornerNoise.builder().meanPixels(1.5).stdDevPixels(0.5).build()));

    assertThat(usual).isNotEmpty().isEqualTo(again);
    assertThat(wide).hasSameSizeAs(usual).isNotEqualTo(usual);
  }

  /** Every corner's x of the first camera's images, standing where it sees tags. */
  private static List<Double> corners(SensorErrors errors) {
    List<Double> corners = new ArrayList<>();
    try (SimulatedWorld world =
        new SimulatedWorld(
            RobotSim.FIELD,
            ExampleRobot.SIM.definition(),
            RobotSim.massOf(ExampleRobot.SIM.definition()),
            errors)) {
      world.place(STANDING);
      CameraIO[] cameras = world.cameras();
      for (int loop = 0; loop < RobotLoop.loops(1.0); loop++) {
        RobotSim.stepClock();
        world.step(RobotController.getTime());
        for (CameraIO camera : cameras) {
          CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
          camera.updateInputs(inputs);
          for (CameraIO.SeenTag tag : inputs.tags) {
            corners.add(tag.corner0X());
          }
        }
      }
    }
    return corners;
  }
}

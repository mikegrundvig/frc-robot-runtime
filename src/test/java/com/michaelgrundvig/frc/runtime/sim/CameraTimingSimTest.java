package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.localization.TagSolver;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.system.RobotController;

/**
 * The simulated cameras stamp each image when it was taken, as a real camera's time sync does.
 * PhotonVision's own simulation renders an image as of when it fell due but stamps it as of the
 * loop that renders it, up to a loop later; while the robot turns, every image would then put it
 * where it was a little earlier, the same way, as if the heading were off ({@code VisionSim}). A
 * camera whose clock is off stamps them off by as much: the truth suite's check that the stamps are
 * right is also the check that the clock-offset fault makes them wrong by exactly that.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class CameraTimingSimTest {
  /** How fast the robot is carried along the field, m/s: a loop late would be 4 cm. */
  private static final double SPEED = 2.0;

  /**
   * Before blue's hub, turned 45° so that two of the robot's corner cameras look along the field:
   * one back at the tags on blue's wall, one ahead at the hub's.
   */
  private static final Pose2d START = new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(45));

  /** Where the robot is carried to, this long after it set off. */
  private static Pose2d carried(long nanos) {
    return new Pose2d(
        START.getX() + SPEED * Math.max(0, nanos) / 1e9, START.getY(), START.getRotation());
  }

  @Test
  void eachImageShowsTheRobotWhereItWasAtTheImagesStamp() {
    try (SimulatedWorld world = world()) {
      double mean = meanAheadOfItsStamp(world);

      // Stamped up to a loop late, the robot would be about 1.5 to 2 cm behind where its stamp
      // says.
      assertThat(Math.abs(mean)).as("meters behind or ahead, on average").isLessThan(0.004);
    }
  }

  /**
   * A camera whose clock is 20 ms off the robot's stamps each image 20 ms after it was taken: each
   * shows the robot where it was 20 ms before its stamp says, 4 cm behind at 2 m/s.
   */
  @Test
  void aCameraWithItsClockOffStampsEachImageThatMuchOff() {
    try (SimulatedWorld world = world()) {
      Components components = new Components(said -> {});
      world.addTo(components);
      for (String camera : world.cameraNames()) {
        assertThat(
                components.apply(
                    Setting.of(
                        FaultCatalog.camera(camera),
                        FaultCatalog.CLOCK_OFFSET,
                        Map.of(FaultCatalog.OFFSET, 20.0))))
            .isNull();
      }

      double mean = meanAheadOfItsStamp(world);

      assertThat(mean).as("meters ahead of where the stamp says").isCloseTo(-0.04, within(0.004));
    }
  }

  private static SimulatedWorld world() {
    return new SimulatedWorld(
        RobotSim.FIELD,
        ExampleRobot.SIM.definition(),
        RobotSim.massOf(ExampleRobot.SIM.definition()),
        SensorErrors.NONE);
  }

  /**
   * Carries the robot along the field at {@link #SPEED}, and returns how far ahead of where it was
   * at each image's stamp each image puts it, on average, in meters: from the cameras' multi-tag
   * solves.
   */
  private static double meanAheadOfItsStamp(SimulatedWorld world) {
    world.place(START);
    CameraIO[] cameras = world.cameras();
    for (int i = 0; i < RobotLoop.loops(1.0); i++) {
      step(world, START);
      for (CameraIO camera : cameras) {
        read(camera);
      }
    }
    long setOff = RobotController.getTime();
    List<Double> along = new ArrayList<>();
    for (int i = 0; i < RobotLoop.loops(0.5); i++) {
      step(world, carried(RobotController.getTime() + PERIOD_NANOS - setOff));
      for (int c = 0; c < cameras.length; c++) {
        Transform3d mount = ExampleRobot.SIM.definition().cameras().get(c).robotToCamera();
        for (CameraIO.Frame frame : read(cameras[c]).frames) {
          if (!frame.hasMultiTag() || frame.timestampNanos() < setOff) {
            continue;
          }
          Transform3d seen = frame.multiTagFieldToCamera();
          Pose3d robot =
              TagSolver.robotFromCamera(
                  new Pose3d(seen.getTranslation(), seen.getRotation()), mount);
          along.add(robot.getX() - carried(frame.timestampNanos() - setOff).getX());
        }
      }
    }
    assertThat(along).as("images with several tags while carried").hasSizeGreaterThan(20);
    return along.stream().mapToDouble(Double::doubleValue).average().orElseThrow();
  }

  private static final long PERIOD_NANOS = Math.round(RobotLoop.PERIOD_SECONDS * 1e9);

  /** One loop, the robot carried to here by its end. */
  private static void step(SimulatedWorld world, Pose2d where) {
    RobotSim.stepClock();
    world.place(where);
    world.step(RobotController.getTime());
  }

  private static CameraIO.CameraIOInputs read(CameraIO camera) {
    CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
    camera.updateInputs(inputs);
    return inputs;
  }
}

package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.localization.TagSolver;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.system.RobotController;

/**
 * The simulator's Visor path, against its PhotonVision path. The same images, from the same robot
 * driven the same way, reach the robot code through Visor's frames, its client and {@code
 * CameraIOVisor} with nothing lost: the same images at the same times, the same tags, corners,
 * poses and ambiguities. Only the multi-tag solution is missing, as Visor makes none. And each
 * image through Visor shows the robot where it was when its stamp says it was taken.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class VisorPathSimTest {
  /** Before blue's hub, turned 45° so that cameras look along the field at tags. */
  private static final Pose2d START = new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(45));

  private static final double SPEED = 2.0;

  /** Carried along the field and turning slowly, this long after setting off. */
  private static Pose2d carried(long nanos) {
    double seconds = Math.max(0, nanos) / 1e9;
    return new Pose2d(
        START.getX() + SPEED * seconds,
        START.getY(),
        START.getRotation().plus(Rotation2d.fromDegrees(20 * seconds)));
  }

  /** One image as the robot code read it, its time from when the robot set off. */
  private record Read(
      int camera, long sinceSetOffNanos, boolean multiTag, List<CameraIO.SeenTag> tags) {}

  /** Drives a world of the given software, and every image its cameras read after setting off. */
  private static List<Read> drive(CameraConfig.Software software) {
    List<Read> images = new ArrayList<>();
    try (SimulatedWorld world =
        new SimulatedWorld(
            RobotSim.FIELD,
            ExampleRobot.SIM.definition(),
            RobotSim.massOf(ExampleRobot.SIM.definition()),
            SensorErrors.NONE,
            software)) {
      world.place(START);
      CameraIO[] cameras = world.cameras();
      for (int i = 0; i < RobotLoop.loops(1.0); i++) {
        step(world, START);
        for (CameraIO camera : cameras) {
          read(camera);
        }
      }
      long setOff = RobotController.getTime();
      for (int i = 0; i < RobotLoop.loops(1.0); i++) {
        step(world, carried(RobotController.getTime() + PERIOD_NANOS - setOff));
        for (int c = 0; c < cameras.length; c++) {
          CameraIO.CameraIOInputs inputs = read(cameras[c]);
          assertThat(inputs.connected || i < 5).as("%s camera %d connected", software, c).isTrue();
          int next = 0;
          for (CameraIO.Frame frame : inputs.frames) {
            List<CameraIO.SeenTag> tags =
                List.of(inputs.tags).subList(next, next + frame.tagCount());
            next += frame.tagCount();
            if (frame.timestampNanos() >= setOff) {
              images.add(new Read(c, frame.timestampNanos() - setOff, frame.hasMultiTag(), tags));
            }
          }
        }
      }
    }
    return images;
  }

  @Test
  void visorsPathLosesNothingOfWhatTheCamerasSaw() {
    List<Read> photonVision = drive(CameraConfig.Software.PHOTONVISION);
    List<Read> visor = drive(CameraConfig.Software.VISOR);

    assertThat(visor).as("images").hasSameSizeAs(photonVision).hasSizeGreaterThan(50);
    int tags = 0;
    double worstMetres = 0;
    double worstRadians = 0;
    double worstPixels = 0;
    for (int i = 0; i < visor.size(); i++) {
      Read sent = photonVision.get(i);
      Read got = visor.get(i);
      assertThat(got.camera()).isEqualTo(sent.camera());
      assertThat(got.sinceSetOffNanos())
          .as("image %d's time", i)
          .isEqualTo(sent.sinceSetOffNanos());
      assertThat(got.multiTag()).as("Visor solves no tags together").isFalse();
      assertThat(got.tags()).hasSameSizeAs(sent.tags());
      for (int t = 0; t < got.tags().size(); t++) {
        CameraIO.SeenTag a = got.tags().get(t);
        CameraIO.SeenTag b = sent.tags().get(t);
        assertThat(a.id()).isEqualTo(b.id());
        worstPixels = Math.max(worstPixels, cornersApart(a, b));
        for (Transform3d[] pair :
            List.of(
                new Transform3d[] {a.bestCameraToTag(), b.bestCameraToTag()},
                new Transform3d[] {a.altCameraToTag(), b.altCameraToTag()})) {
          worstMetres =
              Math.max(worstMetres, pair[0].getTranslation().getDistance(pair[1].getTranslation()));
          // A quaternion and its negative are the same rotation, which reads as a full turn.
          double angle = pair[0].getRotation().relativeTo(pair[1].getRotation()).getAngle();
          worstRadians = Math.max(worstRadians, Math.min(angle, 2 * Math.PI - angle));
        }
        double ambiguity = b.ambiguity() >= 0 ? Math.min(b.ambiguity(), 1) : 1;
        assertThat(a.ambiguity())
            .as("tag %d's ambiguity", a.id())
            .isCloseTo(ambiguity, within(1e-6));
        tags++;
      }
    }
    assertThat(tags).as("tags compared").isGreaterThan(100);
    // Visor's frames carry 32-bit floats: a few float steps from PhotonVision's doubles.
    assertThat(worstPixels).as("corners, pixels").isLessThan(1e-3);
    assertThat(worstMetres).as("poses, metres").isLessThan(1e-5);
    assertThat(worstRadians).as("poses, radians").isLessThan(1e-5);
  }

  @Test
  void eachImageThroughVisorShowsTheRobotWhereItWasAtItsStamp() {
    List<Double> behind = new ArrayList<>();
    for (Read image : drive(CameraConfig.Software.VISOR)) {
      Transform3d mount =
          ExampleRobot.SIM.definition().cameras().get(image.camera()).robotToCamera();
      Pose2d truth = carried(image.sinceSetOffNanos());
      for (CameraIO.SeenTag tag : image.tags()) {
        // As the pipeline solves an image: where the tag is from the robot, turned by the robot's
        // heading (here the true one at the stamp), leaves where the robot stands.
        Pose3d tagPose = RobotSim.FIELD.tag(tag.id()).orElseThrow();
        Transform3d robotToTag = mount.plus(tag.bestCameraToTag());
        TagSolver.PositionFix fix =
            TagSolver.positionFromTag(
                tagPose, robotToTag, new Rotation3d(0, 0, truth.getRotation().getRadians()));
        // Along the way it's carried: a stamp late or early shows here, 4 cm a loop.
        behind.add(fix.position().getX() - truth.getX());
      }
    }
    assertThat(behind).as("tags seen while carried").hasSizeGreaterThan(50);
    // Stamped right, the images show the robot where it was at the stamp, on average: a loop late
    // would put it 4 cm behind.
    double mean = behind.stream().mapToDouble(Double::doubleValue).average().orElseThrow();
    assertThat(Math.abs(mean))
        .as("metres ahead of or behind the stamp, on average")
        .isLessThan(0.004);
  }

  private static double cornersApart(CameraIO.SeenTag a, CameraIO.SeenTag b) {
    double[] x = {
      a.corner0X() - b.corner0X(), a.corner1X() - b.corner1X(),
      a.corner2X() - b.corner2X(), a.corner3X() - b.corner3X()
    };
    double[] y = {
      a.corner0Y() - b.corner0Y(), a.corner1Y() - b.corner1Y(),
      a.corner2Y() - b.corner2Y(), a.corner3Y() - b.corner3Y()
    };
    double worst = 0;
    for (int i = 0; i < 4; i++) {
      worst = Math.max(worst, Math.hypot(x[i], y[i]));
    }
    return worst;
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

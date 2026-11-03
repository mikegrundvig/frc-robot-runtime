package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.util.AlertDataJNI;

/**
 * The calibration procedures on the simulated robot, whose true wheels, gyro, and camera mounts are
 * known: each should find the number the simulator hid.
 */
@SimulationTest
class CalibrationSimTest {
  private static final Pattern NUMBER = Pattern.compile("-?\\d+\\.\\d+");

  @RegisterExtension final RobotSim sim = new RobotSim();

  private RobotContainer build(SensorErrors errors, boolean cameras) {
    return sim.build(cameras ? ExampleRobot.SIM : RobotSim.WITHOUT_CAMERAS, errors);
  }

  /** Runs a procedure to its end, enabled, the way the Pitwall page runs it. */
  private String run(Command procedure, double atMostSeconds) {
    RobotSim.setEnabled(true);
    sim.scheduler().schedule(procedure);
    for (int i = 0; i < RobotLoop.loops(atMostSeconds); i++) {
      sim.loops(1);
      if (!sim.scheduler().isScheduledOrRunning(procedure)) {
        break;
      }
    }
    assertThat(sim.scheduler().isScheduledOrRunning(procedure)).as("finished in time").isFalse();
    return sim.robot().calibration().lastResult();
  }

  /** The numbers in a report, in order. */
  private static double[] numbers(String text) {
    Matcher matcher = NUMBER.matcher(text);
    return matcher.results().mapToDouble(match -> Double.parseDouble(match.group())).toArray();
  }

  @Test
  void theWheelRadiusIsFoundBySpinningInPlace() {
    build(SensorErrors.NONE.withWheelRadiusScale(0.98), false); // tread worn 2%
    sim.run(0.5);

    String report = run(sim.robot().calibration().wheelRadius(), 60);

    assertThat(report).startsWith("Wheel radius ").contains("In Robots: .wheelRadius(Inches.of(");
    assertThat(numbers(report)[0]).as("inches").isCloseTo(2 * 0.98, within(0.005));
  }

  @Test
  void theGyrosScaleIsFoundAgainstTheCameras() {
    SensorErrors gyroHigh =
        SensorErrors.NONE.withGyros(
            SensorErrors.GyroErrors.builder()
                .scale(1.003)
                .driftDegreesPerMinute(0)
                .noiseDegrees(0)
                .build());
    build(gyroHigh, true);
    sim.run(1.0); // placed, and the cameras settle

    String report = run(sim.robot().calibration().gyroScale(), 60);

    // Within 0.03% of the truth (half a degree over five turns): the cameras' noise, which the
    // fixes the procedure turns against carry, allows that much.
    assertThat(report).startsWith("Gyro scale ").contains("% high");
    assertThat(numbers(report)[0]).isCloseTo(1 / 1.003, within(0.0003));
    assertThat(numbers(report)[1]).as("percent high").isCloseTo(0.3, within(0.03));

    // Five turns on a gyro reading 0.3% high leave the heading 5.5° off, and the pose centimeters
    // off while the cameras bring them back: every tag lands off then, and no camera is to blame.
    sim.run(15);
    for (CameraConfig camera : ExampleRobot.SIM.definition().cameras()) {
      assertThat(sim.robot().vision().mountDoubtMeters(camera.name())).as(camera.name()).isZero();
    }
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      assertThat(alert.activeStartTime != 0 && alert.text.contains("out of calibration"))
          .as(alert.text)
          .isFalse();
    }
  }

  /** A camera's clock off the robot's by this, as a coprocessor whose time sync failed. */
  private void offsetClock(String camera, double millis) {
    assertThat(
            sim.components()
                .apply(
                    Setting.of(
                        FaultCatalog.camera(camera),
                        FaultCatalog.CLOCK_OFFSET,
                        Map.of(FaultCatalog.OFFSET, millis))))
        .isNull();
  }

  /**
   * The milliseconds a camera's sentence in a capture-times report says, signed: late is positive.
   */
  private static double lateMillis(String report, String camera) {
    Matcher said =
        Pattern.compile(Pattern.quote(camera) + ": images stamped (\\d+\\.\\d) ms (late|early)")
            .matcher(report);
    assertThat(said.find()).as("%s in: %s", camera, report).isTrue();
    double millis = Double.parseDouble(said.group(1));
    return said.group(2).equals("late") ? millis : -millis;
  }

  /**
   * The front-left camera's clock 25 ms ahead of the robot's: spinning both ways where the cameras
   * see tags, its stamps are found 25 ms late, and the others' on time.
   */
  @Test
  void aCameraStampingLateIsFoundBySpinningBothWays() {
    build(SensorErrors.NONE, true);
    sim.run(1.0);
    offsetClock("front-left", 25);

    String report = run(sim.robot().calibration().captureTimes(), 40);
    System.out.println(report);

    assertThat(lateMillis(report, "front-left")).isCloseTo(25, within(3.0));
    for (CameraConfig camera : ExampleRobot.SIM.definition().cameras()) {
      if (!camera.name().equals("front-left") && report.contains(camera.name() + ": images")) {
        assertThat(lateMillis(report, camera.name())).as(camera.name()).isCloseTo(0, within(3.0));
      }
    }
    assertThat(report).contains("front-left").contains("PhotonVision's time sync isn't working");
  }

  /**
   * The same on Visor: the camera's sentence ends with its time offset to paste, which corrects the
   * stamps by what was found.
   */
  @Test
  void aCameraOnVisorGetsItsTimeOffset() {
    CameraConfig frontLeft =
        ExampleRobot.SIM.definition().cameras().stream()
            .filter(camera -> camera.name().equals("front-left"))
            .findFirst()
            .orElseThrow();
    sim.build(
        ExampleRobot.SIM.with(
            CameraConfigBuilder.builder(frontLeft).software(CameraConfig.Software.VISOR).build()),
        SensorErrors.NONE);
    sim.run(1.0);
    offsetClock("front-left", 25);

    String report = run(sim.robot().calibration().captureTimes(), 40);
    System.out.println(report);

    Matcher offset =
        Pattern.compile(
                "front-left camera's Software\\.visor\\(\\): \\.timeOffset\\(Milliseconds\\.of\\((-?\\d+\\.\\d)\\)\\)")
            .matcher(report);
    assertThat(offset.find()).as(report).isTrue();
    assertThat(Double.parseDouble(offset.group(1))).isCloseTo(-25, within(3.0));
  }

  @Test
  void theLevelCheckReadsTheImu() {
    build(SensorErrors.NONE, false);
    sim.run(0.5);
    String level = run(sim.robot().calibration().level(), 5);

    sim.simulation().tilt(Math.toRadians(1.5), 0);
    String tilted = run(sim.robot().calibration().level(), 5);

    assertThat(level).startsWith("Level: pitch 0.00°, roll 0.00°");
    assertThat(tilted).startsWith("Not level: pitch 1.50°");
  }

  /**
   * The front-left camera knocked 4 in low: driving past tags while the other three cameras hold
   * the robot's pose, its mount is found again.
   */
  @Test
  void aCameraKnockedLowIsPlacedAgainByTheOthers() {
    build(
        SensorErrors.TYPICAL.withCameraMountError(
            "front-left", new Transform3d(0, 0, -0.1016, Rotation3d.ZERO)),
        true);
    sim.run(1.0);
    Command procedure = sim.robot().calibration().cameraMount("front-left");
    RobotSim.setEnabled(true);
    sim.scheduler().schedule(procedure);

    // Stop at spots in view of tags, nearer and farther, turned a little each time.
    double[][] moves = {
      {1.0, -0.5, 0, 0.1},
      {1.0, -0.5, 0.3, 0},
      {1.0, 0, -0.5, -0.15},
      {1.0, 0.5, 0, 0.1},
      {1.0, -0.5, -0.3, 0},
      {1.0, 0.4, 0.4, -0.1}
    };
    for (double[] move : moves) {
      sim.driver().setLeftY(-move[1]);
      sim.driver().setLeftX(-move[2]);
      sim.driver().setRightX(-move[3]);
      sim.driver().notifyNewData();
      sim.run(move[0]);
      sim.driver().setLeftY(0);
      sim.driver().setLeftX(0);
      sim.driver().setRightX(0);
      sim.driver().notifyNewData();
      sim.run(2.0); // standing still
    }
    sim.driver().setLeftY(0);
    sim.driver().setLeftX(0);
    sim.driver().setRightX(0);
    sim.driver().notifyNewData();
    // The camera sees tags from some of the stops, not all: the procedure takes its whole twenty
    // seconds, a second and a half more.
    for (int loop = 0; loop < RobotLoop.loops(3.0); loop++) {
      if (!sim.scheduler().isScheduledOrRunning(procedure)) {
        break;
      }
      sim.loops(1);
    }

    String report = sim.robot().calibration().lastResult();
    assertThat(report)
        .startsWith("front-left: tags landed")
        .contains("In Robots, the front-left camera's mount: .forward(Inches.of(");
    String line = report.substring(report.indexOf(".forward("));
    double[] mount = numbers(line);
    // Configured at (11.75, 11.75, 21) in, roll 0, tilted up 10, turned 45: really 4 in lower.
    assertThat(mount[0]).as("forward").isCloseTo(11.75, within(0.4));
    assertThat(mount[1]).as("left").isCloseTo(11.75, within(0.4));
    assertThat(mount[2]).as("up").isCloseTo(17.0, within(0.4));
    assertThat(mount[4]).as("tilted up").isCloseTo(10, within(0.5));
    assertThat(mount[5]).as("turned").isCloseTo(45, within(0.5));
    assertThat(numbers(report)[0]).as("tags landed off, cm").isGreaterThan(5);
    assertThat(numbers(report)[1]).as("and with the mount found").isLessThan(3);
  }

  /**
   * A canceled command never resumes, so its cleanup can't be code after its loop: disabling the
   * robot mid-calibration must give the camera back to localization at once, not at a restart.
   */
  @Test
  void aCanceledCalibrationGivesTheCamerasBack() {
    build(SensorErrors.NONE, true);
    sim.run(1.0);
    RobotSim.setEnabled(true);
    for (Command procedure :
        List.of(
            sim.robot().calibration().cameraMount("front-left"),
            sim.robot().calibration().gyroScale())) {
      sim.scheduler().schedule(procedure);
      sim.loops(5);
      assertThat(sim.robot().vision().isCalibrating()).as(procedure.name()).isTrue();

      sim.scheduler().cancel(procedure);
      sim.loops(1);

      assertThat(sim.robot().vision().isCalibrating()).as(procedure.name()).isFalse();
      assertThat(sim.robot().calibration().lastResult()).endsWith(": canceled before it finished");
    }
  }
}

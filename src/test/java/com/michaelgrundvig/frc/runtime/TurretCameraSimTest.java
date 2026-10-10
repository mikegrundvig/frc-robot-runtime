package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.RobotController;

/**
 * A camera on a turret, end to end: the turret sweeping side to side as the robot drives, and each
 * image solved from where the turret had the camera when it was taken. It's the robot's only
 * camera, so the estimate holds to the truth through it or not at all.
 */
@SimulationTest
class TurretCameraSimTest {
  /** 15 cm out from the turret's axis and 5 cm above its base, facing its front, tilted up 10°. */
  static final CameraConfig TURRET_CAMERA =
      CameraConfigBuilder.builder(ExampleRobot.SIM.definition().cameras().get(0))
          .name("turret")
          .forward(Meters.of(0.15))
          .left(Meters.of(0))
          .up(Meters.of(0.05))
          .roll(Degrees.of(0))
          .tiltUp(Degrees.of(10))
          .yaw(Degrees.of(0))
          .joint(TURRET)
          .build();

  /** The template's robot with a turret, its one camera on it. */
  static final RobotConfig ROBOT =
      RobotConfigBuilder.builder(ExampleRobot.SIM)
          .definition(
              RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                  .name("turret camera")
                  .joints(List.of(TURRET))
                  .cameras(List.of(TURRET_CAMERA))
                  .build())
          .build();

  /** Forward, across, turning: seconds, then the left stick's y and x and the right stick's x. */
  private static final double[][] ROUTE = {
    {1.5, -0.5, 0, 0}, {1.0, 0, 0.7, 0}, {1.5, 0, 0, 0.6}, {2.0, 0.4, -0.5, 0.2}, {1.0, 0, 0, 0}
  };

  @RegisterExtension final RobotSim sim = new RobotSim();

  private double error() {
    Pose2d truth = sim.simulation().truePose();
    return sim.robot().drive().getPose().getTranslation().getDistance(truth.getTranslation());
  }

  /**
   * Measured 2026-10-06, the sweep timed from the test's start: 1.9 cm at worst while driving, 1.8
   * cm stopped, 127 images accepted and none rejected, the same every run. (Timed on the robot's
   * clock, which carries on from earlier tests, its phase changed with what ran before: 98 to 141
   * images.) Solved as if the camera never turned, the same drive passes 15 cm.
   */
  @Test
  void theTurretsCameraHoldsTheEstimateAsTheTurretSweeps() {
    sim.build(ROBOT, SensorErrors.TYPICAL);
    sim.run(1.0);
    RobotSim.setEnabled(true);
    // Side to side, 40° each way, every four seconds, from now: the robot's clock carries on from
    // whatever ran before in this JVM, and a sweep on it would point the camera differently each
    // time, seeing more tags one run and fewer the next.
    long start = RobotController.getTime();
    sim.scheduler()
        .schedule(
            sim.joints()
                .of(TURRET)
                .follow(
                    () ->
                        Radians.of(
                            Math.toRadians(40)
                                * Math.sin(
                                    2 * Math.PI * (RobotController.getTime() - start) / 4e9))));
    JointSimulation truth = Objects.requireNonNull(sim.world().joints());

    double worst = 0;
    double widest = 0;
    for (double[] leg : ROUTE) {
      sim.driver().setLeftY(-leg[1]);
      sim.driver().setLeftX(-leg[2]);
      sim.driver().setRightX(-leg[3]);
      sim.driver().notifyNewData();
      for (int i = 0; i < RobotLoop.loops(leg[0]); i++) {
        sim.loops(1);
        worst = Math.max(worst, error());
        widest = Math.max(widest, Math.abs(Math.toDegrees(truth.position(TURRET))));
      }
    }

    assertThat(widest).as("the turret swept").isGreaterThan(30);
    assertThat(worst).as("worst while driving").isLessThan(0.15);
    assertThat(error()).as("stopped").isLessThan(0.05);
    LocalizationEstimate.Health health = sim.robot().estimate().health();
    assertThat(health.accepted()).isGreaterThan(10 * health.rejected()).isGreaterThan(100);
  }
}

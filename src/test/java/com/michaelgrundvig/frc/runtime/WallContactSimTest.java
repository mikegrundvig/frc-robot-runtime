package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The whole robot program driven into REBUILT's alliance wall and backed off it, with sensors that
 * err as a real robot's do and its cameras on: what the wall does to the robot, and what the
 * robot's own estimate makes of it.
 */
@SimulationTest
class WallContactSimTest {
  /**
   * In the alliance zone, back to the blue alliance wall, before the bare stretch of it between the
   * outpost's face and the tower's.
   */
  private static final Pose2d START = new Pose2d(1.5, 2.45, Rotation2d.ZERO);

  /** From the robot's center to the back of its bumpers. */
  private static final double HALF_LENGTH =
      ExampleRobot.SIM.definition().body().bumperLength().in(Meters) / 2;

  @RegisterExtension final RobotSim sim = new RobotSim();

  /** What a stretch of driving showed, loop by loop. */
  private record Stretch(double nearestTheWall, double worstErrorMeters, boolean feltACollision) {}

  /** Holds the driver's stick there for some loops, and watches the truth and the estimate. */
  private Stretch drive(double stickAwayFromTheDrivers, int loops) {
    sim.driver().setLeftY(-stickAwayFromTheDrivers);
    sim.driver().notifyNewData();
    double nearest = Double.POSITIVE_INFINITY;
    double worstError = 0;
    boolean felt = false;
    for (int loop = 0; loop < loops; loop++) {
      sim.loops(1);
      Pose2d truth = sim.simulation().truePose();
      nearest = Math.min(nearest, truth.getX());
      worstError = Math.max(worstError, error());
      felt |= sim.estimate().health().collisionActive();
    }
    return new Stretch(nearest, worstError, felt);
  }

  private double error() {
    Pose2d truth = sim.simulation().truePose();
    return sim.robot().drive().getPose().getTranslation().getDistance(truth.getTranslation());
  }

  /**
   * Backed into the wall at about 1 m/s and held there for two seconds, wheels turning, then driven
   * off it and stopped.
   *
   * <p>The wall stops the robot's bumpers on its face, and the IMU feels the stop, which the
   * estimate takes for the collision it is. While the robot is held there its wheels keep counting,
   * about 2.2 m in all, and the estimate runs on past the wall with them until the cameras disagree
   * by enough to pull it back. It was 1.14 m off at worst when this was written, which is the
   * estimator's to improve and is held here only to less than the wheels counted. Once the robot
   * drives off the wall the estimate is back on it.
   */
  @Test
  void theWallStopsTheRobotAndTheEstimateComesBackWhenItBacksOff() {
    // The season's own field, hub, walls and all: the tests' usual field has nothing standing.
    sim.build(ExampleRobot.SIM, Rebuilt::game, ControlsMaker.NONE, SensorErrors.TYPICAL);
    sim.loops(1);
    sim.simulation().place(START);
    sim.robot().localization().resetPose(START);
    sim.loops(5);
    RobotSim.setEnabled(true);

    Stretch intoTheWall = drive(-0.6, 150);
    assertThat(intoTheWall.nearestTheWall())
        .as("the bumpers stop on the wall's face")
        .isBetween(HALF_LENGTH - 1e-6, HALF_LENGTH + 0.01);
    assertThat(intoTheWall.feltACollision()).as("the IMU felt the stop").isTrue();
    assertThat(intoTheWall.worstErrorMeters())
        .as("the estimate, while the wheels spin against the wall")
        .isLessThan(1.5);

    drive(0.6, 75);
    Stretch stopped = drive(0, 50);
    assertThat(sim.simulation().truePose().getX()).as("driven off the wall").isGreaterThan(1.5);
    assertThat(stopped.worstErrorMeters())
        .as("the estimate, once it is off the wall")
        .isLessThan(0.05);
  }
}

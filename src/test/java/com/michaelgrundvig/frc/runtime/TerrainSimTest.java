package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.simulation.SensorErrors;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * Over REBUILT's bump with no cameras: the terrain tilts the simulated robot, and odometry counts
 * the wheels' travel along the ramps instead of throwing it away.
 */
@SimulationTest
class TerrainSimTest {
  /** Facing the blue left bump from the alliance zone, on its middle. */
  private static final Pose2d BEFORE_THE_BUMP = new Pose2d(3.0, 5.5, Rotation2d.ZERO);

  @RegisterExtension final RobotSim sim = new RobotSim();

  @BeforeEach
  void setUp() {
    sim.build(RobotSim.WITHOUT_CAMERAS, SensorErrors.NONE);
  }

  /**
   * Sets the robot before the bump, the estimate agreeing, knocks it back as a hit does (felt, but
   * not how far), then drives it over at 1 m/s.
   */
  private void crossTheBump(Transform2d hit) {
    sim.loops(1);
    sim.simulation().place(BEFORE_THE_BUMP);
    sim.robot().localization().resetPose(BEFORE_THE_BUMP);
    sim.loops(5);
    if (!hit.equals(Transform2d.ZERO)) {
      sim.simulation().hit(hit);
    }
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-0.6);
    sim.driver().notifyNewData();
    sim.run(4.0);
    sim.driver().setLeftY(0);
    sim.driver().notifyNewData();
    sim.run(0.5);
  }

  private double error() {
    return sim.robot()
        .drive()
        .getPose()
        .getTranslation()
        .getDistance(sim.simulation().truePose().getTranslation());
  }

  @Test
  void theRampsTiltTheRobotAndOdometryKeepsItsTravel() {
    crossTheBump(Transform2d.ZERO);

    assertThat(sim.simulation().truePose().getX()).as("over the bump").isGreaterThan(5.2);
    assertThat(error()).as("after crossing, no cameras").isLessThan(0.03);
  }
}

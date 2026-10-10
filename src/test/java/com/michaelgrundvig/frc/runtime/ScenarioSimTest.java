package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.localization.YawSource;
import com.michaelgrundvig.frc.runtime.sim.Scenario;
import com.michaelgrundvig.frc.runtime.sim.Scenarios;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;

/**
 * The simulator's scenarios on the whole robot, cameras and all, started as {@code ./gradlew run
 * -Pscenario=<name>} starts them: built in the scenario, enabled on its start, then the scenario's
 * event. The robot's code is never told; it finds out from its sensors.
 */
@SimulationTest
class ScenarioSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  static Stream<String> scenarios() {
    return Scenarios.ALL.stream().map(Scenario::name);
  }

  /** Builds the robot in the scenario and enables it on its start, ready for the scenario. */
  private Scenario enabledIn(String name) {
    Scenario scenario = Scenarios.named(name);
    sim.build(scenario);
    RobotSim.setEnabled(true);
    sim.loop(); // placed on its start
    return scenario;
  }

  /** Every scenario in the catalog runs on the simulator's robot, whatever it does to it. */
  @ParameterizedTest
  @MethodSource("scenarios")
  void everyScenarioRunsOnTheRobot(String name) {
    sim.start(enabledIn(name));
    sim.run(0.5);

    Pose2d pose = sim.robot().localization().pose();
    assertThat(Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())).isTrue();
  }

  /** Knocked 60 cm with a jolt: the robot feels the hit, and its cameras put the estimate back. */
  @Test
  void aCollisionIsFoundByTheCameras() {
    Scenario collision = enabledIn("collision");
    Pose2d before = sim.simulation().truePose();

    sim.start(collision);

    assertThat(sim.simulation().truePose().getTranslation().getDistance(before.getTranslation()))
        .isGreaterThan(0.5);
    sim.run(1.0);
    // The hit leaves the heading about a degree off, and the cameras' positions lean on it: on this
    // exact field their headings are better than the 2° each may be off by on a real one, so the
    // heading, and the position with it, take a little longer to come back (5.2 cm at 1 s).
    assertThat(error()).isLessThan(0.06);
  }

  /** The gyro unplugged: the robot turns to its backup IMU for its heading. */
  @Test
  void anUnpluggedGyroTurnsTheRobotToItsBackupImu() {
    sim.start(enabledIn("gyro-unplugged"));
    sim.run(0.5);

    assertThat(sim.robot().localization().yawSource()).isEqualTo(YawSource.BACKUP);
  }

  /** Both IMUs unplugged: the heading rests on the wheels. */
  @Test
  void bothImusUnpluggedLeaveTheHeadingToTheWheels() {
    sim.start(enabledIn("imus-unplugged"));
    sim.run(0.5);

    assertThat(sim.robot().localization().yawSource()).isEqualTo(YawSource.WHEELS);
  }

  /** Carried to a bump: the cameras find it there, and forward drives it over. */
  @Test
  void aBumpCrossingIsFoundByTheCamerasAndDrivenOver() {
    sim.start(enabledIn("bump-crossing"));
    sim.run(1.5); // lost, until the cameras agree on where it is
    assertThat(error()).as("found by the cameras").isLessThan(0.05);

    sim.driver().setLeftY(-0.6); // pushed forward: blue's forward, toward the bump
    sim.driver().notifyNewData();
    sim.run(4.0);

    assertThat(sim.simulation().truePose().getX()).as("over the bump").isGreaterThan(5.2);
    assertThat(error()).as("still found, after the crossing").isLessThan(0.05);
  }

  private double error() {
    return sim.robot()
        .localization()
        .pose()
        .getTranslation()
        .getDistance(sim.simulation().truePose().getTranslation());
  }
}

package com.michaelgrundvig.frc.runtime.examples;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/** The whole-robot test docs/testing.md shows: the driver's left stick drives the robot away. */
@SimulationTest
class DrivingSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void theLeftStickDrivesAwayFromTheDrivers() {
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1.0); // pushed forward
    sim.driver().notifyNewData();

    sim.run(1.5); // seconds of robot time, loop by loop

    assertThat(sim.drive().getPose().getX()).isGreaterThan(5.0);
  }
}

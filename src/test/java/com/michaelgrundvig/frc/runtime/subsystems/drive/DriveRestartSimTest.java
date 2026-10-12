package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.localization.OdometrySample;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A drive motor's controller restarts while the robot drives: a breaker that reset. It counts its
 * wheel from zero again, which, taken as a reading like any other, is that wheel rolling all the
 * way back to where the robot started. The module says its drive restarted ({@code ModuleIO}'s M16)
 * and the drivetrain leaves that wheel out of odometry for the one loop, so the estimate does not
 * move.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class DriveRestartSimTest {
  /** The module whose drive restarts: the back left. */
  private static final int RESTARTED = 2;

  /**
   * How far the estimate may be from where the robot really is, in meters, at any loop of the
   * drive. With perfect sensors it stays within a millimeter or two. A wheel taken to have rolled
   * back the two meters driven would put it off by about half a meter.
   */
  private static final double CLOSE_METERS = 0.005;

  /** Whether every one of a loop's readings of the wheels has a wheel, or none has. */
  private enum Reported {
    IN_EVERY_READING,
    IN_NO_READING,
    IN_SOME_READINGS
  }

  private static Reported reported(List<OdometrySample> readings, int wheel) {
    int has = 0;
    for (OdometrySample reading : readings) {
      has += reading.reported(wheel) ? 1 : 0;
    }
    if (has == readings.size()) {
      return Reported.IN_EVERY_READING;
    }
    return has == 0 ? Reported.IN_NO_READING : Reported.IN_SOME_READINGS;
  }

  /**
   * Driving straight ahead, two meters on, the back left's drive restarts and the robot drives on.
   * The estimate stays on the true pose through the restart and after it. The wheel is out of the
   * loop that reads the restart, and back in every reading of the next.
   */
  @Test
  void aRestartedDriveControllerDoesNotMoveTheEstimate() {
    try (DriveSimHarness robot = new DriveSimHarness(ExampleRobot.SIM)) {
      RobotSim.setEnabled(true);
      robot.sticks(0.5, 0, 0);
      robot.scheduler.schedule(robot.drive.driven());
      double farthestOff = 0;
      while (robot.simulation.truePose().getX() < 2) {
        robot.loop();
        farthestOff = Math.max(farthestOff, robot.positionError());
      }
      double offBefore = farthestOff;

      robot.simulation.restartDrive(RESTARTED);
      robot.loop();
      List<OdometrySample> asItRestarted = robot.drive.readings().wheels();
      double offAsItRestarted = robot.positionError();
      robot.loop();
      List<OdometrySample> theLoopAfter = robot.drive.readings().wheels();
      farthestOff = Math.max(offAsItRestarted, robot.positionError());
      for (int i = 0; i < RobotLoop.loops(1); i++) {
        robot.loop();
        farthestOff = Math.max(farthestOff, robot.positionError());
      }

      assertThat(robot.simulation.truePose().getX()).as("it drove on").isGreaterThan(2.5);
      assertThat(offBefore).as("on the true pose before the restart").isLessThan(CLOSE_METERS);
      assertThat(asItRestarted).as("the loop that read the restart has readings").isNotEmpty();
      assertThat(reported(asItRestarted, RESTARTED))
          .as("the restarted wheel, in the loop that read the restart")
          .isEqualTo(Reported.IN_NO_READING);
      assertThat(reported(asItRestarted, 0))
          .as("another wheel, in that loop")
          .isEqualTo(Reported.IN_EVERY_READING);
      assertThat(theLoopAfter).isNotEmpty();
      assertThat(reported(theLoopAfter, RESTARTED))
          .as("the restarted wheel, the loop after")
          .isEqualTo(Reported.IN_EVERY_READING);
      assertThat(offAsItRestarted)
          .as("how far off the estimate is in the loop that read the restart, in meters")
          .isLessThan(CLOSE_METERS);
      assertThat(farthestOff)
          .as("the farthest off the estimate was from the restart on, in meters")
          .isLessThan(CLOSE_METERS);
    }
  }
}

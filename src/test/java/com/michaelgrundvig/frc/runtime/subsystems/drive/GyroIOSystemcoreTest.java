package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.RobotSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.simulation.OnboardIMUSim;
import org.wpilib.simulation.SimHooks;

/**
 * Systemcore's onboard IMU, off a robot: WPILib's simulation doesn't model it (it reads zero), so
 * this checks only that the IO reads it as the drivetrain expects, one reading a loop.
 */
@ExtendWith(RobotSim.class)
class GyroIOSystemcoreTest {
  @Test
  void itReportsOneReadingALoopAndCountsAsConnected() {
    RobotSim.setEnabled(false);
    GyroIOSystemcore flat = new GyroIOSystemcore(Rotation3d.ZERO);
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    flat.updateInputs(inputs);
    SimHooks.stepTiming(0.02);
    flat.updateInputs(inputs);

    assertThat(inputs.connected).isTrue();
    assertThat(inputs.odometryTimestampsNanos).hasSize(1);
    assertThat(inputs.odometryYawsRad).containsExactly(inputs.yaw.getRadians());
    assertThat(inputs.yawVelocityRadPerSec).isFinite();
  }

  /**
   * The board reads along its own axes, which its orientation on the robot turns into the robot's.
   * Turned a quarter turn left, flat, it reads the robot's forward on its -y axis; on its long
   * edge, its x axis up and z axis backward, it reads the robot's forward on its -z axis and the
   * floor holding it up on its x axis.
   */
  @Test
  void theBoardsAccelerationIsTurnedToTheRobotsAxesByItsOrientation() {
    Rotation3d turnedLeft = new Rotation3d(0, 0, Math.PI / 2);
    Rotation3d onEdge = new Rotation3d(0, -Math.PI / 2, 0);

    Translation3d forward = GyroIOSystemcore.robotAxes(turnedLeft, 0, -1, 0);
    Translation3d upright = GyroIOSystemcore.robotAxes(onEdge, 9.81, 0, -1);

    assertThat(forward.getDistance(new Translation3d(1, 0, 0))).isLessThan(1e-12);
    assertThat(upright.getDistance(new Translation3d(1, 0, 9.81))).isLessThan(1e-12);
  }

  /**
   * Nose up 15° on a bump, the board turned a quarter turn left on the robot: the board feels the
   * tilt about its own x axis, a roll, which is the robot's pitch. Read as it is, the robot would
   * seem rolled 15° and level fore and aft, and gravity's share would land on the wrong axis: 2.5
   * m/s² read as the robot sliding sideways, the gyro lost and the backup turning odometry.
   */
  @Test
  void aTurnedBoardsTiltIsTheRobotsAboutItsOwnAxes() {
    OnboardIMUSim.setAngleX(Math.toRadians(-15));
    OnboardIMUSim.setAngleY(0);
    GyroIOSystemcore turned = new GyroIOSystemcore(new Rotation3d(0, 0, Math.PI / 2));
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    turned.updateInputs(inputs);
    OnboardIMUSim.setAngleX(0);

    assertThat(Math.toDegrees(inputs.pitchRad)).as("nose up").isCloseTo(-15, within(1e-9));
    assertThat(Math.toDegrees(inputs.rollRad)).isCloseTo(0, within(1e-9));
  }

  /**
   * How it's mounted follows from how the board is turned on the robot: flat, face up or face down;
   * stood on its long side, landscape; stood on its short side, portrait.
   */
  @Test
  void itsMountFollowsFromHowTheBoardIsTurned() {
    assertThat(GyroIOSystemcore.mount(Rotation3d.ZERO)).isEqualTo(OnboardIMU.MountOrientation.FLAT);
    assertThat(GyroIOSystemcore.mount(new Rotation3d(Math.PI, 0, Math.PI / 2)))
        .as("upside down, turned")
        .isEqualTo(OnboardIMU.MountOrientation.FLAT);
    assertThat(GyroIOSystemcore.mount(new Rotation3d(Math.PI / 2, 0, 0)))
        .as("rolled onto its long side")
        .isEqualTo(OnboardIMU.MountOrientation.LANDSCAPE);
    assertThat(GyroIOSystemcore.mount(new Rotation3d(0, Math.PI / 2, 0)))
        .as("pitched onto its short side")
        .isEqualTo(OnboardIMU.MountOrientation.PORTRAIT);
  }
}

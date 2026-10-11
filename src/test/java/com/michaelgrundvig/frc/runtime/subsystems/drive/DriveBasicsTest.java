package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.BodyMotion;
import com.michaelgrundvig.frc.localization.OdometrySample;
import com.michaelgrundvig.frc.localization.SwerveKinematicsModel;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.world.field.Heading;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/** Does the drivetrain go where the driver points it? */
@ExtendWith(RobotSim.class)
class DriveBasicsTest {
  private DriveSimHarness harness;

  @BeforeEach
  void setUp() {
    harness = new DriveSimHarness(ExampleRobot.SIM);
  }

  @AfterEach
  void tearDown() {
    harness.close();
  }

  @Test
  void drivesStraightForward() {
    harness.drive(2.0, 1.0, 0.0, 0.0);

    Pose2d pose = harness.drive.getPose();
    assertTrue(pose.getX() > 5.0, "moved " + pose.getX() + " m forward");
    assertEquals(0.0, pose.getY(), 0.02, "no sideways drift");
    assertEquals(0.0, pose.getRotation().getDegrees(), 0.5, "no turning");
    double topSpeed = ExampleRobot.SIM.drivetrain().limits().maxLinearSpeed().baseUnitMagnitude();
    assertEquals(topSpeed, harness.drive.getRobotRelativeVelocities().vx, 0.05, "full stick");
  }

  @Test
  void spinsInPlace() {
    harness.drive(1.0, 0.0, 0.0, 1.0);

    assertEquals(0.0, harness.drive.getPose().getTranslation().getNorm(), 0.05, "stays put");
    assertTrue(harness.drive.getRobotRelativeVelocities().omega > 5.0, "turning at full rate");
  }

  @Test
  void drivesStraightDownfieldWhileSpinning() {
    // Field-relative driving: the robot spins, but its path should still run along +x. Broken
    // field-relative driving spirals off by meters. The wheels get new angles once a loop and take
    // time to reach them, so allow some drift.
    harness.drive(3.0, 0.7, 0.0, 0.5);

    Pose2d pose = harness.drive.getPose();
    assertTrue(pose.getX() > 2.0, "made progress downfield: " + pose.getX() + " m");
    assertEquals(0.0, pose.getY(), 0.25, "path stayed straight");
  }

  @Test
  void drivesForwardForAWhileThenStops() {
    harness.run(harness.drive.driveForward(0.5, 1.0), 1.5);

    Pose2d pose = harness.drive.getPose();
    assertEquals(0.5, pose.getX(), 0.05, "half a meter at 0.5 m/s for a second");
    assertEquals(0.0, pose.getY(), 0.02, "straight");
    assertEquals(0.0, harness.drive.getRobotRelativeVelocities().vx, 0.05, "stopped");
  }

  /** AdvantageScope turns the schematic robot's wheels by these: where each is, as it points. */
  @Test
  void eachWheelsPoseIsWhereItIsTurnedAsItsModuleMeasures() {
    harness.drive(1.0, 0.0, 0.5, 0.0); // sideways, to the left

    Pose3d[] wheels = harness.drive.modulePoses();

    Translation2d[] corners = ExampleRobot.SIM.definition().moduleTranslations();
    double radius = ExampleRobot.SIM.definition().drivetrain().module(0).wheelRadius().in(Meters);
    assertEquals(4, wheels.length);
    for (int i = 0; i < 4; i++) {
      assertEquals(corners[i].getX(), wheels[i].getX(), 1e-9, "module " + i + " forward");
      assertEquals(corners[i].getY(), wheels[i].getY(), 1e-9, "module " + i + " left");
      assertEquals(radius, wheels[i].getZ(), 1e-9, "the wheel's center, a radius up");
      // Pointing left, or right with the wheel rolling backward.
      assertEquals(1.0, Math.abs(Math.sin(wheels[i].getRotation().getZ())), 0.01, "sideways");
    }
  }

  @Test
  void needsExactlyFourModules() {
    assertThrows(
        IllegalArgumentException.class,
        () -> hardware(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}));
  }

  /**
   * An IMU that sends values that aren't numbers is reported as it read, for localization to judge
   * (it counts as lost there), and the drivetrain runs on.
   */
  @Test
  void anImuThatSendsNaNIsReportedAsUnusable() {
    GyroIO sendingNaN =
        new GyroIO() {
          @Override
          public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = true;
            inputs.yaw = new Rotation2d(Double.NaN);
            inputs.pitchRad = Double.NaN;
            inputs.rollRad = Double.NaN;
            inputs.odometryTimestampsNanos = new long[] {1};
            inputs.odometryYawsRad = new double[] {Double.NaN};
          }
        };
    try (Drive drive = drive("NaNImu", hardware(sendingNaN, fourModules()))) {
      drive.periodic(true);

      assertTrue(drive.readings().gyro().reporting());
      assertFalse(drive.readings().gyro().usable());
    }
  }

  /** A device that missed its configuration runs without current limits: the pit must know. */
  @Test
  void aModuleWhoseDevicesMissedTheirConfigurationRaisesItsAlert() {
    ModuleIO missed =
        new ModuleIO() {
          @Override
          public void updateInputs(ModuleIOInputs inputs) {
            inputs.unconfigured = "drive motor 1";
          }
        };
    DriveHardware hardware =
        hardware(new GyroIO() {}, missed, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
    try (Drive drive = drive("Unconfigured", hardware)) {
      drive.periodic(false);

      assertTrue(drive.isModuleUnconfiguredAlertActive(0));
      assertFalse(drive.isModuleUnconfiguredAlertActive(1));
    }
  }

  /**
   * A drivetrain the robot hasn't handed the driver's sticks has nothing to drive from. Driving
   * with the sticks and an assist both read the sticks when they run, so each is built all the same
   * ({@code DriveAssistSticksSimTest} runs them), and an assist's gate says what to do until then.
   */
  @Test
  void commandsOnTheSticksAreBuiltBeforeTheRobotHandsThemOver() {
    try (Drive drive = drive("NoSticks", hardware(new GyroIO() {}, fourModules()))) {
      Command driven = drive.driven();
      Gate aimed = drive.facing(Heading.field(Rotation2d.ZERO)).aimed();

      assertEquals("Drive with the sticks", driven.name());
      assertFalse(aimed.getAsBoolean());
      assertTrue(aimed.why().contains("drive.takeDriver"));
    }
  }

  /**
   * Wheels that report connected but send no readings have lost what reads them (on a robot, the
   * odometry thread): the pose would freeze, so the pit must know.
   */
  @Test
  void wheelsThatStopSendingReadingsRaiseAnAlert() {
    ModuleIO[] connectedButSilent = new ModuleIO[4];
    for (int i = 0; i < 4; i++) {
      connectedButSilent[i] =
          new ModuleIO() {
            @Override
            public void updateInputs(ModuleIOInputs inputs) {
              inputs.driveConnected = true;
              inputs.steerConnected = true;
            }
          };
    }
    try (Drive drive = drive("Silent", hardware(new GyroIO() {}, connectedButSilent))) {
      drive.periodic(true);
      boolean afterOneLoop = drive.isOdometryStoppedAlertActive();
      for (int i = 0; i < 10; i++) {
        drive.periodic(true);
      }

      assertFalse(afterOneLoop, "a loop without readings can happen");
      assertTrue(drive.isOdometryStoppedAlertActive());
    }
  }

  /**
   * A module whose motor controllers are off the bus reads what it last sent: its wheel standing
   * still, at the speed it had then. The robot drives on the other three, and so do its odometry
   * and its measured speed: driving straight ahead at 1 m/s, the readings leave that wheel out, the
   * fit from the rest finds the robot's 2 cm a loop, and the measured speed is 1 m/s.
   */
  @Test
  void aDisconnectedModuleIsLeftOutOfOdometryAndTheMeasuredSpeed() {
    double radius = ExampleRobot.SIM.definition().drivetrain().module(0).wheelRadius().in(Meters);
    long[] clock = {1_000_000};
    double[] rolled = {0};
    ModuleIO[] modules = new ModuleIO[4];
    for (int i = 0; i < 4; i++) {
      boolean silent = i == 2;
      modules[i] =
          new ModuleIO() {
            @Override
            public void updateInputs(ModuleIOInputs inputs) {
              double wheel = silent ? 0 : rolled[0] / radius;
              inputs.driveConnected = !silent;
              inputs.steerConnected = !silent;
              inputs.drivePositionRad = wheel;
              inputs.driveVelocityRadPerSec = (silent ? 3.0 : 1.0) / radius;
              inputs.odometryTimestampsNanos = new long[] {clock[0]};
              inputs.odometryDrivePositionsRad = new double[] {wheel};
              inputs.odometrySteerAnglesRad = new double[] {0};
            }
          };
    }
    try (Drive drive = drive("OnThree", hardware(new GyroIO() {}, modules))) {
      drive.periodic(true);
      OdometrySample before = drive.readings().wheels().get(0);
      clock[0] += 20_000_000;
      rolled[0] += 0.02;
      drive.periodic(true);
      OdometrySample after = drive.readings().wheels().get(0);
      BodyMotion moved =
          new SwerveKinematicsModel(ExampleRobot.SIM.definition().moduleTranslations())
              .motion(before, after);

      assertFalse(after.reported(2), "the silent module's reading");
      assertTrue(after.reported(0));
      assertEquals(0.02, moved.dxMeters(), 1e-9, "forward");
      assertEquals(0.0, moved.dyMeters(), 1e-9, "sideways");
      assertEquals(0.0, moved.dthetaRadians(), 1e-9, "turned");
      assertEquals(1.0, drive.getRobotRelativeVelocities().vx, 1e-9, "measured speed");
      assertEquals(0.0, drive.getRobotRelativeVelocities().omega, 1e-9, "measured turning");
    }
  }

  private Drive drive(String name, DriveHardware hardware) {
    return new Drive(
        name,
        hardware,
        ExampleRobot.SIM.definition(),
        ExampleRobot.SIM::drivetrain,
        harness.scheduler,
        () -> Pose2d.ZERO);
  }

  private static ModuleIO[] fourModules() {
    return new ModuleIO[] {
      new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}
    };
  }

  private static DriveHardware hardware(GyroIO gyro, ModuleIO... modules) {
    return DriveHardware.builder()
        .modules(List.of(modules))
        .gyro(gyro)
        .backupImu(new GyroIO() {})
        .odometryLock(new ReentrantLock())
        .build();
  }
}

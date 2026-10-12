package com.michaelgrundvig.frc.runtime.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.OdometrySample;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.localization.YawSamples;
import com.michaelgrundvig.frc.localization.YawSource;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Localization fed by hand: what it makes of the drivetrain's readings, one loop at a time. */
@ExtendWith(RobotSim.class)
class LocalizationTest {
  private static final ImuReadings LEVEL =
      ImuReadings.builder()
          .reporting(true)
          .yawRadians(0.2)
          .yawRateRadiansPerSecond(0)
          .pitchRadians(0.05)
          .rollRadians(0)
          .forwardAccelerationMetersPerSecSq(0)
          .leftAccelerationMetersPerSecSq(0)
          .yaws(YawSamples.of(new long[] {1_000_000L}, new double[] {0.2}))
          .build();

  private static final ImuReadings SENDING_NAN =
      ImuReadings.builder()
          .reporting(true)
          .yawRadians(Double.NaN)
          .yawRateRadiansPerSecond(Double.NaN)
          .pitchRadians(Double.NaN)
          .rollRadians(Double.NaN)
          .forwardAccelerationMetersPerSecSq(Double.NaN)
          .leftAccelerationMetersPerSecSq(Double.NaN)
          .yaws(YawSamples.of(new long[] {1_000_000L}, new double[] {Double.NaN}))
          .build();

  private static DrivetrainReadings readings(ImuReadings gyro, ImuReadings backup) {
    return readings(gyro, backup, 1_000_000L);
  }

  private static DrivetrainReadings readings(ImuReadings gyro, ImuReadings backup, long nanos) {
    return DrivetrainReadings.builder()
        .wheels(
            List.of(
                new OdometrySample(nanos, new double[] {0, 0, 0, 0}, new double[] {0, 0, 0, 0})))
        .gyro(gyro)
        .backupImu(backup)
        .velocities(new ChassisVelocities())
        .build();
  }

  /** One bad reading from an IMU costs its readings, never the robot program. */
  @Test
  void aGyroSendingNaNCountsAsLostAndTheBackupStandsIn() {
    try (Localization localization =
        DriveSimHarness.localizationFor(
            ExampleRobot.SIM, DriveSimHarness.estimatorAt(Pose2d.ZERO))) {
      localization.periodic(readings(SENDING_NAN, ImuReadings.NONE), true);
      localization.update();

      assertThat(localization.isGyroDisconnectedAlertActive()).isTrue();
      assertThat(localization.yawSource()).contains(YawSource.WHEELS);
      assertThat(localization.pitchRadians()).as("no IMU, no tilt").isZero();
      assertThat(localization.pose().getX()).isFinite();

      localization.periodic(readings(SENDING_NAN, LEVEL), true);
      localization.update();

      assertThat(localization.isGyroDisconnectedAlertActive()).isFalse();
      assertThat(localization.isUsingBackupAlertActive()).isTrue();
      assertThat(localization.pitchRadians()).as("the backup's attitude").isEqualTo(0.05);
    }
  }

  /**
   * A robot with more cameras than the estimator keeps apart: the ones after the eighth count each
   * image's errors as new, and an alert names them.
   */
  @Test
  void moreCamerasThanTheEstimatorKeepsApartRaiseAnAlert() {
    try (Localization localization =
        DriveSimHarness.localizationFor(
            ExampleRobot.SIM, DriveSimHarness.estimatorAt(Pose2d.ZERO))) {
      localization.periodic(readings(LEVEL, ImuReadings.NONE), true);
      localization.periodic(readings(LEVEL, ImuReadings.NONE, 21_000_000L), true);
      localization.update();
      assertThat(localization.isCamerasWithoutPlacesAlertActive()).isFalse();

      for (int camera = 1; camera <= 9; camera++) {
        localization
            .service()
            .camera(
                new VisionMeasurement(
                    11_000_000L,
                    Pose2d.ZERO,
                    PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
                    VisionMeasurement.Lever.NONE,
                    "camera " + camera,
                    List.of(26),
                    VisionMeasurement.Solver.HEADING_CONSTRAINED));
      }
      localization.update();

      assertThat(localization.isCamerasWithoutPlacesAlertActive()).isTrue();
    }
  }

  /** The 99.9% ellipse AdvantageScope draws: closed, and 3.72 standard deviations out. */
  @Test
  void theEllipseIsAClosedOutlineAroundThePosition() {
    try (Localization localization =
        DriveSimHarness.localizationFor(
            ExampleRobot.SIM, DriveSimHarness.estimatorAt(Pose2d.ZERO))) {
      Pose2d at = new Pose2d(3, 2, Rotation2d.fromDegrees(40));

      Pose2d[] outline = localization.ellipse(at, PoseCovariance.fromStdDevs(0.1, 0.1, 0.01));

      assertThat(outline).hasSize(17);
      assertThat(outline[16]).isEqualTo(outline[0]);
      for (Pose2d point : outline) {
        assertThat(point.getTranslation().getDistance(at.getTranslation()))
            .isCloseTo(0.1 * Math.sqrt(-2 * Math.log(0.001)), within(1e-9));
      }
      assertThat(localization.ellipse(at, PoseCovariance.fromStdDevs(10, 10, 1)))
          .as("wider than the field: nothing to draw")
          .isEmpty();
      assertThat(localization.service().pose3d(at))
          .as("on flat ground, level")
          .isEqualTo(new Pose3d(3, 2, 0, new Rotation3d(0, 0, at.getRotation().getRadians())));
    }
  }
}

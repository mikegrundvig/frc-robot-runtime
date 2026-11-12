package com.michaelgrundvig.frc.runtime.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.OdometrySample;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.localization.YawSamples;
import com.michaelgrundvig.frc.localization.YawSource;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.examples.LocalizerExamples;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
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

  /**
   * Lost while the heading is still a placement's, which no camera has backed, the pit's line says
   * the heading is the suspect and what to do: two images in a row put the robot facing the other
   * way. Lost after a driver's heading reset, or with the placement backed, it says only that it is
   * recovering.
   */
  @Test
  void lostAfterAPlacementTheHeadingIsTheSuspect() {
    enum Word {
      PLACED,
      PLACED_AND_RESET
    }
    record Row(Word word, String startsWith) {}
    List<Row> rows =
        List.of(
            new Row(Word.PLACED, Localization.LOST_SINCE_PLACED),
            new Row(Word.PLACED_AND_RESET, "Lost: recovering: within"));
    Pose2d placed = new Pose2d(3, 2, Rotation2d.ZERO);
    Pose2d backwards = new Pose2d(3, 2, Rotation2d.k180deg);
    for (Row row : rows) {
      try (Localization localization =
          DriveSimHarness.localizationFor(
              ExampleRobot.SIM, DriveSimHarness.estimatorAt(Pose2d.ZERO))) {
        localization.periodic(readings(LEVEL, ImuReadings.NONE), true);
        localization.place(placed, PoseCovariance.fromStdDevs(0.05, 0.05, 0.035), "placed");
        if (row.word() == Word.PLACED_AND_RESET) {
          localization.resetHeading(Rotation2d.ZERO, 0.035);
        }
        localization.periodic(readings(LEVEL, ImuReadings.NONE, 21_000_000L), true);
        localization.periodic(readings(LEVEL, ImuReadings.NONE, 41_000_000L), true);
        localization.update();
        for (long taken : new long[] {25_000_000L, 35_000_000L}) {
          localization
              .service()
              .camera(
                  new VisionMeasurement(
                      taken,
                      backwards,
                      PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2)),
                      VisionMeasurement.Lever.NONE,
                      "front-left",
                      List.of(26, 27),
                      VisionMeasurement.Solver.MULTI_TAG));
        }
        localization.update();

        assertThat(localization.describe()).as(row.word().name()).startsWith(row.startsWith());
      }
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

  /**
   * A localizer that reports too little for the scoring gate to judge by: the robot is ready to
   * score only if its configuration chose that, an alert says the gate isn't judging either way,
   * and the pit's line says which it is. The default localizer is judged, and raises no alert.
   */
  @Test
  void aGateThatCantJudgeSaysWhatTheRobotChose() {
    record Row(
        String name, RobotConfig robot, boolean alert, boolean ready, String scoringStartsWith) {}
    RobotConfig onWpilib = LocalizerExamples.ON_WPILIB;
    List<Row> rows =
        List.of(
            new Row(
                "WPILib's estimator, by default",
                onWpilib,
                true,
                false,
                "Not judged, so not ready: what the pose rests on is not reported"),
            new Row(
                "WPILib's estimator, the robot choosing to score without the gate",
                onWpilib.withScoringWhenNotJudged(ScoringWhenNotJudged.READY),
                true,
                true,
                "Not judged, so ready: what the pose rests on is not reported"),
            new Row("the default localizer", ExampleRobot.SIM, false, false, "Not ready: "));
    assertThat(onWpilib.scoringWhenNotJudgedOrDefault())
        .as("not ready, unless the robot chose otherwise")
        .isEqualTo(ScoringWhenNotJudged.NOT_READY);
    for (Row row : rows) {
      try (Localization localization =
          new Localization(
              new LocalizationService(
                  RobotSim.FIELD,
                  row.robot().definition(),
                  row.robot().localizerOn(RobotSim.FIELD),
                  RobotLoop.PERIOD_SECONDS),
              row.robot().scoringGate(),
              row.robot().scoringWhenNotJudgedOrDefault())) {
        localization.periodic(readings(LEVEL, ImuReadings.NONE), true);
        localization.update();

        assertThat(localization.isNotJudgedAlertActive()).as(row.name()).isEqualTo(row.alert());
        assertThat(localization.readyToScore()).as(row.name()).isEqualTo(row.ready());
        assertThat(localization.scoring()).as(row.name()).startsWith(row.scoringStartsWith());
      }
    }
  }
}

package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/** How the simulated robot's sensors err: realistic numbers only, and each changed by name. */
class SensorErrorsTest {
  @Test
  void errorsMustBeRealistic() {
    assertThatThrownBy(() -> SensorErrors.NONE.withWheelRadiusScale(0))
        .hasMessage(
            "SensorErrors.wheelRadiusScale: [0] 0 must be positive; [1] 0 must be positive;"
                + " [2] 0 must be positive; [3] 0 must be positive");
    assertThatThrownBy(
            () ->
                SensorErrorsBuilder.builder(SensorErrors.NONE)
                    .wheelRadiusScale(List.of(1.0, 1.0, 1.0))
                    .build())
        .hasMessage("SensorErrors.wheelRadiusScale: must have 4, has 3");
    assertThatThrownBy(
            () ->
                SensorErrorsBuilder.builder(SensorErrors.NONE)
                    .steerOffsetDegrees(List.of(0.0, 0.0, 0.0, Double.NaN))
                    .build())
        .hasMessage("SensorErrors.steerOffsetDegrees: [3] NaN must be a finite number");
    assertThatThrownBy(
            () ->
                SensorErrors.GyroErrors.builder()
                    .scale(1)
                    .driftDegreesPerMinute(Double.NaN)
                    .noiseDegrees(0)
                    .build())
        .hasMessage("GyroErrors.driftDegreesPerMinute: NaN must be a finite number");
    assertThatThrownBy(
            () ->
                SensorErrorsAccelErrorsBuilder.builder(SensorErrors.AccelErrors.IDEAL)
                    .rangeMetersPerSecSq(0)
                    .build())
        .hasMessageContaining("must be positive, or infinite for no range");
  }

  @Test
  void eachErrorChangesByNameAndLeavesTheRest() {
    SensorErrors.GyroErrors drifting =
        SensorErrors.GyroErrors.builder()
            .scale(1.01)
            .driftDegreesPerMinute(2)
            .noiseDegrees(0)
            .build();
    Transform3d knocked = new Transform3d(new Translation3d(0, 0, -0.1), Rotation3d.ZERO);

    SensorErrors errors =
        SensorErrors.NONE
            .withWheelRadiusScale(0.98)
            .withSteerOffset(2, 1.5)
            .withGyro(drifting)
            .withBackupGyro(SensorErrors.GyroErrors.EXACT)
            .withAccelerometer(SensorErrors.AccelErrors.PIGEON_2)
            .withCameraMountError("front", knocked)
            .withTagErrors(Map.of(7, new Translation3d(0.01, 0, 0)));

    assertThat(errors.wheelRadiusScale()).containsOnly(0.98);
    assertThat(errors.steerOffsetDegrees()).containsExactly(0.0, 0.0, 1.5, 0.0);
    assertThat(errors.gyro()).isEqualTo(drifting);
    assertThat(errors.backupGyro()).isEqualTo(SensorErrors.GyroErrors.EXACT);
    assertThat(errors.accelerometer()).isEqualTo(SensorErrors.AccelErrors.PIGEON_2);
    assertThat(errors.backupAccelerometer()).isEqualTo(SensorErrors.AccelErrors.IDEAL);
    assertThat(errors.cameraMountError("front")).isEqualTo(knocked);
    assertThat(errors.cameraMountError("back")).isEqualTo(Transform3d.ZERO);
    assertThat(errors.tagErrors()).containsOnlyKeys(7);
    assertThat(SensorErrors.NONE.withGyros(drifting).backupGyro()).isEqualTo(drifting);
  }

  /** A Pigeon 2's accelerometer reads no further than 2 g either way. */
  @Test
  void aPigeonsAccelerometerReadsTo2G() {
    assertThat(SensorErrors.AccelErrors.PIGEON_2.rangeMetersPerSecSq())
        .isEqualTo(2 * Physics.STANDARD_GRAVITY);
  }
}

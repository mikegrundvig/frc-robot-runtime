package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.robot.Imus;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A simulated IMU reads the robot's true turn with its errors, and fails as a real one does:
 * unplugged, frozen, erring differently, or reset.
 */
class GyroModelTest {
  private static final double STEP = 0.004;

  private final KinematicPhysics physics = TestRobot.physics();
  private final TestRobot.Controllers controllers = new TestRobot.Controllers();

  private GyroModel gyro(SensorErrors.GyroErrors errors) {
    return new GyroModel(
        physics.chassis(),
        errors,
        new AccelerometerModel(
            physics.chassis(), SensorErrors.AccelErrors.IDEAL, Imus.UNCHECKED.gyro(), 2),
        1);
  }

  private static SensorErrors.GyroErrors errors(double scale, double drift, double noise) {
    return SensorErrors.GyroErrors.builder()
        .scale(scale)
        .driftDegreesPerMinute(drift)
        .noiseDegrees(noise)
        .build();
  }

  /** Carried around a quarter turn: it reads the turn at its scale. */
  private void turnByHand(double degrees) {
    Pose2d pose = physics.chassis().pose();
    physics.apply(
        new Disturbance.Placed(
            new Pose2d(
                pose.getTranslation(), pose.getRotation().plus(Rotation2d.fromDegrees(degrees)))));
  }

  /** It reads every turn at its scale, and drifts with time. */
  @Test
  void itReadsTheTurnAtItsScaleAndDrifts() {
    GyroModel gyro = gyro(errors(1.01, 6, 0));
    turnByHand(90);
    for (int i = 0; i < 2500; i++) {
      physics.step(STEP, controllers); // ten seconds
    }

    assertThat(gyro.sample()).isTrue();
    assertThat(Math.toDegrees(gyro.yawRadians())).isCloseTo(90 * 1.01 + 1, within(1e-9));
  }

  /** Its noise spreads as configured: 0.5° within 10%, from 2,000 readings. */
  @Test
  void itsNoiseSpreadsAsConfigured() {
    GyroModel gyro = gyro(errors(1, 0, 0.5));
    double sumSquares = 0;
    int n = 2000;
    for (int i = 0; i < n; i++) {
      gyro.sample();
      sumSquares += Math.pow(Math.toDegrees(gyro.yawRadians()), 2);
    }

    assertThat(Math.sqrt(sumSquares / n)).isCloseTo(0.5, within(0.05));
  }

  /**
   * Unplugged it takes no readings and its accelerometer reads nothing; frozen it reads what it
   * last did; reset it reads zero where the robot points; erring anew, it carries on from where it
   * was.
   */
  @Test
  void itFailsAsARealOneDoes() {
    GyroModel gyro = gyro(errors(1, 0, 0));
    turnByHand(30);
    gyro.setConnected(false);
    assertThat(gyro.isConnected()).isFalse();
    assertThat(gyro.sample()).isFalse();
    assertThat(gyro.acceleration()).isNull();
    gyro.setConnected(true);

    gyro.setFrozen(true);
    gyro.sample();
    turnByHand(30);
    gyro.sample();
    assertThat(gyro.isFrozen()).isTrue();
    assertThat(Math.toDegrees(gyro.yawRadians())).isCloseTo(0, within(1e-9));
    gyro.setFrozen(false);
    gyro.sample();
    assertThat(Math.toDegrees(gyro.yawRadians())).isCloseTo(60, within(1e-9));

    gyro.reset();
    gyro.sample();
    assertThat(gyro.yawRadians()).isCloseTo(0, within(1e-12));

    SensorErrors.GyroErrors high = errors(2, 0, 0);
    gyro.setErrors(high);
    gyro.sample();
    assertThat(gyro.yawRadians()).as("carries on").isCloseTo(0, within(1e-12));
    turnByHand(10);
    gyro.sample();
    assertThat(Math.toDegrees(gyro.yawRadians())).isCloseTo(20, within(1e-9));
    assertThat(gyro.errors()).isEqualTo(high);
    assertThat(gyro.built()).isEqualTo(errors(1, 0, 0));
  }

  /**
   * It reads the robot's turn rate at its scale, its attitude as it truly is, and through its
   * accelerometer, a jolt along the robot's axes.
   */
  @Test
  void itReadsTheTurnRateTheAttitudeAndAJolt() {
    GyroModel gyro = gyro(errors(1.5, 0, 0));
    Translation2d[] wheels = TestRobot.wheels();
    for (int i = 0; i < 4; i++) {
      controllers.steer[i] = wheels[i].getAngle().orElseThrow().getRadians() + Math.PI / 2;
    }
    controllers.volts = 2;
    for (int i = 0; i < 100; i++) {
      physics.step(STEP, controllers);
      gyro.feel(STEP);
    }
    gyro.endLoop(0.4);
    physics.apply(new Disturbance.TiltedByHand(0.1, 0.2));
    gyro.jolt(new Translation2d(5, 0));

    assertThat(gyro.yawRateRadiansPerSecond())
        .isCloseTo(1.5 * physics.chassis().yawRateRadiansPerSecond(), within(1e-12));
    assertThat(gyro.pitchRadians()).isEqualTo(0.1);
    assertThat(gyro.rollRadians()).isEqualTo(0.2);
    Translation2d felt = Objects.requireNonNull(gyro.acceleration());
    assertThat(felt.getX()).isGreaterThan(4);
  }
}

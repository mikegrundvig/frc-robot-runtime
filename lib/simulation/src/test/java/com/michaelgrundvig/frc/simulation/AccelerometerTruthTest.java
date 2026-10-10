package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.ImuMount;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The truth suite's accelerometer, as the slip check needs it: with its errors off it reads the
 * specific force physics gives where it sits (gravity's share on a tilt, the turn swinging it
 * around the center, the robot speeding up); with them on, the statistics configured; it clips
 * exactly at its range, and takes a new sample every 10 ms. It rides today's physics, driven by the
 * simplest motor controllers ({@link TestRobot.Controllers}).
 */
class AccelerometerTruthTest {
  private static final double G = Physics.STANDARD_GRAVITY;
  private static final double STEP = 0.004;

  /** Sampled at 100 Hz, but nothing else: no filter, noise, bias, mount error, or range. */
  private static final SensorErrors.AccelErrors EXACT_SAMPLED =
      SensorErrorsAccelErrorsBuilder.builder(SensorErrors.AccelErrors.IDEAL).sampleHz(100).build();

  private final KinematicPhysics physics = TestRobot.physics();
  private final TestRobot.Controllers controllers = new TestRobot.Controllers();

  /** A part on the robot, mounted level with its x axis forward, here. */
  private AccelerometerModel mounted(SensorErrors.AccelErrors errors, Translation2d place) {
    ImuMount mount =
        ImuMount.builder()
            .forward(Meters.of(place.getX()))
            .left(Meters.of(place.getY()))
            .up(Meters.of(0.08))
            .orientation(Rotation3d.ZERO)
            .accelerometerChecked(true)
            .build();
    return new AccelerometerModel(physics.chassis(), errors, mount, 7);
  }

  /** The physics and the part through this many steps, the part feeling each. */
  private void steps(AccelerometerModel part, int count) {
    for (int i = 0; i < count; i++) {
      physics.step(STEP, controllers);
      part.step(STEP);
    }
  }

  /** Tilted and standing still, it reads gravity's share along its tipped axes, exactly. */
  @Test
  void tiltedItReadsGravitysShare() {
    AccelerometerModel part = mounted(EXACT_SAMPLED, Translation2d.ZERO);
    physics.apply(new Disturbance.TiltedByHand(Math.toRadians(-10), Math.toRadians(5)));
    steps(part, 5);

    Translation2d read = part.read();
    assertThat(read.getX()).isCloseTo(G * Math.sin(Math.toRadians(10)), within(1e-9));
    assertThat(read.getY())
        .isCloseTo(G * Math.sin(Math.toRadians(5)) * Math.cos(Math.toRadians(10)), within(1e-9));
  }

  /**
   * Mounted 20 cm ahead of the center and 10 cm left, on a robot spinning steadily in place, it
   * reads the centripetal pull toward the center, ω² times its distance. It samples the robot's
   * velocity a step apart, so it reads the step's average: its size short by (ωΔt)²/24 and its
   * direction turned back by half a step's turn, ωΔt/2; physics' instant within those.
   */
  @Test
  void offTheCenterOfASpinningRobotItReadsTheCentripetalPull() {
    Translation2d place = new Translation2d(0.2, 0.1);
    AccelerometerModel part = mounted(EXACT_SAMPLED, place);
    Translation2d[] wheels = TestRobot.wheels();
    for (int i = 0; i < 4; i++) {
      controllers.steer[i] = wheels[i].getAngle().orElseThrow().getRadians() + Math.PI / 2;
    }
    controllers.volts = 2;
    steps(part, 750); // up to speed, and steady

    double rate = physics.chassis().yawRateRadiansPerSecond();
    Translation2d read = part.read();
    Translation2d pull = place.times(-rate * rate);
    double halfStep = rate * STEP / 2;

    assertThat(rate).as("spinning").isGreaterThan(1);
    assertThat(read.getNorm())
        .isCloseTo(pull.getNorm(), within(pull.getNorm() * (halfStep * halfStep / 6 + 1e-6)));
    assertThat(read.getAngle().orElseThrow().minus(pull.getAngle().orElseThrow()).getRadians())
        .isCloseTo(-halfStep, within(1e-6));
  }

  /**
   * Speeding up straight ahead at its drive motors' stator limit, it reads how fast the robot's
   * speed changed over its last step, exactly: the robot's own acceleration then. Consistency, not
   * truth: the model takes the physics' velocity a step apart, the very difference this compares it
   * with. Newton's answer for a launch is the robot program's {@code DynamicsTruthSimTest}.
   */
  @Test
  @Consistency("the accelerometer differences the physics' own velocity, as this does")
  void speedingUpItReadsTheRobotsAcceleration() {
    AccelerometerModel part = mounted(EXACT_SAMPLED, Translation2d.ZERO);
    controllers.volts = 12;
    steps(part, 24);
    double before = physics.chassis().velocity().getX();
    steps(part, 1); // the 25th: a sample is due, 100 ms in

    double acceleration = (physics.chassis().velocity().getX() - before) / STEP;
    assertThat(acceleration).as("speeding up").isGreaterThan(4);
    assertThat(part.read().getX()).isCloseTo(acceleration, within(1e-9));
  }

  /**
   * With a Pigeon 2's noise and bias, and nothing else, standing still: its readings average to the
   * bias (within three standard errors) and spread as its noise (within 10%, three standard errors
   * of a spread from 500 readings).
   */
  @Test
  void itsReadingsSpreadAsItsNoiseAroundItsBias() {
    SensorErrors.AccelErrors noisy =
        SensorErrorsAccelErrorsBuilder.builder(EXACT_SAMPLED)
            .noise(0.3)
            .biasForward(0.08)
            .biasLeft(-0.05)
            .build();
    AccelerometerModel part = mounted(noisy, Translation2d.ZERO);
    int n = 500;
    double sum = 0;
    double sumSquares = 0;
    double sumLeft = 0;
    for (int i = 0; i < n; i++) {
      steps(part, 5);
      Translation2d read = part.read();
      sum += read.getX();
      sumSquares += read.getX() * read.getX();
      sumLeft += read.getY();
    }
    double mean = sum / n;
    double spread = Math.sqrt(sumSquares / n - mean * mean);

    assertThat(mean).isCloseTo(0.08, within(3 * 0.3 / Math.sqrt(n)));
    assertThat(sumLeft / n).isCloseTo(-0.05, within(3 * 0.3 / Math.sqrt(n)));
    assertThat(spread).isCloseTo(0.3, within(0.03));
  }

  /** A 4 g hit reads exactly its range, 2 g, as a Pigeon 2's does; missed, it's forgotten. */
  @Test
  void aHitPastItsRangeReadsItsRangeExactly() {
    SensorErrors.AccelErrors ranged =
        SensorErrorsAccelErrorsBuilder.builder(EXACT_SAMPLED).rangeMetersPerSecSq(2 * G).build();
    AccelerometerModel part = mounted(ranged, Translation2d.ZERO);
    steps(part, 5);

    part.jolt(new Translation2d(4 * G, 0));
    steps(part, 5);
    double hit = part.read().getX();
    part.jolt(new Translation2d(4 * G, 0));
    part.missed();

    assertThat(hit).isEqualTo(2 * G);
    assertThat(part.read().getX()).isCloseTo(0, within(1e-12));
  }

  /** At 100 Hz, its reading changes every 10 ms, and holds between: 100 times a second. */
  @Test
  void itTakesANewSampleEvery10Milliseconds() {
    AccelerometerModel part =
        mounted(
            SensorErrorsAccelErrorsBuilder.builder(EXACT_SAMPLED).noise(0.3).build(),
            Translation2d.ZERO);

    int changes = 0;
    Translation2d last = part.read();
    for (int step = 0; step < 250; step++) {
      steps(part, 1);
      Translation2d now = part.read();
      changes += now.equals(last) ? 0 : 1;
      last = now;
    }

    assertThat(changes).isEqualTo(100);
  }

  /**
   * An ideal one reads the robot's average acceleration over each loop, exactly; turned on its
   * mount, filtered, or the other way up, a real one reads along its own axes, turned back by the
   * definition's orientation.
   */
  @Test
  void anIdealOneReadsEachLoopsAverageAndARealOneItsOwnAxes() {
    AccelerometerModel ideal = mounted(SensorErrors.AccelErrors.IDEAL, Translation2d.ZERO);
    AccelerometerModel flipped =
        mounted(
            SensorErrorsAccelErrorsBuilder.builder(EXACT_SAMPLED)
                .flipped(true)
                .filterSeconds(0.016)
                .build(),
            Translation2d.ZERO);
    controllers.volts = 12;
    controllers.steer(Math.PI / 2);
    for (int i = 0; i < 5; i++) {
      physics.step(STEP, controllers);
      ideal.step(STEP);
      flipped.step(STEP);
    }
    ideal.endLoop(0.02);
    double before = physics.chassis().velocity().getY();
    for (int i = 0; i < 5; i++) {
      physics.step(STEP, controllers);
      ideal.step(STEP);
      flipped.step(STEP);
    }
    ideal.endLoop(0.02);

    assertThat(ideal.read().getY())
        .isCloseTo((physics.chassis().velocity().getY() - before) / 0.02, within(1e-9));
    assertThat(flipped.read().getY()).as("upside down, left reads right").isNegative();
  }
}

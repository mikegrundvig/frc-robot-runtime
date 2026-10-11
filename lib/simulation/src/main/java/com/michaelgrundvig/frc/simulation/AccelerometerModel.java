package com.michaelgrundvig.frc.simulation;

import com.michaelgrundvig.frc.world.robot.ImuMount;
import java.util.Random;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A simulated IMU's accelerometer.
 *
 * <p><b>What it feels.</b> The robot's true acceleration at the point where the IMU really sits (an
 * IMU off the robot's center is swung around the center as the robot turns), and the floor holding
 * the robot up against gravity, which a tilted robot feels along all three of its axes ({@link
 * #alongTheRobot}). The acceleration is the change in the point's velocity over one physics step,
 * so it is the step's average.
 *
 * <p><b>How it reads that</b>, as the part does ({@link SensorErrors.AccelErrors}), in this order:
 *
 * <ol>
 *   <li>a first-order low-pass filter smooths what it feels;
 *   <li>the filter's output is sampled at the part's rate, with noise added to each sample, and the
 *       latest sample is held until the next;
 *   <li>a jolt from a hit, and the part's steady bias, are added;
 *   <li>the result is turned into the part's own axes, which may lie a little off where the robot's
 *       definition says they do, and each axis is cut off at the part's range;
 *   <li>the robot's IO turns those axes back into the robot's by the definition's orientation, as
 *       the real IO does, so an error in how the part is really mounted stays in the reading.
 * </ol>
 *
 * <p>An ideal accelerometer ({@link SensorErrors.AccelErrors#IDEAL}) skips the filter and the
 * sampling: it reads the robot's average acceleration over each robot loop, exactly.
 *
 * <p>It reads the chassis's truth ({@link ChassisTruth}) after each physics step.
 */
public final class AccelerometerModel {
  /**
   * How far short of its sampling period a sample may be taken, in seconds. Physics steps add up to
   * a period only to within rounding, and without this a sample would now and then fall a whole
   * step late.
   */
  private static final double SAMPLE_TIME_SLACK_SECONDS = 1e-9;

  private final ChassisTruth chassis;
  private final SensorErrors.AccelErrors errors;

  /** Where it really sits, from the robot's center: where it is defined to be, off by its error. */
  private final Translation2d place;

  /** How the robot's definition says the part is turned on the robot. */
  private final Rotation3d definedOrientation;

  /**
   * How it really is turned: the definition's turn, then its errors' (turned, or the other way up).
   */
  private final Rotation3d trueOrientation;

  /** The noise on its forward and left axes. */
  private final Random noise;

  /**
   * The noise on its up axis. It is a stream of its own so that the other two axes draw the same
   * numbers they drew before the up axis was modeled.
   */
  private final Random upNoise;

  /**
   * Whether it is an ideal accelerometer, which reads each loop's average acceleration exactly. A
   * real part filters and samples instead.
   */
  private final boolean ideal;

  /** Seconds between a real part's samples. Unused for an ideal one. */
  private final double samplePeriodSeconds;

  /** The velocity on the field of the point it sits at, after the last physics step. */
  private Translation2d lastVelocity;

  // A real part: its filter's output, and its latest sample, each along the robot's forward, left
  // and up axes, and how long it has been since it last sampled.
  private double filteredForward;
  private double filteredLeft;
  private double filteredUp;
  private double sampledForward;
  private double sampledLeft;
  private double sampledUp;
  private double secondsSinceSample;

  // An ideal one: its point's velocity as the loop started, and the last loop's average
  // acceleration.
  private Translation2d loopStartVelocity;
  private Translation3d loopAcceleration = Translation3d.ZERO;

  /** A hit's jolt, along the robot's axes, waiting until this IMU is next read. */
  private Translation2d jolt = Translation2d.ZERO;

  /**
   * @param chassis the chassis it rides on, as the physics keeps it
   * @param errors how it reads
   * @param mount where the robot's definition says it's mounted; it really sits off that by its
   *     errors' place, and is turned off it by their turn
   * @param seed for its noise
   */
  public AccelerometerModel(
      ChassisTruth chassis, SensorErrors.AccelErrors errors, ImuMount mount, long seed) {
    this.chassis = chassis;
    this.errors = errors;
    place = mount.place().plus(new Translation2d(errors.forwardMeters(), errors.leftMeters()));
    definedOrientation = mount.orientation();
    // Mounted the other way up is half a turn about its forward axis.
    double flippedRadians = 0;
    if (errors.flipped()) {
      flippedRadians = Math.PI;
    }
    Rotation3d mountError =
        new Rotation3d(flippedRadians, 0, Math.toRadians(errors.turnedDegrees()));
    trueOrientation = definedOrientation.rotateBy(mountError);
    noise = new Random(seed);
    upNoise = new Random(Seeds.upAxis(seed));
    double period = 0;
    if (errors.sampleHz() > 0) {
      period = 1 / errors.sampleHz();
    }
    samplePeriodSeconds = period;
    ideal = !(period > 0);
    lastVelocity = chassis.pointVelocity(place);
    loopStartVelocity = lastVelocity;
  }

  /** After each physics step: a real part filters what it felt, and samples it at its rate. */
  public void step(double seconds) {
    Translation2d velocity = chassis.pointVelocity(place);
    if (!ideal) {
      Translation2d fieldAcceleration = velocity.minus(lastVelocity).div(seconds);
      filter(alongTheRobot(fieldAcceleration), seconds);
      sampleWhenDue(seconds);
    }
    lastVelocity = velocity;
  }

  /**
   * A first-order low-pass filter, stepped exactly: over a step, the output closes on the input by
   * the share 1 − e^(−step / time constant). With no time constant there is no filter, and the
   * output is the input.
   */
  private void filter(Translation3d felt, double seconds) {
    double shareClosed = 1;
    if (errors.filterSeconds() > 0) {
      shareClosed = 1 - Math.exp(-seconds / errors.filterSeconds());
    }
    filteredForward += (felt.getX() - filteredForward) * shareClosed;
    filteredLeft += (felt.getY() - filteredLeft) * shareClosed;
    filteredUp += (felt.getZ() - filteredUp) * shareClosed;
  }

  /**
   * Takes a sample of the filter's output, with noise, once a sampling period has passed. The noise
   * is drawn for the forward axis, then the left, then the up axis from its own stream: the order
   * is part of what a seed means.
   */
  private void sampleWhenDue(double seconds) {
    secondsSinceSample += seconds;
    boolean due = secondsSinceSample >= samplePeriodSeconds - SAMPLE_TIME_SLACK_SECONDS;
    if (!due) {
      return;
    }
    secondsSinceSample -= samplePeriodSeconds;
    sampledForward = filteredForward + errors.noise() * noise.nextGaussian();
    sampledLeft = filteredLeft + errors.noise() * noise.nextGaussian();
    sampledUp = filteredUp + errors.noise() * upNoise.nextGaussian();
  }

  /** At the end of each loop: an ideal accelerometer reads the loop's average acceleration. */
  public void endLoop(double seconds) {
    Translation2d fieldAcceleration = lastVelocity.minus(loopStartVelocity).div(seconds);
    loopAcceleration = alongTheRobot(fieldAcceleration);
    loopStartVelocity = lastVelocity;
  }

  /** A hit's jolt, along the robot's axes: the next reading feels it. */
  public void jolt(Translation2d robotAxes) {
    jolt = jolt.plus(robotAxes);
  }

  /** Forgets a jolt it was never read for: unplugged, it felt nothing. */
  public void missed() {
    jolt = Translation2d.ZERO;
  }

  /**
   * What the robot's IO reports, once a loop: the part's reading along its own axes, turned into
   * the robot's forward and left by the definition's orientation. Reading it uses up any jolt.
   */
  public Translation2d read() {
    Translation3d sensed = ideal ? loopAcceleration : latestSample();
    Translation3d alongTheRobotsAxes =
        sensed
            .plus(new Translation3d(jolt.getX(), jolt.getY(), 0))
            .plus(new Translation3d(errors.biasForward(), errors.biasLeft(), 0));
    jolt = Translation2d.ZERO;

    // The part reports along its own axes, as it is really mounted, and no further than its range
    // on each of them.
    Translation3d alongItsOwnAxes = alongTheRobotsAxes.rotateBy(trueOrientation.inverse());
    double range = errors.rangeMetersPerSecSq();
    Translation3d reported =
        new Translation3d(
            Math.clamp(alongItsOwnAxes.getX(), -range, range),
            Math.clamp(alongItsOwnAxes.getY(), -range, range),
            Math.clamp(alongItsOwnAxes.getZ(), -range, range));

    // The robot's IO believes the definition's orientation, right or wrong.
    Translation3d asTheRobotTakesIt = reported.rotateBy(definedOrientation);
    return new Translation2d(asTheRobotTakesIt.getX(), asTheRobotTakesIt.getY());
  }

  /** A real part's latest sample, along the robot's forward, left and up axes. */
  private Translation3d latestSample() {
    return new Translation3d(sampledForward, sampledLeft, sampledUp);
  }

  /**
   * What the part feels, along the robot's forward, left and up axes, for an acceleration across
   * the field: the acceleration itself, turned into the robot's heading, plus the floor's push
   * holding the robot up against gravity. An accelerometer at rest on a level floor reads g upward.
   * Tipped, that g is shared among the robot's three axes.
   *
   * <p>The angles are WPILib's: a positive pitch is nose down, and a positive roll is the left side
   * up. With the nose down, the floor's push has a share along the robot's backward axis, g·sin
   * (pitch). With the left side up, it has a share along the robot's left axis, g·sin(roll)·cos
   * (pitch). What is left along the robot's up axis is g·cos(roll)·cos(pitch).
   */
  private Translation3d alongTheRobot(Translation2d fieldAcceleration) {
    double pitch = chassis.pitchRadians();
    double roll = chassis.rollRadians();
    Rotation2d fieldToRobot = chassis.pose().getRotation().unaryMinus();
    Translation2d driving = fieldAcceleration.rotateBy(fieldToRobot);
    return new Translation3d(
        driving.getX() - Physics.STANDARD_GRAVITY * Math.sin(pitch),
        driving.getY() + Physics.STANDARD_GRAVITY * Math.sin(roll) * Math.cos(pitch),
        Physics.STANDARD_GRAVITY * Math.cos(roll) * Math.cos(pitch));
  }
}

package com.michaelgrundvig.frc.simulation;

import static com.michaelgrundvig.frc.world.config.Rules.each;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.size;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * How the simulated robot's wheels and sensors differ from what the code believes: the errors that
 * make real odometry drift. The simulator keeps the robot's true pose, the sensors report it with
 * these errors, and the gap between the truth and the code's estimate is what localization has to
 * close.
 *
 * <p>Built by name, like the robot's configuration: start from {@link #NONE} or {@link #TYPICAL}
 * and change what the test needs, {@code SensorErrorsBuilder.builder(SensorErrors.NONE)
 * .steerOffsetDegrees(...).build()}, or use one of the {@code with} methods.
 *
 * @param wheelRadiusScale each wheel's true radius over the configured one (front left, front
 *     right, back left, back right). 0.98 is tread worn 2% smaller than the code thinks: the robot
 *     travels 2% less than odometry says.
 * @param steerOffsetDegrees how far each module's absolute encoder zero is off: the module points
 *     that much away from where the code thinks it points
 * @param gyro the gyro's errors
 * @param backupGyro the backup IMU's errors (Systemcore's own), read once a loop
 * @param accelerometer how the gyro's accelerometer reads: its errors, and the part's limits
 * @param backupAccelerometer the same for the backup IMU's
 * @param cornerNoise how far the simulated cameras' tag corners land from where a perfect camera
 *     would put them: the same for every camera, each drawing its own noise
 * @param cameraMountErrors by camera name, how each camera is really mounted off its configured
 *     mount, in the robot's frame: a camera knocked 4 in low, or turned 2 degrees
 * @param tagErrors by tag ID, how far each tag really sits from where the field's layout says, in
 *     the field's frame: a field built to its tolerance, or a tag knocked at an event. The cameras
 *     see the tags where they really are; the robot, and its cameras' own multi-tag solves, believe
 *     the layout.
 * @param seed for the noise, the gyros' and the cameras', so a run repeats exactly
 */
@ConfigRecord
public record SensorErrors(
    List<Double> wheelRadiusScale,
    List<Double> steerOffsetDegrees,
    GyroErrors gyro,
    GyroErrors backupGyro,
    AccelErrors accelerometer,
    AccelErrors backupAccelerometer,
    CornerNoise cornerNoise,
    @RecordBuilder.Initializer(value = "of", source = Map.class)
        Map<String, Transform3d> cameraMountErrors,
    @RecordBuilder.Initializer(value = "of", source = Map.class)
        Map<Integer, Translation3d> tagErrors,
    long seed) {
  /**
   * Wheels and sensors exactly as configured: odometry is the truth. For tests of exact numbers.
   */
  public static final SensorErrors NONE =
      SensorErrorsBuilder.builder()
          .wheelRadiusScale(List.of(1.0, 1.0, 1.0, 1.0))
          .steerOffsetDegrees(List.of(0.0, 0.0, 0.0, 0.0))
          .gyro(GyroErrors.EXACT)
          .backupGyro(GyroErrors.EXACT)
          .accelerometer(AccelErrors.IDEAL)
          .backupAccelerometer(AccelErrors.IDEAL)
          .cornerNoise(CornerNoise.TYPICAL)
          .seed(1)
          .build();

  /**
   * A robot partway through a season: tread worn 1-2% and unevenly, absolute encoder zeros off by
   * up to half a degree, and a gyro reading 0.3% high, drifting 0.2° a minute; its backup IMU reads
   * 0.5% high and drifts a degree a minute. Both accelerometers read as a Pigeon 2's does, with its
   * range, rate, noise, and a small bias and mount error ({@link AccelErrors#PIGEON_2}). Plausible
   * estimates, not measurements: replace them with a real robot's once it is calibrated.
   */
  public static final SensorErrors TYPICAL =
      SensorErrorsBuilder.builder()
          .wheelRadiusScale(List.of(0.985, 0.99, 0.98, 0.988))
          .steerOffsetDegrees(List.of(0.5, -0.3, 0.0, 0.4))
          .gyro(
              GyroErrors.builder()
                  .scale(1.003)
                  .driftDegreesPerMinute(0.2)
                  .noiseDegrees(0.02)
                  .build())
          .backupGyro(
              GyroErrors.builder()
                  .scale(1.005)
                  .driftDegreesPerMinute(1.0)
                  .noiseDegrees(0.05)
                  .build())
          .accelerometer(AccelErrors.PIGEON_2)
          .backupAccelerometer(AccelErrors.PIGEON_2)
          .cornerNoise(CornerNoise.TYPICAL)
          .seed(1)
          .build();

  /**
   * How far a simulated camera's tag corners land from where a perfect camera would put them. It
   * stands for lens-calibration error and a soft image, and it is most of what scatters a camera's
   * positions.
   *
   * @param meanPixels how far each corner lands from the perfect one, on average
   * @param stdDevPixels and that distance's spread
   */
  @ConfigRecord
  public record CornerNoise(double meanPixels, double stdDevPixels) {
    /** PhotonVision's simulation example, which VisionSim's constants repeat. */
    public static final CornerNoise TYPICAL = builder().meanPixels(0.25).stdDevPixels(0.08).build();

    public CornerNoise {
      Checks.on(CornerNoise.class)
          .field("meanPixels", meanPixels, nonNegative())
          .field("stdDevPixels", stdDevPixels, nonNegative())
          .orThrow();
    }

    /** Starts describing the corners' noise, one named value at a time. */
    public static SensorErrorsCornerNoiseBuilder.MeanPixelsStage builder() {
      return SensorErrorsCornerNoiseBuilder.builder();
    }
  }

  /**
   * How a gyro's readings differ from the truth.
   *
   * @param scale its reading per true turn: 1.003 reads 0.3% too much rotation
   * @param driftDegreesPerMinute a steady drift, from its bias
   * @param noiseDegrees random noise on each reading, as a standard deviation
   */
  @ConfigRecord
  public record GyroErrors(double scale, double driftDegreesPerMinute, double noiseDegrees) {
    /** A gyro that reads the truth. */
    public static final GyroErrors EXACT =
        builder().scale(1).driftDegreesPerMinute(0).noiseDegrees(0).build();

    public GyroErrors {
      Checks.on(GyroErrors.class)
          .field("scale", scale, positive())
          .field("driftDegreesPerMinute", driftDegreesPerMinute, finite())
          .field("noiseDegrees", noiseDegrees, nonNegative())
          .orThrow();
    }

    /** Starts describing a gyro's errors, one named value at a time. */
    public static SensorErrorsGyroErrorsBuilder.ScaleStage builder() {
      return SensorErrorsGyroErrorsBuilder.builder();
    }
  }

  /**
   * How a simulated accelerometer reads: its errors, and the part's own limits. The simulated IMU
   * feels the robot's true motion where it sits, and gravity's share as it tilts; this is what
   * comes between that and the reading.
   *
   * @param biasForward a steady offset along the robot's forward axis, in m/s²: the part's own, and
   *     a mount a little off level, whose tilt puts a share of gravity in it
   * @param biasLeft the same along the robot's left axis
   * @param noise random noise on each sample, as a standard deviation, in m/s²: the part's, and the
   *     robot's vibration
   * @param turnedDegrees how far its axes really lie turned, about the robot's up axis, from the
   *     orientation the robot's definition gives it ({@code ImuMount.orientation}),
   *     counterclockwise from above: an orientation set a little off, or the part turned on its
   *     mount since
   * @param flipped whether it's really mounted the other way up from its orientation in the robot's
   *     definition: its left axis reads the robot's right
   * @param forwardMeters how far ahead of where the robot's definition places it it really sits
   * @param leftMeters and how far left
   * @param rangeMetersPerSecSq the most it reports on each axis, either way: past it, it reads the
   *     range. Infinite for none.
   * @param sampleHz how often its readings are sampled and sent; the robot reads the latest each
   *     loop. Zero for an ideal accelerometer read as the robot's average acceleration over the
   *     loop.
   * @param filterSeconds the time constant of the low-pass filter ahead of its sampling: what it
   *     smooths of the robot's jolts, and how late it reads a change
   */
  @ConfigRecord
  public record AccelErrors(
      double biasForward,
      double biasLeft,
      double noise,
      double turnedDegrees,
      boolean flipped,
      double forwardMeters,
      double leftMeters,
      double rangeMetersPerSecSq,
      double sampleHz,
      double filterSeconds) {
    /** An ideal accelerometer: the robot's average acceleration over each loop, exactly. */
    public static final AccelErrors IDEAL =
        builder()
            .biasForward(0)
            .biasLeft(0)
            .noise(0)
            .turnedDegrees(0)
            .flipped(false)
            .forwardMeters(0)
            .leftMeters(0)
            .rangeMetersPerSecSq(Double.POSITIVE_INFINITY)
            .sampleHz(0)
            .filterSeconds(0)
            .build();

    /**
     * A Pigeon 2's: ±2 g on each axis (the range of Phoenix 6's {@code AccelerationX} signal),
     * sampled at the 100 Hz the robot's code asks for, behind a filter of 16 ms. Its noise and bias
     * are guesses: CTRE publishes neither for the accelerometer (its User's Guide plots a few mg at
     * rest on a vibrating fixture), so the noise is a driving robot's vibration, 0.3 m/s², and the
     * bias a part calibrated at the factory on a mount half a degree off level, under 0.1 m/s². Its
     * mount pose set within half a degree, and its place measured to a centimeter.
     */
    public static final AccelErrors PIGEON_2 =
        builder()
            .biasForward(0.08)
            .biasLeft(-0.05)
            .noise(0.3)
            .turnedDegrees(0.5)
            .flipped(false)
            .forwardMeters(0.01)
            .leftMeters(0)
            .rangeMetersPerSecSq(2 * Physics.STANDARD_GRAVITY)
            .sampleHz(100)
            .filterSeconds(0.016)
            .build();

    public AccelErrors {
      Checks.on(AccelErrors.class)
          .field("biasForward", biasForward, finite())
          .field("biasLeft", biasLeft, finite())
          .field("noise", noise, nonNegative())
          .field("turnedDegrees", turnedDegrees, finite())
          .field("forwardMeters", forwardMeters, finite())
          .field("leftMeters", leftMeters, finite())
          .check(
              "rangeMetersPerSecSq",
              () ->
                  rangeMetersPerSecSq > 0
                      ? null
                      : rangeMetersPerSecSq + " must be positive, or infinite for no range")
          .field("sampleHz", sampleHz, nonNegative())
          .field("filterSeconds", filterSeconds, nonNegative())
          .orThrow();
    }

    /** Starts describing an accelerometer, one named value at a time. */
    public static SensorErrorsAccelErrorsBuilder.BiasForwardStage builder() {
      return SensorErrorsAccelErrorsBuilder.builder();
    }
  }

  public SensorErrors {
    wheelRadiusScale = List.copyOf(wheelRadiusScale);
    steerOffsetDegrees = List.copyOf(steerOffsetDegrees);
    cameraMountErrors = Map.copyOf(cameraMountErrors);
    tagErrors = Map.copyOf(tagErrors);
    Checks.on(SensorErrors.class)
        .field("wheelRadiusScale", wheelRadiusScale, size(4)) // one per module
        .field("wheelRadiusScale", wheelRadiusScale, each(positive()))
        .field("steerOffsetDegrees", steerOffsetDegrees, size(4))
        .field("steerOffsetDegrees", steerOffsetDegrees, each(finite()))
        .orThrow();
  }

  /** The same errors, with every wheel's radius scaled alike: tread worn evenly. */
  public SensorErrors withWheelRadiusScale(double scale) {
    return SensorErrorsBuilder.builder(this)
        .wheelRadiusScale(List.of(scale, scale, scale, scale))
        .build();
  }

  /**
   * The same errors, with one module's absolute encoder zeroed off by this much.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public SensorErrors withSteerOffset(int module, double degrees) {
    List<Double> offsets = new ArrayList<>(steerOffsetDegrees);
    offsets.set(module, degrees);
    return SensorErrorsBuilder.builder(this).steerOffsetDegrees(offsets).build();
  }

  /** The same errors, with the gyro reading like this; the backup IMU as it was. */
  public SensorErrors withGyro(GyroErrors gyro) {
    return SensorErrorsBuilder.builder(this).gyro(gyro).build();
  }

  /** The same errors, with both the gyro and the backup IMU reading like this. */
  public SensorErrors withGyros(GyroErrors both) {
    return SensorErrorsBuilder.builder(this).gyro(both).backupGyro(both).build();
  }

  /** The same errors, with the gyro's accelerometer reading like this. */
  public SensorErrors withAccelerometer(AccelErrors accelerometer) {
    return SensorErrorsBuilder.builder(this).accelerometer(accelerometer).build();
  }

  /** The same errors, with the backup IMU reading like this. */
  public SensorErrors withBackupGyro(GyroErrors backup) {
    return SensorErrorsBuilder.builder(this).backupGyro(backup).build();
  }

  /** The same errors, with every camera's tag corners landing off like this. */
  public SensorErrors withCornerNoise(CornerNoise noise) {
    return SensorErrorsBuilder.builder(this).cornerNoise(noise).build();
  }

  /** The same errors, with one camera mounted off its configuration. */
  public SensorErrors withCameraMountError(String camera, Transform3d error) {
    Map<String, Transform3d> errors = new HashMap<>(cameraMountErrors);
    errors.put(camera, error);
    return SensorErrorsBuilder.builder(this).cameraMountErrors(errors).build();
  }

  /** The same errors, with the field's tags sitting off its layout like this, by tag ID. */
  public SensorErrors withTagErrors(Map<Integer, Translation3d> errors) {
    return SensorErrorsBuilder.builder(this).tagErrors(errors).build();
  }

  /** How a camera is really mounted off its configuration: none, unless one was set. */
  public Transform3d cameraMountError(String camera) {
    return cameraMountErrors.getOrDefault(camera, Transform3d.ZERO);
  }
}

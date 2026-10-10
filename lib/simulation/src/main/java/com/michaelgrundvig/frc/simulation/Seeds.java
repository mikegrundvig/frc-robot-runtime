package com.michaelgrundvig.frc.simulation;

/**
 * Where each of the simulated world's random streams starts, from the world's one seed ({@code
 * SensorErrors.seed}), so a run repeats exactly and each stream draws its own numbers: every stream
 * is the seed mixed with its kind, then with its index among those of its kind ({@link #stream}).
 * The cameras' streams come from here too, for the robot program's camera simulation beside
 * PhotonVision's ({@link #camera}), and the studies mix their own draws the same way ({@link
 * #mix}).
 *
 * <p>Mixed, no stream is another's: not one of the same world's (a seed's gyro and its first camera
 * once drew alike, both seeded with the world's seed itself), and not one of another world's (each
 * seed's gyro once drew what the seed before it gave its gyro's accelerometer); and streams from
 * neighboring seeds don't start alike, as Java's {@code Random} seeded 1, 2, 3 does, its first
 * draws moving together. The robot program's {@code SeedsTruthSimTest} holds them to it.
 */
public final class Seeds {
  // Each kind of stream, mixed into the seed first: constants far apart, and far from any small
  // seed, since one step of mixing depends on the seed and the number only through their sum.
  private static final long GYROS = 0x6779726f73L; // "gyros"
  private static final long ACCELEROMETERS = 0x616363656c73L; // "accels"
  private static final long UP_AXIS = 0x75702d61786973L; // "up-axis"
  private static final long CAMERAS = 0x63616d65726173L; // "cameras"

  private Seeds() {}

  /** The gyro's noise. */
  public static long gyro(long seed) {
    return stream(seed, GYROS, 0);
  }

  /** The gyro's accelerometer's noise, on its forward and left axes. */
  public static long gyroAccelerometer(long seed) {
    return stream(seed, ACCELEROMETERS, 0);
  }

  /** The backup IMU's noise. */
  public static long backupGyro(long seed) {
    return stream(seed, GYROS, 1);
  }

  /** The backup IMU's accelerometer's noise, on its forward and left axes. */
  public static long backupAccelerometer(long seed) {
    return stream(seed, ACCELEROMETERS, 1);
  }

  /**
   * An accelerometer's noise on its up axis: a stream of its own, mixed from its other two axes'
   * seed, so they draw as they would without it.
   *
   * @param accelerometer the seed of its other two axes
   */
  public static long upAxis(long accelerometer) {
    return mix(accelerometer, UP_AXIS);
  }

  /**
   * A camera's noise, in PhotonVision's simulation of it: its corners and its latency.
   *
   * @param camera its place in the order the cameras are added, from 0
   */
  public static long camera(long seed, int camera) {
    return stream(seed, CAMERAS, camera);
  }

  /**
   * One of a world's streams: the seed mixed with the stream's kind, then with its index among
   * those of its kind. Mixing the kind in first keeps a seed and an index apart: one step of mixing
   * alone would make seed 2's first stream seed 1's second.
   *
   * @param kind a constant of the kind's own, far from every other kind's and from any small seed
   */
  public static long stream(long seed, long kind, long index) {
    return mix(mix(seed, kind), index);
  }

  /**
   * A seed from a seed and numbers, each mixed in turn, so each random draw in a study depends only
   * on what it's for (the run's seed, a pose, a camera), never on the order work happens in. The
   * same numbers give the same seed on every computer.
   *
   * <p>One step of mixing depends on the seed and the number only through their sum, so {@code
   * mix(1, 2)} is {@code mix(2, 1)}, and {@code mix(s, t + 1)} is {@code mix(s + 1, t)}. Numbers
   * mixed in after the first are mixed into a seed already scrambled, so their order matters. Where
   * a seed and a count must stay apart, mix the seed with a constant of its own first ({@link
   * #stream}).
   */
  public static long mix(long seed, double... values) {
    long mixed = seed;
    for (double value : values) {
      mixed = mix(mixed, Double.doubleToLongBits(value));
    }
    return mixed;
  }

  /** A seed from a seed and a number: SplitMix64's mixing, so nearby numbers seed far apart. */
  public static long mix(long seed, long value) {
    long h = seed + value + 0x9E3779B97F4A7C15L;
    h = (h ^ (h >>> 30)) * 0xBF58476D1CE4E5B9L;
    h = (h ^ (h >>> 27)) * 0x94D049BB133111EBL;
    return h ^ (h >>> 31);
  }
}

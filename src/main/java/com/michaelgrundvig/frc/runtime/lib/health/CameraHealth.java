package com.michaelgrundvig.frc.runtime.lib.health;

import java.util.EnumSet;
import java.util.Set;

/**
 * One camera's health over the last 10 s, from what its frames say as they arrive: its frame rate,
 * the frames its vision program dropped and the network lost, and how long detection took against
 * the camera's frame period. Each warning's episodes this session are kept too.
 *
 * <p>The frame rate counts any camera's frames. The rest is Visor's alone: each of its frames
 * carries its number, its detection time, and its camera's dropped frames so far (Visor's {@code
 * docs/wire.md}). PhotonVision's results say none of it, and for such a camera it reads as unknown.
 *
 * <p>Ten seconds is Visor's own report's period and the robot loop's "lately": long enough that a
 * frame dropped now and then shows, short enough that the pit sees it come and go.
 */
public final class CameraHealth {
  /** How far back the numbers reach, in seconds. */
  public static final double WINDOW_SECONDS = 10;

  /** The window, in buckets of a second each: it slides by a second at a time. */
  private static final int BUCKETS = 10;

  private static final long BUCKET_NANOS = 1_000_000_000L;

  /** Visor's counts are 32 bits, and wrap. */
  private static final long COUNT_MASK = 0xffff_ffffL;

  /** What's wrong with a camera, as of the last 10 s. */
  public enum Warning {
    /** Its vision program dropped frames: detection fell behind, or the camera sent broken ones. */
    DROPPED_FRAMES,
    /** Frames it sent never arrived: the network lost them. */
    LOST_FRAMES,
    /**
     * Detection took longer on average than the camera's frame period, so it can't keep up with the
     * camera. The mean, not the worst: one slow frame now and then costs only that frame, and shows
     * as a dropped one.
     */
    SLOW_DETECTION
  }

  private final double framePeriodMs;

  // The window's buckets, the current one at `current`.
  private final long[] frames = new long[BUCKETS];
  private final long[] dropped = new long[BUCKETS];
  private final long[] lost = new long[BUCKETS];
  private final long[] detected = new long[BUCKETS];
  private final double[] detectSumMs = new double[BUCKETS];
  private final double[] detectWorstMs = new double[BUCKETS];
  private int current;
  private long bucketStartNanos;
  private long firstNanos;

  /**
   * When each bucket's time begins: the update before its first, since what arrives at an update
   * arrived after the one before it. The window's length is from its oldest bucket's.
   */
  private final long[] openedNanos = new long[BUCKETS];

  /** How many buckets have been opened: the window is full once all have. */
  private long opened;

  private long previousNanos;
  private boolean started;
  private long nowNanos;

  /** Whether the camera numbers its frames and counts its drops: Visor does, PhotonVision not. */
  public enum Numbers {
    /** A frame has said its number and the camera's dropped count. */
    REPORTED,
    /** None has: a camera on PhotonVision, or one on Visor before its first frame. */
    NOT_REPORTED
  }

  private Numbers numbers = Numbers.NOT_REPORTED;

  /**
   * The highest frame number the camera said, and the dropped count it said with it, once {@link
   * #numbers} are reported.
   */
  private long lastSequence;

  private long lastDropped;
  private int restarts;

  /** Each warning's, by its ordinal. */
  private final Episodes[] episodes = new Episodes[Warning.values().length];

  private final Set<Warning> warnings = EnumSet.noneOf(Warning.class);

  /**
   * @param framePeriodSeconds the time between the camera's frames, as configured
   */
  public CameraHealth(double framePeriodSeconds) {
    framePeriodMs = framePeriodSeconds * 1000;
    for (int i = 0; i < episodes.length; i++) {
      episodes[i] = new Episodes();
    }
  }

  /**
   * Takes what arrived this loop from a camera whose frames have no numbers (PhotonVision), or none
   * yet. Once a loop, the time never going back.
   *
   * @param arrived how many of its frames arrived since the last loop, used or not
   * @param detectMs each arrived frame's detection time, in milliseconds; empty when none says
   */
  public void update(long nowNanos, int arrived, double[] detectMs) {
    startLoop(nowNanos, arrived);
    countDetections(detectMs);
    judge();
  }

  /**
   * Takes what arrived this loop from a camera whose frames are numbered (Visor). Once a loop, the
   * time never going back.
   *
   * @param arrived how many of its frames arrived since the last loop, used or not
   * @param sequence the highest number of a frame that has arrived, counted from the vision
   *     program's start
   * @param droppedSoFar the vision program's count of the camera's dropped frames, from its start,
   *     as that frame said
   * @param detectMs each arrived frame's detection time, in milliseconds
   */
  public void update(
      long nowNanos, int arrived, long sequence, long droppedSoFar, double[] detectMs) {
    startLoop(nowNanos, arrived);
    countByNumbers(arrived, sequence, droppedSoFar);
    countDetections(detectMs);
    judge();
  }

  private void startLoop(long nowNanos, int arrived) {
    previousNanos = this.nowNanos;
    this.nowNanos = nowNanos;
    slideTo(nowNanos);
    frames[current] += arrived;
  }

  private void countDetections(double[] detectMs) {
    for (double ms : detectMs) {
      detected[current]++;
      detectSumMs[current] += ms;
      detectWorstMs[current] = Math.max(detectWorstMs[current], ms);
    }
  }

  /** Moves the window on to the bucket this time falls in, emptying those it passes. */
  private void slideTo(long now) {
    if (!started) {
      started = true;
      firstNanos = now;
      bucketStartNanos = now;
      openedNanos[0] = now;
      opened = 1;
      return;
    }
    long seconds = (now - bucketStartNanos) / BUCKET_NANOS;
    for (long i = 0; i < Math.min(seconds, BUCKETS); i++) {
      current = (current + 1) % BUCKETS;
      frames[current] = 0;
      dropped[current] = 0;
      lost[current] = 0;
      detected[current] = 0;
      detectSumMs[current] = 0;
      detectWorstMs[current] = 0;
      openedNanos[current] = previousNanos;
      opened++;
    }
    bucketStartNanos += seconds * BUCKET_NANOS;
  }

  /**
   * Counts the frames dropped and lost since the last loop, from the highest frame number heard and
   * the dropped count said with it.
   *
   * <p>A number above the last is new frames: those numbered since, less those that arrived, were
   * lost. Frames that arrived late, after newer ones ({@link FrameNumbers}), were counted lost when
   * the newer ones arrived, and take that back as they arrive. The same number as the last is no
   * new frame. A number below the last means the vision program started counting again: it
   * restarted, or its 32-bit number wrapped after two years at 60 frames a second, and both counts
   * started again from 0. (The number given is the highest heard since the numbering last started,
   * so a frame that arrived late never lowers it.) A restart whose first frame heard is numbered
   * above the last can't be told from frames lost on the way, and counts as them.
   */
  private void countByNumbers(int arrived, long sequence, long droppedSoFar) {
    // The first frame heard sets where counting starts: what came before the robot listened isn't
    // the robot's to judge.
    if (numbers == Numbers.NOT_REPORTED) {
      numbers = Numbers.REPORTED;
      lastSequence = sequence;
      lastDropped = droppedSoFar;
      return;
    }
    long sent = sequence - lastSequence;
    long droppedSince = (droppedSoFar - lastDropped) & COUNT_MASK;
    boolean restarted = sequence < lastSequence;
    if (restarted) {
      restarts++;
      sent = sequence + 1;
      droppedSince = droppedSoFar;
    }
    // A bucket can hold less than nothing, where frames counted lost in it arrived in the next: the
    // window's sum is what was lost.
    lost[current] += sent - arrived;
    dropped[current] += droppedSince;
    lastSequence = sequence;
    lastDropped = droppedSoFar;
  }

  private void judge() {
    warnings.clear();
    if (numbers == Numbers.REPORTED && droppedLately() > 0) {
      warnings.add(Warning.DROPPED_FRAMES);
    }
    if (numbers == Numbers.REPORTED && lostLately() > 0) {
      warnings.add(Warning.LOST_FRAMES);
    }
    if (detectMeanMs() > framePeriodMs) {
      warnings.add(Warning.SLOW_DETECTION);
    }
    for (Warning warning : Warning.values()) {
      episodes[warning.ordinal()].update(warnings.contains(warning), nowNanos);
    }
  }

  /**
   * Frames that arrived per second, over the last 10 s, or since the first loop when that's less;
   * NaN in the first second, too soon to say.
   */
  public double framesPerSecond() {
    if (!started || nowNanos - firstNanos < BUCKET_NANOS) {
      return Double.NaN;
    }
    int oldest = 0;
    if (opened >= BUCKETS) {
      oldest = (current + 1) % BUCKETS;
    }
    return sum(frames) / ((nowNanos - openedNanos[oldest]) / 1e9);
  }

  /**
   * Whether the camera's frames say their numbers and its dropped count, without which {@link
   * #droppedLately} and {@link #lostLately} can't be known.
   */
  public Numbers numbers() {
    return numbers;
  }

  /**
   * Frames the vision program dropped in the last 10 s.
   *
   * @throws IllegalStateException while the {@link #numbers} aren't reported
   */
  public long droppedLately() {
    requireNumbers("dropped");
    return sum(dropped);
  }

  /**
   * Frames it sent in the last 10 s that never arrived.
   *
   * @throws IllegalStateException while the {@link #numbers} aren't reported
   */
  public long lostLately() {
    requireNumbers("lost");
    // Frames that arrived late can take back more than the window still holds as lost.
    return Math.max(0, sum(lost));
  }

  private void requireNumbers(String count) {
    if (numbers == Numbers.NOT_REPORTED) {
      throw new IllegalStateException(
          "the camera's " + count + " frames aren't reported: check numbers() first");
    }
  }

  /** The mean detection time over the last 10 s, in milliseconds; NaN with none to go by. */
  public double detectMeanMs() {
    long count = sum(detected);
    if (count == 0) {
      return Double.NaN;
    }
    double total = 0;
    for (double each : detectSumMs) {
      total += each;
    }
    return total / count;
  }

  /** The longest detection time over the last 10 s, in milliseconds; NaN with none to go by. */
  public double detectWorstMs() {
    if (sum(detected) == 0) {
      return Double.NaN;
    }
    double worst = 0;
    for (double each : detectWorstMs) {
      worst = Math.max(worst, each);
    }
    return worst;
  }

  /** The camera's frame period, as configured, in milliseconds: what detection must keep under. */
  public double framePeriodMs() {
    return framePeriodMs;
  }

  /** How many times its vision program restarted this session, by its frames' numbers. */
  public int restarts() {
    return restarts;
  }

  /** What's wrong now, as of the last 10 s. */
  public boolean warns(Warning warning) {
    return warnings.contains(warning);
  }

  /** A warning's episodes this session. */
  public Episodes episodes(Warning warning) {
    return episodes[warning.ordinal()];
  }

  /** The time of the last update. */
  public long nowNanos() {
    return nowNanos;
  }

  private static long sum(long[] values) {
    long total = 0;
    for (long each : values) {
      total += each;
    }
    return total;
  }
}

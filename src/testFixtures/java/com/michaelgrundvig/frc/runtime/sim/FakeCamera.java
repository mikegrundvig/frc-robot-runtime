package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A plain camera for the drivetrain's estimator tests, lighter than PhotonVision's simulated ones
 * ({@code VisionSim}, which the whole-robot tests use): it sees the robot's true pose (never the
 * estimate), with noise, a frame rate, and a delay before each result arrives, and reports a
 * heading-constrained solve as a real pipeline would, except that a heading error in the estimate
 * never puts its positions off. It can be told to lie once, as a misread tag does.
 */
public final class FakeCamera {
  private final DriveSimulation robot;
  private final double stdDevMeters;
  private final long latencyNanos;
  private final long frameNanos;
  private final Random noise;
  private final ArrayDeque<VisionMeasurement> inFlight = new ArrayDeque<>();
  private long lastFrameNanos = Long.MIN_VALUE;
  private Transform2d nextLie = Transform2d.ZERO;
  private List<Integer> tags = List.of(26);

  public FakeCamera(
      DriveSimulation robot,
      double stdDevMeters,
      double latencySeconds,
      double framesPerSecond,
      long seed) {
    this.robot = robot;
    this.stdDevMeters = stdDevMeters;
    latencyNanos = Math.round(latencySeconds * 1e9);
    frameNanos = Math.round(1e9 / framesPerSecond);
    noise = new Random(seed);
  }

  /**
   * From now on, every result says it was solved from these tags; one tag, the 26th, unless told.
   * Two tags read rightly are two independent fixes, which a recovery needs from one camera.
   */
  public FakeCamera seeing(List<Integer> seen) {
    tags = List.copyOf(seen);
    return this;
  }

  /** The next picture's result will be off by this much, however sure it claims to be. */
  public void lieOnce(Transform2d lie) {
    nextLie = lie;
  }

  /** Takes a picture if one is due, and returns the results that have arrived by now. */
  public List<VisionMeasurement> poll(long nowNanos) {
    if (lastFrameNanos == Long.MIN_VALUE || nowNanos - lastFrameNanos >= frameNanos) {
      lastFrameNanos = nowNanos;
      Pose2d truth = robot.truePose();
      Pose2d seen =
          new Pose2d(
                  truth
                      .getTranslation()
                      .plus(
                          new Translation2d(
                              stdDevMeters * noise.nextGaussian(),
                              stdDevMeters * noise.nextGaussian())),
                  truth.getRotation())
              .plus(nextLie);
      nextLie = Transform2d.ZERO;
      inFlight.addLast(
          new VisionMeasurement(
              nowNanos,
              seen,
              PoseCovariance.fromStdDevs(stdDevMeters, stdDevMeters, Double.POSITIVE_INFINITY),
              // It places the robot without the estimate's heading, so its position doesn't lean
              // on it, as no real camera's heading-constrained solve does.
              VisionMeasurement.Lever.NONE,
              "fake",
              tags,
              VisionMeasurement.Solver.HEADING_CONSTRAINED));
    }
    List<VisionMeasurement> arrived = new ArrayList<>();
    while (!inFlight.isEmpty() && inFlight.getFirst().timestampNanos() + latencyNanos <= nowNanos) {
      arrived.add(inFlight.removeFirst());
    }
    return arrived;
  }
}

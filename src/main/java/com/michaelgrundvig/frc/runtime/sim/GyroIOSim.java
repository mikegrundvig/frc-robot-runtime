package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.simulation.GyroModel;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.Arrays;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A simulated gyro as the robot's code reads it: the simulation's model of the IMU ({@link
 * GyroModel}), its yaw sampled with the modules' readings for odometry, and everything else read
 * once a loop, as {@code GyroIOPigeon2} reads a real one. Its faults are the model's.
 */
final class GyroIOSim implements GyroIO {
  private final GyroModel model;

  // Readings taken since the last loop, for odometry.
  private long[] sampleTimes = new long[8];
  private double[] sampleYaws = new double[8];
  private int samples;

  GyroIOSim(GyroModel model) {
    this.model = model;
  }

  /** Unplugged: no readings until it's plugged back in. */
  void setConnected(boolean connected) {
    model.setConnected(connected);
  }

  boolean isConnected() {
    return model.isConnected();
  }

  /** Stuck: still reporting, the same yaw every time. */
  void setFrozen(boolean frozen) {
    model.setFrozen(frozen);
  }

  boolean isFrozen() {
    return model.isFrozen();
  }

  /** Errs differently from now on, carrying on from what it reads now. */
  void setErrors(SensorErrors.GyroErrors errors) {
    model.setErrors(errors);
  }

  /** How it errs now. */
  SensorErrors.GyroErrors errors() {
    return model.errors();
  }

  /** How it erred as the robot was built. */
  SensorErrors.GyroErrors built() {
    return model.built();
  }

  /** Reads zero from now on wherever the robot points, as a gyro that restarts does. */
  void reset() {
    model.reset();
  }

  /** After each physics step: its accelerometer feels it. */
  void feel(double seconds) {
    model.feel(seconds);
  }

  /** At the end of each loop. */
  void endLoop(double seconds) {
    model.endLoop(seconds);
  }

  /** A hit's jolt, along the robot's axes: its next reading feels it. */
  void jolt(Translation2d robotAxes) {
    model.jolt(robotAxes);
  }

  /** Takes a reading for odometry, with the modules' readings. */
  void sample(long timestampNanos) {
    if (!model.sample()) {
      return;
    }
    if (samples == sampleTimes.length) {
      sampleTimes = Arrays.copyOf(sampleTimes, samples * 2);
      sampleYaws = Arrays.copyOf(sampleYaws, samples * 2);
    }
    sampleTimes[samples] = timestampNanos;
    sampleYaws[samples] = model.yawRadians();
    samples++;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = model.isConnected();
    inputs.yaw = new Rotation2d(model.yawRadians());
    inputs.yawVelocityRadPerSec = model.yawRateRadiansPerSecond();
    inputs.pitchRad = model.pitchRadians();
    inputs.rollRad = model.rollRadians();
    @Nullable Translation2d felt = model.acceleration();
    if (felt != null) {
      inputs.accelerationXMetersPerSecSq = felt.getX();
      inputs.accelerationYMetersPerSecSq = felt.getY();
    }
    inputs.odometryTimestampsNanos = Arrays.copyOf(sampleTimes, samples);
    inputs.odometryYawsRad = Arrays.copyOf(sampleYaws, samples);
    samples = 0;
  }
}

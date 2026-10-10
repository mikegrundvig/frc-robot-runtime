package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;

/**
 * A SPARK's loop, as the REV helper configures it from the joint's plan: its kP and, holding a
 * position, its kI on the device, every millisecond, on where it reads the joint; the rest, the
 * plan's feedforward and kD, worked out by the robot as it asks, from where it last read the joint,
 * and sent with the request in volts, held until it asks again ({@code JointIOSpark}). So its
 * damping and its gravity are a loop's old, as on a robot. Its loop runs on where it reads the
 * joint: its motor's count, as a TalonFX's does, or the encoder on its data port its loop closes on
 * ({@code JointIOSim}).
 */
final class SparkEmulator implements ControllerEmulator {
  private ControllerPlan plan;
  private double setPosition;
  private double setVelocity;
  private double integral;

  /** What the robot sent with its request: the plan's feedforward and kD, in volts. */
  private double feedforwardVolts;

  SparkEmulator(ControllerPlan plan) {
    this.plan = plan;
  }

  @Override
  public void hold(
      double position,
      double velocity,
      double feedforwardVolts,
      double read,
      double readVelocity,
      boolean fresh) {
    if (fresh) {
      integral = 0;
    }
    setPosition = position;
    this.feedforwardVolts =
        feedforwardVolts
            + plan.feedforward(position, velocity, 0)
            + plan.kD() * (velocity - readVelocity);
  }

  @Override
  public void run(double velocity, double read) {
    setVelocity = velocity;
    feedforwardVolts = plan.feedforward(read, velocity, 0);
  }

  @Override
  public double holding(double position, double velocity, double seconds) {
    double error = setPosition - position;
    integral += error * seconds;
    return plan.kP() * error + plan.kI() * integral + feedforwardVolts;
  }

  @Override
  public double running(double position, double velocity) {
    return plan.kP() * (setVelocity - velocity) + feedforwardVolts;
  }

  @Override
  public void retune(ControllerPlan plan) {
    this.plan = plan;
  }
}

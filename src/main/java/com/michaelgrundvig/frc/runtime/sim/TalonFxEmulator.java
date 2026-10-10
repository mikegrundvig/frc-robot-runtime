package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;

/**
 * A TalonFX's slot 0, as the Phoenix helper configures it from the joint's plan, run every
 * millisecond on where the TalonFX reads the joint: kP, kI and kD on the error, and the plan's
 * feedforward, at the joint's position and the speed asked, which Phoenix's slot runs itself (its
 * kS, kV and gravity), plus the volts the program adds with each request, its kA term. As Phoenix's
 * {@code PositionVoltage} and {@code VelocityVoltage} run it.
 */
final class TalonFxEmulator implements ControllerEmulator {
  private ControllerPlan plan;
  private double setPosition;
  private double setVelocity;
  private double feedforwardVolts;
  private double integral;

  TalonFxEmulator(ControllerPlan plan) {
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
    setVelocity = velocity;
    this.feedforwardVolts = feedforwardVolts;
  }

  @Override
  public void run(double velocity, double read) {
    setVelocity = velocity;
  }

  @Override
  public double holding(double position, double velocity, double seconds) {
    double error = setPosition - position;
    integral += error * seconds;
    return plan.kP() * error
        + plan.kI() * integral
        + plan.kD() * (setVelocity - velocity)
        + plan.feedforward(position, setVelocity, 0)
        + feedforwardVolts;
  }

  @Override
  public double running(double position, double velocity) {
    return plan.kP() * (setVelocity - velocity) + plan.feedforward(position, setVelocity, 0);
  }

  @Override
  public void retune(ControllerPlan plan) {
    this.plan = plan;
  }
}

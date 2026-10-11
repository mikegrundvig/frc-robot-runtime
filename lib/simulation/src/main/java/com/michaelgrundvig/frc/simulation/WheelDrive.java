package com.michaelgrundvig.frc.simulation;

/**
 * What drives one wheel over a physics sub-step, as its actuators say: what its drive's controller
 * asks of the motor, its current limit, and its steering's angle ({@link Actuators}). The physics
 * keeps one for each wheel and hands them out to be filled in every sub-step, so nothing is made
 * per step.
 *
 * <p>The controller's ask comes before its current limit: the physics holds the motor to the limit
 * itself, through the whole sub-step, as a motor controller holds it. A limit applied by the
 * actuators, as a voltage fixed at the sub-step's start, would hold only at that instant.
 */
public final class WheelDrive {
  private double volts;
  private double currentLimitAmps = Double.POSITIVE_INFINITY;
  private double batteryVolts = Double.POSITIVE_INFINITY;
  private double steerRadians;

  /**
   * The drive motor over the sub-step.
   *
   * @param volts what the controller asks to put across the motor as the sub-step starts, within
   *     the battery's voltage: its loop's output, before its current limit. The physics holds it
   *     through the sub-step, so how the loop's ask would move with the wheel's speed inside one is
   *     not said here
   * @param currentLimitAmps the most current the controller lets through the motor, either way: the
   *     physics holds the motor to it, cutting the voltage toward the motor's back EMF
   * @param batteryVolts the most the controller can put across the motor, either way, whatever the
   *     current limit would need
   */
  public void drive(double volts, double currentLimitAmps, double batteryVolts) {
    this.volts = volts;
    this.currentLimitAmps = currentLimitAmps;
    this.batteryVolts = batteryVolts;
  }

  /**
   * The steering's angle at the sub-step's end, turned in closed form by the actuators.
   *
   * @param radians counterclockwise from the robot's forward axis
   */
  public void steer(double radians) {
    steerRadians = radians;
  }

  /** What the controller asks to put across the drive motor as the sub-step starts. */
  public double volts() {
    return volts;
  }

  /** The most current the drive's controller lets through its motor. */
  public double currentLimitAmps() {
    return currentLimitAmps;
  }

  /** The most voltage the drive's controller can put across its motor. */
  public double batteryVolts() {
    return batteryVolts;
  }

  /** The steering's angle at the sub-step's end. */
  public double steerRadians() {
    return steerRadians;
  }
}

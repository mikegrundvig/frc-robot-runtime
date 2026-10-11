package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;

/**
 * A SPARK's loop, as the REV helper configures it from the joint's plan. The control law is split
 * between the device and the robot:
 *
 * <ul>
 *   <li><b>On the device, every millisecond</b>, on where it reads the joint: kP and, holding a
 *       position, kI.
 *   <li><b>On the robot, once a loop</b>, from where the robot last read the joint: the plan's
 *       feedforward and kD. The robot sends them with its request as one number of volts, which the
 *       SPARK adds to its own output and holds until the robot asks again ({@code JointIOSpark}).
 * </ul>
 *
 * <p>So a SPARK's damping and its gravity feedforward are one robot loop old, as they are on a
 * robot. Its loop runs on where it reads the joint: its motor's count, or the encoder on its data
 * port, where its loop closes on that ({@code JointIOSim}).
 */
final class SparkEmulator implements ControllerEmulator {
  private ControllerPlan plan;
  private double askedPosition;
  private double askedVelocity;

  /** The position error added up over time, in the joint's units times seconds. */
  private double errorIntegral;

  /**
   * The volts the robot sent with its last request, held until it asks again: the plan's
   * feedforward, and for a position, kD's damping and the profile's kA term.
   */
  private double requestVolts;

  SparkEmulator(ControllerPlan plan) {
    this.plan = plan;
  }

  @Override
  public void startPositionLoopAfresh() {
    errorIntegral = 0;
  }

  @Override
  public void hold(
      double position, double velocity, double feedforwardVolts, double lastReadVelocity) {
    askedPosition = position;
    // The robot works these out as it asks: the feedforward at the profile's own point and speed,
    // and kD on the difference between the profile's speed and the speed it last read.
    double velocityError = velocity - lastReadVelocity;
    requestVolts =
        feedforwardVolts + plan.feedforward(position, velocity, 0) + plan.kD() * velocityError;
  }

  @Override
  public void run(double velocity, double lastReadPosition) {
    askedVelocity = velocity;
    requestVolts = plan.feedforward(lastReadPosition, velocity, 0);
  }

  @Override
  public double voltsToHold(double position, double velocity, double seconds) {
    double positionError = askedPosition - position;
    errorIntegral += positionError * seconds;
    return plan.kP() * positionError + plan.kI() * errorIntegral + requestVolts;
  }

  @Override
  public double voltsToRun(double position, double velocity) {
    double velocityError = askedVelocity - velocity;
    return plan.kP() * velocityError + requestVolts;
  }

  @Override
  public void retune(ControllerPlan plan) {
    this.plan = plan;
  }
}

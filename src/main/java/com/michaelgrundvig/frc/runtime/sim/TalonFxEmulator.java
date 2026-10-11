package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;

/**
 * A TalonFX's slot 0, as the Phoenix helper configures it from the joint's plan, run every
 * millisecond on where the TalonFX reads the joint. The whole control law runs on the device, as
 * Phoenix's {@code PositionVoltage} and {@code VelocityVoltage} run it:
 *
 * <pre>
 *   holding a position:  kP·(position error) + kI·∫(position error) + kD·(velocity error)
 *                        + the slot's feedforward + the request's feedforward
 *   running at a speed:  kP·(velocity error) + the slot's feedforward
 * </pre>
 *
 * <p>The slot's feedforward is the plan's kS, kV and gravity, worked out at the joint's position
 * now and the speed asked for. The request's feedforward is the volts the robot adds to each
 * position request: the profile's acceleration times kA.
 */
final class TalonFxEmulator implements ControllerEmulator {
  private ControllerPlan plan;
  private double askedPosition;
  private double askedVelocity;

  /** The volts the robot added to its last position request. */
  private double requestFeedforwardVolts;

  /** The position error added up over time, in the joint's units times seconds. */
  private double errorIntegral;

  TalonFxEmulator(ControllerPlan plan) {
    this.plan = plan;
  }

  @Override
  public void startPositionLoopAfresh() {
    errorIntegral = 0;
  }

  /** A TalonFX works the whole law out itself, so what the robot last read plays no part. */
  @Override
  public void hold(
      double position, double velocity, double feedforwardVolts, double lastReadVelocity) {
    askedPosition = position;
    askedVelocity = velocity;
    requestFeedforwardVolts = feedforwardVolts;
  }

  @Override
  public void run(double velocity, double lastReadPosition) {
    askedVelocity = velocity;
  }

  @Override
  public double voltsToHold(double position, double velocity, double seconds) {
    double positionError = askedPosition - position;
    errorIntegral += positionError * seconds;
    double velocityError = askedVelocity - velocity;
    return plan.kP() * positionError
        + plan.kI() * errorIntegral
        + plan.kD() * velocityError
        + slotFeedforwardVolts(position)
        + requestFeedforwardVolts;
  }

  @Override
  public double voltsToRun(double position, double velocity) {
    double velocityError = askedVelocity - velocity;
    return plan.kP() * velocityError + slotFeedforwardVolts(position);
  }

  /**
   * The plan's feedforward as the slot runs it: at the joint's position now and the speed asked
   * for, with no acceleration term, since kA comes with the request.
   */
  private double slotFeedforwardVolts(double position) {
    return plan.feedforward(position, askedVelocity, 0);
  }

  @Override
  public void retune(ControllerPlan plan) {
    this.plan = plan;
  }
}

package com.michaelgrundvig.frc.simulation;

/**
 * What drives the physics' wheels, worked out in closed form beside it: the motor controllers'
 * loops and limits, the motors, and the battery. The {@link Physics} asks at the start of each of
 * its sub-steps, telling the truth as it stands (a wheel's speed, for a velocity loop and a motor's
 * back EMF), and the actuators say what drives each wheel over the sub-step ({@link WheelDrive}).
 *
 * <p>Two couplings, which keep the physics' steps long however quick the motors are:
 *
 * <ul>
 *   <li><b>A wheel's drive is a motor's line, not a torque.</b> Its controller gives the voltage it
 *       asks for across the motor, how that moves with the wheel's speed (its velocity loop's
 *       gain), and its current limit, which the physics applies: together, the motor's torque
 *       against the wheel's speed, a straight line capped at the limit's torque. The physics holds
 *       the limit at the sub-step's start, and the voltage that gives through it.
 *   <li><b>A wheel's steering stays closed form.</b> The steering settles in under a millisecond,
 *       so the actuators turn it themselves ({@link CurrentLimitedMotor}) and give the angle; the
 *       physics follows it, and returns what the floor's scrub pushes back with ({@link
 *       WheelTruth#steerLoadNewtonMeters}) for the next sub-step: unread yet, as the physics models
 *       no scrub. The actuators keep time, not count sub-steps, so their own rates (the steering's
 *       loop, the encoders') hold whatever length the sub-steps are.
 * </ul>
 */
@FunctionalInterface
public interface Actuators {
  /**
   * Runs the motor controllers and the steering over the next sub-step, from the truth as it
   * starts, and says what drives each wheel over it.
   *
   * @param seconds the sub-step's length
   * @param physics the world as the sub-step starts
   * @param drives one for each wheel, in the physics' order, to fill in
   */
  void actuate(double seconds, Physics physics, WheelDrive[] drives);
}

package com.michaelgrundvig.frc.simulation;

/**
 * The simulated world's physics: the robot's chassis and wheels, and their contact with the field,
 * apart from the sensors that read them and the faults that disturb them. Motor commands go in,
 * through the {@link Actuators}; true motion comes out, as the {@link ChassisTruth} and each {@link
 * WheelTruth}, which only the sensor models and the simulator's own outputs read.
 *
 * <p>The physics behind this interface keeps three promises, which the truth suite and the recorded
 * runs rely on:
 *
 * <ul>
 *   <li><b>One world, no static state.</b> Each instance is its own world, so worlds run side by
 *       side on threads, and nothing one does reaches another.
 *   <li><b>Determinism.</b> The same configuration, disturbances, and actuator commands give the
 *       same truth, to the bit: no clock, no unseeded randomness, no order that depends on identity
 *       hash codes, and sub-steps of a fixed length, never chosen by how long a step took.
 *   <li><b>Sub-steps inside.</b> {@link #step} moves the world on by however long the simulator
 *       asks (a fifth of a robot loop, 4 ms, at its odometry's 250 Hz), in sub-steps of its own
 *       choosing, and asks the actuators what drives each wheel at the start of every one. The
 *       template's ({@link KinematicPhysics}) takes each step as one sub-step.
 * </ul>
 */
public interface Physics {
  /**
   * The acceleration of gravity the world pulls with, in m/s²: the standard one, as localization
   * reads an IMU's.
   */
  double STANDARD_GRAVITY = 9.80665;

  /**
   * Moves the world on by this long: its sub-steps, each starting with the actuators told the truth
   * as it stands and asked what drives each wheel over it.
   *
   * @param seconds how long, more than zero
   */
  void step(double seconds, Actuators actuators);

  /**
   * The robot's chassis as it truly is now: a live view, to read before the next step or change.
   */
  ChassisTruth chassis();

  /**
   * One wheel as it truly is now: a live view, to read before the next step or change.
   *
   * @param index front left, front right, back left, back right: 0 to 3
   */
  WheelTruth wheel(int index);

  /** How many wheels the robot has. */
  int wheels();

  /** Something done to the world from outside it, from now on or at once ({@link Disturbance}). */
  void apply(Disturbance disturbance);
}

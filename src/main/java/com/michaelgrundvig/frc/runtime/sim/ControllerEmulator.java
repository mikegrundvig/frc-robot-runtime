package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;

/**
 * A motor controller's own closed loop, as the simulator runs it every millisecond: the volts it
 * puts across its motors to hold the joint at a point of a profile, or to run it at a speed, from
 * where it reads the joint.
 *
 * <p>The two vendors split the joint's control law ({@link ControllerPlan}) between the device and
 * the robot differently, and each emulator works out each part of the law where its controller
 * does:
 *
 * <ul>
 *   <li>A TalonFX runs the whole law on the device, every millisecond ({@link TalonFxEmulator}).
 *   <li>A SPARK runs only kP and kI on the device. The robot works out the rest once a loop, from
 *       what it last read, and sends it with its request ({@link SparkEmulator}).
 * </ul>
 *
 * <p>A loop the robot closes itself, on an absolute encoder it reads, is not here: the robot works
 * out its volts from the same plan and asks for them as a voltage, which every controller puts
 * across its motors as it is.
 */
sealed interface ControllerEmulator permits TalonFxEmulator, SparkEmulator {
  /**
   * The emulator for the controllers a joint's plan names: its block's, or, where its block wires
   * none, the controller its motor usually runs on, such as a SPARK MAX for a NEO ({@link
   * ControllerPlan#controllers}).
   */
  static ControllerEmulator of(ControllerPlan plan) {
    return switch (plan.controllers().get(0).controller()) {
      case TALON_FX, TALON_FXS -> new TalonFxEmulator(plan);
      case SPARK_MAX, SPARK_FLEX -> new SparkEmulator(plan);
    };
  }

  /**
   * Starts its position loop afresh, forgetting what its integral had added up. The joint's IO
   * calls this when a position is asked of a controller that was doing something else.
   */
  void startPositionLoopAfresh();

  /**
   * Takes a request to hold the joint at a point of a profile, in the joint's units.
   *
   * @param position where the profile is
   * @param velocity how fast the profile moves there
   * @param feedforwardVolts what the robot adds of its own: the profile's acceleration times kA
   * @param lastReadVelocity how fast the controller last told the robot the joint moves
   */
  void hold(double position, double velocity, double feedforwardVolts, double lastReadVelocity);

  /**
   * Takes a request to run the joint at a speed, in its units a second.
   *
   * @param lastReadPosition where the controller last told the robot the joint is
   */
  void run(double velocity, double lastReadPosition);

  /**
   * The volts it puts across the motors this millisecond to hold the point it was asked for.
   *
   * @param position where it reads the joint now
   * @param velocity how fast it reads the joint moving now
   * @param seconds the length of its loop, which its integral adds up over
   */
  double voltsToHold(double position, double velocity, double seconds);

  /**
   * The volts it puts across the motors this millisecond to run at the speed it was asked for.
   *
   * @param position where it reads the joint now
   * @param velocity how fast it reads the joint moving now
   */
  double voltsToRun(double position, double velocity);

  /** Takes the pit's tuning: the joint's plan with its tuned control. */
  void retune(ControllerPlan plan);
}

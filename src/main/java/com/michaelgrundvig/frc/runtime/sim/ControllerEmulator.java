package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;

/**
 * A motor controller's own closed loop, as the simulator runs it, every millisecond: what it puts
 * across its motors to hold the joint at the point of a profile it's asked for, or at a speed, from
 * where it reads the joint. Each evaluates the joint's plan ({@link ControllerPlan#feedforward})
 * where its controller does: a TalonFX's slot runs all of it on the device ({@link
 * TalonFxEmulator}); a SPARK runs its kP, and the robot works the rest out as it asks ({@link
 * SparkEmulator}). A loop the robot closes itself, on an absolute encoder it reads, works out its
 * volts from the same plan and asks for them as a voltage, which every controller puts across its
 * motors as it is.
 */
sealed interface ControllerEmulator permits TalonFxEmulator, SparkEmulator {
  /**
   * The emulator for the controllers a joint's plan names: its block's, or its motor's stand-in
   * where its block wires none, a NEO's a SPARK MAX ({@link ControllerPlan#controllers}).
   */
  static ControllerEmulator of(ControllerPlan plan) {
    return switch (plan.controllers().get(0).controller()) {
      case TALON_FX, TALON_FXS -> new TalonFxEmulator(plan);
      case SPARK_MAX, SPARK_FLEX -> new SparkEmulator(plan);
    };
  }

  /**
   * Asked to hold the joint at the point of a profile, in its units.
   *
   * @param position where the profile is
   * @param velocity how fast it moves there
   * @param feedforwardVolts what the program adds of its own, the profile's acceleration times kA
   * @param read where the controller last told the robot the joint is
   * @param readVelocity how fast it last told the robot the joint moves
   * @param fresh whether it was asked for something else until now, so its loop starts afresh
   */
  void hold(
      double position,
      double velocity,
      double feedforwardVolts,
      double read,
      double readVelocity,
      boolean fresh);

  /**
   * Asked to run the joint at a speed, in its units a second.
   *
   * @param read where the controller last told the robot the joint is
   */
  void run(double velocity, double read);

  /**
   * The volts it puts across the motors this millisecond to hold the point it was asked for, the
   * joint where and as fast as it reads it.
   */
  double holding(double position, double velocity, double seconds);

  /** The volts it puts across the motors this millisecond to run at the speed it was asked for. */
  double running(double position, double velocity);

  /** Takes the pit's tuning: the joint's plan with its tuned control. */
  void retune(ControllerPlan plan);
}

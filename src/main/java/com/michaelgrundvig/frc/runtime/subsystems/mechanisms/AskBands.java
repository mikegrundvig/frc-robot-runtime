package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.JointAsk;

/**
 * How near one thing asked of a joint must be to the next for the two to be one request going on,
 * and not a new one: within the band the joint must come inside to be at either. A target a command
 * follows moves a little every loop, and a joint that is at it one loop is at it the next. A target
 * that jumps is a new request, which the joint has yet to be judged against ({@link Tenure}).
 *
 * @param position how near a position must be, in the joint's units: the band to arrive in
 * @param speedShare how near a speed must be, as a share of it: the band to arrive in
 * @param freeSpeed the speed a full battery runs the unloaded joint at, which a speed of zero, that
 *     has no share, is given its band from
 */
record AskBands(double position, double speedShare, double freeSpeed) {
  /** Whether what is asked now is what was asked before, going on. */
  boolean goesOn(JointAsk before, JointAsk now) {
    if (before.getClass() != now.getClass()) {
      return false;
    }
    double moved = Math.abs(Asks.number(now) - Asks.number(before));
    return switch (now) {
      case JointAsk.ToPosition toPosition -> moved <= position;
      case JointAsk.AtVelocity atVelocity -> moved <= speedBand(atVelocity.speed());
      // A voltage or a share of the battery has nothing to reach, so the joint is doing either at
      // once whatever its size. A command with a law of its own asks a new voltage every loop.
      case JointAsk.AtVoltage atVoltage -> true;
      case JointAsk.AtDutyCycle atDutyCycle -> true;
      case JointAsk.Neutral neutral -> true;
    };
  }

  private double speedBand(double speed) {
    return speedShare * (speed == 0 ? freeSpeed : Math.abs(speed));
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.Output;

/** What the runtime needs to know of a {@link JointAsk} that the model's type does not say. */
final class Asks {
  private Asks() {}

  /**
   * What a joint is asked when it is run at one of its named outputs.
   *
   * <p>A share of the battery of exactly zero is neutral, not a duty cycle of zero: the motors then
   * brake or coast as the joint's control says, which is what {@link Output#STOPPED} means.
   */
  static JointAsk running(Output output) {
    return switch (output.kind()) {
      case VELOCITY -> new JointAsk.AtVelocity(output.value());
      case VOLTAGE -> new JointAsk.AtVoltage(output.value());
      case DUTY_CYCLE -> {
        if (output.value() == 0) {
          yield JointAsk.NEUTRAL;
        }
        yield new JointAsk.AtDutyCycle(output.value());
      }
    };
  }

  /**
   * The one number in what a joint is asked: the position to go to, or the speed, the volts or the
   * share of the battery to run at. Neutral asks for nothing, which is zero of any output. It is
   * the number a request is refused for when it is not a number, and the number the log shows as
   * the joint's goal while the joint is not at a position.
   */
  static double number(JointAsk ask) {
    return switch (ask) {
      case JointAsk.ToPosition toPosition -> toPosition.position();
      case JointAsk.AtOutput output -> output.value();
      case JointAsk.Neutral neutral -> 0;
    };
  }

  /** What kind of number an ask carries, as a message says it: "a position", "a speed". */
  static String numberShown(JointAsk ask) {
    return switch (ask) {
      case JointAsk.ToPosition toPosition -> "a position";
      case JointAsk.AtVelocity atVelocity -> "a speed";
      case JointAsk.AtVoltage atVoltage -> "a voltage";
      // Neutral is asked for as a share of the battery of zero.
      case JointAsk.AtDutyCycle _, JointAsk.Neutral _ -> "a share of the battery";
    };
  }
}

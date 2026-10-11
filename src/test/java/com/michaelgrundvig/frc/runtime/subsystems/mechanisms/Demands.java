package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.JointAsk;

/** Demands as the stages' tests make them and read them, each in a word. */
final class Demands {
  private Demands() {}

  /** A robot's sensors with none in them, for a stage whose rules read no sensor. */
  static final Sensors NO_SENSORS = new SwitchReadings().sensors();

  static Demand toPosition(double position, double cruiseShare) {
    return Demand.of(new JointAsk.ToPosition(position, cruiseShare));
  }

  static Demand atVelocity(double speed) {
    return Demand.of(new JointAsk.AtVelocity(speed));
  }

  static Demand atVoltage(double volts) {
    return Demand.of(new JointAsk.AtVoltage(volts));
  }

  static Demand atDutyCycle(double share) {
    return Demand.of(new JointAsk.AtDutyCycle(share));
  }

  /** The position a demand sends its joint to: it fails for a demand that isn't to a position. */
  static double position(Demand demand) {
    return ((JointAsk.ToPosition) demand.ask()).position();
  }

  /** The share of its cruise a demand sends its joint to a position at. */
  static double cruiseShare(Demand demand) {
    return ((JointAsk.ToPosition) demand.ask()).cruiseShare();
  }
}

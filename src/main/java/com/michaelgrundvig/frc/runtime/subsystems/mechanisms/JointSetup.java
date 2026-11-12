package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.function.Supplier;

/**
 * What a joint's mechanism works out once, as it is built, about how the joint is driven: its
 * control, where it can be driven and how fast, and its motors' gains as its motor controller has
 * them. A stand-in for a joint with no mechanism has none of these to work out, since nothing
 * drives it, and is given values that drive nothing.
 *
 * @param control the joint's control as its motor controller takes it at startup
 * @param travel where the joint can be driven, and how fast
 * @param freeSpeed the speed a full battery runs the unloaded joint at, in its units a second
 * @param kA the volts for each unit a second squared, as the motor controller has it
 * @param kV the volts for each unit a second, as the motor controller has it
 */
record JointSetup(MechanismControl control, Travel travel, double freeSpeed, double kA, double kV) {
  /**
   * The parts for a mechanism in a mode: worked out from the joint's block for one that drives its
   * joint, and a stand-in's for one that stands in.
   *
   * @param tuned the joint's control as the pit tunes it, which is all a stand-in has
   */
  static JointSetup of(
      JointMode mode, RobotMath math, Joint joint, Supplier<MechanismControl> tuned) {
    return switch (mode) {
      case JointMode.Drives drives -> driven(math, joint);
      case JointMode.StandsIn standsIn -> standIn(joint, tuned.get());
    };
  }

  /**
   * The parts of a joint its mechanism drives, from its block: its control, its travel, its free
   * speed, and the gains as its motor controller has them, tuned, or else the definition's seed.
   */
  static JointSetup driven(RobotMath math, Joint joint) {
    MechanismControl control = ControllerSettings.control(joint);
    ControllerPlan plan = ControllerPlan.of(math, joint, control);
    return new JointSetup(
        control, Travel.of(math, joint, control), math.freeSpeed(joint), plan.kA(), plan.kV());
  }

  /**
   * The parts of a stand-in for a joint with no mechanism: its joint has no motors and may have no
   * control, so nothing about how it is driven is worked out. Its travel is everywhere, taken as it
   * is, and its speed and gains are nothing.
   */
  static JointSetup standIn(Joint joint, MechanismControl control) {
    Travel everywhere =
        new Travel(
            joint,
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            Travel.Taking.AS_IT_IS,
            1,
            1,
            0);
    return new JointSetup(control, everywhere, 0, 0, 0);
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismWiring;

/** The joint driver for a TalonFX, against the contract every joint driver keeps. */
class JointIOTalonFxConformanceTest extends JointIOConformance {
  JointIOTalonFxConformanceTest() {
    super(new TalonJointBench(MechanismWiring.Controller.TALON_FX));
  }
}

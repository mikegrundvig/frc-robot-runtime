package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismWiring;

/** The joint driver for a TalonFXS, against the contract every joint driver keeps. */
class JointIOTalonFxsConformanceTest extends JointIOConformance {
  JointIOTalonFxsConformanceTest() {
    super(new TalonJointBench(MechanismWiring.Controller.TALON_FXS));
  }
}

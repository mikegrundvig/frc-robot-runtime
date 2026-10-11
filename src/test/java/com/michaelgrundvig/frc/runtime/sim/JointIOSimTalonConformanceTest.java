package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIOConformance;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;

/** The simulator's joint, emulating a TalonFX, against the contract every joint driver keeps. */
class JointIOSimTalonConformanceTest extends JointIOConformance {
  JointIOSimTalonConformanceTest() {
    super(new SimJointBench(MechanismWiring.Controller.TALON_FX));
  }
}

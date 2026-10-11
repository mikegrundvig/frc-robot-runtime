package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIOConformance;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;

/** The simulator's joint, emulating a SPARK MAX, against the contract every joint driver keeps. */
class JointIOSimSparkConformanceTest extends JointIOConformance {
  JointIOSimSparkConformanceTest() {
    super(new SimJointBench(MechanismWiring.Controller.SPARK_MAX));
  }
}

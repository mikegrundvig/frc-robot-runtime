package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismWiring;

/** The joint driver for a SPARK MAX, against the contract every joint driver keeps. */
class JointIOSparkMaxConformanceTest extends JointIOConformance {
  JointIOSparkMaxConformanceTest() {
    super(new SparkJointBench(MechanismWiring.Controller.SPARK_MAX));
  }
}

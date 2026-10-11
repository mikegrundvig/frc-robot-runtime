package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismWiring;

/** The joint driver for a SPARK Flex, against the contract every joint driver keeps. */
class JointIOSparkFlexConformanceTest extends JointIOConformance {
  JointIOSparkFlexConformanceTest() {
    super(new SparkJointBench(MechanismWiring.Controller.SPARK_FLEX));
  }
}

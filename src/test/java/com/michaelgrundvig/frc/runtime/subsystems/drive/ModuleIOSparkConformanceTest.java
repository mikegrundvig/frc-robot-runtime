package com.michaelgrundvig.frc.runtime.subsystems.drive;

/** The driver for a SPARK module, against the contract every module driver keeps. */
class ModuleIOSparkConformanceTest extends ModuleIOConformance {
  ModuleIOSparkConformanceTest() {
    super(new SparkModuleBench());
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

/** The driver for a TalonFX module, against the contract every module driver keeps. */
class ModuleIOTalonConformanceTest extends ModuleIOConformance {
  ModuleIOTalonConformanceTest() {
    super(new TalonModuleBench());
  }
}

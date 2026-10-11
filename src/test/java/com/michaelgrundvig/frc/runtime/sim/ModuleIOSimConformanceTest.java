package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIOConformance;

/** The driver for the simulator's module, against the contract every module driver keeps. */
class ModuleIOSimConformanceTest extends ModuleIOConformance {
  ModuleIOSimConformanceTest() {
    super(new SimModuleBench());
  }
}

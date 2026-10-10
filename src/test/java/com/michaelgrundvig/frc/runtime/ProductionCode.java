package com.michaelgrundvig.frc.runtime;

import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;

/**
 * The runtime's compiled classes, and those of the libraries it runs (the world model,
 * localization, the simulation, Pitwall), as ArchUnit reads them: imported once, the first time a
 * test asks, and shared by every test that checks them. The rules that hold for every class (no
 * static mutable state, configuration built by name) check them here, once.
 */
final class ProductionCode {
  static final JavaClasses CLASSES =
      new ClassFileImporter()
          .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
          // The test fixtures (the simulator's harness, the simulation's truth suite's known gaps)
          // are tests' too.
          .withImportOption(
              location ->
                  !location.contains("/testFixtures/") && !location.contains("-test-fixtures"))
          .importPackages(
              "com.michaelgrundvig.frc.runtime",
              "com.michaelgrundvig.frc.world",
              "com.michaelgrundvig.frc.localization",
              "com.michaelgrundvig.frc.simulation",
              "com.michaelgrundvig.frc.pitwall");

  private ProductionCode() {}
}

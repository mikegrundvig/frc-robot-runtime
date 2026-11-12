package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** A test that can't run on this computer is skipped, but on Linux CI, where it fails. */
class TestComputerTest {
  @Test
  void onlyLinuxCiFailsATestItCantRun() {
    record Row(String name, Optional<String> ci, String os, boolean fails) {}
    List<Row> rows =
        List.of(
            new Row("a developer's Linux computer", Optional.empty(), "Linux", false),
            new Row("Linux CI", Optional.of("true"), "Linux", true),
            new Row("Linux CI, as some runners spell it", Optional.of("TRUE"), "Linux", true),
            new Row("CI turned off", Optional.of("false"), "Linux", false),
            new Row("Windows CI", Optional.of("true"), "Windows 11", false),
            new Row("macOS CI", Optional.of("true"), "Mac OS X", false));
    for (Row row : rows) {
      assertThat(TestComputer.onLinuxCi(row.ci(), row.os())).as(row.name()).isEqualTo(row.fails());
    }
  }

  @Test
  void aComputerThatCanRunsTheTest() {
    TestComputer.needs(true, "nothing");
  }
}

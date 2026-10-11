package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

/**
 * The joint contract's two lists are one: the clauses {@link JointIO}'s Javadoc numbers, and the
 * clauses the conformance suite names ({@link JointClause}).
 */
class JointContractTest {
  /** A clause as the Javadoc numbers it: {@code <li><b>J12.</b>}. */
  private static final Pattern NUMBERED = Pattern.compile("<li><b>(J\\d+)\\.</b>");

  @Test
  void everyClauseTheJavadocNumbersIsOneTheSuiteNamesInTheSameOrder() throws IOException {
    Path source =
        Path.of(
            System.getProperty("frc.projectDir"),
            "src/main/java/com/michaelgrundvig/frc/runtime/subsystems/mechanisms/JointIO.java");
    List<String> numbered = new ArrayList<>();
    Matcher clause = NUMBERED.matcher(Files.readString(source));
    while (clause.find()) {
      numbered.add(clause.group(1));
    }

    assertThat(numbered)
        .containsExactlyElementsOf(Arrays.stream(JointClause.values()).map(Enum::name).toList());
  }
}

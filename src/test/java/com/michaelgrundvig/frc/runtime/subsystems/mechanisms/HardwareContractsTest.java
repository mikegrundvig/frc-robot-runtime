package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleClause;
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
 * Each hardware contract's two lists are one: the clauses its interface's Javadoc numbers ({@link
 * JointIO}, {@code ModuleIO}), and the clauses its conformance suite names ({@link JointClause},
 * {@link ModuleClause}).
 */
class HardwareContractsTest {
  /** A clause as the Javadoc numbers it: {@code <li><b>J12.</b>}. */
  private static final Pattern NUMBERED = Pattern.compile("<li><b>([JM]\\d+)\\.</b>");

  /** The clauses a source file's Javadoc numbers, in the order it numbers them. */
  private static List<String> numberedIn(String sourceFile) throws IOException {
    Path source =
        Path.of(
            System.getProperty("frc.projectDir"),
            "src/main/java/com/michaelgrundvig/frc/runtime/subsystems",
            sourceFile);
    List<String> numbered = new ArrayList<>();
    Matcher clause = NUMBERED.matcher(Files.readString(source));
    while (clause.find()) {
      numbered.add(clause.group(1));
    }
    return numbered;
  }

  @Test
  void everyJointClauseTheJavadocNumbersIsOneTheSuiteNamesInTheSameOrder() throws IOException {
    assertThat(numberedIn("mechanisms/JointIO.java"))
        .containsExactlyElementsOf(Arrays.stream(JointClause.values()).map(Enum::name).toList());
  }

  /** The module's Javadoc groups its clauses by kind, so only the set is the same. */
  @Test
  void everyModuleClauseTheJavadocNumbersIsOneTheSuiteNames() throws IOException {
    assertThat(numberedIn("drive/ModuleIO.java"))
        .containsExactlyInAnyOrderElementsOf(
            Arrays.stream(ModuleClause.values()).map(Enum::name).toList());
  }
}

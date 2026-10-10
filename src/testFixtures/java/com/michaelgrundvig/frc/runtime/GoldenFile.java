package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/**
 * A recording of a seeded session, row by row, beside the test that makes it: the session must come
 * out the same, numbers within a millionth and everything else exactly. With the environment
 * variable {@code GOLDEN_UPDATE=1} the test records the session instead, for a change meant to move
 * it; commit the file with the change, saying why it moved.
 */
public final class GoldenFile {
  /** Close enough for the same arithmetic on another processor; far below any real change. */
  private static final double TOLERANCE = 1e-6;

  private GoldenFile() {}

  /**
   * Checks a session's rows against the recording beside the test, or records them.
   *
   * @param test the test class, whose package the recording sits in
   * @param name the recording's file name
   * @param header what each row's columns are, for the failure's message
   */
  public static void check(Class<?> test, String name, String header, List<String> actual)
      throws IOException {
    if ("1".equals(System.getenv("GOLDEN_UPDATE"))) {
      Path file =
          Path.of(System.getProperty("frc.projectDir", "."), "src", "test", "resources")
              .resolve(test.getPackageName().replace('.', '/'))
              .resolve(name);
      Files.createDirectories(file.getParent());
      Files.writeString(file, String.join("\n", actual) + "\n", StandardCharsets.UTF_8);
      return;
    }
    List<String> expected = recorded(test, name);
    for (int i = 0; i < Math.min(expected.size(), actual.size()); i++) {
      if (!matches(expected.get(i), actual.get(i))) {
        fail(
            "The session differs from the recording %s at row %d (%s):%n  recorded %s%n  now      %s"
                + "%nIf the change is meant, record it again with GOLDEN_UPDATE=1 (see"
                + " docs/testing.md).",
            name, i, header, expected.get(i), actual.get(i));
      }
    }
    assertThat(actual).as("rows").hasSameSizeAs(expected);
  }

  private static List<String> recorded(Class<?> test, String name) throws IOException {
    try (InputStream in = test.getResourceAsStream(name)) {
      if (in == null) {
        return fail("No recording: run this test with GOLDEN_UPDATE=1 to make " + name);
      }
      return new String(in.readAllBytes(), StandardCharsets.UTF_8).lines().toList();
    }
  }

  /** The same row: numbers within the tolerance, everything else exactly. */
  private static boolean matches(String expected, String actual) {
    String[] want = expected.split(",");
    String[] got = actual.split(",");
    if (want.length != got.length) {
      return false;
    }
    for (int i = 0; i < want.length; i++) {
      if (want[i].equals(got[i])) {
        continue;
      }
      try {
        // Written so a number that became NaN (or stopped being one) differs.
        if (!(Math.abs(Double.parseDouble(want[i]) - Double.parseDouble(got[i])) <= TOLERANCE)) {
          return false;
        }
      } catch (NumberFormatException e) {
        return false;
      }
    }
    return true;
  }
}

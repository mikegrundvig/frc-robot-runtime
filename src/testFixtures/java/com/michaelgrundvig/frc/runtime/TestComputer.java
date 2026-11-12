package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.fail;

import java.util.Locale;
import java.util.Optional;
import org.junit.jupiter.api.Assumptions;

/**
 * What a test needs of the computer it runs on. Where the computer can't do it, the test is skipped
 * on a developer's computer, and fails on Linux CI ({@code CI=true}): Linux is the robot's own
 * system, and its CI runner has everything the tests need, so a skip there would hide a test that
 * never ran.
 */
public final class TestComputer {
  private TestComputer() {}

  /**
   * Skips the rest of the test where the computer can't do what it needs, or fails it on Linux CI.
   *
   * @param can whether this computer can
   * @param cant what it can't do, said as the reason
   */
  public static void needs(boolean can, String cant) {
    if (can) {
      return;
    }
    if (onLinuxCi(Optional.ofNullable(System.getenv("CI")), System.getProperty("os.name", ""))) {
      fail("On Linux CI every test runs, and this one can't: " + cant);
    }
    Assumptions.abort(cant);
  }

  /**
   * Whether this is Linux CI: the {@code CI} variable set to {@code true}, as GitHub Actions sets
   * it, on Linux.
   *
   * @param ci the {@code CI} environment variable, if set
   * @param os the {@code os.name} system property
   */
  static boolean onLinuxCi(Optional<String> ci, String os) {
    if (ci.isEmpty() || !ci.get().equalsIgnoreCase("true")) {
      return false;
    }
    return os.toLowerCase(Locale.ROOT).startsWith("linux");
  }
}

package com.michaelgrundvig.frc.runtime;

/**
 * Which code is running, as the robot's log records it at startup: every log says exactly which
 * build produced it. The team's {@code Main} fills it from the build's generated constants.
 *
 * @param projectName the project's name
 * @param buildDate when it was built, as text
 * @param gitSha the commit
 * @param gitDate when the commit was made, as text
 * @param gitBranch the branch
 * @param changes whether the working tree had changes the commit lacks
 */
public record BuildInfo(
    String projectName,
    String buildDate,
    String gitSha,
    String gitDate,
    String gitBranch,
    Changes changes) {
  /** Whether the code that was built is what the commit holds. */
  public enum Changes {
    /** The tree matched the commit. */
    COMMITTED("All changes committed"),
    /** The tree had edits the commit lacks. */
    UNCOMMITTED("Uncommitted changes"),
    /** Git wasn't available to say. */
    UNKNOWN("Unknown");

    private final String text;

    Changes(String text) {
      this.text = text;
    }

    /** As the log says it. */
    public String text() {
      return text;
    }

    /** From the build's flag: 0 for a clean tree, 1 for edits, anything else for unknown. */
    public static Changes fromFlag(int flag) {
      return switch (flag) {
        case 0 -> COMMITTED;
        case 1 -> UNCOMMITTED;
        default -> UNKNOWN;
      };
    }
  }

  /** For a program built without the generated constants, such as a test's. */
  public static final BuildInfo UNKNOWN =
      new BuildInfo("UNKNOWN", "UNKNOWN", "UNKNOWN", "UNKNOWN", "UNKNOWN", Changes.UNKNOWN);
}

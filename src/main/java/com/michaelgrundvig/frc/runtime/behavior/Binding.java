package com.michaelgrundvig.frc.runtime.behavior;

/**
 * One command bound to a drive team's gamepad, as the binding was made: {@code
 * operator.rightTrigger().whileTrue(intake())} is "operator right trigger: while held: Intake".
 *
 * @param gamepad the gamepad's name: "driver" or "operator"
 * @param button what it's bound to, as the gamepad names it: a button ("right trigger"), buttons
 *     together ("left bumper and right bumper"), or a button with a condition ("right bumper and
 *     ready to shoot")
 * @param how when the command starts, and what stops it
 * @param command the command's name
 */
public record Binding(String gamepad, String button, How how, String command) {
  /** When a bound command starts, and what stops it: one for each of v3's ways to bind. */
  public enum How {
    /** {@code onTrue}: starts as it's pressed, and runs until it ends. */
    ON_PRESS("on press", true),
    /** {@code onFalse}: starts as it's let go, and runs until it ends. */
    ON_RELEASE("on release", false),
    /** {@code whileTrue}: starts as it's pressed, and is canceled as it's let go. */
    WHILE_HELD("while held", true),
    /** {@code whileFalse}: starts as it's let go, and is canceled as it's pressed. */
    WHILE_RELEASED("while released", false),
    /** {@code toggleOnTrue}: each press starts it, or cancels it if it's running. */
    TOGGLE_ON_PRESS("toggled on press", true),
    /** {@code toggleOnFalse}: each release starts it, or cancels it if it's running. */
    TOGGLE_ON_RELEASE("toggled on release", false),
    /** {@code retryWhileTrue}: as while held, started again if it ends while still held. */
    RETRY_WHILE_HELD("while held, again if it ends", true),
    /** {@code retryWhileFalse}: as while released, started again if it ends while released. */
    RETRY_WHILE_RELEASED("while released, again if it ends", false);

    private final String words;
    private final boolean onPress;

    How(String words, boolean onPress) {
      this.words = words;
      this.onPress = onPress;
    }

    /** Whether a press starts it; a release otherwise. */
    public boolean startsOnPress() {
      return onPress;
    }

    /** As a binding says it: "while held". */
    @Override
    public String toString() {
      return words;
    }
  }

  /** As the record says it: "operator right trigger: while held: Intake". */
  @Override
  public String toString() {
    return gamepad + " " + button + ": " + how + ": " + command;
  }
}

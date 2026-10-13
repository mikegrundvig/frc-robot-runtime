package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.record.BindingStart;
import com.michaelgrundvig.frc.world.record.ControlSentences;

/**
 * One command bound to a drive team's gamepad, as the binding was made: {@code
 * operator.rightTrigger().whileTrue(intake())} is "operator right trigger: while held: Intake".
 *
 * @param gamepad the gamepad's name: "driver" or "operator"
 * @param button what the command is bound to, as the gamepad names it. That is a button ("right
 *     trigger"), buttons together ("left bumper and right bumper"), or a button with a condition
 *     ("right bumper and ready to shoot").
 * @param how when the command starts, and what stops it
 * @param command the command's name
 */
public record Binding(String gamepad, String button, BindingStart how, String command) {
  /** As the record says it: "operator right trigger: while held: Intake". */
  @Override
  public String toString() {
    return ControlSentences.binding(gamepad, button, how, command);
  }
}

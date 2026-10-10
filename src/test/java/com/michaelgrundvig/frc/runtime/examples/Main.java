package com.michaelgrundvig.frc.runtime.examples;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotProgram;
import com.michaelgrundvig.frc.world.game.Rebuilt;

/**
 * A team's {@code Main} as README.md shows it, compiled so the README's example is checked
 * (DocExamplesTest). It is never run: the runtime's tests build the robot other ways.
 */
final class Main {
  private Main() {}

  static void main(String... args) {
    RobotProgram.start(
        () -> ExampleRobot.COMPETITION, // which robot this is
        Rebuilt::game, // this season's field and venues
        Controls::new); // what it does
  }
}

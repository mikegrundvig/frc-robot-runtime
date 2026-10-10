package com.michaelgrundvig.frc.runtime.examples;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.Monkey;

/** The guide's robot with the guide's controls, for the monkey to drive ({@code MonkeySimTest}). */
public final class ExampleMonkey {
  private ExampleMonkey() {}

  public static final Monkey.Robot ROBOT = Monkey.robot(ExampleRobot.COMPETITION, Controls::new);
}

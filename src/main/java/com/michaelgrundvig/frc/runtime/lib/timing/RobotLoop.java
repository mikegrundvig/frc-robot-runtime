package com.michaelgrundvig.frc.runtime.lib.timing;

/**
 * The robot loop's rate, in one place: {@code Robot} runs at it, and everything that counts loops
 * or steps a loop's worth of time takes it from here. Change it here and nothing else needs
 * changing.
 */
public final class RobotLoop {
  /** How often the robot loop runs. */
  public static final double FREQUENCY_HZ = 50;

  /** How long one loop is. */
  public static final double PERIOD_SECONDS = 1 / FREQUENCY_HZ;

  private RobotLoop() {}

  /** How many loops a stretch of time takes, to the nearest loop. */
  public static int loops(double seconds) {
    return (int) Math.round(seconds * FREQUENCY_HZ);
  }
}

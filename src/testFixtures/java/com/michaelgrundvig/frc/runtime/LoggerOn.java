package com.michaelgrundvig.frc.runtime;

import org.littletonrobotics.junction.Logger;
import org.wpilib.system.RobotController;

/**
 * AdvantageKit's logger running in a test, for a test of what logging costs. With the logger off,
 * {@code Logger.recordOutput} returns at once and costs nothing; with it running, every call makes
 * a log value and builds the key's full name, whether or not the value changed.
 *
 * <p>The logger starts once in a JVM and cannot be started again, so a test that uses this runs in
 * a JVM of its own ({@link InItsOwnJvm}) and the logger stays on in it. It has no receivers, so
 * nothing is written, and its cycle ({@code Logger.periodic}) is not run: a test measures what the
 * robot's own code makes when it logs, not what the logger does with a loop's table.
 */
public final class LoggerOn {
  private LoggerOn() {}

  /** Starts the logger. Where it is already running, this changes nothing. */
  public static void start() {
    // The logger refuses to start unless a LoggedRobot started it. A test is not one.
    Logger.AdvancedHooks.disableRobotBaseCheck();
    // Its console capture would take the test's output.
    Logger.disableConsoleCapture();
    Logger.start();
    // Starting the logger gave the robot's clock to it, and it moves only with the logger's own
    // cycle. The test's clock is the simulator's, as it was before.
    RobotController.setTimeSource(RobotController::getMonotonicTime);
  }
}

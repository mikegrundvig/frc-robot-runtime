package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.dashboard.DashboardStream;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * The robot program started as it starts on a robot ({@link RunMode#REAL}): on a robot's
 * Systemcore, and on a laptop driving real devices. Everywhere else the tests build the robot in
 * the simulator, so this is the one test of the path a match runs. It starts the program itself,
 * with its log, its Pitwall and its devices (here the simulator's HAL's), runs a few loops, and
 * checks where the log went.
 *
 * <p>AdvantageKit's logger starts once in a JVM, so each start is in a JVM of its own.
 */
@InItsOwnJvm
class RobotStartsAsOnARobotTest {
  /** How many loops the started program runs. */
  private static final int LOOPS = 5;

  /** Where the log goes: a robot's USB drive, or a laptop's program folder. */
  @TempDir Path drive;

  @Test
  void onASystemcoreItLogsToItsDriveAndWatchesIt() throws IOException {
    startsAndRuns(RunsOn.SYSTEMCORE, drive);
  }

  @Test
  void onALaptopDrivingRealDevicesItLogsToItsFolder() throws IOException {
    startsAndRuns(RunsOn.DESKTOP, drive);
  }

  /**
   * Starts the program in real mode on a computer, with its log in a logs folder in a temporary
   * one, runs a few loops, and checks what it logged to and watched, and that nothing threw.
   */
  private static void startsAndRuns(RunsOn runsOn, Path drive) throws IOException {
    assertThat(HAL.initialize()).as("the simulated HAL started").isTrue();
    // No NetworkTables server for the world to reach, and the page on a free port.
    NetworkTableInstance.getDefault().startLocal();
    System.setProperty("pitwall.port", "0");
    Path logs = drive.resolve("logs");

    Robot robot =
        new Robot(
            new RobotProgram(
                () -> ExampleRobot.COMPETITION,
                Rebuilt.GAME,
                ControlsMaker.NONE,
                BuildInfo.UNKNOWN),
            RunMode.REAL,
            runsOn,
            logs);
    for (int i = 0; i < LOOPS; i++) {
      robot.robotPeriodic();
    }
    Logger.end(); // the log is written, and closed
    LoopHealth health = robot.loopHealth();

    assertThat(robot.mode()).isEqualTo(RunMode.REAL);
    assertThat(health.isThrewAlertActive()).as("nothing threw in its loops").isFalse();
    assertThat(health.watchesTheLogDrive())
        .as("a robot watches its USB drive; a laptop has none")
        .isEqualTo(runsOn == RunsOn.SYSTEMCORE);
    assertThat(logsIn(logs)).as("its log, in the log folder").hasSize(1);
    assertThat(Robot.logReceivers(RunMode.REAL, logs))
        .as("a file, and the dashboards' stream, thinned on the field")
        .extracting(LogDataReceiver::getClass)
        .containsExactly(WPILOGWriter.class, DashboardStream.class);
  }

  private static List<Path> logsIn(Path folder) throws IOException {
    try (Stream<Path> files = Files.list(folder)) {
      return files.filter(file -> file.toString().endsWith(".wpilog")).toList();
    }
  }
}

package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.api.io.TempDir;

/** Whether the loop keeps up, and its log with it: the loop's times given by hand. */
@ExtendWith(RobotSim.class)
class LoopHealthTest {
  private static final long MS = 1_000_000L;

  private final LoopHealth health = new LoopHealth();

  @AfterEach
  void tearDown() {
    health.close();
  }

  /** One loop, starting at a time and taking so long, as the robot runs it. */
  private void loop(long startMs, long tookMs, boolean logDropped) {
    health.start(startMs * MS);
    health.monitor().mark("Commands", (startMs + tookMs) * MS);
    health.finish((startMs + tookMs) * MS, logDropped);
  }

  @Test
  void anOverrunRaisesAnAlertNamingWhereTheTimeWent() {
    loop(0, 5, false);
    String quick = health.describe();
    boolean alertAfterQuick = health.isOverrunAlertActive();

    loop(20, 35, false);

    assertThat(quick).isEqualTo("Worst 5 ms lately, none overran");
    assertThat(alertAfterQuick).isFalse();
    assertThat(health.isOverrunAlertActive()).isTrue();
    assertThat(health.overrunAlertText())
        .isEqualTo(
            "The loop overran 20 ms 1 time in the last 10 s; the last took 35 ms, 35 of them in"
                + " Commands");
    assertThat(health.describe()).isEqualTo("Overran 1 time lately, worst 35 ms");
  }

  /** Time lost between loops (writing the log, garbage collection) shows as a late start. */
  @Test
  void aLateStartRaisesTheAlertThoughTheRobotsCodeWasQuick() {
    loop(0, 5, false);
    loop(35, 5, false); // 15 ms after its time

    assertThat(health.isOverrunAlertActive()).isTrue();
    assertThat(health.overrunAlertText())
        .startsWith("1 loop started late in the last 10 s: time lost between loops");
  }

  /** A log with a gap keeps its gap: the alert stays raised once data was dropped. */
  @Test
  void droppedLogDataStaysRaisedForTheSession() {
    loop(0, 5, true);
    loop(20, 5, false);

    assertThat(health.isLoggingBehindAlertActive()).isTrue();
  }

  /** A loop whose commands threw, on a robot. */
  private void failingLoop(long startMs, RuntimeException thrown) {
    health.start(startMs * MS);
    health.threw(thrown);
    health.monitor().mark("Commands", (startMs + 1) * MS);
    health.finish((startMs + 1) * MS, false);
  }

  /**
   * Each place commands threw from is reported once, and the alert stays raised. A loop that runs
   * starts the count again; in {@value LoopHealth#FAILING_LOOPS} failing loops in a row, the last
   * rethrows what it was told, so the program ends.
   */
  @Test
  void commandsThatThrowAreReportedOnceEachAndTooManyLoopsInARowEndTheProgram() {
    RuntimeException broken = new IllegalStateException("broken");
    RuntimeException other = new IllegalArgumentException();
    failingLoop(0, broken);
    failingLoop(20, broken);
    failingLoop(40, other);
    loop(60, 1, false);
    loop(80, 1, false);
    boolean raised = health.isThrewAlertActive();
    for (int i = 1; i < LoopHealth.FAILING_LOOPS; i++) {
      failingLoop(80 + 20L * i, broken);
    }

    assertThatThrownBy(() -> failingLoop(1000, broken)).isSameAs(broken);
    assertThat(raised).as("raised for the session").isTrue();
    assertThat(health.threwAlertText())
        .startsWith(
            "Commands threw, and the robot carried on without them: IllegalStateException: broken,"
                + " at com.michaelgrundvig.frc.runtime.LoopHealthTest.")
        .containsOnlyOnce("IllegalStateException")
        .contains("; IllegalArgumentException, at com.michaelgrundvig.frc.runtime.LoopHealthTest.")
        .endsWith(". Fix the code: the stack is in the Driver Station's log");
  }

  @Test
  void aSessionWithoutADriveSaysItIsntLogged(@TempDir Path folder) {
    File drive = folder.toFile();
    File missing = folder.resolve("U").toFile();

    assertThat(LoopHealth.logDriveProblem(drive, 0)).isNull();
    assertThat(LoopHealth.logDriveProblem(missing, 0))
        .isEqualTo("No USB drive at " + missing + ": this session isn't being logged");
  }

  @Test
  void aNearlyFullDriveSaysLoggingWillStop(@TempDir Path folder) {
    assertThat(LoopHealth.logDriveProblem(folder.toFile(), Long.MAX_VALUE))
        .contains("MB free: logging stops when it's full");
  }

  @Test
  void aSessionOnAReadOnlyDriveSaysItIsntLogged(@TempDir Path folder) throws Exception {
    File readOnly = Files.createDirectory(folder.resolve("locked")).toFile();
    // Linux, where the robot runs, can lock a folder against writing. Windows ignores a folder's
    // read-only flag, and root can write anywhere, so there this case can't be set up: skipped.
    assumeTrue(
        readOnly.setWritable(false) && !readOnly.canWrite(),
        "this system can't make a folder read-only");

    assertThat(LoopHealth.logDriveProblem(readOnly, 0)).contains("can't be written");
  }
}

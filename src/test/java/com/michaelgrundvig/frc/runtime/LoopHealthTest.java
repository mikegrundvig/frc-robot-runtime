package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.entry;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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
    health.start(startMs * MS, Robot.AfterAThrow.CARRIES_ON);
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
    health.start(startMs * MS, Robot.AfterAThrow.CARRIES_ON);
    health.threw(LoopSection.COMMANDS, thrown);
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
            "The loop threw, and the robot carried on: Commands threw IllegalStateException:"
                + " broken, at com.michaelgrundvig.frc.runtime.LoopHealthTest.")
        .containsOnlyOnce("IllegalStateException")
        .contains(
            "; Commands threw IllegalArgumentException, at"
                + " com.michaelgrundvig.frc.runtime.LoopHealthTest.")
        .endsWith(". Fix the code: the stack is in the Driver Station's log");
  }

  /**
   * One loop on a robot in which these sections throw.
   *
   * @return how the loop went, as it was finishing
   */
  private LoopHealth.ThisLoop loopThrowingIn(long startMs, List<LoopSection> sections) {
    health.start(startMs * MS, Robot.AfterAThrow.CARRIES_ON);
    for (LoopSection section : sections) {
      health.threw(section, new IllegalStateException("broken in " + section));
    }
    LoopHealth.ThisLoop went = health.thisLoop();
    health.monitor().mark("Commands", (startMs + 1) * MS);
    health.finish((startMs + 1) * MS, false);
    return went;
  }

  /**
   * What a loop's throws make of it: a section that reads sensors leaves its readings untrusted,
   * the commands or the outputs fail it, and a section aside from control leaves it going well.
   */
  @Test
  void eachSectionsPartSaysWhatItsThrowMakesOfTheLoop() {
    Map<List<LoopSection>, LoopHealth.ThisLoop> went = new LinkedHashMap<>();
    List<List<LoopSection>> rows =
        List.of(
            List.of(),
            List.of(LoopSection.PITWALL),
            List.of(LoopSection.DASHBOARD, LoopSection.FIELD, LoopSection.VENUES),
            List.of(LoopSection.COMMANDS),
            List.of(LoopSection.OUTPUTS),
            List.of(LoopSection.DRIVE),
            List.of(LoopSection.COMMANDS, LoopSection.JOINTS),
            List.of(LoopSection.VISION_AND_LOCALIZATION, LoopSection.OUTPUTS));
    long startMs = 0;
    for (List<LoopSection> row : rows) {
      went.put(row, loopThrowingIn(startMs, row));
      startMs += 20;
    }

    assertThat(went)
        .containsExactly(
            entry(List.of(), LoopHealth.ThisLoop.WELL),
            entry(List.of(LoopSection.PITWALL), LoopHealth.ThisLoop.WELL),
            entry(
                List.of(LoopSection.DASHBOARD, LoopSection.FIELD, LoopSection.VENUES),
                LoopHealth.ThisLoop.WELL),
            entry(List.of(LoopSection.COMMANDS), LoopHealth.ThisLoop.FAILED),
            entry(List.of(LoopSection.OUTPUTS), LoopHealth.ThisLoop.FAILED),
            entry(List.of(LoopSection.DRIVE), LoopHealth.ThisLoop.ON_UNTRUSTED_READINGS),
            entry(
                List.of(LoopSection.COMMANDS, LoopSection.JOINTS),
                LoopHealth.ThisLoop.ON_UNTRUSTED_READINGS),
            entry(
                List.of(LoopSection.VISION_AND_LOCALIZATION, LoopSection.OUTPUTS),
                LoopHealth.ThisLoop.ON_UNTRUSTED_READINGS));
  }

  /**
   * A loop in which several sections fail is one failed loop, and a section aside from control
   * never fails one: only {@value LoopHealth#FAILING_LOOPS} failed loops in a row end the program.
   */
  @Test
  void aLoopFailsOnceHoweverManyOfItsSectionsThrow() {
    Map<List<LoopSection>, Integer> endedOn = new LinkedHashMap<>();
    List<List<LoopSection>> rows =
        List.of(
            List.of(LoopSection.JOINTS, LoopSection.COMMANDS, LoopSection.OUTPUTS),
            List.of(LoopSection.DASHBOARD, LoopSection.PITWALL));
    long startMs = 0;
    for (List<LoopSection> row : rows) {
      int ended = 0;
      for (int loop = 1; loop <= 4 * LoopHealth.FAILING_LOOPS && ended == 0; loop++) {
        startMs += 20;
        try {
          loopThrowingIn(startMs, row);
        } catch (IllegalStateException e) {
          ended = loop;
        }
      }
      endedOn.put(row, ended);
      loop(startMs + 20, 1, false); // a loop that goes well starts the count again
      startMs += 20;
    }

    assertThat(endedOn)
        .containsExactly(
            entry(
                List.of(LoopSection.JOINTS, LoopSection.COMMANDS, LoopSection.OUTPUTS),
                LoopHealth.FAILING_LOOPS),
            entry(List.of(LoopSection.DASHBOARD, LoopSection.PITWALL), 0));
  }

  /**
   * What one joint's throw suppressed of another's is reported too, each place once, and in the
   * simulator and tests any section's throw goes straight on.
   */
  @Test
  void whatAThrowSuppressedIsReportedAndTheSimulatorThrowsOn() {
    RuntimeException first = new IllegalStateException("the pivot");
    first.addSuppressed(new IllegalArgumentException("the hood"));
    health.start(0, Robot.AfterAThrow.CARRIES_ON);
    health.threw(LoopSection.OUTPUTS, first);
    health.finish(MS, false);
    RuntimeException inTheSimulator = new IllegalStateException("in the simulator");
    health.start(20 * MS, Robot.AfterAThrow.ENDS_THE_PROGRAM);

    assertThatThrownBy(() -> health.threw(LoopSection.DASHBOARD, inTheSimulator))
        .isSameAs(inTheSimulator);
    assertThat(health.threwAlertText())
        .contains("Outputs threw IllegalStateException: the pivot, at ")
        .contains("; Outputs threw IllegalArgumentException: the hood, at ")
        .doesNotContain("in the simulator");
  }

  @Test
  void aSessionWithoutADriveSaysItIsntLogged(@TempDir Path folder) {
    File drive = folder.toFile();
    File missing = folder.resolve("U").toFile();

    assertThat(LoopHealth.logDriveProblem(drive, 0)).isEmpty();
    assertThat(LoopHealth.logDriveProblem(missing, 0))
        .contains("No USB drive at " + missing + ": this session isn't being logged");
  }

  @Test
  void aNearlyFullDriveSaysLoggingWillStop(@TempDir Path folder) {
    assertThat(LoopHealth.logDriveProblem(folder.toFile(), Long.MAX_VALUE).orElseThrow())
        .contains("MB free: logging stops when it's full");
  }

  @Test
  void aSessionOnAReadOnlyDriveSaysItIsntLogged(@TempDir Path folder) throws Exception {
    File readOnly = Files.createDirectory(folder.resolve("locked")).toFile();
    // Linux, where the robot runs, can lock a folder against writing. Windows ignores a folder's
    // read-only flag, and root can write anywhere, so there this case can't be set up: skipped,
    // but on Linux CI, where it must run.
    TestComputer.needs(
        readOnly.setWritable(false) && !readOnly.canWrite(),
        "this system can't make a folder read-only");

    assertThat(LoopHealth.logDriveProblem(readOnly, 0).orElseThrow()).contains("can't be written");
  }
}

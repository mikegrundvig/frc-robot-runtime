package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.coprocessors.VisorInstaller.Outcome;
import com.michaelgrundvig.frc.runtime.coprocessors.VisorInstaller.Seen;
import com.michaelgrundvig.frc.visor.VisorPack;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.util.AlertDataJNI;

/**
 * The Visor installer, scenario by scenario, on boards that answer as each says: the guide's
 * robot's cameras on Visor (VisorConfigsTest's: two fronts and a back, each tunable), boards that
 * report what the scenario gives them, and steps that finish when it says. Each loop is 20 ms on
 * the robot's clock.
 */
class VisorInstallerTest {
  private static final String VERSION = "2099.1.2";
  private static final String OLD = "2000.1.1";
  private static final String FRONT_LEFT = "usb-Thrifty_Cam_0001-video-index0";
  private static final String FRONT_RIGHT = "usb-Thrifty_Cam_0002-video-index0";
  private static final String BACK = "usb-Thrifty_Cam_0003-video-index0";
  private static final String OTHER = "usb-Other-video-index0";

  /** Both front cameras, which a board of a scenario has together. */
  private static final List<String> FRONTS = List.of(FRONT_LEFT, FRONT_RIGHT);

  /** The back camera alone, on a board of its own. */
  private static final List<String> BACK_ONLY = List.of(BACK);

  /** Every one of the robot's Visor cameras, on one board. */
  private static final List<String> ALL = List.of(FRONT_LEFT, FRONT_RIGHT, BACK);

  /** The back camera's index in the robot's list. */
  private static final int BACK_INDEX = 3;

  private static final byte[] RELEASE = {1, 2, 3};

  /** The release's SHA-256, as the robot's Visor client names it. */
  private static final String RELEASE_SHA256 = VisorPack.sha256(RELEASE);

  /** The start of the alert for cameras on no board, which most scenarios leave aside. */
  private static final String UNPLACED = "Visor cameras plugged into no coprocessor: ";

  /** A scenario: what's done and what's seen, in order. */
  record Scenario(String name, Consumer<World> script) {
    @Override
    public String toString() {
      return name;
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("scenarios")
  void scenario(Scenario scenario) {
    try (World world = new World()) {
      scenario.script().accept(world);
    }
  }

  static Stream<Scenario> scenarios() {
    return Stream.of(
        new Scenario(
            "a fresh board gets the release, then its configuration, and is left alone once right",
            w -> {
              FakeBoard a = w.board("vision-a", report("", "", List.of()));
              w.loops(5);
              assertThat(a.asked()).as("installed, naming no cameras").containsExactly("install");
              assertThat(w.alerts()).contains("Installing Visor " + VERSION + " on vision-a");
              a.last().finish(Outcome.WORKED, "");
              w.loops(5);
              assertThat(a.asked()).as("its report hasn't caught up yet").hasSize(1);
              a.says(report(VERSION, "", ALL));
              w.seconds(10);
              assertThat(a.asked()).containsExactly("install", "configure " + wanted(ALL));
              a.last().finish(Outcome.WORKED, "");
              a.says(report(VERSION, wanted(ALL), ALL));
              w.seconds(120);
              assertThat(a.asked()).as("right: nothing more").hasSize(2);
              assertThat(w.reads).as("the release read once").isEqualTo(1);
              assertThat(w.alerts()).isEmpty();
            }),
        new Scenario(
            "each board is brought to what it needs, and one without Visor's pack is left alone",
            w -> {
              FakeBoard current = w.board("vision-a", report(VERSION, wanted(FRONTS), FRONTS));
              FakeBoard stale = w.board("vision-b", report(VERSION, "old", BACK_ONLY));
              FakeBoard detector = w.board("detector", Seen.WITHOUT_VISOR);
              FakeBoard spare = w.board("spare", report(VERSION, "", List.of(OTHER)));
              w.seconds(60);
              assertThat(current.asked()).isEmpty();
              assertThat(stale.asked()).containsExactly("configure " + wanted(BACK_ONLY));
              assertThat(detector.asked()).as("no Visor pack").isEmpty();
              assertThat(spare.asked())
                  .as("none of the robot's cameras, nor a configuration")
                  .isEmpty();
              assertThat(w.unplaced()).as("every camera placed").isEmpty();
            }),
        new Scenario(
            "a board running cameras the robot no longer has is given a configuration with none",
            w -> {
              FakeBoard b = w.board("vision-b", report(VERSION, wanted(BACK_ONLY), List.of(OTHER)));
              w.loops(1);
              assertThat(b.asked()).containsExactly("configure " + wanted(List.of()));
              b.last().finish(Outcome.REFUSED, "exit isn't 0: Visor refused the configuration");
              w.seconds(180);
              assertThat(b.asked()).as("refused: not tried again").hasSize(1);
              assertThat(w.alerts())
                  .containsExactly(
                      "Visor on vision-b runs cameras that aren't this robot's, and refused the"
                          + " configuration with none: exit isn't 0: Visor refused the"
                          + " configuration");
            }),
        new Scenario(
            "a robot with no camera on Visor leaves every board's Visor alone",
            w -> {
              w.robot(ExampleRobot.SIM.definition().cameras());
              FakeBoard a = w.board("vision-a", report(OLD, "old", List.of()));
              FakeBoard b = w.board("vision-b", report(VERSION, "old", ALL));
              w.seconds(60);
              assertThat(a.asked()).as("its Visor pack is about to be taken away").isEmpty();
              assertThat(b.asked()).isEmpty();
              assertThat(w.reads).as("the release isn't read").isZero();
              assertThat(w.alerts()).isEmpty();
              assertThat(w.unplaced()).isEmpty();
            }),
        new Scenario(
            "a live tune waits while enabled, said as pending, and reaches its board once disabled",
            w -> {
              FakeBoard b = w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.loops(5);
              assertThat(b.asked()).as("as committed: nothing to do").isEmpty();
              assertThat(w.installer.pending(BACK_INDEX)).isFalse();

              w.enabled = true;
              assertThat(w.tune(BACK_INDEX, Map.of("exposure.time", 1.2, "gain.value", 55.0)))
                  .isTrue();
              w.loops(50);
              assertThat(b.asked()).as("enabled: it waits").isEmpty();
              assertThat(w.installer.pending(BACK_INDEX)).isTrue();
              assertThat(w.installer.pending(0)).as("its board isn't known").isFalse();
              assertThat(w.alerts())
                  .containsExactly(
                      "Visor's configuration for "
                          + VisorConfigsTest.cameras().get(BACK_INDEX).name()
                          + " reaches its coprocessor once the robot is disabled off the field");

              w.enabled = false;
              w.loops(1);
              String tuned = w.wantedNow(BACK_ONLY);
              assertThat(w.tomlNow(BACK_ONLY)).contains("exposure_us = 1200\ngain = 55\n");
              assertThat(b.asked()).containsExactly("configure " + tuned);
              b.last().finish(Outcome.WORKED, "");
              b.says(report(VERSION, tuned, BACK_ONLY));
              w.seconds(60);
              assertThat(b.asked()).as("configured: left alone").hasSize(1);
              assertThat(w.installer.pending(BACK_INDEX)).isFalse();

              w.tuning(BACK_INDEX).revert();
              w.loops(1);
              assertThat(b.asked())
                  .containsExactly("configure " + tuned, "configure " + wanted(BACK_ONLY));
            }),
        new Scenario(
            "nothing is changed while the robot is enabled or the field is attached",
            w -> {
              FakeBoard a = w.board("vision-a", report(OLD, "", ALL));
              w.enabled = true;
              w.loops(50);
              w.enabled = false;
              w.field = true;
              w.loops(50);
              assertThat(a.asked()).isEmpty();
              w.field = false;
              w.loops(1);
              assertThat(a.asked()).containsExactly("install");
            }),
        new Scenario(
            "a step under way as the robot is enabled is shown, and finishes",
            w -> {
              FakeBoard a = w.board("vision-a", report(VERSION, "old", ALL));
              w.loops(1);
              w.enabled = true;
              w.loops(1);
              assertThat(w.alerts())
                  .contains(
                      "Configuring Visor on vision-a began before the robot was enabled: it"
                          + " finishes now, and its cameras restart");
              a.last().finish(Outcome.WORKED, "");
              w.loops(1);
              assertThat(w.alerts()).noneMatch(alert -> alert.contains("began before"));
              assertThat(a.asked()).hasSize(1);
            }),
        new Scenario(
            "a configuration Visor refused stays refused until what's wanted changes, a reboot"
                + " too",
            w -> {
              FakeBoard a = w.board("vision-a", report(VERSION, "", FRONTS));
              FakeBoard back = w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              a.uptime = 600;
              w.loops(2);
              a.last()
                  .finish(
                      Outcome.REFUSED,
                      "exit isn't 0: Visor refused the configuration: no such device");
              w.seconds(180);
              assertThat(a.asked()).hasSize(1);
              assertThat(w.alerts())
                  .containsExactly(
                      "Visor on vision-a refused its configuration: exit isn't 0: Visor refused"
                          + " the configuration: no such device");
              a.uptime = 30;
              a.says(report(VERSION, "", FRONTS));
              w.seconds(60);
              assertThat(a.asked()).as("rebooted: still refused").hasSize(1);

              // The back camera moved beside the fronts: another configuration, tried, and it
              // works.
              a.says(report(VERSION, "", ALL));
              back.says(report(VERSION, wanted(BACK_ONLY), List.of()));
              w.loops(2);
              assertThat(a.asked())
                  .containsExactly("configure " + wanted(FRONTS), "configure " + wanted(ALL));
              a.last().finish(Outcome.WORKED, "");
              w.loops(1);
              assertThat(w.alerts()).noneMatch(alert -> alert.contains("refused"));
            }),
        new Scenario(
            "a release that can't be read is said, and read once",
            w -> {
              w.unreadable =
                  new IOException("no such file: /home/systemcore/deploy/visor/x.tar.gz");
              FakeBoard a = w.board("vision-a", report("", "", List.of()));
              w.seconds(120);
              assertThat(a.asked()).isEmpty();
              assertThat(w.reads).as("tried once").isEqualTo(1);
              assertThat(w.alerts())
                  .singleElement()
                  .asString()
                  .startsWith(
                      "Visor on vision-a didn't install: this robot's Visor release can't be used"
                          + " (no such file")
                  .endsWith("deploy again");
            }),
        new Scenario(
            "a release whose SHA-256 isn't the one its client names is never sent",
            w -> {
              w.releaseSha256 = "0".repeat(64);
              FakeBoard a = w.board("vision-a", report(OLD, "", List.of()));
              w.seconds(120);
              assertThat(a.asked()).isEmpty();
              assertThat(w.reads).as("tried once").isEqualTo(1);
              assertThat(w.alerts())
                  .containsExactly(
                      "Visor on vision-a didn't install: this robot's Visor release can't be used"
                          + " (its SHA-256 is "
                          + RELEASE_SHA256
                          + ", not the "
                          + "0".repeat(64)
                          + " its Visor client names): deploy again");
            }),
        new Scenario(
            "a frame's configuration is taken while it's one the installer gave its board, and"
                + " believed until that's known",
            w -> {
              int committed = id(wanted(BACK_ONLY));
              int stale = id(VisorPack.configHash("team = 1234\n"));
              // The program restarts on the field, so nothing is changed until it leaves.
              w.field = true;
              FakeBoard b = w.board("vision-b", Seen.SILENT);
              w.loops(1);
              assertThat(w.installer.otherConfig("vision-b", stale))
                  .as("not reported yet: believed")
                  .isFalse();
              b.says(report(OLD, "old", List.of()));
              w.loops(1);
              assertThat(w.installer.otherConfig("vision-b", stale))
                  .as("another Visor on its disk: its frames are another version's")
                  .isFalse();

              // Running a configuration from before the restart: refused.
              b.says(
                  runs(VERSION, "old", VERSION, VisorPack.configHash("team = 1234\n"), BACK_ONLY));
              w.loops(1);
              assertThat(w.installer.otherConfig("vision-b", stale)).isTrue();
              assertThat(w.installer.otherConfig("vision-b", committed)).isFalse();
              assertThat(w.installer.otherConfig("vision-c", stale)).as("not its board").isFalse();
              w.field = false;
              w.loops(1);
              assertThat(b.asked()).containsExactly("configure " + wanted(BACK_ONLY));
              assertThat(w.installer.otherConfig("vision-b", stale))
                  .as("configuring: still not one it gave")
                  .isTrue();
              b.last().finish(Outcome.WORKED, "");
              b.says(report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.loops(1);
              assertThat(w.installer.otherConfig("vision-b", committed)).isFalse();

              // A tune: while its configure is in flight, the one running stays taken, and so
              // does the tuned one as it lands.
              assertThat(w.tune(BACK_INDEX, Map.of("gain.value", 55.0))).isTrue();
              w.seconds(11);
              int tuned = id(w.wantedNow(BACK_ONLY));
              assertThat(b.asked()).hasSize(2);
              assertThat(w.installer.otherConfig("vision-b", committed)).isFalse();
              assertThat(w.installer.otherConfig("vision-b", tuned)).isFalse();
              b.last().finish(Outcome.WORKED, "");
              b.says(report(VERSION, w.wantedNow(BACK_ONLY), BACK_ONLY));
              w.loops(1);

              // Tuned again while enabled: the first tune, which it runs, stays taken until the
              // second reaches it; the stale one is still refused.
              w.enabled = true;
              assertThat(w.tune(BACK_INDEX, Map.of("gain.value", 60.0))).isTrue();
              w.loops(50);
              assertThat(b.asked()).as("enabled: it waits").hasSize(2);
              assertThat(w.installer.otherConfig("vision-b", tuned)).isFalse();
              assertThat(w.installer.otherConfig("vision-b", id(w.wantedNow(BACK_ONLY)))).isFalse();
              assertThat(w.installer.otherConfig("vision-b", stale)).isTrue();

              // Away for a moment: what it may run is kept.
              b.says(Seen.SILENT);
              w.loops(1);
              assertThat(w.installer.otherConfig("vision-b", stale)).isTrue();
              assertThat(w.installer.otherConfig("vision-b", tuned)).isFalse();
            }),
        new Scenario(
            "the program restarts on the field while a board runs a tuning it pushed before: its"
                + " frames are taken, and the board is said to run another than the committed one",
            w -> {
              FakeBoard b = w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.loops(1);
              assertThat(w.tune(BACK_INDEX, Map.of("gain.value", 55.0))).isTrue();
              w.loops(1);
              String tuned = w.wantedNow(BACK_ONLY);
              assertThat(b.asked()).containsExactly("configure " + tuned);
              b.last().finish(Outcome.WORKED, "");
              b.says(report(VERSION, tuned, BACK_ONLY));
              w.loops(1);

              w.field = true;
              w.restart();
              w.loops(5);
              assertThat(b.asked()).as("on the field: nothing configured").hasSize(1);
              assertThat(w.installer.otherConfig("vision-b", id(tuned))).isFalse();
              assertThat(w.installer.otherConfig("vision-b", id(STALE))).isTrue();
              assertThat(w.alerts())
                  .contains(
                      "Visor on vision-b runs another configuration than the committed one, so"
                          + " the match runs it: a tuned exposure or gain, or one that never"
                          + " reached it. Its frames are taken only if the robot gave it that"
                          + " one. Put tuned values in Robots.java.");

              // Off the field, it's given the committed one, and what was kept has done its work.
              w.field = false;
              w.loops(1);
              assertThat(b.asked())
                  .containsExactly("configure " + tuned, "configure " + wanted(BACK_ONLY));
              b.last().finish(Outcome.WORKED, "");
              b.says(report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.loops(1);
              assertThat(w.installer.otherConfig("vision-b", id(tuned))).isTrue();
              assertThat(w.memory.of("vision-b").ids()).containsExactly(id(wanted(BACK_ONLY)));
            }),
        new Scenario(
            "after a restart, a board running a configuration the robot never gave it is still"
                + " refused",
            w -> {
              FakeBoard b = w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.loops(1);
              w.field = true;
              b.says(runs(VERSION, wanted(BACK_ONLY), VERSION, STALE, BACK_ONLY));
              w.restart();
              w.loops(5);
              assertThat(w.installer.otherConfig("vision-b", id(STALE))).isTrue();
              assertThat(w.installer.otherConfig("vision-b", id(wanted(BACK_ONLY)))).isFalse();
            }),
        new Scenario(
            "what was kept for a board whose committed configuration has changed since is"
                + " forgotten",
            w -> {
              // Kept by a robot whose cameras were another committed configuration for this
              // board: the robot's definition changed, so its tuning means another layout.
              w.memory.kept =
                  Map.of("vision-b", new VisorMemory.Kept(id(STALE), List.of(id(STALE))));
              w.field = true;
              FakeBoard b = w.board("vision-b", runs(VERSION, "", VERSION, STALE, BACK_ONLY));
              w.loops(5);
              assertThat(b.asked()).isEmpty();
              assertThat(w.installer.otherConfig("vision-b", id(STALE))).isTrue();
              assertThat(w.memory.of("vision-b").ids()).doesNotContain(id(STALE));
            }),
        new Scenario(
            "what's kept for a board is at most 4 configurations, newest first, and a board not"
                + " heard since the restart keeps its own",
            w -> {
              int committed = id(wanted(BACK_ONLY));
              List<Integer> before = List.of(0x11111111, 0x22222222, 0x33333333, 0x44444444);
              w.memory.kept =
                  Map.of(
                      "vision-b",
                      new VisorMemory.Kept(committed, before),
                      "vision-a",
                      new VisorMemory.Kept(0x55555555, List.of(0x66666666)));
              w.field = true;
              w.board("vision-a", Seen.SILENT);
              w.board("vision-b", runs(VERSION, "", VERSION, STALE, BACK_ONLY));
              w.loops(5);
              VisorMemory.Kept kept = w.memory.of("vision-b");
              assertThat(kept.ids())
                  .hasSize(VisorMemory.MOST_PER_BOARD)
                  .containsExactly(committed, 0x11111111, 0x22222222, 0x33333333);
              assertThat(w.installer.otherConfig("vision-b", 0x44444444))
                  .as("taken this run, though not kept again")
                  .isFalse();
              assertThat(w.memory.of("vision-a"))
                  .isEqualTo(new VisorMemory.Kept(0x55555555, List.of(0x66666666)));
              // Tuned again and again, each pushed and run: still no more than its own few.
              w.field = false;
              for (int gain = 50; gain < 56; gain++) {
                assertThat(w.tune(BACK_INDEX, Map.of("gain.value", (double) gain))).isTrue();
                w.seconds(11);
                FakeBoard b = w.boards.get(1);
                b.last().finish(Outcome.WORKED, "");
                b.says(report(VERSION, w.wantedNow(BACK_ONLY), BACK_ONLY));
                w.loops(1);
                assertThat(w.memory.of("vision-b").ids().size())
                    .isLessThanOrEqualTo(VisorMemory.MOST_PER_BOARD);
              }
            }),
        new Scenario(
            "a write of what's kept that fails is an alert, and the robot goes on",
            w -> {
              w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.memory.problem = "/home/systemcore/visor: read-only file system";
              w.loops(1);
              assertThat(w.alerts())
                  .containsExactly(
                      "Visor's configurations for the coprocessors can't be kept on the robot's"
                          + " disk (/home/systemcore/visor: read-only file system): after a"
                          + " restart, a board running a tuning is left out until the robot is"
                          + " disabled off the field");
              w.memory.problem = "";
              w.loops(1);
              assertThat(w.alerts()).isEmpty();
            }),
        new Scenario(
            "the release is read off the loop, and the install waits for it",
            w -> {
              w.holdReads = true;
              FakeBoard a = w.board("vision-a", report(OLD, "", List.of()));
              w.loops(50);
              assertThat(a.asked()).as("still reading").isEmpty();
              assertThat(w.reads).isZero();
              w.finishReads();
              w.loops(1);
              assertThat(a.asked()).containsExactly("install");
              assertThat(w.reads).isEqualTo(1);
            }),
        new Scenario(
            "a camera on no board is said once every board with Visor has said what it has",
            w -> {
              FakeBoard fronts = w.board("vision-a", Seen.SILENT);
              w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.board("detector", Seen.WITHOUT_VISOR);
              w.loops(5);
              assertThat(w.unplaced()).as("vision-a hasn't reported").isEmpty();
              fronts.says(report(OLD, "", List.of()));
              w.loops(1);
              assertThat(w.unplaced()).as("vision-a's old Visor can't name its cameras").isEmpty();
              fronts.says(report(VERSION, wanted(FRONTS), FRONTS));
              w.loops(1);
              assertThat(w.unplaced()).isEmpty();
              fronts.says(report(VERSION, wanted(FRONTS), List.of()));
              w.loops(1);
              assertThat(w.unplaced())
                  .as("the detector, without Visor, doesn't hold it back")
                  .singleElement()
                  .asString()
                  .contains(FRONT_LEFT)
                  .contains(FRONT_RIGHT)
                  .doesNotContain(BACK);
            }),
        new Scenario(
            "a board that reboots mid-install is installed again once it's back",
            w -> {
              FakeBoard a = w.board("vision-a", report(OLD, "", List.of()));
              a.uptime = 600;
              w.loops(2);
              assertThat(a.asked()).containsExactly("install");
              a.says(Seen.SILENT);
              a.uptime = Double.NaN;
              w.loops(100);
              a.uptime = 5;
              a.says(report(OLD, "", List.of()));
              a.last().finish(Outcome.NO_ANSWER, "lost: the agent restarted while it ran");
              w.loops(2);
              assertThat(a.asked()).as("back: at once").containsExactly("install", "install");
            }),
        new Scenario(
            "no step starts during a push of the packs, and one a push cuts off is tried again",
            w -> {
              FakeBoard a = w.board("vision-a", report(OLD, "", List.of()));
              a.pushing = true;
              w.seconds(60);
              assertThat(a.asked()).isEmpty();
              a.pushing = false;
              w.loops(1);
              assertThat(a.asked()).containsExactly("install");
              a.pushing = true;
              a.last().finish(Outcome.NO_ANSWER, "lost: the agent restarted while it ran");
              w.seconds(60);
              assertThat(a.asked()).as("pushing").hasSize(1);
              assertThat(w.alerts())
                  .contains(
                      "Visor on vision-a didn't install: lost: the agent restarted while it ran;"
                          + " trying again in 30 s (1 of 4 tries)");
              a.pushing = false;
              w.loops(1);
              assertThat(a.asked()).containsExactly("install", "install");
            }),
        new Scenario(
            "a step that gets no answer is tried 3 more times, 30 s apart, then again once the"
                + " board is back",
            w -> {
              FakeBoard a = w.board("vision-a", report(VERSION, "old", ALL));
              w.loops(1);
              for (int tried = 1; tried <= 4; tried++) {
                assertThat(a.asked()).hasSize(tried);
                a.last().finish(Outcome.NO_ANSWER, "timed out after 30s");
                w.seconds(29);
                assertThat(a.asked()).as("backing off").hasSize(tried);
                w.seconds(2);
              }
              assertThat(a.asked()).hasSize(4);
              assertThat(w.alerts())
                  .containsExactly(
                      "Visor on vision-a didn't take its configuration: timed out after 30s; stopped"
                          + " after 4 tries, until the board reconnects or reboots, or the robot"
                          + " program restarts");
              w.seconds(300);
              assertThat(a.asked()).as("stopped").hasSize(4);
              a.says(Seen.SILENT);
              w.loops(10);
              a.says(report(VERSION, "old", ALL));
              w.loops(1);
              assertThat(a.asked()).as("back: tried again").hasSize(5);
            }),
        new Scenario(
            "a tune while the board is missing reaches it once it's back",
            w -> {
              FakeBoard b = w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.loops(5);
              b.says(Seen.SILENT);
              assertThat(w.tune(BACK_INDEX, Map.of("gain.value", 55.0))).isTrue();
              w.loops(50);
              assertThat(b.asked()).isEmpty();
              assertThat(w.installer.pending(BACK_INDEX)).as("its board isn't known").isFalse();
              b.says(report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.loops(1);
              assertThat(b.asked()).containsExactly("configure " + w.wantedNow(BACK_ONLY));
            }),
        new Scenario(
            "a tune after a configuration was refused is tried",
            w -> {
              FakeBoard b = w.board("vision-b", report(VERSION, "old", BACK_ONLY));
              w.loops(1);
              b.last().finish(Outcome.REFUSED, "exit isn't 0: Visor refused the configuration");
              w.seconds(60);
              assertThat(b.asked()).hasSize(1);
              assertThat(w.tune(BACK_INDEX, Map.of("gain.value", 55.0))).isTrue();
              w.loops(1);
              assertThat(b.asked())
                  .containsExactly(
                      "configure " + wanted(BACK_ONLY), "configure " + w.wantedNow(BACK_ONLY));
            }),
        new Scenario(
            "a value outside its limits is refused, and nothing is sent; the committed ones fit",
            w -> {
              FakeBoard b = w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              assertThat(w.tune(BACK_INDEX, Map.of("gain.value", 70_000.0))).isFalse();
              assertThat(w.tune(BACK_INDEX, Map.of("gain.value", -1.0))).isFalse();
              assertThat(w.tune(BACK_INDEX, Map.of("exposure.time", 0.05))).isFalse();
              assertThat(w.tune(BACK_INDEX, Map.of("exposure.time", 40.0))).isFalse();
              w.seconds(60);
              assertThat(b.asked()).isEmpty();
              // Each camera's committed exposure and gain is within its own tuning's limits.
              assertThat(w.tune(BACK_INDEX, Map.of("exposure.time", 2.5, "gain.value", 40.0)))
                  .isTrue();
              assertThat(w.tune(0, Map.of("exposure.time", 1.0))).isTrue();
              assertThat(w.tune(1, Map.of("exposure.time", 1.0))).isTrue();
              w.seconds(60);
              assertThat(b.asked()).as("the committed values again: nothing to send").isEmpty();
            }),
        new Scenario(
            "a board on an older Visor, which names no cameras, is installed, then configured",
            w -> {
              FakeBoard a = w.board("vision-a", report(OLD, "cfg", List.of()));
              w.loops(1);
              assertThat(a.asked()).containsExactly("install");
              a.last().finish(Outcome.WORKED, "");
              a.says(report(VERSION, "cfg", ALL));
              w.seconds(11);
              assertThat(a.asked()).containsExactly("install", "configure " + wanted(ALL));
            }),
        new Scenario(
            "a Visor on the disk but not running is restarted, then installed again, a few times",
            w -> {
              FakeBoard a = w.board("vision-a", runs(VERSION, wanted(ALL), OLD, wanted(ALL), ALL));
              a.answer = Outcome.WORKED;
              w.loops(1);
              assertThat(a.asked()).containsExactly("configure " + wanted(ALL));
              assertThat(w.alerts()).contains("Restarting Visor " + VERSION + " on vision-a");
              w.seconds(600);
              assertThat(a.asked())
                  .containsExactly(
                      "configure " + wanted(ALL), "install", "install", "install", "install");
              assertThat(w.alerts())
                  .containsExactly(
                      "Visor on vision-a didn't install: it didn't take; stopped after 4 tries,"
                          + " until the board reconnects or reboots, or the robot program"
                          + " restarts");
            }),
        new Scenario(
            "a board that can't say what runs is judged by its disk, and never looped on",
            w -> {
              FakeBoard right = w.board("vision-a", runs(VERSION, wanted(FRONTS), "", "", FRONTS));
              FakeBoard stale = w.board("vision-b", runs(VERSION, "old", "", "", BACK_ONLY));
              stale.answer = Outcome.WORKED;
              w.seconds(600);
              assertThat(right.asked()).isEmpty();
              assertThat(stale.asked())
                  .as("its disk never changes: tried as often as it may be, and no more")
                  .hasSize(4);
            }),
        new Scenario(
            "a configuration on the disk but not running is put in place again, a few times",
            w -> {
              FakeBoard a = w.board("vision-a", runs(VERSION, wanted(ALL), VERSION, "old", ALL));
              a.answer = Outcome.WORKED;
              w.seconds(600);
              assertThat(a.asked())
                  .hasSize(4)
                  .allMatch(step -> step.equals("configure " + wanted(ALL)));
            }),
        new Scenario(
            "on the field, a board running another than the committed configuration is said",
            w -> {
              FakeBoard b = w.board("vision-b", report(VERSION, wanted(BACK_ONLY), BACK_ONLY));
              w.loops(1);
              w.field = true;
              w.loops(1);
              assertThat(w.alerts()).as("committed").isEmpty();
              w.field = false;
              assertThat(w.tune(BACK_INDEX, Map.of("gain.value", 55.0))).isTrue();
              w.loops(1);
              b.last().finish(Outcome.WORKED, "");
              b.says(report(VERSION, w.wantedNow(BACK_ONLY), BACK_ONLY));
              w.loops(1);
              w.field = true;
              w.loops(1);
              assertThat(w.alerts())
                  .containsExactly(
                      "Visor on vision-b runs another configuration than the committed one, so"
                          + " the match runs it: a tuned exposure or gain, or one that never"
                          + " reached it. Its frames are taken only if the robot gave it that"
                          + " one. Put tuned values in Robots.java.");
              w.field = false;
              w.loops(1);
              assertThat(w.alerts()).isEmpty();
            }));
  }

  /** A configuration the robot never gave a board: one from before its cameras changed. */
  private static final String STALE = VisorPack.configHash("team = 1234\n");

  /** The ID frames carry for the configuration with this hash. */
  private static int id(String hash) {
    return VisorPack.configId(hash);
  }

  /** The committed configuration's hash for a board with these devices. */
  private static String wanted(List<String> devices) {
    return VisorConfigs.forBoard(
            VisorConfigsTest.cameras(), 1234, VisorConfigsTest.TAG_SIZE, devices)
        .hash();
  }

  /** A board's report from an image that says what runs: the same as its disk. */
  private static Seen report(String version, String config, List<String> cameras) {
    return runs(version, config, version, config, cameras);
  }

  /** A board's report: what's on its disk, what runs, and its cameras. */
  private static Seen runs(
      String version,
      String config,
      String runningVersion,
      String runningConfig,
      List<String> cameras) {
    return new Seen.Reported(
        new VisorPack.Report(version, config, runningVersion, runningConfig, cameras));
  }

  /** A board's step, which finishes when the scenario says. */
  static final class Pending implements VisorInstaller.Step {
    final String what;
    boolean done;
    Outcome outcome = Outcome.NO_ANSWER;
    String why = "";

    Pending(String what) {
      this.what = what;
    }

    void finish(Outcome how, String reason) {
      done = true;
      outcome = how;
      why = reason;
    }

    @Override
    public boolean done() {
      return done;
    }

    @Override
    public Outcome outcome() {
      return outcome;
    }

    @Override
    public String why() {
      return why;
    }
  }

  /** A board: what it says, and each step it was asked for. */
  static final class FakeBoard implements VisorInstaller.Board {
    final String name;
    Seen seen;
    boolean pushing;
    double uptime = Double.NaN;

    /** How each step ends at once; null for one that waits for the scenario. */
    @Nullable Outcome answer;

    final List<Pending> steps = new ArrayList<>();

    FakeBoard(String name, Seen seen) {
      this.name = name;
      this.seen = seen;
    }

    /** What it says from now on. */
    void says(Seen now) {
      seen = now;
    }

    @Override
    public String name() {
      return name;
    }

    @Override
    public Seen seen() {
      return seen;
    }

    @Override
    public boolean pushPending() {
      return pushing;
    }

    @Override
    public double uptimeSeconds() {
      return uptime;
    }

    @Override
    public String address() {
      return name;
    }

    @Override
    public VisorInstaller.Step install(byte[] input) {
      assertThat(input).isEqualTo(VisorPack.installInput(RELEASE, RELEASE_SHA256));
      return add("install");
    }

    @Override
    public VisorInstaller.Step configure(String config) {
      return add("configure " + VisorPack.configHash(config));
    }

    private Pending add(String what) {
      Pending step = new Pending(what);
      Outcome now = answer;
      if (now != null) {
        step.finish(now, "");
      }
      steps.add(step);
      return step;
    }

    Pending last() {
      return steps.get(steps.size() - 1);
    }

    List<String> asked() {
      return steps.stream().map(step -> step.what).toList();
    }
  }

  /** What the installer keeps across restarts, in memory: what it kept last is what it recalls. */
  static final class FakeMemory implements VisorMemory {
    Map<String, Kept> kept = Map.of();
    String problem = "";
    int keeps;

    @Override
    public Map<String, Kept> recalled() {
      return kept;
    }

    @Override
    public void keep(Map<String, Kept> boards) {
      kept = Map.copyOf(boards);
      keeps++;
    }

    @Override
    public String problem() {
      return problem;
    }

    /** What it last kept for a board, which it must have kept. */
    Kept of(String board) {
      return Objects.requireNonNull(kept.get(board), board);
    }
  }

  /** A scenario's robot: its cameras and tunings, its boards, its state and clock. */
  static final class World implements AutoCloseable {
    Map<String, LiveConfig<Software.Visor>> tunings = VisorTuning.of(VisorConfigsTest.cameras());
    VisorTuning.Live cameras = VisorTuning.live(VisorConfigsTest.cameras(), tunings);
    final List<FakeBoard> boards = new ArrayList<>();
    final List<Runnable> heldReads = new ArrayList<>();
    boolean enabled;
    boolean field;
    boolean holdReads;
    String releaseSha256 = RELEASE_SHA256;
    final FakeMemory memory = new FakeMemory();
    long nanos;
    int reads;
    @Nullable IOException unreadable;
    VisorInstaller installer;

    World() {
      installer = installer();
    }

    /** A board of the scenario's, saying this; the installer is made again with it. */
    FakeBoard board(String name, Seen seen) {
      FakeBoard board = new FakeBoard(name, seen);
      boards.add(board);
      installer.close();
      installer = installer();
      return board;
    }

    /**
     * The robot program restarts: the installer is made again, with the cameras as committed (a
     * tuning is gone), and recalls what it kept. The boards run on as they were.
     */
    void restart() {
      robot(VisorConfigsTest.cameras());
    }

    /** Gives the robot these cameras instead; the installer is made again with them. */
    void robot(List<CameraConfig> committed) {
      tunings = VisorTuning.of(committed);
      cameras = VisorTuning.live(committed, tunings);
      installer.close();
      installer = installer();
    }

    private VisorInstaller installer() {
      return new VisorInstaller(
          cameras,
          cameras.committed(),
          1234,
          VisorConfigsTest.TAG_SIZE,
          VERSION,
          releaseSha256,
          () -> {
            reads++;
            IOException fault = unreadable;
            if (fault != null) {
              throw fault;
            }
            return RELEASE;
          },
          read -> {
            if (holdReads) {
              heldReads.add(read);
            } else {
              read.run();
            }
          },
          () -> enabled,
          () -> field,
          () -> nanos,
          List.copyOf(boards),
          memory);
    }

    void finishReads() {
      heldReads.forEach(Runnable::run);
      heldReads.clear();
    }

    void loops(int count) {
      for (int i = 0; i < count; i++) {
        installer.periodic();
        nanos += 20_000_000;
      }
    }

    void seconds(int count) {
      loops(count * 50);
    }

    LiveConfig<Software.Visor> tuning(int camera) {
      return Objects.requireNonNull(tunings.get(VisorConfigsTest.cameras().get(camera).name()));
    }

    /** Tunes a camera from Pitwall: whether its tuning took the values. */
    boolean tune(int camera, Map<String, Double> values) {
      return tuning(camera).set(values);
    }

    /** The configuration wanted now, as tuned, for a board with these devices. */
    String tomlNow(List<String> devices) {
      return VisorConfigs.forBoard(cameras.get(), 1234, VisorConfigsTest.TAG_SIZE, devices).toml();
    }

    String wantedNow(List<String> devices) {
      return VisorPack.configHash(tomlNow(devices));
    }

    /** The Visor alerts a dashboard shows now, but the one for cameras on no board. */
    List<String> alerts() {
      return active().stream().filter(alert -> !alert.startsWith(UNPLACED)).toList();
    }

    /** The alert for cameras on no board, while a dashboard shows it. */
    List<String> unplaced() {
      return active().stream().filter(alert -> alert.startsWith(UNPLACED)).toList();
    }

    private static List<String> active() {
      List<String> active = new ArrayList<>();
      for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
        if (alert.activeStartTime != 0 && alert.group.equals("Visor")) {
          active.add(alert.text);
        }
      }
      return active;
    }

    @Override
    public void close() {
      installer.close();
    }
  }
}

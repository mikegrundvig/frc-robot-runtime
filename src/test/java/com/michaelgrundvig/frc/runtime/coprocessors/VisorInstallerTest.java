package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.coprocessors.VisorInstaller.Report;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.util.AlertDataJNI;

/**
 * The Visor installer on boards that answer as the scenario says: the guide's robot's cameras on
 * Visor (VisorConfigsTest's), a board with two of them and a board with the back one. Each loop the
 * installer looks; each step finishes when the test says.
 */
class VisorInstallerTest {
  private static final String VERSION = "2099.1.2";
  private static final String FRONT_LEFT = "usb-Thrifty_Cam_0001-video-index0";
  private static final String FRONT_RIGHT = "usb-Thrifty_Cam_0002-video-index0";
  private static final String BACK = "usb-Thrifty_Cam_0003-video-index0";

  /** Both front cameras, which a board of the scenario has together. */
  private static final List<String> FRONTS = List.of(FRONT_LEFT, FRONT_RIGHT);

  /** Every one of the robot's Visor cameras, on one board. */
  private static final List<String> ALL = List.of(FRONT_LEFT, FRONT_RIGHT, BACK);

  private static final byte[] RELEASE = {1, 2, 3};

  private final AtomicBoolean disabled = new AtomicBoolean(true);
  private final AtomicLong nanos = new AtomicLong();
  private final List<String> reads = new ArrayList<>();
  private @Nullable IOException unreadable;
  private @Nullable VisorInstaller installer;

  /** A board's step, which finishes when the test says. */
  private static final class Pending implements VisorInstaller.Step {
    final String what;
    boolean done;
    boolean ok;
    String why = "";

    Pending(String what) {
      this.what = what;
    }

    void finish(boolean worked, String reason) {
      done = true;
      ok = worked;
      why = reason;
    }

    @Override
    public boolean done() {
      return done;
    }

    @Override
    public boolean succeeded() {
      return ok;
    }

    @Override
    public String why() {
      return why;
    }
  }

  /** A board: what it reports, and each step it was asked for. */
  private static final class FakeBoard implements VisorInstaller.Board {
    final String name;
    @Nullable Report report;
    final List<Pending> steps = new ArrayList<>();

    FakeBoard(String name, @Nullable Report report) {
      this.name = name;
      this.report = report;
    }

    @Override
    public String name() {
      return name;
    }

    @Override
    public Optional<Report> report() {
      return Optional.ofNullable(report);
    }

    @Override
    public VisorInstaller.Step install(byte[] release) {
      assertThat(release).isEqualTo(RELEASE);
      return add("install");
    }

    @Override
    public VisorInstaller.Step configure(String config) {
      return add("configure " + VisorInstaller.sha256(config));
    }

    Pending add(String what) {
      Pending step = new Pending(what);
      steps.add(step);
      return step;
    }

    Pending last() {
      return steps.get(steps.size() - 1);
    }

    List<String> asked() {
      return steps.stream().map(step -> step.what.split(" ")[0]).toList();
    }
  }

  private VisorInstaller installer(FakeBoard... boards) {
    VisorInstaller made =
        new VisorInstaller(
            VisorConfigsTest.cameras(),
            1234,
            VERSION,
            () -> {
              reads.add("read");
              if (unreadable != null) {
                throw unreadable;
              }
              return RELEASE;
            },
            disabled::get,
            nanos::get,
            List.of(boards));
    installer = made;
    return made;
  }

  @AfterEach
  void close() {
    if (installer != null) {
      installer.close();
    }
  }

  /** Loops, each 20 ms on the robot's clock. */
  private void loops(VisorInstaller visor, int count) {
    for (int i = 0; i < count; i++) {
      visor.periodic();
      nanos.addAndGet(20_000_000);
    }
  }

  private static String wanted(List<String> devices) {
    return VisorInstaller.sha256(
        VisorConfigs.forBoard(VisorConfigsTest.cameras(), 1234, devices).orElseThrow());
  }

  /** The Visor alerts a dashboard shows now. */
  private static List<String> active() {
    List<String> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.activeStartTime != 0 && alert.group.equals("Visor")) {
        active.add(alert.text);
      }
    }
    return active;
  }

  @Test
  void aFreshBoardGetsTheReleaseThenItsConfigurationAndIsLeftAloneOnceRight() {
    FakeBoard board = new FakeBoard("vision-a", new Report("", "", ALL));
    VisorInstaller visor = installer(board);

    loops(visor, 5);
    assertThat(board.asked()).as("one install, waiting").containsExactly("install");
    assertThat(active()).contains("Installing Visor " + VERSION + " on vision-a");

    board.last().finish(true, "");
    loops(visor, 5);
    assertThat(board.asked()).as("its report hasn't caught up yet").containsExactly("install");
    board.report = new Report(VERSION, "", ALL);
    loops(visor, 5);
    assertThat(board.asked()).containsExactly("install", "configure");
    assertThat(board.last().what).isEqualTo("configure " + wanted(ALL));

    board.last().finish(true, "");
    board.report = new Report(VERSION, wanted(ALL), ALL);
    loops(visor, 50);
    assertThat(board.asked()).as("right: nothing more").containsExactly("install", "configure");
    assertThat(reads).as("the release read once").hasSize(1);
    assertThat(active()).isEmpty();
  }

  @Test
  void eachBoardIsBroughtToWhatItNeedsAndOneWithoutTheRobotsCamerasIsLeftAlone() {
    FakeBoard current = new FakeBoard("vision-a", new Report(VERSION, wanted(FRONTS), FRONTS));
    FakeBoard stale = new FakeBoard("vision-b", new Report(VERSION, "old", List.of(BACK)));
    FakeBoard other =
        new FakeBoard("detector", new Report("", "", List.of("usb-Other-video-index0")));
    VisorInstaller visor = installer(current, stale, other);

    loops(visor, 5);
    assertThat(current.asked()).isEmpty();
    assertThat(stale.asked()).containsExactly("configure");
    assertThat(stale.last().what).isEqualTo("configure " + wanted(List.of(BACK)));
    assertThat(other.asked()).as("none of the robot's cameras").isEmpty();
  }

  @Test
  void nothingIsChangedWhileTheRobotIsEnabled() {
    FakeBoard board = new FakeBoard("vision-a", new Report("2000.1.1", "", ALL));
    VisorInstaller visor = installer(board);
    disabled.set(false);
    loops(visor, 50);
    assertThat(board.asked()).isEmpty();
    disabled.set(true);
    loops(visor, 1);
    assertThat(board.asked()).containsExactly("install");
  }

  @Test
  void aStepThatFailsIsSaidAndNotTriedAgainUntilWhatsWantedChanges() {
    FakeBoard board = new FakeBoard("vision-a", new Report(VERSION, "", FRONTS));
    FakeBoard back =
        new FakeBoard("vision-b", new Report(VERSION, wanted(List.of(BACK)), List.of(BACK)));
    VisorInstaller visor = installer(board, back);
    loops(visor, 2);
    board.last().finish(false, "exit isn't 0: Visor refused the configuration: no such device");
    loops(visor, 50);
    assertThat(board.asked()).containsExactly("configure");
    assertThat(active())
        .containsExactly(
            "Visor on vision-a refused its configuration: exit isn't 0: Visor refused the"
                + " configuration: no such device");

    // The back camera moved beside the fronts: another configuration, tried, and it works.
    board.report = new Report(VERSION, "", ALL);
    back.report = new Report(VERSION, wanted(List.of(BACK)), List.of());
    loops(visor, 2);
    assertThat(board.asked()).containsExactly("configure", "configure");
    board.last().finish(true, "");
    loops(visor, 1);
    assertThat(active()).isEmpty();
  }

  @Test
  void aReleaseThatCantBeReadIsSaid() {
    unreadable = new IOException("no such file: /home/systemcore/deploy/visor/visor.tar.gz");
    FakeBoard board = new FakeBoard("vision-a", new Report("", "", ALL));
    VisorInstaller visor = installer(board);
    loops(visor, 20);
    assertThat(board.asked()).isEmpty();
    assertThat(reads).as("tried once").hasSize(1);
    assertThat(active())
        .singleElement()
        .asString()
        .startsWith("Visor on vision-a didn't install: this robot's Visor release can't be read")
        .endsWith("deploy again");
  }

  @Test
  void aCameraOnNoBoardIsSaidOnceEveryBoardHasReported() {
    FakeBoard fronts = new FakeBoard("vision-a", null);
    FakeBoard back =
        new FakeBoard("vision-b", new Report(VERSION, wanted(List.of(BACK)), List.of(BACK)));
    VisorInstaller visor = installer(fronts, back);
    loops(visor, 5);
    assertThat(active()).as("vision-a hasn't reported").isEmpty();
    fronts.report = new Report(VERSION, wanted(FRONTS), FRONTS);
    loops(visor, 1);
    assertThat(active()).isEmpty();
    fronts.report = new Report(VERSION, wanted(FRONTS), List.of());
    loops(visor, 1);
    assertThat(active())
        .singleElement()
        .asString()
        .startsWith("Visor cameras plugged into no coprocessor: ")
        .contains("usb-Thrifty_Cam_0001-video-index0")
        .contains("usb-Thrifty_Cam_0002-video-index0");
  }
}

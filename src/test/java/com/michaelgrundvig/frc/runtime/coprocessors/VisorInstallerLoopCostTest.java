package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.runtime.InItsOwnJvm;
import com.michaelgrundvig.frc.visor.VisorPack;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * What the Visor installer allocates each loop once its boards are right: two boards with the
 * robot's cameras, one camera tuned live, the robot disabled off the field, so every board is
 * looked at each loop. Before it cached them, it built each board's configuration and its hash, and
 * the tuned camera's configuration, every loop. Measured as the drive's cost test is: in batches
 * after a warm-up, the cheapest batch, in a JVM of its own ({@link InItsOwnJvm}).
 */
@InItsOwnJvm
class VisorInstallerLoopCostTest {
  private static final int WARM_UP_LOOPS = 8000;
  private static final int BATCHES = 5;
  private static final int LOOPS_A_BATCH = 2000;

  /** The most a steady loop may allocate, in bytes, on average: nothing, give or take a stray. */
  private static final double MOST_BYTES_A_LOOP = 8;

  /** How often a board's report is read again: each status, every 2 s, as the pack's are. */
  private static final int LOOPS_A_REREAD = 100;

  private static final String VERSION = "2099.1.2";
  private static final String FRONT_LEFT = "usb-Thrifty_Cam_0001-video-index0";
  private static final String FRONT_RIGHT = "usb-Thrifty_Cam_0002-video-index0";
  private static final String BACK = "usb-Thrifty_Cam_0003-video-index0";

  private static final com.sun.management.ThreadMXBean THREADS =
      (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

  /**
   * How the boards' reports arrive.
   *
   * @param reread whether a board's report is read again every 2 s, as Spotter's board does
   *     whenever any of its values changes (its uptime, say): a new report that says the same
   */
  record Row(String name, boolean reread) {
    @Override
    public String toString() {
      return name;
    }
  }

  static Stream<Row> rows() {
    return Stream.of(
        new Row("each board's report the same one", false),
        new Row("a board's report read again, saying the same", true));
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("rows")
  void aSteadyLoopAllocatesNothing(Row row) {
    List<CameraConfig> committed = VisorConfigsTest.cameras();
    Map<String, LiveConfig<Software.Visor>> tunings = VisorTuning.of(committed);
    LiveConfig<Software.Visor> back = Objects.requireNonNull(tunings.get(committed.get(3).name()));
    assertThat(back.set(Map.of("gain.value", 55.0))).isTrue();
    VisorTuning.Live cameras = VisorTuning.live(committed, tunings);
    List<String> fronts = List.of(FRONT_LEFT, FRONT_RIGHT);
    List<String> backs = List.of(BACK);
    VisorInstallerTest.FakeBoard a = board("vision-a", report(cameras.get(), fronts));
    // Two reports that say the same, made before the loops so the test allocates nothing itself.
    VisorInstaller.Seen backReport = report(cameras.get(), backs);
    VisorInstaller.Seen backReportAgain = report(cameras.get(), new ArrayList<>(backs));
    VisorInstallerTest.FakeBoard b = board("vision-b", backReport);
    long[] now = {0};
    long thread = Thread.currentThread().threadId();
    double least = Double.POSITIVE_INFINITY;
    try (VisorInstaller installer =
        new VisorInstaller(
            cameras,
            committed,
            1234,
            VisorConfigsTest.TAG_SIZE,
            VERSION,
            VisorPack.sha256(new byte[0]),
            () -> new byte[0],
            Runnable::run,
            () -> false,
            () -> false,
            () -> now[0],
            List.of(a, b),
            VisorMemory.NONE)) {
      long bytes = 0;
      int measured = 0;
      int batches = 0;
      for (int loop = 0; batches < BATCHES; loop++) {
        now[0] += 20_000_000L;
        if (row.reread() && loop % LOOPS_A_REREAD == 0) {
          b.says(loop % (2 * LOOPS_A_REREAD) == 0 ? backReportAgain : backReport);
        }
        long before = THREADS.getThreadAllocatedBytes(thread);
        installer.periodic();
        long after = THREADS.getThreadAllocatedBytes(thread);
        if (loop < WARM_UP_LOOPS) {
          continue;
        }
        bytes += after - before;
        measured++;
        if (measured == LOOPS_A_BATCH) {
          least = Math.min(least, (double) bytes / LOOPS_A_BATCH);
          bytes = 0;
          measured = 0;
          batches++;
        }
      }
    }
    assertThat(a.asked()).as("right: nothing asked").isEmpty();
    assertThat(b.asked()).as("right: nothing asked").isEmpty();
    System.out.printf("Visor installer, two boards right, %s: %.1f bytes/loop%n", row, least);
    assertThat(least).isLessThan(MOST_BYTES_A_LOOP);
  }

  private static VisorInstallerTest.FakeBoard board(String name, VisorInstaller.Seen seen) {
    return new VisorInstallerTest.FakeBoard(name, seen);
  }

  /** A board's report with these cameras, running the configuration wanted for them. */
  private static VisorInstaller.Seen report(List<CameraConfig> cameras, List<String> devices) {
    String config = VisorConfigs.forBoard(cameras, 1234, VisorConfigsTest.TAG_SIZE, devices).hash();
    return new VisorInstaller.Seen.Reported(
        new VisorPack.Report(VERSION, config, VERSION, config, devices));
  }
}

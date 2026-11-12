package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.runtime.InItsOwnJvm;
import com.michaelgrundvig.frc.visor.CameraToTag;
import com.michaelgrundvig.frc.visor.VisorFrame;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * What reading four cameras on Visor costs each robot loop, each at 60 frames a second with two
 * tags in view: the bytes allocated and the time taken by each camera's read (its frames sorted out
 * of the receiver, checked, made into its inputs, and its health judged) and the logging of its
 * loop. Frames are offered between loops, as the receiver's thread would take them, and that isn't
 * measured. The numbers are printed, and written to {@code build/visor-loop-cost.txt}.
 *
 * <p>As in the drive's cost test, the loops are measured in batches after a warm-up, the cheapest
 * batch is reported, and only the allocation is asserted, held to a bound a little above what it
 * measured when the bound was set. It runs in a JVM of its own ({@link InItsOwnJvm}), since what
 * the JIT makes of the loop depends on what ran before.
 *
 * <p>Nearly all of what's allocated is the inputs' own records, which hold what a log can replay:
 * for each frame its {@code Frame}, and for each tag its {@code SeenTag} with its two candidate
 * poses, each a {@code Transform3d} of a translation and a rotation.
 */
@InItsOwnJvm
class VisorLoopCostTest {
  private static final int CAMERAS = 4;
  private static final long LOOP_NANOS = 20_000_000L;
  private static final double FRAMES_A_LOOP = 60 * 0.020;

  private static final int WARM_UP_LOOPS = 8000;
  private static final int BATCHES = 5;
  private static final int LOOPS_A_BATCH = 2000;

  /**
   * The most the loop may allocate, in bytes, on average. It measured 5,504 bytes a loop when the
   * bound was set, against 6,688 before the frames were checked (each frame's detection time boxed,
   * and the inputs' arrays made again every loop).
   */
  private static final double MOST_BYTES_A_LOOP = 6_200;

  private static final com.sun.management.ThreadMXBean THREADS =
      (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

  @BeforeEach
  void setUp() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
  }

  @AfterEach
  void tearDown() {
    TunableRegistry.reset();
  }

  private static CameraConfig camera(String name) {
    return CameraConfig.of(
            name,
            CameraConfig.ImageSensor.of(1600, 1304, Hertz.of(60))
                .fieldOfView(Degrees.of(80), CameraConfig.Across.DIAGONAL)
                .latency(Milliseconds.of(30))
                .sightRange(Meters.of(7)))
        .at(Inches.of(12), Inches.of(0), Inches.of(12))
        .facing(Degrees.of(15), Degrees.of(0))
        .roll(Degrees.of(0))
        .noiseScale(1.0)
        .measured()
        .software(CameraConfig.Software.VISOR);
  }

  @Test
  void readingFourVisorCamerasAllocatesNoMoreThanItDid() throws IOException {
    List<CameraConfig> configs = new ArrayList<>();
    for (int i = 0; i < CAMERAS; i++) {
      configs.add(camera("camera" + i));
    }
    long[] now = {10_000_000_000L};
    VisorFeed feed = VisorFeed.inMemory(configs);
    List<Camera> cameras = new ArrayList<>();
    for (int i = 0; i < CAMERAS; i++) {
      cameras.add(new Camera(configs.get(i), new CameraIOVisor(feed, i, () -> now[0]), 500));
    }
    VisorFrame frame = new VisorFrame();
    CameraToTag ahead = new CameraToTag();
    ahead.set(3, 0.5, 0.25, 0, 0, 0, 1);
    CameraToTag other = new CameraToTag();
    other.set(3, 0.5, 0.25, 0.1, 0, 0, 0.995);
    long[] sequence = new long[CAMERAS];
    double owed = 0;
    long thread = Thread.currentThread().threadId();
    long bytes = 0;
    long nanos = 0;
    int measured = 0;
    double leastBytes = Double.POSITIVE_INFINITY;
    double leastNanos = Double.POSITIVE_INFINITY;
    int batches = 0;
    try (feed) {
      for (int loop = 0; batches < BATCHES; loop++) {
        now[0] += LOOP_NANOS;
        owed += FRAMES_A_LOOP;
        int made = (int) owed;
        owed -= made;
        for (int camera = 0; camera < CAMERAS; camera++) {
          for (int f = 0; f < made; f++) {
            frame.setHeader(
                sequence[camera]++, 1600, 1304, now[0] - 25_000_000L, camera, true, false);
            frame.setHealth(4_000, 0);
            for (int id : new int[] {25, 26}) {
              int tag = frame.addTag(id, 0, 80, new float[] {10, 20, 30, 20, 30, 40, 10, 40});
              frame.setPoses(tag, ahead, 0.1f, other, 0.5f);
            }
            feed.offer(frame);
          }
        }

        long bytesBefore = THREADS.getThreadAllocatedBytes(thread);
        long nanosBefore = System.nanoTime();
        for (Camera camera : cameras) {
          camera.read(now[0]);
          camera.logLoop(now[0]);
        }
        long nanosAfter = System.nanoTime();
        long bytesAfter = THREADS.getThreadAllocatedBytes(thread);

        if (loop < WARM_UP_LOOPS) {
          continue;
        }
        bytes += bytesAfter - bytesBefore;
        nanos += nanosAfter - nanosBefore;
        measured++;
        if (measured == LOOPS_A_BATCH) {
          leastBytes = Math.min(leastBytes, (double) bytes / LOOPS_A_BATCH);
          leastNanos = Math.min(leastNanos, (double) nanos / LOOPS_A_BATCH);
          bytes = 0;
          nanos = 0;
          measured = 0;
          batches++;
        }
      }
    } finally {
      cameras.forEach(Camera::close);
    }

    String report =
        String.format(
            Locale.ROOT,
            "%-44s %8.1f bytes/loop %8.0f ns/loop%n",
            "four Visor cameras at 60 fps, two tags",
            leastBytes,
            leastNanos);
    Files.createDirectories(Path.of("build"));
    Files.writeString(Path.of("build", "visor-loop-cost.txt"), report);
    System.out.print(report);
    assertThat(leastBytes).isLessThan(MOST_BYTES_A_LOOP);
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.visor.CameraToTag;
import com.michaelgrundvig.frc.visor.VisorFrame;
import com.michaelgrundvig.frc.visor.VisorWire;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;

/** A camera on Visor, read from frames as its coprocessor would send them. */
class CameraIOVisorTest {
  private static final long SECOND = 1_000_000_000L;

  /** A tenth of a second before now: a frame just taken. */
  private static final long RECENT = 10 * SECOND - SECOND / 10;

  /** The robot's cameras: two on Visor, both 1600 by 1304. */
  private static final List<CameraConfig> CAMERAS = List.of(camera("front"), camera("back"));

  private final VisorFeed feed = VisorFeed.inMemory(CAMERAS);
  private final AtomicLong now = new AtomicLong(10 * SECOND);
  private final CameraIOVisor front = new CameraIOVisor(feed, 0, now::get);
  private final CameraIOVisor back = new CameraIOVisor(feed, 1, now::get);

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

  /** A frame from a camera, captured at a time, with a tag solved 3 m ahead and one unsolved. */
  private static VisorFrame frame(int camera, long sequence, long capturedNanos, boolean synced) {
    VisorFrame frame = new VisorFrame();
    frame.setHeader(sequence, 1600, 1304, capturedNanos, camera, synced, false);
    // Corners in Visor's order: top left, top right, bottom right, bottom left.
    int solved = frame.addTag(7, 0, 80, new float[] {10, 20, 30, 20, 30, 40, 10, 40});
    CameraToTag ahead = new CameraToTag();
    ahead.set(3, 0.5, 0.25, 0, 0, 0, 1);
    CameraToTag other = new CameraToTag();
    other.set(3, 0.5, 0.25, 0.1, 0, 0, 0.995);
    frame.setPoses(solved, ahead, 0.1f, other, 0.5f);
    frame.addTag(8, 0, 80, new float[8]);
    return frame;
  }

  private static CameraIO.CameraIOInputs read(CameraIOVisor camera) {
    CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
    camera.updateInputs(inputs);
    return inputs;
  }

  @Test
  void eachCameraGetsItsOwnFramesWithTheirSolvedTags() {
    feed.offer(frame(0, 0, RECENT, true));
    feed.offer(frame(1, 0, RECENT + 1, true));
    feed.offer(frame(0, 1, RECENT + 2, true));

    CameraIO.CameraIOInputs inputs = read(front);
    assertThat(inputs.frames)
        .extracting(CameraIO.Frame::timestampNanos)
        .containsExactly(RECENT, RECENT + 2);
    assertThat(inputs.frames).allSatisfy(frame -> assertThat(frame.hasMultiTag()).isFalse());
    // The unsolved tag has no pose to use: it goes on by its number alone, as unsolved, so the
    // pipeline can say why its image placed nothing (no lens calibration, say).
    assertThat(inputs.frames).extracting(CameraIO.Frame::tagCount).containsExactly(1, 1);
    assertThat(inputs.frames).extracting(CameraIO.Frame::unsolvedTagCount).containsExactly(1, 1);
    assertThat(inputs.unsolvedTags).containsExactly(8, 8);
    CameraIO.SeenTag tag = inputs.tags[0];
    assertThat(tag.id()).isEqualTo(7);
    // PhotonVision's order, as the rest of the robot code reads corners: Visor's reversed.
    assertThat(new double[] {tag.corner0X(), tag.corner0Y(), tag.corner3X(), tag.corner3Y()})
        .containsExactly(10, 40, 10, 20);
    assertThat(tag.bestCameraToTag().getX()).isCloseTo(3, within(1e-6));
    assertThat(tag.bestCameraToTag().getY()).isCloseTo(0.5, within(1e-6));
    assertThat(tag.ambiguity()).isCloseTo(0.2, within(1e-6));
    assertThat(inputs.connected).isTrue();
    assertThat(inputs.error).isEmpty();

    assertThat(read(back).frames).hasSize(1);
    assertThat(read(front).frames).as("taken already").isEmpty();
  }

  /**
   * Each camera's latest frame says what it was taken at and how bright it is, and the inputs keep
   * it until another frame says, so the log (Vision/<camera>/ExposureMs and the rest) holds it
   * every loop. Before any frame, and from a camera choosing its own exposure, what isn't known is
   * -1.
   */
  @Test
  void theLatestFramesExposureGainAndBrightnessAreTheCamerasInputs() {
    record Read(double exposureMs, int gain, boolean auto, int median, int high, double clipped) {}
    CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
    java.util.function.Supplier<Read> read =
        () -> {
          front.updateInputs(inputs);
          return new Read(
              inputs.exposureMs,
              inputs.gain,
              inputs.autoExposure,
              inputs.brightnessMedian,
              inputs.brightnessHigh,
              inputs.clippedShare);
        };
    assertThat(read.get()).as("no frame yet").isEqualTo(new Read(-1, -1, false, -1, -1, -1));

    VisorFrame fixed = frame(0, 1, RECENT, true);
    fixed.setExposure(1300);
    fixed.setGain(40);
    fixed.setBrightness(37, 201, 0.0125);
    feed.offer(frame(0, 0, RECENT, true)); // older, overtaken by the next
    feed.offer(fixed);
    assertThat(read.get()).isEqualTo(new Read(1.3, 40, false, 37, 201, 0.0125));
    assertThat(read.get()).as("kept, with no frame since").isEqualTo(read.get());

    // The camera's own automatic exposure, on a board whose clock isn't synced yet: the frame is
    // left out, its settings aren't.
    VisorFrame automatic = frame(0, 2, RECENT, false);
    automatic.setAutoExposure();
    automatic.setBrightness(90, 255, 0.2);
    feed.offer(automatic);
    assertThat(read.get()).isEqualTo(new Read(-1, -1, true, 90, 255, 0.2));
  }

  /**
   * Each frame's detection time, and the newest frame's number and Visor's dropped count, are the
   * camera's inputs: its health is worked out from them (Vision/<camera>/Health), in a replay too.
   * A frame left out for its clock still counts.
   */
  @Test
  void eachFramesDetectTimeAndTheNewestsNumberAndDropsAreTheCamerasInputs() {
    CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
    front.updateInputs(inputs);
    assertThat(inputs.sequence).as("no frame yet").isEqualTo(-1);
    assertThat(inputs.dropped).isEqualTo(-1);
    assertThat(inputs.detectMs).isEmpty();

    VisorFrame first = frame(0, 41, RECENT, true);
    first.setHealth(4_200, 7);
    VisorFrame unsynced = frame(0, 43, RECENT, false);
    unsynced.setHealth(19_500, 4_294_967_295L);
    feed.offer(first);
    feed.offer(unsynced);
    front.updateInputs(inputs);
    assertThat(inputs.detectMs).containsExactly(4.2, 19.5);
    assertThat(inputs.sequence).isEqualTo(43);
    assertThat(inputs.dropped).isEqualTo(4_294_967_295L);

    front.updateInputs(inputs);
    assertThat(inputs.detectMs).as("none arrived").isEmpty();
    assertThat(inputs.sequence).as("kept until a frame says").isEqualTo(43);
  }

  @Test
  void framesFromAClockNotYetSyncedAreCountedAndLeftOut() {
    feed.offer(frame(0, 0, RECENT, false));
    feed.offer(frame(0, 1, RECENT, true));
    CameraIO.CameraIOInputs inputs = read(front);
    assertThat(inputs.frames).hasSize(1);
    assertThat(inputs.skippedFrames).isEqualTo(1);

    // Before chrony first syncs, the stamps are a day off, or more: still a camera sending.
    now.addAndGet(5 * SECOND);
    feed.offer(frame(0, 2, -86_400 * SECOND, false));
    inputs = read(front);
    assertThat(inputs.frames).isEmpty();
    assertThat(inputs.skippedFrames).isEqualTo(1);
    assertThat(inputs.connected).as("sending, unsynced").isTrue();
  }

  @Test
  void aCameraThatStopsSendingIsDisconnected() {
    assertThat(read(front).connected).as("before any frame").isFalse();
    feed.offer(frame(0, 0, RECENT, true));
    assertThat(read(front).connected).isTrue();
    now.addAndGet(2 * SECOND);
    assertThat(read(front).connected).as("nothing for two seconds").isFalse();
  }

  /**
   * Where the robot's boards are, as a row has them: listed, the camera reported on one, and the
   * configuration each board was given, by its ID, once that's known.
   */
  private record Boards(List<String> listed, Optional<String> placed, Optional<Integer> given)
      implements VisorBoards {
    Boards(List<String> listed, Optional<String> placed) {
      this(listed, placed, Optional.empty());
    }

    @Override
    public Optional<String> placed(int camera) {
      return camera == 0 ? placed : Optional.empty();
    }

    @Override
    public boolean otherConfig(String board, int configId) {
      return given.isPresent() && given.get() != configId;
    }
  }

  /** A sender check: the boards, a frame from 127.0.0.1, and what became of it. */
  private record Sent(
      String name,
      Boards boards,
      VisorSenders.Resolver names,
      Optional<VisorFeed.Refusal> refused,
      String visorAddress,
      int unplaced,
      int unfound) {}

  /**
   * A frame's sender is checked only against what the robot knows: refused only from where its
   * camera is known not to be, and taken wherever the robot doesn't know. Its live view is on the
   * board it's reported on, or else the listed board its frames come from, never a stranger. Each
   * row's frame comes from 127.0.0.1.
   */
  @Test
  void aFramesSenderIsRefusedOnlyWhereItsKnownToBeWrong() throws Exception {
    Optional<String> nowhere = Optional.empty();
    VisorSenders.Resolver neverFound = name -> Optional.empty();
    InetAddress here = InetAddress.ofLiteral("127.0.0.1");
    InetAddress elsewhere = InetAddress.ofLiteral("127.0.0.2");
    List<Sent> rows =
        List.of(
            new Sent(
                "no boards listed: from anyone",
                new Boards(List.of(), nowhere),
                neverFound,
                Optional.empty(),
                "127.0.0.1",
                0,
                0),
            new Sent(
                "boards listed, no report yet: from a listed board",
                new Boards(List.of("127.0.0.2", "127.0.0.1"), nowhere),
                neverFound,
                Optional.empty(),
                "127.0.0.1",
                1,
                0),
            new Sent(
                "boards listed, no report yet: not from one that isn't",
                new Boards(List.of("127.0.0.2"), nowhere),
                neverFound,
                Optional.of(VisorFeed.Refusal.NOT_A_LISTED_BOARD),
                "",
                1,
                0),
            new Sent(
                "reported on another listed board: refused",
                new Boards(List.of("127.0.0.1", "127.0.0.2"), Optional.of("127.0.0.2")),
                neverFound,
                Optional.of(VisorFeed.Refusal.NOT_FROM_ITS_BOARD),
                "127.0.0.2",
                0,
                0),
            new Sent(
                "reported on its board: taken",
                new Boards(List.of("127.0.0.1", "127.0.0.2"), Optional.of("127.0.0.1")),
                neverFound,
                Optional.empty(),
                "127.0.0.1",
                0,
                0),
            new Sent(
                "a board listed by a name not yet found: taken, its address not a stranger's",
                new Boards(List.of("visor-left.local"), nowhere),
                neverFound,
                Optional.empty(),
                "",
                1,
                1),
            new Sent(
                "a board listed by a name found: taken, its live view by that name",
                new Boards(List.of("visor-left.local"), nowhere),
                name -> Optional.of(here),
                Optional.empty(),
                "visor-left.local",
                1,
                0),
            new Sent(
                "a name found to be another address: refused",
                new Boards(List.of("visor-left.local"), nowhere),
                name -> Optional.of(elsewhere),
                Optional.of(VisorFeed.Refusal.NOT_A_LISTED_BOARD),
                "",
                1,
                0));
    for (Sent row : rows) {
      try (VisorFeed board = VisorFeed.listen(0, CAMERAS, row.boards(), row.names());
          DatagramChannel visor = DatagramChannel.open()) {
        CameraIOVisor camera = new CameraIOVisor(board, 0, now::get);
        // A name is looked up off the loop: the frame is sent once it has been, or not found.
        for (int tries = 0; board.boardsUnfound() != row.unfound() && tries < 500; tries++) {
          Thread.sleep(10);
        }
        send(visor, board, frame(0, 0, RECENT, true));
        CameraIO.CameraIOInputs inputs = read(camera);
        for (int tries = 0;
            inputs.frames.length == 0 && !anyRefused(board) && tries < 500;
            tries++) {
          Thread.sleep(10);
          inputs = read(camera);
        }
        String name = row.name();
        if (row.refused().isPresent()) {
          assertThat(inputs.frames).as(name).isEmpty();
          assertThat(board.refused(row.refused().get())).as(name).isEqualTo(1);
        } else {
          assertThat(inputs.frames).as(name).hasSize(1);
          assertThat(anyRefused(board)).as(name).isFalse();
          assertThat(inputs.error).as(name + ": nothing raised for where it is").isEmpty();
        }
        assertThat(inputs.visorAddress).as(name).isEqualTo(row.visorAddress());
        assertThat(board.unplaced()).as(name).isEqualTo(row.unplaced());
        assertThat(board.boardsUnfound()).as(name).isEqualTo(row.unfound());
      }
    }
  }

  /** A frame's configuration, as a row has it: the board's, and what became of the frame. */
  private record Configured(
      String name, Optional<Integer> given, int sent, Optional<VisorFeed.Refusal> refused) {}

  /**
   * A frame from a configuration the robot didn't give its board (a stale one: cameras reordered, a
   * lens calibrated again) is refused, counted and said on its camera; one the robot gave is taken,
   * and so is any while the robot doesn't know which it gave. Each row's frame comes from
   * 127.0.0.1, the board its camera is reported on.
   */
  @Test
  void aFrameFromAStaleConfigurationIsRefused() throws Exception {
    int given = 0x6a5d27df;
    int stale = 0xa6738b08;
    List<Configured> rows =
        List.of(
            new Configured("not known yet: taken", Optional.empty(), stale, Optional.empty()),
            new Configured("the one given: taken", Optional.of(given), given, Optional.empty()),
            new Configured(
                "another: refused",
                Optional.of(given),
                stale,
                Optional.of(VisorFeed.Refusal.OTHER_CONFIG)));
    for (Configured row : rows) {
      Boards boards = new Boards(List.of("127.0.0.1"), Optional.of("127.0.0.1"), row.given());
      try (VisorFeed board = VisorFeed.listen(0, CAMERAS, boards, name -> Optional.empty());
          DatagramChannel visor = DatagramChannel.open()) {
        CameraIOVisor camera = new CameraIOVisor(board, 0, now::get);
        VisorFrame frame = frame(0, 0, RECENT, true);
        frame.setConfigId(row.sent());
        send(visor, board, frame);
        CameraIO.CameraIOInputs inputs = read(camera);
        for (int tries = 0;
            inputs.frames.length == 0 && !anyRefused(board) && tries < 500;
            tries++) {
          Thread.sleep(10);
          inputs = read(camera);
        }
        String name = row.name();
        if (row.refused().isPresent()) {
          assertThat(inputs.frames).as(name).isEmpty();
          assertThat(board.refused(row.refused().get())).as(name).isEqualTo(1);
          assertThat(inputs.error)
              .as(name)
              .contains("from a configuration the robot didn't give its board");
        } else {
          assertThat(inputs.frames).as(name).hasSize(1);
          assertThat(anyRefused(board)).as(name).isFalse();
          assertThat(inputs.error).as(name).isEmpty();
        }
      }
    }
  }

  /** A frame that was refused, for the camera, is said on it while none is taken. */
  @Test
  void aCameraWhoseFramesAreAllRefusedSaysWhy() throws Exception {
    Boards elsewhere = new Boards(List.of("127.0.0.2"), Optional.empty());
    try (VisorFeed board = VisorFeed.listen(0, CAMERAS, elsewhere, name -> Optional.empty());
        DatagramChannel visor = DatagramChannel.open()) {
      CameraIOVisor camera = new CameraIOVisor(board, 0, now::get);
      send(visor, board, frame(0, 0, RECENT, true));
      CameraIO.CameraIOInputs inputs = read(camera);
      for (int tries = 0; !anyRefused(board) && tries < 500; tries++) {
        Thread.sleep(10);
        inputs = read(camera);
      }
      assertThat(inputs.error).contains("none of the robot's coprocessors has");
      assertThat(inputs.connected).as("its frames aren't believed").isFalse();
    }
  }

  private static boolean anyRefused(VisorFeed feed) {
    for (VisorFeed.Refusal refusal : VisorFeed.Refusal.values()) {
      if (feed.refused(refusal) > 0) {
        return true;
      }
    }
    return false;
  }

  private static void send(DatagramChannel visor, VisorFeed to, VisorFrame frame)
      throws java.io.IOException {
    ByteBuffer bytes = ByteBuffer.allocate(VisorWire.MAX_BYTES);
    VisorWire.encode(frame, bytes);
    bytes.flip();
    visor.send(bytes, new InetSocketAddress("127.0.0.1", to.port()));
  }

  /** What became of frames offered: what the front camera read, and what was refused. */
  private record Refused(
      String name,
      List<VisorFrame> offered,
      List<Long> framesRead,
      long sequence,
      Map<VisorFeed.Refusal, Long> refused,
      String error) {}

  /**
   * Each kind of frame that isn't believed is left out and counted by why; one that arrived late is
   * believed, and only a reason the team must fix is said on the camera.
   */
  @Test
  void framesThatCantBeBelievedAreCountedByWhy() {
    List<Refused> rows =
        List.of(
            new Refused(
                "an index no camera reads",
                List.of(frame(5, 0, RECENT, true)),
                List.of(),
                -1,
                Map.of(VisorFeed.Refusal.NO_READER, 1L),
                ""),
            new Refused(
                "another size than its camera's",
                List.of(sized(frame(0, 0, RECENT, true), 640, 480)),
                List.of(),
                -1,
                Map.of(VisorFeed.Refusal.OTHER_SIZE, 1L),
                "another size than its configured 1600x1304"),
            new Refused(
                "the same frame twice",
                List.of(frame(0, 3, RECENT, true), frame(0, 3, RECENT, true)),
                List.of(RECENT),
                3,
                Map.of(VisorFeed.Refusal.SEEN_BEFORE, 1L),
                ""),
            new Refused(
                "a pair out of order: both believed, the higher number the newest",
                List.of(frame(0, 5, RECENT + 2, true), frame(0, 4, RECENT + 1, true)),
                List.of(RECENT + 2, RECENT + 1),
                5,
                Map.of(),
                ""));
    for (Refused row : rows) {
      try (VisorFeed fresh = VisorFeed.inMemory(CAMERAS)) {
        CameraIOVisor camera = new CameraIOVisor(fresh, 0, now::get);
        row.offered().forEach(fresh::offer);
        CameraIO.CameraIOInputs inputs = read(camera);
        assertThat(inputs.frames)
            .as(row.name())
            .extracting(CameraIO.Frame::timestampNanos)
            .containsExactlyElementsOf(row.framesRead());
        assertThat(inputs.sequence).as(row.name()).isEqualTo(row.sequence());
        for (VisorFeed.Refusal refusal : VisorFeed.Refusal.values()) {
          assertThat(fresh.refused(refusal))
              .as(row.name() + ": " + refusal)
              .isEqualTo(row.refused().getOrDefault(refusal, 0L));
        }
        if (row.error().isEmpty()) {
          assertThat(inputs.error).as(row.name()).isEmpty();
        } else {
          assertThat(inputs.error).as(row.name()).contains(row.error());
        }
      }
    }
  }

  /**
   * Frames for an index no camera reads aren't kept at all, and those for a camera that isn't read
   * wait only up to one ring's worth: neither grows the robot's memory, whatever arrives.
   */
  @Test
  void whatWaitsForACameraIsBounded() {
    for (int i = 0; i < 10_000; i++) {
      feed.offer(frame(7, i, RECENT, true));
      feed.offer(frame(1, i, RECENT, true));
      read(front);
    }
    assertThat(feed.waiting(7)).as("an index nobody reads").isZero();
    assertThat(feed.refused(VisorFeed.Refusal.NO_READER)).isEqualTo(10_000);
    assertThat(feed.waiting(1)).as("a camera not read").isEqualTo(VisorFeed.WAITING_FRAMES);
    assertThat(feed.refused(VisorFeed.Refusal.WAITING_FULL))
        .isEqualTo(10_000 - VisorFeed.WAITING_FRAMES);
    assertThat(read(back).frames).hasSize(VisorFeed.WAITING_FRAMES);
  }

  /**
   * A coprocessor on another Visor version is said on every camera on Visor, since its frames can't
   * say which camera they are, and stays said while its frames keep arriving: until 2 s after the
   * last.
   */
  @Test
  void aCoprocessorOnAnotherVersionIsSaidOnEveryCameraWhileItSends() {
    VisorFrame stale = frame(0, 0, RECENT, true);
    stale.setVersionId(20251225);
    feed.offer(stale);
    CameraIO.CameraIOInputs inputs = read(front);
    assertThat(inputs.frames).isEmpty();
    assertThat(inputs.error).contains("20251225").contains("camera 0").contains("deploy");
    assertThat(read(back).error).as("on every camera").contains("20251225");

    now.addAndGet(SECOND);
    assertThat(read(front).error).as("1 s after the last").contains("20251225");
    feed.offer(stale);
    read(front);
    now.addAndGet(3 * SECOND / 2);
    assertThat(read(back).error).as("1.5 s after the last, 2.5 s after the first").isNotEmpty();
    now.addAndGet(SECOND);
    assertThat(read(front).error).as("2.5 s after the last").isEmpty();
  }

  /** A frame as Visor would send it, at another image size. */
  private static VisorFrame sized(VisorFrame frame, int width, int height) {
    VisorFrame resized = new VisorFrame();
    resized.setHeader(
        frame.sequence(), width, height, frame.captureNanos(), frame.camera(), true, false);
    return resized;
  }
}

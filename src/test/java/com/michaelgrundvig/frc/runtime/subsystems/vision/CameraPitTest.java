package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth;
import com.michaelgrundvig.frc.visor.VisorFrame;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalLong;
import java.util.function.BiConsumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * A camera's card on Pitwall, after a minute of its frames: a 30 fps camera on Visor read by a 50
 * Hz loop, healthy, dropping frames, losing them, slow to detect, restarted, gone quiet, and one on
 * PhotonVision.
 */
class CameraPitTest {
  private static final long SECOND = 1_000_000_000L;

  private static CameraConfig camera(String name, CameraConfig.Software software) {
    return CameraConfig.of(
            name,
            CameraConfig.ImageSensor.of(1600, 1304, Hertz.of(30))
                .fieldOfView(Degrees.of(80), CameraConfig.Across.DIAGONAL)
                .latency(Milliseconds.of(30))
                .sightRange(Meters.of(7)))
        .at(Inches.of(12), Inches.of(0), Inches.of(12))
        .facing(Degrees.of(15), Degrees.of(0))
        .roll(Degrees.of(0))
        .noiseScale(1.0)
        .measured()
        .software(software);
  }

  /** A camera's frames as its IO hands them over, as Visor would number and count them. */
  private static final class Scripted implements CameraIO {
    boolean connected = true;
    boolean numbered = true;
    double detectMs = 5;
    int dropNext;
    int loseNext;
    long sequence = -1;
    long dropped;
    double owed;
    String address = "10.12.34.11";

    @Override
    public void updateInputs(CameraIOInputs inputs) {
      owed += connected ? 0.6 : 0; // 30 fps at 50 Hz
      int made = (int) owed;
      owed -= made;
      int dropping = Math.min(made, dropNext);
      dropNext -= dropping;
      dropped += dropping;
      int sent = made - dropping;
      sequence += sent;
      int arrived = Math.max(0, sent - loseNext);
      loseNext -= sent - arrived;
      inputs.connected = connected;
      inputs.frames = new Frame[arrived];
      Arrays.fill(inputs.frames, new Frame(0, 0, false, Transform3d.ZERO, 0));
      inputs.tags = new SeenTag[] {};
      inputs.skippedFrames = 0;
      inputs.detectMs = new double[] {};
      if (!numbered) {
        return;
      }
      inputs.detectMs = new double[arrived];
      Arrays.fill(inputs.detectMs, detectMs);
      if (arrived > 0) {
        inputs.visorAddress = address;
        inputs.sequence = sequence;
        inputs.dropped = dropped;
        inputs.exposureMs = 1;
        inputs.gain = 40;
        inputs.brightnessMedian = 41;
        inputs.brightnessHigh = 187;
      }
    }
  }

  /**
   * What happens to the camera from 20 s on, and from 30 s, and the card at 60 s.
   *
   * @param lines each of its lines, by label, with its level, in the card's order
   * @param seen what its health saw over the last 10 s, which the lines' values report
   * @param says what some of its lines say, label and the start of its value: the wording
   */
  private record Scenario(
      String name,
      CameraConfig.Software software,
      BiConsumer<Scripted, Integer> atLoop,
      String state,
      Devices.Level level,
      List<Line> lines,
      Seen seen,
      List<String> says) {}

  /** A line of the card: its label, and its level. */
  private record Line(String label, Devices.Level level) {}

  /**
   * What the camera's health saw over the last 10 s.
   *
   * @param fps frames that arrived per second, to a tenth, as the card shows it
   * @param dropped frames the vision program dropped; empty when the camera doesn't say
   * @param lost frames lost on the way; empty when the camera doesn't say
   * @param detectMeanMs how long detection took on average; NaN with no frames
   * @param restarts times the vision program restarted this session
   */
  private record Seen(
      double fps, OptionalLong dropped, OptionalLong lost, double detectMeanMs, int restarts) {
    static Seen of(CameraHealth health) {
      double fps = Math.round(health.framesPerSecond() * 10) / 10.0;
      if (health.numbers() == CameraHealth.Numbers.NOT_REPORTED) {
        return new Seen(
            fps,
            OptionalLong.empty(),
            OptionalLong.empty(),
            health.detectMeanMs(),
            health.restarts());
      }
      return new Seen(
          fps,
          OptionalLong.of(health.droppedLately()),
          OptionalLong.of(health.lostLately()),
          health.detectMeanMs(),
          health.restarts());
    }

    /** On Visor, which counts what it drops and numbers its frames. */
    static Seen onVisor(double fps, long dropped, long lost, double detectMeanMs, int restarts) {
      return new Seen(fps, OptionalLong.of(dropped), OptionalLong.of(lost), detectMeanMs, restarts);
    }
  }

  /** A camera on Visor's lines, with the levels of the three it warns on. */
  private static List<Line> onVisor(
      Devices.Level dropped, Devices.Level lost, Devices.Level detection) {
    return List.of(
        new Line("Frames", Devices.Level.NOTE),
        new Line("Images", Devices.Level.NOTE),
        new Line("Dropped at Visor", dropped),
        new Line("Lost on the way", lost),
        new Line("Detection", detection),
        new Line("Exposure", Devices.Level.NOTE));
  }

  @BeforeEach
  void setUp() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
  }

  @AfterEach
  void tearDown() {
    TunableRegistry.reset();
  }

  private static List<Scenario> scenarios() {
    CameraConfig.Software visor = CameraConfig.Software.VISOR;
    Devices.Level ok = Devices.Level.OK;
    Devices.Level warning = Devices.Level.WARNING;
    Devices.Level problem = Devices.Level.PROBLEM;
    Devices.Level note = Devices.Level.NOTE;
    return List.of(
        new Scenario(
            "healthy",
            visor,
            (io, loop) -> {},
            "Sending",
            ok,
            onVisor(ok, ok, ok),
            Seen.onVisor(30.0, 0, 0, 5.0, 0),
            List.of(
                "Frames: 30.0 fps lately (configured 30)",
                "Dropped at Visor: 0 in the last 10 s",
                "Lost on the way: 0 in the last 10 s",
                "Detection: 5.0 ms mean, 5.0 ms worst, of a 33.3 ms frame",
                "Exposure: 1.00 ms, gain 40; brightness 41 median, 187 at 99%")),
        new Scenario(
            "dropping frames now",
            visor,
            (io, loop) -> {
              if (loop == 2750) {
                io.dropNext = 2;
              }
            },
            "Sending",
            warning,
            onVisor(warning, ok, ok),
            Seen.onVisor(29.8, 2, 0, 5.0, 0),
            List.of("Dropped at Visor: 2 in the last 10 s; for ")),
        new Scenario(
            "dropped frames at 20 s, none since",
            visor,
            (io, loop) -> {
              if (loop == 1000) {
                io.dropNext = 2;
              }
            },
            "Sending",
            warning,
            onVisor(warning, ok, ok),
            Seen.onVisor(30.0, 0, 0, 5.0, 0),
            List.of("Dropped at Visor: 0 in the last 10 s; once this session, for ")),
        new Scenario(
            "losing frames on the network",
            visor,
            (io, loop) -> {
              if (loop % 250 == 0) {
                io.loseNext = 1;
              }
            },
            "Sending",
            warning,
            onVisor(ok, warning, ok),
            Seen.onVisor(29.8, 0, 1, 5.0, 0),
            List.of("Lost on the way: 1 in the last 10 s; ")),
        new Scenario(
            "detection slower than the frame",
            visor,
            (io, loop) -> io.detectMs = 40,
            "Sending",
            warning,
            onVisor(ok, ok, warning),
            Seen.onVisor(30.0, 0, 0, 40.0, 0),
            List.of("Detection: 40.0 ms mean, 40.0 ms worst, of a 33.3 ms frame; for ")),
        new Scenario(
            "Visor restarted at 30 s",
            visor,
            (io, loop) -> {
              if (loop == 1500) {
                io.sequence = -1;
                io.dropped = 0;
              }
            },
            "Sending",
            ok,
            List.of(
                new Line("Frames", note),
                new Line("Images", note),
                new Line("Dropped at Visor", ok),
                new Line("Lost on the way", ok),
                new Line("Detection", ok),
                new Line("Visor restarted", note),
                new Line("Exposure", note)),
            Seen.onVisor(30.0, 0, 0, 5.0, 1),
            List.of("Visor restarted: once this session", "Lost on the way: 0 in the last")),
        new Scenario(
            "gone quiet at 20 s",
            visor,
            (io, loop) -> {
              if (loop == 1000) {
                io.connected = false;
              }
            },
            "Not sending",
            problem,
            onVisor(ok, ok, ok),
            Seen.onVisor(0.0, 0, 0, Double.NaN, 0),
            List.of("Frames: 0.0 fps lately", "Detection: no frames lately")),
        new Scenario(
            "on Visor on a robot, with no lens calibration",
            CameraConfig.Software.visor().device("/dev/v4l/by-id/usb-front-video-index0"),
            (io, loop) -> {},
            "Sending",
            problem,
            List.of(
                new Line("Frames", note),
                new Line("Images", note),
                new Line("Lens", problem),
                new Line("Dropped at Visor", ok),
                new Line("Lost on the way", ok),
                new Line("Detection", ok),
                new Line("Exposure", note)),
            Seen.onVisor(30.0, 0, 0, 5.0, 0),
            List.of(
                "Lens: has no lens calibration: Visor sends its tags' corners and no poses, so"
                    + " it places nothing. Calibrate it with Visor's tools/calibrate.py")),
        new Scenario(
            "on PhotonVision",
            CameraConfig.Software.PHOTONVISION,
            (io, loop) -> io.numbered = false,
            "Sending",
            ok,
            List.of(
                new Line("Frames", note),
                new Line("Images", note),
                new Line("Drops and detection", note)),
            new Seen(30.0, OptionalLong.empty(), OptionalLong.empty(), Double.NaN, 0),
            List.of(
                "Frames: 30.0 fps lately", "Drops and detection: not reported by PhotonVision")));
  }

  /** Each scenario's card: its state and level, each line's level, and what the health saw. */
  @Test
  void eachScenarioShowsOnTheCard() {
    for (Scenario scenario : scenarios()) {
      Scripted io = new Scripted();
      try (Camera camera = aMinuteOf(scenario, io)) {
        Devices.Card card = new CameraPit(camera, CameraPit.Frames.FROM_THE_COPROCESSORS).card();
        String name = scenario.name();
        assertThat(card.state()).as(name).isEqualTo(scenario.state());
        assertThat(card.level()).as(name).isEqualTo(scenario.level());
        assertThat(card.lines())
            .as(name)
            .map(line -> new Line(line.label(), line.level()))
            .isEqualTo(scenario.lines());
        assertThat(Seen.of(camera.health())).as(name).isEqualTo(scenario.seen());
      }
    }
  }

  /** What the card's lines say, in words: the one test of the wording. */
  @Test
  void eachLineSaysItInWords() {
    for (Scenario scenario : scenarios()) {
      Scripted io = new Scripted();
      try (Camera camera = aMinuteOf(scenario, io)) {
        Devices.Card card = new CameraPit(camera, CameraPit.Frames.FROM_THE_COPROCESSORS).card();
        List<String> lines =
            card.lines().stream().map(line -> line.label() + ": " + line.value()).toList();
        for (String said : scenario.says()) {
          assertThat(lines)
              .as(scenario.name())
              .anySatisfy(line -> assertThat(line).startsWith(said));
        }
      }
    }
  }

  /** A minute of the scenario's frames, read each loop as the robot does. */
  private static Camera aMinuteOf(Scenario scenario, Scripted io) {
    Camera camera = new Camera(camera("Front Left", scenario.software()), io, 500);
    for (int loop = 0; loop <= 3000; loop++) {
      scenario.atLoop().accept(io, loop);
      camera.read(loop * SECOND / 50);
    }
    return camera;
  }

  /**
   * What the robot logs of a camera is all a replay needs: a minute of frames with drops, losses
   * and a Visor restart, logged loop by loop as AdvantageKit logs the inputs, then read back from
   * those tables alone, shows the same card.
   */
  @Test
  void aReplayOfTheLogShowsTheSameCard() {
    Scripted io = new Scripted();
    List<LogTable> logged = new ArrayList<>();
    Devices.Card live;
    try (Camera camera = new Camera(camera("front", CameraConfig.Software.VISOR), io, 500)) {
      for (int loop = 0; loop <= 3000; loop++) {
        if (loop % 400 == 0) {
          io.dropNext = 1;
          io.loseNext = 1;
        }
        if (loop == 2000) {
          io.sequence = -1;
          io.dropped = 0;
        }
        camera.read(loop * SECOND / 50);
        LogTable table = new LogTable(loop * 20_000L);
        camera.inputs.toLog(table);
        logged.add(table);
      }
      live = new CameraPit(camera, CameraPit.Frames.FROM_THE_COPROCESSORS).card();
    }
    int[] next = {0};
    CameraIO fromTheLog =
        new CameraIO() {
          @Override
          public void updateInputs(CameraIOInputs inputs) {
            ((CameraIOInputsAutoLogged) inputs).fromLog(logged.get(next[0]++));
          }
        };
    try (Camera replayed =
        new Camera(camera("front", CameraConfig.Software.VISOR), fromTheLog, 500)) {
      for (int loop = 0; loop <= 3000; loop++) {
        replayed.read(loop * SECOND / 50);
      }
      assertThat(new CameraPit(replayed, CameraPit.Frames.FROM_THE_COPROCESSORS).card())
          .isEqualTo(live);
      assertThat(live.level()).isEqualTo(Devices.Level.WARNING);
    }
  }

  /**
   * A camera's live view is on the board it's plugged into, where Visor serves it; otherwise the
   * card says plainly why there's none, in the picture's place.
   */
  @Test
  void aCardsLiveViewIsOnTheBoardItsFramesComeFromOrSaysWhyNot() {
    record Case(
        String name,
        CameraConfig.Software software,
        CameraPit.Frames frames,
        String address,
        Devices.Picture picture) {}
    CameraConfig.Software visor = CameraConfig.Software.VISOR;
    CameraPit.Frames robot = CameraPit.Frames.FROM_THE_COPROCESSORS;
    List<Case> cases =
        List.of(
            new Case(
                "on a robot, on its board",
                visor,
                robot,
                "10.12.34.11",
                Devices.Picture.at("http://10.12.34.11:5809/stream/Front%20Left")),
            new Case(
                "on a robot, before a frame from a listed board or a report of where it is",
                visor,
                robot,
                "",
                Devices.Picture.none(
                    "No live view yet: no frame has come from one of the robot's coprocessors, and"
                        + " none reports the camera plugged in, so the board it's on isn't known")),
            new Case(
                "in the simulator",
                visor,
                CameraPit.Frames.FROM_THE_SIMULATOR,
                "",
                Devices.Picture.none(
                    "No live view in the simulator: its cameras have no pictures")),
            new Case(
                "in a replay of a robot's log",
                visor,
                CameraPit.Frames.FROM_THE_LOG,
                "10.12.34.11",
                Devices.Picture.none("No live view in a replay: the log has no pictures")),
            new Case(
                "on PhotonVision",
                CameraConfig.Software.PHOTONVISION,
                robot,
                "",
                Devices.Picture.none(
                    "No live view here: PhotonVision shows the camera on its own page, at its"
                        + " coprocessor's port 5800")));
    for (Case c : cases) {
      Scripted io = new Scripted();
      io.address = c.address();
      try (Camera camera = new Camera(camera("Front Left", c.software()), io, 500)) {
        for (int loop = 0; loop <= 100; loop++) {
          camera.read(loop * SECOND / 50);
        }
        assertThat(new CameraPit(camera, c.frames()).card().picture())
            .as(c.name())
            .hasValue(c.picture());
      }
    }
  }

  /**
   * The page may load the cameras' live views from Visor's stream on each listed coprocessor, as
   * listed, and from nowhere else; a robot that lists none names no place.
   */
  @Test
  void theLiveViewsComeFromEachListedCoprocessorsStream() {
    record Case(String name, List<String> boards, List<String> origins) {}
    List<Case> cases =
        List.of(
            new Case("none listed", List.of(), List.of()),
            new Case(
                "two by address, one by name",
                List.of("10.12.34.11", "10.12.34.12", "visor-front.local"),
                List.of(
                    "http://10.12.34.11:5809",
                    "http://10.12.34.12:5809",
                    "http://visor-front.local:5809")));
    for (Case c : cases) {
      assertThat(CameraPit.pictureOrigins(c.boards())).as(c.name()).isEqualTo(c.origins());
    }
  }

  /**
   * A coprocessor on another Visor version shows on the card of every camera on Visor, read as the
   * robot reads it, through the camera, and says what to do.
   */
  @Test
  void aVisorVersionMismatchShowsOnEveryVisorCamerasCard() {
    VisorFeed feed = VisorFeed.inMemory();
    long now = 10 * SECOND;
    try (Camera front =
            new Camera(
                camera("front", CameraConfig.Software.VISOR),
                new CameraIOVisor(feed, 0, () -> now),
                500);
        Camera back =
            new Camera(
                camera("back", CameraConfig.Software.VISOR),
                new CameraIOVisor(feed, 1, () -> now),
                500)) {
      VisorFrame stale = new VisorFrame();
      stale.setHeader(0, 1600, 1304, now - SECOND / 10, 0, true, false);
      stale.setVersionId(20251225);
      feed.offer(stale);
      front.read(now);
      back.read(now);
      for (Camera camera : List.of(front, back)) {
        Devices.Card card = new CameraPit(camera, CameraPit.Frames.FROM_THE_COPROCESSORS).card();
        assertThat(card.state()).as(camera.name()).isEqualTo("Can't be read");
        assertThat(card.lines())
            .as(camera.name())
            .anySatisfy(line -> {
              assertThat(line.label()).isEqualTo("Problem");
              assertThat(line.value())
                  .startsWith("a coprocessor runs Visor 20251225")
                  .endsWith("deploy to it again");
            });
      }
    }
  }

  @Test
  void aCamerasNameOnThePageIsItsId() {
    assertThat(CameraPit.deviceName(camera("Front Left", CameraConfig.Software.VISOR)))
        .isEqualTo("front-left");
    assertThat(CameraPit.deviceName(camera("rear.cam_2", CameraConfig.Software.VISOR)))
        .isEqualTo("rear-cam-2");
  }
}

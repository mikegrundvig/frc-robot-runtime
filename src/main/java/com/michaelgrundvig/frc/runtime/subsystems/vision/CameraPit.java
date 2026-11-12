package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth;
import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth.Warning;
import com.michaelgrundvig.frc.runtime.lib.health.Durations;
import com.michaelgrundvig.frc.runtime.lib.health.Episodes;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

/**
 * Each camera's card on Pitwall's Pit view: whether its frames arrive, at what rate, whether its
 * vision program dropped frames or the network lost them, and whether detection keeps up with the
 * camera, over the last 10 s; and each of those this session, so a drop during a match still shows
 * after it. Then the exposure and gain the latest frame was taken at. Beside them, a camera on
 * Visor has its live view, which the browser loads straight from the coprocessor its frames come
 * from (the video never passes through the robot program), and its exposure and gain's tuning,
 * where it has one, to tune there as in the Tune view. It's the camera's one card: what else
 * Pitwall shows of a camera belongs on it.
 *
 * <p>The card is read in the robot loop, from what the loop already read.
 */
public final class CameraPit {
  /** Where the robot's camera frames come from, which says whether a card has a live view. */
  public enum Frames {
    /** The coprocessors: a camera on Visor has its live view, from the board it's on. */
    FROM_THE_COPROCESSORS,
    /** The simulator, whose cameras have no pictures. */
    FROM_THE_SIMULATOR,
    /** A log being replayed, which has no pictures. */
    FROM_THE_LOG
  }

  /**
   * Whether the camera's frames arrive, as the card's state says it: its words, and how the card
   * stands for it at the least.
   */
  enum State {
    /** Its frames arrive. */
    SENDING("Sending", Devices.Level.OK),
    /** Its frames can't be read: the reason is the card's first line. */
    CANT_BE_READ("Can't be read", Devices.Level.PROBLEM),
    /** No frame has come lately. */
    NOT_SENDING("Not sending", Devices.Level.PROBLEM);

    final String label;
    final Devices.Level level;

    State(String label, Devices.Level level) {
      this.label = label;
      this.level = level;
    }

    /** The camera's state from what the loop read of it: an error first, then silence. */
    static State of(CameraIO.CameraIOInputs inputs) {
      if (!inputs.error.isEmpty()) {
        return CANT_BE_READ;
      }
      if (!inputs.connected) {
        return NOT_SENDING;
      }
      return SENDING;
    }
  }

  /** The span a count "lately" covers, as the card says it: camera health's window. */
  private static final String LATELY =
      String.format(Locale.ROOT, " in the last %.0f s", CameraHealth.WINDOW_SECONDS);

  private final Camera camera;
  private final Frames frames;

  CameraPit(Camera camera, Frames frames) {
    this.camera = camera;
    this.frames = frames;
  }

  /**
   * A card for each of the robot's cameras, in its definition's order.
   *
   * @param frames where the frames come from
   * @param tunings each Visor camera's tuning, by the camera's name, shown on its card
   */
  public static List<Devices.Device> devices(
      Vision vision, Frames frames, Map<String, ? extends LiveConfig<?>> tunings) {
    List<Devices.Device> devices = new ArrayList<>();
    for (Camera camera : vision.cameras()) {
      CameraPit pit = new CameraPit(camera, frames);
      Devices.Device device = Devices.Device.named(deviceName(camera.config), "Camera", pit::card);
      LiveConfig<?> tuning = tunings.get(camera.config.name());
      if (tuning != null) {
        device.tuning(tuning.name());
      }
      devices.add(device);
    }
    return devices;
  }

  /**
   * Where the cameras' live views may come from, as the page's policy for images names them ({@link
   * Devices#picturesFrom}): Visor's stream on each coprocessor the robot lists. A camera's view is
   * only ever on a listed board, so these are every place one comes from.
   *
   * @param boards the robot's coprocessors, each by its address as its configuration lists it
   */
  public static List<String> pictureOrigins(List<String> boards) {
    List<String> origins = new ArrayList<>();
    for (String board : boards) {
      // The stream's own address, without its path: one owner for where Visor serves it.
      URI stream = URI.create(CameraIOVisor.liveView(board, "camera"));
      origins.add(stream.getScheme() + "://" + stream.getRawAuthority());
    }
    return origins;
  }

  /**
   * A camera's name on the page: its id, with the dots and underscores a device's name can't have.
   * The robot's configuration checks that no other card has it.
   */
  public static String deviceName(CameraConfig config) {
    return config.id().replaceAll("[._]", "-");
  }

  /** The card as of now. */
  Devices.Card card() {
    CameraIO.CameraIOInputs inputs = camera.inputs;
    CameraHealth health = camera.health();
    boolean onVisor = camera.config.software() instanceof CameraConfig.Software.Visor;
    List<Devices.Line> lines = new ArrayList<>();
    State state = State.of(inputs);
    if (state == State.CANT_BE_READ) {
      lines.add(new Devices.Line("Problem", inputs.error, Devices.Level.PROBLEM));
    }
    lines.add(framesLine(health));
    lines.add(imagesLine(camera));
    Optional<String> withoutALens = Camera.withoutALens(camera.config);
    if (withoutALens.isPresent()) {
      lines.add(new Devices.Line("Lens", withoutALens.get(), Devices.Level.PROBLEM));
    }
    if (onVisor) {
      lines.add(warningLine("Dropped at Visor", dropped(health), health, Warning.DROPPED_FRAMES));
      lines.add(warningLine("Lost on the way", lost(health), health, Warning.LOST_FRAMES));
      lines.add(warningLine("Detection", detection(health), health, Warning.SLOW_DETECTION));
      if (health.restarts() > 0) {
        lines.add(
            new Devices.Line(
                "Visor restarted",
                Durations.times(health.restarts()) + " this session",
                Devices.Level.NOTE));
      }
      lines.add(new Devices.Line("Exposure", exposure(inputs), Devices.Level.NOTE));
    } else {
      lines.add(
          new Devices.Line(
              "Drops and detection", "not reported by PhotonVision", Devices.Level.NOTE));
    }
    // A camera that isn't sending has nothing more to say on a line, so its state alone is wrong.
    Devices.Level level = state.level.worse(Devices.Level.of(lines));
    return Devices.Card.of(state.label, level, lines).withPicture(picture(onVisor, inputs));
  }

  /**
   * The camera's live view: Visor's stream on the board its frames come from, or why there's none.
   */
  private Devices.Picture picture(boolean onVisor, CameraIO.CameraIOInputs inputs) {
    if (!onVisor) {
      return Devices.Picture.none(
          "No live view here: PhotonVision shows the camera on its own page, at its coprocessor's"
              + " port 5800");
    }
    return switch (frames) {
      case FROM_THE_SIMULATOR ->
          Devices.Picture.none("No live view in the simulator: its cameras have no pictures");
      case FROM_THE_LOG ->
          Devices.Picture.none("No live view in a replay: the log has no pictures");
      case FROM_THE_COPROCESSORS -> {
        if (inputs.visorAddress.isEmpty()) {
          yield Devices.Picture.none(
              "No live view yet: no frame has come from one of the robot's coprocessors, and none"
                  + " reports the camera plugged in, so the board it's on isn't known");
        }
        yield Devices.Picture.at(CameraIOVisor.liveView(inputs.visorAddress, camera.config.name()));
      }
    };
  }

  /**
   * What became of the camera's images: how many with tags were used, the leading reason the rest
   * weren't, and the latest image's verdict in its own words, so a camera that places nothing says
   * why ("saw no tags", "tags 31, 32 aren't in this field's layout", no lens calibration). A
   * warning while it has used none of the images with tags it took.
   */
  private static Devices.Line imagesLine(Camera camera) {
    Optional<String> latest = camera.latestVerdict();
    if (latest.isEmpty()) {
      return new Devices.Line("Images", "none yet", Devices.Level.NOTE);
    }
    long withTags = camera.imagesWithTags();
    if (withTags == 0) {
      return new Devices.Line(
          "Images", "none with tags yet; latest: " + latest.get(), Devices.Level.NOTE);
    }
    String value = camera.imagesUsedWords() + "; latest: " + latest.get();
    if (camera.imagesUsed() == 0) {
      return new Devices.Line("Images", value, Devices.Level.WARNING);
    }
    return new Devices.Line("Images", value, Devices.Level.NOTE);
  }

  private static Devices.Line framesLine(CameraHealth health) {
    double fps = health.framesPerSecond();
    double configured = 1000 / health.framePeriodMs();
    String value = "measuring";
    if (!Double.isNaN(fps)) {
      value = String.format(Locale.ROOT, "%.1f fps lately (configured %.0f)", fps, configured);
    }
    return new Devices.Line("Frames", value, Devices.Level.NOTE);
  }

  /**
   * A line that warns while its warning holds, and after it while the session remembers it: what it
   * says lately, then, if it ever held, when.
   */
  private static Devices.Line warningLine(
      String label, String lately, CameraHealth health, Warning warning) {
    Episodes episodes = health.episodes(warning);
    if (episodes.count() == 0) {
      return new Devices.Line(label, lately, Devices.Level.OK);
    }
    String value = lately + "; " + episodes.describe(health.nowNanos());
    return new Devices.Line(label, value, Devices.Level.WARNING);
  }

  private static String dropped(CameraHealth health) {
    if (health.numbers() == CameraHealth.Numbers.NOT_REPORTED) {
      return "not reported yet";
    }
    return health.droppedLately() + LATELY;
  }

  private static String lost(CameraHealth health) {
    if (health.numbers() == CameraHealth.Numbers.NOT_REPORTED) {
      return "not reported yet";
    }
    return health.lostLately() + LATELY;
  }

  private static String detection(CameraHealth health) {
    if (Double.isNaN(health.detectMeanMs())) {
      return "no frames lately";
    }
    return String.format(
        Locale.ROOT,
        "%.1f ms mean, %.1f ms worst, of a %.1f ms frame",
        health.detectMeanMs(),
        health.detectWorstMs(),
        health.framePeriodMs());
  }

  /** The exposure and gain the latest frame was taken at, and how bright it was. */
  private static String exposure(CameraIO.CameraIOInputs inputs) {
    if (inputs.brightnessMedian < 0) {
      return "no frame yet";
    }
    String exposure = "unknown";
    if (inputs.exposureMs >= 0) {
      exposure = String.format(Locale.ROOT, "%.2f ms", inputs.exposureMs);
    } else if (inputs.autoExposure) {
      exposure = "the camera's own";
    }
    String gain = "no gain control";
    if (inputs.gain >= 0) {
      gain = "gain " + inputs.gain;
    }
    return String.format(
        Locale.ROOT,
        "%s, %s; brightness %d median, %d at 99%%",
        exposure,
        gain,
        inputs.brightnessMedian,
        inputs.brightnessHigh);
  }
}

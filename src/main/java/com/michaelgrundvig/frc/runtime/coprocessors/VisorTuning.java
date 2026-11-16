package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.world.field.VenueCameras;
import com.michaelgrundvig.frc.world.robot.Camera;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraType;
import com.michaelgrundvig.frc.world.robot.CameraType.Exposure;
import com.michaelgrundvig.frc.world.robot.CameraType.Gain;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.IntFunction;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.measure.Time;

/**
 * Each Visor camera's exposure and gain: as its type gives them, as the selected venue keeps them,
 * and as the pit tunes them live from Pitwall ({@link CameraSettings} says which wins). While the
 * robot is disabled, a tuned value reaches the camera's coprocessor as a new Visor configuration
 * ({@link VisorInstaller}, which configures a board whenever its configuration isn't the one
 * wanted), and Visor restarts the camera with it. At an event, dial a camera in while disabled,
 * watch its brightness in the log, and keep the value in the venue from the camera's card: it then
 * survives a restart, and the venue's file waits to be downloaded and committed.
 *
 * <p>Every camera on Visor has a gain to tune. One whose type and venue leave the gain to the
 * camera starts from the gain its frames report. Its exposure is tunable where its type fixes one;
 * a camera on its own automatic exposure has none to tune.
 */
public final class VisorTuning {
  /** The shortest exposure tuning goes to: V4L2's step, the shortest a UVC camera can be set to. */
  static final Time SHORTEST = Exposure.Fixed.SHORTEST;

  /** The highest gain tuning goes to: UVC's gain control is an unsigned 16-bit number. */
  static final double HIGHEST_GAIN = Gain.Level.MOST;

  /**
   * The gain a camera left at its own starts at until its frames say what its own is. It's never
   * written to the camera unless it's tuned, so no camera runs at it by mistake.
   */
  static final int GAIN_UNTIL_REPORTED = 0;

  /** The tunable value of a camera's exposure. */
  static final String EXPOSURE = "exposure";

  /** The tunable value of a camera's gain. */
  static final String GAIN = "gain";

  private VisorTuning() {}

  /**
   * Each Visor camera's tuning, by the camera's name, in the definition's order. Each starts at its
   * type's values; the robot moves it to the venue's as it plays on one ({@link Live#update}).
   */
  public static Map<String, LiveConfig<CameraTuning>> of(List<CameraConfig> cameras) {
    Map<String, LiveConfig<CameraTuning>> tunings = new LinkedHashMap<>();
    for (CameraConfig camera : cameras) {
      of(camera).ifPresent(tuning -> tunings.put(camera.name(), tuning));
    }
    return Collections.unmodifiableMap(tunings);
  }

  /**
   * A camera's tuning, named for it ({@code "front-left camera"}): its gain, across UVC's range,
   * which Visor holds to the camera's own, and, where its type fixes its exposure, its exposure,
   * from V4L2's shortest to one frame's time (a longer one slows the frame rate). Empty for a
   * camera not on Visor.
   */
  public static Optional<LiveConfig<CameraTuning>> of(CameraConfig camera) {
    if (!(camera.software() instanceof Camera.Software.Visor)) {
      return Optional.empty();
    }
    CameraType type = camera.type();
    LiveConfig<CameraTuning> tuning = LiveConfig.of(camera.name() + " camera", start(type));
    if (type.exposure() instanceof Exposure.Fixed fixed) {
      Time time = fixed.time();
      Time frame = Seconds.of(1 / camera.sensor().frameRate().in(Hertz));
      // Wide enough for the type's exposure, so none fails the robot's start.
      Time shortest = time.lt(SHORTEST) ? time : SHORTEST;
      Time longest = time.gt(frame) ? time : frame;
      tuning.measure(
          EXPOSURE,
          shortest,
          longest,
          "keep it in the venue from its card, or its type's .exposure(%s)");
    }
    // The gain the kind's control takes, where its type says; UVC's whole range otherwise.
    Limits gains = Limits.between(0, HIGHEST_GAIN);
    if (type.gainRange().isPresent()) {
      gains = Limits.between(type.gainRange().get().lowest(), type.gainRange().get().highest());
    }
    tuning.number(GAIN, gains, "keep it in the venue from its card, or its type's .gain(%s)");
    return Optional.of(tuning);
  }

  /**
   * Where a camera of a type starts: its fixed exposure, and its gain level where it gives one. A
   * type on its own automatic exposure has no exposure to tune, so the shortest stands in, unread;
   * one left at the camera's own gain starts at the least its range takes until its frames say.
   */
  private static CameraTuning start(CameraType type) {
    Time exposure = SHORTEST;
    if (type.exposure() instanceof Exposure.Fixed fixed) {
      exposure = fixed.time();
    }
    int gain = type.gainRange().map(CameraType.GainRange::lowest).orElse(GAIN_UNTIL_REPORTED);
    if (type.gain() instanceof Gain.Level level) {
      gain = level.value();
    }
    return CameraTuning.of(exposure, gain);
  }

  /**
   * The robot's cameras' settings as they are now, kept up each loop ({@link Live#update}).
   *
   * @param cameras the robot's cameras, in its definition's order
   * @param tunings each Visor camera's tuning, by its name ({@link #of(List)})
   */
  public static Live live(
      List<CameraConfig> cameras, Map<String, LiveConfig<CameraTuning>> tunings) {
    return new Live(cameras, tunings);
  }

  /**
   * The robot's cameras' exposure and gain: as kept (each type's, and the venue's), and as they run
   * now, tuned values among them. Each list stays the same object while nothing changed, so a loop
   * with nothing new builds nothing.
   */
  public static final class Live {
    private final List<CameraConfig> cameras;
    private final Map<String, LiveConfig<CameraTuning>> tunings;

    /** Each camera's tuning, by its index; null for one with none. */
    private final List<@Nullable LiveConfig<CameraTuning>> byIndex = new ArrayList<>();

    /** What each tuning held when the lists were last built, by the camera's index. */
    private final List<@Nullable CameraTuning> seen = new ArrayList<>();

    /** The gain each camera reported when the lists were last built, by its index; -1 for none. */
    private final int[] reported;

    /** Why a camera's tuning couldn't start from what's kept, by its index; empty for nothing. */
    private final List<String> problems = new ArrayList<>();

    private VenueCameras venue = VenueCameras.NONE;
    private boolean everUpdated;
    private volatile List<CameraSettings> kept;
    private volatile List<CameraSettings> now;

    /** The robot's report of its cameras' settings, as it was last said. */
    private String[] said = new String[0];

    private Live(List<CameraConfig> cameras, Map<String, LiveConfig<CameraTuning>> tunings) {
      this.cameras = List.copyOf(cameras);
      this.tunings = tunings;
      reported = new int[this.cameras.size()];
      Arrays.fill(reported, -1);
      for (CameraConfig camera : this.cameras) {
        byIndex.add(tunings.get(camera.name()));
        seen.add(null);
        problems.add("");
      }
      kept = List.copyOf(CameraSettings.ofTypes(this.cameras));
      now = kept;
    }

    /** The robot's cameras, as its definition gives them. */
    public List<CameraConfig> cameras() {
      return cameras;
    }

    /** Each camera's settings as kept, its type's and its venue's, by its index. */
    public List<CameraSettings> kept() {
      return kept;
    }

    /** Each camera's settings as it runs now, tuned values among them, by its index. */
    public List<CameraSettings> now() {
      return now;
    }

    /**
     * The robot's report of its cameras' settings, as it last said it: a line for each camera on
     * Visor, and one for what the venue gives that the robot has no camera for.
     */
    public List<String> report() {
      return List.of(said);
    }

    /** A camera's settings as it runs now, by its name, if it's one of the robot's. */
    public Optional<CameraSettings> now(String camera) {
      int index = indexOf(camera);
      if (index < 0) {
        return Optional.empty();
      }
      return Optional.of(now.get(index));
    }

    /**
     * What's wrong with what's kept for a camera, if anything: a venue's value its tuning can't
     * start from. Empty for nothing.
     */
    public String problem(String camera) {
      int index = indexOf(camera);
      if (index < 0) {
        return "";
      }
      return problems.get(index);
    }

    private int indexOf(String camera) {
      for (int i = 0; i < cameras.size(); i++) {
        if (cameras.get(i).name().equals(camera)) {
          return i;
        }
      }
      return -1;
    }

    /**
     * The values tuned on a camera and not kept yet, as a venue keeps them: each that differs from
     * where the camera started. Nothing for a camera with no tuning, or nothing tuned. Safe from
     * Pitwall's threads.
     */
    public VenueCameras.Settings tuned(String camera) {
      LiveConfig<CameraTuning> tuning = tunings.get(camera);
      if (tuning == null) {
        return VenueCameras.Settings.NONE;
      }
      // The values and what of them is tuned, read together: the loop may be moving the tuning.
      LiveConfig.Tuned<CameraTuning> tuned = tuning.tuned();
      CameraTuning values = tuned.value();
      Optional<Time> exposure = Optional.empty();
      Optional<Integer> gain = Optional.empty();
      for (LiveConfig.Change change : tuned.changes()) {
        if (change.path().equals(EXPOSURE)) {
          exposure = Optional.of(values.exposure());
        }
        if (change.path().equals(GAIN)) {
          gain = Optional.of(values.gain());
        }
      }
      return new VenueCameras.Settings(exposure, gain);
    }

    /**
     * Once a loop: what each camera runs at, from the venue in use, each camera's reported gain and
     * each tuning. A tuning starts from what's kept, so a value kept in the venue is no longer a
     * tuned one, and a value tuned away from it stays tuned. The first time, and whenever anything
     * changed, the robot says each camera's settings and where they came from, and logs them.
     *
     * @param venue the venue in use's camera settings: none on the official layout
     * @param reportedGain the gain a camera's latest frame was taken at, by its index, if known
     */
    public void update(VenueCameras venue, IntFunction<OptionalInt> reportedGain) {
      boolean changed = !everUpdated || !venue.equals(this.venue);
      everUpdated = true;
      this.venue = venue;
      for (int i = 0; i < cameras.size(); i++) {
        int gain = reportedGain.apply(i).orElse(-1);
        if (gain != reported[i]) {
          reported[i] = gain;
          changed = true;
        }
        LiveConfig<CameraTuning> tuning = byIndex.get(i);
        if (tuning != null && tuning.get() != seen.get(i)) {
          changed = true;
        }
      }
      if (changed) {
        rebuild();
      }
      Logger.recordOutput("Vision/CameraSettings", said);
    }

    /** Works out each camera's settings again, and says them if they changed. */
    private void rebuild() {
      List<CameraSettings> keptNow = new ArrayList<>(cameras.size());
      List<CameraSettings> runningNow = new ArrayList<>(cameras.size());
      for (int i = 0; i < cameras.size(); i++) {
        CameraSettings keptOne = CameraSettings.kept(cameras.get(i), venue);
        keptNow.add(keptOne);
        runningNow.add(running(i, keptOne));
      }
      if (!keptNow.equals(kept)) {
        kept = List.copyOf(keptNow);
      }
      if (!runningNow.equals(now)) {
        now = List.copyOf(runningNow);
      }
      say();
    }

    /**
     * What a camera runs at: what's kept, its tuning moved to start there, and each value tuned
     * away from it in its place.
     */
    private CameraSettings running(int index, CameraSettings keptOne) {
      LiveConfig<CameraTuning> tuning = byIndex.get(index);
      problems.set(index, "");
      if (tuning == null) {
        return keptOne;
      }
      boolean gainTuned = tuning.changes().stream().anyMatch(change -> change.path().equals(GAIN));
      CameraTuning start =
          startAt(
              tuning.committed(),
              keptOne,
              reported[index],
              gainTuned,
              cameras.get(index).type().gainRange());
      if (!start.equals(tuning.committed())) {
        try {
          tuning.rebase(start);
        } catch (IllegalArgumentException outside) {
          problems.set(index, "its tuning can't start from what's kept: " + outside.getMessage());
        }
      }
      seen.set(index, tuning.get());
      CameraSettings running = keptOne;
      for (LiveConfig.Change change : tuning.changes()) {
        if (change.path().equals(EXPOSURE)) {
          running = running.exposureTuned(Exposure.fixed(tuning.get().exposure()));
        }
        if (change.path().equals(GAIN)) {
          running = running.gainTuned(Gain.level(tuning.get().gain()));
        }
      }
      return running;
    }

    /**
     * Where a camera's tuning starts: its kept exposure where it's fixed, and its kept gain where
     * it's a level, or the gain its frames report where it's the camera's own and they've said. A
     * gain tuned on a camera left at its own is what its frames report once it's applied, so the
     * report moves the start only while no gain is tuned: otherwise the tuned gain would become the
     * start, and so no longer tuned, and the camera would go back to its own.
     */
    private static CameraTuning startAt(
        CameraTuning was,
        CameraSettings keptOne,
        int reported,
        boolean gainTuned,
        Optional<CameraType.GainRange> range) {
      Time exposure = keptOne.fixedExposure().orElse(was.exposure());
      int gain = was.gain();
      if (keptOne.gain() instanceof Gain.Level level) {
        gain = level.value();
      } else if (reported >= 0 && !gainTuned) {
        gain = Math.min(reported, Gain.Level.MOST);
      }
      // A gain the kind's control doesn't take, reported or kept, starts at the nearest it does.
      if (range.isPresent()) {
        gain = range.get().nearest(gain);
      }
      return CameraTuning.of(exposure, gain);
    }

    /**
     * What the venue gives that this robot has no camera for, as a line of the report: a camera
     * type or a station it doesn't have. A venue's file may be shared between robots, so what it
     * gives another robot is carried in the file, unused here, and said once when the venue is
     * taken.
     */
    private List<String> unused() {
      Set<String> types = new TreeSet<>(venue.types().keySet());
      Set<String> stations = new TreeSet<>(venue.stations().keySet());
      for (CameraConfig camera : cameras) {
        types.remove(camera.type().name());
        stations.remove(camera.name());
      }
      List<String> unused = new ArrayList<>();
      if (!types.isEmpty()) {
        unused.add(
            "the venue gives camera types this robot has none of, kept in its file and unused here: "
                + String.join(", ", types));
      }
      if (!stations.isEmpty()) {
        unused.add(
            "the venue gives stations this robot has none of, kept in its file and unused here: "
                + String.join(", ", stations));
      }
      for (CameraConfig camera : cameras) {
        Optional<Integer> kept = venue.forCamera(camera.name(), camera.type().name()).gain();
        Optional<CameraType.GainRange> range = camera.type().gainRange();
        if (kept.isPresent() && range.isPresent() && !range.get().holds(kept.get())) {
          unused.add(
              "the venue's gain for "
                  + camera.name()
                  + ", "
                  + kept.get()
                  + ", is outside the "
                  + range.get().lowest()
                  + " to "
                  + range.get().highest()
                  + " a "
                  + camera.type().name()
                  + " takes: it runs at "
                  + range.get().nearest(kept.get())
                  + ", until a gain is kept in its place");
        }
        Optional<VenueCameras.StationSettings> own = venue.stationsOwn(camera.name());
        if (own.isPresent() && !own.get().type().equals(camera.type().name())) {
          unused.add(
              "the venue's own settings for "
                  + camera.name()
                  + " were kept for camera type "
                  + own.get().type()
                  + ", and it holds camera type "
                  + camera.type().name()
                  + " now: unused until they're kept again");
        }
      }
      return unused;
    }

    /** Says each Visor camera's settings, and where each came from, when they're new. */
    private void say() {
      List<String> lines = new ArrayList<>();
      for (int i = 0; i < cameras.size(); i++) {
        if (!(cameras.get(i).software() instanceof Camera.Software.Visor)) {
          continue;
        }
        String line = now.get(i).sentence(cameras.get(i));
        if (!problems.get(i).isEmpty()) {
          line += "; " + problems.get(i);
        }
        lines.add(line);
      }
      lines.addAll(unused());
      String[] lined = lines.toArray(String[]::new);
      if (!Arrays.equals(lined, said)) {
        said = lined;
        for (String line : said) {
          System.out.println("Camera settings: " + line);
        }
      }
    }
  }
}

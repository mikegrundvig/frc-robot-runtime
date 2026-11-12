package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Time;

/**
 * Each Visor camera's exposure and gain, tuned live from Pitwall's Tune view as the drivetrain's
 * and the joints' gains are. While the robot is disabled, a tuned value reaches the camera's
 * coprocessor as a new Visor configuration ({@link VisorInstaller}, which configures a board
 * whenever its configuration isn't the one wanted), and Visor restarts the camera with it. At an
 * event, dial a camera in while disabled, watch its brightness in the log, and write the value that
 * works into {@code Robots.java}: the Tune view gives the builder call.
 *
 * <p>A camera's tuning has what its definition fixes: the exposure, where it's a fixed time, and
 * the gain, where it's a level. A camera on its own automatic exposure and gain has nothing to
 * tune, and no tuning; give it {@code .gain(...)} to tune its gain.
 */
public final class VisorTuning {
  /** The shortest exposure tuning goes to: V4L2's step, the shortest a UVC camera can be set to. */
  static final Time SHORTEST = Microseconds.of(100);

  /** The highest gain tuning goes to: UVC's gain control is an unsigned 16-bit number. */
  static final double HIGHEST_GAIN = 65_535;

  private VisorTuning() {}

  /**
   * Each Visor camera's tuning that has a value to tune, by the camera's name, in the definition's
   * order.
   */
  public static Map<String, LiveConfig<Software.Visor>> of(List<CameraConfig> cameras) {
    Map<String, LiveConfig<Software.Visor>> tunings = new LinkedHashMap<>();
    for (CameraConfig camera : cameras) {
      of(camera).ifPresent(tuning -> tunings.put(camera.name(), tuning));
    }
    return Collections.unmodifiableMap(tunings);
  }

  /**
   * A camera's tuning, named for it ({@code "front-left camera"}): its fixed exposure, from V4L2's
   * shortest to one frame's time (a longer one slows the frame rate), and its gain level, across
   * UVC's range, which Visor holds to the camera's own. Empty for a camera not on Visor, or with
   * nothing fixed to tune.
   */
  public static Optional<LiveConfig<Software.Visor>> of(CameraConfig camera) {
    if (!(camera.software() instanceof Software.Visor visor)) {
      return Optional.empty();
    }
    boolean fixedExposure = visor.exposure() instanceof Software.Exposure.Fixed;
    boolean givenGain = visor.gain() instanceof Software.Gain.Level;
    if (!fixedExposure && !givenGain) {
      return Optional.empty();
    }
    LiveConfig<Software.Visor> tuning = LiveConfig.of(camera.name() + " camera", visor);
    if (visor.exposure() instanceof Software.Exposure.Fixed fixed) {
      Time time = fixed.time();
      Time frame = Seconds.of(1 / camera.sensor().frameRate().in(Hertz));
      // Wide enough for any committed exposure, so none fails the robot's start.
      Time shortest = time.lt(SHORTEST) ? time : SHORTEST;
      Time longest = time.gt(frame) ? time : frame;
      tuning.measure("exposure.time", shortest, longest, ".exposure(%s)");
    }
    if (givenGain) {
      tuning.number("gain.value", Limits.between(0, HIGHEST_GAIN), ".gain(%s)");
    }
    return Optional.of(tuning);
  }

  /**
   * The robot's cameras, each on Visor with its software as tuned now: what each board's
   * configuration is written from.
   *
   * @param tunings each tuned camera's, by its name ({@link #of(List)})
   */
  public static List<CameraConfig> tuned(
      List<CameraConfig> cameras, Map<String, LiveConfig<Software.Visor>> tunings) {
    List<CameraConfig> tuned = new ArrayList<>(cameras.size());
    for (CameraConfig camera : cameras) {
      LiveConfig<Software.Visor> tuning = tunings.get(camera.name());
      // By value: a tuning that holds the committed software again leaves the camera as it is.
      if (tuning == null || tuning.get().equals(camera.software())) {
        tuned.add(camera);
      } else {
        tuned.add(camera.withSoftware(tuning.get()));
      }
    }
    return tuned;
  }

  /**
   * The robot's cameras as tuned now, read each loop ({@link VisorInstaller}): the same list while
   * no tuning changed, so a loop with nothing tuned, or the same values tuned, builds nothing.
   *
   * @param cameras the robot's cameras as committed, in its definition's order
   * @param tunings each tuned camera's, by its name ({@link #of(List)})
   */
  public static Live live(
      List<CameraConfig> cameras, Map<String, LiveConfig<Software.Visor>> tunings) {
    return new Live(cameras, tunings);
  }

  /** The robot's cameras, as committed and as tuned now ({@link #live}). */
  public static final class Live implements Supplier<List<CameraConfig>> {
    private final List<CameraConfig> committed;
    private final Map<String, LiveConfig<Software.Visor>> tunings;

    /** Each camera's tuning, by its index; null for one with none. */
    private final List<@Nullable LiveConfig<Software.Visor>> byIndex = new ArrayList<>();

    /** The software each tuning held when the list was last built, by the camera's index. */
    private final List<Software.@Nullable Visor> built = new ArrayList<>();

    private List<CameraConfig> tuned;

    private Live(List<CameraConfig> cameras, Map<String, LiveConfig<Software.Visor>> tunings) {
      this.committed = List.copyOf(cameras);
      this.tunings = tunings;
      for (CameraConfig camera : committed) {
        LiveConfig<Software.Visor> tuning = tunings.get(camera.name());
        byIndex.add(tuning);
        built.add(tuning == null ? null : tuning.get());
      }
      this.tuned = List.copyOf(tuned(committed, tunings));
    }

    /** The robot's cameras as its definition commits them. */
    public List<CameraConfig> committed() {
      return committed;
    }

    /** The robot's cameras as tuned now: the same list until a tuning changes. */
    @Override
    public List<CameraConfig> get() {
      boolean changed = false;
      for (int i = 0; i < byIndex.size(); i++) {
        LiveConfig<Software.Visor> tuning = byIndex.get(i);
        if (tuning == null) {
          continue;
        }
        Software.Visor now = tuning.get();
        if (!now.equals(built.get(i))) {
          built.set(i, now);
          changed = true;
        }
      }
      if (changed) {
        tuned = List.copyOf(tuned(committed, tunings));
      }
      return tuned;
    }
  }
}

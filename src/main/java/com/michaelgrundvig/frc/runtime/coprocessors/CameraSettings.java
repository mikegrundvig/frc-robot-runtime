package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.field.VenueCameras;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraType;
import com.michaelgrundvig.frc.world.robot.CameraType.Exposure;
import com.michaelgrundvig.frc.world.robot.CameraType.Gain;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

/**
 * The exposure and gain one camera on Visor runs at, and where each came from. Each is the first of
 * these that gives one:
 *
 * <ol>
 *   <li>a value tuned live from Pitwall, until it's kept in the venue or the robot restarts;
 *   <li>the selected venue's, for the camera's station, kept there in the pit;
 *   <li>the selected venue's, for every camera of its type;
 *   <li>the camera's type's, in the robot's code.
 * </ol>
 *
 * @param exposure the exposure Visor sets
 * @param exposureFrom where it came from
 * @param gain the gain Visor sets
 * @param gainFrom where it came from
 */
public record CameraSettings(Exposure exposure, Source exposureFrom, Gain gain, Source gainFrom) {
  /** Where a camera's exposure or gain came from. */
  public enum Source {
    /** The camera's type, in the robot's code: where every camera starts. */
    CAMERA_TYPE("its type's, from the code"),
    /** The selected venue's, for every camera of the camera's type. */
    VENUE_TYPE("the venue's, for every %s"),
    /** The selected venue's, for this camera's station. */
    VENUE_STATION("the venue's, for its station"),
    /** Tuned live from Pitwall, not kept yet. */
    TUNED("tuned live, not kept");

    /** Where it came from, in words, {@code %s} where the camera type's name goes. */
    private final String words;

    Source(String words) {
      this.words = words;
    }

    /**
     * Where it came from, in words for the startup report and the camera's card: "the venue's, for
     * every ThriftyCam".
     *
     * @param type the camera type's name
     */
    public String words(String type) {
      return words.replace("%s", type);
    }
  }

  /**
   * What a camera runs at before anything is tuned: its venue's where the venue gives one, for its
   * station or for its type, and its type's own otherwise.
   *
   * @param venue the selected venue's camera settings: none on the official layout
   */
  public static CameraSettings kept(CameraConfig camera, VenueCameras venue) {
    CameraType type = camera.type();
    // A station's own were kept for the kind of camera it held then: another kind ignores them.
    VenueCameras.Settings station = venue.stationsOwn(camera.name(), type.name());
    VenueCameras.Settings ofType = venue.typesOwn(type.name()).orElse(VenueCameras.Settings.NONE);

    Exposure exposure = type.exposure();
    Source exposureFrom = Source.CAMERA_TYPE;
    if (station.exposure().isPresent()) {
      exposure = Exposure.fixed(station.exposure().get());
      exposureFrom = Source.VENUE_STATION;
    } else if (ofType.exposure().isPresent()) {
      exposure = Exposure.fixed(ofType.exposure().get());
      exposureFrom = Source.VENUE_TYPE;
    }

    Gain gain = type.gain();
    Source gainFrom = Source.CAMERA_TYPE;
    if (station.gain().isPresent()) {
      gain = Gain.level(inRange(station.gain().get(), type));
      gainFrom = Source.VENUE_STATION;
    } else if (ofType.gain().isPresent()) {
      gain = Gain.level(inRange(ofType.gain().get(), type));
      gainFrom = Source.VENUE_TYPE;
    }
    return new CameraSettings(exposure, exposureFrom, gain, gainFrom);
  }

  /**
   * A venue's gain as a kind of camera takes it: the nearest its range holds, where its type gives
   * one. A file edited by hand, or kept before the range was given, may hold one outside it; the
   * robot runs the nearest and says so, rather than setting the whole venue aside for one number.
   */
  private static int inRange(int gain, CameraType type) {
    return type.gainRange().map(range -> range.nearest(gain)).orElse(gain);
  }

  /**
   * Each camera's settings at its type's, on no venue: what the robot is checked with as it starts.
   */
  public static List<CameraSettings> ofTypes(List<CameraConfig> cameras) {
    List<CameraSettings> settings = new ArrayList<>(cameras.size());
    for (CameraConfig camera : cameras) {
      settings.add(kept(camera, VenueCameras.NONE));
    }
    return settings;
  }

  /** The same, its exposure tuned to this. */
  CameraSettings exposureTuned(Exposure tuned) {
    return new CameraSettings(tuned, Source.TUNED, gain, gainFrom);
  }

  /** The same, its gain tuned to this. */
  CameraSettings gainTuned(Gain tuned) {
    return new CameraSettings(exposure, exposureFrom, tuned, Source.TUNED);
  }

  /** Its exposure in words: "1.5 ms", or "the camera's own". */
  public String exposureInWords() {
    if (exposure instanceof Exposure.Fixed fixed) {
      return String.format(Locale.ROOT, "%.1f ms", fixed.time().in(Milliseconds));
    }
    return "the camera's own";
  }

  /** Its gain in words: "40", or "the camera's own". */
  public String gainInWords() {
    if (gain instanceof Gain.Level level) {
      return String.valueOf(level.value());
    }
    return "the camera's own";
  }

  /**
   * The exposure and gain in words, each with where it came from: {@code "exposure 1.0 ms (its
   * type's, from the code), gain 40 (the venue's, for every ThriftyCam)"}.
   *
   * @param type the camera type's name
   */
  public String inWords(String type) {
    return "exposure "
        + exposureInWords()
        + " ("
        + exposureFrom.words(type)
        + "), gain "
        + gainInWords()
        + " ("
        + gainFrom.words(type)
        + ")";
  }

  /**
   * The line the robot says for a camera as it starts, and whenever its settings change: {@code
   * "front-left: exposure 1.0 ms (its type's, from the code), gain 40 (the venue's, for every
   * ThriftyCam)"}.
   */
  public String sentence(CameraConfig camera) {
    return camera.name() + ": " + inWords(camera.type().name());
  }

  /** Its fixed exposure's time, if it has one. */
  Optional<org.wpilib.units.measure.Time> fixedExposure() {
    if (exposure instanceof Exposure.Fixed fixed) {
      return Optional.of(fixed.time());
    }
    return Optional.empty();
  }
}

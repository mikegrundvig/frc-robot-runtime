package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.TestCameras;
import com.michaelgrundvig.frc.runtime.coprocessors.CameraSettings.Source;
import com.michaelgrundvig.frc.world.field.VenueCameras;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraType;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalInt;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * Where each camera's exposure and gain come from: its type, its venue for every camera or for its
 * station, or a live tuning not kept yet; and what each board is written from them.
 */
class CameraSettingsTest {
  /** The guide's front-left camera, moved to Visor at a device, at its type's defaults. */
  private static final CameraConfig FRONT =
      TestCameras.onVisor(
          ExampleRobot.SIM.definition().cameras().get(0),
          "/dev/v4l/by-id/usb-Maker_Cam_01-video-index0");

  /** The guide's cameras' type's name. */
  private static final String TYPE = FRONT.type().name();

  /** A station's own settings, kept with a camera of this type in it. */
  private static VenueCameras.StationSettings own(
      String type, @Nullable Double exposureMs, @Nullable Integer gain) {
    return new VenueCameras.StationSettings(type, settings(exposureMs, gain));
  }

  /** A venue giving the camera's type these, and nothing to any station. */
  private static VenueCameras ofType(@Nullable Double exposureMs, @Nullable Integer gain) {
    return new VenueCameras(Map.of(TYPE, settings(exposureMs, gain)), Map.of());
  }

  /** The same, its type giving a gain of its own. */
  private static final CameraConfig FRONT_AT_30 = TestCameras.retyped(FRONT, FRONT.type().gain(30));

  private static VenueCameras.Settings settings(
      @Nullable Double exposureMs, @Nullable Integer gain) {
    return new VenueCameras.Settings(
        Optional.ofNullable(exposureMs).map(Milliseconds::of), Optional.ofNullable(gain));
  }

  /** What a camera runs at before anything's tuned: the first of its venue's and its type's. */
  @Test
  void aCameraRunsAtItsVenuesWhereItGivesOneAndItsTypesOtherwise() {
    String station = FRONT.name();
    record Row(
        String what,
        CameraConfig camera,
        VenueCameras venue,
        String exposure,
        Source exposureFrom,
        String gain,
        Source gainFrom) {}
    List<Row> rows =
        List.of(
            new Row(
                "no venue, the type's defaults",
                FRONT,
                VenueCameras.NONE,
                "1.0 ms",
                Source.CAMERA_TYPE,
                "the camera's own",
                Source.CAMERA_TYPE),
            new Row(
                "no venue, the type's gain",
                FRONT_AT_30,
                VenueCameras.NONE,
                "1.0 ms",
                Source.CAMERA_TYPE,
                "30",
                Source.CAMERA_TYPE),
            new Row(
                "the venue's gain for the camera's type",
                FRONT_AT_30,
                ofType(null, 40),
                "1.0 ms",
                Source.CAMERA_TYPE,
                "40",
                Source.VENUE_TYPE),
            new Row(
                "another type's is not this one's",
                FRONT_AT_30,
                new VenueCameras(Map.of("Arducam", settings(2.0, 8)), Map.of()),
                "1.0 ms",
                Source.CAMERA_TYPE,
                "30",
                Source.CAMERA_TYPE),
            new Row(
                "the station's own over its type's",
                FRONT,
                new VenueCameras(
                    Map.of(TYPE, settings(1.5, 40)), Map.of(station, own(TYPE, null, 52))),
                "1.5 ms",
                Source.VENUE_TYPE,
                "52",
                Source.VENUE_STATION),
            new Row(
                "another station's own is not this one's",
                FRONT,
                new VenueCameras(Map.of(), Map.of("elsewhere", own(TYPE, 2.0, 52))),
                "1.0 ms",
                Source.CAMERA_TYPE,
                "the camera's own",
                Source.CAMERA_TYPE),
            new Row(
                "the station's own kept with another kind of camera in it",
                FRONT,
                new VenueCameras(Map.of(), Map.of(station, own("Arducam", 2.0, 8))),
                "1.0 ms",
                Source.CAMERA_TYPE,
                "the camera's own",
                Source.CAMERA_TYPE));
    for (Row row : rows) {
      CameraSettings kept = CameraSettings.kept(row.camera(), row.venue());
      assertThat(kept.exposureInWords()).as(row.what()).isEqualTo(row.exposure());
      assertThat(kept.exposureFrom()).as(row.what()).isEqualTo(row.exposureFrom());
      assertThat(kept.gainInWords()).as(row.what()).isEqualTo(row.gain());
      assertThat(kept.gainFrom()).as(row.what()).isEqualTo(row.gainFrom());
    }
  }

  /**
   * A tuning starts where the camera runs: the venue's value, or for a camera at its own gain, the
   * gain its frames report. A value tuned away from that runs as tuned, even when the venue
   * changes, until it's kept; and each board is written the value running. The robot says each
   * camera's settings, and where they came from, when they change.
   */
  @Test
  void aTunedValueRunsUntilItsKeptAndTheBoardIsWrittenWhatRuns() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
    try {
      List<CameraConfig> cameras = List.of(FRONT);
      Map<String, LiveConfig<CameraTuning>> tunings = VisorTuning.of(cameras);
      VisorTuning.Live live = VisorTuning.live(cameras, tunings);
      LiveConfig<CameraTuning> tuning = Objects.requireNonNull(tunings.get(FRONT.name()));
      List<String> devices = List.of("usb-Maker_Cam_01-video-index0");

      live.update(VenueCameras.NONE, camera -> OptionalInt.of(32));
      assertThat(tuning.value("gain")).as("from the gain the camera reports").isEqualTo(32);
      assertThat(live.now().get(0).gainFrom()).isEqualTo(Source.CAMERA_TYPE);
      assertThat(board(live, devices)).as("its own gain: none written").doesNotContain("gain =");

      // Tuned on its own gain, and then reported back as Visor applies it: still tuned.
      assertThat(tuning.set(Map.of("gain", 38.0))).isTrue();
      live.update(VenueCameras.NONE, camera -> OptionalInt.of(32));
      live.update(VenueCameras.NONE, camera -> OptionalInt.of(38));
      assertThat(live.now().get(0).gainFrom())
          .as("still tuned once the camera reports it")
          .isEqualTo(Source.TUNED);
      assertThat(live.tuned(FRONT.name())).isEqualTo(settings(null, 38));
      assertThat(board(live, devices)).contains("gain = 38\n");
      tuning.revert();
      live.update(VenueCameras.NONE, camera -> OptionalInt.of(32));
      assertThat(live.now().get(0).gainFrom())
          .as("reverted: its own again")
          .isEqualTo(Source.CAMERA_TYPE);
      assertThat(tuning.value("gain")).as("from the gain it reports again").isEqualTo(32);

      VenueCameras week1 = ofType(1.5, 40);
      live.update(week1, camera -> OptionalInt.of(32));
      assertThat(tuning.value("gain")).as("from the venue's").isEqualTo(40);
      assertThat(tuning.value("exposure")).isEqualTo(1.5);
      assertThat(board(live, devices)).contains("exposure_us = 1500\ngain = 40\n");

      assertThat(tuning.set(Map.of("gain", 46.0))).isTrue();
      live.update(week1, camera -> OptionalInt.of(40));
      assertThat(live.now().get(0).gainFrom()).isEqualTo(Source.TUNED);
      assertThat(live.kept().get(0).gainFrom())
          .as("kept: the venue's")
          .isEqualTo(Source.VENUE_TYPE);
      assertThat(live.tuned(FRONT.name())).isEqualTo(settings(null, 46));
      assertThat(board(live, devices)).contains("gain = 46\n");

      VenueCameras otherVenue = ofType(1.5, 30);
      live.update(otherVenue, camera -> OptionalInt.of(46));
      assertThat(live.now().get(0).gainInWords())
          .as("still tuned, on another venue")
          .isEqualTo("46");

      VenueCameras keptThere = otherVenue.keptForType(TYPE, FRONT.name(), settings(null, 46));
      live.update(keptThere, camera -> OptionalInt.of(46));
      assertThat(live.now().get(0).gainFrom())
          .as("kept: the venue's now")
          .isEqualTo(Source.VENUE_TYPE);
      assertThat(live.tuned(FRONT.name())).isEqualTo(VenueCameras.Settings.NONE);
    } finally {
      TunableRegistry.reset();
    }
  }

  /**
   * A venue's exposure the camera's tuning can't start from, longer than one of its frames, is said
   * on its card, and the camera runs it untuned; the robot's report of its settings says so too.
   */
  @Test
  void aVenuesExposureLongerThanAFrameIsSaid() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
    try {
      List<CameraConfig> cameras = List.of(FRONT);
      VisorTuning.Live live = VisorTuning.live(cameras, VisorTuning.of(cameras));
      live.update(ofType(50.0, null), camera -> OptionalInt.empty());

      assertThat(live.problem(FRONT.name()))
          .contains("exposure")
          .contains("outside its tuning limits");
      assertThat(live.now().get(0).exposureInWords()).isEqualTo("50.0 ms");
      assertThat(live.now().get(0).sentence(FRONT))
          .isEqualTo(
              FRONT.name()
                  + ": exposure 50.0 ms (the venue's, for every "
                  + TYPE
                  + "), gain the camera's own (its type's, from the code)");
    } finally {
      TunableRegistry.reset();
    }
  }

  /**
   * A venue shared between robots gives types and stations a robot may not have: each robot carries
   * them in the file, uses its own, and says once, in its report, what it left unused.
   */
  @Test
  void whatAVenueGivesAnotherRobotIsCarriedAndSaidUnused() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
    try {
      List<CameraConfig> cameras = List.of(FRONT);
      VisorTuning.Live live = VisorTuning.live(cameras, VisorTuning.of(cameras));
      VenueCameras shared =
          new VenueCameras(
              Map.of(TYPE, settings(null, 40), "Arducam", settings(null, 8)),
              Map.of("turret", own(TYPE, null, 60), FRONT.name(), own("Arducam", null, 8)));
      live.update(shared, camera -> OptionalInt.empty());

      assertThat(live.now().get(0).gainInWords()).isEqualTo("40");
      assertThat(live.report())
          .contains(
              "the venue gives camera types this robot has none of, kept in its file and unused"
                  + " here: Arducam",
              "the venue gives stations this robot has none of, kept in its file and unused here:"
                  + " turret",
              "the venue's own settings for "
                  + FRONT.name()
                  + " were kept for camera type Arducam, and it holds camera type "
                  + TYPE
                  + " now: unused until they're kept again");
    } finally {
      TunableRegistry.reset();
    }
  }

  /**
   * A camera type that states its gain range: its tuning goes between those values, and starts at
   * the least while the camera's own gain is unreported; a venue's gain outside the range runs at
   * the nearest, and the robot's report says so; one inside runs as kept.
   */
  @Test
  void aTypesGainRangeHoldsItsTuningAndItsVenuesGain() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
    try {
      CameraConfig ranged = TestCameras.retyped(FRONT, FRONT.type().gainRange(16, 255));
      List<CameraConfig> cameras = List.of(ranged);
      Map<String, LiveConfig<CameraTuning>> tunings = VisorTuning.of(cameras);
      LiveConfig<CameraTuning> tuning = Objects.requireNonNull(tunings.get(ranged.name()));
      LiveConfig.Knob gain =
          tuning.knobs().stream().filter(k -> k.path().equals("gain")).findFirst().orElseThrow();
      assertThat(gain.limits().min()).isEqualTo(16);
      assertThat(gain.limits().max()).isEqualTo(255);
      assertThat(tuning.value("gain")).as("until the camera reports its own").isEqualTo(16);

      VisorTuning.Live live = VisorTuning.live(cameras, tunings);
      record Row(String what, int kept, String runs, boolean said) {}
      List<Row> rows =
          List.of(
              new Row("inside the range", 160, "160", false),
              new Row("above it", 400, "255", true),
              new Row("below it", 4, "16", true));
      for (Row row : rows) {
        live.update(ofType(null, row.kept()), camera -> OptionalInt.empty());
        assertThat(live.now().get(0).gainInWords()).as(row.what()).isEqualTo(row.runs());
        assertThat(
                live.report().stream().anyMatch(line -> line.contains("is outside the 16 to 255")))
            .as(row.what() + ": said")
            .isEqualTo(row.said());
      }
      assertThat(live.report())
          .contains(
              "the venue's gain for "
                  + ranged.name()
                  + ", 4, is outside the 16 to 255 a "
                  + TYPE
                  + " takes: it runs at 16, until a gain is kept in its place");
    } finally {
      TunableRegistry.reset();
    }
  }

  /** What a board with the camera is written, at its settings as running now. */
  private static String board(VisorTuning.Live live, List<String> devices) {
    return VisorConfigs.forBoard(live.cameras(), live.now(), 1234, 0.1651, devices).toml();
  }

  /** A type on its own automatic exposure, kept to the venue's fixed one there. */
  @Test
  void aVenuesExposureFixesOneLeftToTheCamera() {
    CameraConfig auto = TestCameras.retyped(FRONT, FRONT.type().camerasOwnExposure());
    assertThat(CameraSettings.kept(auto, VenueCameras.NONE).exposure())
        .isEqualTo(CameraType.Exposure.CAMERAS_OWN);
    assertThat(CameraSettings.kept(auto, ofType(2.0, null)).exposureInWords()).isEqualTo("2.0 ms");
  }
}

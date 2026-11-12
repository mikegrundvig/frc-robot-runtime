package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.visor.VisorPack;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagFamily;
import com.michaelgrundvig.frc.world.field.TagLayout;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.CameraConfigImageSensorBuilder;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;

/**
 * Visor's configuration for each coprocessor, from the guide's robot's four cameras put on Visor as
 * a team would: three on Visor, found by their devices, one left on PhotonVision, and a board
 * configured for whichever of them it reports plugged in.
 */
class VisorConfigsTest {
  private static final List<CameraConfig> GUIDE = ExampleRobot.SIM.definition().cameras();

  /** The season's tags' size, as the field gives it. */
  static final double TAG_SIZE = Rebuilt.GAME.officialField().tagSizeMeters();

  private static final String FRONT_LEFT = "usb-Thrifty_Cam_0001-video-index0";
  private static final String FRONT_RIGHT = "usb-Thrifty_Cam_0002-video-index0";
  private static final String BACK = "usb-Thrifty_Cam_0003-video-index0";

  private static CameraConfig on(CameraConfig camera, Software software) {
    return camera.withSoftware(software);
  }

  private static Software.Visor device(String name) {
    return Software.visor().device("/dev/v4l/by-id/" + name);
  }

  /** The guide's cameras: 0, 1 and 3 on Visor, the back one set up fully; 2 on PhotonVision. */
  static List<CameraConfig> cameras() {
    List<CameraConfig> cameras = new ArrayList<>(GUIDE);
    cameras.set(0, on(GUIDE.get(0), device(FRONT_LEFT)));
    cameras.set(1, on(GUIDE.get(1), device(FRONT_RIGHT)));
    CameraConfig.ImageSensor sensor = GUIDE.get(3).sensor();
    cameras.set(
        3,
        on(
            GUIDE.get(3),
            Software.visor()
                .device("/dev/v4l/by-id/" + BACK)
                .exposure(Milliseconds.of(2.5))
                .gain(40)
                .controls(List.of(Software.Control.of("power_line_frequency", 2)))
                .timeOffset(Microseconds.of(-1500))
                .lens(
                    Software.Lens.builder()
                        .width(sensor.widthPixels())
                        .height(sensor.heightPixels())
                        .fx(905.25)
                        .fy(905.5)
                        .cx(641.25)
                        .cy(398.75)
                        .distortion(List.of(-0.3125, 0.125, 0.0, 0.0, -0.03))
                        .build())));
    return cameras;
  }

  @Test
  void aBoardIsConfiguredForTheRobotsCamerasPluggedIntoIt() {
    assertThat(GUIDE).hasSize(4);
    record Case(String board, List<String> plugged, List<String> configured) {}
    List<Case> cases =
        List.of(
            new Case(
                "both fronts and a camera not the robot's",
                List.of(FRONT_LEFT, "usb-Other-video-index0", FRONT_RIGHT),
                List.of("index = 0", "index = 1")),
            new Case("the back", List.of(BACK), List.of("index = 3")),
            new Case(
                "a front moved beside the back",
                List.of(BACK, FRONT_RIGHT),
                List.of("index = 1", "index = 3")));
    for (Case c : cases) {
      String file = VisorConfigs.forBoard(cameras(), 1234, TAG_SIZE, c.plugged()).toml();
      assertThat(file).as(c.board()).contains("team = 1234\nport = 5807\n");
      assertThat(file.split("\\[\\[camera]]")).as(c.board()).hasSize(c.configured().size() + 1);
      for (String index : c.configured()) {
        assertThat(file).as(c.board()).contains(index + "\n");
      }
    }
    // Each camera's entry as the definition says it.
    String front = VisorConfigs.forBoard(cameras(), 1234, TAG_SIZE, List.of(FRONT_LEFT)).toml();
    assertThat(front)
        .contains("name = \"" + GUIDE.get(0).name() + "\"\nindex = 0\n")
        .contains("device = \"/dev/v4l/by-id/" + FRONT_LEFT + "\"")
        .contains("width = " + GUIDE.get(0).sensor().widthPixels() + "\n")
        .contains("fps = " + Math.round(GUIDE.get(0).sensor().frameRate().baseUnitMagnitude()))
        .doesNotContain("[camera.lens]")
        .doesNotContain("[camera.controls]");
    // Written nothing of its exposure or gain: the defaults, a fixed 1 ms and the camera's gain.
    assertThat(front).contains("exposure_us = 1000\n").doesNotContain("gain =");
    String back = VisorConfigs.forBoard(cameras(), 1234, TAG_SIZE, List.of(BACK)).toml();
    assertThat(back)
        .contains("time_offset_us = -1500\nexposure_us = 2500\ngain = 40\n")
        .contains("[camera.controls]\n\"power_line_frequency\" = 2\n")
        .contains("[camera.lens]\nwidth = " + GUIDE.get(3).sensor().widthPixels())
        .contains("fx = 905.25\nfy = 905.5\ncx = 641.25\ncy = 398.75\n")
        .contains("distortion = [-0.3125, 0.125, 0.0, 0.0, -0.03]\n");
  }

  /** The camera's own automatic exposure and gain are written as nothing: Visor puts them back. */
  @Test
  void aCameraOnItsOwnExposureAndGainIsWrittenWithNeither() {
    List<CameraConfig> cameras = cameras();
    cameras.set(
        0,
        on(
            GUIDE.get(0),
            Software.visor().device("/dev/v4l/by-id/" + FRONT_LEFT).camerasOwnExposure()));
    String front = VisorConfigs.forBoard(cameras, 1234, TAG_SIZE, List.of(FRONT_LEFT)).toml();
    assertThat(front).doesNotContain("exposure_us").doesNotContain("gain");
  }

  /**
   * Each camera's live view as its definition gives it, and never faster than the camera, since
   * Visor refuses a view faster than its camera; one it doesn't give is written as nothing, so
   * Visor's own applies.
   */
  @Test
  void eachCamerasLiveViewIsWrittenNoFasterThanTheCamera() {
    // A camera slower than the fastest live view, so a view can ask for more than it runs at.
    CameraConfig slow =
        CameraConfigBuilder.builder(GUIDE.get(0))
            .sensor(
                CameraConfigImageSensorBuilder.builder(GUIDE.get(0).sensor())
                    .frameRate(Hertz.of(20))
                    .build())
            .build();
    record Case(String name, CameraConfig camera, String written) {}
    Software.Visor visor = device(FRONT_LEFT);
    List<Case> cases =
        List.of(
            new Case("written nothing", on(GUIDE.get(0), visor), ""),
            new Case(
                "its own",
                on(GUIDE.get(0), visor.stream(Percent.of(25), Hertz.of(5), 80)),
                "stream_scale = 0.25\nstream_fps = 5\nstream_quality = 80\n"),
            new Case(
                "faster than the camera",
                on(slow, visor.stream(Percent.of(100), Hertz.of(30), 60)),
                "stream_scale = 1.0\nstream_fps = 20\nstream_quality = 60\n"));
    for (Case c : cases) {
      List<CameraConfig> cameras = cameras();
      cameras.set(0, c.camera());
      String front = VisorConfigs.forBoard(cameras, 1234, TAG_SIZE, List.of(FRONT_LEFT)).toml();
      if (c.written().isEmpty()) {
        assertThat(front).as(c.name()).doesNotContain("stream_");
        continue;
      }
      assertThat(front).as(c.name()).contains(c.written());
    }
  }

  /**
   * A board with none of the robot's cameras is given a configuration with no camera, so a Visor
   * still running cameras the robot no longer has stops them; its hash is the one its report gives.
   */
  @Test
  void aBoardWithNoneOfTheRobotsCamerasIsGivenNoCamera() {
    String none =
        "# Visor's configuration, written by the robot's deploy: edits here are overwritten.\n"
            + "team = 1234\n"
            + "port = 5807\n"
            + "tag_size_m = 0.1651\n";
    record Row(String what, List<CameraConfig> cameras, List<String> plugged) {}
    for (Row row :
        List.of(
            new Row("nothing plugged in", cameras(), List.of()),
            new Row("another camera", cameras(), List.of("usb-Other-video-index0")),
            new Row("every camera on PhotonVision", GUIDE, List.of(FRONT_LEFT)))) {
      VisorConfigs.ForBoard board =
          VisorConfigs.forBoard(row.cameras(), 1234, TAG_SIZE, row.plugged());
      assertThat(board.empty()).as(row.what()).isTrue();
      assertThat(board.cameras()).as(row.what()).isEmpty();
      assertThat(board.toml()).as(row.what()).isEqualTo(none);
      assertThat(board.hash()).as(row.what()).isEqualTo(VisorPack.configHash(none));
    }
    VisorConfigs.ForBoard both =
        VisorConfigs.forBoard(cameras(), 1234, TAG_SIZE, List.of(BACK, FRONT_LEFT));
    assertThat(both.cameras()).as("by index, in the robot's order").containsExactly(0, 3);
    assertThat(both.empty()).isFalse();
  }

  /**
   * Every board's configuration gives the field's tag size, so Visor solves each camera's poses at
   * the size of the tags it sees: the season's 6.5 in, or a game's other size.
   */
  @Test
  void everyConfigurationGivesTheFieldsTagSize() {
    record Row(String field, double tagSizeMeters, String written) {}
    for (Row row :
        List.of(
            new Row("the season's", TAG_SIZE, "tag_size_m = 0.1651\n"),
            new Row("eight-inch tags", 0.2032, "tag_size_m = 0.2032\n"))) {
      for (List<String> plugged : List.of(List.of(FRONT_LEFT), List.<String>of())) {
        String file = VisorConfigs.forBoard(cameras(), 1234, row.tagSizeMeters(), plugged).toml();
        assertThat(file).as(row.field() + ", " + plugged).contains("port = 5807\n" + row.written());
      }
    }
  }

  /** A field of the game's family and size, or one Visor can't detect. */
  private static FieldModel field(TagFamily family) {
    return FieldModel.builder(
            TagLayout.of(Fields.FRC_2026_REBUILT_WELDED.loadField(), family, Inches.of(6.5)),
            Symmetry.ROTATED)
        .build();
  }

  /**
   * A robot with a camera on Visor is refused as it starts on a field whose tags Visor doesn't
   * detect, saying which camera and why; one with every camera on PhotonVision, which detects both
   * families, is not. On the season's field neither is refused.
   */
  @Test
  void aCameraOnVisorIsRefusedOnAFieldOfAnotherFamily() {
    record Row(String what, List<CameraConfig> cameras, TagFamily family, String refused) {}
    String why =
        GUIDE.get(0).name()
            + " is on Visor, which detects TAG_36H11 tags only, and the field's tags are TAG_16H5:"
            + " it would see none of them";
    List<Row> rows =
        List.of(
            new Row("Visor cameras on the season's field", cameras(), TagFamily.TAG_36H11, ""),
            new Row("Visor cameras on 16h5 tags", cameras(), TagFamily.TAG_16H5, why),
            new Row("PhotonVision cameras on 16h5 tags", GUIDE, TagFamily.TAG_16H5, ""));
    for (Row row : rows) {
      FieldModel field = field(row.family());
      if (row.refused().isEmpty()) {
        assertThatCode(() -> VisorConfigs.requireRunnable(row.cameras(), field))
            .as(row.what())
            .doesNotThrowAnyException();
        continue;
      }
      assertThatThrownBy(() -> VisorConfigs.requireRunnable(row.cameras(), field))
          .as(row.what())
          .isInstanceOf(IllegalArgumentException.class)
          .hasMessageContaining(row.refused())
          .hasMessageContaining(GUIDE.get(1).name() + " is on Visor");
    }
  }

  /**
   * A camera that names a control the exposure or gain sets, by any name V4L2's drivers give it, is
   * refused as the robot starts, in Visor's words: a second owner would undo the setting and its
   * live tuning. A camera that names no device yet is judged too. Other controls are taken.
   */
  @Test
  void aCameraNamingTheExposureOrGainAsAControlIsRefusedAtStartup() {
    record Row(String control, String refused) {}
    List<Row> rows =
        List.of(
            new Row("auto_exposure", "control auto_exposure is set by exposureMicros(..)"),
            new Row("exposure_auto", "control exposure_auto is set by exposureMicros(..)"),
            new Row(
                "exposure_time_absolute",
                "control exposure_time_absolute is set by exposureMicros(..)"),
            new Row("exposure_absolute", "control exposure_absolute is set by exposureMicros(..)"),
            new Row("gain", "control gain is set by gain(..)"),
            new Row("power_line_frequency", ""));
    FieldModel field = Rebuilt.GAME.officialField();
    for (Row row : rows) {
      for (Software.Visor visor : List.of(device(FRONT_LEFT), Software.visor())) {
        List<CameraConfig> cameras = cameras();
        Software.Visor named = visor.controls(List.of(Software.Control.of(row.control(), 1)));
        cameras.set(0, on(GUIDE.get(0), named));
        String what = row.control() + (visor.device().isPresent() ? "" : ", no device yet");
        if (row.refused().isEmpty()) {
          assertThatCode(() -> VisorConfigs.requireRunnable(cameras, field))
              .as(what)
              .doesNotThrowAnyException();
          continue;
        }
        assertThatThrownBy(() -> VisorConfigs.requireRunnable(cameras, field))
            .as(what)
            .isInstanceOf(IllegalArgumentException.class)
            .hasMessageContaining(GUIDE.get(0).name() + ": Visor refuses it: " + row.refused());
      }
    }
  }

  @Test
  void aCameraNoBoardHasIsNamed() {
    List<CameraConfig> cameras = cameras();
    cameras.set(2, on(GUIDE.get(2), Software.VISOR));
    assertThat(VisorConfigs.unplaced(cameras, List.of(FRONT_LEFT, FRONT_RIGHT, BACK)))
        .containsExactly(GUIDE.get(2).name() + " (it names no device)");
    assertThat(VisorConfigs.unplaced(cameras(), List.of(FRONT_LEFT)))
        .containsExactly(
            GUIDE.get(1).name() + " (" + FRONT_RIGHT + ")",
            GUIDE.get(3).name() + " (" + BACK + ")");
    assertThat(VisorConfigs.unplaced(cameras(), List.of(FRONT_LEFT, FRONT_RIGHT, BACK))).isEmpty();
  }
}

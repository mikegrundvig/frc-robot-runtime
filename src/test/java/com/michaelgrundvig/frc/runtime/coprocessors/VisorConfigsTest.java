package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Microseconds;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Visor's configuration for each coprocessor, from the guide's robot's four cameras put on Visor as
 * a team would: three on Visor, found by their devices, one left on PhotonVision, and a board
 * configured for whichever of them it reports plugged in.
 */
class VisorConfigsTest {
  private static final List<CameraConfig> GUIDE = ExampleRobot.SIM.definition().cameras();
  private static final String FRONT_LEFT = "usb-Thrifty_Cam_0001-video-index0";
  private static final String FRONT_RIGHT = "usb-Thrifty_Cam_0002-video-index0";
  private static final String BACK = "usb-Thrifty_Cam_0003-video-index0";

  private static CameraConfig on(CameraConfig camera, Software software) {
    return CameraConfigBuilder.builder(camera).software(software).build();
  }

  private static Software.Visor device(String name) {
    return Software.visor().device("/dev/v4l/by-id/" + name).build();
  }

  /** The guide's cameras: 0, 1 and 3 on Visor, the back one set up fully; 2 on PhotonVision. */
  static List<CameraConfig> cameras() {
    List<CameraConfig> cameras = new ArrayList<>(GUIDE);
    cameras.set(0, on(GUIDE.get(0), device(FRONT_LEFT)));
    cameras.set(1, on(GUIDE.get(1), device(FRONT_RIGHT)));
    CameraConfig.Sensor sensor = GUIDE.get(3).sensor();
    cameras.set(
        3,
        on(
            GUIDE.get(3),
            Software.visor()
                .device("/dev/v4l/by-id/" + BACK)
                .controls(
                    List.of(
                        Software.Control.of("auto_exposure", 1),
                        Software.Control.of("exposure_time_absolute", 10)))
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
                        .build())
                .build()));
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
      String file = VisorConfigs.forBoard(cameras(), 1234, c.plugged()).orElseThrow();
      assertThat(file).as(c.board()).contains("team = 1234\n");
      assertThat(file.split("\\[\\[camera]]")).as(c.board()).hasSize(c.configured().size() + 1);
      for (String index : c.configured()) {
        assertThat(file).as(c.board()).contains(index + "\n");
      }
    }
    // Each camera's entry as the definition says it.
    String front = VisorConfigs.forBoard(cameras(), 1234, List.of(FRONT_LEFT)).orElseThrow();
    assertThat(front)
        .contains("name = \"" + GUIDE.get(0).name() + "\"\nindex = 0\n")
        .contains("device = \"/dev/v4l/by-id/" + FRONT_LEFT + "\"")
        .contains("width = " + GUIDE.get(0).sensor().widthPixels() + "\n")
        .contains("fps = " + Math.round(GUIDE.get(0).sensor().frameRate().baseUnitMagnitude()))
        .doesNotContain("[camera.lens]")
        .doesNotContain("[camera.controls]");
    String back = VisorConfigs.forBoard(cameras(), 1234, List.of(BACK)).orElseThrow();
    assertThat(back)
        .contains("time_offset_us = -1500\n")
        .contains("[camera.controls]\n\"auto_exposure\" = 1\n\"exposure_time_absolute\" = 10\n")
        .contains("[camera.lens]\nwidth = " + GUIDE.get(3).sensor().widthPixels())
        .contains("fx = 905.25\nfy = 905.5\ncx = 641.25\ncy = 398.75\n")
        .contains("distortion = [-0.3125, 0.125, 0.0, 0.0, -0.03]\n");
  }

  @Test
  void aBoardWithNoneOfTheRobotsCamerasNeedsNoVisor() {
    assertThat(VisorConfigs.forBoard(cameras(), 1234, List.of())).isEmpty();
    assertThat(VisorConfigs.forBoard(cameras(), 1234, List.of("usb-Other-video-index0"))).isEmpty();
    assertThat(VisorConfigs.forBoard(GUIDE, 1234, List.of(FRONT_LEFT)))
        .as("every camera on PhotonVision")
        .isEmpty();
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

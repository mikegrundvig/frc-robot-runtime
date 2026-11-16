package com.michaelgrundvig.frc.toolconfigs.checklist;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.toolconfigs.advantagescope.TestRobot;
import com.michaelgrundvig.frc.world.robot.Camera;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraStation;
import com.michaelgrundvig.frc.world.robot.CameraType;
import com.michaelgrundvig.frc.world.robot.ImageSensor;
import com.michaelgrundvig.frc.world.tools.RobotDescription;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * A robot's checklist, from its definition: each station's camera by its label and serial, and the
 * pit's and field's checks, for cameras on Visor, on PhotonVision, and on Visor without a serial.
 */
class CompetitionChecklistTest {
  private static final CameraType THRIFTYCAM =
      CameraType.of("ThriftyCam", ImageSensor.of(1600, 1304, Hertz.of(60)));

  private static CameraStation station(String name, double left) {
    return CameraStation.named(name)
        .at(Inches.of(10), Inches.of(left), Inches.of(20))
        .facing(Degrees.of(10), Degrees.of(0));
  }

  /** Three cameras, each a kind of camera the pit meets. */
  private static final List<CameraConfig> CAMERAS =
      List.of(
          station("front-left", 8)
              .holds(
                  Camera.visor(1, THRIFTYCAM)
                      .device("/dev/v4l/by-id/usb-Maker_Camera_00A1B2C3-video-index0")),
          station("front-right", -8)
              .holds(
                  Camera.visor(2, THRIFTYCAM)
                      .device("/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0")),
          station("back", 0).holds(Camera.photonVision(3, THRIFTYCAM)));

  private static RobotDescription robot(String name, List<CameraConfig> cameras) {
    return new RobotDescription(
        name, TestRobot.body(), TestRobot.drivetrain(), cameras, Optional.empty());
  }

  private static final String CHECKLIST = CompetitionChecklist.checklist(robot("comp", CAMERAS));

  @Test
  void eachStationSaysWhichCameraBelongsThere() {
    assertThat(CHECKLIST)
        .startsWith("# Pre-competition checklist: comp\n\n" + CompetitionChecklist.GENERATED)
        .contains("| front-left | 1 | `B2C3` | ThriftyCam | Visor |\n")
        .contains(
            "| front-right | 2 | none: named by its USB port, so it goes back in the same port |"
                + " ThriftyCam | Visor |\n")
        .contains(
            "| back | 3 | kept by PhotonVision, not in the code | ThriftyCam | PhotonVision |\n")
        .contains("No camera is tied to a coprocessor in the code");
  }

  /** Each check, one line a box, in the order the pit and the field meet them. */
  @Test
  void eachCheckIsABoxInTheOrderTheyreMet() {
    List<String> boxes =
        CHECKLIST
            .lines()
            .filter(line -> line.startsWith("- [ ] "))
            .map(line -> line.substring(6))
            .toList();
    assertThat(boxes)
        .containsExactly(
            "front-left holds camera 1, its serial ending `B2C3`.",
            "front-right holds camera 2, plugged into the USB port its code names.",
            "back holds camera 3.",
            "front-left reads a tag on its Pitwall card, with no dark, clipping or"
                + " incomplete-frame warning.",
            "front-right reads a tag on its Pitwall card, with no dark, clipping or"
                + " incomplete-frame warning.",
            "back reads a tag on PhotonVision's own page, which keeps its settings.",
            "The event's venue is chosen on the venue card. With no survey of the field, make it"
                + " there: New venue for this event, with the event's name.",
            "Read front-left's dark and clipping warnings.",
            "Read front-right's dark and clipping warnings.",
            "Adjust the ThriftyCams' gain once, from one of their cards, until none warns dark or"
                + " clipping, and keep it: Keep in this venue: every ThriftyCam.",
            "Download the venue from the venue card, and commit it to `src/main/deploy/venues/`.");
    assertThat(CHECKLIST).contains("with the robot disabled and no field system connected");
  }

  /** A robot with no cameras, or none on Visor, says so in place of the checks it has none of. */
  @Test
  void aRobotWithoutCamerasOnVisorSaysSo() {
    assertThat(CompetitionChecklist.checklist(robot("bare", List.of())))
        .endsWith("The robot has no cameras, so there's nothing about them to check.\n");
    assertThat(CompetitionChecklist.checklist(robot("photon", List.of(CAMERAS.get(2)))))
        .as("no camera on Visor: nothing to say of coprocessors")
        .doesNotContain("coprocessor");
    assertThat(CompetitionChecklist.checklist(robot("photon", List.of(CAMERAS.get(2)))))
        .endsWith(
            "No camera is on Visor: PhotonVision's own page sets each camera for the field's"
                + " light.\n");
  }

  /** The build's plugin runs it: a file a robot, by its name, the same every time. */
  @Test
  void eachRobotHasAFileOfItsOwn(@TempDir Path out) throws IOException {
    CompetitionChecklist.write(
        List.of(robot("comp", CAMERAS), robot("practice bot", List.of())), out);
    assertThat(Files.readString(out.resolve("comp.md"))).isEqualTo(CHECKLIST);
    assertThat(out.resolve("practice_bot.md")).exists();
    assertThatThrownBy(() ->
            CompetitionChecklist.write(
                List.of(robot("a b", List.of()), robot("a_b", List.of())), out))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("would share a checklist");
    assertThat(CompetitionChecklist.tail("AB", 4)).isEqualTo("AB");
  }

  /**
   * The serials' tails are as long as it takes to tell the robot's cameras apart, and a station's
   * name with a {@code |} in it stays in its cell.
   */
  @Test
  void eachCameraReadsApartAndAStationsNameStaysInItsCell() {
    record Row(String what, List<String> serials, List<String> tails) {}
    List<Row> rows =
        List.of(
            new Row("apart in four", List.of("00A1B2C3", "00A1D4E5"), List.of("B2C3", "D4E5")),
            new Row(
                "alike in four", List.of("11A1B2C3", "22A1B2C3"), List.of("1A1B2C3", "2A1B2C3")),
            new Row("one camera", List.of("00A1B2C3"), List.of("B2C3")));
    for (Row row : rows) {
      List<CameraConfig> cameras = new java.util.ArrayList<>();
      for (int i = 0; i < row.serials().size(); i++) {
        cameras.add(
            station("s" + i, i)
                .holds(
                    Camera.visor(i + 1, THRIFTYCAM)
                        .device(
                            "/dev/v4l/by-id/usb-Maker_Camera_"
                                + row.serials().get(i)
                                + "-video-index0")));
      }
      String checklist = CompetitionChecklist.checklist(robot("comp", cameras));
      for (String tail : row.tails()) {
        assertThat(checklist).as(row.what()).contains("| `" + tail + "` |");
      }
    }
    String piped =
        CompetitionChecklist.checklist(
            robot("comp", List.of(station("front|left", 0).holds(CAMERAS.get(0).camera()))));
    assertThat(piped).contains("| front\\|left | 1 |");
  }
}

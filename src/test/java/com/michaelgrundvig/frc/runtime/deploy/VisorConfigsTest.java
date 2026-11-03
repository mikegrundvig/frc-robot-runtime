package com.michaelgrundvig.frc.runtime.deploy;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Microseconds;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.world.bridges.RobotCatalog;
import com.michaelgrundvig.frc.world.bridges.RobotDescription;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Visor's configuration for each coprocessor, from the guide's robot's four cameras put on Visor as
 * a team would: two on one board, one on another, one left on PhotonVision.
 */
class VisorConfigsTest {
  private static final List<CameraConfig> GUIDE = ExampleRobot.SIM.definition().cameras();

  private static Software.Lens lensFor(CameraConfig camera) {
    return Software.Lens.builder()
        .width(camera.sensor().widthPixels())
        .height(camera.sensor().heightPixels())
        .fx(905.25)
        .fy(905.5)
        .cx(641.25)
        .cy(398.75)
        .distortion(List.of(-0.3125, 0.125, 0.0, 0.0, -0.03))
        .build();
  }

  private static CameraConfig on(CameraConfig camera, Software software) {
    return CameraConfigBuilder.builder(camera).software(software).build();
  }

  private static Software.Visor visor(String board, String device) {
    return Software.visor().coprocessor(board).device(device).build();
  }

  /** The guide's cameras: 0 and 1 on vision-a, 2 on PhotonVision, 3 on vision-b, set up fully. */
  private static List<CameraConfig> cameras() {
    List<CameraConfig> cameras = new ArrayList<>(GUIDE);
    cameras.set(
        0, on(GUIDE.get(0), visor("vision-a", "/dev/v4l/by-id/usb-front-left-video-index0")));
    cameras.set(
        1, on(GUIDE.get(1), visor("vision-a", "/dev/v4l/by-id/usb-front-right-video-index0")));
    cameras.set(
        3,
        on(
            GUIDE.get(3),
            Software.visor()
                .coprocessor("vision-b")
                .device("/dev/v4l/by-id/usb-back-video-index0")
                .controls(
                    List.of(
                        Software.Control.of("auto_exposure", 1),
                        Software.Control.of("exposure_time_absolute", 10)))
                .timeOffset(Microseconds.of(-1500))
                .lens(lensFor(GUIDE.get(3)))
                .build()));
    return cameras;
  }

  @Test
  void eachBoardGetsItsCamerasAtTheirPlacesInTheRobotsList() {
    assertThat(GUIDE).hasSize(4);
    Map<String, String> files = VisorConfigs.of(cameras(), 1234, Set.of("vision-a", "vision-b"));

    assertThat(files).containsOnlyKeys("vision-a", "vision-b");
    String a = Objects.requireNonNull(files.get("vision-a"));
    assertThat(a).startsWith("# Visor's configuration").contains("team = 1234\n");
    assertThat(a.split("\\[\\[camera]]")).hasSize(3);
    assertThat(a)
        .contains("name = \"" + GUIDE.get(0).name() + "\"\nindex = 0\n")
        .contains("name = \"" + GUIDE.get(1).name() + "\"\nindex = 1\n")
        .contains("device = \"/dev/v4l/by-id/usb-front-left-video-index0\"")
        .contains("width = " + GUIDE.get(0).sensor().widthPixels() + "\n")
        .contains("fps = " + Math.round(GUIDE.get(0).sensor().frameRate().baseUnitMagnitude()))
        .doesNotContain("[camera.lens]")
        .doesNotContain("[camera.controls]");

    String b = Objects.requireNonNull(files.get("vision-b"));
    assertThat(b)
        .contains("name = \"" + GUIDE.get(3).name() + "\"\nindex = 3\n")
        .contains("time_offset_us = -1500\n")
        .contains("[camera.controls]\n\"auto_exposure\" = 1\n\"exposure_time_absolute\" = 10\n")
        .contains("[camera.lens]\nwidth = " + GUIDE.get(3).sensor().widthPixels())
        .contains("fx = 905.25\nfy = 905.5\ncx = 641.25\ncy = 398.75\n")
        .contains("distortion = [-0.3125, 0.125, 0.0, 0.0, -0.03]\n");
  }

  @Test
  void aCameraTheDeployCantPlaceIsNamed() {
    record Case(String why, List<CameraConfig> cameras, String says) {}
    List<CameraConfig> noBoard = new ArrayList<>(GUIDE);
    noBoard.set(2, on(GUIDE.get(2), Software.visor().device("/dev/video0").build()));
    List<CameraConfig> noDevice = new ArrayList<>(GUIDE);
    noDevice.set(2, on(GUIDE.get(2), Software.visor().coprocessor("vision-a").build()));
    List<CameraConfig> elsewhere = new ArrayList<>(GUIDE);
    elsewhere.set(2, on(GUIDE.get(2), visor("vision-c", "/dev/video0")));
    for (Case c :
        List.of(
            new Case("no board", noBoard, "doesn't say which coprocessor it's on"),
            new Case("no device", noDevice, "doesn't say its device"),
            new Case(
                "a board the build doesn't name",
                elsewhere,
                "is on the coprocessor vision-c, which the build's visor block doesn't name (it names vision-a, vision-b)"))) {
      assertThatIllegalArgumentException()
          .as(c.why())
          .isThrownBy(() -> VisorConfigs.of(c.cameras(), 1234, Set.of("vision-b", "vision-a")))
          .withMessageStartingWith(GUIDE.get(2).name() + " ")
          .withMessageContaining(c.says());
    }
  }

  /** A team's robots, by name: on Visor fully, on Visor in the simulator only, or not at all. */
  private static RobotCatalog catalog(String... robots) {
    List<RobotDescription> described = new ArrayList<>();
    for (String robot : robots) {
      List<CameraConfig> cameras =
          switch (robot.split("-")[0]) {
            case "visor" -> cameras();
            case "sim" -> List.of(on(GUIDE.get(0), Software.VISOR));
            default -> GUIDE;
          };
      RobotDescription guide = new ExampleCatalog().all().get(0);
      described.add(
          new RobotDescription(robot, guide.body(), guide.drivetrain(), cameras, Optional.empty()));
    }
    return () -> described;
  }

  @Test
  void theRobotToDeployToIsTheOneNamedOrTheOnlyOneOnVisor() {
    assertThat(
            VisorConfigs.robot(
                catalog("visor-competition", "sim-only", "photon"), Optional.empty()))
        .map(RobotDescription::name)
        .contains("visor-competition");
    assertThat(VisorConfigs.robot(catalog("sim-only", "photon"), Optional.empty())).isEmpty();
    assertThat(
            VisorConfigs.robot(
                catalog("visor-competition", "visor-practice"), Optional.of("visor-practice")))
        .map(RobotDescription::name)
        .contains("visor-practice");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                VisorConfigs.robot(
                    catalog("visor-competition", "visor-practice"), Optional.empty()))
        .withMessage(
            "Several robots run Visor (visor-competition, visor-practice): name the one to deploy to, -Probot=NAME");
  }

  @TempDir Path out;

  @Test
  void writingReplacesWhatAnEarlierDeployLeft() throws IOException {
    Files.createDirectories(out.resolve("vision-old"));
    Files.writeString(out.resolve("vision-old/visor.toml"), "stale");
    RobotDescription robot = catalog("visor-competition").all().get(0);

    VisorConfigs.write(
        Optional.of(robot), 1234, Set.of("vision-a", "vision-b", "vision-spare"), out);

    assertThat(out.resolve("vision-a/visor.toml")).content().contains("index = 0");
    assertThat(out.resolve("vision-b/visor.toml")).content().contains("index = 3");
    assertThat(out.resolve("vision-spare")).doesNotExist();
    assertThat(out.resolve("vision-old")).doesNotExist();

    VisorConfigs.write(Optional.empty(), 1234, Set.of("vision-a"), out);
    try (var left = Files.list(out)) {
      assertThat(left).isEmpty();
    }
  }

  /** The guide's robot, as a team's catalog describes it. */
  static final class ExampleCatalog implements RobotCatalog {
    @Override
    public List<RobotDescription> all() {
      var definition = ExampleRobot.SIM.definition();
      return List.of(
          new RobotDescription(
              definition.name(),
              definition.body(),
              definition.drivetrain(),
              definition.cameras(),
              Optional.empty()));
    }
  }
}

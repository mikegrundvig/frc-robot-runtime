package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;

/**
 * Which board each Visor camera is plugged into, from two boards' reports as they change: the board
 * each camera's frames must come from.
 */
class VisorPlacementsTest {
  private static final String FRONT = "usb-Thrifty_Cam_0001-video-index0";
  private static final String BACK = "usb-Thrifty_Cam_0002-video-index0";

  /** A board's report as Spotter last had it. */
  private static final class Board implements VisorPlacements.Reporting {
    final String address;
    long changes;
    Optional<String> plugged = Optional.empty();

    Board(String address) {
      this.address = address;
    }

    void says(String devices) {
      plugged = Optional.of(devices);
      changes++;
    }

    void cantSay() {
      plugged = Optional.empty();
      changes++;
    }

    @Override
    public String address() {
      return address;
    }

    @Override
    public long changes() {
      return changes;
    }

    @Override
    public Optional<String> plugged() {
      return plugged;
    }
  }

  /** The guide's cameras: front and back on Visor by their devices, one on Visor with none. */
  private static List<CameraConfig> cameras() {
    List<CameraConfig> guide = ExampleRobot.SIM.definition().cameras();
    List<CameraConfig> cameras = new ArrayList<>();
    cameras.add(on(guide.get(0), Software.visor().device("/dev/v4l/by-id/" + FRONT)));
    cameras.add(on(guide.get(1), Software.visor().device("/dev/v4l/by-id/" + BACK)));
    cameras.add(on(guide.get(2), Software.VISOR));
    return cameras;
  }

  private static CameraConfig on(CameraConfig camera, Software software) {
    return camera.withSoftware(software);
  }

  /**
   * What the boards say, loop by loop (each read before the next), then where each camera, by its
   * index, is placed.
   */
  private record Row(
      String name, List<Consumer<List<Board>>> loops, List<Optional<String>> placed) {}

  @Test
  void eachCameraIsOnTheBoardThatReportsIt() {
    Optional<String> none = Optional.empty();
    Optional<String> left = Optional.of("10.12.34.11");
    Optional<String> right = Optional.of("10.12.34.12");
    List<Row> rows =
        List.of(
            new Row("before any report", List.of(), List.of(none, none, none)),
            new Row(
                "each on its board, the agent's port left off",
                List.of(
                    boards -> {
                      boards.get(0).says(FRONT);
                      boards.get(1).says(BACK + ",usb-Other-video-index0");
                    }),
                List.of(left, right, none)),
            new Row(
                "a board out of reach keeps what it said",
                List.of(
                    boards -> boards.get(0).says(FRONT + "," + BACK),
                    boards -> boards.get(0).cantSay()),
                List.of(left, left, none)),
            new Row(
                "a camera moved to the other board",
                List.of(
                    boards -> {
                      boards.get(0).says(FRONT);
                      boards.get(1).says(BACK);
                    },
                    boards -> {
                      boards.get(0).says("");
                      boards.get(1).says(FRONT + "," + BACK);
                    }),
                List.of(right, right, none)),
            new Row(
                "a camera unplugged is on no board",
                List.of(boards -> boards.get(0).says(FRONT), boards -> boards.get(0).says("")),
                List.of(none, none, none)));
    for (Row row : rows) {
      List<Board> boards = List.of(new Board("10.12.34.11"), new Board("10.12.34.12:5808"));
      VisorPlacements placements =
          new VisorPlacements(cameras(), boards, VisorPlacements.Configs.UNKNOWN);
      for (Consumer<List<Board>> loop : row.loops()) {
        loop.accept(boards);
        placements.placed(0);
      }
      List<Optional<String>> placed = new ArrayList<>();
      for (int camera = 0; camera < 3; camera++) {
        placed.add(placements.placed(camera));
      }
      assertThat(placed).as(row.name()).isEqualTo(row.placed());
      assertThat(placements.placed(7)).as(row.name() + ": an index no camera has").isEmpty();
    }
  }

  /** The feed asks every loop and looks again only at a changed value: the same one while held. */
  @Test
  void aPlacementThatHoldsIsTheSameValue() {
    Board board = new Board("10.12.34.11");
    VisorPlacements placements =
        new VisorPlacements(cameras(), List.of(board), VisorPlacements.Configs.UNKNOWN);
    board.says(FRONT);
    Optional<String> first = placements.placed(0);
    board.says(FRONT + "," + BACK);
    assertThat(placements.placed(0)).isSameAs(first);
  }
}

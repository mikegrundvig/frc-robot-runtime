package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.runtime.subsystems.vision.VisorBoards;
import com.michaelgrundvig.frc.spotter.manager.Board;
import com.michaelgrundvig.frc.spotter.manager.Value;
import com.michaelgrundvig.frc.visor.VisorPack;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Which board each of the robot's Visor cameras is plugged into, by the camera's index, as the
 * boards last reported through Visor's Spotter pack ({@link VisorPack#CAMERAS}): the board the
 * installer configures Visor on for it ({@link VisorConfigs#forBoard}), so the one its frames must
 * come from. Each board is given by its address as the robot's configuration names it, without its
 * agent's port.
 *
 * <p>A board's report is read again only when Spotter says one of its values changed, and a board
 * that can't say right now (it's out of reach) keeps what it said last: a board that's away for a
 * moment doesn't lose its cameras. A camera none of the boards reports is on none, and so is one
 * that names no device.
 *
 * <p>Which configurations each board's frames may come from is the installer's, which gave them
 * ({@link VisorInstaller#otherConfig}).
 *
 * <p>It's asked every loop, so it hands back the same value while nothing changed and allocates
 * nothing then.
 */
final class VisorPlacements implements VisorBoards {
  /** A board as it's read here. */
  interface Reporting {
    /** Its address, as the robot's configuration gives it: {@code 10.12.34.11}, maybe a port. */
    String address();

    /** A count that changes whenever one of its values does. */
    long changes();

    /** The devices it reports plugged in, comma separated; empty while it can't say. */
    Optional<String> plugged();
  }

  /** Which configurations a board's frames may come from, by their IDs. */
  interface Configs {
    /** None known: a frame's configuration is never refused. */
    Configs UNKNOWN = (board, configId) -> false;

    /** Whether a frame from this board, naming this configuration, is known to be from another. */
    boolean other(String board, int configId);
  }

  /** Each camera's device name, by its index: empty for one on PhotonVision or naming none. */
  private final List<Optional<String>> devices = new ArrayList<>();

  private final List<Reporting> boards;
  private final List<String> listed;
  private final long[] changesSeen;
  private final Configs configs;

  /** What each board said last is plugged into it. */
  private final List<List<String>> plugged = new ArrayList<>();

  /** Each camera's board, by its index. */
  private final List<Optional<String>> placed = new ArrayList<>();

  VisorPlacements(List<CameraConfig> cameras, List<? extends Reporting> boards, Configs configs) {
    this.configs = configs;
    for (CameraConfig camera : cameras) {
      Optional<String> device = Optional.empty();
      if (camera.software() instanceof CameraConfig.Software.Visor visor) {
        device = visor.device().map(VisorConfigs::deviceName);
      }
      devices.add(device);
      placed.add(Optional.empty());
    }
    this.boards = List.copyOf(boards);
    listed = this.boards.stream().map(board -> withoutPort(board.address())).toList();
    changesSeen = new long[boards.size()];
    Arrays.fill(changesSeen, Long.MIN_VALUE);
    for (int i = 0; i < boards.size(); i++) {
      plugged.add(List.of());
    }
  }

  /** A Spotter board, read through Visor's pack. */
  static Reporting of(Board board) {
    Value cameras = board.value(VisorPack.CAMERAS);
    return new Reporting() {
      @Override
      public String address() {
        return board.address();
      }

      @Override
      public long changes() {
        return board.changes();
      }

      @Override
      public Optional<String> plugged() {
        if (!cameras.available()) {
          return Optional.empty();
        }
        return Optional.of(cameras.text());
      }
    };
  }

  /** Each board's address, as the robot's configuration gives it, without the agent's port. */
  @Override
  public List<String> listed() {
    return listed;
  }

  @Override
  public boolean otherConfig(String board, int configId) {
    return configs.other(board, configId);
  }

  /** The address of the board a camera is plugged into, or empty until a board reports it. */
  @Override
  public Optional<String> placed(int camera) {
    readWhatChanged();
    if (camera < 0 || camera >= placed.size()) {
      return Optional.empty();
    }
    return placed.get(camera);
  }

  private void readWhatChanged() {
    boolean changed = false;
    for (int i = 0; i < boards.size(); i++) {
      Reporting board = boards.get(i);
      long changes = board.changes();
      if (changes == changesSeen[i]) {
        continue;
      }
      changesSeen[i] = changes;
      Optional<String> said = board.plugged();
      if (said.isEmpty()) {
        continue;
      }
      plugged.set(i, said.get().isEmpty() ? List.of() : List.of(said.get().split(",")));
      changed = true;
    }
    if (changed) {
      placeEach();
    }
  }

  /** Places each camera on the first board reporting its device, keeping a placement that held. */
  private void placeEach() {
    for (int camera = 0; camera < devices.size(); camera++) {
      Optional<String> board = boardWith(devices.get(camera));
      if (!board.equals(placed.get(camera))) {
        placed.set(camera, board);
      }
    }
  }

  private Optional<String> boardWith(Optional<String> device) {
    if (device.isEmpty()) {
      return Optional.empty();
    }
    for (int i = 0; i < boards.size(); i++) {
      if (plugged.get(i).contains(device.get())) {
        return Optional.of(withoutPort(boards.get(i).address()));
      }
    }
    return Optional.empty();
  }

  /** An address without the agent's port: {@code 10.12.34.11:5808} is {@code 10.12.34.11}. */
  static String withoutPort(String address) {
    int colon = address.indexOf(':');
    boolean onePort = colon >= 0 && colon == address.lastIndexOf(':');
    if (!onePort) {
      return address;
    }
    return address.substring(0, colon);
  }
}

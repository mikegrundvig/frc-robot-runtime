package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.runtime.subsystems.vision.VisorBoards;
import com.michaelgrundvig.frc.spotter.manager.BoardDefinition;
import com.michaelgrundvig.frc.spotter.manager.Manager;
import com.michaelgrundvig.frc.spotter.manager.RobotSignals;
import com.michaelgrundvig.frc.spotter.manager.Settings;
import com.michaelgrundvig.frc.spotter.manager.packs.TeamPacks;
import com.michaelgrundvig.frc.visor.VisorPack;
import com.michaelgrundvig.frc.visor.VisorVersion;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.util.Alert;

/**
 * The robot's coprocessors, each by its Spotter agent's address (the robot configuration's {@code
 * coprocessors}): Spotter's manager, which watches them and pushes each Visor's pack from the
 * deploy folder ({@code spotter-packs/visor}) while any camera is on Visor, its alerts shown as the
 * robot's own ({@link SpotterAlerts}), and the Visor installer, which brings each board with
 * Visor's pack up to date. On a robot only: in the simulator and a replay there are none.
 *
 * <p>Visor's configuration finds the robot by its team number, so with none set on Systemcore (team
 * 0), Visor is neither installed nor configured, and an alert says why: its frames would go to
 * 10.0.0.2, and a board's good configuration would be replaced by that one.
 */
public final class Coprocessors implements AutoCloseable {
  /** The writes of what's kept, on one thread of their own, in order: the loop never waits. */
  private static final ExecutorService WRITES =
      Executors.newSingleThreadExecutor(write -> {
        Thread thread = new Thread(write, "Visor memory");
        thread.setDaemon(true);
        return thread;
      });

  private final State state;

  private Coprocessors(State state) {
    this.state = state;
  }

  /** Where the robot's coprocessors are: none, watched with Visor left alone, or with Visor. */
  private sealed interface State {
    void periodic();

    void close();
  }

  /** None: the simulator's, a replay's, and a robot's that lists no coprocessors. */
  private record None() implements State {
    @Override
    public void periodic() {}

    @Override
    public void close() {}
  }

  /** The boards, watched through Spotter: what each of the other states has. */
  private record Watched(Manager manager, SpotterRecorder recorder, SpotterAlerts alerts) {
    void periodic() {
      manager.update();
      alerts.periodic();
    }

    void close() {
      alerts.close();
      manager.close();
    }
  }

  /** Watched, with no team number set: Visor is left alone, and an alert says why. */
  private record TeamMissing(Watched watched, Alert noTeam) implements State {
    @Override
    public void periodic() {
      watched.periodic();
    }

    @Override
    public void close() {
      noTeam.close();
      watched.close();
    }
  }

  /** Watched, with Visor installed and configured on them. */
  private record Running(Watched watched, VisorInstaller visor) implements State {
    @Override
    public void periodic() {
      watched.periodic();
      visor.periodic();
    }

    @Override
    public void close() {
      visor.close();
      watched.close();
    }
  }

  /** None: the simulator's, a replay's, and a robot's that lists no coprocessors. */
  public static Coprocessors none() {
    return new Coprocessors(new None());
  }

  /**
   * A robot's: its boards watched, and Visor installed on them. Started at once, so boards connect
   * and take their packs while the program boots, disabled.
   *
   * @param addresses the robot's coprocessors, each its Spotter agent's address
   * @param cameras the robot's cameras, as committed and as tuned now ({@link VisorTuning#live}):
   *     those on Visor are configured on whichever board they're plugged into, with their exposure
   *     and gain as tuned
   * @param deploy the deploy folder: Visor's pack in {@code spotter-packs/}, and this robot's Visor
   *     release in {@code visor/}
   * @param team the team number set on Systemcore, for Visor's configuration; 0 if none is
   * @param tagSizeMeters the side of the field's tags' black square, for Visor's configuration
   * @param kept where the robot keeps, across its program's restarts, which configurations each
   *     board's Visor frames are taken from ({@link VisorMemory}): on a robot, its disk's {@code
   *     /home/systemcore/visor/}
   * @param enabled whether the robot is enabled
   * @param fieldAttached whether the field is attached
   * @param nanos the robot's clock
   */
  public static Coprocessors on(
      List<String> addresses,
      VisorTuning.Live cameras,
      Path deploy,
      int team,
      double tagSizeMeters,
      Path kept,
      BooleanSupplier enabled,
      BooleanSupplier fieldAttached,
      LongSupplier nanos) {
    return withBoards(
        definitions(addresses, cameras.committed()),
        cameras,
        deploy,
        team,
        tagSizeMeters,
        VisorVersion.RELEASE_SHA256,
        new VisorMemoryDisk(kept, Coprocessors::oneWriteAtATime),
        enabled,
        fieldAttached,
        nanos);
  }

  /** The folder a robot keeps Visor's configurations in, across restarts, on Systemcore. */
  public static Path robotMemoryFolder() {
    return VisorMemoryDisk.ROBOT_FOLDER;
  }

  private static void oneWriteAtATime(Runnable write) {
    WRITES.execute(write);
  }

  /**
   * Each board's definition for Spotter's manager: its address, and Visor's pack while any of the
   * robot's cameras is on Visor; none otherwise, so a robot that leaves Visor has it taken away.
   */
  static List<BoardDefinition> definitions(List<String> addresses, List<CameraConfig> cameras) {
    List<String> packs = onVisor(cameras) ? List.of(VisorPack.NAME) : List.of();
    return addresses.stream().map(address -> new BoardDefinition(address, packs)).toList();
  }

  private static boolean onVisor(List<CameraConfig> cameras) {
    return cameras.stream()
        .anyMatch(camera -> camera.software() instanceof CameraConfig.Software.Visor);
  }

  /**
   * As {@link #on}, with each board's packs given, the SHA-256 the release in the deploy must have,
   * and where what's taken is kept: for a test whose board needs a pack its image would have, or
   * whose release is its own.
   */
  static Coprocessors withBoards(
      List<BoardDefinition> boards,
      VisorTuning.Live cameras,
      Path deploy,
      int team,
      double tagSizeMeters,
      String releaseSha256,
      VisorMemory memory,
      BooleanSupplier enabled,
      BooleanSupplier fieldAttached,
      LongSupplier nanos) {
    if (boards.isEmpty()) {
      return none();
    }
    SpotterRecorder recorder =
        new SpotterRecorder(boards.stream().map(BoardDefinition::address).toList());
    Path packs = deploy.resolve("spotter-packs");
    Manager manager =
        new Manager(
            new RobotSignals(enabled, fieldAttached, nanos),
            boards,
            Settings.DEFAULTS.withPacks(packs),
            // Each board's connects and pushes, for its health; its values are read in the loop.
            recorder);
    manager.start();
    Watched watched = new Watched(manager, recorder, new SpotterAlerts(manager::alerts));
    if (team <= 0 && onVisor(cameras.committed())) {
      Alert noTeam =
          new Alert(
              "Visor",
              "noTeam",
              "Systemcore has no team number, so Visor isn't installed or configured on the"
                  + " coprocessors (its frames would go to 10.0.0.2): set it on Systemcore's page,"
                  + " http://robot.local, Change Team Number, then restart the robot program",
              Alert.Level.HIGH);
      noTeam.set(true);
      return new Coprocessors(new TeamMissing(watched, noTeam));
    }
    Path release = deploy.resolve("visor").resolve(VisorPack.releaseName(VisorVersion.VERSION));
    TeamPacks teamPacks = teamPacks(packs);
    VisorInstaller visor =
        new VisorInstaller(
            cameras,
            cameras.committed(),
            team,
            tagSizeMeters,
            VisorVersion.VERSION,
            releaseSha256,
            () -> Files.readAllBytes(release),
            Coprocessors::offTheLoop,
            enabled,
            fieldAttached,
            nanos,
            manager.boards().stream()
                .map(board ->
                    new SpotterBoards(
                        board,
                        recorder.counts(board.address()),
                        packsHash(teamPacks, board.definition())))
                .toList(),
            memory);
    return new Coprocessors(new Running(watched, visor));
  }

  /** The team's packs, as Spotter's manager reads them as it's made; null when it can't. */
  private static @Nullable TeamPacks teamPacks(Path folder) {
    if (!Files.isDirectory(folder)) {
      return null;
    }
    try {
      return TeamPacks.of(folder);
    } catch (IOException | RuntimeException e) {
      return null; // Spotter's manager says so, by its alert, and pushes none
    }
  }

  /** The hash of a board's packs from the robot, as its description gives its own; null: none. */
  private static @Nullable String packsHash(@Nullable TeamPacks teamPacks, BoardDefinition board) {
    if (teamPacks == null || board.packs().isEmpty()) {
      return null;
    }
    return teamPacks.select(board.packs()).hash();
  }

  /** Runs a read of the release on a thread of its own, so the loop never waits on the disk. */
  private static void offTheLoop(Runnable read) {
    Thread thread = new Thread(read, "Visor release");
    thread.setDaemon(true);
    thread.start();
  }

  /**
   * Each board's health for the log and Pitwall, read from Spotter's manager each loop: none
   * without boards.
   */
  public CoprocessorHealth health() {
    Watched watched = watched();
    if (watched == null) {
      return CoprocessorHealth.none();
    }
    Manager manager = watched.manager();
    List<String> addresses = manager.boards().stream().map(board -> board.address()).toList();
    return CoprocessorHealth.of(
        addresses,
        address -> new BoardIOSpotter(manager.board(address), watched.recorder().counts(address)));
  }

  /**
   * The robot's boards, which each of its Visor cameras is plugged into, by its index, as the
   * boards report it, and which configurations each board's frames may come from, as the installer
   * gave them: what Visor's frames are checked against ({@link VisorPlacements}). None without
   * boards, so frames are taken from anyone; with Visor left alone (no team number), any
   * configuration.
   *
   * @param cameras the robot's cameras, in its definition's order
   */
  public VisorBoards visorBoards(List<CameraConfig> cameras) {
    Watched watched = watched();
    if (watched == null) {
      return VisorBoards.none();
    }
    VisorPlacements.Configs configs = VisorPlacements.Configs.UNKNOWN;
    if (state instanceof Running running) {
      configs = running.visor()::otherConfig;
    }
    return new VisorPlacements(
        cameras, watched.manager().boards().stream().map(VisorPlacements::of).toList(), configs);
  }

  private @Nullable Watched watched() {
    return switch (state) {
      case None none -> null;
      case TeamMissing missing -> missing.watched();
      case Running running -> running.watched();
    };
  }

  /** Once each loop, first: the boards' state as last heard, its alerts, then what Visor needs. */
  public void periodic() {
    state.periodic();
  }

  @Override
  public void close() {
    state.close();
  }
}

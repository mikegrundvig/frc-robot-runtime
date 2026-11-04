package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.spotter.manager.BoardDefinition;
import com.michaelgrundvig.frc.spotter.manager.Manager;
import com.michaelgrundvig.frc.spotter.manager.Recorder;
import com.michaelgrundvig.frc.spotter.manager.RobotSignals;
import com.michaelgrundvig.frc.spotter.manager.Settings;
import com.michaelgrundvig.frc.visor.VisorVersion;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.util.Alert;

/**
 * The robot's coprocessors, each by its Spotter agent (the robot configuration's {@code
 * coprocessors}): Spotter's manager, which watches them and pushes them the team's packs from the
 * deploy folder ({@code spotter-packs/}, where Visor's pack is), its alerts shown as the robot's
 * own ({@link SpotterAlerts}), and the Visor installer, which brings each board that has the
 * robot's Visor cameras up to date. On a robot only: in the simulator and a replay there are none.
 *
 * <p>Visor's configuration finds the robot by its team number, so with none set on Systemcore (team
 * 0), Visor is neither installed nor configured, and an alert says why: its frames would go to
 * 10.0.0.2, and a board's good configuration would be replaced by that one.
 */
public final class Coprocessors implements AutoCloseable {
  private final @Nullable Manager manager;
  private final @Nullable SpotterAlerts alerts;
  private final @Nullable VisorInstaller visor;
  private final @Nullable Alert noTeam;

  private Coprocessors(
      @Nullable Manager manager,
      @Nullable SpotterAlerts alerts,
      @Nullable VisorInstaller visor,
      @Nullable Alert noTeam) {
    this.manager = manager;
    this.alerts = alerts;
    this.visor = visor;
    this.noTeam = noTeam;
  }

  /** None: the simulator's, a replay's, and a robot's that lists no coprocessors. */
  public static Coprocessors none() {
    return new Coprocessors(null, null, null, null);
  }

  /**
   * A robot's: its boards watched, and Visor installed on them. Started at once, so boards connect
   * and take their packs while the program boots, disabled.
   *
   * @param boards the robot's coprocessors, each its agent's address and the team's packs it's to
   *     have
   * @param cameras the robot's cameras, in its definition's order: those on Visor are configured on
   *     whichever board they're plugged into
   * @param deploy the deploy folder: the team's packs in {@code spotter-packs/}, and this robot's
   *     Visor release in {@code visor/}
   * @param team the team number set on Systemcore, for Visor's configuration; 0 if none is
   * @param enabled whether the robot is enabled
   * @param fieldAttached whether the field is attached
   * @param nanos the robot's clock
   */
  public static Coprocessors on(
      List<BoardDefinition> boards,
      List<CameraConfig> cameras,
      Path deploy,
      int team,
      BooleanSupplier enabled,
      BooleanSupplier fieldAttached,
      LongSupplier nanos) {
    if (boards.isEmpty()) {
      return none();
    }
    Manager manager =
        new Manager(
            new RobotSignals(enabled, fieldAttached, nanos),
            boards,
            Settings.DEFAULTS.withPacks(deploy.resolve("spotter-packs")),
            Recorder.NONE);
    manager.start();
    SpotterAlerts alerts = new SpotterAlerts(manager::alerts);
    boolean onVisor =
        cameras.stream()
            .anyMatch(camera -> camera.software() instanceof CameraConfig.Software.Visor);
    if (team <= 0 && onVisor) {
      Alert noTeam =
          new Alert(
              "Visor",
              "noTeam",
              "Systemcore has no team number, so Visor isn't installed or configured on the"
                  + " coprocessors (its frames would go to 10.0.0.2): set it on Systemcore's page,"
                  + " http://robot.local, Change Team Number, then restart the robot program",
              Alert.Level.HIGH);
      noTeam.set(true);
      return new Coprocessors(manager, alerts, null, noTeam);
    }
    Path release = deploy.resolve("visor").resolve(releaseName(VisorVersion.VERSION));
    VisorInstaller visor =
        new VisorInstaller(
            cameras,
            team,
            VisorVersion.VERSION,
            () -> Files.readAllBytes(release),
            () -> !enabled.getAsBoolean() && !fieldAttached.getAsBoolean(),
            nanos,
            manager.boards().stream().map(SpotterBoards::new).toList());
    return new Coprocessors(manager, alerts, visor, null);
  }

  /** A Visor release's archive, as its release workflow names it. */
  public static String releaseName(String version) {
    return "visor-" + version + "-linux-arm64.tar.gz";
  }

  /** Once each loop, first: the boards' state as last heard, its alerts, then what Visor needs. */
  public void periodic() {
    if (manager != null) {
      manager.update();
    }
    if (alerts != null) {
      alerts.periodic();
    }
    if (visor != null) {
      visor.periodic();
    }
  }

  @Override
  public void close() {
    if (visor != null) {
      visor.close();
    }
    if (alerts != null) {
      alerts.close();
    }
    if (noTeam != null) {
      noTeam.close();
    }
    if (manager != null) {
      manager.close();
    }
  }
}

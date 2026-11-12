package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.runtime.lib.health.BoardHealth;
import com.michaelgrundvig.frc.spotter.manager.Connection;
import com.michaelgrundvig.frc.spotter.manager.Level;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * Each coprocessor's health in the robot's log: what its Spotter agent reports of it, as inputs
 * under {@code Coprocessors/<address>} (its CPU, throttling, heat, USB and uptime, from the {@code
 * debian} pack, and whether it's reached), so a replay reads exactly what the robot did; and what
 * that adds up to, under {@code Coprocessors/<address>/Health}: missing, throttled, near slowing
 * for heat now, and its dropouts, reboots and USB disconnects this session ({@link BoardHealth}).
 * Pitwall's Pit view shows each as a card ({@link BoardPit}).
 *
 * <p>The values are read from Spotter's manager in the robot loop, after its update, not taken
 * through its {@code Recorder} on its own threads: AdvantageKit's log is the loop's alone, and a
 * value read in the loop is one the loop's log and its replay agree on. The recorder counts only
 * what the boards' state doesn't hold, each agent's connects and the packs pushed to it ({@link
 * SpotterRecorder}), which the loop reads in turn.
 */
public final class CoprocessorHealth {
  /** One board's. */
  static final class Watched {
    final String address;
    final BoardIO io;
    final BoardIOInputsAutoLogged inputs = new BoardIOInputsAutoLogged();
    final BoardHealth health = new BoardHealth();
    private final String key;
    private final String missingKey;
    private final String dropoutsKey;
    private final String throttledKey;
    private final String hotKey;
    private final String rebootsKey;
    private final String packRestartsKey;
    private final String disconnectsKey;

    Watched(String address, BoardIO io) {
      this.address = address;
      this.io = io;
      key = "Coprocessors/" + address;
      missingKey = key + "/Health/Missing";
      dropoutsKey = key + "/Health/Dropouts";
      throttledKey = key + "/Health/Throttled";
      hotKey = key + "/Health/LowThermalMargin";
      rebootsKey = key + "/Health/Reboots";
      packRestartsKey = key + "/Health/PackRestarts";
      disconnectsKey = key + "/Health/UsbDisconnectsThisSession";
    }

    void periodic(long nowNanos) {
      io.updateInputs(inputs);
      Logger.processInputs(key, inputs);
      health.update(
          nowNanos,
          new BoardHealth.Reading(
              connection(inputs.connection),
              inputs.cpuCapped,
              level(inputs.thermalMarginLevel),
              inputs.uptimeSeconds,
              inputs.usbDisconnects,
              inputs.packPushesSinceDescribed));
      Logger.recordOutput(missingKey, health.missing());
      Logger.recordOutput(dropoutsKey, health.dropouts().count());
      Logger.recordOutput(throttledKey, health.throttled().holds());
      Logger.recordOutput(hotKey, health.lowThermalMargin().holds());
      Logger.recordOutput(rebootsKey, health.reboots());
      Logger.recordOutput(packRestartsKey, health.packRestarts());
      Logger.recordOutput(disconnectsKey, health.usbDisconnects());
    }
  }

  /** Spotter's word for whether a board is reached, as the runtime's health judges it. */
  static BoardHealth.Connection connection(Connection spotters) {
    return switch (spotters) {
      case CONNECTING -> BoardHealth.Connection.CONNECTING;
      case CONNECTED -> BoardHealth.Connection.CONNECTED;
      case MISSING -> BoardHealth.Connection.MISSING;
      case OTHER_PROTOCOL -> BoardHealth.Connection.OTHER_PROTOCOL;
    };
  }

  /** Spotter's level for a value, as the runtime's health judges it. */
  static BoardHealth.Level level(Level spotters) {
    return switch (spotters) {
      case OK -> BoardHealth.Level.OK;
      case WARNING -> BoardHealth.Level.WARNING;
      case FAILING -> BoardHealth.Level.FAILING;
      case UNAVAILABLE -> BoardHealth.Level.UNAVAILABLE;
    };
  }

  private final List<Watched> boards;

  private CoprocessorHealth(List<Watched> boards) {
    this.boards = List.copyOf(boards);
  }

  /** None: the simulator's, which has no coprocessors. */
  public static CoprocessorHealth none() {
    return new CoprocessorHealth(List.of());
  }

  /**
   * The robot's boards, each read through its IO: Spotter's on a robot ({@link
   * Coprocessors#health}), nothing in a replay ({@link #replayed}).
   */
  static CoprocessorHealth of(List<String> addresses, Function<String, BoardIO> io) {
    List<Watched> watched = new ArrayList<>();
    for (String address : addresses) {
      watched.add(new Watched(address, io.apply(address)));
    }
    return new CoprocessorHealth(watched);
  }

  /**
   * A replay's: the robot's boards, read from the log.
   *
   * @param addresses each board's address, as the robot's configuration gives it
   */
  public static CoprocessorHealth replayed(List<String> addresses) {
    return of(addresses, address -> new BoardIO() {});
  }

  /** Once each loop, after Spotter's manager has updated ({@link Coprocessors#periodic}). */
  public void periodic(long nowNanos) {
    for (Watched board : boards) {
      board.periodic(nowNanos);
    }
  }

  List<Watched> boards() {
    return boards;
  }
}

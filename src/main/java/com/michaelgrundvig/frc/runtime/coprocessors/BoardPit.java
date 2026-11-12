package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.runtime.lib.health.BoardHealth;
import com.michaelgrundvig.frc.runtime.lib.health.Durations;
import com.michaelgrundvig.frc.runtime.lib.health.Episodes;
import com.michaelgrundvig.frc.spotter.manager.Connection;
import com.michaelgrundvig.frc.spotter.manager.Level;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.OptionalLong;

/**
 * Each coprocessor's card on Pitwall's Pit view: whether Spotter reaches it, how long it's been up
 * and whether it rebooted, its busiest core and whether it slowed itself for heat or power, how hot
 * it runs and how far from slowing, and its USB devices and whether one dropped off. What happened
 * this session stays on the card, so a reboot or a throttle during a match shows after it.
 *
 * <p>The card is read in the robot loop, from what the loop already read.
 */
public final class BoardPit {
  private final CoprocessorHealth.Watched board;

  BoardPit(CoprocessorHealth.Watched board) {
    this.board = board;
  }

  /** A card for each of the robot's coprocessors, in its configuration's order. */
  public static List<Devices.Device> devices(CoprocessorHealth health) {
    List<Devices.Device> devices = new ArrayList<>();
    for (CoprocessorHealth.Watched board : health.boards()) {
      BoardPit pit = new BoardPit(board);
      devices.add(Devices.Device.named(deviceName(board.address), "Coprocessor", pit::card));
    }
    return devices;
  }

  /**
   * A board's name on the page, from its address: {@code 10.12.34.11} is {@code 10-12-34-11}. The
   * robot's configuration checks that no other card has it.
   */
  public static String deviceName(String address) {
    String name = address.toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9-]+", "-");
    name = name.replaceAll("^-+", "");
    if (name.isEmpty()) {
      return "board";
    }
    return name.substring(0, Math.min(name.length(), 63));
  }

  /** The card as of now. */
  Devices.Card card() {
    BoardIO.BoardIOInputs inputs = board.inputs;
    BoardHealth health = board.health;
    long now = health.nowNanos();
    List<Devices.Line> lines = new ArrayList<>();
    if (!inputs.hostname.isEmpty() && !inputs.hostname.equals(board.address)) {
      lines.add(new Devices.Line("Host", inputs.hostname, Devices.Level.NOTE));
    }
    lines.add(reachedLine(inputs, health, now));
    if (inputs.connection == Connection.CONNECTED || inputs.connection == Connection.MISSING) {
      lines.add(upLine(inputs, health, now));
      lines.add(cpuLine(inputs, health, now));
      lines.add(heatLine(inputs, health, now));
      lines.add(usbLine(inputs, health, now));
    }
    if (health.packRestarts() > 0) {
      lines.add(
          new Devices.Line(
              "Packs",
              "away "
                  + Durations.times(health.packRestarts())
                  + " this session while its agent restarted with the robot's",
              Devices.Level.NOTE));
    }
    if (inputs.connection == Connection.CONNECTED && inputs.unavailable.length > 0) {
      // Most often a board without Spotter's debian pack, which its image installs.
      lines.add(
          new Devices.Line(
              "Not reported", String.join("; ", inputs.unavailable), Devices.Level.NOTE));
    }
    return Devices.Card.of(state(inputs.connection), Devices.Level.of(lines), lines);
  }

  private static String state(Connection connection) {
    return switch (connection) {
      case CONNECTING -> "Connecting";
      case CONNECTED -> "Connected";
      case MISSING -> "Missing";
      case OTHER_PROTOCOL -> "Another Spotter";
    };
  }

  private static Devices.Line reachedLine(
      BoardIO.BoardIOInputs inputs, BoardHealth health, long now) {
    Episodes dropouts = health.dropouts();
    if (inputs.connection == Connection.OTHER_PROTOCOL) {
      return new Devices.Line("Reached", inputs.why, Devices.Level.PROBLEM);
    }
    if (health.missing() && !health.everReached()) {
      return new Devices.Line("Reached", "not yet: " + inputs.why, Devices.Level.PROBLEM);
    }
    if (health.missing()) {
      return new Devices.Line(
          "Reached",
          "missing " + dropouts.describe(now) + ": " + inputs.why,
          Devices.Level.PROBLEM);
    }
    if (dropouts.count() > 0) {
      return new Devices.Line(
          "Reached", "yes; missing " + dropouts.describe(now), Devices.Level.WARNING);
    }
    if (inputs.connection == Connection.CONNECTING) {
      return new Devices.Line("Reached", "not yet", Devices.Level.NOTE);
    }
    return new Devices.Line("Reached", "yes, all session", Devices.Level.OK);
  }

  private static Devices.Line upLine(BoardIO.BoardIOInputs inputs, BoardHealth health, long now) {
    String up = "unknown";
    if (!Double.isNaN(inputs.uptimeSeconds)) {
      up = Durations.said(inputs.uptimeSeconds);
    }
    OptionalLong rebooted = health.lastRebootNanos();
    if (rebooted.isEmpty()) {
      return new Devices.Line("Up for", up, Devices.Level.OK);
    }
    String ago = Durations.said((now - rebooted.getAsLong()) / 1e9);
    String value =
        up
            + "; rebooted "
            + Durations.times(health.reboots())
            + " this session, last seen "
            + ago
            + " ago";
    return new Devices.Line("Up for", value, Devices.Level.WARNING);
  }

  private static Devices.Line cpuLine(BoardIO.BoardIOInputs inputs, BoardHealth health, long now) {
    String busiest = "unknown";
    if (!Double.isNaN(inputs.cpuBusiestPercent)) {
      busiest = String.format(Locale.ROOT, "%.0f%%", inputs.cpuBusiestPercent);
    }
    String value = busiest + " busiest core";
    Episodes throttled = health.throttled();
    if (throttled.count() == 0) {
      return new Devices.Line("CPU", value + ", full speed", Devices.Level.OK);
    }
    value += "; slowed for heat or power " + throttled.describe(now);
    return new Devices.Line("CPU", value, Devices.Level.WARNING);
  }

  private static Devices.Line heatLine(BoardIO.BoardIOInputs inputs, BoardHealth health, long now) {
    if (Double.isNaN(inputs.hottestCelsius)) {
      return new Devices.Line("Heat", "unknown", Devices.Level.NOTE);
    }
    String value = String.format(Locale.ROOT, "%.0f °C hottest", inputs.hottestCelsius);
    if (!Double.isNaN(inputs.thermalMarginCelsius)) {
      value += String.format(Locale.ROOT, ", %.0f °C from slowing", inputs.thermalMarginCelsius);
    }
    Episodes low = health.lowThermalMargin();
    Devices.Level level = Devices.Level.OK;
    if (low.count() > 0) {
      value += "; near slowing " + low.describe(now);
      level = Devices.Level.WARNING;
    }
    if (low.holds() && inputs.thermalMarginLevel == Level.FAILING) {
      level = Devices.Level.PROBLEM;
    }
    return new Devices.Line("Heat", value, level);
  }

  private static Devices.Line usbLine(BoardIO.BoardIOInputs inputs, BoardHealth health, long now) {
    String devices = inputs.usbDevices;
    if (devices.isEmpty()) {
      devices = "unknown";
    }
    OptionalLong last = health.lastUsbDisconnectNanos();
    if (last.isEmpty()) {
      return new Devices.Line("USB", devices, Devices.Level.OK);
    }
    String ago = Durations.said((now - last.getAsLong()) / 1e9);
    String value =
        devices
            + "; a device dropped off "
            + Durations.times(health.usbDisconnects())
            + " this session, last "
            + ago
            + " ago";
    return new Devices.Line("USB", value, Devices.Level.WARNING);
  }
}

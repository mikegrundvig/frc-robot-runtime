package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.spotter.manager.Board;
import com.michaelgrundvig.frc.spotter.manager.Value;
import java.util.ArrayList;
import java.util.List;

/**
 * A board's health from Spotter's manager, as its last update left it: read in the robot loop,
 * after the manager's update, so it never waits on the network. Each value is looked up once and
 * kept, as Spotter's {@link Board#value} asks. Its agent's connects and the robot's pushes to it
 * come from Spotter's recorder ({@link SpotterRecorder}).
 */
final class BoardIOSpotter implements BoardIO {
  // Spotter's catalog names, as literals until a Spotter release ships them as constants.

  /**
   * {@code debian.uptime.seconds}: seconds since the board booted, which the installer reads too.
   */
  static final String UPTIME = "debian.uptime.seconds";

  private final Board board;
  private final SpotterRecorder.Counts counts;
  private final Value busiest;
  private final Value capped;
  private final Value hottest;
  private final Value margin;
  private final Value disconnects;
  private final Value devices;
  private final Value uptime;
  private final List<Value> read;

  /** The board's change count when its unavailable values were last listed; -1 before then. */
  private long listedAt = -1;

  private String[] unavailable = new String[] {};

  BoardIOSpotter(Board board, SpotterRecorder.Counts counts) {
    this.board = board;
    this.counts = counts;
    busiest = board.value("debian.cpu.busiest");
    capped = board.value("debian.cpu.capped");
    hottest = board.value("debian.thermal.hottest");
    margin = board.value("debian.thermal.margin");
    disconnects = board.value("debian.usb.disconnects");
    devices = board.value("debian.usb.devices");
    uptime = board.value(UPTIME);
    read = List.of(busiest, capped, hottest, margin, disconnects, devices, uptime);
  }

  @Override
  public void updateInputs(BoardIOInputs inputs) {
    inputs.connection = board.connection();
    inputs.why = board.why();
    inputs.hostname = board.name();
    inputs.agentDescribes = counts.describes();
    inputs.packPushes = counts.pushes();
    inputs.packPushesSinceDescribed = counts.pushesSinceDescribed();
    inputs.cpuBusiestPercent = busiest.number();
    inputs.cpuCapped = capped.flag();
    inputs.hottestCelsius = hottest.number();
    inputs.thermalMarginCelsius = margin.number();
    inputs.thermalMarginLevel = margin.level();
    inputs.usbDisconnects = disconnects.number();
    inputs.usbDevices = devices.text();
    inputs.uptimeSeconds = uptime.number();
    // Listed again only when something on the board changed, so a steady loop makes no list.
    if (board.changes() != listedAt) {
      listedAt = board.changes();
      List<String> missing = new ArrayList<>();
      for (Value value : read) {
        if (!value.available()) {
          missing.add(value.id() + ": " + value.unavailable());
        }
      }
      unavailable = missing.toArray(String[]::new);
    }
    inputs.unavailable = unavailable;
  }
}

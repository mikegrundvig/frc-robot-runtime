package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.spotter.manager.Connection;
import com.michaelgrundvig.frc.spotter.manager.Level;
import org.littletonrobotics.junction.AutoLog;

/**
 * One coprocessor's health, as its Spotter agent reports it: through Spotter's manager on a robot
 * ({@link BoardIOSpotter}), or nothing in a replay, where the inputs come from the log. The values
 * are the {@code debian} pack's, which the board's image installs; a board without it has them all
 * unavailable, and {@code unavailable} says why.
 */
public interface BoardIO {
  @AutoLog
  class BoardIOInputs {
    /** Whether Spotter reaches the board's agent. */
    public Connection connection = Connection.CONNECTING;

    /** Why it isn't reached, as Spotter says; empty while it is. */
    public String why = "";

    /** The board's hostname, once it's described itself; until then its address. */
    public String hostname = "";

    /** How many times its agent described itself to the robot this session: once each connect. */
    public long agentDescribes = 0;

    /**
     * How many times it took the robot's packs this session: its agent restarts with them, so it's
     * away for a moment after each.
     */
    public long packPushes = 0;

    /**
     * How many of those came after its agent last described itself: while any did, its agent is
     * about to restart, or restarting, with them.
     */
    public long packPushesSinceDescribed = 0;

    /** {@code debian.cpu.busiest}: the busiest core over half a second, %; NaN when unavailable. */
    public double cpuBusiestPercent = Double.NaN;

    /**
     * {@code debian.cpu.capped}: whether the fastest cores are held below their top frequency, for
     * heat or power; false when unavailable.
     */
    public boolean cpuCapped = false;

    /** {@code debian.thermal.hottest}: the hottest thermal zone, °C; NaN when unavailable. */
    public double hottestCelsius = Double.NaN;

    /**
     * {@code debian.thermal.margin}: how far any zone is below its nearest trip point that slows
     * the board, °C; NaN when unavailable.
     */
    public double thermalMarginCelsius = Double.NaN;

    /** Its level by its limits: the pack's warn below 10 °C, fail below 5, unless overridden. */
    public Level thermalMarginLevel = Level.UNAVAILABLE;

    /**
     * {@code debian.usb.disconnects}: the kernel's USB disconnects since the board booted; NaN when
     * unavailable.
     */
    public double usbDisconnects = Double.NaN;

    /** {@code debian.usb.devices}: the USB devices plugged in, by port and product; or empty. */
    public String usbDevices = "";

    /** {@code debian.uptime.seconds}: seconds since the board booted; NaN when unavailable. */
    public double uptimeSeconds = Double.NaN;

    /** Each of those the board can't give, with why, as {@code "id: why"}. */
    public String[] unavailable = new String[] {};
  }

  default void updateInputs(BoardIOInputs inputs) {}
}

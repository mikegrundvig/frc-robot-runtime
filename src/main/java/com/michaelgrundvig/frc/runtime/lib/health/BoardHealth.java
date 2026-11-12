package com.michaelgrundvig.frc.runtime.lib.health;

import java.util.OptionalLong;

/**
 * One coprocessor's health over the session, from what its Spotter agent reports: whether it's
 * reached, whether it rebooted, whether it slowed itself for heat or power, whether it's near
 * slowing for heat, and whether USB devices dropped off it.
 *
 * <p>Nothing here sets a limit of its own. Missing is Spotter's manager's judgement (nothing heard
 * for its missing threshold), and a dropout is missing once it was reached, unless its agent is
 * restarting with packs the robot just pushed it (as Spotter's recorder tells); a low thermal
 * margin is the {@code debian} pack's level for {@code debian.thermal.margin} (a warning below 10
 * °C from slowing, failing below 5, unless the board's pack or robot code's limits say otherwise);
 * throttling is the board's own fact ({@code debian.cpu.capped}). A reboot is its uptime ({@code
 * debian.uptime.seconds}) going back; USB devices dropping off is its disconnect count ({@code
 * debian.usb.disconnects}, since its boot) rising.
 */
public final class BoardHealth {
  /**
   * Whether Spotter reaches the board's agent, in the runtime's own words: the coprocessors' code
   * maps Spotter's onto these, so nothing here depends on Spotter's manager.
   */
  public enum Connection {
    /** Not reached yet this session: the board may still be booting. */
    CONNECTING,
    /** Reached. */
    CONNECTED,
    /** Nothing heard for Spotter's missing threshold. */
    MISSING,
    /** It answers on another protocol than the manager's, so nothing it says is read. */
    OTHER_PROTOCOL
  }

  /** A value's level by the board's limits, in the runtime's own words, as above. */
  public enum Level {
    /** Within its limits. */
    OK,
    /** Past its warning limit. */
    WARNING,
    /** Past its failing limit. */
    FAILING,
    /** The board doesn't give it. */
    UNAVAILABLE
  }

  private final Episodes dropouts = new Episodes();
  private final Episodes throttled = new Episodes();
  private final Episodes lowThermalMargin = new Episodes();
  private long nowNanos;
  private boolean missing;
  private boolean otherProtocol;

  /** Whether Spotter has reached it this session: until then, missing is a board still booting. */
  private boolean everReached;

  /** Whether it's away now because its agent restarts with packs the robot pushed it. */
  private boolean restartingForPacks;

  private int packRestarts;

  /** The last uptime the board gave, in seconds; NaN before it gave one. */
  private double lastUptime = Double.NaN;

  private int reboots;
  private long lastRebootNanos;

  /** The last disconnect count the board gave; NaN before it gave one. */
  private double lastDisconnects = Double.NaN;

  private long disconnects;
  private long lastDisconnectNanos;

  /**
   * What the board reports as of a loop.
   *
   * @param connection whether Spotter reaches its agent
   * @param cpuCapped whether its fastest cores are held below their top frequency: false when it
   *     doesn't say
   * @param thermalMargin the level of its margin to slowing for heat, by its limits
   * @param uptimeSeconds how long since it booted; NaN when it doesn't say
   * @param usbDisconnects how many times a USB device dropped off it since it booted; NaN when it
   *     doesn't say
   * @param pushesSinceDescribed how many times it took the robot's packs since its agent last
   *     described itself (once on each connect): its agent restarts with them
   */
  public record Reading(
      Connection connection,
      boolean cpuCapped,
      Level thermalMargin,
      double uptimeSeconds,
      double usbDisconnects,
      long pushesSinceDescribed) {}

  /** Takes what the board reports as of this loop. Once a loop, the time never going back. */
  public void update(long nowNanos, Reading reading) {
    this.nowNanos = nowNanos;
    Connection connection = reading.connection();
    boolean reached = connection == Connection.CONNECTED;
    everReached |= reached;
    otherProtocol = connection == Connection.OTHER_PROTOCOL;
    boolean missingNow = connection == Connection.MISSING;
    if (missingNow && !missing) {
      // Its agent restarts with packs the robot pushed it since it last connected: that absence is
      // the robot's doing, not the board dropping out.
      restartingForPacks = reading.pushesSinceDescribed() > 0;
      if (restartingForPacks) {
        packRestarts++;
      }
    }
    missing = missingNow;
    // The robot and its boards power on together, and a board's agent can start later than the
    // robot program: missing before it's ever reached is a board booting, not one that dropped out.
    dropouts.update(missing && everReached && !restartingForPacks, nowNanos);
    // A board away says nothing new: what it last said of itself doesn't hold for now.
    throttled.update(reached && reading.cpuCapped(), nowNanos);
    Level margin = reading.thermalMargin();
    boolean low = margin == Level.WARNING || margin == Level.FAILING;
    lowThermalMargin.update(reached && low, nowNanos);
    countReboots(reading.uptimeSeconds());
    countDisconnects(reading.usbDisconnects());
  }

  /** A reboot is the uptime going back. The first uptime given is where counting starts. */
  private void countReboots(double uptimeSeconds) {
    if (Double.isNaN(uptimeSeconds)) {
      return;
    }
    if (uptimeSeconds < lastUptime) {
      reboots++;
      lastRebootNanos = nowNanos;
    }
    lastUptime = uptimeSeconds;
  }

  /**
   * Counts the disconnects since the robot started. The board counts since its boot, so after a
   * reboot its count starts again from 0, and all of it is new.
   */
  private void countDisconnects(double usbDisconnects) {
    if (Double.isNaN(usbDisconnects)) {
      return;
    }
    double added = 0;
    if (usbDisconnects >= lastDisconnects) {
      added = usbDisconnects - lastDisconnects;
    } else if (!Double.isNaN(lastDisconnects)) {
      added = usbDisconnects;
    }
    if (added > 0) {
      disconnects += Math.round(added);
      lastDisconnectNanos = nowNanos;
    }
    lastDisconnects = usbDisconnects;
  }

  /** Whether Spotter can't reach it now. */
  public boolean missing() {
    return missing;
  }

  /** Whether Spotter has reached it this session. */
  public boolean everReached() {
    return everReached;
  }

  /**
   * Its dropouts: missing after Spotter had reached it, now and this session, how often and how
   * long.
   */
  public Episodes dropouts() {
    return dropouts;
  }

  /** Whether it answers on another protocol than the manager's, so nothing it says is read. */
  public boolean otherProtocol() {
    return otherProtocol;
  }

  /** Whether it's slowed itself for heat or power now, and its episodes this session. */
  public Episodes throttled() {
    return throttled;
  }

  /** Whether it's near slowing for heat now, by its limits, and its episodes this session. */
  public Episodes lowThermalMargin() {
    return lowThermalMargin;
  }

  /**
   * How many times it was away this session while its agent restarted with packs the robot pushed
   * it: not dropouts.
   */
  public int packRestarts() {
    return packRestarts;
  }

  /** How many times it rebooted this session. */
  public int reboots() {
    return reboots;
  }

  /** When it was last seen to have rebooted; empty if it hasn't this session. */
  public OptionalLong lastRebootNanos() {
    return reboots == 0 ? OptionalLong.empty() : OptionalLong.of(lastRebootNanos);
  }

  /** How many times a USB device dropped off it this session. */
  public long usbDisconnects() {
    return disconnects;
  }

  /** When one last did; empty if none has this session. */
  public OptionalLong lastUsbDisconnectNanos() {
    return disconnects == 0 ? OptionalLong.empty() : OptionalLong.of(lastDisconnectNanos);
  }

  /** The time of the last update. */
  public long nowNanos() {
    return nowNanos;
  }
}

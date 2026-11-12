package com.michaelgrundvig.frc.runtime.lib.health;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.lib.health.BoardHealth.Connection;
import com.michaelgrundvig.frc.runtime.lib.health.BoardHealth.Level;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * A coprocessor's health over a session, from what its Spotter agent reports each loop: a board
 * that's fine, and one that throttles, runs hot, goes missing and comes back, reboots, or loses a
 * USB device.
 */
class BoardHealthTest {
  private static final long SECOND = 1_000_000_000L;
  private static final double NONE = Double.NaN;

  /**
   * What the board reports from a time on, until the next report.
   *
   * @param atSeconds from when, in seconds into the session
   * @param pushesSinceDescribed how many times it took the robot's packs since its agent last
   *     described itself
   */
  private record Report(
      double atSeconds,
      Connection connection,
      boolean capped,
      Level margin,
      double uptime,
      double usbDisconnects,
      long pushesSinceDescribed) {
    /** No packs pushed since its agent connected. */
    Report(
        double atSeconds,
        Connection connection,
        boolean capped,
        Level margin,
        double uptime,
        double usbDisconnects) {
      this(atSeconds, connection, capped, margin, uptime, usbDisconnects, 0);
    }
  }

  /** Fine, with this many of the robot's pushes since its agent last connected. */
  private static Report agent(double at, Connection connection, long pushesSinceDescribed) {
    return new Report(at, connection, false, Level.OK, 100 + at, 0, pushesSinceDescribed);
  }

  private static Report fine(double at, double uptime) {
    return new Report(at, Connection.CONNECTED, false, Level.OK, uptime, 0);
  }

  /** What the session says once it's over, at 60 s: missing now, and its dropouts. */
  private record Expected(
      boolean missingNow,
      int missingTimes,
      double missingSeconds,
      int throttledTimes,
      int lowMarginTimes,
      int reboots,
      long usbDisconnects) {}

  private record Scenario(String name, List<Report> reports, Expected expected) {}

  @Test
  void eachScenarioReadsAsItHappened() {
    List<Scenario> scenarios =
        List.of(
            new Scenario("fine", List.of(fine(0, 100)), new Expected(false, 0, 0, 0, 0, 0, 0)),
            new Scenario(
                "throttles from 10 s to 20 s",
                List.of(
                    fine(0, 100),
                    new Report(10, Connection.CONNECTED, true, Level.OK, 110, 0),
                    fine(20, 120)),
                new Expected(false, 0, 0, 1, 0, 0, 0)),
            new Scenario(
                "runs hot twice: warning, then failing",
                List.of(
                    fine(0, 100),
                    new Report(10, Connection.CONNECTED, false, Level.WARNING, 110, 0),
                    fine(20, 120),
                    new Report(30, Connection.CONNECTED, true, Level.FAILING, 130, 0),
                    fine(40, 140)),
                new Expected(false, 0, 0, 1, 2, 0, 0)),
            new Scenario(
                "missing for 5 s, then back without a reboot (a network cable)",
                List.of(
                    fine(0, 100),
                    new Report(20, Connection.MISSING, false, Level.OK, 100, 0),
                    fine(25, 125)),
                new Expected(false, 1, 5, 0, 0, 0, 0)),
            new Scenario(
                "missing, then back having rebooted (its power)",
                List.of(
                    fine(0, 600),
                    new Report(20, Connection.MISSING, false, Level.OK, 600, 0),
                    fine(45, 18)),
                new Expected(false, 1, 25, 0, 0, 1, 0)),
            new Scenario(
                "missing at the end, still",
                List.of(fine(0, 100), new Report(50, Connection.MISSING, false, Level.OK, 100, 0)),
                new Expected(true, 1, 10, 0, 0, 0, 0)),
            new Scenario(
                "a camera unplugged twice: its disconnects since boot rise from 3",
                List.of(
                    new Report(0, Connection.CONNECTED, false, Level.OK, 100, 3),
                    new Report(10, Connection.CONNECTED, false, Level.WARNING, 110, 4),
                    new Report(30, Connection.CONNECTED, false, Level.OK, 130, 5)),
                new Expected(false, 0, 0, 0, 1, 0, 2)),
            new Scenario(
                "a reboot starts its disconnects again: one after it is one more",
                List.of(
                    new Report(0, Connection.CONNECTED, false, Level.OK, 600, 3),
                    new Report(20, Connection.CONNECTED, false, Level.OK, 15, 0),
                    new Report(30, Connection.CONNECTED, false, Level.OK, 25, 1)),
                new Expected(false, 0, 0, 0, 0, 1, 1)),
            new Scenario(
                "what it last said while away doesn't count: capped and hot, then missing",
                List.of(
                    fine(0, 100),
                    new Report(20, Connection.MISSING, true, Level.FAILING, 100, 0),
                    fine(30, 130)),
                new Expected(false, 1, 10, 0, 0, 0, 0)),
            new Scenario(
                "no debian pack: no uptime, no disconnects, so no reboots to see",
                List.of(
                    new Report(0, Connection.CONNECTED, false, Level.UNAVAILABLE, NONE, NONE),
                    new Report(20, Connection.MISSING, false, Level.UNAVAILABLE, NONE, NONE),
                    new Report(40, Connection.CONNECTED, false, Level.UNAVAILABLE, NONE, NONE)),
                new Expected(false, 1, 20, 0, 0, 0, 0)),
            new Scenario(
                "missing at the start, until its agent started: booting, not a dropout",
                List.of(
                    new Report(0, Connection.CONNECTING, false, Level.UNAVAILABLE, NONE, NONE),
                    new Report(5, Connection.MISSING, false, Level.UNAVAILABLE, NONE, NONE),
                    fine(25, 30)),
                new Expected(false, 0, 0, 0, 0, 0, 0)),
            new Scenario(
                "never reached",
                List.of(new Report(5, Connection.MISSING, false, Level.UNAVAILABLE, NONE, NONE)),
                new Expected(true, 0, 0, 0, 0, 0, 0)),
            new Scenario(
                "still connecting: not missing yet",
                List.of(new Report(0, Connection.CONNECTING, false, Level.UNAVAILABLE, NONE, NONE)),
                new Expected(false, 0, 0, 0, 0, 0, 0)));
    for (Scenario scenario : scenarios) {
      BoardHealth health = run(scenario.reports());
      Expected expected = scenario.expected();
      String name = scenario.name();
      long end = health.nowNanos();
      assertThat(health.missing()).as(name).isEqualTo(expected.missingNow());
      assertThat(health.dropouts().count()).as(name).isEqualTo(expected.missingTimes());
      assertThat(health.dropouts().seconds(end))
          .as(name)
          .isCloseTo(expected.missingSeconds(), within(0.03));
      assertThat(health.throttled().count()).as(name).isEqualTo(expected.throttledTimes());
      assertThat(health.lowThermalMargin().count()).as(name).isEqualTo(expected.lowMarginTimes());
      assertThat(health.reboots()).as(name).isEqualTo(expected.reboots());
      assertThat(health.lastRebootNanos().isPresent()).as(name).isEqualTo(expected.reboots() > 0);
      assertThat(health.usbDisconnects()).as(name).isEqualTo(expected.usbDisconnects());
      assertThat(health.lastUsbDisconnectNanos().isPresent())
          .as(name)
          .isEqualTo(expected.usbDisconnects() > 0);
    }
  }

  /**
   * The robot pushes a board its packs, and its agent restarts with them: away for a moment, the
   * robot's doing, not a dropout. A dropout after its agent is back is one.
   */
  @Test
  void awayWhileItsAgentRestartsWithTheRobotsPacksIsNoDropout() {
    record Case(String name, List<Report> reports, int packRestarts, int dropouts) {}
    List<Case> cases =
        List.of(
            new Case(
                "pushed at 20 s, away 21 to 23 s, back",
                List.of(
                    agent(0, Connection.CONNECTED, 0),
                    agent(20, Connection.CONNECTED, 1),
                    agent(21, Connection.MISSING, 1),
                    agent(23, Connection.CONNECTED, 0)),
                1,
                0),
            new Case(
                "pushed, back before it counted as missing; a dropout later is one",
                List.of(
                    agent(0, Connection.CONNECTED, 0),
                    agent(20, Connection.CONNECTED, 1),
                    agent(20.5, Connection.CONNECTED, 0),
                    agent(40, Connection.MISSING, 0),
                    agent(45, Connection.CONNECTED, 0)),
                0,
                1),
            new Case(
                "pushed, away, back, then dropped out",
                List.of(
                    agent(0, Connection.CONNECTED, 0),
                    agent(20, Connection.CONNECTED, 1),
                    agent(21, Connection.MISSING, 1),
                    agent(23, Connection.CONNECTED, 0),
                    agent(40, Connection.MISSING, 0)),
                1,
                1));
    for (Case each : cases) {
      BoardHealth health = run(each.reports());
      assertThat(health.packRestarts()).as(each.name()).isEqualTo(each.packRestarts());
      assertThat(health.dropouts().count()).as(each.name()).isEqualTo(each.dropouts());
    }
  }

  @Test
  void aBoardOnAnotherProtocolSaysSo() {
    BoardHealth health = new BoardHealth();
    health.update(
        0,
        new BoardHealth.Reading(
            Connection.OTHER_PROTOCOL, false, Level.UNAVAILABLE, NONE, NONE, 0));
    assertThat(health.otherProtocol()).isTrue();
    assertThat(health.missing()).as("reached, but not read").isFalse();
  }

  /** Sixty seconds of 50 Hz loops, each with the latest report as of its time. */
  private static BoardHealth run(List<Report> reports) {
    BoardHealth health = new BoardHealth();
    for (int loop = 0; loop <= 3000; loop++) {
      long now = loop * SECOND / 50;
      Report latest = reports.get(0);
      for (Report report : reports) {
        if (report.atSeconds() * SECOND <= now) {
          latest = report;
        }
      }
      health.update(
          now,
          new BoardHealth.Reading(
              latest.connection(),
              latest.capped(),
              latest.margin(),
              latest.uptime(),
              latest.usbDisconnects(),
              latest.pushesSinceDescribed()));
    }
    return health;
  }
}

package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.stream.LongStream;

/**
 * A match of the monkey ({@link Monkey}) on a robot, as the monkey's tests play it: the guide's
 * robot in the runtime's, a team's own in the team's. A match of random presses, sticks, commands
 * and faults for each of a few seeds, every loop checked against what must always hold.
 */
public final class MonkeyMatches {
  private MonkeyMatches() {}

  /** How many seeds each robot runs, unless asked for more or fewer. */
  static final int SEEDS = 3;

  /** A match, in seconds: disabled a second, 15 s of autonomous, disabled 2 s, 135 s of teleop. */
  static final double MATCH = 153;

  /** The seeds asked for: {@code -PmonkeySeed} alone, or the first {@code -PmonkeySeeds}. */
  public static LongStream seeds() {
    String one = System.getProperty("frc.monkeySeed");
    if (one != null) {
      return LongStream.of(Long.parseLong(one));
    }
    return LongStream.rangeClosed(
        1, Long.parseLong(System.getProperty("frc.monkeySeeds", "" + SEEDS)));
  }

  /**
   * Plays a match of the monkey at the controls, things failing as it plays: nothing breaks, and it
   * has failed something.
   *
   * @return what it did and judged, for a test to ask more of
   */
  public static Monkey.Report play(RobotSim sim, Monkey.Robot robot, long seed) {
    long started = System.nanoTime();

    Monkey.Report report = new Monkey(sim, robot, seed, MATCH, true).run();

    System.out.printf(
        "Monkey on %s, seed %d: %d loops, %d actions, %d faults, in %.1f s%n",
        robot,
        seed,
        report.loops(),
        report.actions(),
        report.faults(),
        (System.nanoTime() - started) / 1e9);
    assertThat(report.faults()).isPositive();
    assertThat(report.judged())
        .containsKeys(
            "numbers only",
            "known before driven",
            "limit switches stop it",
            "moves end",
            "jams stall",
            "dead channels flagged",
            "drive moves end");
    return report;
  }
}

package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.examples.ExampleMonkey;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The monkey ({@link Monkey}) on the guide's robot, with its controls: a match of random presses,
 * sticks, commands and faults for each of a few seeds, every loop checked against what must always
 * hold. {@code -PmonkeySeeds=<count>} runs more seeds, and {@code -PmonkeySeed=<seed>} runs one
 * alone, the one a failure names (docs/testing.md). A team's own robots are played the same way in
 * the team's tests.
 */
@SimulationTest
class MonkeySimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  static Stream<Arguments> runs() {
    return MonkeyMatches.seeds().mapToObj(seed -> Arguments.of(ExampleMonkey.ROBOT, seed));
  }

  /**
   * A match of the monkey at the controls, things failing as it plays: nothing breaks, and each
   * check is counted, the obvious ones having judged something on the guide's robot.
   */
  @ParameterizedTest(name = "{0}, seed {1}")
  @MethodSource("runs")
  @Timeout(600)
  void aMatchOfMonkeyBreaksNothing(Monkey.Robot robot, long seed) {
    Monkey.Report report = MonkeyMatches.play(sim, robot, seed);

    assertThat(report.judged())
        .hasEntrySatisfying("numbers only", loops -> assertThat(loops).isPositive())
        .hasEntrySatisfying("moves end", loops -> assertThat(loops).isPositive())
        .hasEntrySatisfying("drive moves end", loops -> assertThat(loops).isPositive());
  }

  /** A seed runs the same every time: the same doings, and every joint where it was. */
  @Test
  void aSeedRunsTheSameEveryTime() {
    Monkey.Report first = new Monkey(sim, ExampleMonkey.ROBOT, 7, 20, true).run();
    Monkey.Report again = new Monkey(sim, ExampleMonkey.ROBOT, 7, 20, true).run();

    assertThat(again).isEqualTo(first);
  }
}

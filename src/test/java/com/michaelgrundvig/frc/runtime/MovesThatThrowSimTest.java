package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.entry;
import static org.wpilib.units.Units.Degrees;

import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveMove;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;

/**
 * A move whose own code throws, on a robot: the scheduler ends it without canceling it, so the move
 * records its own ending, says what it threw, and lets go of what it drove.
 */
@SimulationTest
class MovesThatThrowSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  /** How many loops each move runs before its team's code throws. */
  private static final int LOOPS_BEFORE_IT_THROWS = 5;

  /**
   * How a move's run ended.
   *
   * @param outcome how it ended, or empty while it says it runs
   * @param why the text of why it ended so, as logged beside its outcome
   */
  private record Ended(Optional<Outcome> outcome, String why) {}

  /**
   * A joint's follow whose target throws, and a drive move whose condition throws, each end
   * REFUSED, saying what was thrown, in the loop it was thrown. The robot drives on.
   */
  @Test
  void aMoveWhoseTeamsCodeThrowsEndsRefusedSayingWhatItThrew() {
    sim.build(BreakableRobot.JOINTS_WITHOUT_CAMERAS, SensorErrors.NONE);
    RobotSim.setEnabled(true);
    sim.loopOnRobot();
    Move follow =
        sim.joints().of(ExampleRobot.HOOD).follow(aTargetThatThrowsLater(() -> Degrees.of(10)));
    BooleanSupplier never = () -> false;
    DriveMove drive =
        sim.drive().runUntil(new ChassisVelocities(1, 0, 0), aConditionThatThrowsLater(never));

    Map<String, Ended> ended = new LinkedHashMap<>();
    sim.scheduler().schedule(follow);
    runForAWhile();
    ended.put("follow", new Ended(follow.outcome(), follow.ending().orElseThrow().why()));
    sim.scheduler().schedule(drive);
    runForAWhile();
    ended.put("drive", new Ended(drive.outcome(), drive.endingWhy()));

    String threw = "threw IllegalStateException: " + BreakableRobot.THROWN;
    assertThat(ended)
        .containsExactly(
            entry("follow", new Ended(Optional.of(Outcome.REFUSED), threw)),
            entry("drive", new Ended(Optional.of(Outcome.REFUSED), threw)));
    assertThat(sim.scheduler().getRunningCommands())
        .extracting(Command::name)
        .doesNotContain(follow.name(), drive.name());
  }

  /** Loops on a robot for long enough that each move has thrown. */
  private void runForAWhile() {
    for (int i = 0; i < 2 * LOOPS_BEFORE_IT_THROWS; i++) {
      sim.loopOnRobot();
    }
  }

  /** A target that the move reads every loop, and that throws from its sixth reading on. */
  private static Supplier<Angle> aTargetThatThrowsLater(Supplier<Angle> target) {
    int[] calls = {0};
    return () -> {
      calls[0]++;
      if (calls[0] > LOOPS_BEFORE_IT_THROWS) {
        throw new IllegalStateException(BreakableRobot.THROWN);
      }
      return target.get();
    };
  }

  /** A condition that the move reads every loop, and that throws from its sixth reading on. */
  private static BooleanSupplier aConditionThatThrowsLater(BooleanSupplier condition) {
    int[] calls = {0};
    return () -> {
      calls[0]++;
      if (calls[0] > LOOPS_BEFORE_IT_THROWS) {
        throw new IllegalStateException(BreakableRobot.THROWN);
      }
      return condition.getAsBoolean();
    };
  }
}

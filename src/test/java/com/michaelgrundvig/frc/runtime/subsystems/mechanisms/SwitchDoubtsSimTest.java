package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A limit switch checked against where its homed joint is: tripped nowhere near where it trips, or
 * clear well past it, for a quarter second, it's doubted, saying what to check; a doubted switch
 * still stops its joint.
 */
@ExtendWith(RobotSim.class)
class SwitchDoubtsSimTest {
  /** The pivot's bottom switch, tripping at 30°: the pivot rests below it, at 1.5°. */
  private static final Sensor.Switch PIVOT_DOWN =
      JointFixtures.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Math.toRadians(30));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(PIVOT_DOWN)).build();

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final JointMechanism pivot = sim.joints.of(PIVOT);

  /**
   * Out at 130°, its normally closed switch's wire cut: it reads tripped, nowhere near 30°. The
   * pivot can't go back down past it, the safe way to be wrong.
   */
  @Test
  void aCutNormallyClosedWireReadsTrippedFarFromItsSwitch() {
    sim.run(pivot.goTo(PIVOT_OUT), 1.5);
    assertThat(sim.joints.switchDoubts()).as("whole, and clear out there").isEmpty();

    sim.sensors.cut(PIVOT_DOWN, SensorWiring.Polarity.NORMALLY_CLOSED);
    sim.run(0.2);
    List<String> early = sim.joints.switchDoubts();
    sim.run(0.1);

    assertThat(early).as("not for a moment's disagreement").isEmpty();
    assertThat(sim.joints.switchDoubts())
        .containsExactly(
            "pivot down reads tripped with pivot at 130°, nowhere near where it trips, 30°: a"
                + " cut wire, if it's normally closed, or a switch out of place");
    Move in = pivot.goTo(PIVOT_IN);
    sim.scheduler.schedule(in);
    sim.runUntil(() -> in.outcome().isPresent(), 3);
    assertThat(in.reason()).contains("held: pivot down tripped");
  }

  /**
   * Resting at 1.2°, its sag, well past 30°, its normally open switch's wire cut: it reads clear.
   */
  @Test
  void aCutNormallyOpenWireReadsClearWellPastItsSwitch() {
    sim.run(0.5);
    assertThat(sim.joints.switchDoubts()).as("whole, and tripped down here").isEmpty();

    sim.sensors.cut(PIVOT_DOWN, SensorWiring.Polarity.NORMALLY_OPEN);
    sim.run(0.5);

    assertThat(sim.joints.switchDoubts())
        .containsExactly(
            "pivot down reads clear with pivot at 1.21°, past where it trips, 30°: a cut wire, if"
                + " it's normally open, or a switch out of place");
  }
}

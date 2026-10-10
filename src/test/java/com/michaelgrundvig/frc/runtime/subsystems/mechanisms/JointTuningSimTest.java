package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A joint's gains and current limits tuned from the pit: its motor controller takes them on the
 * next loop, and the change says how to commit it. Nothing is saved.
 */
@ExtendWith(RobotSim.class)
class JointTuningSimTest {
  private final Map<String, LiveConfig<MechanismControl>> tunings = new LinkedHashMap<>();
  private final JointsSimHarness sim =
      new JointsSimHarness(
          ROBOT,
          joint ->
              tunings.computeIfAbsent(
                      joint.label(),
                      label -> JointMechanism.tuning(joint, ControllerSettings.control(joint)))
                  ::get);

  /** How long the pivot takes to swing out, from in. */
  private double swingOut(JointsSimHarness harness) {
    JointMechanism pivot = harness.joints.of(JointFixtures.PIVOT);
    harness.run(pivot.goTo(PIVOT_IN), 2);
    Move out = pivot.goTo(PIVOT_OUT);
    harness.scheduler.schedule(out);
    return harness.runUntil(() -> out.outcome().isPresent(), 3);
  }

  @Test
  void aTunedCurrentLimitTakesEffectOnTheNextLoop() {
    double asCommitted = swingOut(sim);

    LiveConfig<MechanismControl> pivot = Objects.requireNonNull(tunings.get("pivot"));
    assertThat(pivot.set(Map.of("currentLimits.stator", 5.0))).isTrue();
    double starved = swingOut(sim);

    // The light test pivot needs its current only to speed up: starved, it ramps slower.
    assertThat(starved).as("5 A against 40 A").isGreaterThan(asCommitted + 0.05);
    assertThat(pivot.changes())
        .singleElement()
        .satisfies(
            change -> assertThat(change.builderCall()).isEqualTo(".withStatorLimit(Amps.of(5.0))"));
    pivot.revert();
    assertThat(swingOut(sim)).isEqualTo(asCommitted);
  }

  @Test
  void aJointsTuningIsItsGainsAndCurrentLimitsWithinWhatsSafe() {
    MechanismControl control = MechanismControl.position(30).withStatorLimit(Amps.of(150));
    LiveConfig<MechanismControl> tuning = JointMechanism.tuning(JointFixtures.PIVOT, control);

    assertThat(tuning.name()).isEqualTo("pivot");
    assertThat(tuning.knobs())
        .extracting(LiveConfig.Knob::path)
        .containsExactly(
            "gains.kP", "gains.kD", "gains.kS", "currentLimits.stator", "currentLimits.supply");
    assertThat(tuning.knobs().get(0).limits().max())
        .as("ten times what's committed")
        .isEqualTo(300);
    assertThat(tuning.knobs().get(3).limits().max())
        .as("never below what's committed")
        .isEqualTo(150);
    assertThatThrownBy(() -> tuning.candidate(Map.of("gains.kP", 400.0)))
        .isInstanceOf(IllegalArgumentException.class);
  }
}

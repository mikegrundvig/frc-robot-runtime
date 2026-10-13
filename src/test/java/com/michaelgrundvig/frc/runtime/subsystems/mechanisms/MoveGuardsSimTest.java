package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Percent;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.record.MoveReason;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/** A move's guards: a gate checked once as it starts, and a gate checked every loop. */
@ExtendWith(RobotSim.class)
class MoveGuardsSimTest {
  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);
  private final boolean[] clear = {false};
  private final Gate intakeClear = Gate.of(sim.scheduler, "intake clear", () -> clear[0]);

  @Test
  void onlyIfRefusesAMoveWhoseGateIsFalseAsItStarts() {
    Move out = pivot.goTo(PIVOT_OUT).onlyIf(intakeClear);

    sim.run(out, 0.5);

    assertThat(out.outcome()).contains(Outcome.REFUSED);
    ended(out).refused().because(MoveReason.GATE_FALSE).saying("intake clear is false");
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(1.5, within(1.0));
  }

  /** Checked only as it starts: the gate falling later doesn't stop it. */
  @Test
  void onlyIfIsCheckedOnce() {
    clear[0] = true;
    Move out = pivot.goTo(PIVOT_OUT).onlyIf(intakeClear);
    sim.scheduler.schedule(out);
    sim.run(0.1);

    clear[0] = false;
    sim.runUntil(() -> out.outcome().isPresent(), 2);

    assertThat(out.arrived()).isTrue();
  }

  @Test
  void onlyWhileCancelsWhenItsGateFallsAndSaysWhy() {
    boolean[] ready = {true};
    Gate atSpeed = Gate.of(sim.scheduler, "shooter at goal", () -> ready[0]);
    Gate readyToShoot = Gate.all("ready to shoot", atSpeed, intakeClear);
    clear[0] = true;
    Move feed = roller.run(ROLLER_INTAKING).onlyWhile(readyToShoot);
    sim.run(feed, 0.5);
    double feeding = sim.simulation.velocity(ROLLER);

    ready[0] = false;
    sim.run(0.5);

    assertThat(feeding).isGreaterThan(100);
    assertThat(feed.outcome()).contains(Outcome.CANCELED);
    ended(feed)
        .canceled()
        .inBetween(0.5, 0.6)
        .because(MoveReason.GATE_FELL)
        .saying("ready to shoot is false: shooter at goal");
    assertThat(sim.simulation.velocity(ROLLER)).isCloseTo(0, within(0.5));
  }

  @Test
  void onlyWhileRefusesAMoveWhoseGateIsFalseToStartWith() {
    Move feed = roller.run(ROLLER_INTAKING).onlyWhile(intakeClear);

    sim.run(feed, 0.2);

    assertThat(feed.outcome()).contains(Outcome.REFUSED);
    ended(feed).refused().because(MoveReason.GATE_FALSE).saying("intake clear is false");
  }

  @Test
  void guardsCarryThroughTheMovesOtherOptions() {
    clear[0] = true;
    Move out = pivot.goTo(PIVOT_OUT).onlyWhile(intakeClear).atSpeed(Percent.of(50));
    sim.scheduler.schedule(out);
    sim.run(0.2);

    clear[0] = false;
    sim.run(0.1);

    assertThat(out.name()).isEqualTo("pivot → out at 50%");
    assertThat(out.outcome()).contains(Outcome.CANCELED);
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.JointAsk;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * What a joint does when no command asks anything of it: a joint held at positions holds its goal,
 * and a roller stops. No simulator: each loop is from what it's given.
 */
class IdleTest {
  private static final Demand TO_ONE = Demands.toPosition(1, 0.5);
  private static final Demand AT_SPEED = Demands.atVelocity(40);
  private static final Applied NOTHING = Applied.Undriven.NOTHING;

  /** In a loop's row: the joint wasn't told where it holds as the loop began. */
  private static final double NOT_TOLD = Double.POSITIVE_INFINITY;

  /** Nothing to hold until it's first read; then where it was read, along the profile it's on. */
  @Test
  void aJointHeldAtPositionsHoldsWhereItWasRead() {
    Idle idle = new Idle("pivot", true);
    boolean before = idle.at().isPresent();
    idle.holdAt(0.3);

    Demand held = idle.demand(TO_ONE, 0.5);

    assertThat(before).isFalse();
    assertThat(held).isEqualTo(Demands.toPosition(0.3, 1));
  }

  /** A move that ended leaves it holding where the move put it. */
  @Test
  void afterAMoveItHoldsWhereTheMoveEnded() {
    Idle idle = new Idle("pivot", true);
    idle.holdAt(0);

    idle.moveEnded(1.2);

    assertThat(idle.demand(TO_ONE, 1.1)).isEqualTo(Demands.toPosition(1.2, 1));
    assertThat(idle.at()).hasValue(1.2);
  }

  /** From an output or a stop, it holds where it is now, its profile starting again from there. */
  @Test
  void fromAnOutputItHoldsWhereItIsNow() {
    Idle idle = new Idle("pivot", true);
    idle.holdAt(0);

    Demand held = idle.demand(Demands.atVoltage(3), 0.8);

    assertThat(held).isEqualTo(Demands.toPosition(0.8, 1).restartingProfile());
    assertThat(idle.at()).hasValue(0.8);
  }

  /**
   * A roller stops; but one sent to a position, as a turret with no range is, holds there after,
   * until it's run at an output again.
   */
  @Test
  void aJointThatOnlyRunsAtOutputsStops() {
    Idle idle = new Idle("roller", false);
    idle.holdAt(0);

    Demand stopped = idle.demand(AT_SPEED, 2);
    idle.commandAsked(TO_ONE.ask());
    Demand holding = idle.demand(TO_ONE, 0.9);
    idle.commandAsked(AT_SPEED.ask());
    Demand stoppedAgain = idle.demand(AT_SPEED, 3);

    assertThat(stopped).isEqualTo(Demand.NEUTRAL);
    assertThat(holding.ask()).isInstanceOf(JointAsk.ToPosition.class);
    assertThat(stoppedAgain.ask()).isEqualTo(JointAsk.NEUTRAL);
  }

  /**
   * A reading that isn't a number is nowhere to hold: left idle, the joint is neutral and says why,
   * the very same demand loop after loop however often it's told so, and it's held at its next
   * reading that is a number, its profile starting from there once. Until 2026-10-11 it was sent to
   * a position of NaN, by a demand built again every loop.
   */
  @Test
  void aReadingThatIsNotANumberIsNowhereToHold() {
    Idle idle = new Idle("pivot", true);
    Demand nowhere = new Demand(JointAsk.NEUTRAL, false, "pivot's position isn't a number");
    List<Demand> idled = new ArrayList<>();

    // Each row is a loop: where the joint was told it holds as the loop began, if it was told, and
    // where it read; the loop before, it did what the idle last handed out.
    double[][] loops = {
      {Double.NaN, Double.NaN},
      {Double.NaN, Double.NaN},
      {Double.NaN, Double.NaN},
      {0.4, 0.4},
      {NOT_TOLD, 0.41},
      {NOT_TOLD, Double.NaN},
      {NOT_TOLD, 0.39},
    };
    Applied lastLoop = NOTHING;
    for (double[] loop : loops) {
      if (!Double.isInfinite(loop[0])) {
        idle.holdAt(loop[0]);
      }
      Demand demand = idle.demand(lastLoop, loop[1]);
      idled.add(demand);
      lastLoop = demand;
    }

    Demand holding = Demands.toPosition(0.4, 1);
    assertThat(idled)
        .containsExactly(
            nowhere, nowhere, nowhere, holding.restartingProfile(), holding, holding, holding);
    assertThat(idled.get(1)).as("nothing built while nowhere to hold").isSameAs(idled.get(0));
    assertThat(idled.get(2)).isSameAs(idled.get(0));
    assertThat(idled.get(5))
        .as("a joint that holds somewhere holds on there, whatever it reads")
        .isSameAs(idled.get(4));
    assertThat(idle.at()).hasValue(0.4);
  }

  /** A roller, which holds nowhere, is neutral whatever it reads, and has nothing to say of it. */
  @Test
  void aRollerThatReadsNoNumberIsNeutralAsEver() {
    Idle idle = new Idle("roller", false);

    idle.holdAt(Double.NaN);

    assertThat(idle.demand(AT_SPEED, Double.NaN)).isSameAs(Demand.NEUTRAL);
  }
}

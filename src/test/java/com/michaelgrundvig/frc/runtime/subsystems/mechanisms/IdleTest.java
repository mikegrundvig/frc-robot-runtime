package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.JointAsk;
import org.junit.jupiter.api.Test;

/**
 * What a joint does when no command asks anything of it: a joint held at positions holds its goal,
 * and a roller stops. No simulator: each loop is from what it's given.
 */
class IdleTest {
  private static final Demand TO_ONE = Demands.toPosition(1, 0.5);
  private static final Demand AT_SPEED = Demands.atVelocity(40);
  private static final Applied NOTHING = Applied.Undriven.NOTHING;

  /** Nothing to hold until it's first read; then where it was read, along the profile it's on. */
  @Test
  void aJointHeldAtPositionsHoldsWhereItWasRead() {
    Idle idle = new Idle(true);
    boolean before = idle.at().isPresent();
    idle.holdAt(0.3);

    Demand held = idle.demand(TO_ONE, 0.5);

    assertThat(before).isFalse();
    assertThat(held).isEqualTo(Demands.toPosition(0.3, 1));
  }

  /** A move that ended leaves it holding where the move put it. */
  @Test
  void afterAMoveItHoldsWhereTheMoveEnded() {
    Idle idle = new Idle(true);
    idle.holdAt(0);

    idle.moveEnded(1.2);

    assertThat(idle.demand(TO_ONE, 1.1)).isEqualTo(Demands.toPosition(1.2, 1));
    assertThat(idle.at()).hasValue(1.2);
  }

  /** From an output or a stop, it holds where it is now, its profile starting again from there. */
  @Test
  void fromAnOutputItHoldsWhereItIsNow() {
    Idle idle = new Idle(true);
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
    Idle idle = new Idle(false);
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

  /** A reading that isn't a number is nowhere to hold: the joint is held at its next reading. */
  @Test
  void aReadingThatIsNotANumberIsNowhereToHold() {
    Idle idle = new Idle(true);

    idle.holdAt(Double.NaN);

    assertThat(idle.at()).isEmpty();
    assertThat(idle.demand(NOTHING, 0.4)).isEqualTo(Demands.toPosition(0.4, 1).restartingProfile());
  }
}

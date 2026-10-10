package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

/**
 * What a joint does when no command asks anything of it: a joint held at positions holds its goal,
 * and a roller stops. No simulator: each loop is from what it's given.
 */
class IdleTest {
  private static final Demand TO_ONE = Demand.of(Request.POSITION, 1, 0.5);
  private static final Demand AT_SPEED = Demand.of(Request.VELOCITY, 40, 1);

  /** A command's request passes through as it is. */
  @Test
  void whatACommandAsksPassesThrough() {
    Idle idle = new Idle(true);

    assertThat(idle.resolve(TO_ONE, Request.NONE, 0)).isSameAs(TO_ONE);
    assertThat(idle.resolve(AT_SPEED, Request.POSITION, 0)).isSameAs(AT_SPEED);
  }

  /** Nothing to hold until it's first read; then where it was read, along the profile it's on. */
  @Test
  void aJointHeldAtPositionsHoldsWhereItWasRead() {
    Idle idle = new Idle(true);
    double before = idle.at();
    idle.holdAt(0.3);

    Demand held = idle.resolve(Demand.NONE, Request.POSITION, 0.5);

    assertThat(before).isNaN();
    assertThat(held).isEqualTo(Demand.of(Request.POSITION, 0.3, 1));
  }

  /** A move that ended leaves it holding where the move put it. */
  @Test
  void afterAMoveItHoldsWhereTheMoveEnded() {
    Idle idle = new Idle(true);
    idle.holdAt(0);

    idle.moveEnded(1.2);

    assertThat(idle.resolve(Demand.NONE, Request.POSITION, 1.1))
        .isEqualTo(Demand.of(Request.POSITION, 1.2, 1));
    assertThat(idle.at()).isEqualTo(1.2);
  }

  /** From an output or a stop, it holds where it is now, its profile starting again from there. */
  @Test
  void fromAnOutputItHoldsWhereItIsNow() {
    Idle idle = new Idle(true);
    idle.holdAt(0);

    Demand held = idle.resolve(Demand.NONE, Request.VOLTAGE, 0.8);

    assertThat(held).isEqualTo(Demand.of(Request.POSITION, 0.8, 1).restarted());
    assertThat(idle.at()).isEqualTo(0.8);
  }

  /**
   * A roller stops; but one sent to a position, as a turret with no range is, holds there after,
   * until it's run at an output again.
   */
  @Test
  void aJointThatOnlyRunsAtOutputsStops() {
    Idle idle = new Idle(false);
    idle.holdAt(0);
    Demand stopped = idle.resolve(Demand.NONE, Request.VELOCITY, 2);

    idle.resolve(TO_ONE, Request.NONE, 0);
    Demand holding = idle.resolve(Demand.NONE, Request.POSITION, 0.9);
    idle.resolve(AT_SPEED, Request.POSITION, 0.9);
    Demand stoppedAgain = idle.resolve(Demand.NONE, Request.VELOCITY, 3);

    assertThat(stopped).isEqualTo(Demand.of(Request.NEUTRAL, 0, 1));
    assertThat(holding.kind()).isEqualTo(Request.POSITION);
    assertThat(stoppedAgain.kind()).isEqualTo(Request.NEUTRAL);
  }
}

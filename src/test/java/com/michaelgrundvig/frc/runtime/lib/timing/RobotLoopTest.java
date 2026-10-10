package com.michaelgrundvig.frc.runtime.lib.timing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;

class RobotLoopTest {
  @Test
  void aStretchOfTimeIsAWholeNumberOfLoops() {
    assertThat(RobotLoop.PERIOD_SECONDS).isCloseTo(0.02, within(1e-15));
    assertThat(RobotLoop.loops(0.3)).isEqualTo(15);
    assertThat(RobotLoop.loops(1.5)).isEqualTo(75);
    assertThat(RobotLoop.loops(10)).isEqualTo(500);
    assertThat(RobotLoop.loops(0)).isZero();
  }
}

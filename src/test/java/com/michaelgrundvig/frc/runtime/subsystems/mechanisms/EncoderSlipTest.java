package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.EncoderGap;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import org.junit.jupiter.api.Test;

/**
 * How long a joint's absolute encoder and its motor's count must be apart before one is taken to
 * have slipped.
 */
class EncoderSlipTest {
  private static final long LOOP = 20_000_000L;

  /**
   * Apart past the margin and a loop's travel at the joint's speed, for a quarter second, they're
   * said apart; a loop's travel more is allowed moving, a reading missing starts it over, and
   * inside the margin they agree.
   */
  @Test
  void theyreApartOnlyPastTheirMarginForAQuarterSecond() {
    EncoderSlip gap =
        new EncoderSlip(new EncoderGap(PIVOT, Math.toRadians(5), Double.POSITIVE_INFINITY));
    double off = Math.toRadians(6);

    String first = gap.slipped(0, off, 0, 0).orElse(null);
    String before = gap.slipped(12 * LOOP, off, 0, 0).orElse(null);
    String after = gap.slipped(13 * LOOP, off, 0, 0).orElse(null);
    String moving = gap.slipped(14 * LOOP, off, 0, 4).orElse(null);
    String missed = gap.slipped(15 * LOOP, Double.NaN, 0, 0).orElse(null);
    String again = gap.slipped(16 * LOOP, off, 0, 0).orElse(null);

    assertThat(first).isNull();
    assertThat(before).isNull();
    assertThat(after).isEqualTo("6° apart, more than 5°");
    assertThat(moving).as("6° inside 5° and a loop at 4 rad/s, 4.6°").isNull();
    assertThat(missed).isNull();
    assertThat(again).as("starting over").isNull();
    assertThat(gap.agree(Math.toRadians(4.9), 0)).isTrue();
    assertThat(gap.agree(off, 0)).isFalse();
  }

  /**
   * A turret that turns freely, its encoder on its axle, travels past the turn its encoder tells
   * apart: an encoder slipped 20° across its wrap, reading -170° with the count at 170°, is 20°
   * apart.
   */
  @Test
  void theyreApartTheNearestWayRoundAJointThatTravelsPastItsEncodersTurn() {
    Joint.Revolute turret =
        TURRET.absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0)));
    EncoderSlip gap =
        new EncoderSlip(
            EncoderGap.of(
                ROBOT.with(turret).math(), turret, turret.driven().control().orElseThrow()));

    gap.slipped(0, Math.toRadians(-170), Math.toRadians(170), 0);
    String slipped =
        gap.slipped(13 * LOOP, Math.toRadians(-170), Math.toRadians(170), 0).orElse(null);

    assertThat(gap.agree(Math.toRadians(-10), Math.toRadians(350))).isTrue();
    assertThat(gap.agree(Math.toRadians(-10), Math.toRadians(710))).isTrue();
    assertThat(slipped).isEqualTo("20° apart, more than 5°");
  }
}

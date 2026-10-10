package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Travel;
import org.junit.jupiter.api.Test;

/**
 * The profile a joint's position follows, a loop at a time: the fixtures' pivot cruises at 8 rad/s
 * and accelerates at 30 rad/s². No simulator: each step is from what it's given.
 */
class JointProfileTest {
  private static JointProfile profile(Joint joint) {
    return new JointProfile(Travel.of(ROBOT.math(), joint, ControllerSettings.control(joint)));
  }

  /** From still at 0, its first step speeds up by 30 rad/s² over a loop: 0.6 rad/s, 6 mm on. */
  @Test
  void itStartsFromWhereTheJointIs() {
    JointProfile profile = profile(PIVOT);

    double accelerating = profile.step(2, 1, 0, 0);

    assertThat(profile.velocity()).isCloseTo(0.6, within(1e-9));
    assertThat(profile.position()).isCloseTo(0.006, within(1e-9));
    assertThat(accelerating).isCloseTo(30, within(1e-9));
    assertThat(profile.goal()).isEqualTo(2);
    assertThat(profile.arrived()).isFalse();
  }

  /**
   * It goes on from its own point, whatever the joint reads, until it's restarted: then from where
   * the joint is.
   */
  @Test
  void itGoesOnFromItsOwnPointUntilRestarted() {
    JointProfile profile = profile(PIVOT);
    profile.step(2, 1, 0, 0);

    profile.step(2, 1, 1, 0);
    double onItsOwn = profile.position();
    profile.restart();
    profile.step(2, 1, 1, 0);

    assertThat(onItsOwn).as("the joint's reading ignored").isLessThan(0.1);
    assertThat(profile.position()).isCloseTo(1.006, within(1e-9));
  }

  /**
   * A point that isn't a number, from a reading that wasn't one, starts again from where the joint
   * is next read, rather than staying one: from 1 rad, 6 mm on.
   */
  @Test
  void aPointThatIsntANumberStartsAgainFromWhereTheJointIs() {
    JointProfile profile = profile(PIVOT);
    profile.step(2, 1, Double.NaN, 0);
    double lost = profile.position();

    profile.step(2, 1, 1, 0);

    assertThat(lost).isNaN();
    assertThat(profile.position()).isCloseTo(1.006, within(1e-9));
    assertThat(profile.velocity()).isCloseTo(0.6, within(1e-9));
  }

  /** At its goal and still, it stays there, and has arrived; a step there takes nothing. */
  @Test
  void itArrivesAndStays() {
    JointProfile profile = profile(PIVOT);
    for (int i = 0; i < 100; i++) {
      profile.step(1, 1, 0, 0);
    }

    double accelerating = profile.step(1, 1, 0, 0);

    assertThat(profile.arrived()).isTrue();
    assertThat(profile.position()).isEqualTo(1);
    assertThat(accelerating).isZero();
  }

  /** At half the cruise, it tops out at half: 4 rad/s. */
  @Test
  void aShareOfTheCruiseTopsOutThere() {
    JointProfile profile = profile(PIVOT);
    double fastest = 0;
    for (int i = 0; i < 50; i++) {
      profile.step(2, 0.5, 0, 0);
      fastest = Math.max(fastest, profile.velocity());
    }

    assertThat(fastest).isCloseTo(4, within(1e-9));
  }

  /** A goal past its limits is held at them; a turret's goal is the short way round. */
  @Test
  void itsGoalIsInsideItsLimitsTheShortWayRound() {
    JointProfile pivot = profile(PIVOT);
    JointProfile turret = profile(TURRET);

    pivot.step(5, 1, 0, 0);
    turret.step(Math.toRadians(-170), 1, Math.toRadians(170), 0);

    assertThat(pivot.goal()).isCloseTo(Math.toRadians(135), within(1e-12));
    assertThat(turret.goal()).isCloseTo(Math.toRadians(190), within(1e-9));
  }

  /** 1 rad from still, at 8 rad/s and 30 rad/s²: never at cruise, 2√(1/30) s. */
  @Test
  void itSaysHowLongAMoveTakes() {
    assertThat(profile(PIVOT).secondsTo(1, 1, 0, 0))
        .isCloseTo(2 * Math.sqrt(1.0 / 30), within(1e-9));
  }
}

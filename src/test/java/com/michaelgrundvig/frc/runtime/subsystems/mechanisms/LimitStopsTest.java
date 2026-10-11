package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * A joint's limit switches beneath what it's asked, a loop at a time: no simulator, each switch
 * tripped as a test says.
 */
class LimitStopsTest {
  /** At 90° of the turret's swing, on its way to the back. */
  private static final Sensor.Switch TURRET_RIGHT =
      JointFixtures.limit(
          "turret right", TURRET, Sensor.Switch.Limit.End.UPPER, Math.toRadians(90));

  /** One at the turret's other end, its position not known. */
  private static final Sensor.Switch TURRET_LEFT =
      JointFixtures.limit("turret left", TURRET, Sensor.Switch.Limit.End.LOWER);

  private final SwitchReadings switches = new SwitchReadings(TURRET_RIGHT, TURRET_LEFT);
  private final LimitStops turret = new LimitStops(List.of(TURRET_RIGHT, TURRET_LEFT));

  private static final Demand BACK = Demands.toPosition(Math.PI, 0.5);
  private static final Demand FRONT = Demands.toPosition(0, 0.5);

  /** Clear, nothing's held. */
  @Test
  void aClearSwitchHoldsNothing() {
    assertThat(turret.apply(BACK, 1, switches.sensors())).isSameAs(BACK);
  }

  /**
   * A position past a tripped switch holds where the joint was as it tripped, and says so; its
   * profile starts again from where it is, in the loop the hold begins.
   */
  @Test
  void aMovePastATrippedSwitchHoldsWhereItTripped() {
    switches.trip(TURRET_RIGHT, true);

    Demand held = turret.apply(BACK, 1.6, switches.sensors());

    assertThat(held)
        .isEqualTo(new Demand(new JointAsk.ToPosition(1.6, 0.5), true, "turret right tripped"));
  }

  /**
   * In the loops after the hold begins the profile is left alone, however far the joint is pushed
   * off where it tripped, so the joint's position loop brings it back there. The demand is the very
   * same each loop: a steady hold makes no objects.
   */
  @Test
  void aHoldRestartsItsProfileOnlyAsItBegins() {
    switches.trip(TURRET_RIGHT, true);
    turret.apply(BACK, 1.6, switches.sensors());

    Demand pushedOn = turret.apply(BACK, 1.65, switches.sensors());
    Demand pushedBack = turret.apply(BACK, 1.57, switches.sensors());

    assertThat(pushedOn)
        .isEqualTo(new Demand(new JointAsk.ToPosition(1.6, 0.5), false, "turret right tripped"));
    assertThat(pushedBack).isSameAs(pushedOn);
  }

  /** The same thing asked again as another object, as a command may each loop, is the same hold. */
  @Test
  void theSameThingAskedAgainIsTheSameHold() {
    switches.trip(TURRET_RIGHT, true);
    turret.apply(BACK, 1.6, switches.sensors());
    Demand lasting = turret.apply(BACK, 1.6, switches.sensors());

    Demand askedAgain = turret.apply(Demands.toPosition(Math.PI, 0.5), 1.62, switches.sensors());

    assertThat(askedAgain).isSameAs(lasting);
  }

  /**
   * The switch clears and trips again under the same move: a new hold, where the joint is as it
   * trips again, its profile starting again from there.
   */
  @Test
  void trippingAgainBeginsANewHoldWhereItIsThen() {
    switches.trip(TURRET_RIGHT, true);
    turret.apply(BACK, 1.6, switches.sensors());
    turret.apply(BACK, 1.6, switches.sensors());

    switches.trip(TURRET_RIGHT, false);
    Demand cleared = turret.apply(BACK, 1.59, switches.sensors());
    switches.trip(TURRET_RIGHT, true);
    Demand again = turret.apply(BACK, 1.62, switches.sensors());

    assertThat(cleared).isSameAs(BACK);
    assertThat(again)
        .isEqualTo(new Demand(new JointAsk.ToPosition(1.62, 0.5), true, "turret right tripped"));
  }

  /**
   * Another position past the switch, asked while it holds: a new hold at the same point, at the
   * new ask's share of the cruise, its profile starting again once.
   */
  @Test
  void anotherPositionPastItBeginsANewHoldAtTheSamePoint() {
    switches.trip(TURRET_RIGHT, true);
    turret.apply(BACK, 1.6, switches.sensors());
    turret.apply(BACK, 1.6, switches.sensors());
    Demand further = Demands.toPosition(2.5, 0.25);

    Demand begun = turret.apply(further, 1.64, switches.sensors());
    Demand lasting = turret.apply(further, 1.64, switches.sensors());

    assertThat(begun)
        .isEqualTo(new Demand(new JointAsk.ToPosition(1.6, 0.25), true, "turret right tripped"));
    assertThat(lasting)
        .isEqualTo(new Demand(new JointAsk.ToPosition(1.6, 0.25), false, "turret right tripped"));
  }

  /**
   * A position short of where it tripped, asked while it holds, ends the hold; the first one asked
   * again is then held where the joint is by then.
   */
  @Test
  void aPositionShortOfItEndsTheHold() {
    switches.trip(TURRET_RIGHT, true);
    turret.apply(BACK, 1.6, switches.sensors());

    Demand away = turret.apply(FRONT, 1.6, switches.sensors());
    Demand back = turret.apply(BACK, 1.55, switches.sensors());

    assertThat(away).isSameAs(FRONT);
    assertThat(back)
        .isEqualTo(new Demand(new JointAsk.ToPosition(1.55, 0.5), true, "turret right tripped"));
  }

  /**
   * A demand that restarts the profile itself, in a loop of a hold, still does; what already holds
   * the joint above the switch is said first, and a change in it begins a new hold.
   */
  @Test
  void whatComesInRestartingOrHeldStillIs() {
    switches.trip(TURRET_RIGHT, true);
    turret.apply(BACK, 1.6, switches.sensors());
    turret.apply(BACK, 1.6, switches.sensors());

    Demand restarting = turret.apply(BACK.restartingProfile(), 1.6, switches.sensors());
    Demand interlocked = BACK.heldTo(BACK.ask(), "turret stays in while elevator is down");
    Demand begun = turret.apply(interlocked, 1.6, switches.sensors());
    Demand lasting = turret.apply(interlocked, 1.6, switches.sensors());

    assertThat(restarting.restartProfile()).as("restarted from above").isTrue();
    assertThat(begun.restartProfile()).as("a new hold").isTrue();
    assertThat(lasting.restartProfile()).isFalse();
    assertThat(lasting.heldBy())
        .isEqualTo("turret stays in while elevator is down; turret right tripped");
  }

  /** Leaving it is free, and once it clears, a move past it is held where it next trips. */
  @Test
  void leavingItIsFree() {
    switches.trip(TURRET_RIGHT, true);
    turret.apply(BACK, 1.6, switches.sensors());

    Demand leaving = turret.apply(FRONT, 1.65, switches.sensors());
    switches.trip(TURRET_RIGHT, false);
    turret.apply(BACK, 1.5, switches.sensors());
    switches.trip(TURRET_RIGHT, true);
    Demand again = turret.apply(BACK, 1.58, switches.sensors());

    assertThat(leaving).isSameAs(FRONT);
    assertThat(Demands.position(again)).isEqualTo(1.58);
  }

  /** An output toward a tripped switch goes neutral; away from it, it's applied. */
  @Test
  void anOutputTowardATrippedSwitchGoesNeutral() {
    switches.trip(TURRET_RIGHT, true);
    Demand toward = Demands.atVoltage(2);
    Demand away = Demands.atDutyCycle(-0.2);

    Demand stopped = turret.apply(toward, 1.6, switches.sensors());

    assertThat(stopped.ask()).isEqualTo(JointAsk.NEUTRAL);
    assertThat(stopped.heldBy()).isEqualTo("turret right tripped");
    assertThat(turret.apply(away, 1.6, switches.sensors())).isSameAs(away);
    assertThat(turret.apply(Demand.NEUTRAL, 1.6, switches.sensors()).heldBy()).isEmpty();
  }

  /** A lost switch holds no condition, so it stops nothing; nor does one never read. */
  @Test
  void aLostSwitchStopsNothing() {
    switches.trip(TURRET_RIGHT, true).lose(TURRET_RIGHT, true);

    assertThat(turret.apply(BACK, 1.6, switches.sensors())).isSameAs(BACK);
  }

  /** The switch homing finds it by: one at that end whose position is known. */
  @Test
  void itsSwitchAtAnEndIsOneWhosePositionIsKnown() {
    assertThat(turret.at(Sensor.Switch.Limit.End.UPPER)).contains(TURRET_RIGHT);
    assertThat(turret.at(Sensor.Switch.Limit.End.LOWER)).isEmpty();
    assertThat(new LimitStops(List.of()).at(Sensor.Switch.Limit.End.UPPER)).isEmpty();
    assertThat(new LimitStops(List.of(TURRET_RIGHT)).at(Sensor.Switch.Limit.End.LOWER))
        .as("a switch at the other end only")
        .isEmpty();
  }

  /** Two tripped at once, a cut wire on one: each says so, and the joint goes neither way. */
  @Test
  void twoTrippedEachSaySo() {
    SwitchReadings both = new SwitchReadings(TURRET_RIGHT, TURRET_LEFT);
    both.trip(TURRET_RIGHT, true).trip(TURRET_LEFT, true);

    Demand right = turret.apply(Demands.atVoltage(2), 0, both.sensors());
    Demand left = turret.apply(Demands.atVoltage(-2), 0, both.sensors());

    assertThat(right.heldBy()).isEqualTo("turret right tripped");
    assertThat(left.heldBy()).isEqualTo("turret left tripped");
  }
}

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
   * A position past a tripped switch holds where the joint was as it tripped, however far it then
   * goes, and says so; its profile starts again from where it is.
   */
  @Test
  void aMovePastATrippedSwitchHoldsWhereItTripped() {
    switches.trip(TURRET_RIGHT, true);

    Demand held = turret.apply(BACK, 1.6, switches.sensors());
    Demand later = turret.apply(BACK, 1.65, switches.sensors());

    assertThat(held)
        .isEqualTo(new Demand(new JointAsk.ToPosition(1.6, 0.5), true, "turret right tripped"));
    assertThat(Demands.position(later)).as("where it tripped").isEqualTo(1.6);
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

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_BACK;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_FRONT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.command3.Command;

/**
 * A tripped limit switch stops its joint in the robot's code, beneath every command: a move past it
 * holds where it tripped, an output toward it goes neutral, and the joint is free to leave. The
 * only stop for a switch its controller can't read; beside the controller's own where it can.
 */
@ExtendWith(RobotSim.class)
class LimitStopSimTest {
  /** At 90° of the turret's swing, on its way to the back. */
  private static final Sensor.Switch TURRET_RIGHT =
      JointFixtures.limit(
          "turret right", TURRET, Sensor.Switch.Limit.End.UPPER, Math.toRadians(90));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(TURRET_RIGHT)).build();

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final JointMechanism turret = sim.joints.of(TURRET);

  private double turretDegrees() {
    return Math.toDegrees(sim.simulation.position(TURRET));
  }

  /**
   * A move to the back holds where the switch tripped, says so, and times out; back home is free.
   */
  @Test
  void aMovePastATrippedSwitchHoldsWhereItTripped() {
    Move back = turret.goTo(TURRET_BACK);
    sim.scheduler.schedule(back);
    sim.runUntil(() -> back.outcome().isPresent(), 5);

    assertThat(back.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(back.reason()).contains("held: turret right tripped");
    assertThat(turretDegrees()).as("stopped past 90°, braking").isBetween(90.0, 100.0);

    Move front = turret.goTo(TURRET_FRONT);
    sim.scheduler.schedule(front);
    sim.runUntil(() -> front.outcome().isPresent(), 3);

    assertThat(front.arrived()).as("leaving it is free").isTrue();
    assertThat(turretDegrees()).isCloseTo(0, within(2.0));
  }

  /**
   * A voltage toward the switch goes neutral once it trips, and braking, the turret stops within
   * what it travels from its speed there: 1.7° past the switch, at 91.7°.
   */
  @Test
  void anOutputTowardATrippedSwitchGoesNeutral() {
    Command push =
        turret
            .run(
                co -> {
                  while (true) {
                    turret.setVoltage(2);
                    co.yield();
                  }
                })
            .named("Push right");
    sim.run(push, 3);

    assertThat(turretDegrees()).isBetween(90.0, 92.0);
    assertThat(turret.heldBy()).isEqualTo("turret right tripped");
  }

  /** A turret run at shares of the battery, either way, its own switch at 90° on its right. */
  private static final Output SPINNING = Output.dutyCycle("spinning", 0.15);

  private static final Output SPINNING_BACK = Output.dutyCycle("spinning back", -0.15);

  private static final Joint.Revolute SPUN =
      JointRevoluteBuilder.builder(TURRET).outputs(List.of(SPINNING, SPINNING_BACK)).build();

  private static final Sensor.Switch SPUN_RIGHT =
      JointFixtures.limit("turret right", SPUN, Sensor.Switch.Limit.End.UPPER, Math.toRadians(90));

  /** Each way an output drives a joint, toward its right (up) or away from it. */
  enum Pushed {
    VOLTAGE,
    VELOCITY,
    DUTY_CYCLE;

    /** The command that drives the turret this way, toward its right switch or away from it. */
    Command toward(JointMechanism turret, boolean right) {
      double sign = right ? 1 : -1;
      return switch (this) {
        case VOLTAGE ->
            turret
                .run(
                    co -> {
                      while (true) {
                        turret.setVoltage(2 * sign);
                        co.yield();
                      }
                    })
                .named("Push at " + 2 * sign + " V");
        case VELOCITY -> turret.follow(() -> RadiansPerSecond.of(2 * sign));
        case DUTY_CYCLE -> turret.run(right ? SPINNING : SPINNING_BACK);
      };
    }
  }

  /** The turret on its tripped right switch, at 95°, still. */
  private static JointsSimHarness onTheSwitch() {
    JointsSimHarness spun =
        new JointsSimHarness(
            RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
                .name("spun")
                .joints(List.of(PIVOT, ROLLER, ELEVATOR, SPUN))
                .sensors(List.of(SPUN_RIGHT))
                .build());
    spun.simulation.place(SPUN, Math.toRadians(95));
    spun.run(0.1);
    return spun;
  }

  /** Each output toward its tripped switch goes neutral, held where it is, and says so. */
  @ParameterizedTest
  @EnumSource(Pushed.class)
  void anOutputTowardATrippedSwitchGoesNeutralWhicheverItIs(Pushed pushed) {
    JointsSimHarness spun = onTheSwitch();
    JointMechanism turret = spun.joints.of(SPUN);

    spun.run(pushed.toward(turret, true), 0.5);

    assertThat(Math.toDegrees(spun.simulation.position(SPUN))).isCloseTo(95, within(0.5));
    assertThat(spun.simulation.velocity(SPUN)).isCloseTo(0, within(0.01));
    assertThat(turret.heldBy()).isEqualTo("turret right tripped");
  }

  /**
   * Each output away from its tripped switch is applied, and moves it off: in half a second, from
   * 95° to 37° at 2 V, to 45° chasing 2 rad/s, and to 43° at 15% of the battery.
   */
  @ParameterizedTest
  @EnumSource(Pushed.class)
  void anOutputAwayFromATrippedSwitchMovesItOff(Pushed pushed) {
    JointsSimHarness spun = onTheSwitch();
    JointMechanism turret = spun.joints.of(SPUN);

    spun.run(pushed.toward(turret, false), 0.5);

    assertThat(Math.toDegrees(spun.simulation.position(SPUN))).isLessThan(50);
    assertThat(turret.heldBy()).isEmpty();
  }
}

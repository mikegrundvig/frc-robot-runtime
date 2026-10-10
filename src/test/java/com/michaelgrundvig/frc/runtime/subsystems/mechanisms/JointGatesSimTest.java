package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_PUSH;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_OUTTAKING;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/** A joint's gates, read from the simulator: each on what the joint measured this loop. */
@ExtendWith(RobotSim.class)
class JointGatesSimTest {
  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);

  /** Sends the pivot to an angle, waits for it to settle, and says whether a gate holds there. */
  private boolean at(Gate gate, double degrees) {
    sim.run(pivot.goTo(Degrees.of(degrees)), 1.2);
    return gate.getAsBoolean();
  }

  /** Into 2° of "out" to be at it, and out past 4° to leave: the enter and stay bands. */
  @Test
  void isAtArrivesInsideTheEnterBandAndLeavesPastTheStayBand() {
    Gate atOut = pivot.isAt(PIVOT_OUT);
    List<Boolean> seen = new ArrayList<>();

    for (double degrees : new double[] {127, 128.6, 127, 125.5, 128.6}) {
      seen.add(at(atOut, degrees));
    }

    assertThat(atOut.name()).isEqualTo("pivot at out");
    assertThat(seen).containsExactly(false, true, true, false, true);
  }

  /**
   * A gate on where the model says a joint is reads what it measures: between two of its places,
   * from one on, or at one within its band, as {@code isAt} is. Named as the condition reads, or as
   * the code names it.
   */
  @Test
  void aGateOnWhereAJointIsReadsWhereItIs() {
    Gate between = sim.joints.gate(PIVOT.between(PIVOT_RUN, PIVOT_OUT));
    Gate out = sim.joints.gate("intake out", PIVOT.atLeast(PIVOT_RUN));
    Gate atOut = sim.joints.gate(PIVOT.at(PIVOT_OUT));
    sim.loop();
    boolean[] stowed = {between.getAsBoolean(), out.getAsBoolean(), atOut.getAsBoolean()};

    boolean[] at120 = {at(between, 120), out.getAsBoolean(), atOut.getAsBoolean()};
    boolean[] at129 = {at(between, 129), out.getAsBoolean(), atOut.getAsBoolean()};
    boolean[] at60 = {at(between, 60), out.getAsBoolean(), atOut.getAsBoolean()};

    assertThat(stowed).containsExactly(false, false, false);
    assertThat(at120).containsExactly(true, true, false);
    assertThat(at129).as("within 2° of out").containsExactly(true, true, true);
    assertThat(at60).containsExactly(false, false, false);
    assertThat(between.name()).isEqualTo("pivot between run and out");
    assertThat(out.name()).isEqualTo("intake out");
    assertThat(atOut.name()).isEqualTo("pivot at out");
  }

  /** A gate on how fast a joint runs reads its speed: a roller up to 1000 rpm once it's run. */
  @Test
  void aGateOnHowFastAJointRunsReadsItsSpeed() {
    Gate spun = sim.joints.gate(ROLLER.upToSpeed(RPM.of(1000)));
    sim.loop();
    boolean still = spun.getAsBoolean();

    sim.run(roller.run(ROLLER_INTAKING), 1);

    assertThat(still).isFalse();
    assertThat(spun.getAsBoolean()).isTrue();
    assertThat(spun.name()).isEqualTo("roller at least 1000 rpm");
  }

  @Test
  void isAtGoalWhileMovingIsFalseAndTrueOnceThere() {
    Gate atGoal = pivot.isAtGoal();
    sim.run(0.1);
    boolean holding = atGoal.getAsBoolean();

    sim.run(pivot.goTo(PIVOT_OUT), 0.2);
    boolean moving = atGoal.getAsBoolean();
    sim.run(1);

    assertThat(holding).as("idle, holding where it rests").isTrue();
    assertThat(moving).isFalse();
    assertThat(atGoal.getAsBoolean()).isTrue();
  }

  @Test
  void aRollerIsAtGoalAtItsSpeedOrStopped() {
    Gate atGoal = roller.isAtGoal();
    Gate atIntaking = roller.isAt(ROLLER_INTAKING);
    sim.run(0.1);
    boolean[] stopped = {atGoal.getAsBoolean(), atIntaking.getAsBoolean()};

    sim.run(roller.run(ROLLER_INTAKING), 1);
    boolean[] intaking = {atGoal.getAsBoolean(), atIntaking.getAsBoolean()};
    sim.run(roller.run(ROLLER_OUTTAKING), 1);

    assertThat(stopped).containsExactly(true, false);
    assertThat(intaking).containsExactly(true, true);
    assertThat(atIntaking.getAsBoolean()).isFalse();
    assertThat(atGoal.getAsBoolean()).as("at its voltage").isTrue();
    assertThat(atIntaking.name()).isEqualTo("roller at intaking");
  }

  /**
   * Pushed up at 3 V, the pivot swings to its top stop and stops there: stalled a quarter of a
   * second later, and not while it was still swinging.
   */
  @Test
  void aJointPushedIntoItsStopIsStalled() {
    Gate stalled = pivot.isStalled();
    sim.run(pivot.run(PIVOT_PUSH), 0.2);
    boolean swinging = stalled.getAsBoolean();

    sim.run(1.5);

    assertThat(swinging).isFalse();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(135, within(0.5));
    assertThat(stalled.getAsBoolean()).isTrue();
    assertThat(stalled.name()).isEqualTo("pivot stalled");
  }

  @Test
  void aConfigurationsGateIsEveryJointAtItsPlace() {
    Gate atIntaking = sim.joints.isAt(INTAKING);
    sim.loop();
    List<String> blocking = atIntaking.blocking();

    sim.run(sim.joints.run(INTAKING), 2);

    assertThat(atIntaking.name()).isEqualTo("at intaking");
    assertThat(blocking).containsExactly("pivot at out", "roller at intaking", "elevator at clear");
    assertThat(atIntaking.getAsBoolean()).isTrue();
  }

  @Test
  void aReadyJointIsReady() {
    Gate ready = pivot.isReady();
    sim.loop();

    assertThat(ready.getAsBoolean()).isTrue();
    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
  }
}

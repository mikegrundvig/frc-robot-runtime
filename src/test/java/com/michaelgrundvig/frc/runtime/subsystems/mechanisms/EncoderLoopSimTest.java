package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.record.MoveReason;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A joint whose loop closes on its absolute encoder on a Systemcore port: the robot works out its
 * motor's voltage each loop from where the encoder says it is, so a motor's count that slipped, by
 * less than the two may disagree before one is taken to have slipped ({@link EncoderSlip}), doesn't
 * send it to the wrong place, as it does a joint whose encoder only told its motor where it was at
 * startup.
 */
@ExtendWith(RobotSim.class)
class EncoderLoopSimTest {
  /** The pivot's through-bore on port 1, mounted reversed and turned a third of a turn. */
  private static final MechanismWiring.PortEncoder PORT_1 =
      MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.3)).inverted();

  /** Its encoder tells its motor where it is, once. */
  private static final Joint.Revolute SEEDED =
      PIVOT.absoluteEncoder(PORT_1).retuned(old -> control(30, 1, profile(8, 30)));

  /** Its loop closes on its encoder. */
  private static final Joint.Revolute CLOSED =
      SEEDED.absoluteEncoder(encoder -> encoder.on(PORT_1.closesLoop()));

  private static JointsSimHarness sim(Joint joint) {
    RobotDefinition robot = ROBOT.with(joint);
    return new JointsSimHarness(robot);
  }

  /** Runs a move to an end, at most 3 s. */
  private static Move run(JointsSimHarness sim, Move move) {
    sim.scheduler.schedule(move);
    sim.runUntil(() -> move.outcome().isPresent(), 3);
    return move;
  }

  /**
   * Its motor's count slips 4° behind the joint, inside the 5° they may disagree: told once where
   * it was, the pivot is sent to its run position by its motor's count, and arrives there by that
   * count 4° past it; closed on its encoder, it arrives at the run position itself, and knows it.
   */
  @Test
  void aSlippedMotorSendsASeededPivotToTheWrongPlaceAndNotOneClosedOnItsEncoder() {
    JointsSimHarness seeded = sim(SEEDED);
    JointsSimHarness closed = sim(CLOSED);
    for (JointsSimHarness each : new JointsSimHarness[] {seeded, closed}) {
      each.run(0.1);
      // The joint's at 5.5°, still, and its motor's count reads where it rests, 1.5°.
      each.simulation.powerOnAt(PIVOT, Math.toRadians(5.5));
      each.run(0.1);
    }

    Move wrong = run(seeded, seeded.joints.of(PIVOT).goTo(PIVOT_RUN));
    Move right = run(closed, closed.joints.of(PIVOT).goTo(PIVOT_RUN));
    closed.run(0.5);

    assertThat(wrong.arrived()).as("by its motor's count").isTrue();
    assertThat(Math.toDegrees(seeded.simulation.position(PIVOT))).isCloseTo(114, within(1.0));
    assertThat(right.arrived()).isTrue();
    assertThat(Math.toDegrees(closed.simulation.position(PIVOT))).isCloseTo(110, within(2.0));
    assertThat(closed.joints.of(PIVOT).position())
        .as("where it says it is, from its encoder")
        .isCloseTo(closed.simulation.position(PIVOT), within(Math.toRadians(0.5)));
  }

  /** An elevator's through-bore on port 2, geared nine to one down from its 1 in drum. */
  private static final Joint.Prismatic ELEVATOR_SEEDED =
      ELEVATOR
          .absoluteEncoder(
              Driven.AbsoluteEncoder.of()
                  .ratio(1.0 / 9)
                  .on(MechanismWiring.PortEncoder.dutyCycle(2).offset(Rotations.of(0.3))))
          .retuned(old -> control(60, 2, profile(1.2, 5)));

  /**
   * An elevator's motor count slips 1.5 cm behind the carriage, inside the 2 cm they may disagree:
   * told once where it was, the elevator is sent to clear, 0.4 m, by its motor's count, and stops
   * 1.5 cm past it; closed on its encoder, geared to its drum, it stops at clear itself, holding
   * its carriage's weight there, and knows it.
   */
  @Test
  void aSlippedMotorSendsASeededElevatorPastItsPlaceAndNotOneClosedOnItsEncoder() {
    JointsSimHarness seeded = sim(ELEVATOR_SEEDED);
    JointsSimHarness closed =
        sim(
            ELEVATOR_SEEDED.absoluteEncoder(
                encoder ->
                    encoder.on(
                        MechanismWiring.PortEncoder.dutyCycle(2)
                            .offset(Rotations.of(0.3))
                            .closesLoop())));
    for (JointsSimHarness each : new JointsSimHarness[] {seeded, closed}) {
      each.run(0.1);
      // The carriage is at 1.5 cm, still, and its motor's count reads where it rests, 0 m.
      each.simulation.powerOnAt(ELEVATOR, 0.015);
      each.run(0.1);
    }

    Move wrong = run(seeded, seeded.joints.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    Move right = run(closed, closed.joints.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    closed.run(1);

    assertThat(wrong.arrived()).as("by its motor's count").isTrue();
    assertThat(seeded.simulation.position(ELEVATOR)).isCloseTo(0.415, within(0.005));
    assertThat(right.arrived()).isTrue();
    assertThat(closed.simulation.position(ELEVATOR)).isCloseTo(0.4, within(0.01));
    assertThat(closed.simulation.velocity(ELEVATOR)).isCloseTo(0, within(0.01));
    assertThat(closed.joints.of(ELEVATOR).position())
        .as("where it says it is, from its encoder")
        .isCloseTo(closed.simulation.position(ELEVATOR), within(0.005));
  }

  /**
   * Closed on its encoder, the pivot holds its place against its weight, the robot's loop running
   * its feedforward, gravity included, as its motor controller's would.
   */
  @Test
  void itHoldsWhereItArrivesAgainstItsWeight() {
    JointsSimHarness sim = sim(CLOSED);
    sim.run(0.1);

    Move out = run(sim, sim.joints.of(PIVOT).goTo(PIVOT_RUN));
    sim.run(2);

    assertThat(out.arrived()).isTrue();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(110, within(1.0));
    assertThat(Math.toDegrees(sim.simulation.velocity(PIVOT))).isCloseTo(0, within(1.0));
  }

  /**
   * The encoder its loop closes on comes unplugged as it moves: it's faulted, its move ends
   * refused, saying why, and the next is refused; it comes to a stop and holds there on its motor's
   * count rather than going limp. Plugged back in, it's ready at once, where the encoder says, and
   * moves again.
   */
  @Test
  void aLostEncoderFaultsItAndItHoldsOnItsMotorUntilTheEncoderAnswers() {
    JointsSimHarness sim = sim(CLOSED);
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.run(0.1);
    Move out = pivot.goTo(PIVOT_OUT);
    sim.scheduler.schedule(out);
    sim.run(0.3);

    sim.simulation.unplug(PIVOT, true);
    sim.loop();
    JointMechanism.State lost = pivot.state();
    Move refused = run(sim, pivot.goTo(PIVOT_RUN));
    sim.run(0.5);
    double stopped = sim.simulation.position(PIVOT);
    sim.run(1);
    double held = sim.simulation.position(PIVOT);
    sim.simulation.unplug(PIVOT, false);
    sim.loop();
    JointMechanism.State back = pivot.state();
    double knows = pivot.position();
    double truly = sim.simulation.position(PIVOT);
    Move again = run(sim, pivot.goTo(PIVOT_RUN));

    String why =
        "pivot's absolute encoder is lost, on port 1: its loop closes on it, so it holds where its"
            + " motor says until the encoder answers";
    assertThat(lost).isEqualTo(JointMechanism.State.FAULTED);
    assertThat(out.outcome()).contains(Outcome.REFUSED);
    ended(out).refused().partWay().because(MoveReason.JOINT_FAULTED).saying(why);
    assertThat(refused.outcome()).contains(Outcome.REFUSED);
    ended(refused).refused().beforeItBegan().because(MoveReason.JOINT_FAULTED).saying(why);
    assertThat(Math.toDegrees(stopped)).as("stopped on its way out").isBetween(60.0, 130.0);
    assertThat(Math.toDegrees(held))
        .as("held where it stopped, not sagging")
        .isCloseTo(Math.toDegrees(stopped), within(0.5));
    assertThat(back).isEqualTo(JointMechanism.State.READY);
    assertThat(knows).isCloseTo(truly, within(Math.toRadians(0.5)));
    assertThat(again.arrived()).isTrue();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(110, within(2.0));
  }
}

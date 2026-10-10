package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A joint whose absolute encoder the robot reads itself, on a Systemcore port or unwired in the
 * simulator: it waits, unhomed, for the encoder's first reading, and tells its motor where it is
 * from it, wherever it was powered on.
 */
@ExtendWith(RobotSim.class)
class AbsoluteSeedingSimTest {
  /** The pivot's through-bore on port 1, mounted reversed and turned a third of a turn. */
  private static final MechanismWiring.PortEncoder PORT_1 =
      MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.3)).inverted();

  private static final Joint.Revolute ON_PORT =
      PIVOT.absoluteEncoder(PORT_1).retuned(old -> control(30, 1, profile(8, 30)));

  private static JointsSimHarness sim(Joint joint) {
    RobotDefinition robot = ROBOT.with(joint);
    return new JointsSimHarness(robot);
  }

  /**
   * Powered on at 60°, its motor reads it at its reference, 1.5°; the encoder's first reading tells
   * it 60°, in the same loop, and a move takes it to the real 110°.
   */
  @Test
  void itsEncoderTellsItsMotorWhereItWasPoweredOn() {
    JointsSimHarness sim = sim(ON_PORT);
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(60));

    sim.loop();
    JointMechanism.State first = pivot.state();
    sim.loop();
    double seeded = pivot.position();
    Move run = pivot.goTo(PIVOT_RUN);
    sim.scheduler.schedule(run);
    sim.runUntil(() -> run.outcome().isPresent(), 3);

    assertThat(first).isEqualTo(JointMechanism.State.READY);
    assertThat(seeded).isCloseTo(Math.toRadians(60), within(Math.toRadians(0.5)));
    assertThat(run.arrived()).isTrue();
    assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(110), within(0.02));
  }

  /**
   * Powered on at its top stop, 135°, the far end of its range from where it rests: the encoder's
   * reading there is inside the window its turn tells apart, and seeds it at 135°, not a turn off,
   * so a move to "in" brings it down to the real 1.5°.
   */
  @Test
  void poweredOnAtItsTopStopItsEncoderTellsItSo() {
    JointsSimHarness sim = sim(ON_PORT);
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(135));

    sim.loop();
    sim.loop();
    double seeded = pivot.position();
    Move in = pivot.goTo(PIVOT_IN);
    sim.scheduler.schedule(in);
    sim.runUntil(() -> in.outcome().isPresent(), 3);

    assertThat(Math.toDegrees(seeded)).isCloseTo(135, within(0.5));
    assertThat(in.arrived()).isTrue();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(1.5, within(1.5));
  }

  /**
   * An elevator's through-bore on port 2, geared nine to one down from its 1 in drum, so its 1.2 m
   * is 0.835 of the encoder's turns: powered on at 0.7 m, its motor reads it where it rests, 0 m,
   * and the encoder's first reading tells it 0.7 m, so a move to clear goes down to the real 0.4 m.
   */
  @Test
  void anElevatorsEncoderGearedToItsDrumTellsItsMotorWhereItWasPoweredOn() {
    JointsSimHarness sim =
        sim(
            ELEVATOR
                .absoluteEncoder(
                    Driven.AbsoluteEncoder.of()
                        .ratio(1.0 / 9)
                        .on(MechanismWiring.PortEncoder.dutyCycle(2).offset(Rotations.of(0.3))))
                .retuned(old -> control(60, 2, profile(1.2, 5))));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 0.7);

    sim.loop();
    JointMechanism.State first = elevator.state();
    sim.loop();
    double seeded = elevator.position();
    Move clear = elevator.goTo(ELEVATOR_CLEAR);
    sim.scheduler.schedule(clear);
    sim.runUntil(() -> clear.outcome().isPresent(), 3);

    assertThat(first).isEqualTo(JointMechanism.State.READY);
    assertThat(seeded).isCloseTo(0.7, within(0.005));
    assertThat(clear.arrived()).isTrue();
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0.4, within(0.01));
  }

  /** An encoder no one wires yet is the simulator's to read: the joint is seeded all the same. */
  @Test
  void anUnwiredEncoderIsReadInTheSimulator() {
    JointsSimHarness sim =
        sim(PIVOT.absoluteEncoder().retuned(old -> control(30, 1, profile(8, 30))));
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(80));

    sim.run(0.1);

    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(pivot.position()).isCloseTo(Math.toRadians(80), within(Math.toRadians(0.5)));
  }

  /**
   * Unplugged, it waits, unpowered, and sags onto its stop; after two seconds it's faulted and says
   * so; plugged in, its first reading seeds it where it really is, and it's ready.
   */
  @Test
  void withoutAReadingItWaitsThenFaultsUntilOneComes() {
    JointsSimHarness sim = sim(ON_PORT);
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.simulation.unplug(PIVOT, true);
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(45));

    sim.run(1);
    JointMechanism.State waiting = pivot.state();
    sim.run(1.2);
    String fault = pivot.fault();
    double lost = pivot.position() - sim.simulation.position(PIVOT);
    sim.simulation.unplug(PIVOT, false);
    sim.run(0.1);

    assertThat(waiting).isEqualTo(JointMechanism.State.UNHOMED);
    assertThat(fault)
        .isEqualTo(
            "pivot's absolute encoder gave no reading in 2.0 s, on port 1: it's seeded when it"
                + " does");
    assertThat(Math.abs(lost)).as("its motor's reading, unseeded").isGreaterThan(0.5);
    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(pivot.position())
        .isCloseTo(sim.simulation.position(PIVOT), within(Math.toRadians(0.5)));
  }

  /** Its motor's count slipped: homing it again seeds it again from the encoder. */
  @Test
  void homingItAgainSeedsItAgain() {
    JointsSimHarness sim = sim(ON_PORT);
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.run(0.1);
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(70)); // the motor's count lost: it reads 1.5°

    sim.run(0.1);
    double slipped = pivot.position();
    Move home = pivot.home();
    sim.scheduler.schedule(home);
    sim.runUntil(() -> home.outcome().isPresent(), 1);
    sim.run(0.05);

    assertThat(slipped).isCloseTo(Math.toRadians(1.5), within(Math.toRadians(0.5)));
    assertThat(home.arrived()).isTrue();
    assertThat(pivot.position())
        .isCloseTo(sim.simulation.position(PIVOT), within(Math.toRadians(0.5)));
    assertThat(pivot.position()).isCloseTo(Math.toRadians(70), within(Math.toRadians(3)));
  }
}

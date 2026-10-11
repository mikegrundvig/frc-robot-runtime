package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;

/**
 * Switches and rangefinders read each loop with the joints, and behavior reading them through the
 * conditions the model names, as gates: ending a move, holding a step, starting a command. A sensor
 * that's lost holds no condition either way, and what waited on it says so.
 */
@ExtendWith(RobotSim.class)
class SensorGatesSimTest {
  private static final Sensor.Switch CLAW_BEAM = Sensor.Switch.beam("claw beam", PIVOT);

  /** At the pivot's bottom, 2°: it rests on it, at 1.5°. */
  private static final Sensor.Switch PIVOT_DOWN =
      JointFixtures.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Math.toRadians(2));

  private static final Sensor.Rangefinder HOPPER =
      Sensor.Rangefinder.builder()
          .label("hopper range")
          .part(Part.CHASSIS)
          .range(Meters.of(1))
          .fieldOfView(Degrees.of(27))
          .build();

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .sensors(List.of(CLAW_BEAM, PIVOT_DOWN, HOPPER))
          .build();

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final Sensors sensors = sim.joints.sensors();
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);
  private final Gate coralIn = sim.joints.gate("coral in claw", CLAW_BEAM.tripped());

  /** A beam's gate opens the loop it trips, and its other side the loop it clears. */
  @Test
  void aBeamsGateOpensTheLoopItTrips() {
    Gate clear = sim.joints.gate(CLAW_BEAM.clear());
    sim.loop();
    assertThat(coralIn.getAsBoolean()).isFalse();
    assertThat(coralIn.blocking()).as("read, and clear: nothing to blame").isEmpty();
    assertThat(clear.name()).isEqualTo("claw beam clear");
    assertThat(clear.getAsBoolean()).isTrue();

    sim.sensors.trip(CLAW_BEAM, true);
    sim.loop();

    assertThat(coralIn.getAsBoolean()).isTrue();
    assertThat(clear.getAsBoolean()).isFalse();
  }

  /** A roller runs until the beam trips, and ends ARRIVED that loop, its name saying on what. */
  @Test
  void aMoveEndsTheLoopTheBeamTrips() {
    Move intake = roller.runUntil(ROLLER_INTAKING, coralIn);
    sim.scheduler.schedule(intake);
    sim.run(0.5);
    assertThat(intake.outcome()).isEmpty();

    sim.sensors.trip(CLAW_BEAM, true);
    sim.loop();

    assertThat(intake.name()).isEqualTo("roller: intaking until coral in claw");
    assertThat(intake.arrived()).isTrue();
    assertThat(intake.reason())
        .as("the loop it trips, 25 loops on")
        .isEqualTo("roller: intaking until coral in claw: ARRIVED in 0.50 s");
  }

  /**
   * A beam whose CAN device stops answering holds neither way: a gate on it never opens, and says
   * the sensor is lost. A move waiting on it that was given a deadline times out saying so, and so
   * never runs its roller forever; without one it would run until canceled.
   */
  @Test
  void aLostSensorHoldsNeitherWayAndWhatWaitedOnItSaysSo() {
    Gate clear = sim.joints.gate(CLAW_BEAM.clear());
    sim.sensors.silence(CLAW_BEAM, true);
    Move intake = roller.runUntil(ROLLER_INTAKING, coralIn).within(Seconds.of(3));
    sim.scheduler.schedule(intake);

    sim.run(3.1);

    assertThat(coralIn.getAsBoolean()).isFalse();
    assertThat(clear.getAsBoolean()).isFalse();
    assertThat(coralIn.blocking()).containsExactly("claw beam lost");
    assertThat(sensors.lost()).containsExactly("claw beam");
    assertThat(intake.reason())
        .isEqualTo(
            "roller: intaking until coral in claw: TIMED_OUT after 3.00 s: coral in claw is false:"
                + " claw beam lost");
  }

  /**
   * A limit switch trips where its joint really is: the pivot rests on its bottom switch, and
   * leaves it on the way out. A test can't trip it by hand.
   */
  @Test
  void aLimitSwitchTripsWhereItsJointReallyIs() {
    Gate down = sim.joints.gate(PIVOT_DOWN.tripped());
    sim.loop();
    assertThat(down.getAsBoolean()).isTrue();

    sim.run(pivot.goTo(PIVOT_OUT), 1.5);

    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(130, within(2.0));
    assertThat(down.getAsBoolean()).isFalse();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> sim.sensors.trip(PIVOT_DOWN, true))
        .withMessage("pivot down trips where pivot really is: move it there");
  }

  /** A rangefinder that sees nothing reads farther than anything; something near reads nearer. */
  @Test
  void aRangefinderReadsNearerOrFarther() {
    Gate full = sim.joints.gate("hopper full", HOPPER.nearerThan(Inches.of(6)));
    Gate empty = sim.joints.gate(HOPPER.fartherThan(Meters.of(0.5)));
    sim.loop();
    assertThat(full.getAsBoolean()).isFalse();
    assertThat(empty.getAsBoolean()).isTrue();

    sim.sensors.see(HOPPER, Inches.of(4));
    sim.loop();

    assertThat(full.getAsBoolean()).isTrue();
    assertThat(empty.getAsBoolean()).isFalse();
  }

  /**
   * A gate is a trigger: a command bound to its rising edge runs once a trip, and a step waiting on
   * it goes on the loop it opens.
   */
  @Test
  void aTripStartsACommandOnceAndReleasesAWait() {
    int[] ran = {0};
    boolean[] waited = {false};
    coralIn.onTrue(Command.noRequirements(co -> ran[0]++).named("Count a piece"));
    sim.scheduler.schedule(
        Command.noRequirements(
                co -> {
                  co.waitUntil(coralIn);
                  waited[0] = true;
                })
            .named("Wait for a piece"));
    sim.run(0.2);
    assertThat(ran[0]).isZero();
    assertThat(waited[0]).isFalse();

    sim.sensors.trip(CLAW_BEAM, true);
    sim.run(0.2);
    sim.sensors.trip(CLAW_BEAM, false);
    sim.run(0.2);
    sim.sensors.trip(CLAW_BEAM, true);
    sim.run(0.2);

    assertThat(ran[0]).as("once a trip").isEqualTo(2);
    assertThat(waited[0]).isTrue();
  }

  /**
   * A gate on a sensor the robot doesn't have is refused as it's made: as the robot starts, it
   * throws; once it runs, it holds false, saying why.
   */
  @Test
  void aGateOnAStrangersSensorIsRefused() {
    Sensor.Switch stray = Sensor.Switch.beam("stray", PIVOT);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> sim.joints.gate(stray.tripped()))
        .withMessage("stray isn't one of the robot's sensors");
    sim.loop();

    Gate gate = sim.joints.gate("stray in", stray.tripped());
    sim.loop();

    assertThat(gate.getAsBoolean()).isFalse();
    assertThat(gate.why()).isEqualTo("stray in is false: stray isn't one of the robot's sensors");
  }
}

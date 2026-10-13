package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * Interlocks on what a sensor reads, enforced beneath every command as those on a joint's position
 * are: a roller that runs only while its beam is clear, a pivot kept out of a band while something
 * is near. A lost sensor restricts, as a joint that doesn't know where it is does.
 */
@ExtendWith(RobotSim.class)
class SensorInterlocksSimTest {
  private static final Sensor.Switch CLAW_BEAM = Sensor.Switch.beam("claw beam", PIVOT);

  private static final Sensor.Rangefinder REEF =
      Sensor.Rangefinder.builder()
          .label("reef range")
          .part(Part.CHASSIS)
          .range(Meters.of(1))
          .fieldOfView(Degrees.of(27))
          .build();

  private static final Interlock ROLLER_ONLY_WHILE_CLEAR =
      Interlock.that(ROLLER).runsOnlyWhen(CLAW_BEAM.clear());

  private static final Interlock PIVOT_AVOIDS_NEAR_REEF =
      Interlock.that(PIVOT).keepsOut(Range.degrees(60, 90)).when(REEF.nearerThan(Meters.of(0.3)));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .sensors(List.of(CLAW_BEAM, REEF))
          .interlocks(List.of(ROLLER_ONLY_WHILE_CLEAR, PIVOT_AVOIDS_NEAR_REEF))
          .build();

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);

  /** The roller runs while the beam is clear, stops as it trips, and stops with it lost. */
  @Test
  void aRollerRunsOnlyWhileItsBeamIsClear() {
    sim.scheduler.schedule(roller.run(ROLLER_INTAKING));
    sim.run(1);
    double running = sim.simulation.velocity(ROLLER);

    sim.sensors.trip(CLAW_BEAM, true);
    sim.run(1);
    double tripped = sim.simulation.velocity(ROLLER);
    List<Interlock> heldBy = Held.interlocksHolding(roller);

    sim.sensors.trip(CLAW_BEAM, false);
    sim.sensors.silence(CLAW_BEAM, true);
    sim.run(1);

    assertThat(running).as("1500 rpm through 3:1, in rad/s").isGreaterThan(40);
    assertThat(tripped).isCloseTo(0, within(1.0));
    assertThat(heldBy).containsExactly(ROLLER_ONLY_WHILE_CLEAR);
    assertThat(sim.simulation.velocity(ROLLER)).as("lost: held").isCloseTo(0, within(1.0));
  }

  /**
   * The pivot on its way out waits at the band's edge while the reef is near, and goes on once it's
   * clear.
   */
  @Test
  void aPivotKeepsOutOfABandWhileSomethingIsNear() {
    sim.sensors.see(REEF, Meters.of(0.2));
    Move out = pivot.goTo(PIVOT_OUT).within(Seconds.of(5));
    sim.scheduler.schedule(out);
    sim.run(1.5);
    double waiting = Math.toDegrees(sim.simulation.position(PIVOT));
    List<Interlock> heldBy = Held.interlocksHolding(pivot);

    sim.sensors.see(REEF, Meters.of(0.8));
    sim.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(waiting).as("at the band's edge").isCloseTo(60, within(2.0));
    assertThat(heldBy).containsExactly(PIVOT_AVOIDS_NEAR_REEF);
    assertThat(out.arrived()).isTrue();
  }

  /**
   * A pivot swinging out as the reef comes near, too close to the band to stop short of it,
   * overshoots into it and holds where it would stop as it entered, a point worked out once: never
   * on through. Worked out again each loop from how fast it still went, the hold ran ahead of a
   * joint its loop hadn't yet slowed, and an elevator coming down ran through its whole region to
   * its stop.
   */
  @Test
  void aJointOvershootingIntoAClosedBandHoldsWhereItWouldStop() {
    Move out = pivot.goTo(PIVOT_OUT).atSpeed(Percent.of(30)).within(Seconds.of(5));
    sim.scheduler.schedule(out);
    sim.runUntil(() -> Math.toDegrees(sim.simulation.position(PIVOT)) > 58, 3);
    sim.sensors.see(REEF, Meters.of(0.2));
    sim.runUntil(() -> Math.toDegrees(sim.simulation.position(PIVOT)) > 60, 1);
    double entering = Math.toDegrees(sim.simulation.velocity(PIVOT));
    sim.loop();
    double held = pivot.goal();
    List<Double> goals = new ArrayList<>();
    for (int i = 0; i < 50; i++) {
      sim.loop();
      goals.add(pivot.goal());
    }

    assertThat(entering).as("it enters the band moving, in °/s").isGreaterThan(20);
    assertThat(Math.toDegrees(held)).as("inside the band").isBetween(60.0, 90.0);
    assertThat(goals).as("one point, worked out once").containsOnly(held);
    assertThat(sim.simulation.position(PIVOT)).isCloseTo(held, within(Math.toRadians(1)));
    assertThat(Held.interlocksHolding(pivot)).containsExactly(PIVOT_AVOIDS_NEAR_REEF);
  }

  /** An interlock on a sensor the robot doesn't have is refused as the definition is built. */
  @Test
  void anInterlockOnAStrangersSensorIsRefused() {
    Sensor.Switch stray = Sensor.Switch.beam("stray beam", PIVOT);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(ROBOT)
                    .interlocks(List.of(Interlock.that(ROLLER).runsOnlyWhen(stray.clear())))
                    .build())
        .withMessageContaining(
            "interlock \"roller runs only when stray beam clear\"'s sensor, stray beam, isn't one"
                + " of the robot's sensors: list it");
  }
}

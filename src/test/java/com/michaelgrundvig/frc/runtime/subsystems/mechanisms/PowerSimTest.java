package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;

/**
 * The PDH is read with the joints, once a loop: in the simulator, each channel reads what the
 * simulated motor it feeds draws from the battery. A motor controller that reads as drawing nothing
 * there while its joint works is flagged.
 */
@ExtendWith(RobotSim.class)
class PowerSimTest {
  static final Position ARM_DOWN = Position.degrees("down", 0);

  /** Pushes the arm down into its stop, where its motors draw what their limit lets them. */
  static final Output PUSH = Output.volts("push", -3);

  /**
   * What each of the arm's Krakens draws from the battery pushed into its stop at 3 V: stalled,
   * held to its 40 A, so 40 A through its windings at 40 A's worth of volts across them, 1.3 V,
   * about 4.4 A.
   */
  static final double STALLED = 40 * 40 * DCMotor.getKrakenX60(1).R / 12;

  /** An arm on two Krakens, fed from channels 4 and 5, stowed by hand against its lower stop. */
  static final Joint.Revolute ARM =
      Joint.pivot("arm", Part.CHASSIS, new Translation3d(0.2, 0, 0.4), Joint.PITCH_UP)
          .range(Range.degrees(0, 90))
          .rest(ARM_DOWN)
          .positions(ARM_DOWN)
          .outputs(PUSH)
          .motors(
              Motor.KRAKEN_X60
                  .through(Gearing.of(50))
                  .on(
                      TALON_FX.can(11).pdh(4, Amps.of(40)),
                      TALON_FX.can(12).clockwise().pdh(5, Amps.of(40))))
          .control(
              MechanismControl.position(30)
                  .withKg(0)
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  /** The template's robot with the arm, and its PDH. */
  static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
          .name("power")
          .pdh(Pdh.can(1))
          .joints(List.of(ARM))
          .build();

  @Test
  void eachChannelReadsWhatItsMotorDraws() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT);
    Power power = sim.joints.power();

    sim.run(sim.joints.of(ARM).run(PUSH), 0.5);

    double each = sim.simulation.supplyAmps(ARM);
    assertThat(power.reads()).isTrue();
    assertThat(each).as("pushed into its stop, each motor draws").isCloseTo(STALLED, within(1e-9));
    assertThat(power.amps(4)).isCloseTo(each, within(1e-9));
    assertThat(power.amps(5)).isCloseTo(each, within(1e-9));
    assertThat(power.amps(0)).as("a channel that feeds nothing").isZero();
    assertThat(power.totalAmps()).isCloseTo(2 * each, within(1e-9));
  }

  /**
   * A test can zero a channel's reading, as a tripped breaker leaves it: the PDH reads nothing
   * there, and the other channel reads on.
   */
  @Test
  void aZeroedChannelReadsNothing() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT);
    sim.pdh.zero(5, true);

    sim.run(sim.joints.of(ARM).run(PUSH), 0.5);

    assertThat(sim.joints.power().amps(5)).isZero();
    assertThat(sim.joints.power().amps(4)).isCloseTo(STALLED, within(1e-9));

    sim.pdh.zero(5, false);
    sim.loop();

    assertThat(sim.joints.power().amps(5)).isCloseTo(STALLED, within(1e-9));
  }

  /** What the arm's follower is flagged for, with its channel zeroed while the arm works. */
  static final String FOLLOWER_DEAD =
      "arm motor 12 draws nothing on channel 5 while arm's motors work: its breaker tripped, its"
          + " power is unplugged, or its block names the wrong channel";

  /**
   * A follower's breaker trips: its leader still drives the joint, and answers, so nothing else
   * says so. Its channel reads nothing while the leader draws: past half a second, it's flagged,
   * and it clears the loop its channel draws again.
   */
  @Test
  void aControllerDrawingNothingWhileItsJointWorksIsFlagged() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT);
    sim.scheduler.schedule(sim.joints.of(ARM).run(PUSH));
    sim.run(0.2);
    sim.pdh.zero(5, true);

    sim.run(0.48);
    assertThat(sim.joints.deadChannels()).as("not yet half a second").isEmpty();

    sim.run(0.04);
    assertThat(sim.joints.deadChannels()).containsExactly(FOLLOWER_DEAD);

    sim.pdh.zero(5, false);
    sim.loop();
    assertThat(sim.joints.deadChannels()).isEmpty();
  }

  /**
   * A joint that isn't working draws next to nothing on every channel, alive or not: nothing is
   * flagged. Nor is a roller spinning freely at a steady speed, which the simulator, with no
   * friction, has drawing nothing at all, though it's asked for volts.
   */
  @Test
  void nothingIsFlaggedWhileTheJointDrawsNothing() {
    JointsSimHarness idle = new JointsSimHarness(ROBOT);
    idle.pdh.zero(5, true);
    idle.run(1);

    Output spinning = Output.volts("spinning", 6);
    Joint.Revolute roller =
        Joint.roller("roller", Part.CHASSIS, new Translation3d(0.3, 0, 0.3))
            .outputs(spinning)
            .motors(
                Motor.KRAKEN_X60.through(Gearing.of(3)).on(TALON_FX.can(21).pdh(6, Amps.of(40))))
            .control(MechanismControl.velocity(0.02));
    JointsSimHarness free =
        new JointsSimHarness(
            RobotDefinitionBuilder.builder(ROBOT).joints(List.of(ARM, roller)).build());
    free.run(free.joints.of(roller).run(spinning), 2);

    assertThat(idle.joints.deadChannels()).isEmpty();
    assertThat(free.joints.power().amps(6)).as("spinning freely").isLessThan(0.5);
    assertThat(free.joints.deadChannels()).isEmpty();
  }

  /** How long, and at what currents, is the joint's control's to say. */
  @Test
  void theJointsControlSaysWhenAChannelIsDead() {
    JointsSimHarness sim =
        new JointsSimHarness(
            ROBOT.with(
                ARM.retuned(
                    control ->
                        control.withDeadChannel(Amps.of(2), Amps.of(0.5), Seconds.of(0.1)))));
    sim.scheduler.schedule(sim.joints.of(ARM).run(PUSH));
    sim.run(0.2);
    sim.pdh.zero(5, true);

    sim.run(0.12);

    assertThat(sim.joints.deadChannels()).containsExactly(FOLLOWER_DEAD);
  }

  /** A robot that declares no PDH reads none: nothing is known of any channel. */
  @Test
  void aRobotWithNoPdhReadsNone() {
    JointsSimHarness sim =
        new JointsSimHarness(
            RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                .joints(List.of(ARM.motors(motors -> motors.on())))
                .build());

    sim.run(0.1);

    assertThat(sim.joints.power().reads()).isFalse();
    assertThat(sim.joints.power().amps(4)).isNaN();
    assertThat(sim.joints.power().totalAmps()).isNaN();
  }
}

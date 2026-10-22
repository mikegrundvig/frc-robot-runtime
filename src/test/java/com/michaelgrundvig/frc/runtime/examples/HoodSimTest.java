package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.examples.ServoHoodRobot.HOOD;
import static com.michaelgrundvig.frc.runtime.examples.ServoHoodRobot.HOOD_DOWN;
import static com.michaelgrundvig.frc.runtime.examples.ServoHoodRobot.HOOD_LOB;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Driven.LimitSwitch;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.MechanismWiring.PortEncoder;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Sensor.Switch.Limit.End;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.hardware.discrete.PWM;
import org.wpilib.simulation.PWMSim;

/**
 * The escape hatch the guide teaches ("When the template can't drive it"): a hood the template
 * doesn't drive, kept in the definition, and driven by its own subsystem ({@link Hood}) from what
 * the model says of it.
 */
@ExtendWith(RobotSim.class)
class HoodSimTest {
  private static final RobotDefinition ROBOT = ServoHoodRobot.ROBOT.definition();

  private final PWM left = new PWM(0);
  private final PWM right = new PWM(1);
  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final Hood hood = new Hood(sim.scheduler, ROBOT, left, right);

  @AfterEach
  void freeThePorts() {
    left.close();
    right.close();
  }

  /** The robot starts with the hood in it, undriven: nothing refused, and the template says why. */
  @Test
  void theTemplateLeavesTheHoodToItsOwnSubsystem() {
    assertThat(ROBOT.refusals()).isEmpty();
    assertThat(Joints.problems(ROBOT)).isEmpty();
    assertThatThrownBy(() -> sim.joints.of(HOOD))
        .hasMessage(
            "hood has no mechanism: it's moved by hood left and hood right, and the template drives"
                + " only motors turning a shaft so far");
  }

  /**
   * Lob, 30°: each servo is told the share of its travel the linkage puts there, 0.921, as a pulse
   * of 1921 µs. The first time, the hood is assumed at the far end of its range, 0°, so the move
   * ends once the servos have had 0.85 s, their travel from there at 32 mm/s. Back down, they're
   * told 0.65, 1650 µs, and take as long.
   */
  @Test
  void goingToAPlaceTellsEachServoItsPulseAndWaitsForItsTravel() {
    Command lob = hood.goTo(HOOD_LOB);
    sim.scheduler.schedule(lob);
    double lobbed = sim.runUntil(() -> !sim.scheduler.isScheduledOrRunning(lob), 2);
    int lobPulse = new PWMSim(left).getPulseMicrosecond();
    Command down = hood.goTo(HOOD_DOWN);
    sim.scheduler.schedule(down);
    double lowered = sim.runUntil(() -> !sim.scheduler.isScheduledOrRunning(down), 2);

    assertThat(lobPulse).isEqualTo(1921);
    assertThat(lobbed).isCloseTo(0.85, within(0.03));
    assertThat(new PWMSim(left).getPulseMicrosecond()).isEqualTo(1650);
    assertThat(new PWMSim(right).getPulseMicrosecond()).isEqualTo(1650);
    assertThat(lowered).isCloseTo(0.85, within(0.03));
  }

  /** A practice robot whose hood lobs at 20° tells its servos the share that puts it there. */
  @Test
  void aPracticeRobotsHoodGoesToItsOwnPlace() {
    RobotDefinition practice =
        ServoHoodRobot.ROBOT
            .named("practice")
            .with(HOOD.replaced(HOOD_LOB, Degrees.of(20)))
            .definition();
    Hood its = new Hood(sim.scheduler, practice, left, right);

    sim.run(its.goTo(HOOD_LOB), 0.1);

    assertThat(new PWMSim(left).getPulseMicrosecond()).isEqualTo(1835);
    assertThat(new PWMSim(right).getPulseMicrosecond()).isEqualTo(1835);
  }

  /**
   * A limit switch at an undriven joint's end, on a Systemcore port, is read with the robot's
   * sensors and is a gate, as any switch is; in the simulator it's a test's to trip, since the
   * joint isn't simulated.
   */
  @Test
  void anUndrivenJointsSwitchIsAGate() {
    Joint.Revolute withSwitch =
        HOOD.limitSwitch(
            LimitSwitch.lower().at(Degrees.of(0)).on(SensorWiring.port(3).normallyClosed()));
    JointsSimHarness switched =
        new JointsSimHarness(
            RobotDefinitionBuilder.builder(ROBOT).joints(List.of(withSwitch)).build());
    Gate down = switched.joints.gate("hood down", withSwitch.switchAt(End.LOWER).tripped());

    switched.loop();
    boolean before = down.getAsBoolean();
    switched.sensors.trip(withSwitch.switchAt(End.LOWER), true);
    switched.loop();

    assertThat(before).isFalse();
    assertThat(down.getAsBoolean()).isTrue();
  }

  /**
   * An absolute encoder on a Systemcore port, in an undriven joint's block, is the team's to read:
   * the template reads none of it, and the model turns what the port reads into where the joint is.
   * A CANcoder is read through its joint's motor controller, so an undriven joint can't have one.
   */
  @Test
  void anUndrivenJointsPortEncoderIsReadThroughTheModel() {
    Joint.Revolute withEncoder =
        HOOD.absoluteEncoder(PortEncoder.dutyCycle(2).offset(Rotations.of(0.1)));
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT).joints(List.of(withEncoder)).build();
    RobotMath math = robot.math();
    MechanismWiring.PortEncoder port =
        withEncoder.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port).orElseThrow();
    Sensor.Encoder encoder = Objects.requireNonNull(math.absoluteEncoder(withEncoder));
    double raw = port.raw(Degrees.of(30).in(Rotations)); // what the port reads, the hood at 30°

    assertThat(robot.refusals()).isEmpty();
    assertThat(new JointsSimHarness(robot).joints.all())
        .as("no mechanism, and its encoder never asked for")
        .isEmpty();
    assertThat(Math.toDegrees(math.fromAbsolute(encoder, port.read(raw))))
        .isCloseTo(30, within(1e-9));
    assertThatThrownBy(
            () ->
                RobotDefinitionBuilder.builder(ROBOT)
                    .joints(List.of(HOOD.absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0)))))
                    .build())
        .hasMessageEndingWith(
            "hood's absolute encoder is wired, and its motors aren't: it's read through their"
                + " controller, so wire them too; a joint your own code drives reads one on a"
                + " Systemcore port instead, MechanismWiring.PortEncoder");
  }
}

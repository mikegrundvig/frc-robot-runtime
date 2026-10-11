package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_FRONT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.homed;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Driven.LimitSwitch;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Sensor.Switch.Limit.End;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A joint homed to its limit switch: driven gently toward it on the first enable, it's where the
 * switch says the loop it trips. One that starts on it backs off first, so it's home where the
 * switch trips, not wherever past it it rested.
 */
@ExtendWith(RobotSim.class)
class SwitchHomingSimTest {
  /** The pivot's bottom switch, tripping at 5°: above where it rests, at 1.5°. */
  private static final Sensor.Switch PIVOT_DOWN =
      JointFixtures.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Math.toRadians(5));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(PIVOT_DOWN)).build();

  /** The turret's home switch, at -90°: it trips at or past it, turning left. */
  private static final Sensor.Switch TURRET_HOME =
      JointFixtures.limit(
          "turret home", TURRET, Sensor.Switch.Limit.End.LOWER, Math.toRadians(-90));

  private static final RobotDefinition TURRET_ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(TURRET_HOME)).build();

  /** A robot with its pivot homed to its lower switch. */
  private static RobotDefinition homingPivot(RobotDefinition robot) {
    return robot.with(
        PIVOT.retuned(control -> homed(control, Homing.toSwitch(Sensor.Switch.Limit.End.LOWER))));
  }

  private final JointsSimHarness sim = new JointsSimHarness(homingPivot(ROBOT));
  private final JointMechanism pivot = sim.joints.of(PIVOT);

  private static double degrees(double radians) {
    return Math.toDegrees(radians);
  }

  /**
   * The turret, powered on 40° to the right of where it reads (0°), homes to its switch at -90° at
   * 1 V: driven left until the switch trips, it takes that as -90°, and the move to the front waits
   * for it, then lands at the real 0°, within a loop's travel at the homing's speed.
   */
  @Test
  void itHomesWhereItsSwitchTripsAndMovesWaitForIt() {
    JointsSimHarness turretSim =
        new JointsSimHarness(
            TURRET_ROBOT.with(
                TURRET.retuned(
                    control ->
                        homed(
                            control,
                            Homing.ToSwitch.builder()
                                .end(Sensor.Switch.Limit.End.LOWER)
                                .volts(1)
                                .timeout(Seconds.of(5))
                                .build()))));
    JointMechanism turret = turretSim.joints.of(TURRET);
    turretSim.simulation.powerOnAt(TURRET, Math.toRadians(-40));
    Move front = turret.goTo(TURRET_FRONT).within(Seconds.of(6));
    turretSim.scheduler.schedule(front);

    turretSim.runUntil(() -> front.outcome().isPresent(), 8);

    assertThat(front.arrived()).isTrue();
    assertThat(degrees(turretSim.simulation.position(TURRET))).isCloseTo(0, within(1.5));
  }

  /**
   * Powered on resting at 1.5°, past where its switch trips, it backs off until the switch clears,
   * then comes back until it trips. Falling back under gravity and its 2 V, it travels 4° a loop,
   * so it reads within half of that: gentler homing is closer.
   */
  @Test
  void oneStartingOnItsSwitchBacksOffFirst() {
    double highest = 0;
    for (int i = 0; i < 100; i++) {
      sim.loop();
      highest = Math.max(highest, degrees(sim.simulation.position(PIVOT)));
    }

    assertThat(highest).as("off the switch first").isGreaterThan(5);
    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(degrees(pivot.position()))
        .isCloseTo(degrees(sim.simulation.position(PIVOT)), within(2.2));
  }

  /**
   * With its switch lost, homing never ends well: the joint's faulted, saying why, once the time
   * worked out from its travel has passed, and saying that time: twice the 0.56 s its model crosses
   * its 135° in at 2 V, and half a second more, 1.625 s from its first loop.
   */
  @Test
  void aLostSwitchFaultsItsHoming() {
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(40));
    sim.sensors.silence(PIVOT_DOWN, true);

    double took = sim.runUntil(() -> pivot.state() == JointMechanism.State.FAULTED, 3);

    assertThat(took).as("the first loop past 1.625 s from the first").isCloseTo(1.66, within(1e-9));
    assertThat(pivot.fault().orElse(null))
        .isEqualTo(
            "pivot's homing didn't trip pivot down in 1.6 s (its travel takes 0.6 s at 2.0 V):"
                + " pivot down is lost");
  }

  /**
   * A joint homes by the recipe in its block, which its mechanism reads once as it is built. A
   * control changed by live tuning brings new gains and limits and nothing else: handed one that
   * names a homing to a switch the pivot does not have, the pivot is not homed by it. It stays
   * ready where its block assumes it rests, and nothing pushes it.
   */
  @Test
  void aRecipeChangedByLiveTuningHomesNothing() {
    Homing toASwitchItLacks = Homing.toSwitch(Sensor.Switch.Limit.End.LOWER);
    JointsSimHarness tuned =
        new JointsSimHarness(
            JointFixtures.ROBOT,
            joint -> () -> homed(ControllerSettings.control(joint), toASwitchItLacks));
    JointMechanism tunedPivot = tuned.joints.of(PIVOT);
    Set<JointMechanism.State> states = EnumSet.noneOf(JointMechanism.State.class);
    double furthest = 0;

    for (int i = 0; i < 150; i++) {
      tuned.loop();
      states.add(tunedPivot.state());
      furthest =
          Math.max(furthest, Math.abs(tuned.simulation.position(PIVOT) - Math.toRadians(1.5)));
    }

    assertThat(states).containsExactly(JointMechanism.State.READY);
    assertThat(tunedPivot.homesToAStop()).isFalse();
    assertThat(tunedPivot.fault()).isEmpty();
    assertThat(degrees(furthest)).as("held where it rests, at 1.5°, not pushed").isLessThan(0.5);
  }

  /**
   * The turret on a SPARK MAX, homed to its switch at -90° on the SPARK's reverse limit input: the
   * SPARK stops pushing as the switch trips, within its millisecond loop, while the robot's code, a
   * loop behind, still asks it to push; then the robot reads the switch through the SPARK, with the
   * joint, and homes where it tripped.
   */
  @Test
  void aControllerStopsAtTheSwitchOnItsOwnPin() {
    assertItsControllerStops(
        unmotored(TURRET)
            .motors(Motor.NEO.through(JointFixtures.gearing(50)).on(SPARK_MAX.can(21)))
            .limitSwitch(
                LimitSwitch.lower()
                    .at(Degrees.of(-90))
                    .on(SensorWiring.Direction.REVERSE.normallyClosed())));
  }

  /**
   * The same on a TalonFX, whose Kraken has no limit inputs: its switch on a CANdi on its bus,
   * which the TalonFX reads over CAN as its reverse limit, and the robot reads as a sensor.
   */
  @Test
  void aTalonFxStopsAtItsSwitchOnACandi() {
    assertItsControllerStops(
        TURRET
            .motors(motors -> motors.on(TALON_FX.can(21)))
            .limitSwitch(
                LimitSwitch.lower().at(Degrees.of(-90)).on(CANDI.can(30).s1().normallyClosed())));
  }

  /** What a joint's controller was last asked as the loop began, and the volts it then put out. */
  private static final class Watched implements JointIO {
    private final JointIO io;
    private String asked = "";
    String askedAsItRead = "";
    double volts = Double.NaN;

    Watched(JointIO io) {
      this.io = io;
    }

    @Override
    public void updateInputs(JointIOInputs inputs) {
      io.updateInputs(inputs);
      askedAsItRead = asked;
      volts = inputs.appliedVolts;
    }

    @Override
    public void setPosition(double position, double velocity, double feedforwardVolts) {
      asked = "position";
      io.setPosition(position, velocity, feedforwardVolts);
    }

    @Override
    public void stop() {
      asked = "stop";
      io.stop();
    }

    @Override
    public void home(double volts) {
      asked = "home";
      io.home(volts);
    }

    @Override
    public void resetPosition(double position) {
      io.resetPosition(position);
    }
  }

  /**
   * A turret, its switch at -90° wired to its controller, powered on 40° to the right of where it
   * reads and homed to the switch at 1 V: the loop it trips, its controller already puts nothing
   * out, braking, though the robot pushed it home all the loop before. (Gently: a joint going
   * faster brakes against its current limit, which puts volts across its motor.)
   */
  private static void assertItsControllerStops(Joint.Revolute wired) {
    Joint.Revolute turret =
        wired.retuned(
            control ->
                homed(
                    control,
                    Homing.ToSwitch.builder()
                        .end(End.LOWER)
                        .volts(1)
                        .timeout(Seconds.of(5))
                        .build()));
    Watched[] watched = new Watched[1];
    JointsSimHarness sim =
        new JointsSimHarness(
            JointFixtures.ROBOT.with(turret),
            (joint, io) -> {
              if (!joint.equals(turret)) {
                return io;
              }
              watched[0] = new Watched(io);
              return watched[0];
            });
    JointMechanism mechanism = sim.joints.of(turret);
    sim.simulation.powerOnAt(turret, Math.toRadians(-40));

    double took = sim.runUntil(() -> degrees(sim.simulation.position(turret)) <= -90, 5);

    assertThat(took).as("it reached its switch").isNotNaN();
    assertThat(watched[0].askedAsItRead).as("the robot's last word").isEqualTo("home");
    assertThat(watched[0].volts).as("its controller stopped it, by itself").isZero();
    sim.run(0.2);
    assertThat(mechanism.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(degrees(mechanism.position()))
        .isCloseTo(degrees(sim.simulation.position(turret)), within(1.0));
  }
}

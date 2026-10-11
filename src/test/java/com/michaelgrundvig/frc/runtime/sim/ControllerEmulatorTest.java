package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FXS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.wiring.CanPorts;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIOSpark;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.simulation.SimDeviceSim;

/**
 * The simulator's motor controllers, each evaluating the joint's plan where its controller does: a
 * TalonFX's slot, all of it on the device every millisecond; a SPARK's kP and kI on the device,
 * with the feedforward the robot works out as it asks, held between requests. The fixtures' pivot,
 * with friction, damping and an integral tuned.
 */
@ExtendWith(RobotSim.class)
class ControllerEmulatorTest {
  private static final Joint.Revolute TUNED =
      PIVOT.retuned(control -> control.withKs(0.2).withKi(2).withKd(1.5));

  private static final ControllerPlan PLAN =
      ControllerPlan.of(ROBOT.math(), TUNED, TUNED.driven().control().orElseThrow());

  /**
   * A TalonFX's slot runs the whole law each millisecond, at where it reads the joint: the feedback
   * on the error and its integral, the damping toward the speed asked from the speed it reads, the
   * plan's feedforward at its position, and the program's own volts.
   */
  @Test
  void aTalonFxsSlotRunsTheWholeLawWhereItReadsTheJoint() {
    TalonFxEmulator talon = new TalonFxEmulator(PLAN);

    talon.startPositionLoopAfresh();
    talon.hold(1.0, 0.5, 0.3, 0.1);
    double first = talon.voltsToHold(0.95, 0.4, 0.001);
    double second = talon.voltsToHold(0.95, 0.4, 0.001);
    talon.run(2.0, 0.9);

    double feedback = PLAN.kP() * 0.05 + PLAN.kD() * (0.5 - 0.4);
    assertThat(first)
        .isCloseTo(
            feedback + PLAN.kI() * 0.05 * 0.001 + PLAN.feedforward(0.95, 0.5, 0) + 0.3,
            within(1e-12));
    assertThat(second - first).as("its integral grows").isCloseTo(2 * 0.05 * 0.001, within(1e-12));
    assertThat(talon.voltsToRun(1.1, 1.5))
        .isCloseTo(PLAN.kP() * 0.5 + PLAN.feedforward(1.1, 2.0, 0), within(1e-12));
  }

  /**
   * A SPARK runs its kP and its integral on the device; the rest the robot worked out as it asked,
   * from where it had last read the joint, and the SPARK holds it whatever the joint does until
   * it's asked again.
   */
  @Test
  void aSparkRunsItsKpAndHoldsWhatTheRobotWorkedOut() {
    SparkEmulator spark = new SparkEmulator(PLAN);

    spark.startPositionLoopAfresh();
    spark.hold(1.0, 0.5, 0.3, 0.1);
    double held = 0.3 + PLAN.feedforward(1.0, 0.5, 0) + PLAN.kD() * (0.5 - 0.1);
    double near = spark.voltsToHold(0.95, 0.4, 0.001);
    double there = spark.voltsToHold(1.0, 7, 0.001);
    spark.run(2.0, 0.9);

    double integral = PLAN.kI() * 0.05 * 0.001;
    assertThat(near).isCloseTo(PLAN.kP() * 0.05 + integral + held, within(1e-12));
    assertThat(there)
        .as("no damping on the device; the integral kept")
        .isCloseTo(held + integral, within(1e-12));
    assertThat(spark.voltsToRun(1.1, 1.5))
        .as("gravity where it was last read")
        .isCloseTo(PLAN.kP() * 0.5 + PLAN.feedforward(0.9, 2.0, 0), within(1e-12));
  }

  /**
   * A joint wired to a SPARK is simulated as one, and asked what the robot's own SPARK is: held at
   * where it rests, the volts the simulator puts across its motor are the feedforward the robot
   * sends REV's simulated SPARK for the same request.
   */
  @Test
  void theSimulatorsSparkIsAskedWhatTheRobotsIs() {
    MechanismWiring.Device wired = SPARK_MAX.can(62);
    Joint.Revolute pivot =
        unmotored(PIVOT)
            .motors(Motor.NEO.through(gearing(25)).on(wired))
            .retuned(control -> control.withKs(0.2).withKd(1.5));
    RobotDefinition robot = ROBOT.with(pivot);
    JointIOSpark onRobot = new JointIOSpark(robot.math(), pivot, ControllerPins.NONE);
    JointIO simulated = new JointSimulation(robot).io(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    double rest = PIVOT.restPosition();

    onRobot.updateInputs(inputs);
    onRobot.setPosition(rest, 0.4, 0.25);
    simulated.updateInputs(inputs);
    simulated.setPosition(rest, 0.4, 0.25);
    ((JointIOSim) simulated).step(JointIOSim.DEVICE_LOOP_SECONDS, 0);
    simulated.updateInputs(inputs);

    SimDeviceSim spark =
        new SimDeviceSim(
            String.format(
                Locale.ROOT,
                "SPARK MAX [%d,%d]",
                CanPorts.of(((CanBus.Port) wired.bus()).port()).value,
                wired.id()));
    assertThat(simulated).isInstanceOf(JointIOSim.class);
    // A SPARK holds its feedforward as a float.
    assertThat(inputs.appliedVolts)
        .isCloseTo(spark.getDouble("Arbitrary Feedforward").get(), within(1e-6))
        .isPositive();
  }

  /**
   * A joint wired to nothing is simulated on the controller its motor runs on, as its settings are
   * checked: a NEO's a SPARK MAX. It runs its kP on the device and holds the rest as the robot
   * worked it out, so it moves as the same joint wired to a SPARK MAX does, volt for volt, and not
   * as one on a TalonFXS, whose slot damps on the speed it reads each millisecond.
   */
  @Test
  void anUnwiredNeoJointIsEmulatedAsASpark() {
    Joint.Revolute onNeo = unmotored(PIVOT);
    List<Double> unwired = voltsGoingOut(onNeo.motors(Motor.NEO.through(gearing(25))));
    List<Double> onSpark =
        voltsGoingOut(onNeo.motors(Motor.NEO.through(gearing(25)).on(SPARK_MAX.can(32))));
    List<Double> onTalon =
        voltsGoingOut(onNeo.motors(Motor.NEO.through(gearing(25)).on(TALON_FXS.can(32))));

    assertThat(unwired).isEqualTo(onSpark).isNotEqualTo(onTalon);
  }

  /**
   * The volts the simulator puts across a damped pivot each loop, for half a second, asked toward
   * 30° past where it rests.
   */
  private static List<Double> voltsGoingOut(Joint.Revolute motored) {
    Joint.Revolute pivot = motored.retuned(control -> control.withKd(1.5));
    JointIO simulated = new JointSimulation(ROBOT.with(pivot)).io(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    List<Double> volts = new ArrayList<>();
    for (int loop = 0; loop < 25; loop++) {
      simulated.updateInputs(inputs);
      volts.add(inputs.appliedVolts);
      simulated.setPosition(PIVOT.restPosition() + Math.toRadians(30), 0, 0);
      ((JointIOSim) simulated).step(0.02, 0);
    }
    return volts;
  }

  /** On a SPARK in the simulator, the pivot goes out as it does on a TalonFX. */
  @Test
  void aJointOnASparkArrivesInTheSimulator() {
    JointsSimHarness sim =
        new JointsSimHarness(
            ROBOT.with(
                unmotored(PIVOT).motors(Motor.NEO.through(gearing(25)).on(SPARK_MAX.can(32)))));
    Move out = sim.joints.of(PIVOT).goTo(PIVOT_OUT);

    sim.scheduler.schedule(out);
    sim.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(out.arrived()).isTrue();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(130, within(2.0));
  }
}

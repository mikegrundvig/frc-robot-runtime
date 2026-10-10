package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_DOWN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_TOP;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_PUSH;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_EJECTING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_OUTTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_BACK;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_FRONT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FXS;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.GoldenFile;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * The fixtures' joints, recorded ({@link JointRecordings}): what their motor controllers are told,
 * wired to nothing, to TalonFXs with and without a CANcoder, and to SPARKs, with every gain the law
 * has tuned; and a session in the simulator, as written and with those gains. Both must come out as
 * recorded: a change meant to change neither (how the settings or the feedforward are worked out,
 * where the simulator's joints are modeled) is shown to change nothing.
 *
 * <p>When a change is meant to move them, record them again with {@code GOLDEN_UPDATE=1} and say
 * why they moved (docs/testing.md).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class JointsRecordedSimTest {
  /** Every gain the feedforward has, tuned: friction, speed, acceleration, gravity at an angle. */
  private static final RobotDefinition TUNED =
      ROBOT.with(
          PIVOT.retuned(
              control ->
                  control
                      .withKi(0.5)
                      .withKs(0.2)
                      .withKv(0.5)
                      .withKa(0.02)
                      .withKg(0.6, Degrees.of(20))),
          ROLLER.retuned(control -> control.withKs(0.1).withKv(0.03)),
          ELEVATOR.retuned(control -> control.withKs(0.15).withKg(0.9)),
          TURRET.retuned(control -> control.withKs(0.3).withKa(0.004)));

  /**
   * The pivot on a TalonFX, its loop closed on its CANcoder, fused, so it needs no homing; the
   * elevator on two, the second turned the other way; the roller on one.
   */
  private static RobotDefinition wired() {
    Joint.Revolute pivot =
        PIVOT
            .motorsOn(TALON_FX.can(32).clockwise())
            .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.2)).fused())
            .retuned(
                control ->
                    MechanismControlBuilder.builder(control).homing(Optional.empty()).build());
    Joint.Prismatic elevator = ELEVATOR.motorsOn(TALON_FX.can(20), TALON_FX.can(21).clockwise());
    Joint.Revolute roller = ROLLER.motorsOn(TALON_FX.can(31));
    return ROBOT.with(pivot, elevator, roller);
  }

  /**
   * The pivot on two NEOs, soft limits and friction tuned, an encoder on its data port, read once
   * or its loop closed on it; the elevator on two Vortexes and the roller on a NEO 550, on SPARK
   * Flexes.
   */
  private static RobotDefinition sparks(boolean closed) {
    var port = DATA_PORT.offset(Rotations.of(0.1)).inverted();
    Joint.Revolute pivot =
        unmotored(PIVOT)
            .motors(Motor.NEO, gearing(25), SPARK_MAX.can(41).clockwise(), SPARK_MAX.can(42))
            .absoluteEncoder(closed ? port.closesLoop() : port)
            .retuned(
                control ->
                    MechanismControlBuilder.builder(
                            control
                                .withSoftLimits(Range.of(Radians.of(0.02), Radians.of(2.3)))
                                .withKs(0.2))
                        .homing(Optional.empty())
                        .build());
    Joint.Prismatic elevator =
        unmotored(ELEVATOR)
            .motors(
                Motor.NEO_VORTEX,
                gearing(10),
                Inches.of(1),
                SPARK_FLEX.can(43),
                SPARK_FLEX.can(44).clockwise());
    Joint.Revolute roller = unmotored(ROLLER).motor(Motor.NEO_550, gearing(3), SPARK_FLEX.can(45));
    return ROBOT.with(pivot, elevator, roller);
  }

  /**
   * A pivot with a kI on a SPARK and a turret with a kG, which the robot refuses, and the roller on
   * a TalonFXS, which it takes.
   */
  private static RobotDefinition odd() {
    Joint.Revolute integral =
        unmotored(PIVOT)
            .motor(Motor.NEO, gearing(25), SPARK_MAX.can(41))
            .retuned(control -> control.withKi(1));
    Joint.Revolute level = TURRET.retuned(control -> control.withKg(0.3));
    Joint.Revolute onFxs = unmotored(ROLLER).motor(Motor.NEO_550, gearing(3), TALON_FXS.can(31));
    return ROBOT.with(integral, level, onFxs);
  }

  /**
   * The pivot's weight pulling hardest a third of a turn back: past Phoenix's reach on a TalonFX,
   * as the simulator runs it wired to nothing, and worked out on the robot on a SPARK.
   */
  private static RobotDefinition lowering(boolean onSpark) {
    Joint.Revolute pivot = PIVOT.retuned(control -> control.withKg(0.4, Degrees.of(-120)));
    return ROBOT.with(
        onSpark ? unmotored(pivot).motor(Motor.NEO, gearing(25), SPARK_MAX.can(41)) : pivot);
  }

  @Test
  void theirSettingsAreAsRecorded() throws IOException {
    List<String> rows = new ArrayList<>();
    for (RobotDefinition robot :
        List.of(
            ROBOT,
            TUNED,
            wired(),
            sparks(false),
            sparks(true),
            odd(),
            lowering(false),
            lowering(true))) {
      rows.addAll(JointRecordings.settings(robot));
    }
    GoldenFile.check(getClass(), "fixture-settings.csv", JointRecordings.SETTINGS, rows);
  }

  @Test
  void theirSessionIsAsRecorded() throws IOException {
    List<String> rows = new ArrayList<>(session(ROBOT));
    rows.addAll(session(TUNED));
    GoldenFile.check(getClass(), "fixture-session.csv", rows.get(0), rows);
  }

  /**
   * Each joint to its far place and back, the roller at each output, the pivot pushed into its
   * upper stop at 3 V, then everything disabled, braking: 6 s.
   */
  private static List<String> session(RobotDefinition robot) {
    JointsSimHarness sim = new JointsSimHarness(robot);
    JointMechanism pivot = sim.joints.of(PIVOT);
    JointMechanism roller = sim.joints.of(ROLLER);
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    JointMechanism turret = sim.joints.of(TURRET);
    JointRecordings.Session session =
        new JointRecordings.Session(sim, List.of(PIVOT, ROLLER, ELEVATOR, TURRET));

    sim.scheduler.schedule(pivot.goTo(PIVOT_OUT));
    sim.scheduler.schedule(elevator.goTo(ELEVATOR_TOP));
    sim.scheduler.schedule(turret.goTo(TURRET_BACK));
    sim.scheduler.schedule(roller.run(ROLLER_INTAKING));
    session.run(1.5);
    sim.scheduler.schedule(roller.run(ROLLER_OUTTAKING));
    sim.scheduler.schedule(pivot.run(PIVOT_PUSH));
    session.run(0.5);
    sim.scheduler.schedule(roller.run(ROLLER_EJECTING));
    sim.scheduler.schedule(pivot.goTo(PIVOT_IN));
    sim.scheduler.schedule(elevator.goTo(ELEVATOR_DOWN));
    sim.scheduler.schedule(turret.goTo(TURRET_FRONT));
    session.run(1.5);
    sim.scheduler.schedule(elevator.goTo(ELEVATOR_TOP));
    session.run(1);
    sim.setEnabled(false);
    session.run(1.5);
    return session.rows();
  }
}

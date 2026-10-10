package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.ARM;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ARM_SCORING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.HOOD;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INTAKE;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.STOWED;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.GoldenFile;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointRecordings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.DrivenBuilder;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * The guide's robot, recorded ({@code JointRecordings}): what its motor controllers are told, the
 * competition robot's and the practice robot's, its arm's and its elevator's CANcoders fused, and
 * its pivot, arm, elevator and shooter on SPARKs; and a session in the simulator, its elevator
 * without its encoder, homing on a hard stop. Both must come out as recorded: a change meant to
 * change neither is shown to change nothing. When a change is meant to move them, record them again
 * with {@code GOLDEN_UPDATE=1} and say why they moved (docs/testing.md).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class ExampleRobotRecordedSimTest {
  /** Its arm's and its elevator's loops closed on their CANcoders, fused. */
  private static RobotDefinition fused() {
    return ExampleRobot.COMPETITION
        .definition()
        .with(
            ARM.encoderOn(CANCODER.can(23).offset(Rotations.of(0.12)).fused()),
            ELEVATOR.encoderOn(CANCODER.can(24).offset(Rotations.of(0.31)).fused()));
  }

  /**
   * Its pivot on a NEO with its encoder on the SPARK's data port, its loop closed on it; its arm on
   * a NEO, read once from the data port, its weight pulling hardest a quarter turn back, which the
   * robot works out each loop; its elevator on two Vortexes, its loop closed on its encoder on the
   * data port, geared to its drum; and its shooter on two Vortexes.
   */
  private static RobotDefinition sparks() {
    Joint.Revolute pivot =
        unmotored(PIVOT)
            .motor(Motor.NEO, Gearing.of(25), SPARK_MAX.can(32))
            .encoderOn(DATA_PORT.offset(Rotations.of(0.82)).closesLoop());
    Joint.Revolute arm =
        unmotored(ARM)
            .motor(Motor.NEO, Gearing.of(60), SPARK_MAX.can(22))
            .encoderOn(DATA_PORT.offset(Rotations.of(0.12)));
    Joint.Prismatic elevator =
        unmotored(ELEVATOR)
            .motors(
                Motor.NEO_VORTEX,
                Gearing.of(5),
                Inches.of(0.75),
                SPARK_FLEX.can(20),
                SPARK_FLEX.can(21).clockwise())
            .encoderOn(DATA_PORT.offset(Rotations.of(0.31)).closesLoop());
    Joint.Revolute shooter =
        unmotored(SHOOTER)
            .motors(
                Motor.NEO_VORTEX,
                Gearing.of(1),
                SPARK_FLEX.can(52),
                SPARK_FLEX.can(53).clockwise());
    return ExampleRobot.COMPETITION.definition().with(pivot, arm, elevator, shooter);
  }

  @Test
  void itsSettingsAreAsRecorded() throws IOException {
    List<String> rows = new ArrayList<>();
    for (RobotConfig robot : ExampleRobot.all()) {
      rows.addAll(JointRecordings.settings(robot.definition()));
    }
    rows.addAll(JointRecordings.settings(fused()));
    rows.addAll(JointRecordings.settings(sparks()));
    GoldenFile.check(getClass(), "example-settings.csv", JointRecordings.SETTINGS, rows);
  }

  /**
   * The elevator, its encoder taken out, homes on its lower stop as the robot enables, so a homing
   * is recorded; the robot stows, intakes (its roller held until the pivot is out), raises its
   * elevator clear and its arm to score, spins its shooter and feeds, then stows again and
   * disables: 8 s.
   */
  @Test
  void itsSessionIsAsRecorded() throws IOException {
    Joint.Prismatic homing =
        ELEVATOR
            .driven(
                DrivenBuilder.builder(ELEVATOR.driven()).absoluteEncoder(Optional.empty()).build())
            .retuned(
                control -> control.withHoming(Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
    RobotDefinition robot = ExampleRobot.COMPETITION.definition().with(homing);
    List<Joint> joints = Joints.driven(robot);
    JointsSimHarness sim = new JointsSimHarness(robot);
    JointRecordings.Session session = new JointRecordings.Session(sim, joints);

    session.run(1.5);
    sim.scheduler.schedule(sim.joints.goTo(STOWED));
    session.run(1);
    sim.scheduler.schedule(sim.joints.run(INTAKE));
    sim.scheduler.schedule(sim.joints.of(HOOD).goTo(Degrees.of(30)));
    session.run(1.5);
    sim.scheduler.schedule(sim.joints.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    session.run(1);
    sim.scheduler.schedule(sim.joints.of(ARM).goTo(ARM_SCORING));
    sim.scheduler.schedule(sim.joints.of(SHOOTER).follow(() -> RPM.of(3000)));
    session.run(1);
    sim.scheduler.schedule(sim.joints.run(FEEDING));
    session.run(0.5);
    sim.scheduler.schedule(sim.joints.of(ROLLER).run(ExampleRobot.OUTTAKING));
    sim.scheduler.schedule(sim.joints.goTo(STOWED));
    session.run(1);
    sim.setEnabled(false);
    session.run(0.5);

    GoldenFile.check(getClass(), "example-session.csv", session.rows().get(0), session.rows());
  }
}

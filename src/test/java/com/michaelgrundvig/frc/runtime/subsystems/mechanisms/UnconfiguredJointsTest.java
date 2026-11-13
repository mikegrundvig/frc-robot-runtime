package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;
import org.wpilib.system.RobotController;

/**
 * A joint's device that never confirms its settings at startup, on each driver and for each kind of
 * device: the driver names it ({@link JointIO.JointIOInputs#unconfigured}), and the joint is
 * faulted and refuses to move. Phoenix's and REV's simulated devices take whatever they are sent,
 * so each driver is handed a send that one device doesn't answer, as one off the bus or at a wrong
 * CAN ID wouldn't.
 */
@ExtendWith(RobotSim.class)
class UnconfiguredJointsTest {
  /**
   * The robot is enabled, as the loops below say it is: the scheduler reads the Driver Station, and
   * refuses a joint's command while it is disabled.
   */
  @BeforeEach
  void enabled() {
    RobotSim.setEnabled(true);
  }

  private static final CanBus CANIVORE = CanBus.canivore("unconfigured");

  /**
   * A pivot on two TalonFXs and a CANcoder, at CAN IDs no other test in the runtime uses: the
   * leader 40, its follower 43, the CANcoder 0.
   */
  private static final Joint TALONS =
      unmotored(PIVOT)
          .motors(
              Motor.KRAKEN_X60
                  .through(gearing(25))
                  .on(TALON_FX.can(CANIVORE, 40), TALON_FX.can(CANIVORE, 43)))
          .absoluteEncoder(CANCODER.can(CANIVORE, 0).offset(Rotations.of(0)));

  /**
   * A pivot on two SPARK MAXes, leader first, at IDs no other test uses. REV allows one SPARK
   * object for each ID in a program, so each row has its own.
   */
  private static Joint sparks(int leader, int follower) {
    return unmotored(PIVOT)
        .motors(Motor.NEO.through(gearing(25)).on(SPARK_MAX.can(leader), SPARK_MAX.can(follower)));
  }

  private static final Joint SPARKS = sparks(50, 51);
  private static final Joint OTHER_SPARKS = sparks(48, 54);

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  /**
   * One row: the joint, its IO with one device not answering, and the name that device goes by.
   *
   * @param io makes the joint's IO on a robot with the joint
   */
  private record Row(
      String what, Joint joint, Function<RobotDefinition, JointIO> io, String named) {}

  /** A TalonFX joint whose device of this number ({@code JointIOTalon.Frames}) doesn't answer. */
  private static JointIO talons(RobotDefinition robot, int notAnswering) {
    return new JointIOTalon(
        robot.math(),
        TALONS,
        ControllerPins.NONE,
        new JointIOTalon.Frames() {
          @Override
          StatusCode send(int device, Supplier<StatusCode> send) {
            if (device == notAnswering) {
              return StatusCode.EcuIsNotPresent;
            }
            return send.get();
          }
        });
  }

  /** A SPARK joint whose SPARK at this CAN ID doesn't answer. */
  private static JointIO sparks(RobotDefinition robot, Joint joint, int notAnswering) {
    return new JointIOSpark(
        robot.math(),
        joint,
        ControllerPins.NONE,
        new JointIOSpark.Startup() {
          @Override
          REVLibError configure(SparkBase spark, SparkBaseConfig settings) {
            if (spark.getDeviceId() == notAnswering) {
              return REVLibError.kTimeout;
            }
            return super.configure(spark, settings);
          }
        });
  }

  @Test
  void aDeviceThatNeverTakesItsSettingsIsNamedAndItsJointRefusesToMove() {
    List<Row> rows =
        List.of(
            new Row("TalonFX leader", TALONS, robot -> talons(robot, 0), "motor 40"),
            new Row("TalonFX follower", TALONS, robot -> talons(robot, 1), "motor 43"),
            new Row("CANcoder", TALONS, robot -> talons(robot, 2), "CANcoder 0"),
            new Row("SPARK leader", SPARKS, robot -> sparks(robot, SPARKS, 50), "motor 50"),
            new Row(
                "SPARK follower",
                OTHER_SPARKS,
                robot -> sparks(robot, OTHER_SPARKS, 54),
                "motor 54"));
    for (Row row : rows) {
      RobotDefinition robot = ROBOT.with(row.joint());
      JointIO io = row.io().apply(robot);
      JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
      io.updateInputs(inputs);

      assertThat(inputs.unconfigured).as("%s named", row.what()).isEqualTo(row.named());

      Scheduler scheduler = Scheduler.createIndependentScheduler();
      try (Joints joints =
          new Joints(
              robot,
              joint -> joint.equals(row.joint()) ? io : new JointIO() {},
              Joints.untuned(),
              scheduler)) {
        JointMechanism pivot = joints.of(row.joint());
        Move out = pivot.goTo(PIVOT_OUT);
        scheduler.schedule(out);
        for (int i = 0; i < RobotLoop.loops(1) && out.outcome().isEmpty(); i++) {
          RobotSim.stepClock();
          joints.read(RobotController.getTime(), true);
          scheduler.run();
          joints.write();
        }

        assertThat(pivot.state())
            .as("%s: faulted", row.what())
            .isEqualTo(JointMechanism.State.FAULTED);
        assertThat(out.outcome()).as("%s: its move", row.what()).contains(Outcome.REFUSED);
      }
      io.stop();
    }
  }
}

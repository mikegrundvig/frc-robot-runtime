package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FXS;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.Coupling;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointPrismaticBuilder;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * Which joints the template drives, each from its block, and what it can't drive yet: what the
 * robot's configuration asks of the program, beside what the definition refuses of itself.
 */
class JointsTest {
  @Test
  void theFixturesJointsAreEachDrivenFromTheirBlocks() {
    assertThat(Joints.problems(ROBOT)).isEmpty();
    assertThat(Joints.driven(ROBOT)).containsExactly(PIVOT, ROLLER, ELEVATOR, TURRET);
  }

  /**
   * A cascade's second stage follows the carriage, and a hatch nothing drives: neither has a
   * mechanism of its own, and a control in the block of one says why.
   */
  @Test
  void aJointThatFollowsOrThatNothingDrivesIsntDriven() {
    Joint.Prismatic stage =
        JointPrismaticBuilder.builder(ELEVATOR)
            .label("second stage")
            .parent(ELEVATOR)
            .link(JointFixtures.link("stage", 1, new Translation3d()))
            .positions(List.of())
            .driven(Driven.NONE)
            .build();
    Joint.Revolute hatch =
        JointRevoluteBuilder.builder(PIVOT)
            .label("hatch")
            .link(JointFixtures.link("hatch", 1, new Translation3d()))
            .driven(Driven.NONE)
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .joints(append(ROBOT.joints(), stage, hatch))
            .couplings(
                List.of(
                    Coupling.Linear.builder()
                        .leader(ELEVATOR)
                        .follower(stage)
                        .ratio(1)
                        .offset(0)
                        .build()))
            .build();

    assertThat(Joints.problems(robot)).isEmpty();
    assertThat(Joints.driven(robot)).containsExactly(PIVOT, ROLLER, ELEVATOR, TURRET);
    assertThat(Joints.whyNotDriven(robot.math(), stage))
        .isEqualTo("it follows elevator, whose moves carry it");
    assertThat(Joints.whyNotDriven(robot.math(), hatch)).isEqualTo("nothing drives it");
    assertThat(
            Joints.problems(
                RobotDefinitionBuilder.builder(robot)
                    .joints(append(ROBOT.joints(), stage, hatch.control(PIVOT_CONTROL)))
                    .build()))
        .extracting(Finding::message)
        .containsExactly(
            "hatch has a control, and the template doesn't drive it: nothing drives it");
  }

  /**
   * Any motor is driven, in the simulator as it is and on a robot by the controller its block
   * names; a controller that can't drive its motor is refused as the definition is built, saying
   * what can.
   */
  @Test
  void anyMotorIsDrivenAndItsControllerMustDriveIt() {
    Joint.Revolute hatch =
        JointRevoluteBuilder.builder(unmotored(PIVOT))
            .label("hatch")
            .link(JointFixtures.link("hatch", 1, new Translation3d()))
            .build()
            .motors(Motor.NEO.through(JointFixtures.gearing(10)).on(SPARK_MAX.can(21)));
    Joint.Revolute flap =
        JointRevoluteBuilder.builder(unmotored(PIVOT))
            .label("flap")
            .link(JointFixtures.link("flap", 1, new Translation3d()))
            .build()
            .motors(Motor.MINION.through(JointFixtures.gearing(10)).on(TALON_FXS.can(22)));
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT).joints(append(ROBOT.joints(), hatch, flap)).build();

    assertThat(Joints.whyNotDriven(robot.math(), hatch)).isNull();
    assertThat(Joints.whyNotDriven(robot.math(), flap)).isNull();
    assertThat(Joints.problems(robot)).isEmpty();
    assertThat(Joints.driven(robot)).contains(hatch, flap);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                unmotored(hatch)
                    .motors(Motor.NEO.through(JointFixtures.gearing(10)).on(TALON_FX.can(21))))
        .withMessageContaining(
            "its NEO is on a TalonFX, which doesn't drive one: a TalonFXS or a SPARK MAX or a SPARK"
                + " Flex does");
  }

  /**
   * An encoder on a TalonFXS's data port, which the template doesn't read yet, is refused as the
   * robot's configuration asks what the program can drive, saying how to wire it instead.
   */
  @Test
  void anEncoderOnATalonFxsDataPortIsntReadYet() {
    Joint.Revolute pivot =
        unmotored(PIVOT)
            .motors(Motor.MINION.through(JointFixtures.gearing(25)).on(TALON_FXS.can(23)))
            .absoluteEncoder(DATA_PORT.offset(Rotations.of(0)))
            .retuned(old -> MechanismControl.position(30));

    assertThat(Joints.problems(ROBOT.with(pivot)))
        .extracting(Finding::message)
        .containsExactly(
            "pivot's wiring names an encoder on a TalonFXS's data port, which the template"
                + " doesn't read yet: wire it as a CANcoder");
  }

  private static final MechanismControl PIVOT_CONTROL = ControllerSettings.control(PIVOT);

  private static List<Joint> append(List<Joint> joints, Joint... more) {
    List<Joint> all = new ArrayList<>(joints);
    all.addAll(List.of(more));
    return all;
  }

  /**
   * A joint whose motors aren't wired is checked on the controller its motor runs on: an arm on
   * NEOs, its weight pulling hardest half a turn from its zero, is taken, as a SPARK takes any
   * gravity offset; on Krakens it's refused for Phoenix's, as a TalonFX would refuse it.
   */
  @Test
  void anUnwiredJointIsCheckedOnItsMotorsController() {
    MechanismControl lowering = PIVOT.driven().control().orElseThrow().withKg(0.4, Degrees.of(180));
    Joint.Revolute onNeos =
        unmotored(PIVOT)
            .motors(Motor.NEO.through(JointFixtures.gearing(25)).count(2))
            .retuned(c -> lowering);
    Joint.Revolute onKrakens =
        unmotored(PIVOT)
            .motors(Motor.KRAKEN_X60.through(JointFixtures.gearing(25)).count(2))
            .retuned(c -> lowering);

    assertThat(ControllerSettings.findings(ROBOT.with(onNeos).math(), onNeos))
        .noneMatch(finding -> finding.level() == Finding.Level.REFUSED);
    assertThat(ControllerSettings.findings(ROBOT.with(onKrakens).math(), onKrakens))
        .anyMatch(
            finding ->
                finding.level() == Finding.Level.REFUSED
                    && finding.message().contains("past Phoenix's"));
  }
}

package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FXS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.DrivenBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryCornerBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiringModuleWiringBuilder;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

class RobotConfigTest {
  @Test
  void everyExampleRobotHasAValidConfiguration() {
    // Building a RobotConfig checks every value, so loading the robots is the check.
    assertThat(ExampleRobot.all()).isNotEmpty();
    assertThat(ExampleRobot.all()).extracting(RobotConfig::name).doesNotHaveDuplicates();
  }

  /**
   * The driver's top speed is how it's driven, the free speed what the robot is: the definition
   * holds one to the other, as it's built. The template's robot reaches 4.73 m/s free.
   */
  @Test
  void topSpeedCannotExceedTheFreeSpeed() {
    var tooFast =
        DrivetrainControlBuilder.builder(ExampleRobot.SIM.drivetrain())
            .limits(
                DrivetrainControlLimitsBuilder.builder(ExampleRobot.SIM.drivetrain().limits())
                    .maxLinearSpeed(MetersPerSecond.of(5))
                    .build())
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotConfigBuilder.builder(ExampleRobot.SIM)
                    .definition(
                        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                            .drivetrain(ExampleRobot.SIM.definition().drivetrain().control(tooFast))
                            .build())
                    .build())
        .withMessage(
            "RobotDefinition[sim].drivetrain.control.limits.maxLinearSpeed: 5 m/s is faster than"
                + " the drivetrain's free speed of 4.729 m/s: give maxLinearSpeed at most that");
  }

  /**
   * Each joint's block is how it's driven: the program drives it from the definition alone, its
   * control and its motors' controllers the block's, and its limit switch's wiring its own. A joint
   * with motors and no control is refused, naming what to give it.
   */
  @Test
  void eachJointsBlockIsItsSetup() {
    Joint.Revolute pivot =
        JointFixtures.unmotored(JointFixtures.PIVOT)
            .motor(Motor.NEO, JointFixtures.gearing(25), SPARK_MAX.can(32).clockwise())
            .limitSwitch(
                Sensor.Switch.Limit.End.LOWER,
                Degrees.of(0),
                SensorWiring.Direction.REVERSE.normallyClosed());
    RobotDefinition wired =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
            .joints(List.of(pivot))
            .configurations(List.of())
            .build();

    RobotConfig robot = RobotConfigBuilder.builder(ExampleRobot.SIM).definition(wired).build();

    Joint driven = Joints.driven(robot.definition()).getFirst();
    assertThat(driven).isEqualTo(JointFixtures.PIVOT);
    assertThat(driven.driven().control()).isEqualTo(JointFixtures.PIVOT.driven().control());
    assertThat(driven.driven().motors().orElseThrow().controllers())
        .containsExactly(SPARK_MAX.can(32).clockwise());
    assertThat(driven.switchAt(Sensor.Switch.Limit.End.LOWER).wiring())
        .contains(
            new SensorWiring.Connection(
                new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.REVERSE),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotConfigBuilder.builder(ExampleRobot.SIM)
                    .definition(
                        RobotDefinitionBuilder.builder(wired)
                            .joints(
                                List.of(
                                    pivot.driven(
                                        DrivenBuilder.builder(pivot.driven())
                                            .control(Optional.empty())
                                            .build())))
                            .build())
                    .build())
        .withMessageContaining(
            "pivot has motors and no control: give it .control(MechanismControl.position(kP))");
  }

  /**
   * A practice robot is the competition robot under its own name, with its differences: a joint, a
   * sensor or the drivetrain re-wired replaces its equal, the rest of it kept, so behavior that
   * names the joint finds the practice one. It's a configuration of its own, checked as every robot
   * is, so Robots.all() lists it beside the competition robot. A part the robot hasn't is refused.
   */
  @Test
  void aPracticeRobotIsTheCompetitionRobotRewired() {
    Joint.Revolute pivot =
        JointFixtures.unmotored(JointFixtures.PIVOT)
            .motor(Motor.NEO, JointFixtures.gearing(25), SPARK_MAX.can(32).clockwise());
    Sensor.Switch beam =
        Sensor.Switch.beam("claw beam", pivot).on(SensorWiring.port(1).normallyOpen());
    RobotConfig competition =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .name("competition")
                    .joints(List.of(pivot))
                    .sensors(List.of(beam))
                    .configurations(List.of())
                    .build())
            .build();
    SwerveWiring wiring =
        SwerveWiring.builder()
            .canBus(CanBus.SYSTEMCORE)
            .driveController(MechanismWiring.Controller.TALON_FX)
            .steerController(MechanismWiring.Controller.TALON_FX)
            .gyroId(1)
            .gyroYawScale(1.0)
            .modules(List.of(module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4)))
            .build();

    RobotConfig practice =
        competition
            .named("practice")
            .with(pivot.motorsOn(SPARK_MAX.can(33).clockwise()))
            .with(beam.on(SensorWiring.port(2).normallyOpen()))
            .with(competition.definition().drivetrain().wired(wiring));
    List<RobotConfig> all = List.of(competition, practice);

    assertThat(all).extracting(RobotConfig::name).containsExactly("competition", "practice");
    assertThat(practice.definition().joints()).containsExactly(JointFixtures.PIVOT);
    assertThat(controllers(practice)).containsExactly(SPARK_MAX.can(33).clockwise());
    assertThat(controllers(competition)).containsExactly(SPARK_MAX.can(32).clockwise());
    assertThat(((Sensor.Switch) practice.definition().sensors().getFirst()).wiring())
        .contains(SensorWiring.port(2).normallyOpen());
    assertThat(practice.drivetrainWiring()).contains(wiring);
    assertThat(competition.drivetrainWiring()).isEmpty();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(JointFixtures.ROLLER))
        .withMessage(
            "competition has no joint roller the same but for how it's driven, its stops, where it"
                + " rests and where its places are");
  }

  private static List<MechanismWiring.Device> controllers(RobotConfig robot) {
    return robot.definition().joints().getFirst().driven().motors().orElseThrow().controllers();
  }

  /**
   * An interlock about a joint the template doesn't drive is refused as the configuration is built:
   * nothing could enforce it. The joints, driven by nothing here, are still the ones it names.
   */
  @Test
  void anInterlockNothingCanEnforceIsRefusedWithTheConfiguration() {
    RobotDefinition guarded =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
            .joints(
                List.of(
                    JointFixtures.PIVOT.driven(Driven.NONE),
                    JointFixtures.ROLLER.driven(Driven.NONE)))
            .interlocks(
                List.of(
                    Interlock.that(JointFixtures.ROLLER)
                        .runsOnlyWhen(JointFixtures.PIVOT.atLeast(JointFixtures.PIVOT_RUN))))
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotConfigBuilder.builder(ExampleRobot.SIM).definition(guarded).build())
        .withMessage(
            "RobotConfig[sim].definition.interlocks: \"roller runs only when pivot at least run\""
                + " can't be enforced: roller has no mechanism, as nothing drives it");
  }

  /**
   * A control a joint's motor controllers can't take is refused as the configuration is built, in
   * the robot's own words, wherever the program runs: an arm's gravity angle a third of a turn from
   * its zero, past Phoenix's gravity offset, on a TalonFXS it's wired to, and on Krakens wired to
   * nothing, which the simulator runs on TalonFXs. On NEOs wired to nothing it runs on a SPARK, as
   * it does wired to one, the robot working out its gravity each loop, any offset, and a kI with
   * it.
   */
  @Test
  void aControlItsMotorControllerWouldRefuseIsRefusedWithTheConfiguration() {
    MechanismControl held =
        MechanismControl.position(30).withHoming(MechanismControl.Homing.assumeAtRest());
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, new Translation3d(0.2, 0, 0.4), Joint.PITCH_UP)
            .range(Range.degrees(0, 90))
            .rest(Degrees.of(0))
            .motor(Motor.NEO, JointFixtures.gearing(50))
            .control(held.withKg(0.4, Degrees.of(120)));
    Joint.Revolute onKrakens =
        Joint.pivot("arm", Part.CHASSIS, new Translation3d(0.2, 0, 0.4), Joint.PITCH_UP)
            .range(Range.degrees(0, 90))
            .rest(Degrees.of(0))
            .motor(Motor.KRAKEN_X60, JointFixtures.gearing(50))
            .control(held.withKg(0.4, Degrees.of(120)));
    String robots =
        "arm's gravity angle, 120°, is more than a quarter turn from its zero, past Phoenix's"
            + " gravity offset of ±0.25 rotations: its positive direction lowers it at its zero."
            + " Turn its axis around in the definition (and its motors' and encoder's directions"
            + " in the wiring), so that positive raises it; its gravity angle is then 60°";
    MechanismControl integral =
        MechanismControlBuilder.builder(held)
            .gains(MechanismControlGainsBuilder.builder(held.gains()).kI(1).build())
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> withJoint(onKrakens))
        .withMessage("RobotConfig[sim].joints: " + robots);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> withJoint(arm.motorsOn(TALON_FXS.can(21))))
        .withMessage("RobotConfig[sim].joints: " + robots);
    assertThat(withJoint(arm).definition().joints())
        .as("on NEOs wired to nothing, a SPARK: any offset")
        .hasSize(1);
    assertThat(withJoint(arm.motorsOn(SPARK_MAX.can(21))).definition().joints())
        .as("on a SPARK, the robot works out its gravity, any offset")
        .hasSize(1);
    assertThat(withJoint(onKrakens.retuned(control -> integral.withKg(0.4, Degrees.of(60)))))
        .as("a TalonFX takes a kI")
        .isNotNull();
    assertThat(withJoint(arm.retuned(control -> integral)).definition().joints()).hasSize(1);
    assertThat(withJoint(arm.motorsOn(SPARK_MAX.can(21)).retuned(control -> integral)))
        .as("a SPARK takes a kI")
        .isNotNull();
  }

  /** The simulator's robot with one joint, its configuration built. */
  private static RobotConfig withJoint(Joint joint) {
    return RobotConfigBuilder.builder(ExampleRobot.SIM)
        .definition(
            RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                .joints(List.of(joint))
                .configurations(List.of())
                .build())
        .build();
  }

  /**
   * Treads said to grip harder than the slip check can tell from a collision, as a path planner's
   * friction often is (a team's PathPlanner gives 2.255), are the robot's doubt, not a refusal: the
   * robot starts, and says what the grip costs and what to give.
   */
  @Test
  void aPathPlannersFrictionAsTheTreadsGripIsADoubt() {
    List<DrivetrainGeometry.Corner> corners =
        ExampleRobot.SIM.definition().drivetrain().corners().stream()
            .map(
                corner ->
                    DrivetrainGeometryCornerBuilder.builder(corner)
                        .module(
                            DrivetrainGeometryModuleTypeBuilder.builder(corner.module())
                                .wheelCoefficientOfFriction(2.255)
                                .build())
                        .build())
            .toList();
    RobotDefinition grippy =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
            .drivetrain(
                DrivetrainGeometryBuilder.builder(ExampleRobot.SIM.definition().drivetrain())
                    .corners(corners)
                    .build())
            .build();

    RobotConfig robot = RobotConfigBuilder.builder(ExampleRobot.SIM).definition(grippy).build();

    assertThat(Doubts.lines(Doubts.of(robot.definition()), Finding.Scope.ROBOT))
        .contains(
            "the treads' grip, 2.26, is past 1.53, above which the slip check can't tell a slide"
                + " from a collision: a jolt up to 2.26 g may be the floor's, so it's none, and a"
                + " hit under it is seen only as a slide. Give the tread's own, about 1.0 worn and"
                + " 1.3 new, from a pull test (docs/characterization.md, tread friction); a path"
                + " planner's friction is a tuning value, not the tread's");
    assertThat(Doubts.lines(Doubts.of(ExampleRobot.SIM.definition()), Finding.Scope.ROBOT))
        .noneMatch(doubt -> doubt.contains("slip check can't tell"));
  }

  /**
   * A drivetrain wired for a robot whose controllers can't drive its motors is refused by its
   * definition as it's wired, saying what does: its modules wouldn't move. In the simulator alone,
   * any motor runs.
   */
  @Test
  void aWiredSwerveOfMotorsNoTalonFxDrivesIsRefused() {
    List<DrivetrainGeometry.Corner> neos =
        ExampleRobot.SIM.definition().drivetrain().corners().stream()
            .map(
                corner ->
                    DrivetrainGeometryCornerBuilder.builder(corner)
                        .module(
                            DrivetrainGeometryModuleTypeBuilder.builder(corner.module())
                                .driveMotor(Motor.NEO)
                                .build())
                        .build())
            .toList();
    RobotDefinition rev =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
            .drivetrain(
                DrivetrainGeometryBuilder.builder(ExampleRobot.SIM.definition().drivetrain())
                    .corners(neos)
                    .build())
            .build();
    SwerveWiring wiring =
        SwerveWiring.builder()
            .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
            .driveController(MechanismWiring.Controller.TALON_FX)
            .steerController(MechanismWiring.Controller.TALON_FX)
            .gyroId(1)
            .gyroYawScale(1.0)
            .modules(List.of(module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4)))
            .build();

    RobotConfig unwired = RobotConfigBuilder.builder(ExampleRobot.SIM).definition(rev).build();

    assertThat(unwired.definition()).isEqualTo(rev);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rev.drivetrain().wired(wiring))
        .withMessage(
            "DrivetrainGeometry.wiring: its drive's NEO is on a TalonFX, which doesn't drive one: a"
                + " TalonFXS or a SPARK MAX or a SPARK Flex does");
  }

  /**
   * A drivetrain whose drive and steering are on different makers' controllers, each driving its
   * motors, is the definition's to have, and the template's modules can't drive it yet: refused as
   * the configuration is built, saying what they drive.
   */
  @Test
  void aSwerveOnTwoMakersControllersIsRefusedByTheProgram() {
    List<DrivetrainGeometry.Corner> neoSteering =
        ExampleRobot.SIM.definition().drivetrain().corners().stream()
            .map(
                corner ->
                    DrivetrainGeometryCornerBuilder.builder(corner)
                        .module(
                            DrivetrainGeometryModuleTypeBuilder.builder(corner.module())
                                .steerMotor(Motor.NEO_550)
                                .build())
                        .build())
            .toList();
    DrivetrainGeometry drivetrain =
        DrivetrainGeometryBuilder.builder(ExampleRobot.SIM.definition().drivetrain())
            .corners(neoSteering)
            .build();
    RobotConfig unwired =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .drivetrain(drivetrain)
                    .build())
            .build();
    DrivetrainGeometry mixed =
        drivetrain.wired(
            SwerveWiring.builder()
                .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
                .driveController(MechanismWiring.Controller.TALON_FX)
                .steerController(SPARK_MAX)
                .gyroId(1)
                .gyroYawScale(1.0)
                .modules(
                    List.of(
                        sparkSteered(1, 2),
                        sparkSteered(3, 4),
                        sparkSteered(5, 6),
                        sparkSteered(7, 8)))
                .build());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> unwired.with(mixed))
        .withMessage(
            "RobotConfig[sim].definition.drivetrain.wiring: the drivetrain drives on a TalonFX and"
                + " steers on a SPARK MAX, and the template's modules are TalonFXs or SPARKs, doing"
                + " both, so far: put both on TalonFXs, or both on SPARKs");
  }

  /** A module steered by a SPARK, which reads its angle on its data port: no CANcoder. */
  private static SwerveWiring.ModuleWiring sparkSteered(int drive, int steer) {
    return SwerveWiringModuleWiringBuilder.builder(module(drive, steer, 1))
        .encoderId(Optional.empty())
        .build();
  }

  private static SwerveWiring.ModuleWiring module(int drive, int steer, int encoder) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(drive)
        .steerId(steer)
        .encoderOffset(Rotations.of(0))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .encoderId(encoder)
        .build();
  }
}

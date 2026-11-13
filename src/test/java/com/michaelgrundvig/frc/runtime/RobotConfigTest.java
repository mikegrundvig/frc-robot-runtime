package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FXS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.localization.LocalizationGate;
import com.michaelgrundvig.frc.localization.LocalizationGateBuilder;
import com.michaelgrundvig.frc.localization.SlipCheck;
import com.michaelgrundvig.frc.localization.SlipCheckSettingsBuilder;
import com.michaelgrundvig.frc.pitwall.CameraIds;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Driven.LimitSwitch;
import com.michaelgrundvig.frc.world.robot.DrivenBuilder;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainCornerBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainModuleBuilder;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RevoluteJoint;
import com.michaelgrundvig.frc.world.robot.RevoluteJointBuilder;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiringModuleWiringBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class RobotConfigTest {
  @Test
  void everyExampleRobotHasAValidConfiguration() {
    // Building a RobotConfig checks every value, so loading the robots is the check.
    assertThat(ExampleRobot.all()).isNotEmpty();
    assertThat(ExampleRobot.all()).extracting(RobotConfig::name).doesNotHaveDuplicates();
  }

  /**
   * A camera's card on Pitwall's Pit view is named by its id and a coprocessor's by its address, so
   * a camera named as another card is (the autos', the venue's, a listed board's), two cameras
   * whose ids differ only by dots and underscores, or one board listed twice, are refused as the
   * configuration is built, saying which and why, rather than failing as Pitwall starts; and so is
   * a camera whose id is too long for Pitwall's World view to name its shapes by.
   */
  @Test
  void aCardThatWouldTakeAnotherCardsNameIsRefused() {
    record Case(List<String> cameras, List<String> boards, String refusal) {}
    String cameras = "RobotConfig[sim].definition.cameras: ";
    String boards = "RobotConfig[sim].coprocessors: ";
    List<Case> cases =
        List.of(
            new Case(
                List.of("Auto"),
                List.of(),
                cameras
                    + "camera \"Auto\"'s card would be named auto on Pitwall's Pit view, as the"
                    + " autos' card is: give the camera another name"),
            new Case(
                List.of("venue"),
                List.of(),
                cameras
                    + "camera \"venue\"'s card would be named venue on Pitwall's Pit view, as the"
                    + " venue's card is: give the camera another name"),
            new Case(
                List.of("rear.cam", "rear_cam"),
                List.of(),
                cameras
                    + "camera \"rear_cam\"'s card would be named rear-cam on Pitwall's Pit view,"
                    + " as camera \"rear.cam\"'s card is: give the camera another name"),
            new Case(
                List.of("10.12.34.11"),
                List.of("10.12.34.11"),
                cameras
                    + "camera \"10.12.34.11\"'s card would be named 10-12-34-11 on Pitwall's Pit"
                    + " view, as coprocessor 10.12.34.11's card is: give the camera another name"),
            new Case(
                List.of(),
                List.of("10.12.34.11", "10.12.34.11"),
                boards
                    + "coprocessor 10.12.34.11's card would be named 10-12-34-11 on Pitwall's Pit"
                    + " view, as coprocessor 10.12.34.11's card is: list each coprocessor once,"
                    + " by its address"),
            new Case(
                List.of("A".repeat(53)),
                List.of(),
                cameras
                    + "camera \""
                    + "A".repeat(53)
                    + "\"'s name is too long as an id (\""
                    + "a".repeat(53)
                    + "\", 53 characters; at most 52): shorten it"),
            new Case(List.of("a".repeat(CameraIds.LONGEST)), List.of(), ""),
            new Case(List.of("auto-cam", "venue-left"), List.of("10.12.34.11"), ""));
    var definition = ExampleRobot.SIM.definition();
    for (Case c : cases) {
      List<CameraConfig> renamed = new ArrayList<>(definition.cameras());
      for (int i = 0; i < c.cameras().size(); i++) {
        renamed.set(
            i, CameraConfigBuilder.builder(renamed.get(i)).name(c.cameras().get(i)).build());
      }
      var building =
          RobotConfigBuilder.builder(ExampleRobot.SIM)
              .definition(RobotDefinitionBuilder.builder(definition).cameras(renamed).build())
              .coprocessors(c.boards());
      String row = c.cameras() + " on " + c.boards();
      if (c.refusal().isEmpty()) {
        assertThat(building.build().definition().cameras()).as(row).isNotEmpty();
      } else {
        assertThatIllegalArgumentException()
            .as(row)
            .isThrownBy(building::build)
            .withMessage(c.refusal());
      }
    }
  }

  /**
   * The driver's top speed is how it's driven, the free speed what the robot is: the definition
   * holds one to the other, as it's built. The template's robot reaches 4.78 m/s free.
   */
  /**
   * The values the guides tell a team to change: left out, the scoring gate and the slip check are
   * the library's; given, they are the robot's.
   */
  @Test
  void theScoringGateAndTheSlipCheckAreTheLibrarysUnlessGiven() {
    LocalizationGate strict =
        LocalizationGateBuilder.builder(LocalizationGate.SCORING)
            .maxPositionStdDevMeters(0.05)
            .build();
    SlipCheck.Settings tolerant =
        SlipCheckSettingsBuilder.builder(SlipCheck.Settings.defaults())
            .speedToleranceMetersPerSecond(0.5)
            .build();

    RobotConfig given =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .scoringGate(strict)
            .slipCheck(tolerant)
            .build();

    assertThat(ExampleRobot.SIM.scoringGate()).isEqualTo(LocalizationGate.SCORING);
    assertThat(ExampleRobot.SIM.slipCheck()).isEqualTo(SlipCheck.Settings.defaults());
    assertThat(given.scoringGate()).isEqualTo(strict);
    assertThat(given.slipCheck()).isEqualTo(tolerant);
  }

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
        .isThrownBy(() ->
            RobotConfigBuilder.builder(ExampleRobot.SIM)
                .definition(
                    RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                        .drivetrain(ExampleRobot.SIM.definition().drivetrain().control(tooFast))
                        .build())
                .build())
        .withMessage(
            "RobotDefinition[sim].drivetrain.control.limits.maxLinearSpeed: 5 m/s is faster than"
                + " the drivetrain's free speed of 4.78 m/s: give withMaxSpeed at most that");
  }

  /**
   * Each joint's block is how it's driven: the program drives it from the definition alone, its
   * control and its motors' controllers the block's, and its limit switch's wiring its own. A joint
   * with motors and no control is refused, naming what to give it.
   */
  @Test
  void eachJointsBlockIsItsSetup() {
    RevoluteJoint pivot =
        JointFixtures.unmotored(JointFixtures.PIVOT)
            .motors(Motor.NEO.through(JointFixtures.gearing(25)).on(SPARK_MAX.can(32).clockwise()))
            .limitSwitch(
                LimitSwitch.lower()
                    .at(Degrees.of(0))
                    .on(SensorWiring.Direction.REVERSE.normallyClosed()));
    RobotDefinition wired =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
            .joints(List.of(pivot))
            .presets(List.of())
            .build();

    RobotConfig robot = RobotConfigBuilder.builder(ExampleRobot.SIM).definition(wired).build();

    Joint driven = Joints.driven(robot.definition()).getFirst();
    assertThat(driven).isEqualTo(JointFixtures.PIVOT);
    assertThat(driven.driven().control()).isEqualTo(JointFixtures.PIVOT.driven().control());
    assertThat(driven.driven().motors().orElseThrow().controllers())
        .containsExactly(SPARK_MAX.can(32).clockwise());
    assertThat(RobotMath.switchAt(driven, Sensor.Switch.Limit.End.LOWER).wiring())
        .contains(
            new SensorWiring.Connection(
                new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.REVERSE),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            RobotConfigBuilder.builder(ExampleRobot.SIM)
                .definition(
                    RobotDefinitionBuilder.builder(wired)
                        .joints(
                            List.of(
                                RevoluteJointBuilder.builder(pivot)
                                    .driven(
                                        DrivenBuilder.builder(pivot.driven())
                                            .control(Optional.empty())
                                            .build())
                                    .build()))
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
    RevoluteJoint pivot =
        JointFixtures.unmotored(JointFixtures.PIVOT)
            .motors(Motor.NEO.through(JointFixtures.gearing(25)).on(SPARK_MAX.can(32).clockwise()));
    Sensor.Switch beam =
        Sensor.Switch.beam("claw beam", pivot).on(SensorWiring.port(1).normallyOpen());
    RobotConfig competition =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .name("competition")
                    .joints(List.of(pivot))
                    .sensors(List.of(beam))
                    .presets(List.of())
                    .build())
            .build();
    Drivetrain rewired =
        TestWiring.talons(competition.definition().drivetrain(), CanBus.SYSTEMCORE, 1, 1);
    SwerveWiring wiring = rewired.wiring().orElseThrow();

    RobotConfig practice =
        competition
            .named("practice")
            .with(pivot.motors(motors -> motors.on(SPARK_MAX.can(33).clockwise())))
            .with(beam.on(SensorWiring.port(2).normallyOpen()))
            .with(rewired);
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
                    RevoluteJointBuilder.builder(JointFixtures.PIVOT).driven(Driven.NONE).build(),
                    RevoluteJointBuilder.builder(JointFixtures.ROLLER).driven(Driven.NONE).build()))
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
    RevoluteJoint arm =
        Joint.pivot("arm", Part.CHASSIS, Joint.PITCH_UP)
            .at(Meters.of(0.2), Meters.of(0), Meters.of(0.4))
            .range(Range.degrees(0, 90))
            .rest(Degrees.of(0))
            .motors(Motor.NEO.through(JointFixtures.gearing(50)))
            .control(held.withKg(0.4, Degrees.of(120)));
    RevoluteJoint onKrakens =
        Joint.pivot("arm", Part.CHASSIS, Joint.PITCH_UP)
            .at(Meters.of(0.2), Meters.of(0), Meters.of(0.4))
            .range(Range.degrees(0, 90))
            .rest(Degrees.of(0))
            .motors(Motor.KRAKEN_X60.through(JointFixtures.gearing(50)))
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
        .isThrownBy(() -> withJoint(arm.motors(motors -> motors.on(TALON_FXS.can(21)))))
        .withMessage("RobotConfig[sim].joints: " + robots);
    assertThat(withJoint(arm).definition().joints())
        .as("on NEOs wired to nothing, a SPARK: any offset")
        .hasSize(1);
    assertThat(withJoint(arm.motors(motors -> motors.on(SPARK_MAX.can(21)))).definition().joints())
        .as("on a SPARK, the robot works out its gravity, any offset")
        .hasSize(1);
    assertThat(withJoint(onKrakens.retuned(control -> integral.withKg(0.4, Degrees.of(60)))))
        .as("a TalonFX takes a kI")
        .isNotNull();
    assertThat(withJoint(arm.retuned(control -> integral)).definition().joints()).hasSize(1);
    assertThat(
            withJoint(
                arm.motors(motors -> motors.on(SPARK_MAX.can(21))).retuned(control -> integral)))
        .as("a SPARK takes a kI")
        .isNotNull();
  }

  /** The simulator's robot with one joint, its configuration built. */
  private static RobotConfig withJoint(Joint joint) {
    return RobotConfigBuilder.builder(ExampleRobot.SIM)
        .definition(
            RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                .joints(List.of(joint))
                .presets(List.of())
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
    List<Drivetrain.Corner> corners =
        ExampleRobot.SIM.definition().drivetrain().corners().stream()
            .map(corner ->
                DrivetrainCornerBuilder.builder(corner)
                    .module(
                        DrivetrainModuleBuilder.builder(corner.module())
                            .wheelCoefficientOfFriction(2.255)
                            .build())
                    .build())
            .toList();
    RobotDefinition grippy =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
            .drivetrain(
                DrivetrainBuilder.builder(ExampleRobot.SIM.definition().drivetrain())
                    .corners(corners)
                    .build())
            .build();

    RobotConfig robot = RobotConfigBuilder.builder(ExampleRobot.SIM).definition(grippy).build();

    assertThat(
            Doubts.lines(
                Doubts.of(
                    robot.definition(),
                    SlipCheck.Settings.defaults(),
                    RobotSim.massOf(robot.definition())),
                Finding.Scope.ROBOT))
        .contains(
            "the treads' grip, 2.26, is past 1.53, above which the slip check can't tell a slide"
                + " from a collision: a jolt up to 2.26 g may be the floor's, so it's none, and a"
                + " hit under it is seen only as a slide. Give the tread's own, about 1.0 worn and"
                + " 1.3 new, from a pull test (docs/characterization.md, tread friction); a path"
                + " planner's friction is a tuning value, not the tread's");
    assertThat(
            Doubts.lines(
                Doubts.of(
                    ExampleRobot.SIM.definition(),
                    SlipCheck.Settings.defaults(),
                    RobotSim.massOf(ExampleRobot.SIM.definition())),
                Finding.Scope.ROBOT))
        .noneMatch(doubt -> doubt.contains("slip check can't tell"));
  }

  /**
   * A drivetrain wired for a robot whose controllers can't drive its motors is refused by its
   * definition as it's wired, saying what does: its modules wouldn't move. In the simulator alone,
   * any motor runs.
   */
  @Test
  void aWiredSwerveOfMotorsNoTalonFxDrivesIsRefused() {
    List<Drivetrain.Corner> neos =
        ExampleRobot.SIM.definition().drivetrain().corners().stream()
            .map(corner ->
                DrivetrainCornerBuilder.builder(corner)
                    .module(
                        DrivetrainModuleBuilder.builder(corner.module())
                            .driveMotor(Motor.NEO)
                            .build())
                    .build())
            .toList();
    RobotDefinition rev =
        RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
            .drivetrain(
                DrivetrainBuilder.builder(ExampleRobot.SIM.definition().drivetrain())
                    .corners(neos)
                    .build())
            .build();
    RobotConfig unwired = RobotConfigBuilder.builder(ExampleRobot.SIM).definition(rev).build();

    assertThat(unwired.definition()).isEqualTo(rev);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> TestWiring.talons(rev.drivetrain(), CanBus.SYSTEMCORE, 1, 1))
        .withMessage(
            "Drivetrain.wiring: its drive's NEO is on a TalonFX, which doesn't drive one: a"
                + " TalonFXS or a SPARK MAX or a SPARK Flex does");
  }

  /**
   * A drivetrain whose drive and steering are on different makers' controllers, each driving its
   * motors, is the definition's to have, and the template's modules can't drive it yet: refused as
   * the configuration is built, saying what they drive.
   */
  @Test
  void aSwerveOnTwoMakersControllersIsRefusedByTheProgram() {
    List<Drivetrain.Corner> neoSteering =
        ExampleRobot.SIM.definition().drivetrain().corners().stream()
            .map(corner ->
                DrivetrainCornerBuilder.builder(corner)
                    .module(
                        DrivetrainModuleBuilder.builder(corner.module())
                            .steerMotor(Motor.NEO_550)
                            .build())
                    .build())
            .toList();
    Drivetrain drivetrain =
        DrivetrainBuilder.builder(ExampleRobot.SIM.definition().drivetrain())
            .corners(neoSteering)
            .build();
    RobotConfig unwired =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .drivetrain(drivetrain)
                    .build())
            .build();
    Drivetrain mixed =
        TestWiring.wired(
            drivetrain,
            SwerveWiring.PIGEON_2.can(1),
            List.of(
                sparkSteered(1, 2), sparkSteered(3, 4), sparkSteered(5, 6), sparkSteered(7, 8)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> unwired.with(mixed))
        .withMessage(
            "RobotConfig[sim].definition.drivetrain.wiring: the drivetrain drives on a TalonFX and"
                + " steers on a SPARK MAX, and the template's modules are TalonFXs or SPARKs, doing"
                + " both, so far: put both on TalonFXs, or both on SPARKs");
  }

  /**
   * A module driven by a TalonFX and steered by a SPARK, which reads its angle on its data port: no
   * CANcoder.
   */
  private static SwerveWiring.ModuleWiring sparkSteered(int drive, int steer) {
    return SwerveWiringModuleWiringBuilder.builder()
        .drive(MechanismWiring.Controller.TALON_FX.can(drive))
        .steering(SPARK_MAX.can(steer).clockwise())
        .encoder(MechanismWiring.DATA_PORT.offset(Rotations.of(0)))
        .build();
  }
}

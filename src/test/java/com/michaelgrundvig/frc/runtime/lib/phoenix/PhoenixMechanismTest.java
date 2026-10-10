package com.michaelgrundvig.frc.runtime.lib.phoenix;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Coupling;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Transmission;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;

/**
 * The helper's settings for a robot made up for it: an arm with a wrist, an elevator, a turret, a
 * flywheel and a differential, each conversion and each refusal on its own. The 2026 robots'
 * mechanisms are checked against their teams' settings in {@code
 * com.michaelgrundvig.frc.runtime.robots2026}.
 */
class PhoenixMechanismTest {
  private static final Body BODY = ExampleRobot.SIM.definition().body();
  private static final Vector3 RAISE = new Vector3(0, -1, 0); // turns +x up as it grows
  private static final Vector3 LOWER = new Vector3(0, 1, 0); // turns +x down as it grows
  private static final double TURN = 2 * Math.PI;
  private static final MechanismControl.NeutralMode BRAKE = MechanismControl.NeutralMode.BRAKE;

  /** The arm: 4 kg, 0.3 m out along its zero, horizontal; two Krakens through 60:1. */
  private static final Joint.Revolute ARM =
      revolute(
          "arm",
          Part.CHASSIS,
          new Translation3d(0.1, 0, 0.5),
          RAISE,
          Optional.of(Range.degrees(-30, 120)),
          Math.toRadians(-30),
          weighed("arm link", 4, new Translation3d(0.3, 0, 0)));

  /** A wrist on the arm's end, which the arm's turning tilts. */
  private static final Joint.Revolute WRIST =
      revolute(
          "wrist",
          ARM,
          new Translation3d(0.6, 0, 0),
          RAISE,
          Optional.of(Range.degrees(-90, 90)),
          0,
          weighed("claw", 1, new Translation3d(0.1, 0, 0)));

  /** An elevator's carriage, 5 kg, on a 2 cm drum through 10:1. */
  private static final Joint.Prismatic ELEVATOR =
      Joint.Prismatic.builder()
          .label("elevator")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(new Translation3d(-0.2, 0, 0.1), Rotation3d.ZERO))
          .axis(Vector3.UP)
          .link(weighed("carriage", 5, Translation3d.ZERO))
          .range(Range.meters(0, 1.2))
          .build();

  /** A turret, unweighed, on 340° of travel. */
  private static final Joint.Revolute TURRET =
      revolute(
          "turret",
          Part.CHASSIS,
          new Translation3d(0, 0, 0.3),
          Vector3.UP,
          Optional.of(Range.degrees(-160, 180)),
          0,
          unseen("turret plate"));

  /** A flywheel, which turns freely. */
  private static final Joint.Revolute FLYWHEEL =
      revolute(
          "flywheel",
          Part.CHASSIS,
          new Translation3d(-0.2, 0, 0.4),
          LOWER,
          Optional.empty(),
          0,
          unseen("wheels"));

  private static final Drive ARM_MOTORS =
      JointFixtures.motors(ARM, Motor.KRAKEN_X60, 2, gearing(60));
  private static final Drive WRIST_MOTOR =
      JointFixtures.motors(WRIST, Motor.KRAKEN_X44, 1, gearing(30));
  private static final Drive ELEVATOR_MOTORS =
      JointFixtures.motors(ELEVATOR, Motor.KRAKEN_X60, 2, gearing(10), Meters.of(0.02));
  private static final Drive TURRET_MOTOR =
      JointFixtures.motors(TURRET, Motor.KRAKEN_X60, 1, gearing(50));
  private static final Drive FLYWHEEL_MOTOR =
      JointFixtures.motors(FLYWHEEL, Motor.KRAKEN_X60, 1, gearing(1));

  /** The turret's CANcoder, geared to turn back as the turret turns forward. */
  private static final Sensor.Encoder TURRET_CANCODER =
      Sensor.Encoder.builder()
          .label("turret cancoder")
          .terms(List.of(term(TURRET, -1)))
          .absolute(true)
          .countsPerTurn(4096)
          .build();

  /** The arm's CANcoder, on its shaft. */
  private static final Sensor.Encoder ARM_CANCODER = encoder("arm cancoder", ARM, 1);

  private final RobotMath math =
      RobotMath.of(
          BODY,
          List.of(ARM, WRIST, ELEVATOR, TURRET, FLYWHEEL),
          List.of(),
          List.of(ARM_MOTORS, WRIST_MOTOR, ELEVATOR_MOTORS, TURRET_MOTOR, FLYWHEEL_MOTOR),
          List.of(TURRET_CANCODER, ARM_CANCODER));

  // ---- Ratios, directions, limits and followers ----

  @Test
  void anArmsRotorReadsItThroughItsGearingInItsRotations() {
    PhoenixMechanism.Settings arm = configure(math, ARM, ARM_MOTORS, wiring(11, 12), control());
    TalonFXConfiguration leader = arm.leader();

    assertThat(leader.Feedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.RotorSensor);
    assertThat(leader.Feedback.SensorToMechanismRatio).isEqualTo(60);
    assertThat(leader.Feedback.RotorToSensorRatio).as("Phoenix's default").isEqualTo(1);
    assertThat(leader.MotorOutput.Inverted).isEqualTo(InvertedValue.CounterClockwise_Positive);
    assertThat(leader.MotorOutput.NeutralMode).isEqualTo(NeutralModeValue.Brake);
    assertThat(leader.ClosedLoopGeneral.ContinuousWrap).as("it has stops").isFalse();
    assertThat(arm.cancoder()).isEmpty();
  }

  @Test
  void everyCurrentLimitIsSetWhereItShows() {
    TalonFXConfiguration leader =
        configure(math, ARM, ARM_MOTORS, wiring(11, 12), control()).leader();

    assertThat(leader.CurrentLimits.StatorCurrentLimitEnable).isTrue();
    assertThat(leader.CurrentLimits.StatorCurrentLimit).isEqualTo(60);
    assertThat(leader.CurrentLimits.SupplyCurrentLimitEnable).isTrue();
    assertThat(leader.CurrentLimits.SupplyCurrentLimit).isEqualTo(50);
    assertThat(leader.CurrentLimits.SupplyCurrentLowerLimit).isEqualTo(35);
    assertThat(leader.CurrentLimits.SupplyCurrentLowerTime).isEqualTo(0.5);
  }

  @Test
  void aMotorTurningClockwiseToRaiseTheJointIsInverted() {
    List<MechanismWiring.Device> wiring = wiring(motor(11, true), motor(12, true));

    TalonFXConfiguration leader = configure(math, ARM, ARM_MOTORS, wiring, coasting()).leader();

    assertThat(leader.MotorOutput.Inverted).isEqualTo(InvertedValue.Clockwise_Positive);
    assertThat(leader.MotorOutput.NeutralMode).isEqualTo(NeutralModeValue.Coast);
  }

  /**
   * A follower turning the leader's way is aligned with it, one turning the other way opposed: the
   * wiring says which way each turns as the joint moves forward, and alignment follows.
   */
  @Test
  void followersAreAlignedOrOpposedByWhichWayTheyTurn() {
    PhoenixMechanism.Settings same =
        configure(math, ARM, ARM_MOTORS, wiring(motor(11, false), motor(12, false)), control());
    PhoenixMechanism.Settings facing =
        configure(math, ARM, ARM_MOTORS, wiring(motor(11, false), motor(12, true)), control());
    PhoenixMechanism.FollowerSettings follower = facing.followers().get(0);

    assertThat(same.followers().get(0).request().MotorAlignment)
        .isEqualTo(MotorAlignmentValue.Aligned);
    assertThat(follower.request().MotorAlignment).isEqualTo(MotorAlignmentValue.Opposed);
    assertThat(follower.request().LeaderID).isEqualTo(11);
    assertThat(follower.id()).isEqualTo(12);
    assertThat(follower.configuration().MotorOutput.Inverted)
        .as("its own direction, for its own readings")
        .isEqualTo(InvertedValue.Clockwise_Positive);
    assertThat(follower.configuration().Feedback.SensorToMechanismRatio).isEqualTo(60);
    assertThat(follower.configuration().CurrentLimits.StatorCurrentLimit).isEqualTo(60);
    assertThat(follower.configuration().MotorOutput.NeutralMode).isEqualTo(NeutralModeValue.Brake);
  }

  @Test
  void aDrivesMotorsAreAllWired() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(math, ARM, ARM_MOTORS, wiring(11), control()))
        .withMessage("arm is 2 motors, and its wiring names 1");
  }

  /** Soft limits past the stops are the definition's to refuse, before a helper sees them. */
  @Test
  void aSoftLimitIsInPhoenixsRotations() {
    MechanismControl limited = control(soft(Math.toRadians(-20), Math.toRadians(110)));

    TalonFXConfiguration leader =
        configure(math, ARM, ARM_MOTORS, wiring(11, 12), limited).leader();

    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitEnable).isTrue();
    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold)
        .isCloseTo(110.0 / 360, within(1e-12));
    assertThat(leader.SoftwareLimitSwitch.ReverseSoftLimitEnable).isTrue();
    assertThat(leader.SoftwareLimitSwitch.ReverseSoftLimitThreshold)
        .isCloseTo(-20.0 / 360, within(1e-12));
  }

  @Test
  void noSoftLimitsLeavesPhoenixsOff() {
    TalonFXConfiguration leader =
        configure(math, ARM, ARM_MOTORS, wiring(11, 12), control()).leader();

    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitEnable).isFalse();
    assertThat(leader.SoftwareLimitSwitch.ReverseSoftLimitEnable).isFalse();
  }

  @Test
  void aJointThatTurnsFreelyTakesTheShortWayRound() {
    TalonFXConfiguration leader =
        configure(math, FLYWHEEL, FLYWHEEL_MOTOR, wiring(20), control()).leader();

    assertThat(leader.ClosedLoopGeneral.ContinuousWrap).isTrue();
    assertThat(leader.Feedback.SensorToMechanismRatio).isEqualTo(1);
  }

  @Test
  void anElevatorIsInItsDrumsRotations() {
    PhoenixMechanism.Settings elevator =
        configure(
            math,
            ELEVATOR,
            ELEVATOR_MOTORS,
            wiring(30, 31),
            control(seeded(), Optional.of(soft(0.1, 1.0)), Optional.of(profile(1, 3)), BRAKE));
    TalonFXConfiguration leader = elevator.leader();
    double drum = TURN * 0.02;

    assertThat(elevator.rotations().perRotation())
        .as("a turn of its drum")
        .isCloseTo(drum, within(1e-15));
    assertThat(leader.Feedback.SensorToMechanismRatio)
        .as("the gearing")
        .isCloseTo(10, within(1e-12));
    assertThat(leader.SoftwareLimitSwitch.ReverseSoftLimitThreshold)
        .isCloseTo(0.1 / drum, within(1e-12));
    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold)
        .isCloseTo(1.0 / drum, within(1e-12));
    assertThat(leader.ClosedLoopGeneral.ContinuousWrap).isFalse();
  }

  /**
   * A winch, 100:1 on a ½ in spool: its rotor turns 2506 times a meter, past Phoenix's 1000, but
   * its Phoenix unit is the spool's rotation, so its ratio is the gearing, 100. Its soft limit, in
   * meters in its control, is in spool rotations in Phoenix.
   */
  @Test
  void aWinchIsInItsSpoolsRotations() {
    Joint.Prismatic winch =
        Joint.Prismatic.builder()
            .label("winch")
            .parent(Part.CHASSIS)
            .origin(Transform3d.ZERO)
            .axis(Vector3.UP)
            .link(unseen("hook"))
            .range(Range.meters(0, 0.5))
            .build();
    Drive slow = JointFixtures.motors(winch, Motor.KRAKEN_X60, 1, gearing(100), Inches.of(0.25));
    RobotMath robot = RobotMath.of(BODY, List.of(winch), List.of(), List.of(slow), List.of());
    double spool = TURN * 0.25 * 0.0254;

    TalonFXConfiguration leader =
        configure(
                robot,
                winch,
                slow,
                wiring(1),
                control(seeded(), Optional.of(soft(0, 0.5)), Optional.empty(), BRAKE))
            .leader();

    assertThat(slow.rotor().ratio(winch)).isGreaterThan(PhoenixMechanism.MAX_RATIO);
    assertThat(leader.Feedback.SensorToMechanismRatio).isCloseTo(100, within(1e-9));
    assertThat(leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold)
        .isCloseTo(0.5 / spool, within(1e-9));
    assertThat(leader.Slot0.kV)
        .as("12 V for the spool's 1 rps, a Kraken's 100 rps through 100:1")
        .isCloseTo(12, within(1e-9));
  }

  /** Geared past 1000:1, a ratio Phoenix can't hold is refused. */
  @Test
  void aRatioPhoenixCantHoldIsRefused() {
    Drive crawl = JointFixtures.motors(TURRET, Motor.KRAKEN_X60, 1, gearing(1500));
    RobotMath robot = RobotMath.of(BODY, List.of(TURRET), List.of(), List.of(crawl), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(robot, TURRET, crawl, wiring(1), control()))
        .withMessage(
            "turret's SensorToMechanismRatio would be 1500, and Phoenix holds from -1000 to 1000,"
                + " never 0");
  }

  @Test
  void onlyMotorsAreConfigured() {
    Drive servo =
        Drive.builder()
            .label("arm servo")
            .actuator(Actuator.Servo.builder().from(0).to(1).force(1).speed(1).build())
            .transmission(Transmission.Geared.builder().joint(ARM).gearing(gearing(1)).build())
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(math, ARM, servo, wiring(11), control()))
        .withMessage("arm servo isn't motors: a motor controller drives motors");
  }

  @Test
  void theDriveMustMoveTheJoint() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(math, WRIST, ARM_MOTORS, wiring(1, 2), control()))
        .withMessage("arm doesn't move wrist");
  }

  // ---- Gains and gravity ----

  /**
   * The seeds, in Phoenix's units: a Kraken X60's 12 V over its 100 rps, through 60:1, is 7.2 V per
   * arm rotation per second; the feedback gains, per radian in the control, per rotation in
   * Phoenix.
   */
  @Test
  void theSeedsAndTheFeedbackGainsArePerRotation() {
    MechanismControl tuned =
        control(gains(2, 0.5, 0.1, 0.2, Optional.empty(), Optional.empty(), Optional.empty()));

    TalonFXConfiguration leader = configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned).leader();
    RobotMath.Gains seed = math.idealGains(ARM);

    assertThat(leader.Slot0.kV).isCloseTo(7.2, within(1e-9));
    assertThat(leader.Slot0.kV).isCloseTo(seed.kV() * TURN, within(1e-12));
    assertThat(leader.Slot0.kA).isCloseTo(seed.kA() * TURN, within(1e-12));
    assertThat(leader.Slot0.kP).isCloseTo(2 * TURN, within(1e-12));
    assertThat(leader.Slot0.kI).isCloseTo(0.5 * TURN, within(1e-12));
    assertThat(leader.Slot0.kD).isCloseTo(0.1 * TURN, within(1e-12));
    assertThat(leader.Slot0.kS).as("volts").isEqualTo(0.2);
  }

  @Test
  void anArmHoldsAgainstOneCosineFromItsDefinition() {
    PhoenixMechanism.Settings arm = configure(math, ARM, ARM_MOTORS, wiring(11, 12), control());
    RobotMath.Gains seed = math.idealGains(ARM);

    assertThat(arm.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Arm_Cosine);
    assertThat(arm.leader().Slot0.GravityArmPositionOffset)
        .as("its center of mass on its zero, horizontal")
        .isCloseTo(0, within(1e-12));
    assertThat(arm.leader().Slot0.kG).isCloseTo(seed.kG(), within(1e-12)).isPositive();
    assertThat(arm.notes()).isEmpty();
  }

  @Test
  void tunedGainsWinOverTheSeeds() {
    double idealKv = math.idealGains(ARM).kV();
    MechanismControl tuned =
        control(gains(0, 0, 0, 0, Optional.of(idealKv * 1.1), Optional.of(0.05), Optional.of(0.3)));

    PhoenixMechanism.Settings arm = configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned);

    assertThat(arm.leader().Slot0.kV).isCloseTo(idealKv * 1.1 * TURN, within(1e-12));
    assertThat(arm.leader().Slot0.kA).isCloseTo(0.05 * TURN, within(1e-12));
    assertThat(arm.leader().Slot0.kG).isEqualTo(0.3);
    assertThat(arm.notes()).as("each near its ideal").isEmpty();
  }

  /**
   * A kV worked out per turn of the arm and entered as if per turn of its rotor, converted to the
   * control's radians from there: 60 times the ideal, the gearing, named.
   */
  @Test
  void aTunedKvInAnotherFrameIsNamed() {
    double perArmTurn = math.idealGains(ARM).kV() * TURN;
    double enteredPerRotorTurn = perArmTurn * 60 / TURN; // in V/(rad/s), as the control takes it
    MechanismControl tuned =
        control(
            gains(
                0, 0, 0, 0, Optional.of(enteredPerRotorTurn), Optional.empty(), Optional.empty()));

    List<String> notes = configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned).notes();

    assertThat(notes)
        .containsExactly(
            "arm's tuned kV, 68.75 V/(rad/s), is 60 times the ideal, 1.146 V/(rad/s), from its"
                + " motors and what it moves: the ideal per turn of arm, within 1%, entered as if"
                + " per turn of its rotor, by the look of it");
  }

  @Test
  void aTunedKvFarFromTheIdealInNoFrameIsStillNoted() {
    double ideal = math.idealGains(ARM).kV();
    MechanismControl tuned =
        control(gains(0, 0, 0, 0, Optional.of(ideal * 3.7), Optional.empty(), Optional.empty()));

    assertThat(configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned).notes())
        .singleElement()
        .asString()
        .startsWith("arm's tuned kV, 4.24 V/(rad/s), is 3.7 times the ideal")
        .doesNotContain("look of it");
  }

  @Test
  void aTunedKgFarFromTheLinksWeightIsNoted() {
    double ideal = math.idealGains(ARM).kG();
    MechanismControl tuned =
        control(gains(0, 0, 0, 0, Optional.empty(), Optional.empty(), Optional.of(ideal / 3)));

    assertThat(configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned).notes())
        .singleElement()
        .asString()
        .startsWith("arm's tuned kG, ")
        .contains("is 0.3333 times the ideal");
  }

  @Test
  void anElevatorHoldsItsCarriagesWeightEverywhere() {
    TalonFXConfiguration leader =
        configure(math, ELEVATOR, ELEVATOR_MOTORS, wiring(30, 31), control()).leader();
    RobotMath.Gains seed = math.idealGains(ELEVATOR);

    assertThat(leader.Slot0.GravityType).isEqualTo(GravityTypeValue.Elevator_Static);
    assertThat(leader.Slot0.kG).isCloseTo(seed.kG(), within(1e-12)).isPositive();
    assertThat(leader.Slot0.kV)
        .as("per drum rotation")
        .isCloseTo(seed.kV() * TURN * 0.02, within(1e-12));
    assertThat(leader.Slot0.kA).isCloseTo(seed.kA() * TURN * 0.02, within(1e-12));
  }

  @Test
  void aWristOnAnArmIsHeldOnlyWithTheArmAtItsReference() {
    PhoenixMechanism.Settings wrist = configure(math, WRIST, WRIST_MOTOR, wiring(40), control());

    assertThat(wrist.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Arm_Cosine);
    assertThat(wrist.notes())
        .containsExactly(
            "wrist rides arm, whose turning tilts its gravity: a controller's gravity term holds it"
                + " only with arm at rest, so feed RobotMath.gravityTorque at the joints'"
                + " positions as the request's feedforward");
  }

  /**
   * An unweighed arm has no gravity seed: kG 0, its weight taken to pull hardest at its zero, until
   * kG is tuned. The definition, not the helper, says it has none ({@code
   * RobotDefinition.missing}); an unweighed turret or flywheel needs none, since gravity doesn't
   * turn it however much it weighs.
   */
  @Test
  void anUnweighedArmHasNoGravitySeed() {
    Joint.Revolute bare = bareArm();
    Drive motor = JointFixtures.motors(bare, Motor.KRAKEN_X60, 1, gearing(40));
    RobotMath robot = RobotMath.of(BODY, List.of(bare), List.of(), List.of(motor), List.of());
    PhoenixMechanism.Settings arm = configure(robot, bare, motor, wiring(1), control());

    assertThat(arm.leader().Slot0.kG).isZero();
    assertThat(arm.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Arm_Cosine);
    assertThat(arm.leader().Slot0.GravityArmPositionOffset).isZero();
    assertThat(arm.notes()).isEmpty();
    assertThat(configure(math, TURRET, TURRET_MOTOR, wiring(50), control()).notes()).isEmpty();
    assertThat(configure(math, FLYWHEEL, FLYWHEEL_MOTOR, wiring(20), control()).notes()).isEmpty();
  }

  /**
   * A tuned kG needs no mass: an unweighed arm takes it along one cosine about its zero, horizontal
   * as WPILib takes an arm's, or about the angle given with it, where its weight pulls hardest.
   */
  @Test
  void anUnweighedArmTakesItsTunedKgAboutItsZeroOrTheAngleGiven() {
    Joint.Revolute bare = bareArm();
    Drive motor = JointFixtures.motors(bare, Motor.KRAKEN_X60, 1, gearing(40));
    RobotMath robot = RobotMath.of(BODY, List.of(bare), List.of(), List.of(motor), List.of());

    PhoenixMechanism.Settings atZero =
        configure(robot, bare, motor, wiring(1), control().withKg(0.4));
    PhoenixMechanism.Settings hanging =
        configure(robot, bare, motor, wiring(1), control().withKg(0.4, Degrees.of(-90)));
    PhoenixMechanism.Settings tipped =
        configure(robot, bare, motor, wiring(1), control().withKg(0.4, Degrees.of(30)));

    assertThat(atZero.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Arm_Cosine);
    assertThat(atZero.leader().Slot0.kG).isEqualTo(0.4);
    assertThat(atZero.leader().Slot0.GravityArmPositionOffset).isZero();
    assertThat(atZero.notes()).isEmpty();
    assertThat(hanging.leader().Slot0.GravityArmPositionOffset)
        .as("a quarter turn: Phoenix's limit, taken as given")
        .isCloseTo(0.25, within(1e-12));
    assertThat(hanging.notes()).as("a given angle doesn't move with CAD").isEmpty();
    assertThat(tipped.leader().Slot0.GravityArmPositionOffset).isCloseTo(-1.0 / 12, within(1e-12));
  }

  /**
   * The slot is the plan's law, per rotation: its gains, the plan's feedforward gains, and its
   * gravity's kG and where it pulls hardest, so a TalonFX runs what the robot would work out
   * itself.
   */
  @Test
  void theSlotIsThePlansLawPerRotation() {
    ControllerPlan plan =
        ControllerPlan.of(
            math,
            ARM,
            ARM_MOTORS,
            wiring(11, 12),
            control(gains(2, 0, 0.5, 0.1, Optional.empty(), Optional.empty(), Optional.of(0.3)))
                .withKg(0.3, Degrees.of(10)));
    SlotConfigs slot = SlotConfigs.from(PhoenixMechanism.configure(plan).leader().Slot0);

    assertThat(slot.kP).isCloseTo(plan.kP() * TURN, within(1e-12));
    assertThat(slot.kD).isCloseTo(plan.kD() * TURN, within(1e-12));
    assertThat(slot.kS).isEqualTo(plan.kS());
    assertThat(slot.kV).isCloseTo(plan.kV() * TURN, within(1e-12));
    assertThat(slot.kA).isCloseTo(plan.kA() * TURN, within(1e-12));
    assertThat(slot.kG).isEqualTo(plan.gravity().kG());
    assertThat(slot.GravityArmPositionOffset)
        .isCloseTo(plan.gravity().offset() / TURN, within(1e-12))
        .isCloseTo(-10.0 / 360, within(1e-12));
  }

  /** A gravity angle given wins over where the links' mass puts the arm's weight. */
  @Test
  void aGivenGravityAngleWinsOverTheMasses() {
    PhoenixMechanism.Settings arm =
        configure(math, ARM, ARM_MOTORS, wiring(11, 12), control().withKg(0.3, Degrees.of(10)));

    assertThat(math.gravityOffset(ARM)).as("its mass on its zero").isCloseTo(0, within(1e-12));
    assertThat(arm.leader().Slot0.GravityArmPositionOffset).isCloseTo(-10.0 / 360, within(1e-12));
    assertThat(arm.leader().Slot0.kG).isEqualTo(0.3);
  }

  /** A gravity angle more than a quarter turn from the zero is past Phoenix's offset. */
  @Test
  void aGravityAnglePastPhoenixsOffsetIsRefusedWithTheTurnedAngle() {
    Joint.Revolute bare = bareArm();
    Drive motor = JointFixtures.motors(bare, Motor.KRAKEN_X60, 1, gearing(40));
    RobotMath robot = RobotMath.of(BODY, List.of(bare), List.of(), List.of(motor), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> configure(robot, bare, motor, wiring(1), control().withKg(0.4, Degrees.of(120))))
        .withMessage(
            "bare arm's gravity angle, 120°, is more than a quarter turn from its zero, past"
                + " Phoenix's gravity offset of ±0.25 rotations: its positive direction lowers it"
                + " at its zero. Turn its axis around in the definition (and its motors' and"
                + " encoder's directions in the wiring), so that positive raises it; its gravity"
                + " angle is then 60°");
    assertThat(
            configure(robot, bare, motor, wiring(1), control().withKg(0.4, Degrees.of(270)))
                .leader()
                .Slot0
                .GravityArmPositionOffset)
        .as("270° is -90°")
        .isCloseTo(0.25, within(1e-12));
  }

  /** An unweighed carriage has no gravity seed; tuned, its kG holds it all along. */
  @Test
  void anUnweighedCarriageTakesItsTunedKg() {
    Joint.Prismatic slide = slide("slide", Vector3.UP);
    Drive motor = JointFixtures.motors(slide, Motor.KRAKEN_X60, 1, gearing(5), Meters.of(0.02));
    RobotMath robot = RobotMath.of(BODY, List.of(slide), List.of(), List.of(motor), List.of());
    PhoenixMechanism.Settings seeded = configure(robot, slide, motor, wiring(1), control());
    PhoenixMechanism.Settings tuned =
        configure(robot, slide, motor, wiring(1), control().withKg(0.5));

    assertThat(seeded.leader().Slot0.kG).isZero();
    assertThat(seeded.notes()).isEmpty();
    assertThat(tuned.leader().Slot0.GravityType).isEqualTo(GravityTypeValue.Elevator_Static);
    assertThat(tuned.leader().Slot0.kG).isEqualTo(0.5);
    assertThat(tuned.notes()).isEmpty();
  }

  /**
   * A tuned kG on a joint gravity can't move is refused, whatever it carries: a turret's would turn
   * it, and a level slide's would push it.
   */
  @Test
  void aTunedKgWhereGravityCantMoveTheJointIsRefused() {
    Joint.Prismatic level = slide("hopper", new Vector3(1, 0, 0));
    Drive motor = JointFixtures.motors(level, Motor.KRAKEN_X60, 1, gearing(5), Meters.of(0.02));
    RobotMath robot = RobotMath.of(BODY, List.of(level), List.of(), List.of(motor), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(math, TURRET, TURRET_MOTOR, wiring(50), control().withKg(0.3)))
        .withMessage(
            "turret has a tuned kG of 0.3 V, but gravity can't move it: it turns about the"
                + " vertical. Leave its kG out, or write 0");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(robot, level, motor, wiring(1), control().withKg(0.3)))
        .withMessage(
            "hopper has a tuned kG of 0.3 V, but gravity can't move it: it slides level. Leave its"
                + " kG out, or write 0");
    assertThat(
            configure(math, TURRET, TURRET_MOTOR, wiring(50), control().withKg(0))
                .leader()
                .Slot0
                .kG)
        .as("0 written is fine")
        .isZero();
  }

  @Test
  void voltsPhoenixCantHoldAreRefused() {
    MechanismControl tuned =
        control(gains(0, 0, 0, 200, Optional.empty(), Optional.empty(), Optional.empty()));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(math, ARM, ARM_MOTORS, wiring(11, 12), tuned))
        .withMessage("arm's kS, 200 V, is past what Phoenix holds, -128 to 127 V");
  }

  /**
   * An arm whose zero points straight up has its gravity offset at Phoenix's limit, a quarter turn:
   * Phoenix takes it, and the helper says how close it is.
   */
  @Test
  void anArmZeroedStraightUpSitsAtPhoenixsLimit() {
    Joint.Revolute upright = upright(0);
    Drive motor = JointFixtures.motors(upright, Motor.KRAKEN_X60, 1, gearing(25));
    RobotMath robot = RobotMath.of(BODY, List.of(upright), List.of(), List.of(motor), List.of());

    PhoenixMechanism.Settings settings = configure(robot, upright, motor, wiring(1), control());

    assertThat(settings.leader().Slot0.GravityArmPositionOffset).isCloseTo(0.25, within(1e-12));
    assertThat(settings.notes())
        .containsExactly(
            "upright's gravity offset is 0.25 rotations, at Phoenix's limit of ±0.25: its center of"
                + " mass is 0° from straight above or below the pivot at its zero, and a CAD"
                + " change that moves it past makes the offset one Phoenix can't take. Zero"
                + " upright nearer horizontal in CAD");
  }

  /**
   * Its center of mass two degrees forward of straight up, Phoenix can't take it, and the helper
   * says what can.
   */
  @Test
  void anOffsetPastPhoenixsLimitIsRefused() {
    Joint.Revolute upright = upright(Math.toRadians(2));
    Drive motor = JointFixtures.motors(upright, Motor.KRAKEN_X60, 1, gearing(25));
    RobotMath robot = RobotMath.of(BODY, List.of(upright), List.of(), List.of(motor), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(robot, upright, motor, wiring(1), control()))
        .withMessage(
            "upright's gravity offset is 0.2556 rotations, past Phoenix's ±0.25: its positive"
                + " direction lowers it. Turn its axis around in the definition (and its motors'"
                + " and encoder's directions in the wiring), so that positive raises it; the offset"
                + " is then 0.2444");
  }

  /** An arm zeroed level whose positive direction lowers it: half a turn off. */
  @Test
  void anArmWhosePositiveLowersItIsRefusedWithTheTurnedOffset() {
    Joint.Revolute lowering =
        revolute(
            "lowering",
            Part.CHASSIS,
            new Translation3d(0, 0, 0.5),
            LOWER,
            Optional.of(Range.degrees(-30, 90)),
            0,
            weighed("lowering link", 2, new Translation3d(0.3, 0, 0)));
    Drive motor = JointFixtures.motors(lowering, Motor.KRAKEN_X60, 1, gearing(25));
    RobotMath robot = RobotMath.of(BODY, List.of(lowering), List.of(), List.of(motor), List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> configure(robot, lowering, motor, wiring(1), control()))
        .withMessageContaining("is 0.5 rotations, past Phoenix's ±0.25")
        .withMessageEndingWith("the offset is then 0");
  }

  /**
   * A flap the arm swings at half its angle, weighed: the arm's gravity isn't one cosine, and the
   * helper says by how much.
   */
  @Test
  void gravityOneCosineMissesIsNoted() {
    Joint.Revolute flap =
        revolute(
            "flap",
            Part.CHASSIS,
            new Translation3d(0.1, 0, 0.3),
            RAISE,
            Optional.of(Range.degrees(-15, 60)),
            Math.toRadians(-15),
            weighed("flap plate", 3, new Translation3d(0.4, 0, 0)));
    Coupling linkage =
        Coupling.Linear.builder().leader(ARM).follower(flap).ratio(0.5).offset(0).build();
    RobotMath robot =
        RobotMath.of(BODY, List.of(ARM, flap), List.of(linkage), List.of(ARM_MOTORS), List.of());

    List<String> notes = configure(robot, ARM, ARM_MOTORS, wiring(11, 12), control()).notes();

    assertThat(robot.gravityFit(ARM).misfit()).isGreaterThan(ControllerPlan.MISFIT);
    assertThat(notes)
        .singleElement()
        .asString()
        .startsWith("one cosine misses arm's gravity by ")
        .endsWith(
            "cosine can't follow it, so feed RobotMath.gravityTorque at the joint's angle as the"
                + " request's feedforward");
  }

  // ---- CANcoders ----

  /**
   * A CANcoder on the arm's shaft, read remotely: the TalonFX reads the arm through it, 1:1, and
   * its rotor 60 turns for each of the CANcoder's.
   */
  @Test
  void aRemoteCancoderReadsTheJointAndItsRotorTheGearing() {
    PhoenixMechanism.Settings arm =
        configure(
            math,
            ARM,
            ARM_MOTORS,
            ARM_CANCODER,
            cancoder(MechanismWiring.Feedback.REMOTE),
            wiring(11, 12),
            control());
    TalonFXConfiguration leader = arm.leader();
    CANcoderConfiguration cancoder = arm.cancoder().orElseThrow();

    assertThat(leader.Feedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.RemoteCANcoder);
    assertThat(leader.Feedback.FeedbackRemoteSensorID).isEqualTo(9);
    assertThat(leader.Feedback.SensorToMechanismRatio).isEqualTo(1);
    assertThat(leader.Feedback.RotorToSensorRatio).isEqualTo(60);
    assertThat(arm.followers().get(0).configuration().Feedback.SensorToMechanismRatio)
        .as("a follower reads its own rotor")
        .isEqualTo(60);
    assertThat(cancoder.MagnetSensor.MagnetOffset).isEqualTo(-0.125);
    assertThat(cancoder.MagnetSensor.SensorDirection)
        .isEqualTo(SensorDirectionValue.Clockwise_Positive);
    assertThat(cancoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint)
        .as("the middle of -30° to 120°, plus half a turn")
        .isCloseTo(45.0 / 360 + 0.5, within(1e-12));
    assertThat(arm.notes()).isEmpty();
  }

  @Test
  void eachFeedbackIsItsSource() {
    for (var feedback : MechanismWiring.Feedback.values()) {
      TalonFXConfiguration leader =
          configure(
                  math,
                  ARM,
                  ARM_MOTORS,
                  ARM_CANCODER,
                  cancoder(feedback),
                  wiring(11, 12),
                  control())
              .leader();
      FeedbackSensorSourceValue expected =
          switch (feedback) {
            case ROTOR -> FeedbackSensorSourceValue.RotorSensor;
            case REMOTE -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FUSED -> FeedbackSensorSourceValue.FusedCANcoder;
            case SYNC -> FeedbackSensorSourceValue.SyncCANcoder;
          };

      assertThat(leader.Feedback.FeedbackSensorSource).as(feedback.name()).isEqualTo(expected);
      assertThat(leader.Feedback.SensorToMechanismRatio)
          .as(feedback.name())
          .isEqualTo(feedback == MechanismWiring.Feedback.ROTOR ? 60 : 1);
    }
  }

  /**
   * A CANcoder geared to turn back as the turret turns forward reads forward once its wiring says
   * it turns the other way: its ratio is a magnitude, and its wrap is the definition's turned
   * round.
   */
  @Test
  void aCancoderGearedBackwardsWrapsWhereTheDefinitionsTurnedRoundDoes() {
    double definitions = math.discontinuityPoint(TURRET_CANCODER);
    PhoenixMechanism.Settings turret =
        configure(
            math,
            TURRET,
            TURRET_MOTOR,
            TURRET_CANCODER,
            cancoder(MechanismWiring.Feedback.FUSED),
            wiring(50),
            control());

    assertThat(turret.leader().Feedback.SensorToMechanismRatio).isEqualTo(1);
    assertThat(turret.leader().Feedback.RotorToSensorRatio).isEqualTo(50);
    assertThat(turret.cancoder().orElseThrow().MagnetSensor.AbsoluteSensorDiscontinuityPoint)
        .isCloseTo(1 - definitions, within(1e-12));
  }

  /** The turret's 340° leave 10° either side of the wrap: closer than 9° would be noted. */
  @Test
  void aRangeNearlyATurnLeavesLittleMargin() {
    double point =
        PhoenixMechanism.cancoder(math, TURRET_CANCODER, cancoder(MechanismWiring.Feedback.REMOTE))
            .MagnetSensor
            .AbsoluteSensorDiscontinuityPoint;

    assertThat(ControllerPlan.wrapMargin(TURRET_CANCODER, point))
        .isCloseTo(10.0 / 360, within(1e-12));
    assertThat(ControllerPlan.wrapProblem(TURRET_CANCODER, point)).isNull();
    assertThat(ControllerPlan.wrapProblem(TURRET_CANCODER, point + 0.01))
        .isEqualTo(
            "turret cancoder wraps at 0.5378 of its turn, 6.4° from turret's range (-160° to"
                + " 180°), and 166.4° from where it stows (0°): a reading a hair past the"
                + " wrap reads a turn away");
  }

  /** A team's own wrap, across the range: where the arm reads a turn away inside its travel. */
  @Test
  void aWrapInsideTheRangeIsNamed() {
    assertThat(ControllerPlan.wrapMargin(ARM_CANCODER, 1.0)).isNegative();
    assertThat(ControllerPlan.wrapProblem(ARM_CANCODER, 1.0))
        .startsWith("arm cancoder wraps at 1 of its turn, inside arm's range");
    assertThat(ControllerPlan.wrapMargin(encoder("flywheel encoder", FLYWHEEL, 1), 0.5))
        .as("nothing to keep clear")
        .isInfinite();
  }

  /**
   * An elevator's CANcoder, geared twelve to one down from its 2 cm drum, as its block writes it:
   * its turns are counted in the drum's rotations, a twelfth of one each, so the TalonFX reads the
   * carriage through it fused, in the drum's rotations, and its rotor turns 120 times for each of
   * the CANcoder's. Read once instead, the TalonFX reads its rotor through the gearing. Its wrap is
   * as far from the travel as a turn allows: 1.2 m is 0.796 of the CANcoder's turns, centered.
   */
  @Test
  void anElevatorsCancoderGearedToItsDrumIsInTheDrumsRotations() {
    Joint.Prismatic elevator =
        ELEVATOR
            .motors(
                Motor.KRAKEN_X60,
                gearing(10),
                Meters.of(0.02),
                MechanismWiring.Controller.TALON_FX.can(30),
                MechanismWiring.Controller.TALON_FX.can(31))
            .absoluteEncoder(
                Driven.AbsoluteEncoder.of()
                    .ratio(1.0 / 12)
                    .on(CANCODER.can(32).offset(Rotations.of(0.31)).fused()));
    Sensor.Encoder encoder = (Sensor.Encoder) elevator.ownSensors().getFirst();
    RobotMath robot =
        RobotMath.of(BODY, List.of(elevator), List.of(), elevator.ownDrives(), List.of(encoder));
    MechanismWiring.Cancoder fused =
        elevator.driven().absoluteEncoder().orElseThrow().cancoder().orElseThrow();
    double travel = 1.2 / (TURN * 0.02) / 12; // the CANcoder's turns over the travel

    PhoenixMechanism.Settings settings =
        PhoenixMechanism.configure(
            robot, ControllerPlan.of(robot, elevator, control()), encoder, fused);
    PhoenixMechanism.Settings once =
        PhoenixMechanism.configure(
            robot,
            ControllerPlan.of(robot, elevator, control()),
            encoder,
            CANCODER.can(32).offset(Rotations.of(0.31)));
    TalonFXConfiguration leader = settings.leader();
    CANcoderConfiguration cancoder = settings.cancoder().orElseThrow();

    assertThat(leader.Feedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.FusedCANcoder);
    assertThat(leader.Feedback.FeedbackRemoteSensorID).isEqualTo(32);
    assertThat(leader.Feedback.SensorToMechanismRatio).isCloseTo(1.0 / 12, within(1e-12));
    assertThat(leader.Feedback.RotorToSensorRatio).isCloseTo(120, within(1e-9));
    assertThat(settings.followers().get(0).configuration().Feedback.SensorToMechanismRatio)
        .as("a follower reads its own rotor, through the gearing")
        .isCloseTo(10, within(1e-12));
    assertThat(once.leader().Feedback.FeedbackSensorSource)
        .isEqualTo(FeedbackSensorSourceValue.RotorSensor);
    assertThat(once.leader().Feedback.SensorToMechanismRatio).isCloseTo(10, within(1e-12));
    assertThat(cancoder.MagnetSensor.MagnetOffset).isCloseTo(0.31, within(1e-12));
    assertThat(cancoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint)
        .isCloseTo(travel / 2 + 0.5, within(1e-12));
    assertThat(settings.notes()).isEmpty();
    assertThat(ControllerPlan.wrapProblem(encoder, travel + 0.01))
        .as("a team's own wrap, said in meters")
        .isEqualTo(
            "elevator absolute encoder wraps at 0.8058 of its turn, 0.0151 m from elevator's range"
                + " (0 m to 1.2 m), and 0.2929 m from where it stows (0 m): a reading a hair past"
                + " the wrap reads a turn away");
  }

  @Test
  void aCancoderMustBeAbsolute() {
    Sensor.Encoder relative =
        Sensor.Encoder.builder()
            .label("arm quadrature")
            .terms(List.of(term(ARM, 1)))
            .absolute(false)
            .countsPerTurn(2048)
            .build();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                configure(
                    math,
                    ARM,
                    ARM_MOTORS,
                    relative,
                    cancoder(MechanismWiring.Feedback.REMOTE),
                    wiring(11, 12),
                    control()))
        .withMessage("arm quadrature must be an absolute encoder on arm alone, as a CANcoder is");
  }

  // ---- Coupling and differentials ----

  /** A differential wrist: pitch both motors turn together, roll they turn against each other. */
  private static final Joint.Revolute PITCH =
      revolute(
          "pitch",
          Part.CHASSIS,
          new Translation3d(0.2, 0, 0.6),
          RAISE,
          Optional.of(Range.degrees(-90, 90)),
          0,
          unseen("pitch yoke"));

  private static final Joint.Revolute ROLL =
      revolute(
          "roll",
          PITCH,
          Translation3d.ZERO,
          new Vector3(1, 0, 0),
          Optional.of(Range.degrees(-180, 180)),
          0,
          unseen("roll head"));

  private static final Drive LEFT = side("left", 20, 12);
  private static final Drive RIGHT = side("right", 20, -12);

  private final RobotMath wrist =
      RobotMath.of(BODY, List.of(PITCH, ROLL), List.of(), List.of(LEFT, RIGHT), List.of());

  @Test
  void aMotorTurningTwoJointsSaysPhoenixReadsBoth() {
    assertThat(configure(wrist, PITCH, LEFT, wiring(1), control()).notes().get(0))
        .isEqualTo(
            "left's rotor turns 12 times for each turn of roll too, which the controller reads as"
                + " pitch moving: take it out in code, as a swerve module's coupling is");
  }

  @Test
  void aCanivoreIsPhoenixsBusByName() {
    assertThat(PhoenixMechanism.bus(CanBus.canivore("arm bus")).getName()).isEqualTo("arm bus");
    assertThat(PhoenixMechanism.bus(CanBus.port(CanBus.Systemcore.CAN_S1)).getName())
        .isEqualTo("can_s1");
  }

  // ---- What the tests are built from ----

  private static Joint.Revolute revolute(
      String label,
      Part parent,
      Translation3d at,
      Vector3 axis,
      Optional<Range> range,
      double reference,
      Link link) {
    return Joint.Revolute.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .link(link)
        .range(range)
        .rest(reference)
        .build();
  }

  /**
   * An arm turning forward and down from straight up, its center of mass on its zero, or tipped
   * forward of it by an angle.
   */
  private static Joint.Revolute upright(double tipped) {
    return revolute(
        "upright",
        Part.CHASSIS,
        new Translation3d(0.2, 0, 0.3),
        LOWER,
        Optional.of(Range.degrees(0, 135)),
        Math.toRadians(1.5),
        weighed(
            "upright link",
            2,
            new Translation3d(0.3 * Math.sin(tipped), 0, 0.3 * Math.cos(tipped))));
  }

  private static Link weighed(String label, double kilograms, Translation3d cg) {
    return Link.builder()
        .label(label)
        .mass(
            MassProperties.builder()
                .mass(Kilograms.of(kilograms))
                .cg(cg)
                .inertia(Inertia.box(Kilograms.of(kilograms), new Translation3d(0.1, 0.1, 0.1)))
                .build())
        .build();
  }

  private static Link unseen(String label) {
    return Link.builder().label(label).mass(MassProperties.NONE).build();
  }

  private static Gearing gearing(double ratio) {
    return Gearing.builder().ratio(ratio).efficiency(1).build();
  }

  private static Transmission.Term term(Joint joint, double ratio) {
    return Transmission.Term.builder().joint(joint).ratio(ratio).build();
  }

  private static Sensor.Encoder encoder(String label, Joint joint, double ratio) {
    return Sensor.Encoder.builder()
        .label(label)
        .terms(List.of(term(joint, ratio)))
        .absolute(true)
        .countsPerTurn(4096)
        .build();
  }

  /** One side of the wrist: a Kraken X44 turning pitch and roll. */
  private static Drive side(String label, double pitch, double roll) {
    return Drive.builder()
        .label(label)
        .actuator(Actuator.Motors.builder().motor(Motor.KRAKEN_X44).count(1).build())
        .transmission(
            Transmission.Combined.builder()
                .terms(List.of(term(PITCH, pitch), term(ROLL, roll)))
                .efficiency(1)
                .build())
        .build();
  }

  /** An arm on a level parent, unweighed: its zero points forward, raised as it grows. */
  private static Joint.Revolute bareArm() {
    return revolute(
        "bare arm",
        Part.CHASSIS,
        new Translation3d(0, 0, 0.5),
        RAISE,
        Optional.of(Range.degrees(0, 90)),
        0,
        unseen("bare link"));
  }

  /** A slide along an axis, unweighed, half a meter long. */
  private static Joint.Prismatic slide(String label, Vector3 axis) {
    return Joint.Prismatic.builder()
        .label(label)
        .parent(Part.CHASSIS)
        .origin(Transform3d.ZERO)
        .axis(axis)
        .link(unseen(label + " carriage"))
        .range(Range.meters(0, 0.5))
        .build();
  }

  private static MechanismWiring.Device motor(int id, boolean clockwise) {
    MechanismWiring.Device talon = MechanismWiring.Controller.TALON_FX.can(id);
    return clockwise ? talon.clockwise() : talon;
  }

  /** TalonFXs at these IDs, the first leading, each turning counterclockwise for positive. */
  private static List<MechanismWiring.Device> wiring(int... ids) {
    List<MechanismWiring.Device> motors = new ArrayList<>();
    for (int id : ids) {
      motors.add(motor(id, false));
    }
    return motors;
  }

  private static List<MechanismWiring.Device> wiring(MechanismWiring.Device... motors) {
    return List.of(motors);
  }

  private static MechanismWiring.Cancoder cancoder(MechanismWiring.Feedback feedback) {
    Angle offset = Rotations.of(-0.125);
    MechanismWiring.Cancoder cancoder = CANCODER.can(9).offset(offset).clockwise();
    return switch (feedback) {
      case ROTOR -> cancoder;
      case REMOTE -> cancoder.remote();
      case FUSED -> cancoder.fused();
      case SYNC -> cancoder.sync();
    };
  }

  private static MechanismControl.Gains gains(
      double kP,
      double kI,
      double kD,
      double kS,
      Optional<Double> kV,
      Optional<Double> kA,
      Optional<Double> kG) {
    return MechanismControl.Gains.builder()
        .kP(kP)
        .kI(kI)
        .kD(kD)
        .kS(kS)
        .kV(kV)
        .kA(kA)
        .kG(kG)
        .build();
  }

  private static MechanismControl.Gains seeded() {
    return MechanismControl.Gains.seeded();
  }

  private static MechanismControl.SoftLimits soft(double lower, double upper) {
    return MechanismControl.SoftLimits.builder().lower(lower).upper(upper).build();
  }

  private static MechanismControl.Profile profile(double cruise, double acceleration) {
    return MechanismControl.Profile.builder()
        .cruiseVelocity(cruise)
        .acceleration(acceleration)
        .build();
  }

  private static MechanismControl control() {
    return control(
        seeded(), Optional.empty(), Optional.empty(), MechanismControl.NeutralMode.BRAKE);
  }

  private static MechanismControl coasting() {
    return control(
        seeded(), Optional.empty(), Optional.empty(), MechanismControl.NeutralMode.COAST);
  }

  private static MechanismControl control(MechanismControl.Gains gains) {
    return control(gains, Optional.empty(), Optional.empty(), MechanismControl.NeutralMode.BRAKE);
  }

  private static MechanismControl control(MechanismControl.SoftLimits limits) {
    return control(
        seeded(), Optional.of(limits), Optional.empty(), MechanismControl.NeutralMode.BRAKE);
  }

  private static MechanismControl control(
      MechanismControl.Gains gains,
      Optional<MechanismControl.SoftLimits> limits,
      Optional<MechanismControl.Profile> profile,
      MechanismControl.NeutralMode neutral) {
    return MechanismControl.builder()
        .loop(MechanismControl.Loop.POSITION)
        .gains(gains)
        .currentLimits(
            MechanismControl.CurrentLimits.builder()
                .stator(Amps.of(60))
                .supply(Amps.of(50))
                .supplyLower(Amps.of(35))
                .supplyLowerTime(Seconds.of(0.5))
                .build())
        .neutralMode(neutral)
        .deadChannel(MechanismControl.DeadChannel.DEFAULTS)
        .softLimits(limits)
        .profile(profile)
        .build();
  }

  /** The helper's settings for a mechanism's parts, through the plan the program works out. */
  private static PhoenixMechanism.Settings configure(
      RobotMath math,
      Joint joint,
      Drive drive,
      List<MechanismWiring.Device> controllers,
      MechanismControl control) {
    return PhoenixMechanism.configure(ControllerPlan.of(math, joint, drive, controllers, control));
  }

  /** The same, with a CANcoder. */
  private static PhoenixMechanism.Settings configure(
      RobotMath math,
      Joint joint,
      Drive drive,
      Sensor.Encoder encoder,
      MechanismWiring.Cancoder cancoder,
      List<MechanismWiring.Device> controllers,
      MechanismControl control) {
    return PhoenixMechanism.configure(
        math, ControllerPlan.of(math, joint, drive, controllers, control), encoder, cancoder);
  }
}

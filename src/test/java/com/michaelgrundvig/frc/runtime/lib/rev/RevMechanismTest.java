package com.michaelgrundvig.frc.runtime.lib.rev;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * The REV helper: a SPARK's settings, said from the program's plan for the mechanism in a SPARK's
 * units. The fixtures' pivot on two NEOs through 25:1.
 */
class RevMechanismTest {
  private static final RobotDefinition ROBOT =
      JointFixtures.ROBOT.with(
          JointFixtures.unmotored(PIVOT)
              .motors(Motor.NEO.through(JointFixtures.gearing(25)).count(2)));

  private static final RobotMath MATH = ROBOT.math();

  private static final Drive NEO = MATH.drivesOf(PIVOT).get(0);

  private static final MechanismControl CONTROL =
      MechanismControlBuilder.builder(ControllerSettings.control(PIVOT))
          .softLimits(MechanismControl.SoftLimits.builder().lower(0.1).upper(2.2).build())
          .build();

  /** Two motor controllers of a kind, at 41, turning clockwise, and 42, the other way. */
  private static List<MechanismWiring.Device> wiring(MechanismWiring.Controller controller) {
    return List.of(controller.can(41).clockwise(), controller.can(42));
  }

  private static final List<MechanismWiring.Device> SPARKS =
      wiring(MechanismWiring.Controller.SPARK_MAX);

  /**
   * Its loop's gain is the plan's kP per rotor turn as a share of 12 V, for positions, and per
   * rotor RPM for speeds; its soft limits are in rotor turns; and what the robot adds is the plan's
   * feedforward and kD, as a TalonFX's slot holds them per rotation.
   */
  @Test
  void itSaysThePlanInASparksUnits() {
    ControllerPlan plan = ControllerPlan.of(MATH, PIVOT, NEO, SPARKS, CONTROL);
    RevMechanism.Settings rev = RevMechanism.configure(plan);
    SlotConfigs slot = SlotConfigs.from(PhoenixMechanism.configure(plan).leader().Slot0);

    assertThat(rev.rotorRatio()).isCloseTo(25, within(1e-12));
    assertThat(rev.positionKp()).isCloseTo(slot.kP / (12 * 25), within(1e-12));
    assertThat(rev.velocityKp()).isCloseTo(slot.kP / (12 * 25 * 60), within(1e-15));
    assertThat(rev.softLimits().orElseThrow())
        .containsExactly(0.1 / (2 * Math.PI) * 25, 2.2 / (2 * Math.PI) * 25);
    assertThat(rev.plan()).isSameAs(plan);
    assertThat(rev.plan().kV() * 2 * Math.PI).isCloseTo(slot.kV, within(1e-12));
    assertThat(rev.plan().gravity().kG()).isEqualTo(slot.kG);
    assertThat(rev.plan().gravity().kind())
        .as("a pivot's gravity turns with it")
        .isEqualTo(ControllerPlan.Gravity.Kind.COSINE);
    assertThat(rev.plan().gravity().offset() / (2 * Math.PI))
        .isCloseTo(slot.GravityArmPositionOffset, within(1e-12));
    assertThat(rev.plan().kD() * 2 * Math.PI).isCloseTo(slot.kD, within(1e-12));
    assertThat(rev.followers()).extracting(RevMechanism.Follower::id).containsExactly(42);
  }

  /**
   * An unweighed pivot's tuned kG reaches a SPARK as it does a TalonFX: the robot adds it each
   * loop, along one cosine about where its weight pulls hardest, as given.
   */
  @Test
  void anUnweighedPivotsTunedKgPullsHardestAtItsGravityAngle() {
    Joint.Revolute bare = JointRevoluteBuilder.builder(PIVOT).link(Link.unseen("bare arm")).build();
    Drive neo = JointFixtures.motors(bare, Motor.NEO, 2, JointFixtures.gearing(25));
    RobotMath math = RobotMath.of(ROBOT.body(), List.of(bare), List.of(), List.of(neo), List.of());

    ControllerPlan plan =
        RevMechanism.configure(
                ControllerPlan.of(math, bare, neo, SPARKS, CONTROL.withKg(0.4, Degrees.of(-30))))
            .plan();

    assertThat(plan.gravity().kind()).isEqualTo(ControllerPlan.Gravity.Kind.COSINE);
    assertThat(plan.feedforward(Degrees.of(-30).in(Radians), 0, 0)).isCloseTo(0.4, within(1e-12));
    assertThat(plan.feedforward(Degrees.of(60).in(Radians), 0, 0)).isCloseTo(0, within(1e-12));
  }

  /**
   * An arm whose positive direction lowers it, its weight pulling hardest half a turn from its
   * zero, runs on a SPARK: the robot works out its gravity each loop, with no limit on the offset.
   * On a TalonFX, Phoenix's onboard gravity can't take it, and the robot says how to turn its axis.
   */
  @Test
  void anArmWhosePositiveLowersItRunsOnASpark() {
    Joint.Revolute bare = JointRevoluteBuilder.builder(PIVOT).link(Link.unseen("bare arm")).build();
    Drive neo = JointFixtures.motors(bare, Motor.NEO, 2, JointFixtures.gearing(25));
    RobotMath math = RobotMath.of(ROBOT.body(), List.of(bare), List.of(), List.of(neo), List.of());
    MechanismControl lowering = CONTROL.withKg(0.4, Degrees.of(180));

    ControllerPlan plan =
        RevMechanism.configure(ControllerPlan.of(math, bare, neo, SPARKS, lowering)).plan();

    assertThat(plan.feedforward(0, 0, 0))
        .as("pushing up, against positive")
        .isCloseTo(-0.4, within(1e-12));
    assertThat(plan.feedforward(Math.PI, 0, 0)).isCloseTo(0.4, within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                PhoenixMechanism.configure(
                    ControllerPlan.of(
                        math,
                        bare,
                        JointFixtures.motors(bare, Motor.KRAKEN_X60, 2, JointFixtures.gearing(25)),
                        wiring(MechanismWiring.Controller.TALON_FX),
                        lowering)))
        .withMessageContaining("past Phoenix's");
  }

  /**
   * An absolute encoder on the data port is carried in the settings, for reading at startup, with
   * where the SPARK's reading of it is zero: the low edge of the window its turn tells apart,
   * centered on the pivot's travel, 0° to 135°: -112.5°, so the reading wraps at 247.5°.
   */
  @Test
  void anEncoderOnTheDataPortIsReadAtStartup() {
    Sensor.Encoder absolute = JointFixtures.absoluteEncoder("pivot absolute", PIVOT, 8192);
    RobotMath math =
        RobotDefinitionBuilder.builder(ROBOT).sensors(List.of(absolute)).build().math();
    ControllerPlan plan = ControllerPlan.of(math, PIVOT, NEO, SPARKS, CONTROL);
    RevMechanism.Settings read =
        RevMechanism.configure(math, plan, absolute, DATA_PORT.offset(Rotations.of(0)));

    assertThat(read.absolute()).contains(absolute);
    assertThat(read.windowEdge()).as("in turns").isCloseTo(-0.3125, within(1e-12));
    assertThat(RevMechanism.configure(plan).absolute()).isEmpty();
  }

  /**
   * Closed on its data-port encoder, the SPARK's loop runs in the encoder's turns from the window's
   * edge, -112.5°: on the pivot's axle, a turn of the encoder is a turn of the pivot, so its kP is
   * the slot's per turn of the pivot, 25 times what it is per rotor turn, and its soft limits are
   * where the encoder reads them. Seeded once, the same encoder leaves the loop on the rotor. The
   * feedforward is the plan's either way.
   */
  @Test
  void aSparksLoopClosesOnItsDataPortEncoderInItsTurns() {
    Sensor.Encoder absolute = JointFixtures.absoluteEncoder("pivot absolute", PIVOT, 8192);
    RobotMath math =
        RobotDefinitionBuilder.builder(ROBOT).sensors(List.of(absolute)).build().math();
    MechanismWiring.DataPortEncoder port = DATA_PORT.offset(Rotations.of(0));

    ControllerPlan plan = ControllerPlan.of(math, PIVOT, NEO, SPARKS, CONTROL);
    RevMechanism.Settings once = RevMechanism.configure(math, plan, absolute, port);
    RevMechanism.Settings closed = RevMechanism.configure(math, plan, absolute, port.closesLoop());

    assertThat(once.closedLoop()).isFalse();
    assertThat(once.loopRatio()).isCloseTo(25, within(1e-12));
    assertThat(once.loopZero()).isZero();
    assertThat(closed.closedLoop()).isTrue();
    assertThat(closed.loopRatio()).isCloseTo(1, within(1e-12));
    assertThat(closed.loopZero()).isCloseTo(0.3125, within(1e-12));
    assertThat(closed.positionKp()).isCloseTo(once.positionKp() * 25, within(1e-12));
    assertThat(closed.velocityKp()).isCloseTo(once.velocityKp() * 25, within(1e-12));
    assertThat(closed.softLimits().orElseThrow())
        .containsExactly(0.1 / (2 * Math.PI) + 0.3125, 2.2 / (2 * Math.PI) + 0.3125);
    assertThat(closed.plan()).isSameAs(once.plan());
  }

  /**
   * An elevator on two Vortexes through 10:1 onto a 1 in drum, its through-bore on the leader's
   * data port geared nine to one down from the drum: closed on it, the SPARK's loop runs in the
   * encoder's turns, a ninth of one for each of the drum's rotations, its kP 90 times what it is
   * per rotor turn and its soft limits where the encoder reads them, from the low edge of the
   * window its turn tells apart, centered on the travel: 1.2 m is 0.835 of its turns, so the edge
   * is 0.082 of a turn below the elevator's zero.
   */
  @Test
  void anElevatorsLoopClosesOnItsDataPortEncoderGearedToItsDrum() {
    Joint.Prismatic elevator =
        JointFixtures.unmotored(JointFixtures.ELEVATOR)
            .motors(
                Motor.NEO_VORTEX
                    .through(JointFixtures.gearing(10))
                    .drum(Inches.of(1))
                    .on(
                        MechanismWiring.Controller.SPARK_FLEX.can(41),
                        MechanismWiring.Controller.SPARK_FLEX.can(42)))
            .absoluteEncoder(
                Driven.AbsoluteEncoder.of()
                    .ratio(1.0 / 9)
                    .on(DATA_PORT.offset(Rotations.of(0.4)).closesLoop()))
            .retuned(
                control ->
                    MechanismControlBuilder.builder(control).homing(Optional.empty()).build());
    RobotMath math = JointFixtures.ROBOT.with(elevator).math();
    Sensor.Encoder absolute = Objects.requireNonNull(math.absoluteEncoder(elevator));
    MechanismControl limited =
        MechanismControlBuilder.builder(ControllerSettings.control(elevator))
            .softLimits(MechanismControl.SoftLimits.builder().lower(0.05).upper(1.1).build())
            .build();
    ControllerPlan plan = ControllerPlan.of(math, elevator, limited);
    MechanismWiring.DataPortEncoder port =
        elevator.driven().absoluteEncoder().orElseThrow().dataPort().orElseThrow();
    double drum = 2 * Math.PI * 0.0254;
    double edge = (1.2 / drum / 9) / 2 - 0.5;

    RevMechanism.Settings closed = RevMechanism.configure(math, plan, absolute, port);
    RevMechanism.Settings once =
        RevMechanism.configure(math, plan, absolute, DATA_PORT.offset(Rotations.of(0.4)));

    assertThat(closed.closedLoop()).isTrue();
    assertThat(closed.windowEdge()).as("in the encoder's turns").isCloseTo(edge, within(1e-12));
    assertThat(closed.loopRatio()).isCloseTo(1.0 / 9, within(1e-12));
    assertThat(closed.loopZero()).isCloseTo(-edge, within(1e-12));
    assertThat(once.loopRatio())
        .as("its rotor's, through the gearing")
        .isCloseTo(10, within(1e-12));
    assertThat(closed.positionKp()).isCloseTo(once.positionKp() * 90, within(1e-9));
    assertThat(closed.softLimits().orElseThrow()[0])
        .isCloseTo(0.05 / drum / 9 - edge, within(1e-12));
    assertThat(closed.softLimits().orElseThrow()[1])
        .isCloseTo(1.1 / drum / 9 - edge, within(1e-12));
    assertThat(port.raw(closed.windowEdge()))
        .as("the SPARK's zero offset: the raw reading at the edge")
        .isCloseTo(edge - 0.4 + 1, within(1e-12));
  }

  /**
   * Its kI goes to the loop for positions as REV documents it: a share of 12 V per rotor turn of
   * error per millisecond, the loop's period, so a TalonFX slot's kI per second over a thousand.
   */
  @Test
  void itsIntegralIsPerMillisecondInASparksUnits() {
    MechanismControl integral =
        MechanismControlBuilder.builder(CONTROL)
            .gains(MechanismControlGainsBuilder.builder(CONTROL.gains()).kI(1).build())
            .build();
    ControllerPlan plan = ControllerPlan.of(MATH, PIVOT, NEO, SPARKS, integral);
    SlotConfigs slot = SlotConfigs.from(PhoenixMechanism.configure(plan).leader().Slot0);

    assertThat(RevMechanism.configure(plan).positionKi())
        .isCloseTo(slot.kI / (12 * 25 * 1000), within(1e-15));
    assertThat(
            RevMechanism.configure(ControllerPlan.of(MATH, PIVOT, NEO, SPARKS, CONTROL))
                .positionKi())
        .isZero();
  }

  @Test
  void itRefusesWhatASparkCantTake() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RevMechanism.configure(
                    ControllerPlan.of(
                        MATH, PIVOT, NEO, wiring(MechanismWiring.Controller.TALON_FXS), CONTROL)))
        .withMessage("pivot's wiring is a TalonFXS's, not a SPARK's");
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.rev.RevMechanism;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkSimFaultManager;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfigAccessor;
import com.revrobotics.util.Signal;
import java.util.Locale;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SimDeviceSim;

/**
 * The REV joint against REV's own simulated SPARK: what it tells the SPARK, in the SPARK's units,
 * and what it reads back, in the joint's. The settings are the REV helper's, from the program's
 * plan, tested there.
 */
@ExtendWith(RobotSim.class)
class JointIOSparkTest {
  /** The pivot on a NEO through 25:1, its SPARK MAX at an ID. */
  private static Joint.Revolute pivot(int id) {
    return unmotored(PIVOT).motors(Motor.NEO.through(gearing(25)).on(SPARK_MAX.can(id)));
  }

  /** A joint's IO, on the fixtures' robot with the joint as given. */
  private static JointIOSpark io(Joint joint, ControllerPins pins) {
    return new JointIOSpark(ROBOT.with(joint).math(), joint, pins);
  }

  private static JointIOSpark io(Joint joint) {
    return io(joint, ControllerPins.NONE);
  }

  /**
   * A pivot read by its rotor starts where its definition rests it, and reads its rotor's turns
   * through its 25:1: twenty-five more are one more turn of the pivot.
   */
  @Test
  void itReadsItsRotorThroughItsGearingFromWhereItRests() {
    JointIOSpark io = io(pivot(31));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double resting = inputs.position;
    // The simulation's own position starts at 0, not where the startup set the encoder.
    sim.setPosition(PIVOT.restPosition() / (2 * Math.PI) * 25 + 25);
    sim.iterate(0, 12, 0.02);
    io.updateInputs(inputs);

    assertThat(resting)
        .as("it starts at its reference, 1.5°")
        .isCloseTo(PIVOT.restPosition(), within(1e-9));
    // A SPARK reports its position as a float.
    assertThat(inputs.position).isCloseTo(PIVOT.restPosition() + 2 * Math.PI, within(1e-6));
    assertThat(inputs.unconfigured).as("every SPARK took its settings").isEmpty();
  }

  /**
   * A frame of its encoder that doesn't come, REV's signal invalid, reads REV's default, 0: the
   * pivot's last position and speed stand instead, not 0, and its reading isn't fresh that loop, so
   * the interlocks reading it hold at once. Its SPARK reads as silent only after half a second of
   * them; a frame that comes is fresh again at once.
   */
  @Test
  void aFrameThatDoesntComeLeavesTheLastReadingNotFresh() {
    JointIOSpark io = io(pivot(48));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    // Turning at 250 rpm, a turn and some on from where it rested.
    sim.setPosition(PIVOT.restPosition() / (2 * Math.PI) * 25 + 25);
    sim.iterate(250, 12, 0.02);
    io.updateInputs(inputs);
    double position = inputs.position;
    double velocity = inputs.velocity;
    Signal<Double> lost = Signal.ofError(0.0, REVLibError.kTimeout, 0);

    io.updateInputs(inputs, lost, lost);
    boolean fresh = inputs.fresh;
    boolean connected = inputs.connected;
    double kept = inputs.position;
    double keptSpeed = inputs.velocity;
    for (int i = 0; i < 26; i++) {
      RobotSim.stepClock();
      io.updateInputs(inputs, lost, lost);
    }
    boolean silent = !inputs.connected;
    io.updateInputs(inputs);

    assertThat(position).isGreaterThan(PIVOT.restPosition() + 2 * Math.PI);
    assertThat(velocity).isPositive();
    assertThat(fresh).isFalse();
    assertThat(connected).as("not silent yet").isTrue();
    assertThat(kept).as("not 0").isEqualTo(position);
    assertThat(keptSpeed).isEqualTo(velocity);
    assertThat(silent).as("after half a second").isTrue();
    assertThat(inputs.fresh).isTrue();
    assertThat(inputs.position).isCloseTo(position, within(1e-6));
  }

  /**
   * The SPARK is told each limit input's polarity, and stops on a limit switch only: a beam read
   * through its forward input lets the motor run on. REV applies the polarity on the device (its
   * docs: {@code isPressed} is "based on the selected polarity"), and its simulator's "pressed" is
   * after it, so what the SPARK took is read back from it, and a press reads tripped.
   *
   * <p>REVLib 2027.0.0-alpha-8 reads a limit input's type back the other way round: set normally
   * open, it reads normally closed, and a fresh SPARK reads normally closed where REV's docs say
   * its default is normally open, which points at the reading, not the setting. Pinned here, so a
   * REVLib that changes it fails this test; which side is right waits on a SPARK on the bench (open
   * question M22).
   */
  @Test
  void itsLimitInputsTakeTheirPolarityAndStopOnlyOnALimitSwitch() {
    ControllerPins pins =
        ControllerPins.NONE
            .with(
                SensorWiring.Direction.FORWARD,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false))
            .with(
                SensorWiring.Direction.REVERSE,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true));
    JointIOSpark io = io(pivot(47), pins);
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    LimitSwitchConfigAccessor took = ((SparkMax) io.leader()).configAccessor.limitSwitch;

    sim.getForwardLimitSwitchSim().setPressed(true);
    io.updateInputs(inputs);

    assertThat(took.getForwardSwitchType())
        .as("set normally open; alpha-8 reads it back the other way")
        .isEqualTo(LimitSwitchConfig.Type.kNormallyClosed);
    assertThat(took.getReverseSwitchType())
        .as("set normally closed; alpha-8 reads it back the other way")
        .isEqualTo(LimitSwitchConfig.Type.kNormallyOpen);
    assertThat(took.getForwardLimitSwitchTriggerBehavior())
        .as("a beam: read, never stopped on")
        .isEqualTo(LimitSwitchConfig.Behavior.kKeepMovingMotor);
    assertThat(took.getReverseLimitSwitchTriggerBehavior())
        .as("a limit switch: the SPARK stops on it")
        .isEqualTo(LimitSwitchConfig.Behavior.kStopMovingMotor);
    assertThat(inputs.forwardLimit).isTrue();
    assertThat(inputs.reverseLimit).isFalse();
  }

  /**
   * A position goes to the SPARK's loop in rotor turns, with the feedforward the robot works out: a
   * pivot 45° up is 25 × 1/8 rotor turns. A little past where it rests, its loop's output is its kP
   * on the error plus the feedforward's share of 12 V (far off, the current limit caps it).
   */
  @Test
  void aPositionGoesInRotorTurnsWithItsFeedforwardInVolts() {
    Joint.Revolute pivot = pivot(32);
    RobotMath math = ROBOT.with(pivot).math();
    JointIOSpark io = io(pivot);
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    sim.enable();
    RevMechanism.Settings settings =
        RevMechanism.configure(
            ControllerPlan.of(
                math,
                PIVOT,
                math.drivesOf(PIVOT).get(0),
                pivot.driven().motors().orElseThrow().controllers(),
                ControllerSettings.control(pivot)));

    // REV's simulation steps its own position into the encoder: start it where the pivot rests.
    sim.setPosition(PIVOT.restPosition() / (2 * Math.PI) * 25);
    io.setPosition(Math.toRadians(45), 0, 0.6);
    sim.iterate(0, 12, 0.02);
    double far = sim.getSetpoint();
    double near = PIVOT.restPosition() + Math.toRadians(0.5);
    io.setPosition(near, 0, 0.6);
    for (int i = 0; i < 25; i++) {
      sim.iterate(0, 12, 0.02);
    }

    double rest = PIVOT.restPosition() / (2 * Math.PI) * 25;
    double target = near / (2 * Math.PI) * 25;
    double feedforward = 0.6 + settings.plan().feedforward(near, 0, 0);
    // A SPARK holds its setpoint as a float.
    assertThat(far).isCloseTo(25.0 / 8, within(1e-6));
    assertThat(sim.getSetpoint()).isCloseTo(target, within(1e-6));
    assertThat(sim.getAppliedOutput())
        .isCloseTo(settings.positionKp() * (target - rest) + feedforward / 12, within(1e-3));
  }

  /** The pit's tuning reaches the SPARK's loop, as a TalonFX's slot: its kP, in rotor turns. */
  @Test
  void aTunedGainReachesItsLoop() {
    Joint.Revolute pivot = pivot(33);
    JointIOSpark io = io(pivot);
    MechanismControl control = ControllerSettings.control(pivot);
    MechanismControl tuned =
        MechanismControlBuilder.builder(control)
            .gains(MechanismControlGainsBuilder.builder(control.gains()).kP(60).build())
            .build();

    io.retune(tuned);

    // 60 V a radian is 60 × 2π a turn of the pivot, 25 rotor turns, as a share of 12 V.
    double expected = 60 * 2 * Math.PI / (12 * 25);
    assertThat(((SparkMax) io.leader()).configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .isCloseTo(expected, within(1e-6 * expected));
  }

  /** A control with its gains and its stator limit changed, as the pit tunes one. */
  private static MechanismControl tuned(
      MechanismControl control, double kP, double kI, double statorAmps) {
    return MechanismControlBuilder.builder(control)
        .gains(MechanismControlGainsBuilder.builder(control.gains()).kP(kP).kI(kI).build())
        .build()
        .withStatorLimit(Amps.of(statorAmps));
  }

  /**
   * Everything a retune promises that a SPARK's own loop runs reaches the SPARK, read back from
   * what it took: its kP for positions and for speeds, its kI, and its current limit. A kI tuned
   * back to nothing is sent too, or the SPARK would go on integrating with the last one. The rest
   * of a retune, its kD and its feedforward, never goes to the SPARK: the robot works them out each
   * loop from the tuned plan ({@link #aTunedDampingAndFrictionGoWithTheNextRequest}).
   */
  @Test
  void aRetuneReachesEverythingTheSparksLoopRuns() {
    Joint.Revolute pivot = pivot(46);
    JointIOSpark io = io(pivot);
    SparkMax spark = (SparkMax) io.leader();
    MechanismControl control = ControllerSettings.control(pivot);
    double kIAtStartup = spark.configAccessor.closedLoop.getI(ClosedLoopSlot.kSlot0);

    io.retune(tuned(control, 60, 4, 25));
    double kP = spark.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0);
    double speedKp = spark.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot1);
    double kI = spark.configAccessor.closedLoop.getI(ClosedLoopSlot.kSlot0);
    int statorAmps = spark.configAccessor.getSmartCurrentLimit();
    io.retune(tuned(control, 60, 0, 25));
    double kITunedAway = spark.configAccessor.closedLoop.getI(ClosedLoopSlot.kSlot0);

    // A gain per radian is 2π times that per turn of the pivot, 25 rotor turns, as a share of
    // 12 V. A speed's is per rotor RPM, and the integral's per millisecond.
    double perRotorTurn = 2 * Math.PI / (12 * 25);
    assertThat(kIAtStartup).as("the fixture's pivot starts with no kI").isZero();
    assertThat(kP).isCloseTo(60 * perRotorTurn, within(1e-6));
    assertThat(speedKp).isCloseTo(60 * perRotorTurn / 60, within(1e-8));
    assertThat(kI).isCloseTo(4 * perRotorTurn / 1000, within(1e-9));
    assertThat(statorAmps).isEqualTo(25);
    assertThat(kITunedAway).as("a kI tuned back to nothing").isZero();
  }

  /**
   * What a retune changes that the SPARK's loop does not run, the robot sends with its next
   * request, in volts: a tuned kD on how far the speed last read is from the profile's, and a tuned
   * kS with the rest of the plan's feedforward.
   */
  @Test
  void aTunedDampingAndFrictionGoWithTheNextRequest() {
    Joint.Revolute pivot = pivot(29);
    RobotMath math = ROBOT.with(pivot).math();
    JointIOSpark io = io(pivot);
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNEO(1));
    sim.enable();
    MechanismControl control = ControllerSettings.control(pivot);
    MechanismControl tuned =
        MechanismControlBuilder.builder(control)
            .gains(MechanismControlGainsBuilder.builder(control.gains()).kD(2).kS(0.5).build())
            .build();
    ControllerPlan plan = ControllerSettings.spark(math, pivot, tuned).plan();
    ControllerPlan untuned = ControllerSettings.spark(math, pivot, control).plan();
    double rest = PIVOT.restPosition();
    // The loop has no error, so its output is the robot's volts alone.
    sim.setPosition(rest / (2 * Math.PI) * 25);

    io.retune(tuned);
    // Asked to pass where it rests at 0.2 rad/s while it reads still: kD works on the 0.2.
    io.setPosition(rest, 0.2, 0);
    for (int i = 0; i < 25; i++) {
      sim.iterate(0, 12, 0.02);
    }

    double volts = plan.feedforward(rest, 0.2, 0) + 2 * 0.2;
    assertThat(plan.feedforward(rest, 0.2, 0))
        .as("the tuned kS is in the plan's feedforward")
        .isCloseTo(untuned.feedforward(rest, 0.2, 0) + 0.5, within(1e-9));
    assertThat(sim.getAppliedOutput()).isCloseTo(volts / 12, within(1e-3));
  }

  /**
   * The slide's two Vortexes on SPARK Flexes: the first leads, and reads meters through its drum.
   */
  @Test
  void aSlideOnTwoSparkFlexesReadsMetersThroughItsDrum() {
    JointIOSpark io =
        io(
            unmotored(ELEVATOR)
                .motors(
                    Motor.NEO_VORTEX
                        .through(gearing(10))
                        .drum(Inches.of(1))
                        .on(SPARK_FLEX.can(34), SPARK_FLEX.can(35))));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNeoVortex(1));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double start = inputs.position;
    // Ten rotor turns through 10:1 are one turn of the 1 in drum: 2π inches of travel.
    sim.setPosition(sim.getPosition() + 10);
    sim.iterate(0, 12, 0.02);
    io.updateInputs(inputs);

    assertThat(inputs.position - start).isCloseTo(2 * Math.PI * 0.0254, within(1e-9));
    assertThat(inputs.unconfigured).isEmpty();
  }

  /**
   * Volts and a share of the battery go to the SPARK as they are, as REV's simulated SPARK holds
   * them; neutral is a share of none, which brakes or coasts as its settings say.
   */
  @Test
  void voltsAShareAndNeutralGoAsTheyAre() {
    JointIOSpark io = io(pivot(49));
    SparkBase spark = io.leader();
    SimDeviceSim device =
        new SimDeviceSim(
            String.format(
                Locale.ROOT, "SPARK MAX [%d,%d]", spark.getCanPort().value, spark.getDeviceId()));

    io.setVoltage(4);
    int voltsMode = device.getInt("Control Mode").get();
    double volts = device.getDouble("Setpoint").get();
    io.setDutyCycle(-0.5);
    int shareMode = device.getInt("Control Mode").get();
    double share = device.getDouble("Setpoint").get();
    io.stop();

    assertThat(voltsMode).isEqualTo(SparkLowLevel.ControlType.kVoltage.value);
    assertThat(volts).isEqualTo(4);
    assertThat(shareMode).isEqualTo(SparkLowLevel.ControlType.kDutyCycle.value);
    assertThat(share).isEqualTo(-0.5);
    assertThat(device.getInt("Control Mode").get())
        .isEqualTo(SparkLowLevel.ControlType.kDutyCycle.value);
    assertThat(device.getDouble("Setpoint").get()).isZero();
  }

  /** A speed goes to the SPARK's speed slot in rotor RPM: the roller's 1 rad/s through 3:1. */
  @Test
  void aSpeedGoesInRotorRpm() {
    JointIOSpark io =
        io(unmotored(ROLLER).motors(Motor.NEO_550.through(gearing(3)).on(SPARK_MAX.can(36))));
    SparkSim sim = new SparkSim(io.leader(), DCMotor.getNeo550(1));
    sim.enable();

    io.setVelocity(1);
    sim.iterate(0, 12, 0.02);

    assertThat(sim.getSetpoint()).isCloseTo(1 / (2 * Math.PI) * 3 * 60, within(1e-4));
    assertThat(sim.getClosedLoopSlot()).isEqualTo(ClosedLoopSlot.kSlot1);
  }

  /**
   * Homing pushes past the soft limits, which a SPARK keeps itself: they're switched off while it
   * homes, and back on once it's found where it is.
   */
  @Test
  void homingSwitchesItsSoftLimitsOffAndBackOn() {
    JointIOSpark io =
        io(
            pivot(37)
                .retuned(
                    control ->
                        control.withSoftLimits(Range.of(Radians.of(0.01), Radians.of(2.3)))));
    SparkMax spark = (SparkMax) io.leader();
    boolean before = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();

    io.home(-2);
    boolean homing = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();
    io.resetPosition(0);
    boolean after = spark.configAccessor.softLimit.getForwardSoftLimitEnabled();

    assertThat(before).isTrue();
    assertThat(homing).isFalse();
    assertThat(after).isTrue();
  }

  /**
   * A pivot with REV's Through Bore on its SPARK's data port, mounted reversed: the SPARK takes the
   * encoder's direction and its offset, and reads zero at the low edge of the window one turn of it
   * tells apart, centered on the pivot's travel (0° to 135°): -112.5°, so its reading never wraps
   * between the stops. Powered on at 60°, not where it rests, the pivot is told so by the encoder's
   * first good reading, and its motor's encoder carries it on. REV's simulation reports the reading
   * it's given, applying neither the offset nor the direction, so the test gives it the SPARK's
   * reading.
   */
  @Test
  void aPivotWithAnEncoderOnItsDataPortIsToldWhereItIs() {
    JointIOSpark io = io(pivot(38).absoluteEncoder(REVERSED));
    SparkMax spark = (SparkMax) io.leader();
    // At 60°, a sixth of a turn past the pivot's zero and so 0.3125 + 1/6 of one past the edge.
    new SparkAbsoluteEncoderSim(spark).setPosition(0.3125 + 60.0 / 360);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    io.updateInputs(inputs);
    double told = inputs.position;
    io.updateInputs(inputs);
    double carried = inputs.position;
    // The joint moves on without its motor, as across backlash or a slipping belt.
    new SparkAbsoluteEncoderSim(spark).setPosition(0.3125 + 70.0 / 360);
    io.updateInputs(inputs);

    assertThat(spark.configAccessor.absoluteEncoder.getInverted()).isTrue();
    // The raw reading at the edge, -0.3125 turns, reversed and offset 0.1: 0.4125.
    assertThat(spark.configAccessor.absoluteEncoder.getZeroOffset())
        .isCloseTo(0.4125, within(1e-6));
    assertThat(Math.toDegrees(told)).isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(carried)).as("then from its rotor").isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(inputs.position)).as("its rotor's").isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(inputs.absolutePosition))
        .as("its encoder's, beside it")
        .isCloseTo(70, within(1e-3));
  }

  /**
   * A pivot whose SPARK's loop closes on its data-port encoder is read by the encoder each loop,
   * wherever its motor's count is, and is moved in the encoder's turns from the window's edge: 45°
   * is 0.3125 + 1/8 of a turn, and a turn a second 60 of the encoder's RPM. Its loop runs on the
   * encoder, without position wrapping, its kP per turn of the encoder. REV's simulation runs its
   * loop on the motor's own encoder whatever it's told, so what the SPARK does with these is read
   * from what it took, not seen.
   */
  @Test
  void aPivotClosedOnItsDataPortEncoderIsReadByItAndMovedInItsTurns() {
    // Its loop on the encoder, which tells it where it is: no homing recipe.
    MechanismControl unhomed =
        MechanismControlBuilder.builder(ControllerSettings.control(PIVOT))
            .homing(Optional.empty())
            .build();
    JointIOSpark io = io(pivot(39).absoluteEncoder(REVERSED.closesLoop()).retuned(old -> unhomed));
    SparkMax spark = (SparkMax) io.leader();
    SparkSim sim = new SparkSim(spark, DCMotor.getNEO(1));
    sim.enable();
    SparkAbsoluteEncoderSim encoder = new SparkAbsoluteEncoderSim(spark);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    encoder.setPosition(0.3125 + 60.0 / 360);
    io.updateInputs(inputs);
    io.updateInputs(inputs);
    double seeded = inputs.position;
    // The motor's count stays where the encoder first said; the joint moves on, or slipped.
    encoder.setPosition(0.3125 + 70.0 / 360);
    io.updateInputs(inputs);
    double moved = inputs.position;
    io.setPosition(Math.toRadians(45), 0, 0);
    sim.iterate(0, 12, 0.02);
    double toward = sim.getSetpoint();
    io.setVelocity(2 * Math.PI);
    sim.iterate(0, 12, 0.02);

    assertThat(spark.configAccessor.closedLoop.getFeedbackSensor())
        .isEqualTo(FeedbackSensor.kAbsoluteEncoder);
    assertThat(spark.configAccessor.closedLoop.getPositionWrappingEnabled()).isFalse();
    // 30 V a radian is 30 × 2π a turn of the pivot, one of the encoder's, as a share of 12 V.
    double kP = 30 * 2 * Math.PI / 12;
    assertThat(spark.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .isCloseTo(kP, within(1e-6 * kP));
    assertThat(Math.toDegrees(seeded)).isCloseTo(60, within(1e-3));
    assertThat(Math.toDegrees(moved)).as("from its encoder").isCloseTo(70, within(1e-3));
    assertThat(toward).isCloseTo(0.3125 + 45.0 / 360, within(1e-6));
    assertThat(sim.getSetpoint()).isCloseTo(60, within(1e-4));
  }

  /**
   * An elevator on two Vortexes, its through-bore on the leader's data port geared nine to one down
   * from its 1 in drum, its loop closed on it: 1.2 m is 0.835 of the encoder's turns, so the SPARK
   * reads zero 0.082 of a turn below the elevator's zero, the edge of the window centered on its
   * travel. It's read in meters from the encoder, wherever its motor's count is, and moved in the
   * encoder's turns: 0.3 m is 0.209 of a turn past its zero. Its kP is 60 V a meter, per turn of
   * the encoder as a share of 12 V.
   */
  @Test
  void anElevatorClosedOnItsDataPortEncoderIsReadInMetersThroughItsDrum() {
    MechanismControl unhomed =
        MechanismControlBuilder.builder(ControllerSettings.control(ELEVATOR))
            .homing(Optional.empty())
            .build();
    JointIOSpark io =
        io(
            unmotored(ELEVATOR)
                .motors(
                    Motor.NEO_VORTEX
                        .through(gearing(10))
                        .drum(Inches.of(1))
                        .on(SPARK_FLEX.can(43), SPARK_FLEX.can(44)))
                .absoluteEncoder(
                    Driven.AbsoluteEncoder.of()
                        .ratio(1.0 / 9)
                        .on(DATA_PORT.offset(Rotations.of(0.1)).closesLoop()))
                .retuned(old -> unhomed));
    SparkFlex spark = (SparkFlex) io.leader();
    SparkSim sim = new SparkSim(spark, DCMotor.getNeoVortex(1));
    sim.enable();
    SparkAbsoluteEncoderSim encoder = new SparkAbsoluteEncoderSim(spark);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    double perMeter = 1.0 / 9 / (2 * Math.PI * 0.0254);
    double edge = 1.2 * perMeter / 2 - 0.5;

    encoder.setPosition(0.6 * perMeter - edge);
    io.updateInputs(inputs);
    io.updateInputs(inputs);
    double seeded = inputs.position;
    encoder.setPosition(0.7 * perMeter - edge);
    io.updateInputs(inputs);
    double moved = inputs.position;
    io.setPosition(0.3, 0, 0);
    sim.iterate(0, 12, 0.02);

    assertThat(spark.configAccessor.closedLoop.getFeedbackSensor())
        .isEqualTo(FeedbackSensor.kAbsoluteEncoder);
    // The raw reading at the edge, offset 0.1.
    assertThat(spark.configAccessor.absoluteEncoder.getZeroOffset())
        .isCloseTo(edge - 0.1 + 1, within(1e-6));
    double kP = 60 / perMeter / 12;
    assertThat(spark.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .isCloseTo(kP, within(1e-6 * kP));
    assertThat(seeded).isCloseTo(0.6, within(1e-5));
    assertThat(moved).as("from its encoder").isCloseTo(0.7, within(1e-5));
    assertThat(sim.getSetpoint()).isCloseTo(0.3 * perMeter - edge, within(1e-6));
  }

  /** The pivot's through-bore on its SPARK's data port, reversed and offset 0.1, seeded once. */
  private static final MechanismWiring.DataPortEncoder REVERSED =
      DATA_PORT.offset(Rotations.of(0.1)).inverted();

  /**
   * A SPARK that restarts says so in its sticky warnings: believed once the warning has read clear
   * since the robot cleared it as it started, said once, and its tuning told it again, which a
   * restart loses. REV's simulation keeps what it was told through a restart, so the test puts the
   * SPARK back on its saved tuning, as a restart would.
   */
  @Test
  void aRestartedSparkSaysSoAndIsToldItsTuningAgain() {
    Joint.Revolute pivot = pivot(40);
    JointIOSpark io = io(pivot);
    SparkMax spark = (SparkMax) io.leader();
    MechanismControl control = ControllerSettings.control(pivot);
    MechanismControl tuned =
        MechanismControlBuilder.builder(control)
            .gains(MechanismControlGainsBuilder.builder(control.gains()).kP(60).build())
            .build();
    io.retune(tuned);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    io.updateInputs(inputs);
    boolean before = inputs.restarted;

    spark.configure(
        ControllerSettings.spark(ROBOT.with(pivot).math(), pivot, control)
            .tuning(SPARK_MAX, control),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    double fellBack = spark.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0);
    new SparkSimFaultManager(spark)
        .setStickyWarnings(
            new SparkBase.Warnings(false, false, false, false, false, false, true, false));
    io.updateInputs(inputs);
    boolean restarted = inputs.restarted;
    io.updateInputs(inputs);

    double expected = 60 * 2 * Math.PI / (12 * 25);
    assertThat(before).isFalse();
    assertThat(fellBack).isNotCloseTo(expected, within(1e-6 * expected));
    assertThat(restarted).isTrue();
    assertThat(inputs.restarted).as("once").isFalse();
    assertThat(spark.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0))
        .as("its tuning, again")
        .isCloseTo(expected, within(1e-6 * expected));
  }

  /**
   * Its SPARKs' health, a few times a second: each one's faults, its warnings above them, and its
   * follower answering. A gate-driver fault and a brownout warning on the leader read as their
   * bits.
   */
  @Test
  void itsSparksHealthIsReadAFewTimesASecond() {
    JointIOSpark io =
        io(
            unmotored(PIVOT)
                .motors(Motor.NEO.through(gearing(25)).on(SPARK_MAX.can(52), SPARK_MAX.can(53))));
    SparkMax spark = (SparkMax) io.leader();
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    io.updateInputs(inputs);
    int[] before = inputs.faults.clone();

    SparkSimFaultManager faults = new SparkSimFaultManager(spark);
    faults.setFaults(new SparkBase.Faults(false, false, false, false, false, true, false, false));
    faults.setWarnings(
        new SparkBase.Warnings(true, false, false, false, false, false, false, false));
    for (int i = 0; i < JointIO.HEALTH_LOOPS; i++) {
      io.updateInputs(inputs);
    }
    SparkBase.Faults gateDriver =
        new SparkBase.Faults(false, false, false, false, false, true, false, false);
    SparkBase.Warnings brownout =
        new SparkBase.Warnings(true, false, false, false, false, false, false, false);

    assertThat(before).containsExactly(0, 0);
    assertThat(inputs.faults[0]).isEqualTo(brownout.rawBits << 16 | gateDriver.rawBits);
    assertThat(inputs.faults[1]).isZero();
    assertThat(inputs.followersConnected).containsExactly(true);
  }
}

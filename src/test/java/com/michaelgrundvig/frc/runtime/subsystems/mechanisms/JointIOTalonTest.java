package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_MOTOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FXS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * The CTRE joint against Phoenix's own simulated devices: what the robot reads from them, in the
 * joint's units. The settings it applies are the Phoenix helper's, tested there.
 */
@ExtendWith(RobotSim.class)
class JointIOTalonTest {
  private static final String BUS = "joints";

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  private static final CanBus CANIVORE = CanBus.canivore(BUS);

  /** The pivot, its motor on a TalonFX at an ID on the test's CANivore. */
  private static Joint.Revolute pivot(int id) {
    return PIVOT.motorsOn(TALON_FX.can(CANIVORE, id));
  }

  /** A joint's IO, on the fixtures' robot with the joint as given. */
  private static JointIOTalon io(Joint joint, ControllerPins pins) {
    return new JointIOTalon(ROBOT.with(joint).math(), joint, pins);
  }

  private static JointIOTalon io(Joint joint) {
    return io(joint, ControllerPins.NONE);
  }

  /** Waits up to two seconds, reading again every few milliseconds, for something to be true. */
  private static boolean eventually(Runnable read, BooleanSupplier done)
      throws InterruptedException {
    for (int i = 0; i < 200; i++) {
      read.run();
      if (done.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10);
    }
    return false;
  }

  /**
   * What the robot asks of a TalonFX, as it was last told: a profile's point as a PositionVoltage,
   * in turns of the pivot (its gearing is in its own settings), with the profile's speed and the
   * volts the robot adds; a speed as a VelocityVoltage; volts as a VoltageOut and a share of the
   * battery as a DutyCycleOut, each within its soft limits; neutral; and homing's push as a
   * VoltageOut that ignores its soft limits, which until it's homed are in the wrong place. A
   * Kraken X60 without FOC runs none with it.
   */
  @Test
  void itsRequestsAreWhatItsJointAsks() {
    JointIOTalon io = io(pivot(59));
    ParentDevice talon = io.leader();

    io.setPosition(Math.toRadians(45), 1.0, 0.6);
    PositionVoltage position = (PositionVoltage) talon.getAppliedControl();
    io.setVelocity(3.0);
    VelocityVoltage velocity = (VelocityVoltage) talon.getAppliedControl();
    io.setVoltage(4);
    VoltageOut volts = (VoltageOut) talon.getAppliedControl();
    io.setDutyCycle(-0.5);
    DutyCycleOut share = (DutyCycleOut) talon.getAppliedControl();
    io.stop();
    ControlRequest neutral = talon.getAppliedControl();
    io.home(-2);
    VoltageOut homing = (VoltageOut) talon.getAppliedControl();

    assertThat(position.Position).as("an eighth of a turn").isCloseTo(0.125, within(1e-12));
    assertThat(position.Velocity).isCloseTo(1 / (2 * Math.PI), within(1e-12));
    assertThat(position.FeedForward).isEqualTo(0.6);
    assertThat(position.EnableFOC).isFalse();
    assertThat(position.IgnoreSoftwareLimits).isFalse();
    assertThat(velocity.Velocity).isCloseTo(3 / (2 * Math.PI), within(1e-12));
    assertThat(volts.Output).isEqualTo(4);
    assertThat(volts.IgnoreSoftwareLimits).isFalse();
    assertThat(share.Output).isEqualTo(-0.5);
    assertThat(share.IgnoreSoftwareLimits).isFalse();
    assertThat(neutral).isInstanceOf(NeutralOut.class);
    assertThat(homing).as("not the VoltageOut volts go as").isNotSameAs(volts);
    assertThat(homing.Output).isEqualTo(-2);
    assertThat(homing.IgnoreSoftwareLimits).isTrue();
  }

  /** Told where the pivot is, as homing tells it, it reads there from then on, at 100°. */
  @Test
  void toldWhereItIsItReadsThere() throws InterruptedException {
    JointIOTalon io = io(pivot(50));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    eventually(() -> io.updateInputs(inputs), () -> inputs.connected);

    io.resetPosition(Math.toRadians(100));
    boolean there =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - Math.toRadians(100)) < 0.01);

    assertThat(there).as("100°, read %s°", Math.toDegrees(inputs.position)).isTrue();
  }

  /**
   * Its leader's limit pins read tripped by their switches' polarity: a normally open switch as it
   * closes to ground, a normally closed one as it opens. Phoenix reports the pin as it is, closed
   * or open, whatever polarity it's told for its own stop, so the polarity is applied as it's read.
   */
  @Test
  void itsLimitPinsReadTrippedByTheirPolarity() throws InterruptedException {
    ControllerPins pins =
        ControllerPins.NONE
            .with(
                SensorWiring.Direction.FORWARD,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false))
            .with(
                SensorWiring.Direction.REVERSE,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true));
    JointIOTalon io = io(pivot(55), pins);
    TalonFX device = new TalonFX(55, new CANBus(BUS));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    device.getSimState().setForwardLimit(true);
    device.getSimState().setReverseLimit(true);
    boolean closed =
        eventually(
            () -> io.updateInputs(inputs), () -> inputs.forwardLimit && !inputs.reverseLimit);
    device.getSimState().setForwardLimit(false);
    device.getSimState().setReverseLimit(false);
    boolean open =
        eventually(
            () -> io.updateInputs(inputs), () -> !inputs.forwardLimit && inputs.reverseLimit);

    assertThat(closed).as("both closed: forward, normally open, tripped; reverse clear").isTrue();
    assertThat(open).as("both open: forward clear; reverse, normally closed, tripped").isTrue();
  }

  /**
   * A pivot read by its rotor alone starts where its definition rests it, and reads its rotor's
   * turns through its 25:1: twenty-five more are one more turn of the pivot. Within 0.01 rad, since
   * Phoenix's simulated position reads a turn 0.0016 rad short; a wrong ratio or a missing
   * reference (1.5°, 0.026 rad) is far outside it.
   */
  @Test
  void itReadsItsRotorThroughItsGearingFromWhereItRests() throws InterruptedException {
    JointIOTalon io = io(pivot(51));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    boolean resting =
        eventually(
            () -> io.updateInputs(inputs),
            () -> inputs.connected && Math.abs(inputs.position - PIVOT.restPosition()) < 0.01);
    // Its raw rotor started at 0, where the startup told it the pivot rests.
    new TalonFX(51, new CANBus(BUS)).getSimState().setRawRotorPosition(25);
    boolean turned =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - (PIVOT.restPosition() + 2 * Math.PI)) < 0.01);

    assertThat(resting).as("it starts at its reference, 1.5°").isTrue();
    assertThat(turned).as("25 rotor turns are one of the pivot").isTrue();
    assertThat(inputs.unconfigured).as("every device took its configuration").isEmpty();
  }

  /** The elevator's two motors: the first leads, and it reads meters through its drum. */
  @Test
  void aSlideReadsMetersThroughItsDrum() throws InterruptedException {
    JointIOTalon io = io(ELEVATOR.motorsOn(TALON_FX.can(CANIVORE, 53), TALON_FX.can(CANIVORE, 54)));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    eventually(() -> io.updateInputs(inputs), () -> inputs.connected);

    // Ten rotor turns through 10:1 are one turn of the 1 in drum: 2π inches of travel.
    new TalonFX(53, new CANBus(BUS)).getSimState().setRawRotorPosition(10);
    double travel = 2 * Math.PI * 0.0254;
    boolean moved =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - (ELEVATOR.restPosition() + travel)) < 1e-4);

    assertThat(moved).isTrue();
    assertThat(inputs.position).isCloseTo(travel, within(1e-4));
  }

  /** A tuned kP reaches the motor controller's slot, in Phoenix's units, without a restart. */
  @Test
  void aTunedGainReachesItsSlot() throws InterruptedException {
    Joint.Revolute pivot = pivot(52);
    JointIOTalon io = io(pivot);
    MechanismControl control = ControllerSettings.control(pivot);
    MechanismControl tuned =
        MechanismControlBuilder.builder(control)
            .gains(MechanismControlGainsBuilder.builder(control.gains()).kP(55).build())
            .build();
    double expected =
        PhoenixMechanism.configure(
                ControllerPlan.of(
                    ROBOT.math(),
                    PIVOT,
                    ROBOT.math().drivesOf(PIVOT).get(0),
                    pivot.driven().motors().orElseThrow().controllers(),
                    tuned))
            .leader()
            .Slot0
            .kP;

    io.retune(tuned);
    Slot0Configs slot = new Slot0Configs();
    TalonFX motor = new TalonFX(52, new CANBus(BUS));
    boolean took =
        eventually(
            () -> motor.getConfigurator().refresh(slot),
            // Phoenix keeps a gain as a float.
            () -> Math.abs(slot.kP - expected) < 1e-6 * expected);

    assertThat(took).as("kP %s in Phoenix's units, read %s", expected, slot.kP).isTrue();
  }

  @Test
  void onlyAnFocMotorRunsWithFoc() {
    Drive foc = JointFixtures.motors(PIVOT, Motor.KRAKEN_X60_FOC, 1, JointFixtures.gearing(25));

    assertThat(JointIOTalon.foc(PIVOT_MOTOR)).isFalse();
    assertThat(JointIOTalon.foc(foc)).isTrue();
  }

  @Test
  void aJointWhoseMotorsArentWiredIsRefused() {
    assertThatThrownBy(() -> new JointIOTalon(ROBOT.math(), PIVOT))
        .hasMessage("pivot has no wiring: wire it");
  }

  /**
   * A pivot whose TalonFX reads its rotor, its CANcoder read by the robot once: powered on at 60°,
   * not where it rests, it's told so by the CANcoder's first good reading, and holds it. Until
   * 2026-10-05 it took its reference, 1.5°, whatever the CANcoder said.
   */
  @Test
  void aPivotReadByItsRotorIsToldWhereItIsByItsCancoder() throws InterruptedException {
    Joint.Revolute pivot =
        pivot(56).absoluteEncoder(CANCODER.can(CANIVORE, 57).offset(Rotations.of(0)));
    new CANcoder(57, new CANBus(BUS)).getSimState().setRawPosition(60.0 / 360);

    JointIOTalon io = io(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    boolean told =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - Math.toRadians(60)) < 0.01);
    boolean held =
        eventually(
            () -> io.updateInputs(inputs),
            () -> inputs.connected && Math.abs(inputs.position - Math.toRadians(60)) < 0.01);

    assertThat(told).as("its CANcoder's 60°, not its reference").isTrue();
    assertThat(held).as("its rotor reads it from then on").isTrue();
  }

  /**
   * A practice robot's elevator, its TalonFXs reading their rotors and its CANcoder, geared nine to
   * one down from its 1 in drum, read by the robot once: its CANcoder offset for its own magnet,
   * 0.06 rather than the competition robot's 0.31. Powered on at 0.6 m, not where it rests, it's
   * told so through its own offset, in meters, and holds it.
   */
  @Test
  void aPracticeElevatorIsToldWhereItIsByItsCancoderThroughItsOwnOffset()
      throws InterruptedException {
    MechanismControl unhomed =
        MechanismControlBuilder.builder(ControllerSettings.control(ELEVATOR))
            .homing(Optional.empty())
            .build();
    Joint.Prismatic competition =
        ELEVATOR
            .motorsOn(TALON_FX.can(CANIVORE, 60), TALON_FX.can(CANIVORE, 61))
            .absoluteEncoder(
                Driven.AbsoluteEncoder.of()
                    .ratio(1.0 / 9)
                    .on(CANCODER.can(CANIVORE, 62).offset(Rotations.of(0.31))))
            .retuned(old -> unhomed);
    Joint.Prismatic practice = competition.encoderOffset(Rotations.of(0.06));
    double perMeter = 1.0 / 9 / (2 * Math.PI * 0.0254);
    // Its magnet's raw reading at 0.6 m: where it reads, less its own offset.
    new CANcoder(62, new CANBus(BUS)).getSimState().setRawPosition(0.6 * perMeter - 0.06);

    JointIOTalon io = io(practice);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    boolean told =
        eventually(() -> io.updateInputs(inputs), () -> Math.abs(inputs.position - 0.6) < 1e-3);
    boolean held =
        eventually(
            () -> io.updateInputs(inputs),
            () -> inputs.connected && Math.abs(inputs.position - 0.6) < 1e-3);

    assertThat(told).as("its CANcoder's 0.6 m, through the practice robot's offset").isTrue();
    assertThat(held).as("its rotor reads it from then on").isTrue();
  }

  /**
   * A NEO pivot on a TalonFXS, through 25:1: it takes the same settings a TalonFX would, and reads
   * its rotor, the NEO's hall sensor, through its gearing from where it rests.
   */
  @Test
  void aNeoOnATalonFxsReadsItsRotorThroughItsGearing() throws InterruptedException {
    JointIOTalon io =
        io(
            unmotored(PIVOT)
                .motor(Motor.NEO, JointFixtures.gearing(25), TALON_FXS.can(CANIVORE, 58)));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    boolean resting =
        eventually(
            () -> io.updateInputs(inputs),
            () -> inputs.connected && Math.abs(inputs.position - PIVOT.restPosition()) < 0.01);
    new TalonFXS(58, new CANBus(BUS)).getSimState().setRawRotorPosition(25);
    boolean turned =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - (PIVOT.restPosition() + 2 * Math.PI)) < 0.01);

    assertThat(resting).isTrue();
    assertThat(turned).as("25 rotor turns are one of the pivot").isTrue();
    assertThat(inputs.unconfigured).as("the TalonFXS took its configuration").isEmpty();
  }

  /**
   * A device that started as the robot did hasn't restarted: the frame each sends as it starts, the
   * leader's, its follower's and its CANcoder's, came before the robot read them, and isn't one.
   * (CTRE's simulation can't restart a device, so a restart itself is a bench check.)
   */
  @Test
  void devicesThatStartedWithTheRobotHaventRestarted() throws InterruptedException {
    Joint.Revolute pivot =
        unmotored(PIVOT)
            .motors(
                Motor.KRAKEN_X60,
                gearing(25),
                TALON_FX.can(CANIVORE, 44),
                TALON_FX.can(CANIVORE, 45))
            .absoluteEncoder(CANCODER.can(CANIVORE, 46).offset(Rotations.of(0)));
    JointIOTalon io = io(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    boolean restarted = false;
    for (int i = 0; i < 20; i++) {
      io.updateInputs(inputs);
      restarted |= inputs.restarted || inputs.encoderRestarted;
      for (boolean follower : inputs.followersRestarted) {
        restarted |= follower;
      }
      Thread.sleep(10);
    }

    assertThat(inputs.followersRestarted).hasSize(1);
    assertThat(restarted).isFalse();
    assertThat(inputs.followersConnected).as("its follower answering").containsExactly(true);
    assertThat(inputs.faults)
        .as("leader, follower, CANcoder: none faulted")
        .containsExactly(0, 0, 0);
  }

  /**
   * A CANcoder that says its magnet is too close or too far isn't seeded from: the pivot reads
   * where it rests, and says its magnet is bad. Once the CANcoder says the magnet is good, the
   * pivot is seeded from it, at its 60°.
   */
  @Test
  void aCancoderWithABadMagnetIsntSeededFrom() throws InterruptedException {
    Joint.Revolute pivot =
        pivot(47).absoluteEncoder(CANCODER.can(CANIVORE, 48).offset(Rotations.of(0)));
    CANcoder cancoder = new CANcoder(48, new CANBus(BUS));
    cancoder.getSimState().setRawPosition(60.0 / 360);
    cancoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Red);

    JointIOTalon io = io(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    boolean bad = eventually(() -> io.updateInputs(inputs), () -> inputs.magnetBad);
    double unseeded = inputs.position;
    cancoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Green);
    boolean seeded =
        eventually(
            () -> io.updateInputs(inputs),
            () -> Math.abs(inputs.position - Math.toRadians(60)) < 0.01);

    assertThat(bad).isTrue();
    assertThat(Math.toDegrees(unseeded)).as("where it rests").isCloseTo(1.5, within(0.1));
    assertThat(seeded).isTrue();
    assertThat(inputs.magnetBad).isFalse();
    assertThat(Math.toDegrees(inputs.absolutePosition))
        .as("read beside")
        .isCloseTo(60, within(0.1));
  }
}

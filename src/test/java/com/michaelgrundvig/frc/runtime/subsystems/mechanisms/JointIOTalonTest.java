package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
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
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.ControlRequest;
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
import com.michaelgrundvig.frc.runtime.Waits;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.PrismaticJoint;
import com.michaelgrundvig.frc.world.robot.RevoluteJoint;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
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

  /** Reads made in a hold at the least, each {@link #READ_EVERY_MILLIS} apart. */
  private static final int LEAST_READS = 3 * JointIO.HEALTH_LOOPS;

  /** How often a hold reads again, as the robot loop would. */
  private static final long READ_EVERY_MILLIS = 10;

  /** The pivot, its motor on a TalonFX at an ID on the test's CANivore. */
  private static RevoluteJoint pivot(int id) {
    return PIVOT.motors(motors -> motors.on(TALON_FX.can(CANIVORE, id)));
  }

  /** A joint's IO, on the fixtures' robot with the joint as given. */
  private static JointIOTalon io(Joint joint, ControllerPins pins) {
    return new JointIOTalon(ROBOT.with(joint).math(), joint, pins);
  }

  private static JointIOTalon io(Joint joint) {
    return io(joint, ControllerPins.NONE);
  }

  /**
   * What the robot asks of a TalonFX, as it was last told: a profile's point as a PositionVoltage,
   * in turns of the pivot (its gearing is in its own settings), with the profile's speed and the
   * volts the robot adds; a speed as a VelocityVoltage; volts as a VoltageOut, and a share of 12 V
   * as a VoltageOut of that share, each within its soft limits; neutral; and homing's push as a
   * VoltageOut that ignores its soft limits, which until it's homed are in the wrong place.
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
    double voltsAsked = volts.Output;
    io.setDutyCycle(-0.5);
    VoltageOut share = (VoltageOut) talon.getAppliedControl();
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
    assertThat(voltsAsked).isEqualTo(4);
    assertThat(volts.IgnoreSoftwareLimits).isFalse();
    assertThat(share.Output).as("half of 12 V, backward").isEqualTo(-6);
    assertThat(share.IgnoreSoftwareLimits).isFalse();
    assertThat(neutral).isInstanceOf(NeutralOut.class);
    assertThat(homing).as("not the VoltageOut volts go as").isNotSameAs(volts);
    assertThat(homing.Output).isEqualTo(-2);
    assertThat(homing.IgnoreSoftwareLimits).isTrue();
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

    Waits.until(
        "it starts at its reference, 1.5°",
        () -> io.updateInputs(inputs),
        () -> inputs.connected && Math.abs(inputs.position - RobotMath.restPosition(PIVOT)) < 0.01);
    // Its raw rotor started at 0, where the startup told it the pivot rests.
    new TalonFX(51, new CANBus(BUS)).getSimState().setRawRotorPosition(25);
    Waits.until(
        "25 rotor turns are one of the pivot",
        () -> io.updateInputs(inputs),
        () -> Math.abs(inputs.position - (RobotMath.restPosition(PIVOT) + 2 * Math.PI)) < 0.01);
    assertThat(inputs.unconfigured).as("every device took its configuration").isEmpty();
  }

  /** The elevator's two motors: the first leads, and it reads meters through its drum. */
  @Test
  void aSlideReadsMetersThroughItsDrum() throws InterruptedException {
    JointIOTalon io =
        io(
            ELEVATOR.motors(
                motors -> motors.on(TALON_FX.can(CANIVORE, 53), TALON_FX.can(CANIVORE, 54))));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    Waits.until(
        "the elevator's motors answering", () -> io.updateInputs(inputs), () -> inputs.connected);

    // Ten rotor turns through 10:1 are one turn of the 1 in drum: 2π inches of travel.
    new TalonFX(53, new CANBus(BUS)).getSimState().setRawRotorPosition(10);
    double travel = 2 * Math.PI * 0.0254;
    Waits.until(
        "ten rotor turns through the drum read as its travel",
        () -> io.updateInputs(inputs),
        () -> Math.abs(inputs.position - (RobotMath.restPosition(ELEVATOR) + travel)) < 1e-4);
    assertThat(inputs.position).isCloseTo(travel, within(1e-4));
  }

  /**
   * A share is of 12 V, whatever the battery reads: a tenth is 1.2 V across the motor, on a battery
   * at 12.5 V and on one sagged to 10 V, as on a SPARK, where a share of the battery would be 1.25
   * V and 1 V. (A tenth: the pivot stands stalled, and more would meet its current limit.) And it
   * goes as a voltage, the leader's motor voltage sent every loop, which is what its followers
   * copy: a Phoenix duty cycle would have them copy its duty cycle signal, which Phoenix sends four
   * times a second once trimmed.
   */
  @Test
  void aShareIsOfTwelveVoltsWhateverTheBatteryAndFollowersHearItEveryLoop()
      throws InterruptedException {
    RobotSim.setEnabled(true);
    JointIOTalon io =
        io(
            unmotored(PIVOT)
                .motors(
                    Motor.KRAKEN_X60
                        .through(gearing(25))
                        .on(TALON_FX.can(CANIVORE, 57), TALON_FX.can(CANIVORE, 55))));
    TalonFX leader = new TalonFX(57, new CANBus(BUS));
    for (double battery : new double[] {12.5, 10}) {
      leader.getSimState().setSupplyVoltage(battery);
      io.setDutyCycle(0.1);
      // Phoenix's simulated TalonFX runs again as its rotor is set.
      Waits.until(
          () ->
              String.format(
                  "on %s V, across the motor %s V",
                  battery, leader.getSimState().getMotorVoltage()),
          () -> {
            RobotSim.stepClock();
            leader.getSimState().setRawRotorPosition(0);
          },
          () -> Math.abs(leader.getSimState().getMotorVoltage() - 1.2) < 0.05);
    }
    io.stop();

    assertThat(leader.getMotorVoltage().getAppliedUpdateFrequency())
        .as("the leader's motor voltage, which followers of a voltage copy")
        .isGreaterThanOrEqualTo(20);
  }

  /**
   * A CANcoder whose word on its magnet Phoenix calls invalid isn't seeded from, as one that says
   * red isn't: it hasn't said its magnet is good.
   */
  @Test
  void aCancoderWhoseMagnetIsInvalidIsntSeededFrom() throws InterruptedException {
    RevoluteJoint pivot =
        pivot(50).absoluteEncoder(CANCODER.can(CANIVORE, 49).offset(Rotations.of(0)));
    CANcoder cancoder = new CANcoder(49, new CANBus(BUS));
    cancoder.getSimState().setRawPosition(60.0 / 360);
    cancoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Invalid);

    JointIOTalon io = io(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    Waits.until(
        "the CANcoder's position, read beside the rotor",
        () -> io.updateInputs(inputs),
        () -> Math.abs(inputs.absolutePosition - Math.toRadians(60)) < 0.01);
    // Its magnet's health comes at a rate of its own: three of its reports, and still unseeded.
    double magnetHz = cancoder.getMagnetHealth().getAppliedUpdateFrequency();
    assertThat(magnetHz).as("how often the CANcoder reports its magnet").isPositive();
    holds(
        "the pivot unseeded while its CANcoder's magnet is invalid",
        reportsOf(magnetHz),
        () -> io.updateInputs(inputs),
        () -> Math.abs(inputs.position - Math.toRadians(60)) > 0.01);
    double unseeded = inputs.position;
    cancoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Green);
    Waits.until(
        "the pivot seeded from its CANcoder at 60°",
        () -> io.updateInputs(inputs),
        () -> Math.abs(inputs.position - Math.toRadians(60)) < 0.01);

    assertThat(Math.toDegrees(unseeded)).as("where it rests").isCloseTo(1.5, within(0.1));
    assertThat(inputs.magnetBad).as("not red: just not said").isFalse();
  }

  /** A tuned kP reaches the motor controller's slot, in Phoenix's units, without a restart. */
  @Test
  void aTunedGainReachesItsSlot() throws InterruptedException {
    RevoluteJoint pivot = pivot(52);
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
    Waits.until(
        () -> String.format(Locale.ROOT, "kP %s in Phoenix's units, read %s", expected, slot.kP),
        () -> motor.getConfigurator().refresh(slot),
        // Phoenix keeps a gain as a float.
        () -> Math.abs(slot.kP - expected) < 1e-6 * expected);
  }

  /**
   * Positions run on slot 0 and speeds on slot 1, which holds the same gains without the kI. A
   * retune sends both, so the two never hold different tunings.
   */
  @Test
  void aRetuneKeepsTheSpeedSlotInStepWithoutItsKi() throws InterruptedException {
    RevoluteJoint pivot = pivot(56);
    JointIOTalon io = io(pivot);
    MechanismControl tuned = ControllerSettings.control(pivot).withKp(55).withKi(4).withKd(2);
    Slot0Configs positions = new Slot0Configs();
    Slot1Configs speeds = new Slot1Configs();
    TalonFX motor = new TalonFX(56, new CANBus(BUS));
    motor.getConfigurator().refresh(speeds);
    double speedKiAtStartup = speeds.kI;
    double speedKpAtStartup = speeds.kP;

    io.retune(tuned);
    Waits.until(
        () ->
            String.format(
                "slot 0 kP %s kI %s, slot 1 kP %s", positions.kP, positions.kI, speeds.kP),
        () -> {
          motor.getConfigurator().refresh(positions);
          motor.getConfigurator().refresh(speeds);
        },
        () -> positions.kI > 0 && speeds.kP == positions.kP && speeds.kD == positions.kD);

    assertThat(speedKpAtStartup).as("the speed slot has the plan's kP at startup").isPositive();
    assertThat(speedKiAtStartup).as("and no kI").isZero();
    assertThat(speeds.kI).as("the tuned kI is the position slot's alone").isZero();
    assertThat(speeds.kS).isEqualTo(positions.kS);
    assertThat(speeds.kV).isEqualTo(positions.kV);
    assertThat(speeds.kG).isEqualTo(positions.kG);
    assertThat(speeds.GravityType).isEqualTo(positions.GravityType);
  }

  /**
   * Every request a TalonFX is sent asks for FOC when its joint's controllers say {@code .foc()},
   * and none does when they don't: a profile's point, a speed, volts, a share of 12 V and homing's
   * push.
   */
  @Test
  void eachRequestAsksForFocOnlyWhenItsControllersSaySo() {
    record Row(String name, RevoluteJoint joint, boolean foc) {}
    List<Row> rows =
        List.of(
            new Row("not said", pivot(41), false),
            new Row(
                "said .foc()",
                PIVOT.motors(motors -> motors.on(TALON_FX.can(CANIVORE, 42).foc())),
                true));
    for (Row row : rows) {
      JointIOTalon io = io(row.joint());
      ParentDevice talon = io.leader();
      List<Boolean> asked = new ArrayList<>();
      io.setPosition(Math.toRadians(45), 1.0, 0.6);
      asked.add(((PositionVoltage) talon.getAppliedControl()).EnableFOC);
      io.setVelocity(3.0);
      asked.add(((VelocityVoltage) talon.getAppliedControl()).EnableFOC);
      io.setVoltage(4);
      asked.add(((VoltageOut) talon.getAppliedControl()).EnableFOC);
      io.setDutyCycle(-0.5);
      asked.add(((VoltageOut) talon.getAppliedControl()).EnableFOC);
      io.home(-2);
      asked.add(((VoltageOut) talon.getAppliedControl()).EnableFOC);
      io.stop();

      assertThat(asked).as(row.name()).hasSize(5).containsOnly(row.foc());
    }
  }

  @Test
  void aJointWhoseMotorsArentWiredIsRefused() {
    assertThatThrownBy(() -> new JointIOTalon(ROBOT.math(), PIVOT))
        .hasMessage("pivot has no wiring: wire it");
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
    PrismaticJoint competition =
        ELEVATOR
            .motors(motors -> motors.on(TALON_FX.can(CANIVORE, 60), TALON_FX.can(CANIVORE, 61)))
            .absoluteEncoder(
                Driven.AbsoluteEncoder.of()
                    .ratio(1.0 / 9)
                    .on(CANCODER.can(CANIVORE, 62).offset(Rotations.of(0.31))))
            .retuned(old -> unhomed);
    PrismaticJoint practice =
        competition.absoluteEncoder(encoder -> encoder.offset(Rotations.of(0.06)));
    double perMeter = 1.0 / 9 / (2 * Math.PI * 0.0254);
    // Its magnet's raw reading at 0.6 m: where it reads, less its own offset.
    new CANcoder(62, new CANBus(BUS)).getSimState().setRawPosition(0.6 * perMeter - 0.06);

    JointIOTalon io = io(practice);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    Waits.until(
        "its CANcoder's 0.6 m, through the practice robot's offset",
        () -> io.updateInputs(inputs),
        () -> Math.abs(inputs.position - 0.6) < 1e-3);
    Waits.until(
        "its rotor reads it from then on",
        () -> io.updateInputs(inputs),
        () -> inputs.connected && Math.abs(inputs.position - 0.6) < 1e-3);
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
                .motors(
                    Motor.NEO.through(JointFixtures.gearing(25)).on(TALON_FXS.can(CANIVORE, 58))));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    Waits.until(
        "it starts where it rests",
        () -> io.updateInputs(inputs),
        () -> inputs.connected && Math.abs(inputs.position - RobotMath.restPosition(PIVOT)) < 0.01);
    new TalonFXS(58, new CANBus(BUS)).getSimState().setRawRotorPosition(25);
    Waits.until(
        "25 rotor turns are one of the pivot",
        () -> io.updateInputs(inputs),
        () -> Math.abs(inputs.position - (RobotMath.restPosition(PIVOT) + 2 * Math.PI)) < 0.01);
    assertThat(inputs.unconfigured).as("the TalonFXS took its configuration").isEmpty();
  }

  /**
   * A device that started as the robot did hasn't restarted: the frame each sends as it starts, the
   * leader's, its follower's and its CANcoder's, came before the robot read them, and isn't one.
   * (CTRE's simulation can't restart a device, so a restart itself is a bench check.)
   */
  @Test
  void devicesThatStartedWithTheRobotHaventRestarted() throws InterruptedException {
    RevoluteJoint pivot =
        unmotored(PIVOT)
            .motors(
                Motor.KRAKEN_X60
                    .through(gearing(25))
                    .on(TALON_FX.can(CANIVORE, 44), TALON_FX.can(CANIVORE, 45)))
            .absoluteEncoder(CANCODER.can(CANIVORE, 46).offset(Rotations.of(0)));
    JointIOTalon io = io(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    // Every read from the first, while the devices come up and after: none says a restart.
    Runnable read =
        () -> {
          io.updateInputs(inputs);
          assertThat(restartedNow(inputs)).as("a restart said, read by read").isFalse();
        };
    Waits.until(
        "the leader's frames and its CANcoder's position",
        read,
        () -> inputs.fresh && !Double.isNaN(inputs.absolutePosition));
    // The follower sends only its fault field, the slowest frame of the three: three of those.
    holds(
        "no device said it restarted",
        reportsOf(JointIOTalon.FAULTS_HZ),
        read,
        () -> !restartedNow(inputs));

    assertThat(inputs.followersRestarted).hasSize(1);
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
    RevoluteJoint pivot =
        pivot(47).absoluteEncoder(CANCODER.can(CANIVORE, 48).offset(Rotations.of(0)));
    CANcoder cancoder = new CANcoder(48, new CANBus(BUS));
    cancoder.getSimState().setRawPosition(60.0 / 360);
    cancoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Red);

    JointIOTalon io = io(pivot);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    Waits.until(
        "the CANcoder said its magnet is bad",
        () -> io.updateInputs(inputs),
        () -> inputs.magnetBad);
    double unseeded = inputs.position;
    cancoder.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Green);
    Waits.until(
        "the pivot seeded from its CANcoder at 60°",
        () -> io.updateInputs(inputs),
        () -> Math.abs(inputs.position - Math.toRadians(60)) < 0.01);
    assertThat(Math.toDegrees(unseeded)).as("where it rests").isCloseTo(1.5, within(0.1));
    assertThat(inputs.magnetBad).isFalse();
    assertThat(Math.toDegrees(inputs.absolutePosition))
        .as("read beside")
        .isCloseTo(60, within(0.1));
  }

  /** Whether a device said, as of the last read, that it restarted. */
  private static boolean restartedNow(JointIO.JointIOInputs inputs) {
    if (inputs.restarted || inputs.encoderRestarted) {
      return true;
    }
    for (boolean follower : inputs.followersRestarted) {
      if (follower) {
        return true;
      }
    }
    return false;
  }

  /** Three reports of a frame sent this often: past the one in flight, and two more. */
  private static Duration reportsOf(double hertz) {
    return Duration.ofNanos(Math.round(3e9 / hertz));
  }

  /**
   * Reads for as long as it takes to see something not happen: past {@code atLeast} by the clock,
   * and at least {@link #LEAST_READS} reads, so a slow machine reads for longer, never less.
   *
   * @param what what holds, as the failure says it
   * @param holds whether it still holds, from what was read
   */
  private static void holds(String what, Duration atLeast, Runnable read, BooleanSupplier holds)
      throws InterruptedException {
    long until = System.nanoTime() + atLeast.toNanos();
    int reads = 0;
    while (System.nanoTime() < until || reads < LEAST_READS) {
      read.run();
      reads++;
      assertThat(holds.getAsBoolean()).as(what + ", at read " + reads).isTrue();
      Thread.sleep(READ_EVERY_MILLIS);
    }
  }
}

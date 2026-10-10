package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_MOTOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.assumed;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.soft;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO.JointIOInputs;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIOInputsAutoLogged;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;

/**
 * One simulated joint, its motor controller and its physics, from the fixtures' robot: gravity as
 * the model works it out, against the feedforward the model seeds; its hard stops; neutral. And
 * joints without mass, whose gravity is what the kG tuned in their blocks holds.
 */
class JointIOSimTest {
  private static final double STEP = 0.02;

  /** An arm with no mass, its kG tuned: 0.4 V, its weight pulling hardest at its zero. */
  private static final Joint.Revolute BARE_ARM =
      Joint.pivot("bare arm", Part.CHASSIS, new Translation3d(0.1, 0, 0.4), Joint.PITCH_UP)
          .range(Range.degrees(-120, 90))
          .rest(Degrees.of(0))
          .motor(Motor.KRAKEN_X60, Gearing.of(50))
          .control(
              MechanismControl.position(30)
                  .withKd(1)
                  .withKg(0.4)
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  /** An elevator with no mass, its kG tuned: 0.5 V all along it. */
  private static final Joint.Prismatic BARE_LIFT =
      Joint.elevator("bare lift", Part.CHASSIS, new Translation3d(-0.2, 0, 0.1))
          .range(Range.meters(0, 1))
          .rest(Meters.of(0))
          .motor(Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1))
          .control(
              MechanismControl.position(60)
                  .withKd(2)
                  .withKg(0.5)
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  private static final RobotDefinition MASSLESS =
      RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
          .name("massless")
          .joints(List.of(BARE_ARM, BARE_LIFT))
          .build();

  /** The pivot's seeded gravity feedforward: the volts that hold it level. */
  private static final double KG = ROBOT.math().idealGains(PIVOT).kG();

  private static JointIOSim joint(Joint joint, MechanismControl control) {
    return new JointIOSim(ROBOT.math(), joint.retuned(old -> assumed(control)));
  }

  /** Runs one joint alone for a time, gravity worked out where it is at each step. */
  private static void run(JointIOSim sim, double seconds) {
    for (int i = 0; i < Math.round(seconds / STEP); i++) {
      double gravity =
          -ROBOT.math().gravityTorque(sim.joint(), Map.of(sim.joint(), sim.position()));
      sim.step(STEP, gravity);
    }
  }

  /**
   * Holds a massless joint at a target for 2 s in the simulator, its block's kG setting gravity on
   * it and its motor controller running a control of its own, and says where it ends.
   */
  private static double held(Joint joint, MechanismControl control, double target) {
    JointSimulation simulation = new JointSimulation(MASSLESS.with(joint));
    JointIO io = simulation.io(joint);
    io.retune(control);
    simulation.place(joint, target);
    io.setPosition(target, 0, 0);
    for (int i = 0; i < 100; i++) {
      simulation.step(STEP);
    }
    return simulation.position(joint);
  }

  private static MechanismControl blocks(Joint joint) {
    return joint.driven().control().orElseThrow();
  }

  private static MechanismControl pivotControl(Optional<Double> kG) {
    MechanismControl seeded = control(30, 1, profile(8, 30));
    return MechanismControl.builder()
        .loop(MechanismControl.Loop.POSITION)
        .gains(MechanismControl.Gains.builder().kP(30).kI(0).kD(1).kS(0).kG(kG).build())
        .currentLimits(seeded.currentLimits())
        .neutralMode(seeded.neutralMode())
        .deadChannel(seeded.deadChannel())
        .softLimits(seeded.softLimits())
        .profile(seeded.profile())
        .bands(seeded.bands())
        .homing(seeded.homing())
        .build();
  }

  @Test
  void itStartsStillWhereItsDefinitionRestsIt() {
    JointIOSim sim = joint(PIVOT, pivotControl(Optional.empty()));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    sim.updateInputs(inputs);

    assertThat(inputs.connected).isTrue();
    assertThat(inputs.position).isEqualTo(PIVOT.restPosition());
    assertThat(inputs.velocity).isZero();
    assertThat(inputs.unconfigured).isEmpty();
  }

  /**
   * Held at 45° on its seeded gravity feedforward, the pivot stays there: the model's gravity and
   * its feedforward agree. Without it, the pivot sags until its feedback makes up gravity's volts.
   */
  @Test
  void theSeededGravityFeedforwardHoldsWhatTheModelsGravityPulls() {
    JointIOSim seeded = joint(PIVOT, pivotControl(Optional.empty()));
    JointIOSim none = joint(PIVOT, pivotControl(Optional.of(0.0)));
    double target = Math.toRadians(45);
    seeded.place(target);
    none.place(target);

    seeded.setPosition(target, 0, 0);
    none.setPosition(target, 0, 0);
    run(seeded, 2);
    run(none, 2);

    assertThat(KG).as("2 kg 0.3 m out takes volts to hold").isGreaterThan(0.1);
    assertThat(seeded.position()).isCloseTo(target, within(1e-4));
    assertThat(target - none.position())
        .as("its sag: the volts gravity takes, over kP")
        .isCloseTo(KG * Math.cos(target) / 30, within(0.02 * KG / 30));
  }

  /** Coasting, the pivot falls under its weight to its lower stop, and stays on it. */
  @Test
  void coastingItFallsToItsStopAndStaysThere() {
    MechanismControl coast =
        MechanismControlBuilder.builder(pivotControl(Optional.empty()))
            .neutralMode(MechanismControl.NeutralMode.COAST)
            .build();
    JointIOSim sim = joint(PIVOT, coast);
    sim.place(Math.toRadians(60));

    sim.stop();
    run(sim, 2);

    assertThat(sim.position()).isEqualTo(0);
    assertThat(sim.velocity()).isZero();
  }

  /** Braking, its motors' back EMF slows the fall: after the same 0.3 s, it's fallen less. */
  @Test
  void brakingItFallsSlowerThanCoasting() {
    JointIOSim brake = joint(PIVOT, pivotControl(Optional.empty()));
    JointIOSim coast =
        joint(
            PIVOT,
            MechanismControlBuilder.builder(pivotControl(Optional.empty()))
                .neutralMode(MechanismControl.NeutralMode.COAST)
                .build());
    brake.place(Math.toRadians(60));
    coast.place(Math.toRadians(60));

    brake.stop();
    coast.stop();
    run(brake, 0.3);
    run(coast, 0.3);

    assertThat(brake.position()).isLessThan(Math.toRadians(60));
    assertThat(brake.position()).isGreaterThan(coast.position());
  }

  /** Full voltage up: the elevator stops driving at its soft limit, short of its hard stop. */
  @Test
  void itDrivesNoFurtherThanItsSoftLimits() {
    JointIOSim sim = joint(ELEVATOR, control(60, 2, soft(0, 1.15), profile(1.2, 5), 40));

    sim.setVoltage(12);
    double highest = 0;
    for (int i = 0; i < 100; i++) {
      run(sim, STEP);
      highest = Math.max(highest, sim.position());
    }

    assertThat(highest).isLessThan(1.2);
    assertThat(sim.position()).isCloseTo(1.15, within(0.03));
  }

  /** A duty cycle is a share of the battery's 12 V: half of it, as 6 V does. */
  @Test
  void aDutyCycleIsAShareOfTheBattery() {
    JointIOSim duty = joint(ELEVATOR, control(60, 2, profile(1.2, 5)));
    JointIOSim volts = joint(ELEVATOR, control(60, 2, profile(1.2, 5)));

    duty.setDutyCycle(0.5);
    volts.setVoltage(6);
    run(duty, 0.5);
    run(volts, 0.5);

    assertThat(duty.position()).isEqualTo(volts.position());
    assertThat(duty.position()).isGreaterThan(0.1);
  }

  /**
   * Its stator limit holds each motor: each of the elevator's two draws 20 A, as its leader reads.
   */
  @Test
  void itsStatorLimitHoldsEachMotor() {
    JointIOSim sim = joint(ELEVATOR, control(60, 2, Optional.empty(), profile(1.2, 5), 20));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    sim.setVoltage(12);
    sim.step(0.001, 0);
    sim.updateInputs(inputs);

    assertThat(inputs.statorCurrentAmps).isCloseTo(20, within(1e-6));
  }

  /**
   * Pushed into its upper stop at 3 V, the pivot is held there, its Kraken stalled at its 40 A
   * limit: what the motor's model says it draws from the battery standing still, 40 A through its
   * windings at 1.3 V, 4.4 A, and no more. Its controller and its PDH channel read that.
   */
  @Test
  void pushedIntoItsStopItDrawsWhatItsStalledMotorDraws() {
    JointIOSim sim = joint(PIVOT, pivotControl(Optional.empty()));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    sim.place(Math.toRadians(130));

    sim.setVoltage(3);
    run(sim, 1);
    sim.updateInputs(inputs);

    double stalled = 40 * 40 * DCMotor.getKrakenX60(1).R / 12;
    assertThat(sim.position()).isEqualTo(Math.toRadians(135));
    assertThat(inputs.statorCurrentAmps).isCloseTo(40, within(1e-9));
    assertThat(inputs.supplyCurrentAmps)
        .isCloseTo(stalled, within(1e-9))
        .isCloseTo(4.4, within(0.1));
    assertThat(sim.supplyAmps()).isEqualTo(inputs.supplyCurrentAmps);
  }

  /**
   * A tuned kG holds an arm with no mass: the simulator's gravity is what that kG holds, so its
   * control's kG holds it where it's put. With kG 0 in its motor controller, it sags until its
   * feedback makes up the volts: kG times the cosine, over kP. Told nothing, it falls.
   */
  @Test
  void aTunedKgHoldsAnArmWithoutMass() {
    double target = Math.toRadians(30);
    MechanismControl tuned = blocks(BARE_ARM);

    double holding = held(BARE_ARM, tuned, target);
    double sagging = held(BARE_ARM, tuned.withKg(0), target);
    JointSimulation simulation = new JointSimulation(MASSLESS);
    simulation.io(BARE_ARM).stop();
    simulation.place(BARE_ARM, target);
    for (int i = 0; i < 25; i++) {
      simulation.step(STEP);
    }

    assertThat(BARE_ARM.link().mass().isMassless()).isTrue();
    assertThat(holding).isCloseTo(target, within(1e-4));
    assertThat(target - sagging)
        .as("its sag: the volts gravity takes, over kP")
        .isCloseTo(0.4 * Math.cos(target) / 30, within(0.02 * 0.4 / 30));
    assertThat(simulation.position(BARE_ARM)).isLessThan(target - 0.05);
  }

  /** The same for a slide: its tuned kG holds it all along, and without it, it sags by kG / kP. */
  @Test
  void aTunedKgHoldsASlideWithoutMass() {
    MechanismControl tuned = blocks(BARE_LIFT);

    assertThat(held(BARE_LIFT, tuned, 0.5)).isCloseTo(0.5, within(1e-5));
    assertThat(0.5 - held(BARE_LIFT, tuned.withKg(0), 0.5))
        .isCloseTo(0.5 / 60, within(0.02 * 0.5 / 60));
  }

  /**
   * An arm's weight pulls hardest at the angle given with its kG: on feedback alone, it sags most
   * there, kG over kP, and not at all a quarter turn on, where it hangs straight.
   */
  @Test
  void anArmsWeightPullsHardestAtTheAngleGiven() {
    Joint.Revolute arm = BARE_ARM.retuned(control -> control.withKg(0.4, Degrees.of(-90)));
    MechanismControl feedbackAlone = blocks(arm).withKg(0);
    double hardest = Math.toRadians(-90);

    assertThat(held(arm, blocks(arm), hardest)).isCloseTo(hardest, within(1e-4));
    assertThat(hardest - held(arm, feedbackAlone, hardest))
        .isCloseTo(0.4 / 30, within(0.02 * 0.4 / 30));
    assertThat(held(arm, feedbackAlone, 0)).isCloseTo(0, within(1e-6));
  }

  /**
   * What the robot's settings for a joint would doubt, the robot says at startup wherever it runs,
   * in the robot's words: a pivot whose tuned kG is a third of what its arm's weight takes is
   * doubted wired to nothing, run as the TalonFX it's wired to on a robot.
   */
  @Test
  void itDoubtsWhatTheRobotsSettingsWould() {
    MechanismControl light = assumed(pivotControl(Optional.of(KG / 3)));
    MechanismWiring.Device onTalon = MechanismWiring.Controller.TALON_FX.can(32).clockwise();
    List<String> robots =
        PhoenixMechanism.configure(
                ControllerPlan.of(ROBOT.math(), PIVOT, PIVOT_MOTOR, List.of(onTalon), light))
            .notes();
    Joint unwiredPivot = PIVOT.retuned(old -> light);
    Joint wiredPivot = PIVOT.motorsOn(onTalon).retuned(old -> light);

    List<Finding> unwired =
        ControllerSettings.findings(ROBOT.with(unwiredPivot).math(), unwiredPivot);
    List<Finding> wired = ControllerSettings.findings(ROBOT.with(wiredPivot).math(), wiredPivot);

    assertThat(robots)
        .singleElement()
        .asString()
        .startsWith("pivot's tuned kG, ")
        .contains("is 0.3333 times the ideal");
    assertThat(unwired).extracting(Finding::message).isEqualTo(robots);
    assertThat(unwired).allMatch(finding -> finding.level() == Finding.Level.DOUBT);
    assertThat(wired).extracting(Finding::message).isEqualTo(robots);
  }

  /**
   * The simulator refuses a joint the robot would refuse at startup, in the robot's words, though
   * its own motor controller could run it: an arm whose weight pulls hardest a third of a turn from
   * its zero, past the quarter turn Phoenix takes, on the TalonFX the simulator stands in for,
   * wired to nothing.
   */
  @Test
  void itRefusesAJointTheRobotWouldRefuse() {
    Joint.Revolute turned =
        BARE_ARM.retuned(
            control ->
                control
                    .withKg(0.4, Degrees.of(-120))
                    .withHoming(MechanismControl.Homing.assumeAtRest()));
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(MASSLESS).joints(List.of(turned)).build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> new JointsSimHarness(robot))
        .withMessageStartingWith(
            "bare arm's gravity angle, -120°, is more than a quarter turn from its zero, past"
                + " Phoenix's gravity offset of ±0.25 rotations");
  }

  /**
   * Restarted, its motor controller says so the next time it's read, and only then; it counts from
   * 0, its motor's own count being all it reads; and it's neutral until it's asked again.
   */
  @Test
  void aRestartedControllerSaysSoOnceCountsFromZeroAndGoesNeutral() {
    JointIOSim sim = joint(PIVOT, control(30, 1, profile(8, 30)));
    sim.place(Math.toRadians(60));
    sim.setVoltage(4);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();

    sim.restart();
    sim.updateInputs(inputs);
    boolean said = inputs.restarted;
    double read = inputs.position;
    run(sim, 0.2);
    sim.updateInputs(inputs);

    assertThat(said).isTrue();
    assertThat(read).isZero();
    assertThat(inputs.restarted).as("once").isFalse();
    assertThat(Math.toDegrees(sim.position()))
        .as("not pushed on at 4 V: braking, it sags under its weight")
        .isLessThan(60.01);
  }

  /**
   * The fixtures' turret, which turns freely, on a TalonFX that reads a CANcoder turning with it.
   */
  private static final Joint.Revolute TURRET_ON_CANCODER =
      TURRET.motorsOn(TALON_FX.can(23)).absoluteEncoder(CANCODER.can(35).offset(Rotations.of(0)));

  /**
   * The turret a turn and a quarter round from where it rests, its motor controller counting it
   * there since it started: restarted, it's seeded from its CANcoder, as a real one is, inside the
   * window the CANcoder's turn tells apart, a half turn either side of where the turret rests. It
   * reads 90°, not 450°. Until 2026-10-08 it was seeded with the turns the CANcoder can't tell.
   */
  @Test
  void aRestartedControllerIsSeededInsideItsEncodersTurn() {
    JointSimulation simulation = new JointSimulation(ROBOT.with(TURRET_ON_CANCODER));
    JointIO io = simulation.io(TURRET);
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    simulation.place(TURRET, Math.toRadians(450));
    io.updateInputs(inputs);
    double counted = inputs.position;

    simulation.restart(TURRET);
    io.updateInputs(inputs);

    assertThat(Math.toDegrees(counted)).isCloseTo(450, within(1e-9));
    assertThat(inputs.restarted).isTrue();
    assertThat(Math.toDegrees(inputs.position)).isCloseTo(90, within(1e-9));
  }

  /**
   * Silent, off the bus, its motor controller answers nothing, its last readings standing, and
   * hears nothing, so it goes neutral; brought back, it answers again.
   */
  @Test
  void aSilentControllerAnswersNothingAndHearsNothing() {
    JointIOSim sim = joint(PIVOT, control(30, 1, profile(8, 30)));
    sim.place(Math.toRadians(60));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    sim.updateInputs(inputs);

    sim.silence(true);
    sim.setVoltage(4);
    run(sim, 0.2);
    sim.updateInputs(inputs);
    boolean answered = inputs.connected;
    double read = inputs.position;
    sim.silence(false);
    sim.updateInputs(inputs);

    assertThat(answered).isFalse();
    assertThat(Math.toDegrees(read)).as("its last reading").isCloseTo(60, within(1e-9));
    assertThat(Math.toDegrees(sim.position())).as("not pushed at 4 V").isLessThan(60.01);
    assertThat(inputs.connected).as("back").isTrue();
  }

  /**
   * Jammed, it stays where it is whatever its motors do: driven at 3 V, they stall against what
   * holds it, at their 40 A stator limit, drawing what a stalled motor draws, as at its stop.
   * Freed, it moves again.
   */
  @Test
  void aJammedJointStaysPutItsMotorsStalled() {
    JointIOSim sim = joint(PIVOT, pivotControl(Optional.empty()));
    JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
    sim.place(Math.toRadians(60));

    sim.jam(true);
    sim.setVoltage(3);
    run(sim, 1);
    sim.updateInputs(inputs);
    double jammedAt = sim.position();
    sim.jam(false);
    run(sim, 0.2);

    assertThat(Math.toDegrees(jammedAt)).isCloseTo(60, within(1e-9));
    assertThat(inputs.velocity).isZero();
    assertThat(inputs.statorCurrentAmps).isCloseTo(40, within(1e-9));
    assertThat(inputs.supplyCurrentAmps)
        .isCloseTo(40 * 40 * DCMotor.getKrakenX60(1).R / 12, within(1e-9));
    assertThat(Math.toDegrees(sim.position())).as("freed").isGreaterThan(65);
  }

  /** An elevator on two TalonFXs, fed from the PDH's channels 0 and 1, its kG holding it. */
  private static final Joint.Prismatic WIRED_LIFT =
      Joint.elevator("wired lift", Part.CHASSIS, new Translation3d(0, 0.2, 0.1))
          .range(Range.meters(0, 1.2))
          .rest(Meters.of(0))
          .motors(
              Motor.KRAKEN_X60,
              Gearing.of(5),
              Inches.of(0.75),
              TALON_FX.can(20).pdh(0, Amps.of(40)),
              TALON_FX.can(21).clockwise().pdh(1, Amps.of(40)))
          .control(
              MechanismControl.position(30)
                  .withKg(0.35)
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  private static final RobotDefinition WIRED =
      RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
          .name("wired")
          .joints(List.of(WIRED_LIFT))
          .pdh(Pdh.can(1))
          .build();

  /** Runs the wired lift up at 8 V for a time, its follower off the bus or not: where it got. */
  private static double raised(boolean followerOff, PdhIOInputsAutoLogged pdh, JointIOInputs read) {
    JointSimulation simulation = new JointSimulation(WIRED);
    PdhIOSim channels = new PdhIOSim(WIRED, simulation, null);
    JointIO io = simulation.io(WIRED_LIFT);
    simulation.silenceFollower(WIRED_LIFT, 0, followerOff);
    io.setVoltage(8);
    for (int i = 0; i < 10; i++) {
      simulation.step(STEP);
    }
    io.updateInputs(read);
    channels.updateInputs(pdh);
    return simulation.position(WIRED_LIFT);
  }

  /**
   * A follower off the CAN bus answers nothing and goes neutral: it reads disconnected, its PDH
   * channel reads nothing, and its leader lifts the joint alone, slower, drawing what it draws on
   * its own channel. Back on the bus, it reads connected.
   */
  @Test
  void aFollowerOffTheBusDrawsNothingAndItsLeaderDrivesAlone() {
    PdhIOInputsAutoLogged pdh = new PdhIOInputsAutoLogged();
    JointIOInputs read = new JointIOInputs();
    PdhIOInputsAutoLogged both = new PdhIOInputsAutoLogged();
    JointIOInputs bothRead = new JointIOInputs();

    double alone = raised(true, pdh, read);
    double together = raised(false, both, bothRead);

    assertThat(read.connected).isTrue();
    assertThat(read.followersConnected).containsExactly(false);
    assertThat(bothRead.followersConnected).containsExactly(true);
    assertThat(pdh.channelCurrentAmps[1]).as("the follower's channel").isZero();
    assertThat(pdh.channelCurrentAmps[0])
        .as("the leader's")
        .isGreaterThan(1)
        .isCloseTo(read.supplyCurrentAmps, within(1e-9));
    assertThat(both.channelCurrentAmps[1]).isCloseTo(both.channelCurrentAmps[0], within(1e-9));
    assertThat(alone).as("lifted by one motor").isGreaterThan(0).isLessThan(together);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new JointSimulation(WIRED).silenceFollower(WIRED_LIFT, 1, true));
  }
}

package com.michaelgrundvig.frc.runtime.sim;

import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.runtime.behavior.Debounce;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.simulation.JointPlant;
import com.michaelgrundvig.frc.world.robot.AbsoluteReader;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.wpilib.math.filter.Debouncer.DebounceType;

/**
 * A simulated joint, as the robot's code reads and drives it: the joint's motor controller,
 * emulated here, driving the joint's plant ({@link JointPlant}, in the simulation library: the
 * motors through the transmission, the inertia, gravity, the hard stops).
 *
 * <p><b>The motor controller</b> runs every millisecond, on where it reads the joint ({@link
 * #deviceLoop}). It is the controller the joint's block names, running what its vendor's helper
 * would configure from the joint's plan ({@link ControllerPlan}): a TalonFX or TalonFXS runs the
 * plan's whole law on the device ({@link TalonFxEmulator}), and a SPARK runs kP on the device with
 * the feedforward the robot sends ({@link SparkEmulator}). A joint wired to nothing runs on the
 * controller its motor usually runs on: a SPARK MAX for a NEO, a TalonFX for a Kraken ({@link
 * ControllerPlan#controllers}). Whichever it is, it has a stator current limit, soft limits past
 * which it drives no further, and a neutral that brakes or coasts as the joint's control says.
 *
 * <p><b>Where it reads the joint.</b> Like a real controller, it knows only its own count: where
 * the joint was at power on, taken to be where the joint rests, plus how far its motor has turned
 * since. Homing tells it otherwise ({@link #powerOnAt}, {@link #resetPosition}). A controller that
 * reads an absolute encoder, a CANcoder or one on a SPARK's data port, is seeded from the encoder
 * instead, and where the robot's IO reads that encoder each loop beside the count, so does this
 * ({@link JointIO.JointIOInputs#absolutePosition}).
 *
 * <p><b>Its limit switches</b> are what the robot's IO for the joint is given ({@link
 * ControllerPins#of}): a switch on one of the controller's own pins, which it reports to the robot
 * with the joint ({@link JointIO.JointIOInputs#forwardLimit}), or a switch on a CANdi that a
 * TalonFX reads over CAN, which the robot reads as a sensor of its own. The controller stops on a
 * tripped switch by itself, in its millisecond loop, whatever the robot asks, a homing push
 * included. Its forward limit stops it driving the joint forward, as the template takes a
 * controller's forward to be (open question M22). Whether a switch reads tripped comes from the
 * simulated world ({@link SensorSimulation}).
 *
 * <p><b>Gravity</b> is the plant's, from the joint's block in the definition the simulator was
 * built from. It is not the motor controller's: a test can run the controller with another kG
 * ({@code PIVOT.retuned(control -> control.withKg(0))}) in a robot whose definition keeps its own,
 * and live tuning does not move it.
 *
 * <p><b>How a test can fail it</b>, as a robot's fails ({@link JointSimulation}):
 *
 * <ul>
 *   <li><b>Silent</b>, as a controller off the CAN bus is: it answers nothing and hears nothing, so
 *       it goes neutral. The robot reads it as connected, its last readings standing, for half a
 *       second, and then as silent, as a real driver reports one.
 *   <li><b>Restarted</b>, as one whose breaker reset is: it says so once, goes neutral until it is
 *       asked again, and counts from 0, or from the absolute encoder it reads. It keeps its tuning:
 *       a TalonFX's is saved on it, and a SPARK's IO tells it again.
 *   <li><b>Jammed</b>: the joint stays where it is whatever its motors do, and they stall.
 *   <li><b>A follower off the bus</b> alone: neutral, drawing nothing, the joint driven by the
 *       rest.
 *   <li><b>Its absolute encoder frozen or slipped</b> on its shaft ({@link EncoderReading}).
 * </ul>
 *
 * <p><b>What is left out.</b> The battery holds 12 V, and the supply current limit is not modeled,
 * as the drivetrain's is not ({@code ModuleIOSim}). What the robot would refuse of the joint's
 * settings, its configuration refused before the simulator started, and what it would doubt, the
 * robot says at startup, in the simulator as on the robot ({@link ControllerSettings#findings}).
 */
final class JointIOSim implements JointIO {
  /**
   * A motor controller runs its closed loop every millisecond: a TalonFX's (CTRE's Phoenix 6
   * documentation: 1 kHz), and a SPARK's.
   */
  static final double DEVICE_LOOP_SECONDS = 0.001;

  /**
   * How far short of a whole millisecond the time owed may be for a device loop to run, in seconds.
   * Robot loops add up to whole milliseconds only to within rounding, and without this a device
   * loop would now and then run a robot loop late.
   */
  private static final double DEVICE_LOOP_SLACK_SECONDS = 1e-12;

  /** What the robot last asked of the motor controller. */
  private enum Asked {
    /** Nothing: the motors brake or coast, as the joint's control says. */
    NEUTRAL,
    /** To hold a point of a profile, on its own position loop. */
    POSITION,
    /** To run at a speed, on its own velocity loop. */
    VELOCITY,
    /** A voltage, within its soft limits. */
    VOLTAGE,
    /** A share of the battery's voltage, within its soft limits. */
    DUTY_CYCLE,
    /**
     * A voltage for homing, which ignores its soft limits: they are in the wrong place until the
     * joint is homed.
     */
    HOMING_PUSH
  }

  /**
   * An absolute encoder the motor controller reads, as the robot's definition has it, with what
   * depends only on the definition worked out once: looking its ratio up and working its window out
   * both make objects, and it is read every loop, or every millisecond where the controller's loop
   * runs on it.
   *
   * @param turnsPerJointTurn how many turns it makes for one turn of the joint, or one meter of it
   * @param reader turns its reading into where the joint is, inside the window it tells apart
   */
  private record EncoderRead(double turnsPerJointTurn, AbsoluteReader reader) {}

  private final Joint joint;
  private final RobotMath math;
  private final JointPlant plant;
  private final ControllerEmulator controller;

  /**
   * The joint's control as its motor controller was configured at startup. Its soft limits and its
   * neutral mode are set then and never again.
   */
  private final MechanismControl atStartup;

  /**
   * How its motor controller drives it: its block's control, or the pit's gains and current limits
   * with the soft limits and neutral mode it started with.
   */
  private ControllerPlan plan;

  private Asked asked = Asked.NEUTRAL;

  /** The volts last asked for, as a voltage or as a homing push. */
  private double askedVolts;

  /** The share of the battery's voltage last asked for, from -1 to 1. */
  private double askedDutyCycle;

  /**
   * Where the motor controller reads the joint by its motor's count, less where the joint really
   * is. Zero while the count is right. It is not zero for a joint powered on away from where it
   * rests, until homing tells the controller where the joint is.
   */
  private double countOffset;

  /** Where, and how fast, the motor controller last told the robot the joint is. */
  private double lastReadPosition;

  private double lastReadVelocity;

  /** Seconds of the robot's loop not yet run, always under a millisecond: the next step's. */
  private double owedSeconds;

  /** Whether it is off the CAN bus: it answers nothing and hears nothing. */
  private boolean silent;

  /** Whether it restarted since the robot last read it. */
  private boolean restarted;

  // What is on its two limit inputs, as the robot's IO for the joint is told (ControllerPins), and
  // whether the switch on each reads tripped, as the simulated world has it. Until the world's
  // sensors are simulated (readsLimits), no switch ever reads tripped.
  private final boolean stopsOnForwardLimit;
  private final boolean stopsOnReverseLimit;
  private final boolean reportsForwardLimit;
  private final boolean reportsReverseLimit;
  private BooleanSupplier forwardTripped = () -> false;
  private BooleanSupplier reverseTripped = () -> false;

  /**
   * The absolute encoder the robot's IO reads each loop beside the motor's count: a CANcoder that
   * its TalonFX reads its rotor beside, or a SPARK's data port. Empty where the IO reads none so.
   */
  private final Optional<EncoderRead> readBesideCount;

  /**
   * The absolute encoder the motor controller is seeded from, as a real one is at power on and
   * after it restarts. Empty where the controller reads no absolute encoder.
   */
  private final Optional<EncoderRead> seedsFrom;

  /**
   * Whether the motor controller's own loop runs on the absolute encoder it reads, and not on its
   * motor's count: a TalonFX's on its CANcoder (remote, fused or sync), or a SPARK's on its data
   * port. Where it reads the joint is then where the encoder says the joint is.
   */
  private final boolean loopRunsOnEncoder;

  /** What the absolute encoder the controller reads says, as a test has failed it. */
  private final EncoderReading encoder = new EncoderReading();

  /**
   * How many devices it reports faults for, as its block wires them: its motor controllers, and its
   * CANcoder if it has one. None of them ever faults here.
   */
  private final int deviceCount;

  /** How many motor controllers follow its leader. */
  private final int followerCount;

  /** Which of its followers are off the CAN bus: neutral, with the joint driven without them. */
  private final boolean[] followerSilent;

  /**
   * How long a controller may go without answering before the robot is told it is silent, as the
   * real drivers have it: one missed frame is a glitch, not a lost device.
   */
  private static final double SILENT_AFTER_SECONDS = 0.5;

  /**
   * Whether the leader, and each follower, answered within the last half second. They are kept in
   * whole nanoseconds on the robot's clock, as the real drivers keep theirs, so the simulator calls
   * a controller silent in the same loop a robot would.
   */
  private final Debounce leaderAnswers = silentAfterHalfASecond();

  private final Debounce[] followerAnswers;

  /** How many loops it has been read in, to read its followers' health every few. */
  private int loops;

  /**
   * @param math what the robot's definition derives: its joint's block sets the gravity on it
   * @param joint the joint as its motor controller runs it: its block's control and wiring
   */
  JointIOSim(RobotMath math, Joint joint) {
    this(math, joint, ControllerPins.NONE);
  }

  /**
   * The same, with what's on its motor controller's limits.
   *
   * @param pins its limits, from the robot's sensor wiring ({@link ControllerPins#of})
   */
  JointIOSim(RobotMath math, Joint joint, ControllerPins pins) {
    this.joint = joint;
    this.math = math;
    atStartup = ControllerSettings.control(joint);
    plan = ControllerPlan.of(math, joint, atStartup);
    plant = JointPlant.of(math, joint);
    controller = ControllerEmulator.of(plan);
    lastReadPosition = plant.position();

    stopsOnForwardLimit = pins.forward().map(ControllerPins.Limit::stops).orElse(false);
    stopsOnReverseLimit = pins.reverse().map(ControllerPins.Limit::stops).orElse(false);
    reportsForwardLimit = pins.forward().filter(ControllerPins.OnPin.class::isInstance).isPresent();
    reportsReverseLimit = pins.reverse().filter(ControllerPins.OnPin.class::isInstance).isPresent();

    Optional<Driven.AbsoluteEncoder> wired = joint.driven().absoluteEncoder();
    Optional<EncoderRead> model =
        Optional.ofNullable(math.absoluteEncoder(joint))
            .map(
                encoder ->
                    new EncoderRead(Math.abs(encoder.ratio(joint)), math.absoluteReader(encoder)));
    readBesideCount = wired.filter(JointIOSim::isReadBesideCount).flatMap(encoder -> model);
    seedsFrom = wired.filter(Driven.AbsoluteEncoder::readByController).flatMap(encoder -> model);
    loopRunsOnEncoder = wired.map(Driven.AbsoluteEncoder::runsControllerLoop).orElse(false);

    int controllerCount =
        joint.driven().motors().map(motors -> motors.controllers().size()).orElse(0);
    followerCount = Math.max(0, controllerCount - 1);
    followerSilent = new boolean[followerCount];
    followerAnswers = new Debounce[followerCount];
    for (int i = 0; i < followerCount; i++) {
      followerAnswers[i] = silentAfterHalfASecond();
    }
    boolean hasCancoder = wired.flatMap(Driven.AbsoluteEncoder::cancoder).isPresent();
    deviceCount = hasCancoder ? controllerCount + 1 : controllerCount;
  }

  private static Debounce silentAfterHalfASecond() {
    return new Debounce(SILENT_AFTER_SECONDS, DebounceType.FALLING);
  }

  /**
   * Whether the robot's IO reads an absolute encoder each loop beside the motor's count: one on a
   * SPARK's data port, or a CANcoder that a TalonFX reads its own rotor beside.
   */
  private static boolean isReadBesideCount(Driven.AbsoluteEncoder encoder) {
    if (encoder.dataPort().isPresent()) {
      return true;
    }
    return encoder
        .cancoder()
        .filter(cancoder -> cancoder.feedback() == MechanismWiring.Feedback.ROTOR)
        .isPresent();
  }

  /** The joint. */
  Joint joint() {
    return joint;
  }

  /**
   * Reads the switches on its limits from now on, as the simulated world has them: whether each is
   * tripped.
   */
  void readsLimits(BooleanSupplier forward, BooleanSupplier reverse) {
    forwardTripped = forward;
    reverseTripped = reverse;
  }

  /**
   * Gravity's torque on the joint, positive turning it forward, with the joints where they stand:
   * newton meters, or newtons for a slide ({@link JointPlant#gravityTorque}).
   */
  double gravityTorque(Map<Joint, Double> positions) {
    return plant.gravityTorque(positions);
  }

  /**
   * The current each of its motors draws from the battery, in amps: what its controller reads, and
   * what its PDH channel does.
   */
  double supplyAmps() {
    return plant.supplyAmps();
  }

  /**
   * The current one of its motor controllers draws from the battery, in amps: what its PDH channel
   * reads. Each that drives the joint draws alike, and a follower off the bus draws nothing.
   *
   * @param controller which, from 0 for the leader
   */
  double supplyAmps(int controller) {
    boolean isAFollower = controller > 0 && controller <= followerCount;
    if (isAFollower && followerSilent[controller - 1]) {
      return 0;
    }
    return plant.supplyAmps();
  }

  /** Where the joint really is, in its units. */
  double position() {
    return plant.position();
  }

  /** How fast it really moves. */
  double velocity() {
    return plant.velocity();
  }

  /** The voltage across its motors now: what its motor controller puts there. */
  double appliedVolts() {
    return plant.appliedVolts();
  }

  /**
   * How its motor controller drives it now: its block's control, or the pit's last gains and
   * current limits with the soft limits and neutral mode it started with.
   */
  ControllerPlan plan() {
    return plan;
  }

  /** The absolute encoder its motor controller reads, for a test to freeze or slip. */
  EncoderReading encoder() {
    return encoder;
  }

  /** Silences it, or brings it back: off the CAN bus, it answers nothing and goes neutral. */
  void silence(boolean silent) {
    this.silent = silent;
    if (silent) {
      asked = Asked.NEUTRAL;
    }
  }

  /**
   * Takes one of its followers off the CAN bus, or brings it back: it answers nothing and hears
   * nothing, so it goes neutral, drawing nothing from the battery, and its leader drives the joint
   * without it.
   *
   * @param follower which, from 0 for the first after the leader
   * @throws IllegalArgumentException for a follower it hasn't
   */
  void silenceFollower(int follower, boolean silent) {
    if (follower < 0 || follower >= followerCount) {
      throw new IllegalArgumentException(
          joint.label() + " has " + followerCount + " followers, not one numbered " + follower);
    }
    followerSilent[follower] = silent;
    int stillDriving = 1; // the leader
    for (boolean eachSilent : followerSilent) {
      if (!eachSilent) {
        stillDriving++;
      }
    }
    plant.drivenBy(stillDriving);
  }

  /**
   * Jams it where it is, or frees it: stuck still, as a game piece wedged in it or a bent part
   * leaves it, whatever its motors do, which stall against it as against a hard stop.
   */
  void jam(boolean jammed) {
    plant.jam(jammed);
  }

  /**
   * Restarts it. It goes neutral until it is asked again, and it says it restarted the next time it
   * is read. Its count starts again: from 0, or, for a controller that reads an absolute encoder,
   * from where that encoder says the joint is, as a real one is seeded.
   */
  void restart() {
    asked = Asked.NEUTRAL;
    restarted = true;
    double countsFrom = 0;
    if (seedsFrom.isPresent()) {
      countsFrom = withinTheEncodersWindow(seedsFrom.get());
    }
    countOffset = countsFrom - plant.position();
  }

  /** Puts the joint somewhere, still, where its motor controller reads it too: a test's start. */
  void place(double position) {
    plant.place(position);
    countOffset = 0;
  }

  /**
   * Carries the joint somewhere, moving at a speed when it gets there, as a hand moving it would.
   * Its motor turns with it, so its motor controller's count moves as far as the joint does: a
   * count that was off stays as far off. A test's, of what the controller reads as the joint moves.
   */
  void carry(double position, double velocity) {
    plant.place(position, velocity);
  }

  /**
   * Powers it on somewhere other than where it rests, still. Its motor controller reads it where it
   * rests anyway, as a joint read by its motor alone does, until it is homed. A controller that
   * reads an absolute encoder is seeded from it instead.
   */
  void powerOnAt(double position) {
    plant.place(position);
    double countsFrom = joint.restPosition();
    if (seedsFrom.isPresent()) {
      countsFrom = withinTheEncodersWindow(seedsFrom.get());
    }
    countOffset = countsFrom - position;
  }

  /**
   * Runs the motor controller and the joint for a time, a millisecond at a time as the motor
   * controller does. Time short of a whole millisecond is kept for the next step.
   *
   * @param gravityTorque gravity's torque on the joint through the step, positive turning it
   *     forward: newton meters, or newtons for a slide
   */
  void step(double seconds, double gravityTorque) {
    owedSeconds += seconds;
    while (owedSeconds >= DEVICE_LOOP_SECONDS - DEVICE_LOOP_SLACK_SECONDS) {
      owedSeconds -= DEVICE_LOOP_SECONDS;
      deviceLoop(gravityTorque);
    }
  }

  /**
   * One millisecond: the motor controller reads the joint and decides what to put across its
   * motors, and the joint moves under that for the millisecond, held at its stops.
   */
  private void deviceLoop(double gravityTorque) {
    double position = controllerReads();
    double velocity = plant.velocity();
    double statorLimitAmps = plan.currentLimits().stator().in(Amps);
    if (asked == Asked.NEUTRAL) {
      runInNeutral(statorLimitAmps, gravityTorque);
      return;
    }
    double loopVolts = loopVolts(position, velocity);
    double volts = Math.clamp(loopVolts, -Motor.NOMINAL_VOLTS, Motor.NOMINAL_VOLTS);
    boolean obeysSoftLimits = asked != Asked.HOMING_PUSH;
    if (obeysSoftLimits && drivesPastASoftLimit(position, volts)) {
      volts = 0;
    }
    // A limit switch stops everything, a homing push included.
    if (drivesIntoATrippedLimitSwitch(volts)) {
      volts = 0;
    }
    plant.run(volts, statorLimitAmps, gravityTorque, DEVICE_LOOP_SECONDS);
  }

  /**
   * A millisecond in neutral. A controller set to brake shorts its motors, which is 0 V across
   * them: the back EMF drives a current that slows the joint. One set to coast opens them, so no
   * current flows and only gravity moves the joint.
   */
  private void runInNeutral(double statorLimitAmps, double gravityTorque) {
    if (plan.brakes()) {
      plant.run(0, statorLimitAmps, gravityTorque, DEVICE_LOOP_SECONDS);
      return;
    }
    plant.coast(gravityTorque, DEVICE_LOOP_SECONDS);
  }

  /** What the motor controller's loop asks to put across the motors this millisecond. */
  private double loopVolts(double position, double velocity) {
    return switch (asked) {
      case POSITION -> controller.voltsToHold(position, velocity, DEVICE_LOOP_SECONDS);
      case VELOCITY -> controller.voltsToRun(position, velocity);
      case VOLTAGE, HOMING_PUSH -> askedVolts;
      case DUTY_CYCLE -> askedDutyCycle * Motor.NOMINAL_VOLTS;
      case NEUTRAL -> 0;
    };
  }

  /**
   * Whether it is at or past one of its soft limits and these volts would drive it further past.
   */
  private boolean drivesPastASoftLimit(double position, double volts) {
    if (plan.softLimits().isEmpty()) {
      return false;
    }
    MechanismControl.SoftLimits limits = plan.softLimits().get();
    boolean pastUpperDrivenUp = position >= limits.upper() && volts > 0;
    boolean pastLowerDrivenDown = position <= limits.lower() && volts < 0;
    return pastUpperDrivenUp || pastLowerDrivenDown;
  }

  /**
   * Whether these volts would drive it toward a limit switch that it stops on and that reads
   * tripped. Its forward limit stops positive volts, and its reverse limit negative ones.
   */
  private boolean drivesIntoATrippedLimitSwitch(double volts) {
    if (volts > 0 && stopsOnForwardLimit && forwardTripped.getAsBoolean()) {
      return true;
    }
    return volts < 0 && stopsOnReverseLimit && reverseTripped.getAsBoolean();
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    if (inputs.faults.length != deviceCount) {
      inputs.faults = new int[deviceCount];
      inputs.followersConnected = new boolean[followerCount];
      inputs.followersRestarted = new boolean[followerCount];
    }
    // A controller off the bus is called silent only after half a second, as a real driver calls
    // it: until then it reads as connected, with its last readings standing.
    inputs.connected = leaderAnswers.calculate(!silent);
    // Its readings are this loop's only while it answers, so what reads the joint through an
    // interlock holds at once, without waiting out the half second.
    inputs.fresh = !silent;
    readWhetherFollowersAnswer(inputs);
    if (silent) {
      // Off the bus, nothing new arrives: its last readings stand, as a robot's do.
      return;
    }
    inputs.restarted = restarted;
    restarted = false;
    inputs.position = controllerReads();
    inputs.motorPosition = motorPositionBesideEncoder();
    inputs.velocity = plant.velocity();
    inputs.appliedVolts = plant.appliedVolts();
    inputs.statorCurrentAmps = plant.statorAmps();
    inputs.supplyCurrentAmps = plant.supplyAmps();
    inputs.absolutePosition = absolutePositionBesideCount();
    inputs.forwardLimit = reportsForwardLimit && forwardTripped.getAsBoolean();
    inputs.reverseLimit = reportsReverseLimit && reverseTripped.getAsBoolean();
    inputs.unconfigured = "";
    lastReadPosition = inputs.position;
    lastReadVelocity = inputs.velocity;
  }

  /**
   * Reads whether each follower answers, once every {@link JointIO#HEALTH_LOOPS} loops as a real
   * driver reads its followers' health, and calls one silent after half a second of not answering.
   * A joint whose leader is off the bus has its followers off it too.
   */
  private void readWhetherFollowersAnswer(JointIOInputs inputs) {
    boolean readsHealthThisLoop = loops % HEALTH_LOOPS == 0;
    loops++;
    if (!readsHealthThisLoop) {
      return;
    }
    for (int i = 0; i < followerCount; i++) {
      boolean answers = !silent && !followerSilent[i];
      inputs.followersConnected[i] = followerAnswers[i].calculate(answers);
    }
  }

  /**
   * Where its motor controller reads the joint: where the absolute encoder its loop runs on says,
   * or else by its motor's count.
   */
  private double controllerReads() {
    if (loopRunsOnEncoder) {
      return encoder.of(plant.position());
    }
    return motorCount();
  }

  /** Where its motor's count says the joint is. */
  private double motorCount() {
    return plant.position() + countOffset;
  }

  /**
   * The motor's count, for a controller whose loop runs on an encoder that the IO also reads beside
   * the count: a SPARK closed on its data port, whose IO reads its motor's count too. The IO's
   * inputs say NaN for every other joint.
   */
  private double motorPositionBesideEncoder() {
    if (loopRunsOnEncoder && readBesideCount.isPresent()) {
      return motorCount();
    }
    return Double.NaN;
  }

  /**
   * Where the absolute encoder read beside the count says the joint is, as a real one's IO reads
   * it. The IO's inputs say NaN for a joint with no such encoder.
   */
  private double absolutePositionBesideCount() {
    if (readBesideCount.isEmpty()) {
      return Double.NaN;
    }
    return withinTheEncodersWindow(readBesideCount.get());
  }

  /**
   * Where an absolute encoder the motor controller reads says the joint is, as a real one reads it.
   * An absolute encoder knows only the share of a turn it is in, so the reading is that share, put
   * back into the joint's units inside the window of positions the encoder's one turn tells apart.
   */
  private double withinTheEncodersWindow(EncoderRead model) {
    double unitsPerTurn = joint instanceof Joint.Revolute ? 2 * Math.PI : 1;
    double encoderTurns = encoder.of(plant.position()) * model.turnsPerJointTurn() / unitsPerTurn;
    double shareOfATurn = encoderTurns - Math.floor(encoderTurns);
    return model.reader().position(shareOfATurn);
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    if (silent) {
      return;
    }
    if (asked != Asked.POSITION) {
      controller.startPositionLoopAfresh();
    }
    controller.hold(position, velocity, feedforwardVolts, lastReadVelocity);
    asked = Asked.POSITION;
  }

  @Override
  public void setVelocity(double velocity) {
    if (silent) {
      return;
    }
    controller.run(velocity, lastReadPosition);
    asked = Asked.VELOCITY;
  }

  @Override
  public void setVoltage(double volts) {
    if (silent) {
      return;
    }
    asked = Asked.VOLTAGE;
    askedVolts = volts;
  }

  @Override
  public void setDutyCycle(double share) {
    if (silent) {
      return;
    }
    asked = Asked.DUTY_CYCLE;
    askedDutyCycle = share;
  }

  @Override
  public void stop() {
    asked = Asked.NEUTRAL;
  }

  @Override
  public void home(double volts) {
    if (silent) {
      return;
    }
    asked = Asked.HOMING_PUSH;
    askedVolts = volts;
  }

  @Override
  public void resetPosition(double position) {
    if (silent) {
      return;
    }
    countOffset = position - plant.position();
  }

  /**
   * Takes the pit's tuning: its gains and current limits, as a real motor controller is sent them.
   * Its soft limits and its neutral mode stay as they were at startup, whatever the tuned control
   * says: no driver sends those again ({@code JointIO}'s J19), so a test in the simulator must not
   * see them move either.
   */
  @Override
  public void retune(MechanismControl control) {
    MechanismControl gainsAndCurrentLimits =
        MechanismControlBuilder.builder(control)
            .softLimits(atStartup.softLimits())
            .neutralMode(atStartup.neutralMode())
            .build();
    plan = ControllerPlan.of(math, joint, gainsAndCurrentLimits);
    controller.retune(plan);
  }
}

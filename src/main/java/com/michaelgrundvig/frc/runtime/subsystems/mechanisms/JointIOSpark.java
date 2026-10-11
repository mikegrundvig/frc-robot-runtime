package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.rev.RevMechanism;
import com.michaelgrundvig.frc.runtime.lib.wiring.CanPorts;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.util.Signal;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.filter.Debouncer;

/**
 * A driven joint on REV hardware: one SPARK MAX or SPARK Flex leading the joint's motors, and the
 * rest following it.
 *
 * <p><b>Its settings</b> all come from the REV helper ({@link RevMechanism}), which works them out
 * from the robot's definition, the joint's wiring and its control ({@link
 * ControllerSettings#spark}), as a TalonFX's come from the Phoenix helper. No setting is typed in
 * here. They are saved on each SPARK at startup, because a SPARK that browns out falls back to what
 * is saved on it.
 *
 * <p><b>What it asks of the leader.</b> A SPARK's loop runs only the joint's kP (and kI). A
 * position goes to that loop as the profile's point, with a feedforward in volts that the robot
 * works out each loop: the plan's own feedforward ({@code ControllerPlan#feedforward}), the volts
 * the caller adds, and the plan's kD on the difference between the profile's speed and the speed
 * last read. A speed goes the same way, without the kD term.
 *
 * <p><b>Where the joint is.</b> A SPARK counts its motor's turns from wherever it was powered on,
 * so at startup it is told where the joint is:
 *
 * <ul>
 *   <li>A joint with no absolute encoder is told it is where its definition rests it. Homing
 *       corrects that later ({@link #resetPosition}).
 *   <li>A joint with an absolute encoder on the leader's data port is told where it is from that
 *       encoder's first good reading ({@link #seedMotorOnce}). After that its position is its
 *       motor's count, and the encoder is read each loop beside it, so a slip between the two shows
 *       ({@link JointIOInputs#absolutePosition}).
 *   <li>A joint whose wiring closes the SPARK's loop on that encoder ({@code
 *       DataPortEncoder#closesLoop}) is read by the encoder every loop instead, and its positions
 *       go to the SPARK's loop in the encoder's turns ({@link RevMechanism.Settings#loopRatio}).
 *       The motor's count is reported beside it ({@link JointIOInputs#motorPosition}).
 * </ul>
 *
 * <p><b>Restarts.</b> A SPARK that loses power and comes back says so in its sticky warnings
 * ({@link Restarts}), in a frame it sends anyway. It comes back counting its motor from 0, with the
 * settings saved on it. So a restarted leader is told where the joint is again from its absolute
 * encoder, if it has one, and the joint handles the rest ({@link JointMechanism}). The pit's tuning
 * is not saved on the SPARK, since it belongs to the session, so every restarted SPARK is sent it
 * again. The leader's soft limits come back switched on, as they were saved.
 */
public final class JointIOSpark implements JointIO {
  /** At startup each SPARK is sent its settings up to this many times, until it confirms them. */
  private static final int STARTUP_TRIES = 5;

  /**
   * How long a SPARK may go without answering before it is called silent. One missed frame is a
   * glitch, not a lost device.
   */
  private static final double SILENT_AFTER_SECONDS = 0.5;

  /** A SPARK reports its motor's speed in rotations a minute. */
  private static final double SECONDS_PER_MINUTE = 60;

  /** Whether the leader's soft limits are in force. A SPARK keeps them in its own settings. */
  private enum SoftLimits {
    /** In force, as they were saved at startup. */
    ON,
    /**
     * Switched off while the joint homes. Homing pushes the joint into a hard stop, and until the
     * joint is homed its soft limits are in the wrong place.
     */
    OFF_FOR_HOMING
  }

  /**
   * Whether a SPARK has restarted since it was last asked.
   *
   * <p>A SPARK sets a sticky warning, "has reset", as it starts, and keeps it until its faults are
   * cleared. This clears it as the robot starts and again after each restart it reports. A clear
   * takes a frame or more to show, so after clearing, the warning is not believed until a frame has
   * shown it clear. Otherwise one restart would be reported for several loops.
   */
  private static final class Restarts {
    private final SparkBase spark;

    /** Whether the warning has read clear since it was last cleared. Only then is it believed. */
    private boolean seenClear;

    Restarts(SparkBase spark) {
      this.spark = spark;
      spark.clearFaults();
    }

    boolean happened() {
      Signal<SparkBase.Warnings> sticky = spark.getStickyWarnings();
      if (!sticky.isValid()) {
        return false;
      }
      boolean hasReset = sticky.get().hasReset;
      if (!seenClear) {
        seenClear = !hasReset;
        return false;
      }
      if (!hasReset) {
        return false;
      }
      spark.clearFaults();
      seenClear = false;
      return true;
    }
  }

  /**
   * One SPARK that follows the leader.
   *
   * @param answers whether it has sent its faults within the last half second
   */
  private record FollowingSpark(SparkBase spark, Restarts restarts, Debouncer answers) {}

  /**
   * The absolute encoder on the leader's data port.
   *
   * @param reading the SPARK's reading of it, in turns from 0 to 1, with the encoder's direction
   *     and offset already applied by the SPARK
   * @param model the definition's encoder that it is, which converts its reading into the joint's
   *     units
   */
  private record DataPortEncoder(SparkAbsoluteEncoder reading, Sensor.Encoder model) {}

  private final RobotMath math;
  private final SparkBase leader;
  private final List<FollowingSpark> followers;

  /** The leader's own count of its motor's turns. */
  private final RelativeEncoder motorEncoder;

  private final SparkClosedLoopController loop;

  /** The kind of SPARK the joint's motors are on: a SPARK MAX or a SPARK Flex. */
  private final MechanismWiring.Controller controller;

  /** Converts between the joint's units (radians or meters) and the mechanism's rotations. */
  private final MechanismRotations rotations;

  /** How many turns the motor makes for one rotation of the mechanism. */
  private final double rotorRatio;

  /** The SPARKs that never confirmed their settings at startup, such as "motor 32". */
  private final String unconfigured;

  private final Debouncer leaderAnswers = silentAfterHalfASecond();
  private final Restarts leaderRestarts;

  /** The REV settings for this joint under a given control: its block's, or the pit's. */
  private final Function<MechanismControl, RevMechanism.Settings> settingsFor;

  /** The settings in force: the block's at startup, then whatever the pit last tuned. */
  private RevMechanism.Settings settings;

  /** The control those settings came from. A restarted SPARK is sent its tuning again. */
  private MechanismControl tuned;

  /** Where the joint was last read, and how fast, in the mechanism's rotations and per second. */
  private double positionRotations;

  private double velocityRotationsPerSecond;

  private SoftLimits softLimits = SoftLimits.ON;

  /** The absolute encoder on the leader's data port. Empty for a joint without one. */
  private final Optional<DataPortEncoder> absoluteEncoder;

  /**
   * Whether the motor's count has been told where the joint is from the absolute encoder. It is
   * told once, and once more after each time the leader restarts.
   */
  private boolean motorSeeded;

  /**
   * Whether the motor was told where the joint is during this loop, and where, in the mechanism's
   * rotations. A flag and a number rather than an Optional, because this is set and read every loop
   * and an Optional would allocate.
   */
  private boolean seededThisLoop;

  private double seededRotations;

  /** How many loops the joint has been read in, to read its SPARKs' health every few. */
  private int loops;

  /**
   * Configures every SPARK of the joint at startup, waiting for each to confirm.
   *
   * @param math what the robot's definition derives: the joint's drive, ratios and encoder
   * @param joint the joint. Its block must name its control and its motors' SPARKs.
   * @throws IllegalArgumentException for a joint whose motors aren't wired, and for what its
   *     settings refuse ({@link ControllerSettings#spark})
   */
  public JointIOSpark(RobotMath math, Joint joint) {
    this(math, joint, ControllerPins.NONE);
  }

  /**
   * The same, with what is on the leading SPARK's limit inputs.
   *
   * @param pins the switches on them, from the robot's sensor wiring
   */
  public JointIOSpark(RobotMath math, Joint joint, ControllerPins pins) {
    List<MechanismWiring.Device> wired =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    if (wired.isEmpty()) {
      throw new IllegalArgumentException(joint.label() + " has no wiring: wire it");
    }
    MechanismWiring.Device wiredLeader = wired.get(0);
    controller = wiredLeader.controller();
    this.math = math;
    settingsFor = control -> ControllerSettings.spark(math, joint, control);
    tuned = ControllerSettings.control(joint);
    settings = settingsFor.apply(tuned);
    // The definition requires a SPARK to be on one of Systemcore's own CAN ports.
    CANPort port = CanPorts.of(((CanBus.Port) wiredLeader.bus()).port());
    boolean hasDataPortEncoder =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::dataPort).isPresent();
    Optional<Sensor.Encoder> encoderModel =
        hasDataPortEncoder
            ? Optional.of(Objects.requireNonNull(math.absoluteEncoder(joint)))
            : Optional.empty();
    rotations = settings.rotations();
    rotorRatio = settings.rotorRatio();

    // From here on the SPARKs are opened and told their settings: the leader, then each follower.
    // A SPARK that never confirms is noted and runs on what was saved on it before.
    List<String> neverConfirmed = new ArrayList<>();
    leader = open(port, wiredLeader.id());
    SparkBaseConfig leaderSettings = settings.leader();
    RevMechanism.limits(leaderSettings.limitSwitch, pins);
    if (!savedAtStartup(leader, leaderSettings)) {
      neverConfirmed.add("motor " + wiredLeader.id());
    }
    List<SparkBase> following = new ArrayList<>();
    for (RevMechanism.Follower each : settings.followers()) {
      SparkBase follower = open(port, each.id());
      if (!savedAtStartup(follower, each.config())) {
        neverConfirmed.add("motor " + each.id());
      }
      following.add(follower);
    }
    unconfigured = String.join(", ", neverConfirmed);

    // Each SPARK's "has reset" warning is cleared now, the leader's and then each follower's, so
    // that what set it before the robot started is not taken for a restart.
    leaderRestarts = new Restarts(leader);
    List<Restarts> followerRestarts = new ArrayList<>();
    for (SparkBase follower : following) {
      followerRestarts.add(new Restarts(follower));
    }
    // Each follower's half second of silence is counted from here, once every SPARK has been
    // configured and cleared, and not from partway through startup.
    List<FollowingSpark> followersWithHealth = new ArrayList<>();
    for (int i = 0; i < following.size(); i++) {
      followersWithHealth.add(
          new FollowingSpark(following.get(i), followerRestarts.get(i), silentAfterHalfASecond()));
    }
    followers = List.copyOf(followersWithHealth);

    motorEncoder = leader.getEncoder();
    loop = leader.getClosedLoopController();
    absoluteEncoder =
        encoderModel.map(model -> new DataPortEncoder(leader.getAbsoluteEncoder(), model));
    // Until an absolute encoder says otherwise (seedMotorOnce) or the joint is homed, the joint is
    // taken to be where its definition rests it.
    motorEncoder.setPosition(rotations.toRotations(joint.restPosition()) * rotorRatio);
  }

  /** Opens a SPARK of the joint's kind, driving a brushless motor. */
  private SparkBase open(CANPort port, int id) {
    if (controller == MechanismWiring.Controller.SPARK_FLEX) {
      return new SparkFlex(port, id, SparkLowLevel.MotorType.kBrushless);
    }
    return new SparkMax(port, id, SparkLowLevel.MotorType.kBrushless);
  }

  /**
   * Sends a SPARK its whole configuration at startup and waits for it to confirm, trying a few
   * times. The SPARK is first reset to REV's defaults, so nothing left on it from other code
   * survives, and the result is saved on it, so a brownout falls back to these settings.
   *
   * @return whether the SPARK confirmed
   */
  private static boolean savedAtStartup(SparkBase spark, SparkBaseConfig settings) {
    return confirmed(
        () ->
            spark.configure(
                settings, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /** Whether a SPARK confirmed something it was sent at startup, after a few tries. */
  private static boolean confirmed(Supplier<REVLibError> attempt) {
    for (int i = 0; i < STARTUP_TRIES; i++) {
      if (attempt.get() == REVLibError.kOk) {
        return true;
      }
    }
    return false;
  }

  /**
   * Sends a SPARK one group of settings while the robot runs: without waiting for it to confirm,
   * leaving its other settings as they are, and without saving, since what is sent while running
   * (the pit's tuning, soft limits switched off for homing) belongs to the session.
   */
  private static void sendWhileRunning(SparkBase spark, SparkBaseConfig change) {
    spark.configureAsync(
        change, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private static Debouncer silentAfterHalfASecond() {
    return new Debouncer(SILENT_AFTER_SECONDS, Debouncer.DebounceType.FALLING);
  }

  /** The leading SPARK. For the tests, which read what it was told through REV's simulation. */
  SparkBase leader() {
    return leader;
  }

  @Override
  public void retune(MechanismControl control) {
    tuned = control;
    settings = settingsFor.apply(control);
    sendTuning(leader);
    for (int i = 0; i < followers.size(); i++) {
      sendTuning(followers.get(i).spark());
    }
  }

  /**
   * Sends a SPARK the part of the pit's tuning that it holds itself: its loop's kP and kI and its
   * current limit ({@link RevMechanism.Settings#tuning}). The rest of the tuning is in {@link
   * #settings}, which the robot works its volts out from each loop.
   */
  private void sendTuning(SparkBase spark) {
    sendWhileRunning(spark, settings.tuning(controller, tuned));
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    updateInputs(inputs, motorEncoder.getPosition(), motorEncoder.getVelocity());
  }

  /**
   * Fills the inputs from this loop's frames of the motor's encoder.
   *
   * <p>A frame that did not come reads as REV's default, 0, which is also what a joint at 0 reads.
   * So a missing frame is never taken as a reading: the last reading stands, and the inputs say it
   * is not fresh this loop ({@link JointIOInputs#fresh}). After half a second of missing frames the
   * SPARK counts as silent.
   *
   * @param motorTurns the motor's position frame, in the motor's turns
   * @param motorRpm the motor's speed frame, in the motor's turns a minute
   */
  void updateInputs(JointIOInputs inputs, Signal<Double> motorTurns, Signal<Double> motorRpm) {
    boolean leaderRestarted = leaderRestarts.happened();
    if (leaderRestarted) {
      recoverTheLeaderFromARestart();
    }
    seedMotorOnce();
    double appliedShare = leader.getAppliedOutput().get(0.0);
    double statorAmps = Math.abs(leader.getOutputCurrent().get(0.0));
    boolean motorFramesCame = motorTurns.isValid() && motorRpm.isValid();
    inputs.connected = leaderAnswers.calculate(motorFramesCame);

    double encoderRotations = readAbsoluteEncoder();
    boolean encoderRead = !Double.isNaN(encoderRotations);
    // Once the motor has been told where the joint is, a joint whose loop is closed on the absolute
    // encoder is read by that encoder.
    boolean readByEncoder = settings.closedLoop() && motorSeeded;
    boolean positionCame = takePosition(readByEncoder, encoderRotations, motorTurns);
    if (motorRpm.isValid()) {
      velocityRotationsPerSecond = motorRpm.get() / SECONDS_PER_MINUTE / rotorRatio;
    }

    inputs.fresh = motorFramesCame && positionCame;
    inputs.absolutePosition = encoderRead ? rotations.fromRotations(encoderRotations) : Double.NaN;
    // With the loop closed on the encoder, the motor's own count is reported beside it, so a slip
    // between the two shows.
    boolean reportsMotorBesideEncoder = readByEncoder && motorTurns.isValid();
    inputs.motorPosition =
        reportsMotorBesideEncoder
            ? rotations.fromRotations(motorTurns.get() / rotorRatio)
            : Double.NaN;
    inputs.position = rotations.fromRotations(positionRotations);
    inputs.velocity = rotations.fromRotations(velocityRotationsPerSecond);
    inputs.appliedVolts = appliedShare * leader.getBusVoltage().get(0.0);
    inputs.statorCurrentAmps = statorAmps;
    // A SPARK measures the current through its motor, not what it draws from the battery. The
    // battery's share is estimated as the motor's current times the share of the battery's voltage
    // across the motor.
    inputs.supplyCurrentAmps = Math.abs(appliedShare) * statorAmps;
    inputs.forwardLimit = leader.getForwardLimitSwitch().isPressed().get(false);
    inputs.reverseLimit = leader.getReverseLimitSwitch().isPressed().get(false);
    inputs.unconfigured = unconfigured;

    inputs.restarted = leaderRestarted;
    readFollowersRestarts(inputs);

    boolean readsHealthThisLoop = loops % HEALTH_LOOPS == 0;
    loops++;
    if (readsHealthThisLoop) {
      readHealth(inputs);
    }
  }

  /**
   * Takes this loop's reading of where the joint is, from whichever sensor says so this loop. When
   * that sensor's frame did not come, the last reading stands.
   *
   * @param readByEncoder whether the joint is read by its absolute encoder, and not by its motor
   * @param encoderRotations the absolute encoder's reading, in the mechanism's rotations, or NaN
   * @param motorTurns the motor's position frame
   * @return false when the joint is read by its absolute encoder and the encoder's frame did not
   *     come. A missing motor frame is not reported here: the caller already knows of it.
   */
  private boolean takePosition(
      boolean readByEncoder, double encoderRotations, Signal<Double> motorTurns) {
    if (readByEncoder) {
      seededThisLoop = false;
      if (Double.isNaN(encoderRotations)) {
        return false;
      }
      positionRotations = encoderRotations;
      return true;
    }
    if (seededThisLoop) {
      // The motor was told where the joint is this loop, and its own reading has not caught up:
      // its next frame has not come. For this one loop the joint is where the motor was told.
      positionRotations = seededRotations;
      seededThisLoop = false;
      return true;
    }
    if (motorTurns.isValid()) {
      positionRotations = motorTurns.get() / rotorRatio;
    }
    return true;
  }

  /**
   * Reads whether each follower restarted since the last loop. A restarted follower has fallen back
   * to the settings saved on it, so it is sent the pit's tuning again.
   */
  private void readFollowersRestarts(JointIOInputs inputs) {
    if (inputs.followersRestarted.length != followers.size()) {
      inputs.followersRestarted = new boolean[followers.size()];
    }
    for (int i = 0; i < followers.size(); i++) {
      FollowingSpark follower = followers.get(i);
      boolean restarted = follower.restarts().happened();
      if (restarted) {
        sendTuning(follower.spark());
      }
      inputs.followersRestarted[i] = restarted;
    }
  }

  /**
   * Puts a restarted leader back as the robot left it. It came back counting its motor from 0, with
   * the settings saved on it at startup: its soft limits on, and no pit tuning. So the absolute
   * encoder must tell it where the joint is again, which {@link #seedMotorOnce} does next in the
   * same loop, and it is sent the pit's tuning again.
   */
  private void recoverTheLeaderFromARestart() {
    motorSeeded = false;
    softLimits = SoftLimits.ON;
    sendTuning(leader);
  }

  /**
   * Tells the motor's count where the joint is, from the absolute encoder's first good reading. It
   * happens once at startup, and once more after each restart of the leader.
   */
  private void seedMotorOnce() {
    if (motorSeeded) {
      return;
    }
    double encoderRotations = readAbsoluteEncoder();
    if (Double.isNaN(encoderRotations)) {
      return;
    }
    motorEncoder.setPosition(encoderRotations * rotorRatio);
    seededRotations = encoderRotations;
    seededThisLoop = true;
    motorSeeded = true;
  }

  /**
   * Where the absolute encoder on the data port says the joint is, in the mechanism's rotations.
   * The SPARK reads the encoder with its direction and offset applied, as a share of a turn
   * measured from the low edge of the window of positions one turn of the encoder can tell apart
   * ({@link RevMechanism.Settings#windowEdge}).
   *
   * @return the position, or NaN when there is no such encoder or its frame did not come. NaN
   *     rather than an Optional because this runs every loop, where an Optional would allocate.
   */
  private double readAbsoluteEncoder() {
    if (absoluteEncoder.isEmpty()) {
      return Double.NaN;
    }
    Signal<Double> turnsFromTheEdge = absoluteEncoder.get().reading().getPosition();
    if (!turnsFromTheEdge.isValid()) {
      return Double.NaN;
    }
    double encoderTurns = settings.windowEdge() + turnsFromTheEdge.get();
    return rotations.toRotations(math.fromAbsolute(absoluteEncoder.get().model(), encoderTurns));
  }

  /**
   * Reads each SPARK's faults, and whether each follower still answers. Faults are listed leader
   * first, then each follower in order. A SPARK whose frames did not come keeps the faults it last
   * reported.
   */
  private void readHealth(JointIOInputs inputs) {
    if (inputs.faults.length != 1 + followers.size()) {
      inputs.faults = new int[1 + followers.size()];
    }
    if (inputs.followersConnected.length != followers.size()) {
      inputs.followersConnected = new boolean[followers.size()];
    }
    readFaults(leader, inputs.faults, 0);
    for (int i = 0; i < followers.size(); i++) {
      FollowingSpark follower = followers.get(i);
      boolean answered = readFaults(follower.spark(), inputs.faults, 1 + i);
      inputs.followersConnected[i] = follower.answers().calculate(answered);
    }
  }

  /**
   * Reads one SPARK's faults and warnings into its place in the list, if both frames came. A SPARK
   * sends both anyway. They are packed into one number: the fault bits in the low sixteen bits, and
   * the warning bits in the sixteen above them.
   *
   * @return whether the SPARK answered
   */
  private static boolean readFaults(SparkBase spark, int[] faults, int index) {
    Signal<SparkBase.Faults> faultBits = spark.getFaults();
    Signal<SparkBase.Warnings> warningBits = spark.getWarnings();
    boolean answered = faultBits.isValid() && warningBits.isValid();
    if (answered) {
      faults[index] = warningBits.get().rawBits << 16 | faultBits.get().rawBits;
    }
    return answered;
  }

  /**
   * A position in the joint's units as the SPARK's loop takes it: in turns of the sensor the loop
   * runs on, the motor or the absolute encoder, counted from where that sensor reads zero.
   */
  private double loopTurns(double position) {
    return rotations.toRotations(position) * settings.loopRatio() + settings.loopZero();
  }

  /**
   * Sends the profile's point to the SPARK's position loop, which runs its kP on it. The rest of
   * the control law is worked out here, in volts, and sent with it as the loop's arbitrary
   * feedforward: the caller's volts, the plan's feedforward at this point, and the plan's kD on how
   * far the speed last read is from the profile's.
   */
  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    ControllerPlan plan = settings.plan();
    double speedLastRead = rotations.fromRotations(velocityRotationsPerSecond);
    double volts =
        feedforwardVolts
            + plan.feedforward(position, velocity, 0)
            + plan.kD() * (velocity - speedLastRead);
    loop.setSetpoint(
        loopTurns(position),
        SparkLowLevel.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        volts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  /**
   * Sends a speed to the SPARK's velocity loop, in turns a minute of the sensor its loop runs on,
   * with the plan's feedforward for that speed at where the joint was last read.
   */
  @Override
  public void setVelocity(double velocity) {
    double positionLastRead = rotations.fromRotations(positionRotations);
    loop.setSetpoint(
        rotations.toRotations(velocity) * settings.loopRatio() * SECONDS_PER_MINUTE,
        SparkLowLevel.ControlType.kVelocity,
        ClosedLoopSlot.kSlot1,
        settings.plan().feedforward(positionLastRead, velocity, 0),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setDutyCycle(double share) {
    leader.setThrottle(share);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  /**
   * A voltage that ignores the soft limits. A SPARK has no request that ignores them, so they are
   * switched off in its settings first, once, and stay off until the joint is told where it is
   * ({@link #resetPosition}).
   */
  @Override
  public void home(double volts) {
    boolean hasSoftLimits = settings.softLimits().isPresent();
    if (hasSoftLimits && softLimits == SoftLimits.ON) {
      sendWhileRunning(leader, settings.softLimitsEnabled(controller, false));
      softLimits = SoftLimits.OFF_FOR_HOMING;
    }
    leader.setVoltage(volts);
  }

  /**
   * Tells the motor's count where the joint is, and switches the soft limits back on if homing
   * switched them off.
   */
  @Override
  public void resetPosition(double position) {
    motorEncoder.setPosition(rotations.toRotations(position) * rotorRatio);
    if (softLimits == SoftLimits.OFF_FOR_HOMING) {
      sendWhileRunning(leader, settings.softLimitsEnabled(controller, true));
      softLimits = SoftLimits.ON;
    }
  }
}

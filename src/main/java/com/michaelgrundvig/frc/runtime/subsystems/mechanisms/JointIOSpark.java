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
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.filter.Debouncer;

/**
 * A driven joint on REV hardware: a SPARK MAX or SPARK Flex leading its drive's motors, the rest
 * following it, read by its motor's own encoder. Every setting comes from the REV helper ({@link
 * RevMechanism}), from the robot's definition, the joint's wiring and its control ({@link
 * ControllerSettings#spark}), as a TalonFX's come from the Phoenix helper: nothing is retyped here.
 *
 * <p>Positions go to the SPARK's loop with the profile's point and the feedforward the robot works
 * out in volts, the plan's law and its kD ({@code ControllerPlan#feedforward}); speeds the same
 * way. A joint is read by its rotor: one with an absolute encoder on the SPARK's data port is told
 * where it is from that encoder's first good reading; one without takes where the joint rests as
 * where it is at power on, until it homes. One whose loop closes on that encoder is read by it
 * instead, each loop, and its positions go to the SPARK's loop in the encoder's turns ({@link
 * RevMechanism.Settings#loopRatio}). Its settings are kept through a brownout: REV's controllers
 * fall back to what's saved on them.
 *
 * <p>Each SPARK's restart is read from its sticky warnings ({@link Restarts}), from a frame it
 * sends anyway. A leader that restarts counts its motor from 0: one told where it is by the encoder
 * on its data port is told again, at once; the joint does the rest ({@link JointMechanism}). A
 * restarted SPARK falls back to what's saved on it, so each is told the pit's tuning again, which
 * isn't saved, and the leader's soft limits are back on.
 */
public final class JointIOSpark implements JointIO {
  /** At startup, configuring a device is tried this many times. */
  private static final int TRIES = 5;

  private final SparkBase leader;
  private final List<SparkBase> followers = new ArrayList<>();
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController loop;

  /** The kind of SPARK its motors are on: a SPARK MAX or a SPARK Flex. */
  private final MechanismWiring.Controller controller;

  private final MechanismRotations rotations;
  private final double rotorRatio;
  private final String unconfigured;

  /** A controller counts as silent once it has been for half a second, not at one glitch. */
  private final Debouncer connected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  /** The REV settings for a control: the joint's, its drive's and its block's, with it. */
  private final Function<MechanismControl, RevMechanism.Settings> configure;

  private RevMechanism.Settings settings;

  /** Where the joint is and how fast it moves, last read, in the mechanism's rotations. */
  private double position;

  private double velocity;

  /** Whether its soft limits are switched off, while it homes past them. */
  private boolean softLimitsOff;

  /** What the robot's definition derives: where the joint is from its absolute encoder. */
  private final RobotMath math;

  /** The absolute encoder on the leader's data port, and the definition's, if it has one. */
  private final @Nullable SparkAbsoluteEncoder absolute;

  private final Sensor.@Nullable Encoder absoluteModel;

  /** Whether the motor's encoder has been told where the joint is, from the absolute encoder. */
  private boolean seeded;

  /** Its control as the pit last tuned it: what a restarted SPARK is told again. */
  private MechanismControl tuned;

  /** Whether each SPARK restarted: the leader, then each follower in order. */
  private final Restarts leaderRestarts;

  private final Restarts[] followerRestarts;

  /**
   * Whether each follower answers, from its frames of faults: half a second without, it doesn't.
   */
  private final Debouncer[] followersAnswer;

  /** The loops it's been read in. */
  private int loops;

  /**
   * Whether a SPARK restarted since it was last asked, from its sticky warning that it reset, which
   * a SPARK sets as it starts and keeps until it's cleared. Cleared as the robot starts and after
   * each restart, the warning is believed only once a frame has read it clear again, since a clear
   * shows a frame or more late.
   */
  private static final class Restarts {
    private final SparkBase spark;

    /** Whether its warning has read clear since it was last cleared: then it's believed. */
    private boolean armed;

    Restarts(SparkBase spark) {
      this.spark = spark;
      spark.clearFaults();
    }

    boolean happened() {
      Signal<SparkBase.Warnings> sticky = spark.getStickyWarnings();
      if (!sticky.isValid()) {
        return false;
      }
      boolean reset = sticky.get().hasReset;
      if (!armed) {
        armed = !reset;
        return false;
      }
      if (!reset) {
        return false;
      }
      spark.clearFaults();
      armed = false;
      return true;
    }
  }

  /**
   * Configures every device of the joint, waiting for each, at startup.
   *
   * @param math what the robot's definition derives: the joint's drive, ratios and seeds
   * @param joint the joint, its block naming its control and its motors' SPARKs, which it must
   * @throws IllegalArgumentException for a joint whose motors aren't wired, and what its settings
   *     refuse ({@link ControllerSettings#spark})
   */
  public JointIOSpark(RobotMath math, Joint joint) {
    this(math, joint, ControllerPins.NONE);
  }

  /**
   * The same, with what's on its leading SPARK's limit inputs.
   *
   * @param pins its switches there, from the robot's sensor wiring
   */
  public JointIOSpark(RobotMath math, Joint joint, ControllerPins pins) {
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    if (controllers.isEmpty()) {
      throw new IllegalArgumentException(joint.label() + " has no wiring: wire it");
    }
    MechanismWiring.Device wiredLeader = controllers.get(0);
    controller = wiredLeader.controller();
    this.math = math;
    configure = control -> ControllerSettings.spark(math, joint, control);
    tuned = ControllerSettings.control(joint);
    settings = configure.apply(tuned);
    // On Systemcore's own CAN, as the definition requires of a SPARK, and the definition's encoder
    // on the data port, which the settings found.
    CanBus.Port bus = (CanBus.Port) wiredLeader.bus();
    Sensor.Encoder model =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::dataPort).isPresent()
            ? math.absoluteEncoder(joint)
            : null;
    absoluteModel = model;
    rotations = settings.rotations();
    rotorRatio = settings.rotorRatio();
    List<String> failed = new ArrayList<>();
    leader = spark(CanPorts.of(bus.port()), wiredLeader.id());
    SparkBaseConfig leading = settings.leader();
    RevMechanism.limits(leading.limitSwitch, pins);
    if (!applied(
        () ->
            leader.configure(
                leading, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))) {
      failed.add("motor " + wiredLeader.id());
    }
    for (RevMechanism.Follower each : settings.followers()) {
      SparkBase follower = spark(CanPorts.of(bus.port()), each.id());
      if (!applied(
          () ->
              follower.configure(
                  each.config(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))) {
        failed.add("motor " + each.id());
      }
      followers.add(follower);
    }
    unconfigured = String.join(", ", failed);
    leaderRestarts = new Restarts(leader);
    followerRestarts = followers.stream().map(Restarts::new).toArray(Restarts[]::new);
    followersAnswer = new Debouncer[followers.size()];
    for (int i = 0; i < followersAnswer.length; i++) {
      followersAnswer[i] = new Debouncer(0.5, Debouncer.DebounceType.FALLING);
    }
    encoder = leader.getEncoder();
    loop = leader.getClosedLoopController();
    absolute = model != null ? leader.getAbsoluteEncoder() : null;
    encoder.setPosition(rotations.toRotations(joint.restPosition()) * rotorRatio);
  }

  /** The leading SPARK: for the tests, which read what it was told through REV's simulation. */
  SparkBase leader() {
    return leader;
  }

  private SparkBase spark(CANPort port, int id) {
    return controller == MechanismWiring.Controller.SPARK_FLEX
        ? new SparkFlex(port, id, SparkLowLevel.MotorType.kBrushless)
        : new SparkMax(port, id, SparkLowLevel.MotorType.kBrushless);
  }

  /** Tries something a device must confirm, a few times: at startup. Whether it took. */
  private static boolean applied(Supplier<REVLibError> attempt) {
    for (int i = 0; i < TRIES; i++) {
      if (attempt.get() == REVLibError.kOk) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void retune(MechanismControl control) {
    tuned = control;
    settings = configure.apply(control);
    tune(leader);
    for (SparkBase follower : followers) {
      tune(follower);
    }
  }

  /**
   * Tells a SPARK the pit's tuning: without waiting, as the loop runs, and not saved, since the
   * pit's tuning is the session's.
   */
  private void tune(SparkBase spark) {
    spark.configureAsync(
        settings.tuning(controller, tuned),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    updateInputs(inputs, encoder.getPosition(), encoder.getVelocity());
  }

  /**
   * The inputs from its motor's encoder's frames this loop, its position and its speed. A frame
   * that didn't come reads as REV's default, 0, which a joint at 0 would read too: its last reading
   * stands instead, and the joint's reading isn't fresh this loop; half a second of them, its SPARK
   * is silent.
   */
  void updateInputs(JointIOInputs inputs, Signal<Double> rotor, Signal<Double> rpm) {
    boolean restarted = leaderRestarts.happened();
    if (restarted) {
      // It counts its motor from 0, on what's saved on it: an encoder on its data port seeds it
      // again, now, and it's told the pit's tuning again; its soft limits are back on.
      seeded = false;
      softLimitsOff = false;
      tune(leader);
    }
    seed();
    double share = leader.getAppliedOutput().get(0.0);
    double current = Math.abs(leader.getOutputCurrent().get(0.0));
    boolean fresh = rotor.isValid() && rpm.isValid();
    inputs.connected = connected.calculate(fresh);
    double fromEncoder = fromAbsolute();
    if (settings.closedLoop() && seeded) {
      // Its loop runs on its absolute encoder, whose word is where the joint is, once it's given
      // one: a frame of it that didn't come leaves its last.
      if (Double.isNaN(fromEncoder)) {
        fresh = false;
      } else {
        position = fromEncoder;
      }
      seededAt = Double.NaN;
    } else if (!Double.isNaN(seededAt)) {
      // Told where it is this loop: its encoder's reading catches up on the next.
      position = seededAt;
      seededAt = Double.NaN;
    } else if (rotor.isValid()) {
      position = rotor.get() / rotorRatio;
    }
    if (rpm.isValid()) {
      velocity = rpm.get() / 60 / rotorRatio;
    }
    inputs.fresh = fresh;
    inputs.absolutePosition =
        Double.isNaN(fromEncoder) ? Double.NaN : rotations.fromRotations(fromEncoder);
    // Its loop closed on the encoder, its motor's own count beside it, so a slip between shows.
    inputs.motorPosition =
        settings.closedLoop() && seeded && rotor.isValid()
            ? rotations.fromRotations(rotor.get() / rotorRatio)
            : Double.NaN;
    inputs.position = rotations.fromRotations(position);
    inputs.velocity = rotations.fromRotations(velocity);
    inputs.appliedVolts = share * leader.getBusVoltage().get(0.0);
    inputs.statorCurrentAmps = current;
    // A SPARK doesn't measure what it draws from the battery: its share of the motor's current.
    inputs.supplyCurrentAmps = Math.abs(share) * current;
    inputs.forwardLimit = leader.getForwardLimitSwitch().isPressed().get(false);
    inputs.reverseLimit = leader.getReverseLimitSwitch().isPressed().get(false);
    inputs.unconfigured = unconfigured;
    inputs.restarted = restarted;
    if (inputs.followersRestarted.length != followerRestarts.length) {
      inputs.followersRestarted = new boolean[followerRestarts.length];
    }
    for (int i = 0; i < followerRestarts.length; i++) {
      boolean follower = followerRestarts[i].happened();
      if (follower) {
        tune(followers.get(i));
      }
      inputs.followersRestarted[i] = follower;
    }
    if (loops++ % HEALTH_LOOPS == 0) {
      health(inputs);
    }
  }

  /**
   * Its SPARKs' health, the leader's and each follower's: its faults, and its warnings above them,
   * from the frames each sends anyway, and each follower's answering; the last reading standing
   * where none came.
   */
  private void health(JointIOInputs inputs) {
    if (inputs.faults.length != 1 + followers.size()) {
      inputs.faults = new int[1 + followers.size()];
    }
    if (inputs.followersConnected.length != followers.size()) {
      inputs.followersConnected = new boolean[followers.size()];
    }
    for (int i = 0; i <= followers.size(); i++) {
      SparkBase spark = i == 0 ? leader : followers.get(i - 1);
      Signal<SparkBase.Faults> faults = spark.getFaults();
      Signal<SparkBase.Warnings> warnings = spark.getWarnings();
      boolean answered = faults.isValid() && warnings.isValid();
      if (answered) {
        inputs.faults[i] = warnings.get().rawBits << 16 | faults.get().rawBits;
      }
      if (i > 0) {
        inputs.followersConnected[i - 1] = followersAnswer[i - 1].calculate(answered);
      }
    }
  }

  /** Where it was seeded this loop, in the mechanism's rotations: NaN otherwise. */
  private double seededAt = Double.NaN;

  /**
   * Tells the motor's encoder where the joint is, from the absolute encoder's first good reading:
   * once, at startup.
   */
  private void seed() {
    double at = seeded ? Double.NaN : fromAbsolute();
    if (Double.isNaN(at)) {
      return;
    }
    encoder.setPosition(at * rotorRatio);
    seededAt = at;
    seeded = true;
  }

  /**
   * Where the absolute encoder on its data port says the joint is, in the mechanism's rotations:
   * NaN without one, or before its first good reading. The SPARK reads it with its direction and
   * offset applied, from the window's low edge ({@link RevMechanism.Settings#windowEdge}).
   */
  private double fromAbsolute() {
    SparkAbsoluteEncoder reading = absolute;
    Sensor.Encoder model = absoluteModel;
    if (reading == null || model == null) {
      return Double.NaN;
    }
    Signal<Double> fromEdge = reading.getPosition();
    return fromEdge.isValid()
        ? rotations.toRotations(math.fromAbsolute(model, settings.windowEdge() + fromEdge.get()))
        : Double.NaN;
  }

  /**
   * The profile's point to the SPARK's loop, which runs its kP on it, with the plan's feedforward
   * and kD toward the profile's speed from the speed last read, worked out here in volts.
   */
  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    ControllerPlan plan = settings.plan();
    loop.setSetpoint(
        rotations.toRotations(position) * settings.loopRatio() + settings.loopZero(),
        SparkLowLevel.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts
            + plan.feedforward(position, velocity, 0)
            + plan.kD() * (velocity - rotations.fromRotations(this.velocity)),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  /** A speed to the SPARK's loop, with the plan's feedforward at where the joint was last read. */
  @Override
  public void setVelocity(double velocity) {
    loop.setSetpoint(
        rotations.toRotations(velocity) * settings.loopRatio() * 60,
        SparkLowLevel.ControlType.kVelocity,
        ClosedLoopSlot.kSlot1,
        settings.plan().feedforward(rotations.fromRotations(position), velocity, 0),
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

  /** Switches its soft limits off first: a SPARK keeps them in its own settings. */
  @Override
  public void home(double volts) {
    if (settings.softLimits().isPresent() && !softLimitsOff) {
      leader.configureAsync(
          settings.softLimitsEnabled(controller, false),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      softLimitsOff = true;
    }
    leader.setVoltage(volts);
  }

  /** Sets where the joint is, and its soft limits back on if homing switched them off. */
  @Override
  public void resetPosition(double position) {
    encoder.setPosition(rotations.toRotations(position) * rotorRatio);
    if (softLimitsOff) {
      leader.configureAsync(
          settings.softLimitsEnabled(controller, true),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      softLimitsOff = false;
    }
  }
}

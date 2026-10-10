package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Angle;

/**
 * A driven joint on CTRE hardware: a TalonFX, or a TalonFXS as its wiring says, leading its drive's
 * motors, the rest following it, and its CANcoder, if its wiring names one. A TalonFXS takes the
 * settings a TalonFX would, in its own terms ({@link PhoenixMechanism#talonFxs}). Every setting
 * comes from the Phoenix helper ({@link PhoenixMechanism}), from the robot's definition, the
 * joint's wiring and its control ({@link ControllerSettings#talon}): ratios, gains seeded from the
 * definition, gravity, soft limits, current limits, directions and followers. Nothing is retyped
 * here.
 *
 * <p>Runs on Phoenix 6 without a Pro license, unless the definition's motors are its FOC ones
 * ({@code Motor.KRAKEN_X60_FOC}): only then do its requests use FOC. Positions go to the motor
 * controller as {@code PositionVoltage} with the profile's velocity, which its own kV, kS and
 * gravity turn into volts; speeds as {@code VelocityVoltage}. A joint read by its rotor is told
 * where it is from its CANcoder's first good reading, when its wiring reads one that way ({@code
 * Feedback.ROTOR}), and reads the CANcoder each loop beside its rotor, so a slip between them
 * shows; with no CANcoder, it takes where the joint rests as where it is at power on. It isn't
 * seeded from a CANcoder that says its magnet is too close or too far, and says so ({@link
 * JointIOInputs#magnetBad}). The CANcoder sends both its reading and its magnet's health anyway, at
 * its own rates: reading them adds nothing on the bus.
 *
 * <p>Each device's restart is read from the frame Phoenix says it sends as it starts ({@code
 * ParentDevice.getResetOccurredChecker}), once a loop, with no traffic of its own. A leader that
 * restarts counts its rotor from 0: one told where it is by its CANcoder is told again, at once;
 * the joint does the rest ({@link JointMechanism}). Phoenix keeps a TalonFX's settings through a
 * restart, a retune's included, and sends the followers their request again on its own.
 */
public final class JointIOTalon implements JointIO {
  /** At startup, configuring a device is tried this many times, waiting this long each time. */
  private static final int TRIES = 5;

  private static final double STARTUP_TIMEOUT_SECONDS = 0.25;

  private final Talon leader;
  private final List<Talon> followers = new ArrayList<>();
  private final MechanismRotations rotations;
  private final BaseStatusSignal[] signals;
  private final BaseStatusSignal position;
  private final BaseStatusSignal velocity;
  private final BaseStatusSignal volts;
  private final BaseStatusSignal statorCurrent;
  private final BaseStatusSignal supplyCurrent;

  /** What's on the leader's limits: a switch on a pin, whose polarity its reading takes. */
  private final ControllerPins pins;

  /** The leader's limit pins, as they are: closed to ground, or open. */
  private final StatusSignal<ForwardLimitValue> forwardLimit;

  private final StatusSignal<ReverseLimitValue> reverseLimit;

  /** A motor controller counts as silent once it has been for half a second, not at one glitch. */
  private final Debouncer connected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;
  private final DutyCycleOut dutyCycleRequest;
  private final VoltageOut homingRequest;
  private final NeutralOut neutral = new NeutralOut();
  private final String unconfigured;

  /** The Phoenix settings for a control: the joint's, its drive's and its block's, with it. */
  private final Function<MechanismControl, PhoenixMechanism.Settings> configure;

  /**
   * The CANcoder the robot reads once, to tell the rotor where the joint is, and the definition's
   * encoder it is: a joint whose motor controller reads its rotor ({@code Feedback.ROTOR}).
   */
  private final @Nullable StatusSignal<Angle> seedFrom;

  private final Sensor.@Nullable Encoder seedModel;

  /** Its CANcoder's magnet's health, whichever way it's read: none without one. */
  private final @Nullable StatusSignal<MagnetHealthValue> magnet;

  private final RobotMath math;
  private boolean seeded;

  /** Where it was seeded this loop, in Phoenix's units: NaN otherwise. */
  private double seededAt = Double.NaN;

  /**
   * Whether each device restarted since it was last asked: the leader, each follower in order, and
   * its CANcoder, if it has one.
   */
  private final BooleanSupplier leaderRestarts;

  private final BooleanSupplier[] followerRestarts;
  private final @Nullable BooleanSupplier encoderRestarts;

  /**
   * Each device's fault field, the leader, each follower in order, then its CANcoder: read once in
   * {@link JointIO#HEALTH_LOOPS} loops. A TalonFX's sent four times a second, the least Phoenix
   * sends, and a follower answering while it comes.
   */
  private final List<StatusSignal<Integer>> faultFields = new ArrayList<>();

  private final Debouncer[] followersAnswer;

  /** The loops it's been read in. */
  private int loops;

  /**
   * Configures every device of the joint, waiting for each, at startup.
   *
   * @param math what the robot's definition derives: the joint's drive, ratios and seeds
   * @param joint the joint, its block naming its control and its motors' controllers, which it must
   * @throws IllegalArgumentException for a joint whose motors aren't wired, and what its settings
   *     refuse ({@link ControllerSettings#talon})
   */
  public JointIOTalon(RobotMath math, Joint joint) {
    this(math, joint, ControllerPins.NONE);
  }

  /**
   * The same, with what's on its leader's limits: switches on its pins, or a limit switch on a
   * CANdi it reads over CAN.
   *
   * @param pins its limits, from the robot's sensor wiring
   */
  public JointIOTalon(RobotMath math, Joint joint, ControllerPins pins) {
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    if (controllers.isEmpty()) {
      throw new IllegalArgumentException(joint.label() + " has no wiring: wire it");
    }
    MechanismWiring.Device wiredLeader = controllers.get(0);
    Optional<MechanismWiring.Cancoder> wiredCancoder =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder);
    Drive drive = math.drivesOf(joint).get(0);
    configure = control -> ControllerSettings.talon(math, joint, control);
    PhoenixMechanism.Settings settings = configure.apply(ControllerSettings.control(joint));
    // The definition's encoder its CANcoder is, which the settings found.
    Sensor.Encoder absolute = wiredCancoder.isPresent() ? math.absoluteEncoder(joint) : null;
    rotations = settings.rotations();
    CANBus bus = PhoenixMechanism.bus(wiredLeader.bus());
    List<String> failed = new ArrayList<>();
    Motor motor = ((Actuator.Motors) drive.actuator()).motor();
    leader = new Talon(wiredLeader.controller(), wiredLeader.id(), bus, motor);
    TalonFXConfiguration leading = settings.leader();
    PhoenixMechanism.limits(leading.HardwareLimitSwitch, pins);
    if (!applied(() -> leader.apply(leading, STARTUP_TIMEOUT_SECONDS))) {
      failed.add("motor " + wiredLeader.id());
    }
    for (PhoenixMechanism.FollowerSettings each : settings.followers()) {
      Talon follower = new Talon(wiredLeader.controller(), each.id(), bus, motor);
      if (!applied(() -> follower.apply(each.configuration(), STARTUP_TIMEOUT_SECONDS))) {
        failed.add("motor " + each.id());
      }
      follower.device().setControl(each.request());
      followers.add(follower);
    }
    StatusSignal<Angle> cancoderReading = null;
    StatusSignal<MagnetHealthValue> cancoderMagnet = null;
    BooleanSupplier cancoderRestarts = null;
    CANcoder wired = null;
    if (settings.cancoder().isPresent()) {
      CANcoderConfiguration config = settings.cancoder().get();
      int id = wiredCancoder.orElseThrow().id();
      CANcoder cancoder = new CANcoder(id, bus);
      wired = cancoder;
      if (!applied(() -> cancoder.getConfigurator().apply(config, STARTUP_TIMEOUT_SECONDS))) {
        failed.add("CANcoder " + id);
      }
      cancoderReading = cancoder.getAbsolutePosition();
      cancoderMagnet = cancoder.getMagnetHealth();
      cancoderRestarts = restarts(cancoder);
    }
    faultFields.add(leader.device().getFaultField(false));
    for (Talon follower : followers) {
      faultFields.add(follower.device().getFaultField(false));
    }
    // A TalonFX's sent four times a second: a frame a device of its own on the bus, which Phoenix
    // would otherwise stop as it optimizes it (below). A CANcoder sends its own anyway.
    BaseStatusSignal.setUpdateFrequencyForAll(4, faultFields.toArray(BaseStatusSignal[]::new));
    if (wired != null) {
      faultFields.add(wired.getFaultField(false));
    }
    followersAnswer = new Debouncer[followers.size()];
    for (int i = 0; i < followersAnswer.length; i++) {
      followersAnswer[i] = new Debouncer(0.5, Debouncer.DebounceType.FALLING);
    }
    encoderRestarts = cancoderRestarts;
    magnet = cancoderMagnet;
    leaderRestarts = restarts(leader.parent());
    followerRestarts =
        followers.stream()
            .map(follower -> restarts(follower.parent()))
            .toArray(BooleanSupplier[]::new);
    boolean readsRotor =
        wiredCancoder.isEmpty() || wiredCancoder.get().feedback() == MechanismWiring.Feedback.ROTOR;
    if (readsRotor) {
      // Where it rests, until a CANcoder read by the robot says where it is (seed).
      leader
          .device()
          .setPosition(rotations.toRotations(joint.restPosition()), STARTUP_TIMEOUT_SECONDS);
    }
    this.math = math;
    seedFrom = readsRotor ? cancoderReading : null;
    seedModel = readsRotor ? absolute : null;
    unconfigured = String.join(", ", failed);

    position = leader.device().getPosition();
    velocity = leader.device().getVelocity();
    volts = leader.device().getMotorVoltage();
    statorCurrent = leader.device().getStatorCurrent();
    supplyCurrent = leader.device().getSupplyCurrent();
    this.pins = pins;
    forwardLimit = leader.device().getForwardLimit();
    reverseLimit = leader.device().getReverseLimit();
    signals =
        new BaseStatusSignal[] {
          position, velocity, volts, statorCurrent, supplyCurrent, forwardLimit, reverseLimit
        };
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, signals);
    List<ParentDevice> devices = new ArrayList<>();
    for (Talon follower : followers) {
      devices.add(follower.parent());
    }
    devices.add(leader.parent());
    ParentDevice.optimizeBusUtilizationForAll(devices.toArray(ParentDevice[]::new));

    boolean foc = foc(drive);
    positionRequest = new PositionVoltage(0).withEnableFOC(foc);
    velocityRequest = new VelocityVoltage(0).withEnableFOC(foc);
    voltageRequest = new VoltageOut(0).withEnableFOC(foc);
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(foc);
    homingRequest = new VoltageOut(0).withEnableFOC(foc).withIgnoreSoftwareLimits(true);
  }

  /** The leading motor controller: for the tests, which read what it was last told. */
  ParentDevice leader() {
    return leader.parent();
  }

  @Override
  public void retune(MechanismControl control) {
    PhoenixMechanism.Settings settings = configure.apply(control);
    // Without waiting, as the loop runs: what Phoenix doesn't take shows in the log as it reads.
    leader.apply(settings.leader().Slot0, 0);
    leader.apply(settings.leader().CurrentLimits, 0);
    for (int i = 0; i < followers.size(); i++) {
      followers.get(i).apply(settings.followers().get(i).configuration().CurrentLimits, 0);
    }
  }

  /**
   * Whether a switch on a pin is tripped: closed to ground for a normally open one, open for a
   * normally closed one. Phoenix reports the pin as it is, whatever polarity it's told for its own
   * stop. Nothing on the pin, or a CANdi's limit read over CAN, reads nothing here.
   */
  private static boolean tripped(Optional<ControllerPins.Limit> limit, boolean closed) {
    return limit.orElse(null) instanceof ControllerPins.OnPin pin
        && closed == (pin.polarity() == SensorWiring.Polarity.NORMALLY_OPEN);
  }

  /**
   * Whether the definition's motors are driven with FOC, which needs Phoenix Pro: its FOC motors
   * are named so ({@code Motor.KRAKEN_X60_FOC}, "Kraken X60 FOC").
   */
  static boolean foc(Drive drive) {
    return drive.actuator() instanceof Actuator.Motors motors
        && motors.motor().name().endsWith(" FOC");
  }

  /**
   * A device's restarts once the robot runs: its first read takes what it reported as it and the
   * robot started, which isn't one.
   */
  private static BooleanSupplier restarts(ParentDevice device) {
    BooleanSupplier restarted = device.getResetOccurredChecker();
    boolean[] started = {false};
    return () -> {
      boolean reported = restarted.getAsBoolean();
      if (!started[0]) {
        started[0] = true;
        return false;
      }
      return reported;
    };
  }

  /**
   * Tries something a device must confirm, a few times: at startup, where waiting is fine. Whether
   * it took.
   */
  private static boolean applied(Supplier<StatusCode> attempt) {
    for (int i = 0; i < TRIES; i++) {
      if (attempt.get().isOK()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Tells the rotor where the joint is, from its CANcoder's first good reading: once, at startup,
   * and again after it restarts. Only once the CANcoder has said its magnet isn't bad: it waits for
   * that, a frame it sends four times a second.
   *
   * @param at where the CANcoder says the joint is, in its units: NaN without a good reading
   * @param magnetGood whether the CANcoder has said its magnet isn't too close or too far
   */
  private void seed(double at, boolean magnetGood) {
    if (seeded || Double.isNaN(at) || !magnetGood) {
      return;
    }
    double rotationsAt = rotations.toRotations(at);
    leader.device().setPosition(rotationsAt, 0);
    seededAt = rotationsAt;
    seeded = true;
  }

  /**
   * Where its CANcoder says the joint is this loop, in its units, where the robot reads it beside
   * the rotor: NaN for a joint whose TalonFX reads it instead, or without a good reading. The
   * CANcoder's own settings give its offset, direction and wrap.
   */
  private double fromCancoder() {
    StatusSignal<Angle> reading = seedFrom;
    Sensor.Encoder model = seedModel;
    if (reading == null || model == null || !reading.refresh().getStatus().isOK()) {
      return Double.NaN;
    }
    return math.fromAbsolute(model, reading.getValueAsDouble());
  }

  /**
   * What its CANcoder says of its magnet as of this loop: null without one, or before it has said:
   * red, too close or too far; orange, adequate; green; or that it can't tell.
   */
  private @Nullable MagnetHealthValue magnet() {
    StatusSignal<MagnetHealthValue> health = magnet;
    return health != null && health.refresh().getStatus().isOK() ? health.getValue() : null;
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    boolean restarted = leaderRestarts.getAsBoolean();
    if (restarted) {
      // Its rotor counts from 0 again: a CANcoder it was seeded from seeds it again, now.
      seeded = false;
    }
    MagnetHealthValue health = magnet();
    boolean magnetBad = health == MagnetHealthValue.Magnet_Red;
    double fromCancoder = fromCancoder();
    seed(fromCancoder, health != null && !magnetBad);
    inputs.connected = connected.calculate(BaseStatusSignal.refreshAll(signals).isOK());
    // Told where it is this loop: its rotor's reading catches up on the next.
    inputs.position =
        rotations.fromRotations(Double.isNaN(seededAt) ? position.getValueAsDouble() : seededAt);
    seededAt = Double.NaN;
    inputs.velocity = rotations.fromRotations(velocity.getValueAsDouble());
    inputs.appliedVolts = volts.getValueAsDouble();
    inputs.statorCurrentAmps = Math.abs(statorCurrent.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.forwardLimit =
        tripped(pins.forward(), forwardLimit.getValue() == ForwardLimitValue.ClosedToGround);
    inputs.reverseLimit =
        tripped(pins.reverse(), reverseLimit.getValue() == ReverseLimitValue.ClosedToGround);
    inputs.absolutePosition = fromCancoder;
    inputs.magnetBad = magnetBad;
    inputs.unconfigured = unconfigured;
    inputs.restarted = restarted;
    if (inputs.followersRestarted.length != followerRestarts.length) {
      inputs.followersRestarted = new boolean[followerRestarts.length];
    }
    for (int i = 0; i < followerRestarts.length; i++) {
      inputs.followersRestarted[i] = followerRestarts[i].getAsBoolean();
    }
    BooleanSupplier encoder = encoderRestarts;
    inputs.encoderRestarted = encoder != null && encoder.getAsBoolean();
    if (loops++ % HEALTH_LOOPS == 0) {
      health(inputs);
    }
  }

  /**
   * Its devices' health: each one's faults, and each follower's answering, from its fault field,
   * the last reading standing where none came.
   */
  private void health(JointIOInputs inputs) {
    if (inputs.faults.length != faultFields.size()) {
      inputs.faults = new int[faultFields.size()];
    }
    if (inputs.followersConnected.length != followersAnswer.length) {
      inputs.followersConnected = new boolean[followersAnswer.length];
    }
    for (int i = 0; i < faultFields.size(); i++) {
      StatusSignal<Integer> field = faultFields.get(i);
      boolean answered = field.refresh().getStatus().isOK();
      if (answered) {
        inputs.faults[i] = field.getValue();
      }
      if (i >= 1 && i <= followersAnswer.length) {
        inputs.followersConnected[i - 1] = followersAnswer[i - 1].calculate(answered);
      }
    }
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    leader
        .device()
        .setControl(
            positionRequest
                .withPosition(rotations.toRotations(position))
                .withVelocity(rotations.toRotations(velocity))
                .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setVelocity(double velocity) {
    leader.device().setControl(velocityRequest.withVelocity(rotations.toRotations(velocity)));
  }

  @Override
  public void setVoltage(double volts) {
    leader.device().setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setDutyCycle(double share) {
    leader.device().setControl(dutyCycleRequest.withOutput(share));
  }

  @Override
  public void stop() {
    leader.device().setControl(neutral);
  }

  @Override
  public void home(double volts) {
    leader.device().setControl(homingRequest.withOutput(volts));
  }

  /** Sets the leader's position without waiting for it to confirm: this runs in the loop. */
  @Override
  public void resetPosition(double position) {
    leader.device().setPosition(rotations.toRotations(position), 0);
  }

  /**
   * One motor controller: a TalonFX, or a TalonFXS driving a motor through its connector, which
   * takes the settings worked out for a TalonFX in its own terms.
   */
  private static final class Talon {
    private final @Nullable TalonFX fx;
    private final @Nullable TalonFXS fxs;
    private final Motor motor;

    Talon(MechanismWiring.Controller controller, int id, CANBus bus, Motor motor) {
      this.motor = motor;
      if (controller == MechanismWiring.Controller.TALON_FXS) {
        fx = null;
        fxs = new TalonFXS(id, bus);
      } else {
        fx = new TalonFX(id, bus);
        fxs = null;
      }
    }

    CommonTalon device() {
      return fx != null ? fx : Objects.requireNonNull(fxs);
    }

    ParentDevice parent() {
      return fx != null ? fx : Objects.requireNonNull(fxs);
    }

    StatusCode apply(TalonFXConfiguration config, double timeout) {
      return fx != null
          ? fx.getConfigurator().apply(config, timeout)
          : Objects.requireNonNull(fxs)
              .getConfigurator()
              .apply(PhoenixMechanism.talonFxs(config, motor), timeout);
    }

    StatusCode apply(Slot0Configs slot, double timeout) {
      return fx != null
          ? fx.getConfigurator().apply(slot, timeout)
          : Objects.requireNonNull(fxs).getConfigurator().apply(slot, timeout);
    }

    StatusCode apply(CurrentLimitsConfigs limits, double timeout) {
      return fx != null
          ? fx.getConfigurator().apply(limits, timeout)
          : Objects.requireNonNull(fxs).getConfigurator().apply(limits, timeout);
    }
  }
}

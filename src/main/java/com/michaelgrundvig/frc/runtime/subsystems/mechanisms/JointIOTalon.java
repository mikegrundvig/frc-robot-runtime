package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.AbsoluteReader;
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
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Angle;

/**
 * A driven joint on CTRE hardware: one TalonFX or TalonFXS leading the joint's motors, the rest
 * following it, and the joint's CANcoder if its wiring names one ({@link PhoenixTalon} says how the
 * two controllers differ).
 *
 * <p><b>Its settings</b> all come from the Phoenix helper ({@link PhoenixMechanism}), which works
 * them out from the robot's definition, the joint's wiring and its control ({@link
 * ControllerSettings#talon}): ratios, gains, gravity, soft limits, current limits, directions and
 * followers. No setting is typed in here.
 *
 * <p><b>What it asks of the leader.</b> A position goes as a {@code PositionVoltage} with the
 * profile's speed, which the controller's own kV, kS and gravity terms turn into volts. A speed
 * goes as a {@code VelocityVoltage} on a slot of its own, which has the same gains without the kI:
 * kI is the position loop's alone. Requests use FOC, which needs a Phoenix Pro license, only when
 * the definition's motor is one of its FOC motors ({@code Motor.KRAKEN_X60_FOC}); otherwise this
 * runs on Phoenix 6 without a license.
 *
 * <p><b>Where the joint is.</b> A controller counts its motor's turns from wherever it was powered
 * on, so at startup it has to be told where the joint is:
 *
 * <ul>
 *   <li>A joint with no CANcoder is told it is where its definition rests it. Homing corrects that
 *       later ({@link #resetPosition}).
 *   <li>A joint whose controller reads its own rotor, with a CANcoder beside it ({@code
 *       Feedback.ROTOR}), is told where it is from the CANcoder's first good reading ({@link
 *       #seedRotorOnce}). After that the robot reads the CANcoder every loop beside the rotor, so a
 *       slip between the two shows ({@link JointIOInputs#absolutePosition}). A CANcoder that says
 *       its magnet is too close or too far is not trusted for this, and the inputs say so ({@link
 *       JointIOInputs#magnetBad}).
 *   <li>A joint whose controller reads the CANcoder itself (remote, fused or sync) needs telling
 *       nothing: the controller's position is the CANcoder's.
 * </ul>
 *
 * <p>Reading the CANcoder costs no CAN traffic: it sends its position and its magnet's health at
 * its own rates whether or not anything reads them.
 *
 * <p><b>Restarts.</b> A device that loses power and comes back (a breaker that reset, a loose
 * connector) sends a frame saying it has started, and Phoenix exposes that ({@code
 * ParentDevice.getResetOccurredChecker}). It is read once a loop and costs no traffic. A restarted
 * leader counts its rotor from 0 again, so one that was told where it is by its CANcoder is told
 * again at once; the joint handles the rest ({@link JointMechanism}). Phoenix keeps a TalonFX's
 * settings through a restart, including what {@link #retune} sent, and sends the followers their
 * follow request again by itself.
 */
public final class JointIOTalon implements JointIO {
  /** At startup each device is sent its settings up to this many times, until it confirms them. */
  private static final int STARTUP_TRIES = 5;

  /** How long each of those tries waits for the device to confirm. Startup can afford the wait. */
  private static final double STARTUP_TIMEOUT_SECONDS = 0.25;

  /**
   * A timeout of zero sends a setting and returns without waiting for the device to confirm it.
   * Everything sent while the robot's loop runs uses this, because a loop that waits on a device
   * stutters.
   */
  private static final double NO_WAIT_SECONDS = 0;

  /**
   * How long a device may go without answering before it is called silent. One missed frame is a
   * glitch, not a lost device.
   */
  private static final double SILENT_AFTER_SECONDS = 0.5;

  /**
   * How often each TalonFX sends its fault field: 4 Hz, the slowest rate Phoenix sends anything.
   * Faults are read only ten times a second ({@link JointIO#HEALTH_LOOPS}), and the frame doubles
   * as proof that a follower is still on the bus.
   */
  private static final double FAULTS_HZ = 4;

  private final PhoenixTalon leader;
  private final List<FollowingTalon> followers;

  /** Where the devices' frames are read from: Phoenix, unless a test stands in. */
  private final Frames frames;

  /** Converts between the joint's units (radians or meters) and Phoenix's rotations. */
  private final MechanismRotations rotations;

  /** The Phoenix settings for this joint under a given control: its block's, or the pit's. */
  private final Function<MechanismControl, PhoenixMechanism.Settings> settingsFor;

  /** The devices that never confirmed their settings at startup, such as "motor 32, CANcoder 9". */
  private final String unconfigured;

  // The leader's signals, which it sends once a loop. They are refreshed together, each loop.
  private final BaseStatusSignal position;
  private final BaseStatusSignal velocity;
  private final BaseStatusSignal motorVolts;
  private final BaseStatusSignal statorCurrent;
  private final BaseStatusSignal supplyCurrent;
  private final StatusSignal<ForwardLimitValue> forwardLimit;
  private final StatusSignal<ReverseLimitValue> reverseLimit;
  private final BaseStatusSignal[] leaderSignals;

  /** What the leader's two limit pins mean to the robot, from the switches wired to them. */
  private final LimitPin forwardPin;

  private final LimitPin reversePin;

  private final Debouncer leaderAnswers = silentAfterHalfASecond();
  private final Restarts leaderRestarts;
  private final StatusSignal<Integer> leaderFaults;

  /** The joint's CANcoder. Empty for a joint without one. */
  private final Optional<Cancoder> cancoder;

  /**
   * Whether the leader's rotor has been told where the joint is from the CANcoder. It is told once,
   * and once more after each time it restarts.
   */
  private boolean rotorSeeded;

  /**
   * Whether the rotor was told where the joint is during this loop, and where, in Phoenix's
   * rotations. A flag and a number rather than an Optional, because this is set and read every loop
   * and an Optional would allocate.
   */
  private boolean seededThisLoop;

  private double seededRotations;

  /** How many loops the joint has been read in, to read its devices' health every few. */
  private int loops;

  // The requests sent to the leader. Each is made once and reused, so asking allocates nothing.
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;
  private final DutyCycleOut dutyCycleRequest;

  /**
   * A voltage that ignores the soft limits. Homing pushes the joint into a hard stop, and until the
   * joint is homed its soft limits are in the wrong place.
   */
  private final VoltageOut homingPush;

  private final NeutralOut neutral = new NeutralOut();

  /**
   * What a switch on one of the leader's limit pins means. Phoenix reports each pin as it is,
   * closed to ground or open, whatever polarity the controller was told for its own stop. So the
   * switch's polarity is applied here, as the pin is read.
   */
  private enum LimitPin {
    /** Nothing on the pin, or a CANdi's limit that the controller reads over CAN: never tripped. */
    NOT_READ,
    /** A normally open switch: tripped when the pin is closed to ground. */
    TRIPPED_WHEN_CLOSED,
    /** A normally closed switch: tripped when the pin is open. */
    TRIPPED_WHEN_OPEN;

    static LimitPin of(Optional<ControllerPins.Limit> limit) {
      if (limit.isEmpty() || !(limit.get() instanceof ControllerPins.OnPin pin)) {
        return NOT_READ;
      }
      if (pin.polarity() == SensorWiring.Polarity.NORMALLY_OPEN) {
        return TRIPPED_WHEN_CLOSED;
      }
      return TRIPPED_WHEN_OPEN;
    }

    boolean tripped(boolean closedToGround) {
      return switch (this) {
        case NOT_READ -> false;
        case TRIPPED_WHEN_CLOSED -> closedToGround;
        case TRIPPED_WHEN_OPEN -> !closedToGround;
      };
    }
  }

  /** What the CANcoder says of its magnet, as far as trusting its reading goes. */
  private enum Magnet {
    /**
     * No CANcoder, or it has not said yet: its health comes in a frame sent four times a second.
     */
    NOT_SAID,
    /** Red: the magnet is too close or too far, and the reading is not to be trusted. */
    BAD,
    /** Anything else: the reading can be used. */
    USABLE
  }

  /**
   * Whether a device has restarted since it was last asked.
   *
   * <p>Phoenix's checker also reports the frame a device sent when it and the robot first started.
   * That is not a restart, so the first answer is thrown away.
   */
  private static final class Restarts {
    private final BooleanSupplier startFrameSeen;
    private boolean askedBefore;

    Restarts(BooleanSupplier startFrameSeen) {
      this.startFrameSeen = startFrameSeen;
    }

    boolean happened() {
      boolean seen = startFrameSeen.getAsBoolean();
      if (!askedBefore) {
        askedBefore = true;
        return false;
      }
      return seen;
    }
  }

  /**
   * Where the joint's devices are heard from: the status frames each sends, and the frame each
   * sends as it starts. On a robot this is Phoenix, as here.
   *
   * <p>CTRE's simulated devices always answer and never restart. So the tests of what the joint
   * does when a frame is lost, a device goes silent or a device restarts put their own in its
   * place, which says a frame did not come, or that a device started again.
   *
   * <p>The devices are numbered as their faults are listed ({@link JointIOInputs#faults}): 0 for
   * the leader, then each follower in block order, then the CANcoder.
   */
  static class Frames {
    /** Takes the newest frame of each of a device's signals, and says whether they all came. */
    StatusCode refresh(int device, BaseStatusSignal[] signals) {
      return BaseStatusSignal.refreshAll(signals);
    }

    /** Takes the newest frame of one of a device's signals, and says whether it came. */
    StatusCode refresh(int device, StatusSignal<?> signal) {
      return signal.refresh().getStatus();
    }

    /** Whether a device has sent the frame it sends as it starts, since this was last asked. */
    BooleanSupplier startFrames(int device, ParentDevice parent) {
      return parent.getResetOccurredChecker();
    }
  }

  /** The leader's number among the joint's devices ({@link Frames}). */
  private static final int LEADER = 0;

  /**
   * One motor controller that follows the leader.
   *
   * @param number its number among the joint's devices ({@link Frames})
   * @param faults its fault field, which also shows that it is still on the bus
   * @param answers whether it has answered within the last half second
   */
  private record FollowingTalon(
      PhoenixTalon talon,
      int number,
      Restarts restarts,
      StatusSignal<Integer> faults,
      Debouncer answers) {}

  /**
   * The joint's CANcoder, as the robot reads it.
   *
   * @param number its number among the joint's devices ({@link Frames})
   * @param absolutePosition where its magnet is, with the CANcoder's own offset, direction and wrap
   *     point already applied
   * @param reader converts its reading into the joint's units, as the definition's encoder that
   *     this CANcoder is does. It is made once, at startup, because working the encoder's window
   *     out makes objects and the CANcoder is read every loop ({@code RobotMath#absoluteReader}).
   * @param seedsTheRotor whether the leader reads its own rotor, so the robot tells it where the
   *     joint is from this CANcoder and reads the CANcoder each loop beside it. False when the
   *     leader reads the CANcoder itself.
   */
  private record Cancoder(
      int number,
      StatusSignal<Angle> absolutePosition,
      StatusSignal<MagnetHealthValue> magnetHealth,
      StatusSignal<Integer> faults,
      Restarts restarts,
      AbsoluteReader reader,
      boolean seedsTheRotor) {}

  /**
   * Configures every device of the joint at startup, waiting for each to confirm.
   *
   * @param math what the robot's definition derives: the joint's drive, ratios and encoder
   * @param joint the joint. Its block must name its control and its motors' controllers.
   * @throws IllegalArgumentException for a joint whose motors aren't wired, and for what its
   *     settings refuse ({@link ControllerSettings#talon})
   */
  public JointIOTalon(RobotMath math, Joint joint) {
    this(math, joint, ControllerPins.NONE);
  }

  /**
   * The same, with what is on the leader's limit inputs: switches on its pins, or a limit switch on
   * a CANdi that it reads over CAN.
   *
   * @param pins the leader's limits, from the robot's sensor wiring
   */
  public JointIOTalon(RobotMath math, Joint joint, ControllerPins pins) {
    this(math, joint, pins, new Frames());
  }

  /**
   * The same, with where its devices' frames are read from. For the tests, which lose a frame or
   * restart a device there ({@link Frames}).
   */
  JointIOTalon(RobotMath math, Joint joint, ControllerPins pins, Frames frames) {
    this.frames = frames;
    List<MechanismWiring.Device> wired =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    if (wired.isEmpty()) {
      throw new IllegalArgumentException(joint.label() + " has no wiring: wire it");
    }
    MechanismWiring.Device wiredLeader = wired.get(0);
    Optional<MechanismWiring.Cancoder> wiredCancoder =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder);
    Drive drive = math.drivesOf(joint).get(0);
    settingsFor = control -> ControllerSettings.talon(math, joint, control);
    PhoenixMechanism.Settings settings = settingsFor.apply(ControllerSettings.control(joint));
    rotations = settings.rotations();
    CANBus bus = PhoenixMechanism.bus(wiredLeader.bus());
    Motor motor = ((Actuator.Motors) drive.actuator()).motor();
    MechanismWiring.Controller kind = wiredLeader.controller();

    // From here on the devices are opened and told their settings, in this order: the leader, each
    // follower, then the CANcoder. A device that never confirms is noted and runs on its defaults.
    List<String> neverConfirmed = new ArrayList<>();
    leader = PhoenixTalon.open(kind, wiredLeader.id(), bus, motor);
    configureLeader(settings.leader(), pins, wiredLeader.id(), neverConfirmed);
    List<PhoenixTalon> following = new ArrayList<>();
    for (PhoenixMechanism.FollowerSettings each : settings.followers()) {
      PhoenixTalon follower = PhoenixTalon.open(kind, each.id(), bus, motor);
      following.add(startFollower(follower, each, neverConfirmed));
    }
    // The leader reads its own rotor unless its wiring has it read a CANcoder instead.
    boolean leaderReadsItsRotor =
        wiredCancoder.isEmpty() || wiredCancoder.get().feedback() == MechanismWiring.Feedback.ROTOR;
    int cancoderNumber = 1 + following.size();
    cancoder =
        wiredCancoder.map(
            wiring ->
                startCancoder(
                    new CANcoder(wiring.id(), bus),
                    cancoderNumber,
                    settings.cancoder().orElseThrow(),
                    math.absoluteReader(Objects.requireNonNull(math.absoluteEncoder(joint))),
                    leaderReadsItsRotor,
                    neverConfirmed));
    unconfigured = String.join(", ", neverConfirmed);

    leaderFaults = leader.device().getFaultField(false);
    List<StatusSignal<Integer>> followerFaults = new ArrayList<>();
    for (PhoenixTalon follower : following) {
      followerFaults.add(follower.device().getFaultField(false));
    }
    sendFaultFieldsSlowly(leaderFaults, followerFaults);
    // Each follower's half second of silence is counted from here, once every device has been
    // configured, and not from partway through startup.
    leaderRestarts = new Restarts(frames.startFrames(LEADER, leader.parent()));
    List<FollowingTalon> watched = new ArrayList<>();
    for (int i = 0; i < following.size(); i++) {
      PhoenixTalon follower = following.get(i);
      watched.add(
          new FollowingTalon(
              follower,
              1 + i,
              new Restarts(frames.startFrames(1 + i, follower.parent())),
              followerFaults.get(i),
              silentAfterHalfASecond()));
    }
    followers = List.copyOf(watched);
    if (leaderReadsItsRotor) {
      // Until a CANcoder says otherwise (seedRotorOnce) or the joint is homed, the joint is taken
      // to be where its definition rests it.
      leader
          .device()
          .setPosition(rotations.toRotations(joint.restPosition()), STARTUP_TIMEOUT_SECONDS);
    }

    position = leader.device().getPosition();
    velocity = leader.device().getVelocity();
    motorVolts = leader.device().getMotorVoltage();
    statorCurrent = leader.device().getStatorCurrent();
    supplyCurrent = leader.device().getSupplyCurrent();
    forwardLimit = leader.device().getForwardLimit();
    reverseLimit = leader.device().getReverseLimit();
    forwardPin = LimitPin.of(pins.forward());
    reversePin = LimitPin.of(pins.reverse());
    leaderSignals =
        new BaseStatusSignal[] {
          position, velocity, motorVolts, statorCurrent, supplyCurrent, forwardLimit, reverseLimit
        };
    sendOnlyWhatIsRead();

    boolean foc = foc(drive);
    positionRequest = new PositionVoltage(0).withEnableFOC(foc);
    velocityRequest =
        new VelocityVoltage(0).withEnableFOC(foc).withSlot(PhoenixMechanism.SPEED_SLOT);
    voltageRequest = new VoltageOut(0).withEnableFOC(foc);
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(foc);
    homingPush = new VoltageOut(0).withEnableFOC(foc).withIgnoreSoftwareLimits(true);
  }

  /**
   * Sends the leader its whole configuration, with what is on its limit inputs added to it, and
   * waits for it to confirm.
   */
  private void configureLeader(
      TalonFXConfiguration settings, ControllerPins pins, int id, List<String> neverConfirmed) {
    PhoenixMechanism.limits(settings.HardwareLimitSwitch, pins);
    if (!confirmed(() -> leader.apply(settings, STARTUP_TIMEOUT_SECONDS))) {
      neverConfirmed.add("motor " + id);
    }
  }

  /**
   * Sends a follower its own configuration, waits for it to confirm, and then tells it to follow
   * the leader. A follower needs its own current limits, neutral mode and direction: following
   * copies only the leader's output.
   */
  private static PhoenixTalon startFollower(
      PhoenixTalon follower,
      PhoenixMechanism.FollowerSettings settings,
      List<String> neverConfirmed) {
    if (!confirmed(() -> follower.apply(settings.configuration(), STARTUP_TIMEOUT_SECONDS))) {
      neverConfirmed.add("motor " + settings.id());
    }
    follower.device().setControl(settings.request());
    return follower;
  }

  /**
   * Sends the joint's CANcoder its offset, direction and wrap point, and waits for it to confirm.
   *
   * @param number its number among the joint's devices ({@link Frames})
   * @param reader converts the CANcoder's reading into the joint's units
   * @param seedsTheRotor whether the leader reads its own rotor, and is told where the joint is
   *     from this CANcoder
   */
  private Cancoder startCancoder(
      CANcoder device,
      int number,
      CANcoderConfiguration settings,
      AbsoluteReader reader,
      boolean seedsTheRotor,
      List<String> neverConfirmed) {
    boolean tookIt =
        confirmed(() -> device.getConfigurator().apply(settings, STARTUP_TIMEOUT_SECONDS));
    if (!tookIt) {
      neverConfirmed.add("CANcoder " + device.getDeviceID());
    }
    return new Cancoder(
        number,
        device.getAbsolutePosition(),
        device.getMagnetHealth(),
        device.getFaultField(false),
        new Restarts(frames.startFrames(number, device)),
        reader,
        seedsTheRotor);
  }

  /**
   * Asks each TalonFX to send its fault field {@link #FAULTS_HZ} times a second. The rate has to be
   * set explicitly: a frame whose rate nobody set is one Phoenix stops when the bus is trimmed
   * ({@link #sendOnlyWhatIsRead}). A CANcoder's fault field needs nothing: a CANcoder is not
   * trimmed, and sends it anyway.
   */
  private static void sendFaultFieldsSlowly(
      StatusSignal<Integer> leaderFaults, List<StatusSignal<Integer>> followerFaults) {
    List<BaseStatusSignal> faultFields = new ArrayList<>();
    faultFields.add(leaderFaults);
    faultFields.addAll(followerFaults);
    BaseStatusSignal.setUpdateFrequencyForAll(
        FAULTS_HZ, faultFields.toArray(BaseStatusSignal[]::new));
  }

  /**
   * Sets how often the motor controllers send each signal, to keep the CAN bus's load down.
   *
   * <p>The leader sends what {@link #updateInputs} reads once a robot loop ({@link
   * RobotLoop#FREQUENCY_HZ}): reading faster than the loop gains nothing. Then Phoenix is told to
   * stop every frame whose rate was never set, on the leader and on each follower. A follower is
   * left sending only its fault field.
   */
  private void sendOnlyWhatIsRead() {
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, leaderSignals);
    ParentDevice[] devices = new ParentDevice[followers.size() + 1];
    for (int i = 0; i < followers.size(); i++) {
      devices[i] = followers.get(i).talon().parent();
    }
    devices[followers.size()] = leader.parent();
    ParentDevice.optimizeBusUtilizationForAll(devices);
  }

  /**
   * Whether a device confirmed something it was sent at startup, after a few tries. Only for
   * startup, where waiting is fine.
   */
  private static boolean confirmed(Supplier<StatusCode> attempt) {
    for (int i = 0; i < STARTUP_TRIES; i++) {
      if (attempt.get().isOK()) {
        return true;
      }
    }
    return false;
  }

  private static Debouncer silentAfterHalfASecond() {
    return new Debouncer(SILENT_AFTER_SECONDS, Debouncer.DebounceType.FALLING);
  }

  /** The leading motor controller. For the tests, which read what it was last told. */
  ParentDevice leader() {
    return leader.parent();
  }

  /**
   * Whether the definition's motors are driven with FOC, which needs Phoenix Pro. The catalog names
   * its FOC motors so ({@code Motor.KRAKEN_X60_FOC} is "Kraken X60 FOC").
   */
  static boolean foc(Drive drive) {
    return drive.actuator() instanceof Actuator.Motors motors
        && motors.motor().name().endsWith(" FOC");
  }

  /**
   * Sends the gains and current limits the pit has tuned, without waiting for the devices to
   * confirm. The leader takes both, and takes the gains twice: once for the slot its positions run
   * on and once for the slot its speeds run on, so the two never hold different tunings. A follower
   * takes only its current limits, since it runs no loop of its own. A setting Phoenix refuses
   * shows in the log as the device reads on.
   */
  @Override
  public void retune(MechanismControl control) {
    PhoenixMechanism.Settings settings = settingsFor.apply(control);
    leader.apply(settings.leader().Slot0, NO_WAIT_SECONDS);
    leader.apply(settings.leader().Slot1, NO_WAIT_SECONDS);
    leader.apply(settings.leader().CurrentLimits, NO_WAIT_SECONDS);
    for (int i = 0; i < followers.size(); i++) {
      followers
          .get(i)
          .talon()
          .apply(settings.followers().get(i).configuration().CurrentLimits, NO_WAIT_SECONDS);
    }
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    boolean leaderRestarted = leaderRestarts.happened();
    if (leaderRestarted) {
      // A restarted leader counts its rotor from 0, so the CANcoder must tell it where the joint
      // is again. It does so below, in this same loop.
      rotorSeeded = false;
    }
    Magnet magnet = readMagnet();
    double besideRotor = readCancoderBesideRotor();
    seedRotorOnce(besideRotor, magnet);

    inputs.connected = leaderAnswers.calculate(frames.refresh(LEADER, leaderSignals).isOK());
    inputs.position = rotations.fromRotations(rotorRotationsThisLoop());
    inputs.velocity = rotations.fromRotations(velocity.getValueAsDouble());
    inputs.appliedVolts = motorVolts.getValueAsDouble();
    inputs.statorCurrentAmps = Math.abs(statorCurrent.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.forwardLimit =
        forwardPin.tripped(forwardLimit.getValue() == ForwardLimitValue.ClosedToGround);
    inputs.reverseLimit =
        reversePin.tripped(reverseLimit.getValue() == ReverseLimitValue.ClosedToGround);
    inputs.absolutePosition = besideRotor;
    inputs.magnetBad = magnet == Magnet.BAD;
    inputs.unconfigured = unconfigured;

    inputs.restarted = leaderRestarted;
    readFollowersRestarts(inputs);
    inputs.encoderRestarted = cancoder.isPresent() && cancoder.get().restarts().happened();

    boolean readsHealthThisLoop = loops % HEALTH_LOOPS == 0;
    loops++;
    if (readsHealthThisLoop) {
      readHealth(inputs);
    }
  }

  /** Reads whether each follower restarted since the last loop. */
  private void readFollowersRestarts(JointIOInputs inputs) {
    if (inputs.followersRestarted.length != followers.size()) {
      inputs.followersRestarted = new boolean[followers.size()];
    }
    for (int i = 0; i < followers.size(); i++) {
      inputs.followersRestarted[i] = followers.get(i).restarts().happened();
    }
  }

  /** What the CANcoder says of its magnet as of this loop. */
  private Magnet readMagnet() {
    if (cancoder.isEmpty()) {
      return Magnet.NOT_SAID;
    }
    StatusSignal<MagnetHealthValue> health = cancoder.get().magnetHealth();
    if (!frames.refresh(cancoder.get().number(), health).isOK()) {
      return Magnet.NOT_SAID;
    }
    if (health.getValue() == MagnetHealthValue.Magnet_Red) {
      return Magnet.BAD;
    }
    return Magnet.USABLE;
  }

  /**
   * Where the CANcoder says the joint is this loop, in the joint's units, for a joint whose leader
   * reads its own rotor. The CANcoder's own settings have already applied its offset, direction and
   * wrap point.
   *
   * @return the position, or NaN when there is no CANcoder to read beside the rotor or its reading
   *     did not come. NaN is what {@link JointIOInputs#absolutePosition} holds for "no reading",
   *     and this runs every loop, where an Optional would allocate.
   */
  private double readCancoderBesideRotor() {
    if (cancoder.isEmpty() || !cancoder.get().seedsTheRotor()) {
      return Double.NaN;
    }
    StatusSignal<Angle> reading = cancoder.get().absolutePosition();
    if (!frames.refresh(cancoder.get().number(), reading).isOK()) {
      return Double.NaN;
    }
    return cancoder.get().reader().position(reading.getValueAsDouble());
  }

  /**
   * Tells the leader's rotor where the joint is, from the CANcoder's first good reading. It happens
   * once at startup and once more after each restart of the leader, and never from a CANcoder that
   * has not yet said its magnet is good.
   *
   * @param cancoderPosition where the CANcoder says the joint is, in the joint's units, or NaN
   *     without a good reading
   */
  private void seedRotorOnce(double cancoderPosition, Magnet magnet) {
    if (rotorSeeded || Double.isNaN(cancoderPosition) || magnet != Magnet.USABLE) {
      return;
    }
    seededRotations = rotations.toRotations(cancoderPosition);
    leader.device().setPosition(seededRotations, NO_WAIT_SECONDS);
    seededThisLoop = true;
    rotorSeeded = true;
  }

  /**
   * Where the leader reads the joint this loop, in Phoenix's rotations. In the loop the rotor is
   * told where the joint is, the leader's own reading is still its old count, because its next
   * frame has not come yet. So for that one loop the answer is what it was just told.
   */
  private double rotorRotationsThisLoop() {
    if (seededThisLoop) {
      seededThisLoop = false;
      return seededRotations;
    }
    return position.getValueAsDouble();
  }

  /**
   * Reads each device's faults, and whether each follower still answers. Faults are listed leader
   * first, then each follower in order, then the CANcoder. A device whose fault field did not come
   * keeps the faults it last reported.
   */
  private void readHealth(JointIOInputs inputs) {
    int devices = 1 + followers.size() + (cancoder.isPresent() ? 1 : 0);
    if (inputs.faults.length != devices) {
      inputs.faults = new int[devices];
    }
    if (inputs.followersConnected.length != followers.size()) {
      inputs.followersConnected = new boolean[followers.size()];
    }
    readFaults(leaderFaults, inputs.faults, LEADER);
    for (int i = 0; i < followers.size(); i++) {
      FollowingTalon follower = followers.get(i);
      boolean answered = readFaults(follower.faults(), inputs.faults, follower.number());
      inputs.followersConnected[i] = follower.answers().calculate(answered);
    }
    if (cancoder.isPresent()) {
      readFaults(cancoder.get().faults(), inputs.faults, cancoder.get().number());
    }
  }

  /**
   * Reads one device's fault field into its place in the list, if it came. A device's place in the
   * list is its number among the joint's devices ({@link Frames}).
   *
   * @return whether the device answered
   */
  private boolean readFaults(StatusSignal<Integer> faultField, int[] faults, int device) {
    boolean answered = frames.refresh(device, faultField).isOK();
    if (answered) {
      faults[device] = faultField.getValue();
    }
    return answered;
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
    leader.device().setControl(homingPush.withOutput(volts));
  }

  /** Tells the leader where the joint is, without waiting for it to confirm. */
  @Override
  public void resetPosition(double position) {
    leader.device().setPosition(rotations.toRotations(position), NO_WAIT_SECONDS);
  }
}

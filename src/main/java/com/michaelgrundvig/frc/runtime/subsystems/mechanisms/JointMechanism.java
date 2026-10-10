package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.config.Rules;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.EncoderGap;
import com.michaelgrundvig.frc.world.robot.EncoderLoop;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.PlaceNames;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.util.Alert;

/**
 * One driven joint, as a Commands v3 {@link Mechanism}: the template builds one for each joint the
 * robot's motors drive ({@link Joints}), from the robot's definition and the joint's block, and
 * behavior code looks it up by its joint ({@code joints.of(PIVOT)}). Nobody writes one.
 *
 * <p>Its commands are pre-built, each a {@link Move} that ends with an outcome:
 *
 * <ul>
 *   <li>{@link #goTo(Position)} one of its named positions, profiled from where it is, and {@code
 *       goTo(PIVOT_OUT).atSpeed(Percent.of(90))} at a share of its cruise; {@link #goTo(Angle)} or
 *       {@link #goTo(Distance)} a computed one, inside its limits, and a turret's at the turn of it
 *       nearest where it is;
 *   <li>{@link #follow} a target that changes every loop, which it never stops chasing;
 *   <li>{@link #hold()} its goal, owning the joint;
 *   <li>{@link #run(Output)} one of its named outputs, and {@link #runUntil} one until a condition
 *       holds;
 *   <li>{@link #home()} its homing recipe again.
 * </ul>
 *
 * <p>Its checks are {@link Gate}s, v3 triggers that log why they're false: {@link #isAt}, {@link
 * #isReady}, {@link #isAtGoal}, {@link #isStalled}; and where it is or how fast it runs, as the
 * model says it, is a gate from {@link Joints#gate}: {@code joints.gate(PIVOT.atLeast(PIVOT_RUN))}.
 *
 * <p><b>Idle.</b> A joint does what a command told it this loop, or else what it does idle: a joint
 * held at positions holds its goal (where its last move arrived, or where it came to a stop if it
 * was canceled or timed out), and one that only runs at outputs, a roller, stops. Idle is the
 * joint's own, so a joint a command owns but drives no longer doesn't sag; a default command
 * replaces it, as v3's do.
 *
 * <p><b>Homing.</b> A joint held at positions knows where it is from its absolute encoder, or from
 * its control's homing recipe. Pushed into a hard stop, it homes on the first enable; moves wait
 * for it, and if it finds no stop it's faulted, and its moves are refused. An absolute encoder its
 * motor controller reads tells it at startup; one the robot reads itself, on a Systemcore port,
 * seeds it from its first reading, and until then it waits as if homing.
 *
 * <p><b>Interlocks</b> from the robot's definition apply beneath every command, as the last thing
 * before the joint's output: no command, binding or auto skips one. While a {@code runsOnlyWhen}
 * condition doesn't hold, the joint holds where it is, or stops; while a {@code keepsOut} region is
 * closed, its target is held at the region's edge on the side it's on, so its profile stops it
 * there. A condition on a joint whose position isn't known (unhomed, faulted, silent) fails safe.
 * The command keeps its goal, and carries on when the interlock lets it.
 *
 * <p><b>Each loop</b>, the joint reads its sensors before the commands run ({@link Joints#read}),
 * and applies what they asked after ({@link Joints#write}), so a goal set this loop is applied this
 * loop. Moves are profiled here, on the robot, each from where the joint is as it starts, and
 * streamed to the motor controller as a position and velocity, with the profile's acceleration
 * times kA added; the motor controller adds its own kV, kS and gravity. A joint whose loop closes
 * on its absolute encoder on a Systemcore port closes it here instead: each loop, the volts for the
 * profile's point from where the encoder says the joint is ({@link EncoderLoop}), sent as a
 * voltage, and the encoder's word is where the joint is. Its readings, the motor controller's and
 * the encoder's, are AdvantageKit inputs, so a replay reproduces everything here. It logs under
 * {@code Joints/<label>}: {@code State}, {@code Command}, {@code Goal}, {@code At}, {@code HeldBy}
 * and {@code Outcome}, in the order to read them when it won't move.
 *
 * <p><b>Inside</b>, what it applies each loop goes through stages, each a class of its own with
 * tests of its own: what a command asked, or its idle ({@link Idle}), as a {@link Demand}; its
 * interlocks beneath that ({@link Interlocks}), reading what other joints publish ({@link
 * JointView}), never their mechanisms; its limit switches beneath them ({@link LimitStops}); the
 * profile toward what's left ({@link JointProfile}); and its output to its IO, here. Where it's
 * driven is its {@link Travel}; how it finds where it is, and where it stands meanwhile, its {@link
 * JointHoming}; what's doubted of its switches and its PDH channels, {@link SwitchDoubts} and
 * {@link DeadChannels}.
 */
@CheckReturnValue
public final class JointMechanism implements Mechanism {
  /** A revolute joint's bands until its control says: 2° to arrive, 4° to stay. */
  static final MechanismControl.Bands ANGLE_BANDS = bands(Math.toRadians(2), Math.toRadians(4));

  /** A prismatic joint's: 1 cm to arrive, 2 cm to stay. */
  static final MechanismControl.Bands LENGTH_BANDS = bands(0.01, 0.02);

  /** A speed's, as shares of it: 3% to arrive, 6% to stay. */
  static final MechanismControl.Bands SPEED_BANDS = bands(0.03, 0.06);

  /** A joint run at an output is stalled once it's moved slower than this share of the speed... */
  static final double STALL_SHARE = 0.1;

  /** ...the output asks for, for this long, in seconds. */
  static final double STALL_SECONDS = 0.25;

  /** Where a joint stands, as {@code State} logs it. */
  public enum State {
    /**
     * Held at positions, and doesn't yet know where it is: it homes on the first enable, or from
     * its absolute encoder's first reading.
     */
    UNHOMED,
    /** Finding where it is, now. */
    HOMING,
    /** It knows where it is, or only runs at outputs. */
    READY,
    /** Its homing failed: its moves are refused until it homes again ({@link #home()}). */
    FAULTED
  }

  /**
   * What a command asked of it, its name for what it asked ("out"), the command that asked, and in
   * which loop.
   */
  private record Asked(Demand demand, @Nullable String name, @Nullable Command by, long loop) {}

  // What it is, from its block.

  private final Joint joint;
  private final String label;
  private final RobotMath math;
  private final Scheduler scheduler;

  /**
   * Why it's a stand-in for a joint with no mechanism, asked for once the robot runs ({@link
   * #standIn}): its every move refused and every gate false, saying this; null for a joint driven.
   */
  private final @Nullable String standsIn;

  /** Whether it holds a position when idle: it has a range or named positions. */
  private final boolean positional;

  /** Where it's driven, and how fast. */
  private final Travel travel;

  private final double kA;
  private final double kV;
  private final double freeSpeed;

  /** How near a position it must be to be there, and may stray: in its units. */
  private final MechanismControl.Bands positionBands;

  /** How near a speed: shares of it. */
  private final MechanismControl.Bands speedBands;

  /**
   * Its control as the pit tunes it, read each loop; and the one its motor controller last took.
   */
  private final Supplier<MechanismControl> tuned;

  private MechanismControl retuned;

  // What it reads and drives.

  private final JointIO io;
  private final JointIOInputsAutoLogged inputs = new JointIOInputsAutoLogged();

  /**
   * Its absolute encoder, where the robot reads it itself and tells the motor where the joint is
   * from it ({@link Joints#readsAbsolute}), or null.
   */
  private final @Nullable AbsoluteEncoderIO absoluteIo;

  private final AbsoluteEncoderIOInputsAutoLogged absoluteInputs =
      new AbsoluteEncoderIOInputsAutoLogged();

  /** The encoder in the robot's definition, and its port, if it's on one. */
  private final Sensor.@Nullable Encoder absoluteModel;

  private final MechanismWiring.@Nullable PortEncoder absolutePort;

  /**
   * The loop it closes itself on its absolute encoder, each loop, where its port's wiring says so
   * ({@code PortEncoder#closesLoop}); null where its motor controller runs the loop on its motor's
   * own encoder.
   */
  private @Nullable EncoderLoop encoderLoop;

  /**
   * Its absolute encoder against its motor's count, each loop it has both, which it doesn't trust
   * apart; null for a joint without an absolute encoder.
   */
  private final @Nullable EncoderSlip gap;

  /**
   * Whether its motor controller's loop runs on its motor's own count, so it can hold on it while
   * its encoder isn't trusted: not a TalonFX's on its CANcoder, nor a SPARK's on its data port.
   */
  private final boolean loopOnMotor;

  /** Its CANcoder, for what's said of its magnet: "CANcoder 34", or empty for none. */
  private final String cancoder;

  /**
   * Whether its absolute encoder is read beside its motor's count, so the two can be compared: one
   * the robot reads, a SPARK's data port, or a CANcoder a TalonFX is seeded from once; not one a
   * TalonFX's loop runs on, whose count is the encoder's.
   */
  private final boolean readsBeside;

  /** The robot's sensors, which its limit switches and its interlocks read. */
  private final Sensors sensors;

  /**
   * The switches on its leading motor controller's forward and reverse limit inputs, which it reads
   * from its controller and hands the sensors each loop; null for none.
   */
  private final Sensor.@Nullable Switch forwardPin;

  private final Sensor.@Nullable Switch reversePin;

  // Its stages.

  /** What it does when no command asks anything of it. */
  private final Idle idle;

  /** Its interlocks, beneath every command. */
  private final Interlocks interlocks;

  /** Its limit switches, beneath its interlocks, and what each is doubted for. */
  private final LimitStops stops;

  private final SwitchDoubts doubts;

  /** The profile its position follows. */
  private final JointProfile profile;

  /** How it finds where it is, and where it stands meanwhile. */
  private final JointHoming homing;

  /** Its motor controllers that read as drawing nothing on their PDH channels while it works. */
  private final DeadChannels deadChannels;

  /** Where it is, in its places' names, as {@code At} reads. */
  private final PlaceNames places;

  /** What it shows the interlocks on other joints. */
  private final JointView view =
      new JointView() {
        // A reading that isn't fresh, a status frame that didn't come, is no reading: the
        // interlocks that read it hold at once, without waiting for it to be silent.
        @Override
        public boolean answers() {
          return inputs.connected && inputs.fresh;
        }

        @Override
        public boolean knowsWhere() {
          return homing.state() == State.READY && inputs.connected && inputs.fresh;
        }

        @Override
        public double position() {
          return measured;
        }

        @Override
        public double velocity() {
          return inputs.velocity;
        }
      };

  // This loop.

  /** This loop: its count, when it started, and whether the robot is enabled. */
  private long loop;

  private long nowNanos;
  private boolean enabled;

  /**
   * Where the joint is this loop, in its units: its motor controller's reading, or, where its loop
   * closes on its absolute encoder, the encoder's.
   */
  private double measured;

  /** How fast its speed changed since the loop before, in its units a second squared. */
  private double accelerating;

  /** What a command last asked of it: what it does this loop, if it asked this loop. */
  private @Nullable Asked asked;

  /** What it applied last loop, the value it was asked for, and the name it was asked by. */
  private Request applied = Request.NONE;

  private double appliedValue = Double.NaN;
  private @Nullable String appliedName;

  /** What holds it this loop, its interlocks and its limit switches, as logged; empty for none. */
  private String heldBy = "";

  /** How the last move on it ended, as logged. */
  private String outcome = "";

  /**
   * What it was last asked that isn't a number, by whom, and in which loop: no request, so it idled
   * that loop. Logged the loop it's asked, empty otherwise.
   */
  private String rejected = "";

  private long rejectedLoop;

  /**
   * Raised the first time it's asked for what isn't a number, a bug, and left raised: made then.
   */
  private @Nullable Alert rejecting;

  // Where it logs, under Joints/<label>.

  private final String inputsKey;
  private final String stateKey;
  private final String commandKey;
  private final String goalKey;
  private final String goalNameKey;
  private final String atKey;
  private final String heldByKey;

  /** Where what its limit switches are doubted for is logged. */
  private final String switchDoubtsKey;

  /** Where the current its motors draw on their PDH channels is logged. */
  private final String pdhCurrentKey;

  /** Where its motor controllers drawing nothing on their channels while it works are logged. */
  private final String deadChannelsKey;

  /** Where its absolute encoder's reading is logged, and where it says the joint is. */
  private final String absoluteKey;

  private final String absoluteAtKey;

  private final String setpointKey;
  private final String setpointVelocityKey;
  private final String outcomeKey;

  /** Where what it was asked that isn't a number is logged, the loop it's asked. */
  private final String rejectedKey;

  /**
   * How many times each of its devices has restarted since the robot started, the leader, each
   * follower, then its CANcoder, and where that's logged, with its devices' health ({@link
   * JointIO#HEALTH_LOOPS}).
   */
  private int[] restarts = new int[0];

  private final String restartsKey;

  /**
   * A joint's control as the pit tunes it: its feedback and friction gains and its motors' current
   * limits, each within what's safe to dial in on a running robot, and each committed by the call
   * that writes it in the joint's block: its kP by the factory the control starts from, {@code
   * MechanismControl.position(31.5)}, the rest by their withers, {@code .withKs(0.25)}. Its
   * feedforward, profile and named positions aren't tunable yet: optional and listed values are
   * beyond what {@link LiveConfig} reaches.
   *
   * @param joint the joint, whose label names its tuning
   * @param committed its control as written in {@code Robots.java}
   */
  public static LiveConfig<MechanismControl> tuning(Joint joint, MechanismControl committed) {
    MechanismControl.Gains gains = committed.gains();
    MechanismControl.CurrentLimits limits = committed.currentLimits();
    double stator = limits.stator().in(Amps);
    double supply = limits.supply().in(Amps);
    String factory = committed.loop() == MechanismControl.Loop.POSITION ? "position" : "velocity";
    return LiveConfig.of(joint.label(), committed)
        // Volts per radian (or meter), and per radian (or meter) per second: up to ten times what's
        // committed, which tells the joint's scale.
        .number(
            "gains.kP",
            Limits.between(0, Math.max(10 * gains.kP(), 1)),
            "MechanismControl." + factory + "(%s)")
        .number("gains.kD", Limits.between(0, Math.max(10 * gains.kD(), 1)), ".withKd(%s)")
        .number("gains.kS", Limits.between(0, Math.max(gains.kS(), 2)), ".withKs(%s)") // volts
        .measure(
            "currentLimits.stator",
            Amps.of(Math.min(5, stator)),
            Amps.of(Math.max(120, stator)),
            ".withStatorLimit(%s)")
        .measure(
            "currentLimits.supply",
            Amps.of(Math.min(5, supply)),
            Amps.of(Math.max(80, supply)),
            ".withSupplyLimit(%s)");
  }

  /**
   * @param joint one the template drives ({@link Joints#driven}), its block saying how
   * @param tuned its control as the pit tunes it, read each loop; its block's until it's tuned
   * @param absolute its absolute encoder, where the robot reads it itself ({@link
   *     Joints#readsAbsolute}): until its first reading the joint waits, unhomed, and then tells
   *     its motor controller where it is from it, and where its port's wiring says the joint's loop
   *     closes on it, the joint closes that loop itself, each loop, on the encoder's reading; null
   *     for none
   * @param sensors the robot's sensors: its limit switches stop it, beneath every command and
   *     interlock, beside the controller's own stop where its controller reads the switch, and the
   *     only stop where it doesn't (a Systemcore port, a CANdi beside a SPARK); and a switch on its
   *     controller's limit input it reads from its controller, and hands them each loop
   */
  JointMechanism(
      RobotMath math,
      Joint joint,
      Supplier<MechanismControl> tuned,
      JointIO io,
      @Nullable AbsoluteEncoderIO absolute,
      Sensors sensors,
      Scheduler scheduler) {
    this(math, joint, tuned, io, absolute, sensors, scheduler, null);
  }

  /**
   * A stand-in for a joint with no mechanism, asked for once the robot runs, where a throw would
   * stop the robot ({@link Joints#of}): it's never read or driven, its every move ends {@link
   * Outcome#REFUSED} as it starts and its every gate holds false, each saying why, and it stands
   * {@link State#FAULTED}.
   *
   * @param why why the joint has no mechanism
   */
  static JointMechanism standIn(
      RobotMath math, Joint joint, Sensors sensors, Scheduler scheduler, String why) {
    MechanismControl none = MechanismControl.position(0);
    return new JointMechanism(
        math, joint, () -> none, new JointIO() {}, null, sensors, scheduler, why);
  }

  private JointMechanism(
      RobotMath math,
      Joint joint,
      Supplier<MechanismControl> tuned,
      JointIO io,
      @Nullable AbsoluteEncoderIO absolute,
      Sensors sensors,
      Scheduler scheduler,
      @Nullable String standsIn) {
    this.joint = joint;
    this.math = math;
    this.tuned = tuned;
    this.standsIn = standsIn;
    // A stand-in's joint has no control and no motors: nothing about how it's driven is worked
    // out, since nothing drives it.
    MechanismControl control = standsIn == null ? ControllerSettings.control(joint) : tuned.get();
    retuned = control;
    label = joint.label();
    this.scheduler = scheduler;
    this.io = io;
    positional = joint.holdsPositions();
    idle = new Idle(positional);
    this.sensors = sensors;
    List<Sensor.Switch> limits = new ArrayList<>();
    Sensor.Switch forward = null;
    Sensor.Switch reverse = null;
    for (Sensor sensor : math.sensors()) {
      if (!(sensor instanceof Sensor.Switch onSwitch)) {
        continue;
      }
      if (onSwitch.kind() instanceof Sensor.Switch.Limit limit && limit.joint().equals(joint)) {
        limits.add(onSwitch);
      }
      if (ControllerPins.onPin(onSwitch) instanceof SensorWiring.ControllerLimit pin
          && pin.joint().equals(joint.label())) {
        if (pin.direction() == SensorWiring.Direction.FORWARD) {
          forward = onSwitch;
        } else {
          reverse = onSwitch;
        }
      }
    }
    forwardPin = forward;
    reversePin = reverse;
    stops = new LimitStops(limits);
    doubts = new SwitchDoubts(joint, limits);
    travel =
        standsIn == null
            ? Travel.of(math, joint, control)
            : new Travel(
                joint, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, false, false, 1, 1, 0);
    freeSpeed = standsIn == null ? math.freeSpeed(joint) : 0;
    if (standsIn == null) {
      // The plan's: tuned, else the definition's seed, as its motor controller has them.
      ControllerPlan plan = ControllerPlan.of(math, joint, control);
      kA = plan.kA();
      kV = plan.kV();
    } else {
      kA = 0;
      kV = 0;
    }
    MechanismControl.Bands kindBands = joint instanceof Joint.Revolute ? ANGLE_BANDS : LENGTH_BANDS;
    positionBands = positional ? control.bands().orElse(kindBands) : kindBands;
    speedBands = positional ? SPEED_BANDS : control.bands().orElse(SPEED_BANDS);
    absoluteIo = absolute;
    MechanismWiring.PortEncoder port =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port).orElse(null);
    absoluteModel = absolute == null ? null : math.absoluteEncoder(joint);
    absolutePort = absolute == null ? null : port;
    if (absolute != null && port != null && port.closedLoop()) {
      encoderLoop = ControllerSettings.encoderLoop(math, joint, control);
    }
    Driven.AbsoluteEncoder encoder = joint.driven().absoluteEncoder().orElse(null);
    gap =
        standsIn == null && positional && encoder != null && math.absoluteEncoder(joint) != null
            ? new EncoderSlip(EncoderGap.of(math, joint, control))
            : null;
    loopOnMotor =
        encoder == null
            || !(encoder
                    .cancoder()
                    .filter(on -> on.feedback() != MechanismWiring.Feedback.ROTOR)
                    .isPresent()
                || encoder
                    .dataPort()
                    .filter(MechanismWiring.DataPortEncoder::closedLoop)
                    .isPresent());
    cancoder =
        encoder == null ? "" : encoder.cancoder().map(on -> "CANcoder " + on.id()).orElse("");
    readsBeside =
        encoder != null
            && (encoder.port().isPresent()
                || encoder.dataPort().isPresent()
                || encoder
                    .cancoder()
                    .filter(on -> on.feedback() == MechanismWiring.Feedback.ROTOR)
                    .isPresent());
    profile = new JointProfile(travel);
    homing =
        new JointHoming(
            math,
            joint,
            positional ? control.homing().orElse(null) : null,
            absolute != null,
            encoderLoop != null,
            absolutePort == null ? -1 : absolutePort.port(),
            at -> {
              io.resetPosition(at);
              idle.holdAt(clamp(at));
            });
    if (standsIn != null) {
      homing.faulted(standsIn);
    }
    interlocks = new Interlocks(travel, positional);
    places = new PlaceNames(joint, positionBands.enter());
    String key = "Joints/" + label;
    inputsKey = key;
    stateKey = key + "/State";
    commandKey = key + "/Command";
    goalKey = key + "/Goal";
    goalNameKey = key + "/GoalName";
    atKey = key + "/At";
    heldByKey = key + "/HeldBy";
    switchDoubtsKey = key + "/SwitchDoubts";
    pdhCurrentKey = key + "/PdhCurrentAmps";
    deadChannelsKey = key + "/DeadChannels";
    absoluteKey = key + "/Absolute";
    absoluteAtKey = key + "/AbsoluteAt";
    setpointKey = key + "/Setpoint";
    setpointVelocityKey = key + "/SetpointVelocity";
    outcomeKey = key + "/Outcome";
    rejectedKey = key + "/Rejected";
    restartsKey = key + "/Restarts";
    deadChannels = new DeadChannels(joint, control.deadChannel());
  }

  private static MechanismControl.Bands bands(double enter, double stay) {
    return MechanismControl.Bands.builder().enter(enter).stay(stay).build();
  }

  /**
   * An interlock on this joint, and what the joint its condition reads publishes, or null for a
   * sensor's: set once, by {@link Joints}.
   */
  void restrict(Interlock interlock, @Nullable JointView other) {
    interlocks.add(interlock, other);
  }

  /** What it shows the interlocks on other joints: where it is and how fast, and whether known. */
  JointView view() {
    return view;
  }

  /** The joint, as the robot's definition has it. */
  public Joint joint() {
    return joint;
  }

  @Override
  public String getName() {
    return label;
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  /** Where the joint is, as of this loop: radians, or meters. */
  public double position() {
    return measured;
  }

  /** How fast it moves, as of this loop: radians, or meters, a second. */
  public double velocity() {
    return inputs.velocity;
  }

  /** The voltage across its motor, as of this loop's logged inputs. */
  double appliedVolts() {
    return inputs.appliedVolts;
  }

  /** The current through its motor's windings, as of this loop's logged inputs. */
  double statorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  /**
   * Where it's held or headed: the goal of the move driving it, or of its idle, as its interlocks
   * leave it; where it is, while it runs at an output. NaN before it's first read.
   */
  public double goal() {
    return switch (applied) {
      case POSITION -> profile.goal();
      case NONE, HOMING -> idle.at();
      case VELOCITY, VOLTAGE, DUTY_CYCLE, NEUTRAL -> clamp(measured);
    };
  }

  /** Where it stands: unhomed, homing, ready or faulted. */
  public State state() {
    return homing.state();
  }

  /**
   * Where it stands, as {@code State} logs it: why, when it's faulted, and while it homes, toward
   * what and for how long at most ("HOMING: toward its lower stop, for at most 9.5 s").
   */
  String stateShown() {
    return homing.shown();
  }

  /** What holds it this loop, its interlocks and its limit switches, as logged; empty for none. */
  public String heldBy() {
    return heldBy;
  }

  // ---- The pre-built commands ----

  /**
   * Moves to one of its positions at its cruise, profiled from where it is, and ends when it gets
   * there ({@link Outcome#ARRIVED}) or its deadline passes: twice the profile's time from where it
   * starts, plus half a second, unless {@link Move#within} says otherwise. {@code
   * pivot.goTo(PIVOT_OUT)}.
   *
   * <p>A position is a name on the joint, and it goes where the robot running it puts it: a
   * practice robot's joint, its position re-placed ({@code replaced}), goes to its own, given the
   * competition robot's constant. A position that isn't its own ({@link Joint#whyNotOwn}) is
   * refused, saying so: a name it doesn't have, another joint's or one left out of its {@code
   * .positions(...)}, or one of its names at a value it has never had, another joint's of the same
   * name, elsewhere ({@code elevator.goTo(HOPPER_OUT)}, each with an "out"). Built as the robot
   * starts, as its bindings and the autos built with them are, the move throws: the robot doesn't
   * start, and a test that builds it fails. Built once the robot runs, in a command or anywhere
   * else, it ends {@link Outcome#REFUSED} as it starts, its reason logged, so a robot on the field
   * is never stopped by one ({@link Refusals}). Every other move, gate and option that can't work
   * is refused the same way.
   *
   * @throws IllegalArgumentException for a position not its own, built as the robot starts
   */
  public Move goTo(Position position) {
    String why = refusedAsBuilt(joint.whyNotOwn(position));
    return Move.goTo(
        this,
        (why == null ? joint.own(position) : position).value(),
        position.name(),
        position.name(),
        why);
  }

  /**
   * Moves to a computed angle, as {@link #goTo(Position)} does: {@link Outcome#REFUSED} if it's
   * outside the joint's limits, its soft limits or else its range. A joint whose range spans a
   * whole turn or more, a turret's, takes it as a heading: it goes to whichever turn of it, the
   * angle plus or minus whole turns, lies inside its limits nearest where it is as the move starts,
   * and is refused only if none does. A joint that slides is refused it as a place not its own is.
   *
   * @throws IllegalArgumentException if the joint slides, built as the robot starts
   */
  public Move goTo(Angle angle) {
    if (!(joint instanceof Joint.Revolute)) {
      return Move.goTo(
          this,
          Double.NaN,
          null,
          Rules.shown(angle),
          refusedAsBuilt(label + " slides: send it to a distance"));
    }
    double radians = angle.in(Radians);
    return Move.goTo(this, radians, null, joint.shown(radians), refusedAsBuilt(null));
  }

  /**
   * Moves to a computed distance, as {@link #goTo(Position)} does: {@link Outcome#REFUSED} if it's
   * outside the joint's limits. A joint that turns is refused it as a place not its own is.
   *
   * @throws IllegalArgumentException if the joint turns, built as the robot starts
   */
  public Move goTo(Distance distance) {
    if (!(joint instanceof Joint.Prismatic)) {
      return Move.goTo(
          this,
          Double.NaN,
          null,
          Rules.shown(distance),
          refusedAsBuilt(label + " turns: send it to an angle"));
    }
    double meters = distance.in(Meters);
    return Move.goTo(this, meters, null, joint.shown(meters), refusedAsBuilt(null));
  }

  /**
   * Chases a target every loop, never ending on its own: a position (an angle or a distance),
   * profiled toward wherever it is now and held inside the joint's limits, or a speed, on the
   * velocity loop. A turret, a joint whose range spans a whole turn or more, chases an angle as a
   * heading, at the turn of it inside its limits nearest where it is each loop ({@link
   * #goTo(Angle)}). {@code hood.follow(() -> Degrees.of(HOOD_AT_METERS.get(distance())))}, {@code
   * shooter.follow(() -> RPM.of(speedFor(distance())))}. A target of the wrong kind for the joint
   * ends it {@link Outcome#REFUSED}.
   */
  public Move follow(Supplier<? extends Measure<?>> target) {
    return Move.follow(this, target, refusedAsBuilt(null));
  }

  /** Owns the joint and holds its goal, never ending: what it does idle, as a command. */
  public Move hold() {
    return Move.hold(this, refusedAsBuilt(null));
  }

  /**
   * Runs it at one of its outputs, a speed, a voltage or a duty cycle, until canceled: for "while
   * held" and for work in the background. {@code roller.run(INTAKING)}. {@link Output#STOPPED} is
   * every joint's. An output is a name on the joint, run as the robot running it has it; one it
   * doesn't have is refused as a position is by {@link #goTo(Position)}: thrown as the robot
   * starts, {@link Outcome#REFUSED} once it runs.
   *
   * @throws IllegalArgumentException for an output not its own, built as the robot starts
   */
  public Move run(Output output) {
    String why = refusedAsBuilt(joint.whyNotOwn(output));
    return Move.run(this, why == null ? joint.own(output) : output, null, why);
  }

  /**
   * Runs it at one of its outputs until a condition holds, which it ends on: {@link
   * Outcome#ARRIVED}, or {@link Outcome#TIMED_OUT} after 3 s unless {@link Move#within} says
   * otherwise. {@code roller.runUntil(INTAKING, coralIn)}. A {@link Gate} names it ("roller:
   * intaking until coral in claw") and, timed out, says why it never held ("coral in claw is false:
   * claw beam lost"). An output that isn't its own is refused as a position is by {@link
   * #goTo(Position)}: thrown as the robot starts, {@link Outcome#REFUSED} once it runs.
   *
   * @throws IllegalArgumentException for an output not its own, built as the robot starts
   */
  public Move runUntil(Output output, BooleanSupplier condition) {
    String why = refusedAsBuilt(joint.whyNotOwn(output));
    return Move.run(this, why == null ? joint.own(output) : output, condition, why);
  }

  /**
   * Why a move or a gate on it can't be, as it's built ({@link Refusals#asBuilt}): thrown as the
   * robot starts, kept once it runs for the move to end {@link Outcome#REFUSED} on, or the gate to
   * hold false. A stand-in's every one can't be, for why it stands in.
   *
   * @param why why this one can't be; null for nothing
   * @throws IllegalArgumentException with the reason, as the robot starts
   */
  private @Nullable String refusedAsBuilt(@Nullable String why) {
    return Refusals.asBuilt(scheduler, standsIn != null ? standsIn : why);
  }

  /**
   * Homes it again by its recipe, owning it meanwhile: {@link Outcome#ARRIVED} once it knows where
   * it is, {@link Outcome#REFUSED} if its homing fails. A joint with an absolute encoder, one
   * stowed by hand, or a roller is home already.
   */
  public Move home() {
    return Move.home(this, refusedAsBuilt(null));
  }

  /**
   * Asks for a voltage across its motors, this loop only: for a command with a law of its own, such
   * as a push held to a current. The loop it isn't asked again, the joint goes back to its idle.
   */
  public void setVoltage(double volts) {
    ask(Request.VOLTAGE, volts, 1, null);
  }

  // ---- Its gates ----

  /**
   * A gate on it, named, its logic built as the gate is; or, where that can't be (a place not its
   * own), refused as a move is ({@link Refusals#gate}): thrown as the robot starts, and once it
   * runs, a gate that holds false saying why. A stand-in's every gate is refused.
   *
   * @param logic builds its logic, throwing {@link IllegalArgumentException} with why it can't be
   * @throws IllegalArgumentException with why it can't be, as the robot starts
   */
  private Gate gate(String name, Supplier<Gate.Logic> logic) {
    if (standsIn != null) {
      return Refusals.gate(scheduler, name, standsIn);
    }
    Gate.Logic built;
    try {
      built = logic.get();
    } catch (IllegalArgumentException e) {
      return Refusals.gate(scheduler, name, String.valueOf(e.getMessage()));
    }
    return new Gate(scheduler, name, built);
  }

  /**
   * Whether it's at one of its positions, within its band, measured: "pivot at out". One not its
   * own is refused as {@link #goTo(Position)} refuses it: thrown as the robot starts, and once it
   * runs, a gate that holds false saying why.
   *
   * @throws IllegalArgumentException if the position isn't one of its own, as the robot starts
   */
  public Gate isAt(Position position) {
    return isAt(label + " at " + position.name(), position);
  }

  /** Whether it's at one of its positions, as {@link #isAt(Position)}, under another name. */
  Gate isAt(String name, Position position) {
    return gate(
        name,
        () -> {
          double target = joint.own(position).value();
          boolean[] inside = {false};
          return why -> {
            double off = Math.abs(measured - target);
            inside[0] =
                homing.state() == State.READY
                    && (inside[0] ? off <= positionBands.stay() : off <= positionBands.enter());
            return inside[0];
          };
        });
  }

  /**
   * Whether it runs at one of its outputs, within its band for a speed: "shooter at shooting". One
   * not its own is refused as {@link #isAt(Position)} refuses a position.
   *
   * @throws IllegalArgumentException if the output isn't one of its own, as the robot starts
   */
  public Gate isAt(Output output) {
    return isAt(label + " at " + output.name(), output);
  }

  /** Whether it runs at one of its outputs, as {@link #isAt(Output)}, under another name. */
  Gate isAt(String name, Output output) {
    return gate(
        name,
        () -> {
          Output own = joint.own(output);
          boolean[] inside = {false};
          return why -> {
            boolean running = enabled && own.name().equals(appliedName);
            inside[0] =
                running
                    && (own.kind() != Output.Kind.VELOCITY || withinSpeed(own.value(), inside[0]));
            return inside[0];
          };
        });
  }

  /** Whether it knows where it is and its motor controllers answer: "pivot ready". */
  public Gate isReady() {
    return gate(label + " ready", () -> why -> homing.state() == State.READY && inputs.connected);
  }

  /**
   * Whether it's where it's asked to be: at its goal, within its band, with its profile there, for
   * a position (its idle's included); within its band of the speed, for a speed; running, for a
   * voltage or a duty cycle; still, stopped. "shooter at goal".
   */
  public Gate isAtGoal() {
    boolean[] inside = {false};
    return gate(
        label + " at goal",
        () ->
            why -> {
              inside[0] =
                  enabled
                      && homing.state() == State.READY
                      && switch (applied) {
                        case POSITION -> {
                          double off = Math.abs(measured - profile.goal());
                          yield profile.arrived()
                              && (inside[0]
                                  ? off <= positionBands.stay()
                                  : off <= positionBands.enter());
                        }
                        case VELOCITY -> withinSpeed(appliedValue, inside[0]);
                        case VOLTAGE, DUTY_CYCLE -> true;
                        case NEUTRAL -> Math.abs(inputs.velocity) <= speedBands.enter() * freeSpeed;
                        case NONE, HOMING -> false;
                      };
              return inside[0];
            });
  }

  /**
   * Whether it's stalled: run at an output, it has moved slower than a tenth of the speed it asks
   * for, for a quarter of a second. A jam, for an unjam to start on: {@code
   * indexer.isStalled().onTrue(indexer.run(INDEXER_UNJAMMING).withTimeout(Seconds.of(0.2)))}.
   */
  public Gate isStalled() {
    Debouncer stalling = new Debouncer(STALL_SECONDS, Debouncer.DebounceType.RISING);
    return gate(
        label + " stalled",
        () ->
            why -> {
              double asked =
                  switch (applied) {
                    case VELOCITY -> Math.abs(appliedValue);
                    case VOLTAGE -> Math.abs(appliedValue) / Motor.NOMINAL_VOLTS * freeSpeed;
                    case DUTY_CYCLE -> Math.abs(appliedValue) * freeSpeed;
                    default -> 0;
                  };
              return stalling.calculate(
                  enabled && asked > 0 && Math.abs(inputs.velocity) < STALL_SHARE * asked);
            });
  }

  /** Whether it runs within its speed band of a speed: the stay band if it already was. */
  private boolean withinSpeed(double velocity, boolean was) {
    double share = was ? speedBands.stay() : speedBands.enter();
    double allowed = share * (velocity == 0 ? freeSpeed : Math.abs(velocity));
    return Math.abs(inputs.velocity - velocity) <= allowed;
  }

  // ---- What moves ask of it, each loop they run ----

  /** Asks it to be at a position this loop, profiled at a share of its cruise. */
  void askPosition(double position, double fraction, @Nullable String name) {
    ask(Request.POSITION, position, fraction, name);
  }

  /** Asks it to run at an output this loop. */
  void askOutput(Output output) {
    switch (output.kind()) {
      case VELOCITY -> ask(Request.VELOCITY, output.value(), 1, output.name());
      case VOLTAGE -> ask(Request.VOLTAGE, output.value(), 1, output.name());
      case DUTY_CYCLE ->
          ask(
              output.value() == 0 ? Request.NEUTRAL : Request.DUTY_CYCLE,
              output.value(),
              1,
              output.name());
    }
  }

  /** Asks it to run at a speed this loop, a computed one. */
  void askVelocity(double velocity) {
    ask(Request.VELOCITY, velocity, 1, null);
  }

  /**
   * A request this loop, from the command running. One that isn't a number (NaN, or infinite, as a
   * computed target's division by zero gives) is no request: the joint idles this loop, beneath its
   * interlocks as ever, rather than be driven at it or let it past a keep-out. It's logged ({@code
   * Joints/<label>/Rejected}), with who asked, and the first raises an alert that stays raised.
   */
  private void ask(Request kind, double value, double fraction, @Nullable String name) {
    Command by = outermost(scheduler.currentCommand());
    if (!Double.isFinite(value)) {
      reject(kind, value, by);
      return;
    }
    asked = new Asked(Demand.of(kind, value, fraction), name, by, loop);
  }

  /** What it was asked this loop that isn't a number, by whom, as logged: empty for nothing. */
  String rejected() {
    return rejectedLoop == loop ? rejected : "";
  }

  /** A request that isn't a number: logged this loop, and the first one raises an alert. */
  private void reject(Request kind, double value, @Nullable Command by) {
    String what =
        switch (kind) {
          case POSITION -> "a position";
          case VELOCITY -> "a speed";
          case VOLTAGE -> "a voltage";
          default -> "a share of the battery";
        };
    String who = by == null ? "code outside any command" : by.name();
    rejected =
        who
            + " asked "
            + label
            + " for "
            + what
            + " of "
            + value
            + ", which isn't a number: "
            + label
            + " idles instead; check how "
            + who
            + " works it out";
    rejectedLoop = loop;
    if (rejecting == null) {
      Alert alert = new Alert("Joints", label + " rejected", rejected, Alert.Level.HIGH);
      alert.set(true);
      rejecting = alert;
    }
  }

  /** Releases its alert, so a test can build another robot. */
  void close() {
    Alert alert = rejecting;
    if (alert != null) {
      alert.close();
      rejecting = null;
    }
  }

  /** The command a command runs inside, all the way out: the one bound or scheduled, "Intake". */
  private @Nullable Command outermost(@Nullable Command command) {
    Command outer = command;
    for (Command up = outer; up != null; up = scheduler.getParentOf(up)) {
      outer = up;
    }
    return outer;
  }

  /** A move begins: its profile starts from where the joint is now, not where it was headed. */
  void beginMove() {
    profile.restart();
  }

  /**
   * A move on it ended: idle holds where it arrived, or, if it didn't, where the joint comes to a
   * stop from where it is measured, so a joint that jammed holds where it jammed rather than
   * pushing on toward a target it couldn't reach.
   */
  void moveEnded(boolean arrived, double target) {
    if (applied != Request.POSITION) {
      return; // it ran at a speed, or never moved: its idle decides from what it did
    }
    if (arrived) {
      idle.moveEnded(target);
    } else {
      idle.moveEnded(stoppingPoint());
      profile.restart();
    }
  }

  /** Logs how the last move on it ended, from now on. */
  void recordOutcome(String outcome) {
    this.outcome = outcome;
  }

  /** How the last move on it ended: "pivot → out: ARRIVED in 0.41 s"; empty before any has. */
  public String lastOutcome() {
    return outcome;
  }

  /** When this loop began, in nanoseconds on the robot's clock. */
  long now() {
    return nowNanos;
  }

  /**
   * Why moves on it can't run, or null when they can: it's a stand-in for a joint with no
   * mechanism, its motor controllers aren't answering, or its homing failed.
   */
  @Nullable String fault() {
    if (standsIn != null) {
      return standsIn;
    }
    if (!inputs.connected) {
      return label + " isn't answering: its motor controllers are silent";
    }
    return homing.fault();
  }

  /**
   * A move on it waits for it to know where it is: whether it must, homing or unhomed. Unhomed with
   * the robot enabled, as a canceled homing leaves it, a joint whose recipe homes it homes again
   * now ({@link JointHoming#awaited}).
   */
  boolean homesFirst() {
    return homing.awaited(nowNanos, enabled, homingSwitch());
  }

  /** Whether it homes by moving, into a hard stop or onto a switch, and so can be homed again. */
  boolean homesToAStop() {
    return homing.toAStop();
  }

  /** Whether it finds where it is from an absolute encoder the robot reads, and can again. */
  boolean homesByEncoder() {
    return homing.byEncoder();
  }

  /**
   * Starts homing it again by its recipe, or from its absolute encoder's next reading: a command
   * asked ({@link #home()}).
   */
  void startHoming() {
    homing.again(nowNanos, enabled, homingSwitch());
  }

  /** A command homing it was canceled: where it is isn't known. */
  void stopHoming() {
    homing.stop();
  }

  /**
   * Why it can't be sent to a position, or null when it can: outside its limits, at every turn of
   * it for a joint whose range spans a turn or more ({@link Travel#whyUnreachable}).
   */
  @Nullable String whyUnreachable(double position) {
    return travel.whyUnreachable(position, measured);
  }

  /**
   * Where a computed angle takes the joint from a point: for a turret, the turn of the heading
   * inside its limits nearest it; for any other joint, the angle as it is ({@link
   * Travel#turnNear}).
   */
  double turnNear(double angle, double from) {
    return travel.turnNear(angle, from);
  }

  /**
   * A computed position as the joint chases it from where it is now: inside its limits, at the turn
   * of it nearest where it is for a joint whose range spans a turn or more ({@link #turnNear}).
   */
  double reachable(double position) {
    // One that isn't a number stays one, refused as it's asked: infinity, clamped, would be a
    // limit.
    return Double.isFinite(position) ? travel.clamp(travel.turnNear(position, measured)) : position;
  }

  /**
   * Whether the move to a position has it there: its profile has reached the position, nothing
   * holds it short, and the joint is within its band of it.
   */
  boolean settledAt(double position) {
    return applied == Request.POSITION
        && appliedValue == position
        && heldBy.isEmpty()
        && profile.arrived()
        && Math.abs(measured - profile.goal()) <= positionBands.enter();
  }

  /** Whether it runs at a speed, within its band. */
  boolean atSpeed(double velocity) {
    return applied == Request.VELOCITY && appliedValue == velocity && withinSpeed(velocity, false);
  }

  /** How long its profile takes from where it is to a position, at a share of its cruise. */
  double secondsTo(double position, double fraction) {
    return profile.secondsTo(position, fraction, measured, inputs.velocity);
  }

  /** How long its profile takes from one position to another, from rest, at its full cruise. */
  double secondsBetween(double from, double to) {
    return profile.secondsTo(to, 1, from, 0);
  }

  /** How long its velocity loop takes to come within its band of a new speed, from the model. */
  double secondsToSpeed() {
    return kV > 0 ? kA / kV * Math.log(1 / speedBands.enter()) : 0;
  }

  /** A position inside its limits. */
  double clamp(double position) {
    return travel.clamp(position);
  }

  /** Whether a measure is a position the joint takes: an angle if it turns, a distance if not. */
  boolean isPosition(Measure<?> measure) {
    return joint instanceof Joint.Revolute ? measure instanceof Angle : measure instanceof Distance;
  }

  /** Whether a measure is a speed the joint runs at. */
  boolean isSpeed(Measure<?> measure) {
    return joint instanceof Joint.Revolute
        ? measure instanceof AngularVelocity
        : measure instanceof LinearVelocity;
  }

  // ---- Each loop ----

  /** Reads its sensors, and moves its homing on: before the commands run. */
  void read(long nowNanos, boolean enabled) {
    loop++;
    this.nowNanos = nowNanos;
    MechanismControl control = tuned.get();
    if (!control.equals(retuned)) {
      // The pit tuned its gains or current limits: its motor controller takes them now, and so
      // does the loop it closes itself, if it does.
      retuned = control;
      io.retune(control);
      if (encoderLoop != null) {
        encoderLoop = ControllerSettings.encoderLoop(math, joint, control);
      }
    }
    double before = inputs.velocity;
    io.updateInputs(inputs);
    Logger.processInputs(inputsKey, inputs);
    // The switches on its controller's limit inputs, as its controller read them: lost with it.
    if (forwardPin != null) {
      sensors.pinned(forwardPin, inputs.connected, inputs.forwardLimit);
    }
    if (reversePin != null) {
      sensors.pinned(reversePin, inputs.connected, inputs.reverseLimit);
    }
    accelerating = loop == 1 ? 0 : (inputs.velocity - before) / RobotLoop.PERIOD_SECONDS;
    AbsoluteEncoderIO encoder = absoluteIo;
    if (encoder != null) {
      encoder.updateInputs(absoluteInputs);
      Logger.processInputs(absoluteKey, absoluteInputs);
    }
    Sensor.Encoder model = absoluteModel;
    // Where the joint is: its motor's word, unless its loop closes on its encoder, whose word it
    // then takes while the encoder answers.
    measured =
        encoderLoop != null && model != null && absoluteInputs.connected
            ? math.fromAbsolute(model, absoluteReading())
            : inputs.position;
    boolean enabling = enabled && !this.enabled;
    if (loop == 1) {
      homing.start(nowNanos);
    }
    if (inputs.restarted) {
      // Its leading motor controller restarted, and counts from 0, or from the encoder it reads:
      // inside the encoder's turn, a turn or more from where it counted a joint that turns past it.
      // Its profile starts again from where the joint is now read, not from a point a turn away.
      homing.restarted(nowNanos, homingSwitch());
      profile.restart();
    }
    countRestarts();
    if (!inputs.unconfigured.isEmpty() && homing.state() != State.FAULTED) {
      // Without its settings, a motor controller has no current limits or ratio: driving it would
      // be a guess at full power, so nothing does.
      homing.faulted(
          possessive(label)
              + " "
              + inputs.unconfigured
              + " didn't take its settings: check its CAN ID and wiring, then restart the robot");
    }
    if (Double.isNaN(idle.at()) || enabling) {
      // First read, or just enabled: idle holds where the joint is, not where it was left.
      idle.holdAt(clamp(measured));
      profile.restart();
    }
    this.enabled = enabled;
    if (model != null && homing.awaiting()) {
      // Each loop it waits, its encoder's first reading tells its motor where it is.
      homing.seek(
          nowNanos,
          absoluteInputs.connected,
          absoluteInputs.connected ? math.fromAbsolute(model, absoluteReading()) : Double.NaN);
    }
    if (encoderLoop != null) {
      homing.watch(nowNanos, enabled, absoluteInputs.connected, stoppingPoint(), inputs.position);
    }
    trust(enabled);
    homing.goOn(nowNanos, enabled, enabling, inputs.velocity, homingSwitch());
  }

  /**
   * Counts each of its devices' restarts, and logs them with its devices' health, a few times a
   * second ({@link JointIO#HEALTH_LOOPS}): the leader, each follower, then its CANcoder, as its
   * inputs number its devices' faults.
   */
  private void countRestarts() {
    int followers = inputs.followersRestarted.length;
    int devices = 1 + followers + (cancoder.isEmpty() ? 0 : 1);
    if (restarts.length != devices) {
      restarts = Arrays.copyOf(restarts, devices);
    }
    restarts[0] += inputs.restarted ? 1 : 0;
    for (int i = 0; i < followers; i++) {
      restarts[1 + i] += inputs.followersRestarted[i] ? 1 : 0;
    }
    if (!cancoder.isEmpty()) {
      restarts[devices - 1] += inputs.encoderRestarted ? 1 : 0;
    }
    if (loop % JointIO.HEALTH_LOOPS == 1) {
      Logger.recordOutput(restartsKey, restarts);
    }
  }

  /**
   * Whether its absolute encoder is trusted this loop. A CANcoder that says its magnet is too close
   * or too far isn't; nor is an encoder that has been apart from its motor's count for a quarter
   * second ({@link EncoderSlip}): one of them slipped. Either way where the joint is isn't known:
   * it's faulted, and holds where its motor says, where it can, until its encoder is trusted again,
   * its magnet good and the two agreeing where they're read side by side, and then it holds where
   * it is.
   */
  private void trust(boolean enabled) {
    EncoderSlip apart = gap;
    if (apart == null) {
      return;
    }
    Sensor.Encoder model = absoluteModel;
    double encoder =
        model != null
            ? absoluteInputs.connected ? math.fromAbsolute(model, absoluteReading()) : Double.NaN
            : inputs.absolutePosition;
    double motor = Double.isNaN(inputs.motorPosition) ? inputs.position : inputs.motorPosition;
    // Judged only while it knows where it is, and not the loop it found it, before its motor's
    // count catches up.
    boolean knows = homing.state() == State.READY && !homing.justFound();
    String slipped = apart.apart(nowNanos, knows ? encoder : Double.NaN, motor, inputs.velocity);
    OptionalDouble held =
        loopOnMotor
            ? OptionalDouble.of(travel.stoppingPoint(motor, enabled ? inputs.velocity : 0))
            : OptionalDouble.empty();
    String holds =
        loopOnMotor ? label + " holds where its motor says until " : label + " is stopped until ";
    if (inputs.magnetBad) {
      homing.distrust(
          possessive(label)
              + " "
              + cancoder
              + " says its magnet is too close or too far, so what it reads isn't trusted: "
              + holds
              + "it's good. Check the magnet's gap and that it's centered on the CANcoder",
          held);
    } else if (slipped != null) {
      homing.distrust(
          possessive(label)
              + " absolute encoder and its motor's count are "
              + slipped
              + ": one of them slipped, a magnet loose on its shaft or a belt that skipped, so"
              + " where "
              + label
              + " is isn't known, and "
              + holds
              + "they agree. Check both, then "
              + (homing.byEncoder()
                  ? "home it to tell its motor where it is from the encoder again"
                  : "restart the robot code to tell its motor where it is again"),
          held);
    } else if (homing.distrusted()) {
      if (!readsBeside || apart.agree(encoder, motor)) {
        homing.trust();
        idle.holdAt(clamp(measured));
        profile.restart();
      } else if (!enabled) {
        homing.stillDistrusted(held);
      }
    }
  }

  /**
   * What it does with this loop's sensors, read after the joints: homing to its switch goes on, on
   * the switch's reading this loop, not the last.
   */
  void sensed() {
    homing.sensed(nowNanos, measured, homingSwitch(), sensors);
    doubts.check(nowNanos, homing.state() == State.READY && inputs.connected, measured, sensors);
    if (doubts.logged().length > 0) {
      Logger.recordOutput(switchDoubtsKey, doubts.logged());
    }
  }

  /**
   * What it does with this loop's PDH reading, read after its sensors: the current its motors draw
   * together on their channels, logged beside its leading controller's own ({@code
   * SupplyCurrentAmps}), NaN while the PDH isn't read; and each of its motor controllers that reads
   * as drawing nothing while the joint works, for longer than its control allows ({@link
   * MechanismControl.DeadChannel}), flagged until it draws again. Nothing for a joint whose block
   * says no channel.
   */
  void powered(Power power) {
    if (!deadChannels.any()) {
      return;
    }
    double together =
        deadChannels.check(nowNanos, power, inputs.connected, inputs.supplyCurrentAmps);
    Logger.recordOutput(pdhCurrentKey, together);
    Logger.recordOutput(deadChannelsKey, deadChannels.logged());
  }

  /**
   * Its motor controllers that read as drawing nothing on their PDH channels while it works, as of
   * this loop: each said, with what to check. Empty while each draws as it should.
   */
  List<String> deadChannels() {
    return deadChannels.flagged();
  }

  /**
   * How many times each of its devices has restarted since the robot started, as logged: the
   * leader, each follower, then its CANcoder.
   */
  int[] restarts() {
    return restarts.clone();
  }

  /** What its limit switches are doubted for this loop, by switch: none while they agree. */
  List<String> switchDoubts() {
    return doubts.doubts();
  }

  /**
   * Applies what the commands asked this loop, or its idle, beneath its interlocks and then its
   * limit switches: a position along its profile, anything else as it's left, to its IO.
   */
  void write() {
    Asked now = asked != null && asked.loop() == loop ? asked : null;
    Request requested = now == null ? Request.NONE : now.demand().kind();
    heldBy = "";
    if (!enabled || homing.justFound() || (positional && homing.state() != State.READY)) {
      // Disabled; or where it is isn't known, so nothing is driven to a position on a guess,
      // except homing's push; or it found its stop, and reads its new position from the next loop.
      OptionalDouble heldOnMotor = homing.heldOnMotor();
      if (enabled && homing.state() == State.HOMING) {
        io.home(homing.volts());
        applied = Request.HOMING;
      } else if (enabled && heldOnMotor.isPresent()) {
        // The encoder its loop closes on is lost: it holds where its motor says, on its motor
        // controller's own loop, rather than going limp.
        io.setPosition(heldOnMotor.getAsDouble(), 0, 0);
        applied = Request.NONE;
      } else {
        io.stop();
        applied = Request.NONE;
      }
      homing.settled();
      profile.restart();
      appliedName = null;
      log(requested, now);
      return;
    }
    Demand demand =
        idle.resolve(now == null ? Demand.NONE : now.demand(), applied, clamp(measured));
    appliedValue = demand.value();
    appliedName = now == null ? null : now.name();
    // The interlocks, beneath every command.
    demand = interlocks.apply(demand, measured, inputs.velocity, accelerating, sensors);
    // A tripped limit switch, beneath them all: nothing drives the joint on toward its end.
    demand = stops.apply(demand, measured, sensors);
    if (demand.restart()) {
      profile.restart();
    }
    heldBy = demand.heldBy();
    double value = demand.value();
    switch (demand.kind()) {
      case POSITION -> drive(value, demand.fraction());
      case VELOCITY -> io.setVelocity(value);
      case VOLTAGE -> io.setVoltage(value);
      case DUTY_CYCLE -> io.setDutyCycle(value);
      case NEUTRAL, NONE, HOMING -> io.stop();
    }
    if (demand.kind() != Request.POSITION) {
      profile.restart();
    }
    applied = demand.kind();
    log(requested, now);
  }

  /** Where the joint comes to a stop from where it is and how fast, braking at its acceleration. */
  private double stoppingPoint() {
    return travel.stoppingPoint(measured, inputs.velocity);
  }

  /** Drives it along its profile toward a position, one loop on. */
  private void drive(double position, double fraction) {
    double accelerating = profile.step(position, fraction, measured, inputs.velocity);
    EncoderLoop closed = encoderLoop;
    if (closed != null) {
      // Its loop closes here, on its encoder: the volts for the profile's point, sent as they are.
      io.setVoltage(
          closed.volts(
              profile.position(),
              profile.velocity(),
              kA * accelerating,
              measured,
              inputs.velocity));
    } else {
      io.setPosition(profile.position(), profile.velocity(), kA * accelerating);
    }
  }

  // ---- Homing ----

  /**
   * Its absolute encoder's reading, as its sensor in the definition reads: offset and direction.
   */
  private double absoluteReading() {
    MechanismWiring.PortEncoder port = absolutePort;
    return port == null ? absoluteInputs.share : port.read(absoluteInputs.share);
  }

  /** The limit switch it homes to, or null when it doesn't home to one. */
  private Sensor.@Nullable Switch homingSwitch() {
    Sensor.Switch.Limit.End end = homing.switchEnd();
    return end == null ? null : stops.at(end);
  }

  // ---- The log ----

  /**
   * What it did this loop, under its fixed keys.
   *
   * @param requested what a command asked of it this loop: {@link Request#NONE} for nothing
   * @param now what the command asked, by name and by whom: null for nothing
   */
  private void log(Request requested, @Nullable Asked now) {
    @Nullable Command by = now == null ? null : now.by();
    Logger.recordOutput(stateKey, homing.shown());
    Logger.recordOutput(
        commandKey,
        requested == Request.NONE ? "idle" : by == null ? "outside any command" : by.name());
    Logger.recordOutput(goalKey, applied == Request.POSITION ? profile.goal() : appliedValue);
    String name = requested == Request.NONE || now == null ? null : now.name();
    Logger.recordOutput(goalNameKey, name == null ? "" : name);
    Logger.recordOutput(atKey, where());
    Logger.recordOutput(heldByKey, heldBy);
    Logger.recordOutput(setpointKey, profile.position());
    Logger.recordOutput(setpointVelocityKey, profile.velocity());
    Logger.recordOutput(outcomeKey, outcome);
    Logger.recordOutput(rejectedKey, rejected());
    Sensor.Encoder model = absoluteModel;
    if (model != null && absoluteInputs.connected) {
      Logger.recordOutput(absoluteAtKey, math.fromAbsolute(model, absoluteReading()));
    } else if (!Double.isNaN(inputs.absolutePosition)) {
      // An encoder its motor controller reads: where it says the joint is, as the controller read.
      Logger.recordOutput(absoluteAtKey, inputs.absolutePosition);
    }
  }

  /**
   * Where it is, in its names: "at out", "between run and out", "below in"; a number for a joint
   * without names; what it runs at, for a roller.
   */
  private String where() {
    if (!positional) {
      String name = appliedName;
      return name != null ? name : applied == Request.NEUTRAL ? "stopped" : "idle";
    }
    if (homing.state() != State.READY) {
      return "unknown";
    }
    return places.at(measured);
  }
}

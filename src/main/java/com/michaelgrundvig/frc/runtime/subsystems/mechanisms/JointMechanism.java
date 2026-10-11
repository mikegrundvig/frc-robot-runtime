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
import com.michaelgrundvig.frc.world.config.Rules;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.EncoderGap;
import com.michaelgrundvig.frc.world.robot.EncoderLoop;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.PlaceNames;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

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
 * <p><b>Inside</b>, a loop is three steps, and each part of a step is a class of its own with tests
 * of its own.
 *
 * <ul>
 *   <li><b>Read</b> ({@link #read}): the joint's hardware is read ({@link JointReadings}), and what
 *       was read is handed to the joint's homing ({@link JointHoming}), which keeps track of
 *       whether the joint knows where it is, and to the check on whether its absolute encoder can
 *       be trusted ({@link EncoderTrust}).
 *   <li><b>Decide</b> ({@link #write}): what a command asked, or what the joint does idle ({@link
 *       Idle}), is a {@link Demand}. The joint's interlocks ({@link Interlocks}, which asks the
 *       model what they allow) and then its limit switches ({@link LimitStops}) each leave of it
 *       what they allow. Interlocks read other joints only through what those publish ({@link
 *       JointView}), never through their mechanisms.
 *   <li><b>Write</b>: a position is followed along the joint's profile ({@link JointProfile}), and
 *       anything else is sent to the joint's IO as it is. What was written is remembered as what
 *       the joint {@link Applied}, and logged ({@link JointLog}).
 * </ul>
 *
 * <p>Where the joint can be driven is its {@link Travel}. What is doubted of its limit switches and
 * of its PDH channels is in {@link SwitchDoubts} and {@link DeadChannels}.
 */
@CheckReturnValue
public final class JointMechanism implements Mechanism {
  /** A revolute joint's bands until its control says: 2° to arrive, 4° to stay. */
  static final MechanismControl.Bands ANGLE_BANDS = bands(Math.toRadians(2), Math.toRadians(4));

  /** A prismatic joint's: 1 cm to arrive, 2 cm to stay. */
  static final MechanismControl.Bands LENGTH_BANDS = bands(0.01, 0.02);

  /** A speed's, as shares of it: 3% to arrive, 6% to stay. */
  static final MechanismControl.Bands SPEED_BANDS = bands(0.03, 0.06);

  /**
   * A joint run at an output is stalled once it has moved slower than this share of the speed the
   * output asks for, for {@link #STALL_SECONDS}.
   */
  static final double STALL_SHARE = 0.1;

  /** How long a joint must have been that slow to be stalled, in seconds. */
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

  // What the joint is, from its block.

  private final Joint joint;
  private final String label;
  private final Scheduler scheduler;

  /**
   * Why this mechanism is a stand-in for a joint that has none ({@link #standIn}), or empty for a
   * joint that is driven. A stand-in refuses every move and holds every gate false, saying this.
   */
  private final Optional<String> standsIn;

  /** Whether the joint holds a position when idle: it has a range or named positions. */
  private final boolean holdsPositions;

  /** Where the joint can be driven, and how fast. */
  private final Travel travel;

  private final double kA;
  private final double kV;
  private final double freeSpeed;

  /** How near a position the joint must be to arrive there, and may then stray: in its units. */
  private final MechanismControl.Bands positionBands;

  /** How near a speed it must be, and may then stray: as shares of the speed. */
  private final MechanismControl.Bands speedBands;

  /** The joint's control as the pit tunes it, read each loop. */
  private final Supplier<MechanismControl> tuned;

  /** The control the joint's motor controller last took, to notice when the pit changes it. */
  private MechanismControl retuned;

  // What it reads and drives.

  /** What the joint reads each loop. */
  private final JointReadings readings;

  /** What it tells its motor controller each loop. */
  private final JointOutput output;

  /** The robot's sensors, which the joint's limit switches and interlocks read. */
  private final Sensors sensors;

  // The stages that decide what it does.

  /** What it does when no command asks anything of it. */
  private final Idle idle;

  /** Its interlocks, beneath every command. */
  private final Interlocks interlocks;

  /** Its limit switches, beneath its interlocks. */
  private final LimitStops stops;

  /** What each of its limit switches is doubted for. */
  private final SwitchDoubts doubts;

  /** The profile its position follows. */
  private final JointProfile profile;

  /** How it finds where it is, and where it stands meanwhile. */
  private final JointHoming homing;

  /** Whether its absolute encoder can be trusted, for a joint that has one to compare. */
  private final Optional<EncoderTrust> encoderTrust;

  /** Its motor controllers that read as drawing nothing on their PDH channels while it works. */
  private final DeadChannels deadChannels;

  /** Where it is, in its places' names, as {@code At} reads. */
  private final PlaceNames places;

  private final JointLog log;

  /** The logic of its gates: whether it is where, or doing what, a gate asks. */
  private final JointGates gates;

  /** What it shows the interlocks on other joints. */
  private final JointView view =
      new JointView() {
        @Override
        public boolean answers() {
          return readings.answersThisLoop();
        }

        @Override
        public boolean knowsWhere() {
          return homing.state() == State.READY && readings.answersThisLoop();
        }

        @Override
        public double position() {
          return readings.position();
        }

        @Override
        public double velocity() {
          return readings.velocity();
        }
      };

  // This loop.

  /** When this loop began, in nanoseconds on the robot's clock. */
  private long nowNanos;

  private boolean enabled;

  /** What its commands ask of it, loop by loop. */
  private final JointRequests requests;

  /** What the joint's motors were told in the last loop that was written. */
  private Applied applied = Applied.Undriven.NOTHING;

  /**
   * The number in what the joint was last asked, by a command or by its idle, before its interlocks
   * and limit switches were applied. The log shows it as the joint's goal while the joint is not at
   * a position, and shows NaN until the joint has been asked anything.
   */
  private double lastAskedNumber = Double.NaN;

  /** The name of what a command asked that the joint is being driven at, if it has a name. */
  private Optional<String> drivenName = Optional.empty();

  /** How the last move on it ended, as logged. */
  private String outcome = "";

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
   * @param absolute the joint's absolute encoder, where the robot reads it itself ({@link
   *     Joints#readsAbsolute}), or empty. Until the encoder's first reading the joint waits,
   *     unhomed, and then tells its motor controller where it is from it. Where the encoder's
   *     port's wiring says the joint's loop closes on it, the joint closes that loop itself, each
   *     loop, on the encoder's reading.
   * @param sensors the robot's sensors. The joint's limit switches stop it, beneath every command
   *     and interlock. A switch on its motor controller's limit input is read from the controller,
   *     and handed to the sensors each loop.
   */
  JointMechanism(
      RobotMath math,
      Joint joint,
      Supplier<MechanismControl> tuned,
      JointIO io,
      Optional<AbsoluteEncoderIO> absolute,
      Sensors sensors,
      Scheduler scheduler) {
    this(math, joint, tuned, io, absolute, sensors, scheduler, Optional.empty());
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
        math,
        joint,
        () -> none,
        new JointIO() {},
        Optional.empty(),
        sensors,
        scheduler,
        Optional.of(why));
  }

  private JointMechanism(
      RobotMath math,
      Joint joint,
      Supplier<MechanismControl> tuned,
      JointIO io,
      Optional<AbsoluteEncoderIO> absolute,
      Sensors sensors,
      Scheduler scheduler,
      Optional<String> standsIn) {
    this.joint = joint;
    this.tuned = tuned;
    this.standsIn = standsIn;
    this.scheduler = scheduler;
    this.sensors = sensors;
    label = joint.label();
    String key = "Joints/" + label;
    log = new JointLog(key);
    requests = new JointRequests(label, scheduler);
    // A stand-in's joint has no control and no motors. Nothing about how it is driven is worked
    // out, since nothing drives it.
    boolean isDriven = standsIn.isEmpty();
    MechanismControl control = isDriven ? ControllerSettings.control(joint) : tuned.get();
    retuned = control;
    holdsPositions = joint.holdsPositions();

    if (isDriven) {
      travel = Travel.of(math, joint, control);
      freeSpeed = math.freeSpeed(joint);
      // The gains as the joint's motor controller has them: tuned, or else the definition's seed.
      ControllerPlan plan = ControllerPlan.of(math, joint, control);
      kA = plan.kA();
      kV = plan.kV();
    } else {
      travel =
          new Travel(
              joint,
              Double.NEGATIVE_INFINITY,
              Double.POSITIVE_INFINITY,
              Travel.Taking.AS_IT_IS,
              1,
              1,
              0);
      freeSpeed = 0;
      kA = 0;
      kV = 0;
    }
    MechanismControl.Bands kindBands = joint instanceof Joint.Revolute ? ANGLE_BANDS : LENGTH_BANDS;
    positionBands = holdsPositions ? control.bands().orElse(kindBands) : kindBands;
    speedBands = holdsPositions ? SPEED_BANDS : control.bands().orElse(SPEED_BANDS);

    idle = new Idle(label, holdsPositions);
    interlocks = new Interlocks(travel);
    List<Sensor.Switch> limits = LimitStops.switchesOf(math, joint);
    stops = new LimitStops(limits);
    doubts = new SwitchDoubts(joint, limits);
    profile = new JointProfile(travel);
    places = new PlaceNames(joint, positionBands.enter());
    deadChannels = new DeadChannels(joint, control.deadChannel());

    Optional<Driven.AbsoluteEncoder> encoder = joint.driven().absoluteEncoder();
    Optional<JointReadings.RobotReadEncoder> robotReadEncoder =
        absolute.map(encoderIo -> JointReadings.RobotReadEncoder.of(math, joint, encoderIo));
    boolean closesLoopOnEncoder =
        robotReadEncoder.isPresent() && robotReadEncoder.get().closesLoop();
    Optional<EncoderLoop> encoderLoop =
        closesLoopOnEncoder
            ? Optional.of(ControllerSettings.encoderLoop(math, joint, control))
            : Optional.empty();
    boolean hasCancoder = encoder.flatMap(Driven.AbsoluteEncoder::cancoder).isPresent();
    readings =
        new JointReadings(
            math,
            key,
            io,
            robotReadEncoder,
            JointReadings.PinnedSwitches.of(math, joint),
            sensors,
            hasCancoder);

    output = new JointOutput(math, joint, io, readings, profile, kA, encoderLoop);

    Optional<MechanismControl.Homing> recipe = holdsPositions ? control.homing() : Optional.empty();
    homing =
        new JointHoming(
            math,
            joint,
            recipe,
            stops.homingSwitch(recipe),
            robotReadEncoder.map(JointReadings.RobotReadEncoder::forHoming),
            this::found);
    if (standsIn.isPresent()) {
      homing.faultFor(standsIn.get());
    }

    gates =
        new JointGates(
            readings,
            homing,
            profile,
            positionBands,
            speedBands,
            freeSpeed,
            new JointGates.Doing() {
              @Override
              public boolean enabled() {
                return enabled;
              }

              @Override
              public Applied applied() {
                return applied;
              }

              @Override
              public Optional<String> drivenName() {
                return drivenName;
              }
            });

    boolean hasEncoderToCompare =
        isDriven && holdsPositions && encoder.isPresent() && math.absoluteEncoder(joint) != null;
    encoderTrust =
        hasEncoderToCompare
            ? Optional.of(
                new EncoderTrust(
                    label,
                    travel,
                    EncoderTrust.Wiring.of(encoder.get()),
                    new EncoderSlip(EncoderGap.of(math, joint, control)),
                    readings,
                    homing))
            : Optional.empty();
  }

  /**
   * Homing has found where the joint is. The joint's motor controller is told, and the joint's idle
   * holds it there.
   */
  private void found(double at) {
    output.tellWhereItIs(at);
    idle.holdAt(clamp(at));
  }

  private static MechanismControl.Bands bands(double enter, double stay) {
    return MechanismControl.Bands.builder().enter(enter).stay(stay).build();
  }

  /**
   * Puts an interlock on this joint. {@link Joints} does so once, as the robot is built.
   *
   * @param other what the joint the interlock's condition is about publishes, or empty for a
   *     condition on a sensor
   */
  void restrict(Interlock interlock, Optional<JointView> other) {
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
    return readings.position();
  }

  /** How fast it moves, as of this loop: radians, or meters, a second. */
  public double velocity() {
    return readings.velocity();
  }

  /** The voltage across its motor, as of this loop's logged inputs. */
  double appliedVolts() {
    return readings.appliedVolts();
  }

  /** The current through its motor's windings, as of this loop's logged inputs. */
  double statorCurrentAmps() {
    return readings.statorCurrentAmps();
  }

  /**
   * Where it's held or headed: the goal of the move driving it, or of its idle, as its interlocks
   * leave it; where it is, while it runs at an output. NaN before it's first read.
   */
  public double goal() {
    if (applied instanceof Applied.Undriven) {
      return idle.at().orElse(Double.NaN);
    }
    if (applied.wasToAPosition()) {
      return profile.goal();
    }
    return clamp(readings.position());
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

  /**
   * What holds it this loop, as logged: its interlocks, its limit switches, and, left idle with
   * nowhere to hold, a position that isn't a number. Empty for none.
   */
  public String heldBy() {
    return applied.heldBy();
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
    Optional<String> refused = refusedAsBuilt(joint.whyNotOwn(position));
    if (refused.isPresent()) {
      return Move.refusedGoTo(this, position.name(), refused);
    }
    Optional<String> name = Optional.of(position.name());
    return Move.goTo(this, joint.own(position).value(), name, position.name(), refused);
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
      Optional<String> refused =
          refusedAsBuilt(Optional.of(label + " slides: send it to a distance"));
      return Move.refusedGoTo(this, Rules.shown(angle), refused);
    }
    double radians = angle.in(Radians);
    return Move.goTo(
        this, radians, Optional.empty(), joint.shown(radians), refusedAsBuilt(Optional.empty()));
  }

  /**
   * Moves to a computed distance, as {@link #goTo(Position)} does: {@link Outcome#REFUSED} if it's
   * outside the joint's limits. A joint that turns is refused it as a place not its own is.
   *
   * @throws IllegalArgumentException if the joint turns, built as the robot starts
   */
  public Move goTo(Distance distance) {
    if (!(joint instanceof Joint.Prismatic)) {
      Optional<String> refused = refusedAsBuilt(Optional.of(label + " turns: send it to an angle"));
      return Move.refusedGoTo(this, Rules.shown(distance), refused);
    }
    double meters = distance.in(Meters);
    return Move.goTo(
        this, meters, Optional.empty(), joint.shown(meters), refusedAsBuilt(Optional.empty()));
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
    return Move.follow(this, target, refusedAsBuilt(Optional.empty()));
  }

  /** Owns the joint and holds its goal, never ending: what it does idle, as a command. */
  public Move hold() {
    return Move.hold(this, refusedAsBuilt(Optional.empty()));
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
    return run(output, Optional.empty());
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
    return run(output, Optional.of(condition));
  }

  /** Runs the joint at one of its outputs, until a condition holds if there is one. */
  private Move run(Output output, Optional<BooleanSupplier> until) {
    Optional<String> refused = refusedAsBuilt(joint.whyNotOwn(output));
    Output own = refused.isEmpty() ? joint.own(output) : output;
    return Move.run(this, own, until, refused);
  }

  /**
   * Why a move or a gate on the joint cannot be, as it is built ({@link Refusals#asBuilt}). As the
   * robot starts, the reason is thrown. Once the robot runs, it is handed back, for the move to end
   * {@link Outcome#REFUSED} on or the gate to hold false with. Every move and gate of a stand-in
   * cannot be, for the reason it stands in.
   *
   * @param why why this one cannot be, or empty when nothing is wrong with it
   * @throws IllegalArgumentException with the reason, as the robot starts
   */
  private Optional<String> refusedAsBuilt(Optional<String> why) {
    Optional<String> cannot = standsIn.isPresent() ? standsIn : why;
    return Optional.ofNullable(Refusals.asBuilt(scheduler, cannot.orElse(null)));
  }

  /**
   * Homes it again by its recipe, owning it meanwhile: {@link Outcome#ARRIVED} once it knows where
   * it is, {@link Outcome#REFUSED} if its homing fails. A joint with an absolute encoder, one
   * stowed by hand, or a roller is home already.
   */
  public Move home() {
    return Move.home(this, refusedAsBuilt(Optional.empty()));
  }

  /**
   * Asks for a voltage across its motors, this loop only: for a command with a law of its own, such
   * as a push held to a current. The loop it isn't asked again, the joint goes back to its idle.
   */
  public void setVoltage(double volts) {
    ask(new JointAsk.AtVoltage(volts), Optional.empty());
  }

  // ---- Its gates ----

  /**
   * A gate on the joint, named. Its logic is built as the gate is. Where that cannot be done (a
   * place that is not the joint's own), the gate is refused as a move is ({@link Refusals#gate}):
   * thrown as the robot starts, and once it runs, a gate that holds false and says why. Every gate
   * of a stand-in is refused.
   *
   * @param logic builds the gate's logic, throwing {@link IllegalArgumentException} with why it
   *     cannot
   * @throws IllegalArgumentException with why it cannot be, as the robot starts
   */
  private Gate gate(String name, Supplier<Gate.Logic> logic) {
    if (standsIn.isPresent()) {
      return Refusals.gate(scheduler, name, standsIn.get());
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
    return gate(name, () -> gates.at(joint.own(position).value()));
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
    return gate(name, () -> gates.runningAt(joint.own(output)));
  }

  /** Whether it knows where it is and its motor controllers answer: "pivot ready". */
  public Gate isReady() {
    return gate(label + " ready", gates::ready);
  }

  /**
   * Whether it's where it's asked to be: at its goal, within its band, with its profile there, for
   * a position (its idle's included); within its band of the speed, for a speed; running, for a
   * voltage or a duty cycle; still, stopped. "shooter at goal".
   */
  public Gate isAtGoal() {
    return gate(label + " at goal", gates::atGoal);
  }

  /**
   * Whether it's stalled: run at an output, it has moved slower than a tenth of the speed it asks
   * for, for a quarter of a second. A jam, for an unjam to start on: {@code
   * indexer.isStalled().onTrue(indexer.run(INDEXER_UNJAMMING).withTimeout(Seconds.of(0.2)))}.
   */
  public Gate isStalled() {
    return gate(label + " stalled", gates::stalled);
  }

  // ---- What moves ask of it, each loop they run ----

  /**
   * Takes a request for this loop from the command that is running ({@link JointRequests#ask}). A
   * request holds for one loop only: in a loop in which nothing asks, the joint does what it does
   * idle.
   *
   * @param name the name of what is asked for ("out", "intaking"), or empty for something computed
   */
  void ask(JointAsk ask, Optional<String> name) {
    requests.ask(ask, name);
  }

  /** What it was asked this loop that isn't a number, by whom, as logged: empty for nothing. */
  String rejected() {
    return requests.rejected();
  }

  /** Releases its alert, so a test can build another robot. */
  void close() {
    requests.close();
  }

  /** A move begins: its profile starts from where the joint is now, not where it was headed. */
  void beginMove() {
    profile.restart();
  }

  /**
   * A move to a position ended there, and the joint's idle holds where it arrived.
   *
   * <p>A move during which the joint was last driven at something else, or not at all, leaves the
   * idle alone: it decides from what the joint did.
   */
  void moveArrived(double at) {
    if (applied.wasToAPosition()) {
      idle.moveEnded(at);
    }
  }

  /**
   * A move to a position ended without getting there. The joint's idle holds where the joint comes
   * to a stop from where it is measured, so that a joint that jammed holds where it jammed and does
   * not push on toward a position it could not reach.
   *
   * <p>A move during which the joint was last driven at something else, or not at all, leaves the
   * idle alone: it decides from what the joint did.
   */
  void moveEndedShort() {
    if (applied.wasToAPosition()) {
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
   * Why moves on the joint cannot run, if they cannot: it is a stand-in for a joint with no
   * mechanism, its motor controllers are not answering, or its homing failed.
   */
  Optional<String> fault() {
    if (standsIn.isPresent()) {
      return standsIn;
    }
    if (!readings.answers()) {
      return Optional.of(label + " isn't answering: its motor controllers are silent");
    }
    return homing.fault();
  }

  /**
   * Whether a move on the joint must wait for it to know where it is. Asking also starts a canceled
   * homing again, if the robot is enabled ({@link JointHoming#moveWaits}).
   */
  boolean homesFirst() {
    return homing.moveWaits(nowNanos, enabled);
  }

  /** Whether it homes by moving, into a hard stop or onto a switch, and so can be homed again. */
  boolean homesToAStop() {
    return homing.homesByMoving();
  }

  /** Whether it finds where it is from an absolute encoder the robot reads, and can again. */
  boolean homesByEncoder() {
    return homing.homesByEncoder();
  }

  /**
   * Starts homing it again by its recipe, or from its absolute encoder's next reading: a command
   * asked ({@link #home()}).
   */
  void startHoming() {
    homing.homeAgain(nowNanos, enabled);
  }

  /** A command homing it was canceled: where it is isn't known. */
  void stopHoming() {
    homing.canceled();
  }

  /**
   * Why the joint cannot be sent to a position, if it cannot: the position is outside its limits,
   * at every turn of it for a joint whose range spans a turn or more ({@link
   * Travel#whyUnreachable}).
   */
  Optional<String> whyUnreachable(double position) {
    return Optional.ofNullable(travel.whyUnreachable(position, readings.position()));
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
   *
   * <p>A position that is not a number is handed back as it is, to be refused when it is asked for.
   * Brought inside the limits, an infinite position would become a limit, and be obeyed.
   */
  double reachable(double position) {
    if (!Double.isFinite(position)) {
      return position;
    }
    return travel.clamp(travel.turnNear(position, readings.position()));
  }

  /**
   * Whether the move to a position has it there: its profile has reached the position, nothing
   * holds it short, and the joint is within its band of it.
   */
  boolean settledAt(double position) {
    return gates.settledAt(position);
  }

  /** Whether it runs at a speed, within its band. */
  boolean atSpeed(double velocity) {
    return gates.runsAt(velocity);
  }

  /** How long its profile takes from where it is to a position, at a share of its cruise. */
  double secondsTo(double position, double cruiseShare) {
    return profile.secondsTo(position, cruiseShare, readings.position(), readings.velocity());
  }

  /** How long its profile takes from one position to another, from rest, at its full cruise. */
  double secondsBetween(double from, double to) {
    return profile.secondsTo(to, 1, from, 0);
  }

  /** How long its velocity loop takes to come within its band of a new speed, from the model. */
  double secondsToSpeed() {
    boolean hasAVelocityGain = kV > 0;
    if (!hasAVelocityGain) {
      return 0;
    }
    return kA / kV * Math.log(1 / speedBands.enter());
  }

  /** A position inside its limits. */
  double clamp(double position) {
    return travel.clamp(position);
  }

  /** Whether a measure is a position the joint takes: an angle if it turns, a distance if not. */
  boolean isPosition(Measure<?> measure) {
    if (joint instanceof Joint.Revolute) {
      return measure instanceof Angle;
    }
    return measure instanceof Distance;
  }

  /** Whether a measure is a speed the joint runs at. */
  boolean isSpeed(Measure<?> measure) {
    if (joint instanceof Joint.Revolute) {
      return measure instanceof AngularVelocity;
    }
    return measure instanceof LinearVelocity;
  }

  // ---- Each loop: read ----

  /**
   * Starts a loop: reads the joint's hardware, and tells the joint's homing what was read. This
   * runs before the commands do, so that they judge this loop's readings.
   */
  void read(long nowNanos, boolean enabled) {
    this.nowNanos = nowNanos;
    requests.newLoop();
    takeTuningFromThePit();
    readings.read();

    boolean enabling = enabled && !this.enabled;
    if (readings.isFirstLoop()) {
      homing.robotStarted(nowNanos);
    }
    if (readings.controllerRestarted()) {
      // The leading motor controller restarted, and counts from zero, or from the encoder it
      // reads, which may be a turn or more from where it counted a joint that turns past a turn.
      // The joint's profile starts again from where the joint is now read, and not from a point
      // that may be a turn away.
      homing.controllerRestarted(nowNanos);
      profile.restart();
    }
    faultIfSettingsWereNotTaken();
    boolean hasNowhereToHold = idle.at().isEmpty();
    if (hasNowhereToHold || enabling) {
      // First read, or just enabled: the joint's idle holds where the joint is, and not where the
      // joint was left.
      holdWhereItIs();
    }
    this.enabled = enabled;

    tellHomingWhatItsEncoderRead();
    if (encoderTrust.isPresent()
        && encoderTrust.get().check(nowNanos, enabled) == EncoderTrust.Change.TRUSTED_AGAIN) {
      holdWhereItIs();
    }
    homing.advance(nowNanos, enabled, enabling, readings.velocity());
  }

  /** Takes the joint's control from the pit, if it was tuned since the last loop. */
  private void takeTuningFromThePit() {
    MechanismControl control = tuned.get();
    if (control.equals(retuned)) {
      return;
    }
    retuned = control;
    output.retune(control);
  }

  /**
   * Faults the joint if one of its motor controllers did not take its settings at startup. Such a
   * controller has no current limits and no ratio, so driving it would be a guess at full power,
   * and nothing does.
   */
  private void faultIfSettingsWereNotTaken() {
    if (readings.unconfigured().isEmpty() || homing.state() == State.FAULTED) {
      return;
    }
    homing.faultFor(
        possessive(label)
            + " "
            + readings.unconfigured()
            + " didn't take its settings: check its CAN ID and wiring, then restart the robot");
  }

  /**
   * From now on the joint's idle holds where the joint is, and its profile starts from there.
   *
   * <p>A joint whose position reads as something that is not a number has nowhere to hold, and its
   * idle is told only that. Its profile is left alone: there is nowhere to start it from, and this
   * is asked again every loop until the joint reads a number.
   */
  private void holdWhereItIs() {
    double at = clamp(readings.position());
    idle.holdAt(at);
    if (!Double.isNaN(at)) {
      profile.restart();
    }
  }

  /**
   * Tells the joint's homing what the absolute encoder the robot reads has to say this loop: its
   * reading, while the joint waits for one to know where it is, and whether it answers at all, for
   * a joint whose loop closes on it.
   */
  private void tellHomingWhatItsEncoderRead() {
    if (readings.hasRobotReadEncoder() && homing.awaitsEncoder()) {
      OptionalDouble reading =
          readings.encoderSaysWhere()
              ? OptionalDouble.of(readings.positionByEncoder())
              : OptionalDouble.empty();
      homing.encoderRead(nowNanos, reading);
    }
    if (output.closesLoopOnEncoder()) {
      homing.loopEncoderWatched(
          nowNanos,
          enabled,
          readings.encoderAnswers(),
          stoppingPoint(),
          readings.positionByController());
    }
  }

  /**
   * What the joint does with this loop's sensors, which are read after the joints. Homing to a
   * switch moves on here, so that it acts on the switch's reading from this loop and not the last,
   * and each limit switch is checked against where the joint is.
   */
  void sensed() {
    homing.switchRead(nowNanos, readings.position(), sensors);
    boolean knowsWhere = homing.state() == State.READY && readings.answers();
    doubts.check(nowNanos, knowsWhere, readings.position(), sensors);
    if (doubts.logged().length > 0) {
      log.switchDoubts(doubts.logged());
    }
  }

  /**
   * What the joint does with this loop's PDH reading, which is read after its sensors. It logs the
   * current its motors draw together on their channels, beside its leading controller's own reading
   * ({@code SupplyCurrentAmps}), and flags each of its motor controllers that reads as drawing
   * nothing while the joint works ({@link DeadChannels}). A joint whose block names no PDH channel
   * does neither.
   */
  void powered(Power power) {
    if (!deadChannels.any()) {
      return;
    }
    double togetherAmps =
        deadChannels.check(nowNanos, power, readings.answers(), readings.supplyCurrentAmps());
    log.power(togetherAmps, deadChannels.logged());
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
    return readings.restarts();
  }

  /**
   * What is doubted of the joint's sensors this loop: each of its limit switches that disagrees
   * with where the joint is, and its own position while that reads as something that is not a
   * number. None while they agree and the position is a number.
   */
  List<String> switchDoubts() {
    List<String> doubted = doubts.doubts();
    if (Double.isNaN(readings.position())) {
      doubted.add(
          possessive(label)
              + " position reads NaN, which isn't a number: "
              + label
              + " is neutral while it has nowhere to hold; check its encoder and its motor"
              + " controller's settings");
    }
    return doubted;
  }

  // ---- Each loop: decide and write ----

  /**
   * Ends a loop: decides what the joint does and tells its motor controller. This runs after the
   * commands, so that what a command asked this loop is applied this loop.
   *
   * <p>A joint that may be driven does what a command asked, or what it does idle, as its
   * interlocks and then its limit switches leave it. A joint that may not be driven is stopped,
   * unless its homing has something for it to do.
   */
  void write() {
    if (mayBeDriven()) {
      Demand demand = decide();
      output.drive(demand);
      applied = demand;
    } else {
      applied = writeUndriven();
    }
    log();
  }

  /**
   * Whether the joint may be driven where it is asked this loop. It may not while the robot is
   * disabled. Nor may a joint that holds positions while it does not know where it is, since
   * driving it to a position would be driving it on a guess. Nor may any joint in the loop it found
   * where it is, because it reads its new position only from the next loop.
   */
  private boolean mayBeDriven() {
    if (!enabled || homing.justFound()) {
      return false;
    }
    boolean knowsWhereItIs = homing.state() == State.READY;
    return knowsWhereItIs || !holdsPositions;
  }

  /**
   * What the joint is to do this loop: what a command asked, or what the joint does idle, and then
   * what its interlocks and its limit switches leave of that.
   */
  private Demand decide() {
    Demand wanted = wanted();
    lastAskedNumber = Asks.number(wanted.ask());
    Demand allowed =
        interlocks.apply(
            wanted, readings.position(), readings.velocity(), readings.acceleration(), sensors);
    // A tripped limit switch comes last, beneath even the interlocks: nothing drives the joint on
    // toward the end of its travel.
    return stops.apply(allowed, readings.position(), sensors);
  }

  /** What a command asked this loop, or, when none did, what the joint does idle. */
  private Demand wanted() {
    Optional<JointRequests.Request> requested = requests.thisLoop();
    if (requested.isEmpty()) {
      drivenName = Optional.empty();
      return idle.demand(applied, clamp(readings.position()));
    }
    JointRequests.Request asked = requested.get();
    idle.commandAsked(asked.demand().ask());
    drivenName = asked.name();
    return asked.demand();
  }

  /**
   * Writes a loop in which the joint may not be driven where it is asked. Three things can still
   * happen to it, and all need the robot enabled: its homing pushes it toward its stop or its
   * switch; having lost the encoder its loop closes on, it holds where its motor's count says, on
   * its motor controller's own loop, so that it does not go limp; or it is stopped.
   */
  private Applied writeUndriven() {
    Applied told = Applied.Undriven.NOTHING;
    OptionalDouble heldOnMotor = homing.heldOnMotor();
    if (enabled && homing.state() == State.HOMING) {
      output.pushHome(homing.pushVolts());
      told = Applied.Undriven.HOMING;
    } else if (enabled && heldOnMotor.isPresent()) {
      output.holdOnMotor(heldOnMotor.getAsDouble());
    } else {
      output.stop();
    }
    homing.foundLoopWritten();
    drivenName = Optional.empty();
    return told;
  }

  /** Where the joint comes to a stop from where it is and how fast, braking at its acceleration. */
  private double stoppingPoint() {
    return travel.stoppingPoint(readings.position(), readings.velocity());
  }

  // ---- The log ----

  /** Logs what the joint was asked and what it did this loop. */
  private void log() {
    double goal = applied.wasToAPosition() ? profile.goal() : lastAskedNumber;
    log.did(homing.shown(), goal, where(), applied.heldBy(), outcome);
    log.asked(requests.askerShown(), requests.nameShown(), requests.rejected());
    log.setpoint(profile.position(), profile.velocity());
    if (readings.encoderSaysWhere()) {
      log.encoderAt(readings.positionByEncoder());
    } else if (!Double.isNaN(readings.positionByControllersEncoderOrNaN())) {
      // An encoder the motor controller reads: where it says the joint is, as the controller read
      // it.
      log.encoderAt(readings.positionByControllersEncoderOrNaN());
    }
  }

  /**
   * Where the joint is, as the log's {@code At} says it. A joint that holds positions is said in
   * its names for them: "at out", "between run and out", "below in", or a number if it has no
   * names, and "unknown" while it does not know. A joint that only runs at outputs, a roller, is
   * said by what it runs at.
   */
  private String where() {
    if (!holdsPositions) {
      if (drivenName.isPresent()) {
        return drivenName.get();
      }
      return applied.wasNeutral() ? "stopped" : "idle";
    }
    if (homing.state() != State.READY) {
      return "unknown";
    }
    return places.at(readings.position());
  }
}

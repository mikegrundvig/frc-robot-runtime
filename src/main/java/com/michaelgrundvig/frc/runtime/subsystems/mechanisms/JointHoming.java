package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism.State;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.Locale;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalLong;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Time;

/**
 * How a joint finds where it is, and where it stands meanwhile: one of a few phases, each with its
 * own data ({@link Phase}), each a {@link State} its mechanism logs. A joint homes by its control's
 * recipe, pushed into a hard stop or driven onto its switch on the first enable; or from an
 * absolute encoder the robot reads itself, at its first reading. One with neither, or one that only
 * runs at outputs, is ready from the start.
 *
 * <p>Its mechanism hands it what it reads, a loop at a time, in the order the loop reads them; it
 * tells the joint where it is through {@link Found}, and its mechanism pushes the joint home at
 * {@link #volts} while it homes.
 */
final class JointHoming {
  /** Homing into a hard stop: the joint has found it once it has moved slower than this share... */
  static final double STILL_SHARE = 0.02;

  /** ...of its free speed for this long. */
  static final long STILL_NANOS = 250_000_000L;

  /**
   * A joint that knows where it is from an absolute encoder the robot reads waits this long for the
   * encoder's first reading before it's faulted, in nanoseconds: the encoder still seeds it when
   * the reading comes.
   */
  static final long ABSOLUTE_WAIT_NANOS = 2_000_000_000L;

  /** Tells the joint where it is: its motor controller, and where its idle holds it. */
  @FunctionalInterface
  interface Found {
    void at(double position);
  }

  /** Where the joint stands, as one of its states with what it needs to go on. */
  sealed interface Phase {
    /** The state its mechanism logs. */
    State state();
  }

  /** It knows where it is, or only runs at outputs. */
  record Ready() implements Phase {
    @Override
    public State state() {
      return State.READY;
    }
  }

  /**
   * It found where it is this loop, and is ready: its motor's new count is read from the next, so
   * nothing drives it until then.
   */
  record JustFound() implements Phase {
    @Override
    public State state() {
      return State.READY;
    }
  }

  /** It homes by its recipe on the next enable. */
  record Unhomed() implements Phase {
    @Override
    public State state() {
      return State.UNHOMED;
    }
  }

  /**
   * It waits for its absolute encoder's reading, since a time, to know where it is.
   *
   * @param since when it began to wait, in nanoseconds
   * @param held where it holds by its motor's count meanwhile, if the encoder its loop closes on
   *     was lost: on its motor controller's own loop, rather than going limp
   */
  record Awaiting(long since, OptionalDouble held) implements Phase {
    @Override
    public State state() {
      return State.UNHOMED;
    }
  }

  /**
   * It's pushed into its hard stop, since a time.
   *
   * @param why toward what, and for how long at most
   * @param since when it began, in nanoseconds
   * @param still since when it has been still, if it is
   */
  record ToStop(String why, long since, OptionalLong still) implements Phase {
    @Override
    public State state() {
      return State.HOMING;
    }
  }

  /** How homing to a switch is getting on. */
  enum Approach {
    /** It hasn't yet seen its switch: it decides from the first reading whether to back off. */
    STARTING,
    /** It started on its switch, past where it trips, and backs off it until it clears. */
    BACKING_OFF,
    /** It's driven toward its switch, until it trips. */
    HEADING_IN
  }

  /**
   * It's driven onto its switch, since a time.
   *
   * @param why toward what, and for how long at most
   * @param since when it began, in nanoseconds
   * @param approach how it's getting on
   * @param last heading in, where it read the loop before: the switch trips between there and where
   *     it reads now
   */
  record ToSwitch(String why, long since, Approach approach, double last) implements Phase {
    @Override
    public State state() {
      return State.HOMING;
    }
  }

  /**
   * Its moves are refused, saying why, until it homes again.
   *
   * @param why what's wrong, and what to do
   * @param awaiting whether its absolute encoder's reading is still awaited: one tells its motor
   *     where it is, though it stays faulted
   */
  record Faulted(String why, boolean awaiting) implements Phase {
    @Override
    public State state() {
      return State.FAULTED;
    }
  }

  /**
   * Its moves are refused, saying why, while the absolute encoder it knows where it is by is
   * silent: the encoder's first reading readies it again.
   *
   * @param why what's wrong
   * @param since when it began to wait for the reading, in nanoseconds
   * @param held where it holds by its motor's count meanwhile, if its loop closes on the encoder
   */
  record Silent(String why, long since, OptionalDouble held) implements Phase {
    @Override
    public State state() {
      return State.FAULTED;
    }
  }

  /**
   * Its moves are refused, saying why, while its absolute encoder isn't trusted: a CANcoder that
   * says its magnet is bad, or an encoder and its motor's count apart, one of them slipped. It
   * holds where its motor says meanwhile, where its motor controller's loop runs on its motor's
   * count; trusted again, it's ready ({@link #trust}).
   *
   * @param why what's wrong, and what to do
   * @param held where it holds by its motor's count meanwhile, if it can
   */
  record Distrusted(String why, OptionalDouble held) implements Phase {
    @Override
    public State state() {
      return State.FAULTED;
    }
  }

  private final Joint joint;
  private final RobotMath math;
  private final String label;

  /** Its homing recipe, or null for one that needs none: its absolute encoder, or a roller. */
  private final MechanismControl.@Nullable Homing recipe;

  /**
   * How long its homing may push it before it's faulted, in seconds: its recipe's timeout, or one
   * worked out from its travel ({@link MechanismControl.Homing#secondsAllowed}); NaN for none.
   */
  private final double seconds;

  /** Whether it finds where it is from an absolute encoder the robot reads, and can again. */
  private final boolean byEncoder;

  /** Whether its loop closes on that encoder, so it's read where the joint is from it at once. */
  private final boolean closesLoop;

  /** The encoder's port, for what's said of it: ", on port 1"; empty for none. */
  private final String onPort;

  private final double freeSpeed;
  private final Found found;

  private Phase phase = new Ready();

  /**
   * @param recipe its control's homing recipe, or null
   * @param byEncoder whether it finds where it is from an absolute encoder the robot reads
   * @param closesLoop whether its loop closes on that encoder
   * @param port the encoder's port, or -1 for one with none, read only by the simulator
   * @param found tells the joint where it is, once homing finds it
   */
  JointHoming(
      RobotMath math,
      Joint joint,
      MechanismControl.@Nullable Homing recipe,
      boolean byEncoder,
      boolean closesLoop,
      int port,
      Found found) {
    this.joint = joint;
    this.math = math;
    label = joint.label();
    this.recipe = recipe;
    seconds =
        switch (recipe) {
          case MechanismControl.Homing.ToHardStop toStop -> toStop.secondsOn(math, joint);
          case MechanismControl.Homing.ToSwitch toSwitch -> toSwitch.secondsOn(math, joint);
          case null, default -> Double.NaN;
        };
    this.byEncoder = byEncoder;
    this.closesLoop = closesLoop;
    onPort = port < 0 ? "" : ", on port " + port;
    // Only a hard stop's stillness reads it: a joint homed otherwise needn't have motors to have
    // it.
    freeSpeed =
        recipe instanceof MechanismControl.Homing.ToHardStop ? math.freeSpeed(joint) : Double.NaN;
    this.found = found;
  }

  /** Where it stands. */
  Phase phase() {
    return phase;
  }

  State state() {
    return phase.state();
  }

  /**
   * Where it stands, as {@code State} logs it: why, when it's faulted, and while it homes, toward
   * what and for how long at most ("HOMING: toward its lower stop, for at most 9.5 s").
   */
  String shown() {
    return switch (phase) {
      case ToStop toStop -> "HOMING: " + toStop.why();
      case ToSwitch toSwitch -> "HOMING: " + toSwitch.why();
      case Faulted faulted -> "FAULTED: " + faulted.why();
      case Silent silent -> "FAULTED: " + silent.why();
      case Distrusted distrusted -> "FAULTED: " + distrusted.why();
      default -> phase.state().name();
    };
  }

  /** Why it's faulted, or null when it isn't. */
  @Nullable String fault() {
    return switch (phase) {
      case Faulted faulted -> faulted.why();
      case Silent silent -> silent.why();
      case Distrusted distrusted -> distrusted.why();
      default -> null;
    };
  }

  /** Whether moves on it wait: it's homing, or will. */
  boolean waits() {
    State state = phase.state();
    return state == State.UNHOMED || state == State.HOMING;
  }

  /**
   * A move on it waits for it to know where it is: whether it must, homing or unhomed. Unhomed with
   * the robot enabled, as a canceled homing leaves it, a joint whose recipe homes it homes again
   * now, so the move waits no longer than its homing may take, and a homing that faults refuses it.
   * Disabled, it homes on the next enable, as it would anyway.
   *
   * @param limit the switch it homes to, if it does
   */
  boolean awaited(long now, boolean enabled, Sensor.@Nullable Switch limit) {
    if (enabled && phase instanceof Unhomed && toAStop()) {
      begin(now, limit);
    }
    return waits();
  }

  /** Whether it homes by moving, into a hard stop or onto a switch, and so can be homed again. */
  boolean toAStop() {
    return recipe instanceof MechanismControl.Homing.ToHardStop
        || recipe instanceof MechanismControl.Homing.ToSwitch;
  }

  /** Whether it finds where it is from an absolute encoder the robot reads, and can again. */
  boolean byEncoder() {
    return byEncoder;
  }

  /**
   * Whether it found where it is this loop: its motor's new count is read from the next, so nothing
   * drives it this loop.
   */
  boolean justFound() {
    return phase instanceof JustFound;
  }

  /** The end of its travel whose switch it homes to, or null when it doesn't home to a switch. */
  Sensor.Switch.Limit.@Nullable End switchEnd() {
    return recipe instanceof MechanismControl.Homing.ToSwitch toSwitch ? toSwitch.end() : null;
  }

  /** Whether it waits for its absolute encoder's reading. */
  boolean awaiting() {
    return switch (phase) {
      case Awaiting awaiting -> true;
      case Silent silent -> true;
      case Faulted faulted -> faulted.awaiting();
      default -> false;
    };
  }

  /**
   * Where it holds by its motor's count while the absolute encoder its loop closes on is lost, if
   * it is.
   */
  OptionalDouble heldOnMotor() {
    return switch (phase) {
      case Awaiting awaiting -> awaiting.held();
      case Silent silent -> silent.held();
      case Distrusted distrusted -> distrusted.held();
      default -> OptionalDouble.empty();
    };
  }

  // ---- A loop's readings, in the order its mechanism reads them ----

  /**
   * Its first loop: it homes on the first enable, by its recipe, or waits for its encoder's
   * reading; with neither, it's ready.
   */
  void start(long now) {
    phase = toAStop() || byEncoder ? new Unhomed() : new Ready();
    if (byEncoder) {
      seekAgain(now);
    }
  }

  /**
   * Nothing may drive it, and its moves are refused saying why: its motor controllers didn't take
   * their settings at startup, without whose current limits or ratio driving it would be a guess at
   * full power; or it's a stand-in for a joint with no mechanism ({@link JointMechanism#standIn}).
   *
   * @param why what's wrong, and what to do
   */
  void faulted(String why) {
    if (phase.state() != State.FAULTED) {
      phase = new Faulted(why, phase instanceof Awaiting);
    }
  }

  /**
   * Waits for its absolute encoder's next reading, unhomed, to tell its motor where it is; or,
   * faulted otherwise, to tell its motor still, faulted.
   */
  private void seekAgain(long now) {
    phase =
        switch (phase) {
          case Faulted faulted -> new Faulted(faulted.why(), true);
          default -> new Awaiting(now, heldOnMotor());
        };
  }

  /**
   * While it waits for its absolute encoder: a reading tells the joint where it is, and readies it
   * unless it's faulted for something else; with none for a while, it's faulted, and the reading
   * still readies it when it comes.
   *
   * @param answers whether the encoder gave a reading this loop
   * @param at where the encoder says the joint is, when it answers
   */
  void seek(long now, boolean answers, double at) {
    if (!awaiting()) {
      return;
    }
    if (answers) {
      found.at(at);
      phase =
          switch (phase) {
            case Faulted faulted -> new Faulted(faulted.why(), false);
            // Its motor's new count is read from the next loop; a joint whose loop closes on its
            // encoder reads where it is from the encoder this loop already.
            default -> closesLoop ? new Ready() : new JustFound();
          };
    } else if (phase instanceof Awaiting awaiting
        && now - awaiting.since() >= ABSOLUTE_WAIT_NANOS) {
      phase =
          new Silent(
              String.format(
                  Locale.ROOT,
                  "%s's absolute encoder gave no reading in %.1f s%s: it's seeded when it does",
                  label,
                  ABSOLUTE_WAIT_NANOS / 1e9,
                  onPort),
              awaiting.since(),
              awaiting.held());
    }
  }

  /**
   * Whether the absolute encoder its loop closes on was lost this loop: then it's faulted, so its
   * moves are refused and the interlocks that read it hold, as for a motor controller gone silent,
   * and it holds where it comes to a stop by its motor's count, on its motor controller's own loop,
   * rather than going limp; it doesn't know where it is, so it's driven nowhere else. It waits for
   * the encoder as it did at startup, and is ready again at its first reading ({@link #seek}).
   * Disabled meanwhile, it holds wherever it is when it's enabled.
   *
   * @param answers whether the encoder gave a reading this loop
   * @param stopping where it comes to a stop, by its motor's count, from how fast it's going
   * @param onMotor where its motor's count says it is
   */
  void watch(long now, boolean enabled, boolean answers, double stopping, double onMotor) {
    if (phase.state() == State.READY && !answers) {
      phase =
          new Silent(
              possessive(label)
                  + " absolute encoder is lost"
                  + onPort
                  + ": its loop closes on it, so it holds where its motor says until the encoder"
                  + " answers",
              now,
              OptionalDouble.of(stopping));
    } else if (!enabled && heldOnMotor().isPresent()) {
      phase =
          switch (phase) {
            case Awaiting awaiting -> new Awaiting(awaiting.since(), OptionalDouble.of(onMotor));
            case Silent silent ->
                new Silent(silent.why(), silent.since(), OptionalDouble.of(onMotor));
            default -> phase;
          };
    }
  }

  /**
   * Homing by its recipe, a loop on: disabled, it stops, to start again on the next enable;
   * enabled, it starts; pushed into its stop, it goes on.
   *
   * @param enabling whether the robot was enabled this loop
   * @param velocity how fast the joint moves, measured
   * @param limit the switch it homes to, if it does
   */
  void goOn(
      long now, boolean enabled, boolean enabling, double velocity, Sensor.@Nullable Switch limit) {
    if (!enabled && phase.state() == State.HOMING) {
      phase = new Unhomed();
    } else if (enabling && phase.state() == State.UNHOMED && toAStop()) {
      begin(now, limit);
    } else if (phase instanceof ToStop toStop
        && recipe instanceof MechanismControl.Homing.ToHardStop toHardStop) {
      pushedOn(now, toStop, toHardStop, velocity);
    }
  }

  /**
   * Homing to its switch, a loop on, on the switch's reading this loop: one that started on it
   * backs off until it clears, then comes back until it trips, so it's home where the switch trips,
   * not wherever past it it rested. It's as close as the joint travels in a loop at the homing's
   * speed: approach slowly where it matters.
   *
   * @param at where the joint reads
   * @param limit the switch it homes to
   */
  void sensed(long now, double at, Sensor.@Nullable Switch limit, @Nullable Sensors sensors) {
    if (!(phase instanceof ToSwitch toSwitch)
        || !(recipe instanceof MechanismControl.Homing.ToSwitch to)
        || limit == null
        || sensors == null) {
      return;
    }
    boolean tripped = sensors.holds(limit.tripped());
    Approach approach =
        switch (toSwitch.approach()) {
          // On its switch already, past where it trips: it backs off it first, so it trips on the
          // way in.
          case STARTING, BACKING_OFF -> tripped ? Approach.BACKING_OFF : Approach.HEADING_IN;
          case HEADING_IN -> Approach.HEADING_IN;
        };
    if (toSwitch.approach() == Approach.HEADING_IN && tripped) {
      // It tripped somewhere in this loop's travel: the middle is the best guess, half a loop's
      // travel at worst.
      found.at(
          ((Sensor.Switch.Limit) limit.kind()).position().orElseThrow()
              + (at - toSwitch.last()) / 2);
      phase = new JustFound();
      return;
    }
    phase = new ToSwitch(toSwitch.why(), toSwitch.since(), approach, at);
    if (now - toSwitch.since() >= Math.round(seconds * 1e9)) {
      phase =
          new Faulted(
              String.format(
                  Locale.ROOT,
                  "%s's homing didn't %s %s in %.1f s%s%s",
                  label,
                  approach == Approach.BACKING_OFF ? "back off" : "trip",
                  limit.label(),
                  seconds,
                  timedFrom(to.timeout(), to.volts()),
                  sensors.reads(limit) ? "" : ": " + limit.label() + " is lost"),
              false);
    }
  }

  /**
   * Starts homing it again by its recipe, or from its absolute encoder's next reading: a command
   * asked.
   *
   * @param limit the switch it homes to, if it does
   */
  void again(long now, boolean enabled, Sensor.@Nullable Switch limit) {
    if (byEncoder) {
      seekAgain(now);
    } else if (enabled) {
      begin(now, limit);
    } else {
      phase = new Unhomed();
    }
  }

  /**
   * Its leading motor controller restarted, and counts its motor from 0, or from the absolute
   * encoder it reads, whose IO tells it where it is again at once. Where it is, if it knew: an
   * absolute encoder the robot reads tells it again at its next reading; a homing that was going on
   * starts over, and one that had found where it is is lost, so it homes again on the next move or
   * enable, as a canceled one does; and one assumed at rest is faulted, since it's taken to be
   * there only as the robot starts.
   *
   * @param limit the switch it homes to, if it does
   */
  void restarted(long now, Sensor.@Nullable Switch limit) {
    if (byEncoder) {
      seekAgain(now);
    } else if (toAStop()) {
      if (phase.state() == State.HOMING) {
        begin(now, limit);
      } else if (phase.state() == State.READY) {
        phase = new Unhomed();
      }
    } else if (recipe instanceof MechanismControl.Homing.AssumeAtRest) {
      faulted(
          possessive(label)
              + " motor controller restarted, so where "
              + label
              + " is isn't known: it's taken to be at rest only as the robot starts. Stow it, then"
              + " restart the robot code");
    }
  }

  /**
   * Its absolute encoder isn't trusted from now on, if it knew where it was: its moves are refused,
   * saying why, and it holds where its motor says, if it can, until it's trusted again. Disabled
   * meanwhile, it holds wherever it is when it's enabled.
   *
   * @param why what's wrong, and what to do
   * @param held where it holds by its motor's count, where its motor controller's loop runs on it:
   *     where it comes to a stop
   */
  void distrust(String why, OptionalDouble held) {
    if (phase.state() == State.READY) {
      phase = new Distrusted(why, held);
    }
  }

  /**
   * Disabled while its absolute encoder isn't trusted, it holds wherever it is as it's enabled.
   *
   * @param held where its motor says it is, if it holds on its motor's count
   */
  void stillDistrusted(OptionalDouble held) {
    if (phase instanceof Distrusted distrusted) {
      phase = new Distrusted(distrusted.why(), held);
    }
  }

  /** Whether its absolute encoder isn't trusted. */
  boolean distrusted() {
    return phase instanceof Distrusted;
  }

  /** Its absolute encoder is trusted again: it's ready, where it is. */
  void trust() {
    if (phase instanceof Distrusted) {
      phase = new Ready();
    }
  }

  /** A command homing it was canceled: where it is isn't known. */
  void stop() {
    if (phase.state() == State.HOMING) {
      phase = new Unhomed();
    }
  }

  /** Nothing drove it the loop it found where it is: it's driven from the next. */
  void settled() {
    if (phase instanceof JustFound) {
      phase = new Ready();
    }
  }

  /** Homing's push this loop, toward its stop or its switch, in volts: none while it doesn't. */
  double volts() {
    return switch (recipe) {
      case MechanismControl.Homing.ToHardStop toStop ->
          toStop.end() == Sensor.Switch.Limit.End.LOWER ? -toStop.volts() : toStop.volts();
      case MechanismControl.Homing.ToSwitch toSwitch ->
          (toSwitch.end() == Sensor.Switch.Limit.End.LOWER ? -toSwitch.volts() : toSwitch.volts())
              * (phase instanceof ToSwitch on && on.approach() == Approach.BACKING_OFF ? -1 : 1);
      case null, default -> 0;
    };
  }

  private void begin(long now, Sensor.@Nullable Switch limit) {
    String toward =
        recipe instanceof MechanismControl.Homing.ToHardStop toStop
            ? "its " + end(toStop.end()) + " stop"
            : limit != null ? limit.label() : "its switch";
    String why = String.format(Locale.ROOT, "toward %s, for at most %.1f s", toward, seconds);
    phase =
        recipe instanceof MechanismControl.Homing.ToSwitch
            ? new ToSwitch(why, now, Approach.STARTING, Double.NaN)
            : new ToStop(why, now, OptionalLong.empty());
  }

  /**
   * Pushed into its stop, a loop on: still there long enough, it's found it; or it's out of time.
   */
  private void pushedOn(
      long now, ToStop toStop, MechanismControl.Homing.ToHardStop recipe, double velocity) {
    OptionalLong still = OptionalLong.empty();
    if (Math.abs(velocity) <= STILL_SHARE * freeSpeed) {
      if (toStop.still().isEmpty()) {
        still = OptionalLong.of(now);
      } else if (now - toStop.still().getAsLong() >= STILL_NANOS) {
        var range = joint.range().orElseThrow();
        found.at(recipe.end() == Sensor.Switch.Limit.End.LOWER ? range.lower() : range.upper());
        phase = new JustFound();
        return;
      } else {
        still = toStop.still();
      }
    }
    phase = still.equals(toStop.still()) ? toStop : new ToStop(toStop.why(), toStop.since(), still);
    if (now - toStop.since() >= Math.round(seconds * 1e9)) {
      phase =
          new Faulted(
              String.format(
                  Locale.ROOT,
                  "%s's homing found no stop in %.1f s%s, pushing toward its %s one",
                  label,
                  seconds,
                  timedFrom(recipe.timeout(), recipe.volts()),
                  end(recipe.end())),
              false);
    }
  }

  private static String end(Sensor.Switch.Limit.End end) {
    return end == Sensor.Switch.Limit.End.LOWER ? "lower" : "upper";
  }

  /**
   * Where its homing's timeout came from, when it was worked out from the joint, for a fault to
   * say: " (its travel takes 6.4 s at 2.0 V)". Empty when its recipe gave one.
   */
  private String timedFrom(Optional<Time> timeout, double volts) {
    return timeout.isPresent()
        ? ""
        : String.format(
            Locale.ROOT,
            " (its travel takes %.1f s at %.1f V)",
            MechanismControl.Homing.travelSeconds(math, joint, volts),
            volts);
  }
}

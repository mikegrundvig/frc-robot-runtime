package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.Approach;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.AwaitingEncoder;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.EncoderDistrusted;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.EncoderSilent;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.Faulted;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.JustFound;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.PushingToStop;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.Ready;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.SeekingSwitch;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.HomingPhase.Unhomed;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism.State;
import com.michaelgrundvig.frc.world.record.JointState;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Sensor.Switch.Limit.End;
import java.util.Locale;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.OptionalLong;
import org.wpilib.units.measure.Time;

/**
 * How a joint finds out where it is, as a state machine. The states are the {@link HomingPhase}s,
 * and the joint's mechanism reports what happens each loop through the methods here, each of which
 * may move the joint to another phase.
 *
 * <p>A joint finds where it is in one of three ways:
 *
 * <ul>
 *   <li><b>By moving</b>, as its control's homing recipe says: it is pushed into a hard stop, or
 *       driven onto a limit switch, when the robot is first enabled.
 *   <li><b>From an absolute encoder the robot reads itself</b>: the encoder's first reading says
 *       where the joint is.
 *   <li><b>Not at all</b>: a joint with neither, or one that only runs at outputs, is ready from
 *       the start. So is one whose motor controller reads its absolute encoder for it.
 * </ul>
 *
 * <p>The machine, with the method that causes each step:
 *
 * <pre>
 *   Unhomed ──enabled──▶ PushingToStop ──still for a quarter second──▶ JustFound ──▶ Ready
 *      ▲                 SeekingSwitch ──the switch trips on the way in──▶ JustFound ──▶ Ready
 *      │                      │
 *      └──disabled, canceled──┘──out of time──▶ Faulted
 *
 *   Unhomed ──enabled, homing to a switch it does not have──▶ Faulted (never pushed)
 *
 *   AwaitingEncoder ──a reading──▶ JustFound ──▶ Ready
 *         │                                        │ the encoder its loop closes on is lost
 *         └──none for 2 s──▶ EncoderSilent ◀───────┘
 *                                 └──a reading──▶ Ready
 *
 *   Ready ──its encoder is not trusted──▶ EncoderDistrusted ──trusted again──▶ Ready
 * </pre>
 *
 * <p>When homing finds the joint, it tells the joint's motor controller and its idle through {@link
 * Found}. While the joint homes by moving, its mechanism pushes it at {@link #pushVolts}.
 */
final class JointHoming {
  /**
   * A joint pushed into a hard stop has found it once it has moved slower than this share of its
   * free speed for {@link #STILL_NANOS}.
   */
  static final double STILL_SHARE = 0.02;

  /** How long a joint pushed into a hard stop must be still to have found it. */
  static final long STILL_NANOS = 250_000_000L;

  /**
   * How long a joint waits for its absolute encoder's first reading before it is faulted, in
   * nanoseconds. The encoder still says where the joint is when the reading does come.
   */
  static final long ABSOLUTE_WAIT_NANOS = 2_000_000_000L;

  /** Tells the joint where it is: its motor controller, and where its idle holds it. */
  @FunctionalInterface
  interface Found {
    void at(double position);
  }

  /**
   * The absolute encoder a joint finds where it is from, when the robot reads that encoder itself.
   *
   * @param closesLoop whether the joint's loop closes on the encoder. Such a joint reads where it
   *     is from the encoder in the same loop the encoder first answers, with no loop to wait.
   * @param port the Systemcore port the encoder is on, which a fault names, if it is on one
   */
  record ReadEncoder(boolean closesLoop, OptionalInt port) {
    /** Where the encoder is, as a fault says it after a comma: ", on port 1". Empty for none. */
    String onPort() {
      if (port.isEmpty()) {
        return "";
      }
      return ", on port " + port.getAsInt();
    }
  }

  private final Joint joint;
  private final RobotMath math;
  private final String label;

  /** The joint's homing recipe, if its control has one. */
  private final Optional<Homing> recipe;

  /** The recipe again, when it is one that pushes the joint into a hard stop. */
  private final Optional<Homing.ToHardStop> toHardStop;

  /** The recipe again, when it is one that drives the joint onto a limit switch. */
  private final Optional<Homing.ToSwitch> toSwitch;

  /** The limit switch the joint homes to, if it homes to one and has it. */
  private final Optional<Sensor.Switch> homingSwitch;

  /**
   * How long homing by moving may take before the joint is faulted, in seconds: the recipe's own
   * timeout, or one worked out from the joint's travel. Not a number for a joint that does not home
   * by moving, which never reads it.
   */
  private final double secondsAllowed;

  /** The encoder the joint finds where it is from, if the robot reads one for it. */
  private final Optional<ReadEncoder> encoder;

  /** The speed a full battery runs the joint at, which says how slow counts as still. */
  private final double freeSpeed;

  private final Found found;

  private HomingPhase phase = new Ready();

  /**
   * @param recipe the homing recipe of the joint's control, if it has one
   * @param homingSwitch the limit switch the joint homes to, if its recipe homes it to one
   * @param encoder the absolute encoder that says where the joint is, if the robot reads one
   * @param found tells the joint where it is, once homing finds it
   */
  JointHoming(
      RobotMath math,
      Joint joint,
      Optional<Homing> recipe,
      Optional<Sensor.Switch> homingSwitch,
      Optional<ReadEncoder> encoder,
      Found found) {
    this.joint = joint;
    this.math = math;
    label = joint.label();
    this.recipe = recipe;
    toHardStop =
        recipe.filter(Homing.ToHardStop.class::isInstance).map(Homing.ToHardStop.class::cast);
    toSwitch = recipe.filter(Homing.ToSwitch.class::isInstance).map(Homing.ToSwitch.class::cast);
    this.homingSwitch = homingSwitch;
    this.encoder = encoder;
    this.found = found;
    if (toHardStop.isPresent()) {
      secondsAllowed = toHardStop.get().secondsOn(math, joint);
      // Only a push into a hard stop asks how fast the joint could go. A joint that homes another
      // way need not have motors in its definition at all.
      freeSpeed = math.freeSpeed(joint);
    } else if (toSwitch.isPresent()) {
      secondsAllowed = toSwitch.get().secondsOn(math, joint);
      freeSpeed = Double.NaN;
    } else {
      secondsAllowed = Double.NaN;
      freeSpeed = Double.NaN;
    }
  }

  // ---- What the joint's mechanism asks ----

  State state() {
    return phase.state();
  }

  /**
   * The joint's phase as {@code State} logs it: where it stands, which of the reasons it is faulted
   * for when it is, and while it homes, toward what and for how long at most.
   */
  JointState recorded() {
    return phase.recorded();
  }

  /** Why moves on the joint are refused, if it is faulted. */
  Optional<String> fault() {
    return phase.fault();
  }

  /** Whether moves on the joint wait for it: it is homing, or will. */
  boolean waits() {
    State state = phase.state();
    return state == State.UNHOMED || state == State.HOMING;
  }

  /** Whether the joint homes by moving, into a hard stop or onto a switch, and so can again. */
  boolean homesByMoving() {
    return toHardStop.isPresent() || toSwitch.isPresent();
  }

  /**
   * Whether the joint finds where it is from an absolute encoder the robot reads, and can again.
   */
  boolean homesByEncoder() {
    return encoder.isPresent();
  }

  /**
   * Whether the joint found where it is in this loop. Its motor controller reports its new count
   * only from the next loop, so nothing drives the joint in this one.
   */
  boolean justFound() {
    return phase instanceof JustFound;
  }

  /** The end of the joint's travel whose switch it homes to, if it homes to a switch. */
  Optional<End> switchEnd() {
    return toSwitch.map(Homing.ToSwitch::end);
  }

  /** Whether the joint is waiting for its absolute encoder's reading. */
  boolean awaitsEncoder() {
    return phase.awaitsEncoder();
  }

  /**
   * Where the joint holds by its motor's count while its absolute encoder cannot be used, if it
   * holds that way now.
   */
  OptionalDouble heldOnMotor() {
    return phase.heldOnMotor();
  }

  /** Whether the joint's absolute encoder is not trusted. */
  boolean distrusted() {
    return phase instanceof EncoderDistrusted;
  }

  /**
   * The voltage homing pushes the joint at in this loop, toward its stop or its switch. A joint
   * backing off its switch is pushed the other way. Zero for a joint that does not home by moving.
   */
  double pushVolts() {
    if (toHardStop.isPresent()) {
      return toward(toHardStop.get().end(), toHardStop.get().volts());
    }
    if (toSwitch.isPresent()) {
      double towardSwitch = toward(toSwitch.get().end(), toSwitch.get().volts());
      return isBackingOff() ? -towardSwitch : towardSwitch;
    }
    return 0;
  }

  // ---- What happens in a loop, in the order the joint's mechanism reports it ----

  /**
   * The robot's first loop. A joint that homes by moving is unhomed until the robot is enabled, one
   * that an encoder tells starts waiting for the encoder, and any other is ready.
   */
  void robotStarted(long now) {
    if (homesByMoving() || homesByEncoder()) {
      phase = new Unhomed();
    } else {
      phase = new Ready();
    }
    if (homesByEncoder()) {
      awaitEncoder(now);
    }
  }

  /**
   * Faults the joint, unless it is faulted already: nothing may drive it, and its moves are refused
   * saying why. This is for a joint whose motor controllers did not take their settings at startup,
   * since driving it without their current limits or its ratio would be a guess at full power, and
   * for a stand-in for a joint with no mechanism ({@link JointMechanism#standIn}).
   *
   * @param fault which of the reasons a joint is faulted for this is
   * @param why what is wrong and what to do about it
   */
  void faultFor(JointState.Fault fault, String why) {
    if (phase.state() == State.FAULTED) {
      return;
    }
    phase = new Faulted(JointState.faulted(fault), why, phase instanceof AwaitingEncoder);
  }

  /**
   * What the joint's absolute encoder read this loop, while the joint waits for it.
   *
   * <p>A reading says where the joint is. It makes the joint ready, unless the joint is faulted for
   * something else, which it then stays. With no reading for {@link #ABSOLUTE_WAIT_NANOS}, the
   * joint is faulted until a reading comes.
   *
   * @param reading where the encoder says the joint is, or empty if it gave no reading this loop
   */
  void encoderRead(long now, OptionalDouble reading) {
    if (!phase.awaitsEncoder()) {
      return;
    }
    if (reading.isPresent()) {
      found.at(reading.getAsDouble());
      phase = afterEncoderFoundIt();
      return;
    }
    boolean waitedTooLong =
        phase instanceof AwaitingEncoder awaiting && now - awaiting.since() >= ABSOLUTE_WAIT_NANOS;
    if (waitedTooLong) {
      AwaitingEncoder awaiting = (AwaitingEncoder) phase;
      phase =
          new EncoderSilent(
              JointState.faulted(JointState.Fault.ENCODER_GAVE_NO_READING),
              String.format(
                  Locale.ROOT,
                  "%s's absolute encoder gave no reading in %.1f s%s: it's seeded when it does",
                  label,
                  ABSOLUTE_WAIT_NANOS / 1e9,
                  onPort()),
              awaiting.since(),
              awaiting.heldOnMotor());
    }
  }

  /** The phase a joint moves to when its absolute encoder has just said where it is. */
  private HomingPhase afterEncoderFoundIt() {
    if (phase instanceof Faulted faulted) {
      return new Faulted(faulted.recorded(), faulted.why(), false);
    }
    // A joint whose loop closes on its encoder already reads where it is from the encoder in this
    // loop. Any other reads its motor's new count only from the next loop.
    boolean readsItAtOnce = encoder.isPresent() && encoder.get().closesLoop();
    if (readsItAtOnce) {
      return new Ready();
    }
    return new JustFound();
  }

  /**
   * Watches the absolute encoder that a joint's loop closes on, each loop.
   *
   * <p>If a ready joint's encoder gives no reading, the joint no longer knows where it is. It is
   * faulted, so its moves are refused and the interlocks that read it hold, as they do for a motor
   * controller gone silent. It holds where it comes to a stop by its motor's count, on its motor
   * controller's own loop, so that it does not go limp, and it waits for the encoder as it did at
   * startup ({@link #encoderRead}).
   *
   * <p>While the robot is disabled, a joint held this way is moved by hand, so where it holds
   * follows where its motor says it is: it holds wherever it is when the robot is enabled.
   *
   * @param encoderAnswers whether the encoder gave a reading this loop
   * @param stoppingPoint where the joint comes to a stop, by its motor's count, from its speed
   * @param onMotor where the motor's count says the joint is
   */
  void loopEncoderWatched(
      long now, boolean enabled, boolean encoderAnswers, double stoppingPoint, double onMotor) {
    boolean lostNow = phase.state() == State.READY && !encoderAnswers;
    if (lostNow) {
      phase =
          new EncoderSilent(
              JointState.faulted(JointState.Fault.ENCODER_LOST),
              possessive(label)
                  + " absolute encoder is lost"
                  + onPort()
                  + ": its loop closes on it, so it holds where its motor says until the encoder"
                  + " answers",
              now,
              OptionalDouble.of(stoppingPoint));
      return;
    }
    boolean heldWhileDisabled = !enabled && phase.heldOnMotor().isPresent();
    if (!heldWhileDisabled) {
      return;
    }
    if (phase instanceof AwaitingEncoder awaiting) {
      phase = new AwaitingEncoder(awaiting.since(), OptionalDouble.of(onMotor));
    } else if (phase instanceof EncoderSilent silent) {
      phase =
          new EncoderSilent(
              silent.recorded(), silent.why(), silent.since(), OptionalDouble.of(onMotor));
    }
  }

  /**
   * Moves homing by the joint's recipe on by one loop.
   *
   * <ul>
   *   <li>Disabled while it homes, the joint is unhomed again, and starts over on the next enable.
   *   <li>Enabled while unhomed, it starts homing.
   *   <li>Pushed into its hard stop, it has found the stop once it is still, or runs out of time.
   * </ul>
   *
   * <p>A joint driven onto its switch is moved on by {@link #switchRead}, once the sensors have
   * been read.
   *
   * @param enabling whether the robot was enabled in this loop
   * @param velocity how fast the joint is measured moving
   */
  void advance(long now, boolean enabled, boolean enabling, double velocity) {
    boolean isHoming = phase.state() == State.HOMING;
    if (!enabled && isHoming) {
      phase = new Unhomed();
      return;
    }
    boolean startsNow = enabling && phase.state() == State.UNHOMED && homesByMoving();
    if (startsNow) {
      begin(now);
      return;
    }
    if (phase instanceof PushingToStop pushing && toHardStop.isPresent()) {
      phase = pushedOn(now, pushing, toHardStop.get(), velocity);
    }
  }

  /**
   * Moves homing to a limit switch on by one loop, on what the switch reads in this loop.
   *
   * <p>A joint that starts on its switch is somewhere past the point where the switch trips, so it
   * first backs off until the switch clears. It then heads in until the switch trips, and that is
   * where it is home: at the switch's edge, not wherever past it the joint happened to rest. The
   * edge is found to within the distance the joint travels in one loop at the homing's speed, so
   * home slowly where it matters.
   *
   * @param at where the joint reads
   * @param sensors the robot's sensors, as read this loop
   */
  void switchRead(long now, double at, Sensors sensors) {
    if (!(phase instanceof SeekingSwitch seeking)) {
      return;
    }
    // A joint seeking a switch always has the switch: the phase carries it, and a joint with none
    // to seek is faulted before it sets off ({@link #begin}). So every loop of the push comes
    // through here, and the push ends when the switch trips or, at the latest, when it is out of
    // time.
    Sensor.Switch limit = seeking.onto();
    boolean tripped = sensors.holds(limit.tripped());
    if (seeking.approach() instanceof Approach.HeadingIn headingIn && tripped) {
      // The switch tripped somewhere in this loop's travel. The middle of it is the best guess,
      // and is wrong by half a loop's travel at most.
      double edge = ((Sensor.Switch.Limit) limit.kind()).position().orElseThrow();
      found.at(edge + (at - headingIn.lastRead()) / 2);
      phase = new JustFound();
      return;
    }
    Approach approach = nextApproach(seeking.approach(), tripped, at);
    boolean outOfTime = now - seeking.since() >= Math.round(secondsAllowed * 1e9);
    if (!outOfTime) {
      phase = new SeekingSwitch(seeking.recorded(), seeking.since(), approach, limit);
      return;
    }
    Homing.ToSwitch recipe = toSwitch.orElseThrow();
    phase =
        new Faulted(
            JointState.faulted(JointState.Fault.HOMING_MISSED_ITS_SWITCH),
            String.format(
                Locale.ROOT,
                "%s's homing didn't %s %s in %.1f s%s%s",
                label,
                approach instanceof Approach.BackingOff ? "back off" : "trip",
                limit.label(),
                secondsAllowed,
                timeoutWorkedOutFrom(recipe.timeout(), recipe.volts()),
                sensors.reads(limit) ? "" : ": " + limit.label() + " is lost"),
            false);
  }

  /**
   * How a joint that has not found its switch yet carries on, given what the switch reads now.
   *
   * @param at where the joint reads, which a joint heading in remembers for the next loop
   */
  private static Approach nextApproach(Approach approach, boolean tripped, double at) {
    return switch (approach) {
      // On its switch, the joint is past where the switch trips. It backs off first, so that the
      // switch trips on the way in.
      case Approach.SwitchNotYetRead notYetRead ->
          tripped ? new Approach.BackingOff() : new Approach.HeadingIn(at);
      case Approach.BackingOff backingOff -> tripped ? backingOff : new Approach.HeadingIn(at);
      case Approach.HeadingIn headingIn -> new Approach.HeadingIn(at);
    };
  }

  /**
   * A command asked for the joint to be homed again: by its recipe, or from its absolute encoder's
   * next reading. A joint that homes by moving starts now if the robot is enabled, and otherwise on
   * the next enable. A joint with nothing to home by stays as it is: there is no stop or switch to
   * push it toward, so nothing is started.
   */
  void homeAgain(long now, boolean enabled) {
    if (homesByEncoder()) {
      awaitEncoder(now);
      return;
    }
    if (!homesByMoving()) {
      return;
    }
    if (enabled) {
      begin(now);
    } else {
      phase = new Unhomed();
    }
  }

  /**
   * A move on the joint asks whether it must wait for the joint to know where it is.
   *
   * <p>Asking also restarts a homing that was canceled: an unhomed joint that homes by moving
   * starts homing now, if the robot is enabled. The move then waits no longer than the homing may
   * take, and a homing that faults refuses the move. With the robot disabled, the joint homes on
   * the next enable, as it would anyway.
   */
  boolean moveWaits(long now, boolean enabled) {
    if (enabled && phase instanceof Unhomed && homesByMoving()) {
      begin(now);
    }
    return waits();
  }

  /**
   * The joint's leading motor controller restarted, so it counts the motor from zero again, or from
   * the absolute encoder it reads itself. Whatever the joint knew of where it is, is lost:
   *
   * <ul>
   *   <li>An absolute encoder the robot reads says where it is again at its next reading.
   *   <li>A joint that homes by moving starts over if it was homing. If it had found where it is,
   *       it is unhomed, and homes again on the next move or enable, as after a canceled homing.
   *   <li>A joint assumed to be at rest is faulted. It is taken to be at rest only as the robot
   *       starts, and nothing says where it is now.
   * </ul>
   */
  void controllerRestarted(long now) {
    if (homesByEncoder()) {
      awaitEncoder(now);
      return;
    }
    if (homesByMoving()) {
      if (phase.state() == State.HOMING) {
        begin(now);
      } else if (phase.state() == State.READY) {
        phase = new Unhomed();
      }
      return;
    }
    boolean assumedAtRest = recipe.isPresent() && recipe.get() instanceof Homing.AssumeAtRest;
    if (assumedAtRest) {
      faultFor(
          JointState.Fault.CONTROLLER_RESTARTED,
          possessive(label)
              + " motor controller restarted, so where "
              + label
              + " is isn't known: it's taken to be at rest only as the robot starts. Stow it, then"
              + " restart the robot code");
    }
  }

  /**
   * The joint's absolute encoder is not trusted from now on. A joint that knew where it was is
   * faulted: its moves are refused, saying why, until the encoder is trusted again ({@link
   * #trust}).
   *
   * @param fault which of the reasons an encoder is not trusted for this is
   * @param why what is wrong and what to do about it
   * @param heldOnMotor where the joint holds by its motor's count meanwhile, which is where it
   *     comes to a stop, for a joint whose motor controller's loop runs on the motor's count
   */
  void distrust(JointState.Fault fault, String why, OptionalDouble heldOnMotor) {
    if (phase.state() == State.READY) {
      phase = new EncoderDistrusted(JointState.faulted(fault), why, heldOnMotor);
    }
  }

  /**
   * The robot is disabled while the joint's encoder is still not trusted. Where the joint holds
   * follows where its motor says it is, so it holds wherever it is when the robot is enabled.
   *
   * @param heldOnMotor where the motor says the joint is, if the joint holds on its motor's count
   */
  void stillDistrusted(OptionalDouble heldOnMotor) {
    if (phase instanceof EncoderDistrusted distrusted) {
      phase = new EncoderDistrusted(distrusted.recorded(), distrusted.why(), heldOnMotor);
    }
  }

  /** The joint's absolute encoder is trusted again, and the joint is ready where it is. */
  void trust() {
    if (phase instanceof EncoderDistrusted) {
      phase = new Ready();
    }
  }

  /** The command that was homing the joint was canceled: where the joint is, is not known. */
  void canceled() {
    if (phase.state() == State.HOMING) {
      phase = new Unhomed();
    }
  }

  /**
   * The loop in which the joint found where it is has been written, with nothing driving the joint.
   * It is driven from the next loop.
   */
  void foundLoopWritten() {
    if (phase instanceof JustFound) {
      phase = new Ready();
    }
  }

  // ---- The steps themselves ----

  /**
   * Starts waiting for the absolute encoder's next reading to say where the joint is. A joint that
   * is faulted for something else stays faulted, and the reading only tells its motor controller
   * where it is.
   */
  private void awaitEncoder(long now) {
    if (phase instanceof Faulted faulted) {
      phase = new Faulted(faulted.recorded(), faulted.why(), true);
    } else {
      phase = new AwaitingEncoder(now, phase.heldOnMotor());
    }
  }

  /**
   * Starts homing by moving: toward the hard stop, or onto the switch. Only a joint that homes by
   * moving is ever started.
   *
   * <p>Every push that starts here has a deadline: a joint pushed into its stop is timed in {@link
   * #pushedOn}, and one driven onto its switch in {@link #switchRead}. A joint whose recipe homes
   * it to a switch that it does not have could never be timed by its switch's readings, so it is
   * not started at all. It is faulted here instead, before anything pushes it.
   *
   * <p>That last case should not arise. A robot's definition is refused as it is built if a joint
   * homes to a limit switch that it lacks at that end, or whose position is not known, so no
   * mechanism is made for such a joint. The fault is here so that this class does not depend on
   * that to keep its pushes bounded.
   */
  private void begin(long now) {
    if (toHardStop.isPresent()) {
      boolean towardItsLowerEnd = toHardStop.get().end() == End.LOWER;
      JointState.Toward stop =
          towardItsLowerEnd ? JointState.Toward.LOWER_STOP : JointState.Toward.UPPER_STOP;
      phase = new PushingToStop(homingToward(stop), now, OptionalLong.empty());
      return;
    }
    if (homingSwitch.isPresent()) {
      Sensor.Switch onto = homingSwitch.get();
      boolean towardItsLowerEnd = ((Sensor.Switch.Limit) onto.kind()).end() == End.LOWER;
      JointState.Toward itsSwitch =
          towardItsLowerEnd ? JointState.Toward.LOWER_SWITCH : JointState.Toward.UPPER_SWITCH;
      phase =
          new SeekingSwitch(homingToward(itsSwitch), now, new Approach.SwitchNotYetRead(), onto);
      return;
    }
    phase =
        new Faulted(
            JointState.faulted(JointState.Fault.HOMING_HAS_NO_SWITCH),
            whyItHasNoSwitchToHomeTo(),
            false);
  }

  /** What a joint that begins to home logs: what it is pushed toward, and for how long at most. */
  private JointState homingToward(JointState.Toward toward) {
    return JointState.homing(toward, secondsAllowed);
  }

  /** Why a joint whose recipe homes it to a limit switch it does not have is faulted. */
  private String whyItHasNoSwitchToHomeTo() {
    return String.format(
        Locale.ROOT,
        "%s's homing has no %s limit switch with a known position to home to, so it isn't pushed:"
            + " give its block one where it trips",
        label,
        shown(toSwitch.orElseThrow().end()));
  }

  /**
   * The phase of a joint pushed into its hard stop, one loop on. Once it has been still for {@link
   * #STILL_NANOS} it has found the stop, and is at that end of its range. Otherwise it carries on,
   * until it is out of time.
   */
  private HomingPhase pushedOn(
      long now, PushingToStop pushing, Homing.ToHardStop recipe, double velocity) {
    boolean isStill = Math.abs(velocity) <= STILL_SHARE * freeSpeed;
    OptionalLong stillSince = stillSince(pushing, isStill, now);
    boolean stillLongEnough =
        isStill
            && pushing.stillSince().isPresent()
            && now - pushing.stillSince().getAsLong() >= STILL_NANOS;
    if (stillLongEnough) {
      var range = joint.range().orElseThrow();
      found.at(recipe.end() == End.LOWER ? range.lower() : range.upper());
      return new JustFound();
    }
    boolean outOfTime = now - pushing.since() >= Math.round(secondsAllowed * 1e9);
    if (outOfTime) {
      return new Faulted(
          JointState.faulted(JointState.Fault.HOMING_FOUND_NO_STOP),
          String.format(
              Locale.ROOT,
              "%s's homing found no stop in %.1f s%s, pushing toward its %s one",
              label,
              secondsAllowed,
              timeoutWorkedOutFrom(recipe.timeout(), recipe.volts()),
              shown(recipe.end())),
          false);
    }
    if (stillSince.equals(pushing.stillSince())) {
      return pushing;
    }
    return new PushingToStop(pushing.recorded(), pushing.since(), stillSince);
  }

  /** Since when a joint pushed into its stop has been still: from now, if it has just stopped. */
  private static OptionalLong stillSince(PushingToStop pushing, boolean isStill, long now) {
    if (!isStill) {
      return OptionalLong.empty();
    }
    if (pushing.stillSince().isPresent()) {
      return pushing.stillSince();
    }
    return OptionalLong.of(now);
  }

  private boolean isBackingOff() {
    return phase instanceof SeekingSwitch seeking
        && seeking.approach() instanceof Approach.BackingOff;
  }

  /** A homing voltage with the sign that pushes the joint toward an end of its travel. */
  private static double toward(End end, double volts) {
    return end == End.LOWER ? -volts : volts;
  }

  private static String shown(End end) {
    return end == End.LOWER ? "lower" : "upper";
  }

  /** Where the encoder is, as a fault says it after a comma: ", on port 1". Empty for none. */
  private String onPort() {
    return encoder.map(ReadEncoder::onPort).orElse("");
  }

  /**
   * Where a homing's time limit came from, for a fault to say, when it was worked out from the
   * joint and not given by the recipe: " (its travel takes 6.4 s at 2.0 V)". Empty when the recipe
   * gave it.
   */
  private String timeoutWorkedOutFrom(Optional<Time> timeout, double volts) {
    if (timeout.isPresent()) {
      return "";
    }
    return String.format(
        Locale.ROOT,
        " (its travel takes %.1f s at %.1f V)",
        Homing.travelSeconds(math, joint, volts),
        volts);
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.behavior.Debounce;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism.State;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Output;
import java.util.Optional;
import org.wpilib.math.filter.Debouncer;

/**
 * The logic of one joint's gates: whether the joint is at a place, runs at an output, is ready, is
 * at its goal, or is stalled. {@link JointMechanism} names each gate and hands it to behavior code;
 * what a gate asks each loop is answered here.
 *
 * <p>Most of these have a band with two widths ({@link MechanismControl.Bands}). A joint must come
 * inside the narrower one to arrive, and may then stray as far as the wider one before it is said
 * to have left. Without that, a joint sitting at the edge of a single band would flicker in and out
 * of it. So each such gate remembers whether it held the last time it was asked.
 *
 * <p>Some of these state a fact about the joint, which is so whoever has the joint: where it is
 * ({@link #at}), whether it knows where it is ({@link #ready}), and whether it is driven and not
 * moving ({@link #stalled}). The others say whether the joint has done what it was asked: {@link
 * #atGoal}, and for an output {@link #runningAt}, which is as much about what the joint is asked
 * for as about how fast it turns. What a joint is asked can change in the middle of a loop, when a
 * command takes it or asks it something new, and what its motors were last told then belongs to the
 * question before. So each of these holds false until the joint has been judged against the
 * question it is under now ({@link Tenure}), and tells its {@link Gate} when the question changes
 * ({@link Gate.Logic#question}), so that an answer worked out earlier in the loop is not handed on.
 */
final class JointGates {
  /** What the gates need to know of what the joint is doing, which its mechanism tells them. */
  interface Doing {
    /** Whether the robot is enabled. */
    boolean enabled();

    /** What the joint's motors were told in the last loop that was written. */
    Applied applied();

    /** The name of what a command asked that the joint is being driven at, if it has a name. */
    Optional<String> drivenName();

    /** Which question the joint is under now, as a count ({@link Tenure#question}). */
    long question();

    /**
     * A count that moves on with each new question, and again when the joint is judged against it
     * ({@link Tenure#standing}): whenever an answer worked out before may no longer stand.
     */
    long standing();

    /**
     * Whether the joint has been judged against the question it is under now: what its motors were
     * last told is that question's ({@link Tenure#isJudged}).
     */
    boolean isJudged();
  }

  /**
   * The logic of a gate on whether the joint has done what it was asked. Its question is the
   * joint's, and what it remembers of the last time it was asked, it remembers of one question
   * only.
   */
  private abstract class OfWhatWasAsked implements Gate.Logic {
    /** Whether the gate held the last time it was worked out. */
    private boolean held;

    /** The question it was worked out for then. */
    private long heldFor;

    /**
     * Whether the joint is doing what the gate is about, given that it has been judged against the
     * question it is under.
     *
     * @param wasDoingIt whether it was the last time this was asked, of the same question
     */
    abstract boolean isDoingIt(boolean wasDoingIt);

    @Override
    public final boolean holds(Gate.Terms why) {
      long question = doing.question();
      boolean wasDoingIt = held && heldFor == question;
      held = doing.isJudged() && isDoingIt(wasDoingIt);
      heldFor = question;
      return held;
    }

    /**
     * Its gate is worked out again when it is read after the joint's question has changed, or after
     * the joint has been judged against it: either changes the answer.
     */
    @Override
    public final long question() {
      return doing.standing();
    }
  }

  /** What a gate with a band remembers between the times it is asked: whether it held last time. */
  private static final class Remembered {
    boolean held;
  }

  private final JointReadings readings;
  private final JointHoming homing;
  private final JointProfile profile;
  private final MechanismControl.Bands positionBands;
  private final MechanismControl.Bands speedBands;
  private final double freeSpeed;
  private final Doing doing;

  /**
   * @param positionBands how near a position the joint must be, in its units
   * @param speedBands how near a speed it must be, as shares of the speed
   * @param freeSpeed the speed a full battery runs the unloaded joint at
   */
  JointGates(
      JointReadings readings,
      JointHoming homing,
      JointProfile profile,
      MechanismControl.Bands positionBands,
      MechanismControl.Bands speedBands,
      double freeSpeed,
      Doing doing) {
    this.readings = readings;
    this.homing = homing;
    this.profile = profile;
    this.positionBands = positionBands;
    this.speedBands = speedBands;
    this.freeSpeed = freeSpeed;
    this.doing = doing;
  }

  /** Whether the joint is measured at a position, within its band. */
  Gate.Logic at(double position) {
    Remembered inside = new Remembered();
    return why -> {
      double off = Math.abs(readings.position() - position);
      inside.held = isReady() && off <= band(positionBands, inside.held);
      return inside.held;
    };
  }

  /**
   * Whether the joint runs at one of its named outputs, within its band if the output is a speed:
   * it is asked for that output now, and has been judged against the asking.
   */
  Gate.Logic runningAt(Output output) {
    // Only a speed has a band to be inside. Any other output is run at as soon as it is applied.
    boolean isASpeed = output.kind() == Output.Kind.VELOCITY;
    return new OfWhatWasAsked() {
      @Override
      boolean isDoingIt(boolean wasRunningAtIt) {
        Optional<String> drivenName = doing.drivenName();
        boolean isAskedForIt = drivenName.isPresent() && output.name().equals(drivenName.get());
        boolean isRunningIt = doing.enabled() && isAskedForIt;
        return isRunningIt && (!isASpeed || withinSpeed(output.value(), wasRunningAtIt));
      }
    };
  }

  /** Whether the joint knows where it is and its motor controllers answer. */
  Gate.Logic ready() {
    return why -> isReady() && readings.answers();
  }

  /**
   * Whether the joint is where it is asked to be, whatever that is: at the goal of its profile for
   * a position, within its band of a speed, running for a voltage or a share of the battery, and
   * still for neutral.
   *
   * <p>What it is asked is what the command that has it asked. So a joint a command has taken is
   * not at its goal until that command, or one it started, has asked something of it and the joint
   * has been judged against that. A joint nobody has is asked what it does idle: to hold where it
   * is, or to stop.
   */
  Gate.Logic atGoal() {
    return new OfWhatWasAsked() {
      @Override
      boolean isDoingIt(boolean wasAtGoal) {
        return doing.enabled()
            && isReady()
            && doing.applied() instanceof Demand demand
            && isDoing(demand.ask(), wasAtGoal);
      }
    };
  }

  /**
   * Whether the joint is stalled: run at an output, it has moved slower than {@link
   * JointMechanism#STALL_SHARE} of the speed the output asks for, for {@link
   * JointMechanism#STALL_SECONDS}.
   */
  Gate.Logic stalled() {
    Debounce stalling = new Debounce(JointMechanism.STALL_SECONDS, Debouncer.DebounceType.RISING);
    return why -> {
      double asked = speedAsked();
      boolean isTooSlow = Math.abs(readings.velocity()) < JointMechanism.STALL_SHARE * asked;
      return stalling.calculate(doing.enabled() && asked > 0 && isTooSlow);
    };
  }

  /**
   * Whether a move to a position has the joint there: the joint was driven to exactly that position
   * with nothing holding it short, its profile has arrived, and it is measured inside the band to
   * arrive in.
   */
  boolean settledAt(double position) {
    return doing.applied() instanceof Demand demand
        && demand.ask() instanceof JointAsk.ToPosition toPosition
        && toPosition.position() == position
        && demand.heldBy().isEmpty()
        && profile.arrived()
        && Math.abs(readings.position() - profile.goal()) <= positionBands.enter();
  }

  /** Whether the joint is run at exactly a speed, and is inside the band to arrive in. */
  boolean runsAt(double velocity) {
    return doing.applied() instanceof Demand demand
        && demand.ask() instanceof JointAsk.AtVelocity atVelocity
        && atVelocity.speed() == velocity
        && withinSpeed(velocity, false);
  }

  private boolean isReady() {
    return homing.state() == State.READY;
  }

  /**
   * Whether the joint is doing what it was driven at.
   *
   * @param wasDoingIt whether it was the last time this was asked, which allows it the band to stay
   *     in, where otherwise it must come inside the band to arrive in
   */
  private boolean isDoing(JointAsk ask, boolean wasDoingIt) {
    return switch (ask) {
      case JointAsk.ToPosition toPosition -> {
        double off = Math.abs(readings.position() - profile.goal());
        yield profile.arrived() && off <= band(positionBands, wasDoingIt);
      }
      case JointAsk.AtVelocity atVelocity -> withinSpeed(atVelocity.speed(), wasDoingIt);
      // A voltage or a share of the battery has no speed to reach, so the joint is doing it at
      // once.
      case JointAsk.AtVoltage atVoltage -> true;
      case JointAsk.AtDutyCycle atDutyCycle -> true;
      case JointAsk.Neutral neutral ->
          Math.abs(readings.velocity()) <= speedBands.enter() * freeSpeed;
    };
  }

  /**
   * The speed the output the joint is driven at asks for, with nothing loading the joint, as a size
   * without a direction. Zero while the joint is not run at an output.
   */
  private double speedAsked() {
    if (doing.applied() instanceof Demand demand
        && demand.ask() instanceof JointAsk.AtOutput output) {
      return Math.abs(output.unloadedSpeed(freeSpeed));
    }
    return 0;
  }

  /**
   * The band a joint must be inside: the one to stay in if it was inside, or the one to arrive in.
   */
  private static double band(MechanismControl.Bands bands, boolean wasInside) {
    return wasInside ? bands.stay() : bands.enter();
  }

  /**
   * Whether the joint runs within its band of a speed. The band is a share of the speed, or, for a
   * speed of zero, which has no share, of the joint's free speed.
   */
  private boolean withinSpeed(double velocity, boolean wasWithin) {
    double share = band(speedBands, wasWithin);
    double allowed = share * (velocity == 0 ? freeSpeed : Math.abs(velocity));
    return Math.abs(readings.velocity() - velocity) <= allowed;
  }
}

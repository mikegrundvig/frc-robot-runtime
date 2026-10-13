package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.google.errorprone.annotations.CanIgnoreReturnValue;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveMove;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MoveEnding;
import com.michaelgrundvig.frc.world.record.DriveOutcome;
import com.michaelgrundvig.frc.world.record.MoveOutcome;
import com.michaelgrundvig.frc.world.record.MoveProgress;
import com.michaelgrundvig.frc.world.record.MoveReason;
import com.michaelgrundvig.frc.world.record.MoveResult;
import com.michaelgrundvig.frc.world.robot.Joint;
import java.util.Optional;

/**
 * What a test asks of how a move ended, read from the facts the move logs and never from its
 * sentence: {@code ended(out).arrived().in(0.60)}, {@code
 * ended(feed).canceled().because(MoveReason.GATE_FELL)}. A test that checks the facts goes on
 * passing when the sentence is reworded. When a check fails, the sentence is what the failure
 * shows, so the person reading it sees the whole ending at once.
 *
 * <p>The wording of an ending's sentence has one test of its own, in the model ({@code
 * MoveSentencesTest}).
 */
public final class Ended {
  /** How close a time must be to the one a test names: half of a hundredth of a second. */
  private static final double SECONDS_SHOWN = 0.005;

  private final String name;
  private final MoveResult result;
  private final double seconds;
  private final MoveReason reason;
  private final String why;

  /** The parts of a move of joints, or empty for a move of the drivetrain. */
  private final Optional<MoveEnding> joints;

  /** The whole ending as a person reads it, which a failed check shows. */
  private final String sentence;

  private Ended(
      String name,
      MoveResult result,
      double seconds,
      MoveReason reason,
      String why,
      Optional<MoveEnding> joints,
      String sentence) {
    this.name = name;
    this.result = result;
    this.seconds = seconds;
    this.reason = reason;
    this.why = why;
    this.joints = joints;
    this.sentence = sentence;
  }

  /** How the latest run of a move of joints ended. The test fails if no run has ended. */
  public static Ended ended(Move move) {
    assertThat(move.ending()).as("%s has ended", move.name()).isPresent();
    return of(move.ending().orElseThrow());
  }

  /** How a move of joints ended, from the ending a test kept when it ended. */
  public static Ended ended(MoveEnding ending) {
    return of(ending);
  }

  /** How the last move on a joint ended. The test fails if none has ended on it. */
  public static Ended ended(JointMechanism joint) {
    assertThat(joint.lastEnding()).as("a move on %s has ended", joint.getName()).isPresent();
    return of(joint.lastEnding().orElseThrow());
  }

  /** How the latest run of a move of the drivetrain ended. The test fails if no run has ended. */
  public static Ended ended(DriveMove move) {
    assertThat(move.ending()).as("%s has ended", move.name()).isPresent();
    DriveOutcome outcome = move.ending().orElseThrow();
    return new Ended(
        move.name(),
        outcome.result(),
        outcome.seconds(),
        outcome.reason(),
        move.endingWhy(),
        Optional.empty(),
        move.reason());
  }

  private static Ended of(MoveEnding ending) {
    return new Ended(
        ending.name(),
        ending.result(),
        ending.seconds(),
        ending.reason(),
        ending.why(),
        Optional.of(ending),
        ending.sentence());
  }

  /**
   * The move that ended is the one of this name. A joint remembers only the last move that ended on
   * it, so this says which move a test expects that to be.
   */
  @CanIgnoreReturnValue
  public Ended named(String expectedName) {
    assertThat(name).as(sentence).isEqualTo(expectedName);
    return this;
  }

  /** It got there, or its condition held. */
  @CanIgnoreReturnValue
  public Ended arrived() {
    return endedAs(MoveResult.ARRIVED);
  }

  /** Its deadline passed. */
  @CanIgnoreReturnValue
  public Ended timedOut() {
    return endedAs(MoveResult.TIMED_OUT);
  }

  /** Something canceled it. */
  @CanIgnoreReturnValue
  public Ended canceled() {
    return endedAs(MoveResult.CANCELED);
  }

  /** It would not start, or could not go on. */
  @CanIgnoreReturnValue
  public Ended refused() {
    return endedAs(MoveResult.REFUSED);
  }

  private Ended endedAs(MoveResult expected) {
    assertThat(result).as(sentence).isEqualTo(expected);
    return this;
  }

  /** It ran for this long, to the hundredth of a second its sentence shows. */
  @CanIgnoreReturnValue
  public Ended in(double expectedSeconds) {
    assertThat(seconds).as(sentence).isCloseTo(expectedSeconds, within(SECONDS_SHOWN));
    return this;
  }

  /** It ran for this long, to the hundredth of a second its sentence shows. */
  @CanIgnoreReturnValue
  public Ended after(double expectedSeconds) {
    return in(expectedSeconds);
  }

  /** It ran for at least one time and less than another, in seconds. */
  @CanIgnoreReturnValue
  public Ended inBetween(double atLeast, double lessThan) {
    assertThat(seconds).as(sentence).isGreaterThanOrEqualTo(atLeast).isLessThan(lessThan);
    return this;
  }

  /** It ended for this reason, of the closed set. */
  @CanIgnoreReturnValue
  public Ended because(MoveReason expected) {
    assertThat(reason).as(sentence).isEqualTo(expected);
    return this;
  }

  /** Its result says it all: it ended for no reason beyond how it ended. */
  @CanIgnoreReturnValue
  public Ended forNoReason() {
    return because(MoveReason.NONE);
  }

  /** The text of its reason, which its code gave and which is logged beside its facts. */
  @CanIgnoreReturnValue
  public Ended saying(String expectedWhy) {
    assertThat(why).as(sentence).isEqualTo(expectedWhy);
    return this;
  }

  /** Its reason is about the joint of this label. */
  @CanIgnoreReturnValue
  public Ended about(String jointLabel) {
    MoveEnding ending = joints.orElseThrow();
    int about = part().reasonJoint();
    assertThat(about).as(sentence).isNotNegative();
    assertThat(ending.definition().joints().get(about).label()).as(sentence).isEqualTo(jointLabel);
    return this;
  }

  /** It began, and was stopped part way: it had driven its joints. */
  @CanIgnoreReturnValue
  public Ended partWay() {
    assertThat(part().progress()).as(sentence).isEqualTo(MoveProgress.BEGUN);
    return this;
  }

  /** It ended before it began: refused as it started, or canceled while it waited. */
  @CanIgnoreReturnValue
  public Ended beforeItBegan() {
    assertThat(part().progress()).as(sentence).isEqualTo(MoveProgress.NOT_BEGUN);
    return this;
  }

  /** The part of a move of joints that every part agrees on. */
  private MoveOutcome part() {
    return joints.orElseThrow().parts().getFirst();
  }

  /**
   * One joint's part of a move of joints, for a test to read where the joint was and when it
   * arrived. The test fails if the move did not move the joint.
   */
  public MoveOutcome partOf(Joint joint) {
    MoveEnding ending = joints.orElseThrow();
    for (MoveOutcome each : ending.parts()) {
      boolean isItsPart =
          each.joint() >= 0 && ending.definition().joints().get(each.joint()).equals(joint);
      if (isItsPart) {
        return each;
      }
    }
    throw new AssertionError(joint.label() + " has no part in: " + sentence);
  }

  /** The facts of a move of joints, for what the checks above do not cover. */
  public MoveEnding facts() {
    return joints.orElseThrow();
  }

  /** The whole ending as a person reads it. */
  @Override
  public String toString() {
    return sentence;
  }
}

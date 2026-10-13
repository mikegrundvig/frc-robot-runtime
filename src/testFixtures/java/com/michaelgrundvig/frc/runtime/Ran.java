package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.google.errorprone.annotations.CanIgnoreReturnValue;
import com.michaelgrundvig.frc.runtime.autos.Autos;

/**
 * What a test asks of how a run of an auto went, read from the run's facts and never from its
 * sentence: {@code ran(autos).named("Grab and shoot").cutOff().after(1.00)}. As {@link Ended} does
 * for a move, a failed check shows the whole sentence the pit would read.
 *
 * <p>The wording of a run's sentence has one test of its own, in the model ({@code
 * MoveSentencesTest}).
 */
public final class Ran {
  /** How close a time must be to the one a test names: half of a hundredth of a second. */
  private static final double SECONDS_SHOWN = 0.005;

  private final Autos.Run run;

  private Ran(Autos.Run run) {
    this.run = run;
  }

  /** The latest run of an auto. The test fails if none has ended. */
  public static Ran lastRunOf(Autos autos) {
    assertThat(autos.lastRun()).as("an auto has run").isPresent();
    return new Ran(autos.lastRun().orElseThrow());
  }

  /** A run of an auto that a test already holds. */
  public static Ran ran(Autos.Run run) {
    return new Ran(run);
  }

  /** The auto that ran is the one of this name. */
  @CanIgnoreReturnValue
  public Ran named(String expectedName) {
    assertThat(run.auto()).as(run.describe()).isEqualTo(expectedName);
    return this;
  }

  /** Its command finished. */
  @CanIgnoreReturnValue
  public Ran completed() {
    return endedAs(Autos.Ending.COMPLETED);
  }

  /** Autonomous ended while it was still running. */
  @CanIgnoreReturnValue
  public Ran cutOff() {
    return endedAs(Autos.Ending.CUT_OFF);
  }

  /** Something else canceled it. */
  @CanIgnoreReturnValue
  public Ran canceled() {
    return endedAs(Autos.Ending.CANCELED);
  }

  /** Its command threw, as it was built or as it ran. */
  @CanIgnoreReturnValue
  public Ran failed() {
    return endedAs(Autos.Ending.FAILED);
  }

  private Ran endedAs(Autos.Ending expected) {
    assertThat(run.ending()).as(run.describe()).isEqualTo(expected);
    return this;
  }

  /** It ran for this long, to the hundredth of a second its sentence shows. */
  @CanIgnoreReturnValue
  public Ran after(double expectedSeconds) {
    assertThat(run.seconds()).as(run.describe()).isCloseTo(expectedSeconds, within(SECONDS_SHOWN));
    return this;
  }

  /** It ran for at least one time and less than another, in seconds. */
  @CanIgnoreReturnValue
  public Ran inBetween(double atLeast, double lessThan) {
    assertThat(run.seconds())
        .as(run.describe())
        .isGreaterThanOrEqualTo(atLeast)
        .isLessThan(lessThan);
    return this;
  }

  /** What its command threw, as the exception names itself: "IllegalStateException: boom". */
  @CanIgnoreReturnValue
  public Ran threw(String expectedThrown) {
    assertThat(run.why()).as(run.describe()).endsWith(expectedThrown);
    return this;
  }

  /** The whole run as the pit reads it. */
  @Override
  public String toString() {
    return run.describe();
  }
}

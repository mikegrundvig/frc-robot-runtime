package com.michaelgrundvig.frc.runtime.behavior;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.Optional;
import org.junit.jupiter.api.Test;

/** The one policy a joint's moves and the drive's end by: their deadlines and their reasons. */
class OutcomeTest {
  @Test
  void aMovesDeadlineIsTwiceItsProfilesTimeAndHalfASecond() {
    assertThat(Outcome.deadline(1.2)).isCloseTo(2.9, within(1e-12));
    assertThat(Outcome.deadline(0)).as("a move already there").isEqualTo(0.5);
  }

  @Test
  void aReasonSaysHowItEndedForHowLongWhatTheMoveAddsAndWhy() {
    assertThat(Outcome.ARRIVED.reason("pivot → out", 0.62, "", Optional.empty()))
        .isEqualTo("pivot → out: ARRIVED in 0.62 s");
    assertThat(
            Outcome.TIMED_OUT.reason(
                "pivot → out", 1.6, ", pivot at 40.3°", Optional.of("aimed is false")))
        .isEqualTo("pivot → out: TIMED_OUT after 1.60 s, pivot at 40.3°: aimed is false");
    assertThat(Outcome.CANCELED.reason("Intake", 0.5, "", Optional.empty()))
        .isEqualTo("Intake: CANCELED after 0.50 s");
    assertThat(
            Outcome.REFUSED.reason("pivot → out", 0.02, "", Optional.of("pivot isn't answering")))
        .as("refused as it starts: no time")
        .isEqualTo("pivot → out: REFUSED: pivot isn't answering");
  }
}

package com.michaelgrundvig.frc.runtime.behavior;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.record.MoveResult;
import org.junit.jupiter.api.Test;

/**
 * The one policy a joint's moves and the drive's end by: their deadlines. How an ending reads as a
 * sentence is the model's, and is tested there ({@code MoveSentencesTest}).
 */
class OutcomeTest {
  @Test
  void aMovesDeadlineIsTwiceItsProfilesTimeAndHalfASecond() {
    assertThat(Outcome.deadline(1.2)).isCloseTo(2.9, within(1e-12));
    assertThat(Outcome.deadline(0)).as("a move already there").isEqualTo(0.5);
  }

  /** Each outcome is logged as the model's result of the same name. */
  @Test
  void anOutcomeIsLoggedAsTheModelsResult() {
    assertThat(Outcome.values())
        .extracting(Outcome::result)
        .containsExactly(
            MoveResult.ARRIVED, MoveResult.TIMED_OUT, MoveResult.CANCELED, MoveResult.REFUSED);
  }
}

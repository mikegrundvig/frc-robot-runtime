package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command3.Command;

/**
 * Whether a joint has been judged against what it is asked now, story by story. Each story is what
 * happens to one joint in order, and ends with whether its gates may answer for it.
 */
class TenureTest {
  private static final Command SHOT = Command.noRequirements(co -> {}).named("Shot");
  private static final Command OTHER = Command.noRequirements(co -> {}).named("Other");

  /** One thing that happens to the joint. */
  private enum Step implements Consumer<Tenure> {
    SHOT_TAKES_IT(tenure -> tenure.takenBy(SHOT)),
    OTHER_TAKES_IT(tenure -> tenure.takenBy(OTHER)),
    SHOT_ENDS(tenure -> tenure.endedFor(SHOT)),
    OTHER_ENDS(tenure -> tenure.endedFor(OTHER)),
    ASKED_SOMETHING_NEW(tenure -> tenure.asked(false)),
    ASKED_THE_SAME_AGAIN(tenure -> tenure.asked(true)),
    /** The joint's step at the end of a loop in which it was asked something. */
    LOOP_ENDS_ASKED(tenure -> tenure.wrote(true)),
    /**
     * The joint's step at the end of a loop in which nothing asked, so it did what it does idle.
     */
    LOOP_ENDS_IDLE(tenure -> tenure.wrote(false));

    private final Consumer<Tenure> happens;

    Step(Consumer<Tenure> happens) {
      this.happens = happens;
    }

    @Override
    public void accept(Tenure tenure) {
      happens.accept(tenure);
    }
  }

  /**
   * @param name what the story shows
   * @param judged whether the joint has been judged against what it is asked, at the story's end
   * @param steps what happens to the joint, in order
   */
  private record Story(String name, boolean judged, List<Step> steps) {
    @Override
    public String toString() {
      return name;
    }
  }

  private static Story story(String name, boolean judged, Step... steps) {
    return new Story(name, judged, List.of(steps));
  }

  static Stream<Story> stories() {
    return Stream.of(
        story("before its first loop ends, it has not been judged at all", false),
        story("nobody's and idle, it is judged as idle", true, Step.LOOP_ENDS_IDLE),
        story("taken, it is not", false, Step.LOOP_ENDS_IDLE, Step.SHOT_TAKES_IT),
        story(
            "taken and never asked, it is not, however many loops go by",
            false,
            Step.LOOP_ENDS_IDLE,
            Step.SHOT_TAKES_IT,
            Step.LOOP_ENDS_IDLE,
            Step.LOOP_ENDS_IDLE),
        story(
            "taken and asked, it is not until the loop ends",
            false,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW),
        story(
            "taken and asked, it is once the loop ends",
            true,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED),
        story(
            "asked the same again, it stays judged",
            true,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED,
            Step.ASKED_THE_SAME_AGAIN),
        story(
            "asked something new, it is not until the loop ends",
            false,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED,
            Step.ASKED_SOMETHING_NEW),
        story(
            "its holder has asked and asks no more: its idle is judged as its holder's",
            true,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED,
            Step.LOOP_ENDS_IDLE),
        story(
            "its holder ends, and it is not until it is judged as idle",
            false,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED,
            Step.SHOT_ENDS),
        story(
            "its holder ends, and it is judged as idle when the loop ends",
            true,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED,
            Step.SHOT_ENDS,
            Step.LOOP_ENDS_IDLE),
        story(
            "its holder ends in the loop it asked: judged for the request is not judged as idle",
            false,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.SHOT_ENDS,
            Step.LOOP_ENDS_ASKED),
        story(
            "another command taking it starts over",
            false,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED,
            Step.OTHER_TAKES_IT,
            Step.LOOP_ENDS_IDLE),
        story(
            "a command that ends without having it changes nothing",
            true,
            Step.SHOT_TAKES_IT,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED,
            Step.OTHER_ENDS),
        story(
            "asked by code outside any command, with nobody holding it, it is judged as asked",
            true,
            Step.ASKED_SOMETHING_NEW,
            Step.LOOP_ENDS_ASKED));
  }

  @ParameterizedTest
  @MethodSource("stories")
  void aJointIsJudgedOnlyAgainstWhatItIsAskedNow(Story story) {
    Tenure tenure = new Tenure();

    story.steps().forEach(step -> step.accept(tenure));

    assertThat(tenure.isJudged()).as(story.name()).isEqualTo(story.judged());
  }

  /**
   * The count a gate watches moves on with each new question and again when the joint is judged
   * against it, and never goes back: a gate worked out before either is worked out again.
   */
  @ParameterizedTest
  @MethodSource("stories")
  void theCountAGateWatchesOnlyGoesUpAndMovesWithEveryChange(Story story) {
    Tenure tenure = new Tenure();
    long before = tenure.standing();
    boolean wasJudged = tenure.isJudged();
    long questionBefore = tenure.question();

    for (Step step : story.steps()) {
      step.accept(tenure);
      boolean changed = tenure.isJudged() != wasJudged || tenure.question() != questionBefore;
      assertThat(tenure.standing())
          .as("%s, after %s", story.name(), step)
          .isGreaterThanOrEqualTo(before);
      assertThat(tenure.standing() != before)
          .as("%s, after %s", story.name(), step)
          .isEqualTo(changed);
      before = tenure.standing();
      wasJudged = tenure.isJudged();
      questionBefore = tenure.question();
    }
  }
}

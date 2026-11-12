package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.CardNames;
import com.michaelgrundvig.frc.runtime.autos.AutoPit;
import com.michaelgrundvig.frc.runtime.coprocessors.BoardPit;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraPit;
import com.michaelgrundvig.frc.runtime.venues.VenuePit;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * Every card the robot program puts on Pitwall's Pit view, by name: the autos' and the venue's,
 * each listed coprocessor's and each camera's. The robot's configuration checks their names by
 * Pitwall's rule for them ({@link CardNames}) as it's built, so a camera or a board whose card
 * would take another card's name is refused there, saying which, and not as Pitwall starts. A test
 * builds the robot and checks that these are the cards it puts on the page ({@code PitCardsTest}),
 * so a card added to the page and not here fails it.
 */
final class PitCards {
  private PitCards() {}

  /** What a card's name comes from, which says what to change when it clashes. */
  enum Source {
    /** The robot program's own name for it: the autos' card and the venue's. */
    PROGRAM("rename one of the robot program's cards"),
    /** A listed coprocessor's address. */
    COPROCESSOR("list each coprocessor once, by its address"),
    /** A camera's name. */
    CAMERA("give the camera another name");

    /** What to change, said after the clash. */
    final String advice;

    Source(String advice) {
      this.advice = advice;
    }
  }

  /**
   * One card.
   *
   * @param name its name on the page
   * @param whose whose card it is, as a reason names it: "camera \"front\"'s card"
   * @param source what its name comes from
   */
  record Card(String name, String whose, Source source) {}

  /**
   * The cards a robot with this definition and these coprocessors has. The robot program's own come
   * first, then the boards', then the cameras', so that a clash is said of the card whose name is
   * easiest to change: a camera's, before a board's address.
   */
  static List<Card> of(RobotDefinition definition, List<String> coprocessors) {
    List<Card> cards = new ArrayList<>();
    cards.add(new Card(AutoPit.NAME, "the autos' card", Source.PROGRAM));
    cards.add(new Card(VenuePit.NAME, "the venue's card", Source.PROGRAM));
    for (String board : coprocessors) {
      String whose = "coprocessor " + board + "'s card";
      cards.add(new Card(BoardPit.deviceName(board), whose, Source.COPROCESSOR));
    }
    for (CameraConfig camera : definition.cameras()) {
      String whose = "camera \"" + camera.name() + "\"'s card";
      cards.add(new Card(CameraPit.deviceName(camera), whose, Source.CAMERA));
    }
    return cards;
  }

  /**
   * Why a card whose name comes from this source can't have it, if one can't: the first such card
   * named as a card before it is. Every card before it takes its name, whatever its source, so a
   * camera named as a board is said of the camera.
   *
   * @return the reason, or null if every such card can have its name
   */
  static @Nullable String whyNamesClash(List<Card> cards, Source source) {
    CardNames names = new CardNames();
    for (Card card : cards) {
      Optional<String> other = names.take(card.name(), card.whose());
      if (other.isEmpty()) {
        continue;
      }
      if (card.source() != source) {
        continue;
      }
      return card.whose()
          + " would be named "
          + card.name()
          + " on Pitwall's Pit view, as "
          + other.get()
          + " is: "
          + source.advice;
    }
    return null;
  }
}

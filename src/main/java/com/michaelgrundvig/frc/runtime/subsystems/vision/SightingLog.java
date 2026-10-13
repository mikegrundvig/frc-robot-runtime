package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.TagVerdict;
import com.michaelgrundvig.frc.localization.Verdict;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.pitwall.robot.Sighting;
import com.michaelgrundvig.frc.pitwall.robot.SightingWords;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Logs what became of each tag in one camera's images, as facts: what Pitwall's World view draws a
 * camera's sight lines from, at any loop of the session.
 *
 * <p>A tag's fact ({@link Sighting}) holds no text: the tag by its number, and its verdict and its
 * image's solver by their numbers in localization's own lists. Those facts are logged under {@code
 * Vision/<camera>/Sightings}, and only when they differ from the last logged, which is when the
 * camera sees other tags or its tags fare otherwise. The measurements behind them are new with
 * every image, so they are logged beside as plain numbers, {@code Vision/<camera>/SightingNumbers},
 * {@link Sighting#NUMBERS} for each fact in the facts' order, and as an empty list in a loop
 * without an image.
 */
final class SightingLog {
  /**
   * The words for a sighting's numbers: a verdict with its numbers as localization says it, and a
   * solver by its name. Pitwall is handed these, and makes no sentence of its own.
   */
  static final SightingWords WORDS =
      new SightingWords() {
        @Override
        public String verdict(int verdict, double value, double threshold) {
          boolean known = verdict >= 0 && verdict < VERDICTS.length;
          return known ? VERDICTS[verdict].describe(value, threshold) : "";
        }

        @Override
        public String solver(int solver) {
          boolean known = solver >= 0 && solver < SOLVERS.length;
          return known ? SOLVERS[solver].name() : "";
        }
      };

  private static final Verdict[] VERDICTS = Verdict.values();
  private static final VisionMeasurement.Solver[] SOLVERS = VisionMeasurement.Solver.values();
  private static final double[] NO_NUMBERS = new double[0];
  private static final Sighting[] NO_SIGHTINGS = new Sighting[0];

  /** No solver: the image never became a measurement. */
  private static final int NO_SOLVER = -1;

  private final String camera;
  private final String sightingsKey;
  private final String numbersKey;

  /** This loop's sightings, filled again every loop. */
  private final List<Sighting> sighted = new ArrayList<>();

  /** The sightings as last logged. */
  private List<Sighting> logged = List.of();

  /** This loop's numbers. AdvantageKit copies what it logs, so it is filled again next loop. */
  private double[] numbers = NO_NUMBERS;

  /**
   * @param camera the camera's name, as its images' verdicts name it
   * @param key the camera's key in the log: {@code Vision/<camera>}
   */
  SightingLog(String camera, String key) {
    this.camera = camera;
    sightingsKey = key + "/Sightings";
    numbersKey = key + "/SightingNumbers";
  }

  /** Logs this camera's tags among a loop's images, with what became of each. */
  void log(List<FrameVerdict> frames) {
    sighted.clear();
    int tags = 0;
    for (FrameVerdict frame : frames) {
      if (frame.camera().equals(camera)) {
        tags += frame.tags().size();
      }
    }
    if (tags == 0) {
      Logger.recordOutput(numbersKey, NO_NUMBERS);
      return;
    }
    if (numbers.length != tags * Sighting.NUMBERS) {
      numbers = new double[tags * Sighting.NUMBERS];
    }
    int image = 0;
    for (FrameVerdict frame : frames) {
      if (!frame.camera().equals(camera)) {
        continue;
      }
      for (TagVerdict tag : frame.tags()) {
        putNumbers(sighted.size(), tag, frame.measured());
        sighted.add(sighting(tag, image, frame.measured()));
      }
      image++;
    }
    if (!sighted.equals(logged)) {
      logged = List.copyOf(sighted);
      Logger.recordOutput(sightingsKey, logged.toArray(NO_SIGHTINGS));
    }
    Logger.recordOutput(numbersKey, numbers);
  }

  private static Sighting sighting(
      TagVerdict tag, int image, Optional<FrameVerdict.Measured> measured) {
    int solver = NO_SOLVER;
    if (measured.isPresent()) {
      solver = measured.get().solver().ordinal();
    }
    return new Sighting(
        tag.tag(), image, fate(tag.verdict(), measured), tag.verdict().ordinal(), solver);
  }

  /** What became of a tag, from its verdict and whether its image became a measurement. */
  private static Sighting.Fate fate(Verdict verdict, Optional<FrameVerdict.Measured> measured) {
    if (verdict == Verdict.ACCEPTED) {
      return Sighting.Fate.ACCEPTED;
    }
    if (verdict == Verdict.GIVEN) {
      return Sighting.Fate.GIVEN;
    }
    if (verdict == Verdict.CALIBRATING || verdict == Verdict.HELD) {
      return Sighting.Fate.HELD_BACK;
    }
    if (measured.isPresent()) {
      return Sighting.Fate.REJECTED;
    }
    return Sighting.Fate.LEFT_OUT;
  }

  /** Puts one sighting's numbers in their places. An image left out has no measured pose. */
  private void putNumbers(int which, TagVerdict tag, Optional<FrameVerdict.Measured> measured) {
    int at = which * Sighting.NUMBERS;
    numbers[at + Sighting.VALUE] = tag.value();
    numbers[at + Sighting.THRESHOLD] = tag.threshold();
    numbers[at + Sighting.INNOVATION_METERS] = tag.innovationMeters();
    numbers[at + Sighting.CHI_SQUARED] = tag.chiSquared();
    numbers[at + Sighting.MEASURED_X] = Double.NaN;
    numbers[at + Sighting.MEASURED_Y] = Double.NaN;
    numbers[at + Sighting.MEASURED_HEADING] = Double.NaN;
    if (measured.isPresent()) {
      numbers[at + Sighting.MEASURED_X] = measured.get().pose().getX();
      numbers[at + Sighting.MEASURED_Y] = measured.get().pose().getY();
      numbers[at + Sighting.MEASURED_HEADING] = measured.get().pose().getRotation().getRadians();
    }
  }
}

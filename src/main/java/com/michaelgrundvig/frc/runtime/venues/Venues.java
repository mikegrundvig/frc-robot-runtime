package com.michaelgrundvig.frc.runtime.venues;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.TagLayout;
import com.michaelgrundvig.frc.world.field.Venue;
import com.michaelgrundvig.frc.world.field.VenueCheck;
import com.michaelgrundvig.frc.world.field.VenueFile;
import com.michaelgrundvig.frc.world.game.Game;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.fields.Field;
import org.wpilib.util.Alert;

/**
 * The field the robot believes it plays on: the official layout, or the venue the pit selected, its
 * tags where a survey found them. Taken at the start of a loop, and changed only while disabled.
 *
 * <ul>
 *   <li><b>The disk is the default.</b> With no selection, or one that can't be used, the robot
 *       uses the official layout, and a selection it can't use raises a HIGH alert saying why.
 *   <li><b>A practice field's survey is never valid with the field attached.</b> If one is selected
 *       when the field connects, the robot falls back to the official layout and says so, until the
 *       pit selects the event's venue.
 *   <li><b>Not in Git:</b> a venue uploaded in the pit and not committed raises a MEDIUM alert
 *       until a build carries the same file.
 * </ul>
 *
 * <p>The selected file is logged as an input, so a replay uses exactly that venue.
 */
public final class Venues implements AutoCloseable {
  private final Game game;
  private final VenueIO io;
  private final VenueIOInputsAutoLogged inputs = new VenueIOInputsAutoLogged();

  private final Alert unusable =
      new Alert("Venue", "unusable", "The selected venue can't be used", Alert.Level.HIGH);
  private final Alert practiceOnField =
      new Alert(
          "Venue",
          "practiceOnField",
          "A practice field's venue with the field attached: the robot uses the official layout"
              + " until the pit selects the event's venue",
          Alert.Level.HIGH);
  private final Alert notInGit =
      new Alert(
          "Venue",
          "notInGit",
          "Venue data not in Git: download it from Pitwall and commit it",
          Alert.Level.MEDIUM);

  /** The text last parsed, and what it gave: a venue, or why not. */
  private @Nullable String parsedText;

  private @Nullable Venue parsed;

  /** The official layout the parsed venue was measured against: its tags' nominal poses. */
  private @Nullable Field parsedBase;

  private String parsedHash = "";
  private String parseProblem = "";

  /** What's in use: a venue's hash, or the official layout's name, which the robot starts with. */
  private String applied;

  private String active;
  private boolean pending;

  public Venues(Game game, VenueIO io) {
    this.game = game;
    this.io = io;
    applied = game.official().getName();
    active = applied;
  }

  /**
   * At the start of a loop: reads the selection and, when it differs from what's in use and the
   * robot is disabled, gives the field model to use from now on. The robot starts on the official
   * layout, so a venue selected on disk takes its place at the first loop.
   *
   * @param disabled whether the robot is disabled: a venue never changes while it drives
   * @param fieldAttached whether the field management system is attached
   */
  public Optional<FieldModel> update(boolean disabled, boolean fieldAttached) {
    io.updateInputs(inputs);
    Logger.processInputs("Venue", inputs);
    Venue venue = null;
    Field base = null;
    String problem = inputs.problem;
    if (problem.isEmpty() && !inputs.text.isEmpty()) {
      if (!inputs.text.equals(parsedText)) {
        parsedText = inputs.text;
        parse(inputs.text);
      }
      venue = parsed;
      base = parsedBase;
      problem = parseProblem;
    }
    unusable.setText(
        "The selected venue can't be used, so the robot uses the official layout: " + problem);
    unusable.set(!problem.isEmpty());
    boolean practice = venue != null && venue.kind() == Venue.Kind.PRACTICE && fieldAttached;
    practiceOnField.set(practice);
    if (practice) {
      venue = null;
    }
    notInGit.set(venue != null && !inputs.inGit);
    String wanted = venue != null ? parsedHash : game.official().getName();
    Optional<FieldModel> change = Optional.empty();
    if (!wanted.equals(applied) && disabled) {
      applied = wanted;
      active = venue != null ? venue.name() : game.official().getName();
      change =
          Optional.of(
              game.model()
                  .apply(
                      venue != null && base != null
                          ? venue.tagLayout(base)
                          : TagLayout.of(game.official())));
    }
    pending = !wanted.equals(applied);
    Logger.recordOutput("Venue/Active", active);
    Logger.recordOutput("Venue/Pending", pending);
    return change;
  }

  /** What the robot believes it plays on: a venue's name, or the official layout's. */
  public String active() {
    return active;
  }

  private void parse(String text) {
    parsed = null;
    parsedBase = null;
    parseProblem = "";
    try {
      Venue venue = VenueFile.read(text);
      List<String> problems = problems(venue);
      if (problems.isEmpty()) {
        parsed = venue;
        parsedBase = base(venue).orElseThrow();
        parsedHash = venue.hash();
      } else {
        parseProblem = String.join("; ", problems);
      }
    } catch (IllegalArgumentException e) {
      parseProblem = String.valueOf(e.getMessage());
    }
  }

  /** What's wrong with a venue on this game's field: empty when the robot can use it. */
  List<String> problems(Venue venue) {
    return base(venue)
        .map(field -> VenueCheck.problems(venue, field, game.symmetry()))
        .orElse(List.of("it was measured against \"" + venue.base() + "\", not this game's field"));
  }

  /** The official layout a venue was measured against, if it's one of this game's. */
  private Optional<Field> base(Venue venue) {
    return game.variants().stream()
        .filter(field -> field.getName().equals(venue.base()))
        .findFirst();
  }

  /** Whether a choice waits for the robot to be disabled. */
  boolean pending() {
    return pending;
  }

  /** Whether a selection the robot can't use is shown, and what it says. */
  @Nullable String unusable() {
    return unusable.get() ? unusable.getText() : null;
  }

  boolean practiceOnField() {
    return practiceOnField.get();
  }

  boolean notInGit() {
    return notInGit.get();
  }

  @Override
  public void close() {
    unusable.close();
    practiceOnField.close();
    notInGit.close();
  }
}

package com.michaelgrundvig.frc.runtime.venues;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.TagLayout;
import com.michaelgrundvig.frc.world.field.Venue;
import com.michaelgrundvig.frc.world.field.VenueCheck;
import com.michaelgrundvig.frc.world.field.VenueFile;
import com.michaelgrundvig.frc.world.game.Game;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.fields.Field;
import org.wpilib.util.Alert;

/**
 * The field the robot believes it plays on. That is the official layout, or the venue the pit
 * selected, whose tags are where a survey found them. The selection is read at the start of every
 * loop, and the field changes only while the robot is disabled.
 *
 * <ul>
 *   <li><b>The official layout is the default.</b> The robot uses it when nothing is selected, and
 *       when the selection cannot be used. In the second case a HIGH alert says why.
 *   <li><b>A practice field's survey is never used with the field attached.</b> If one is selected
 *       when the field connects, the robot goes back to the official layout and says so, until the
 *       pit selects the event's venue.
 *   <li><b>Not in Git.</b> A venue that was uploaded in the pit and never committed raises a MEDIUM
 *       alert, until a build carries the same file.
 * </ul>
 *
 * <p>The selected file is logged as an input, so a replay uses exactly that venue.
 */
public final class Venues implements AutoCloseable {
  /** What the selection on the robot's disk comes to. */
  private sealed interface Selection {
    /** What is wrong with the selection, in words for the pit. Empty when nothing is. */
    default String problem() {
      return "";
    }

    /** Nothing is selected: the robot plays on the official layout. */
    record Official() implements Selection {}

    /**
     * A venue the robot can use.
     *
     * @param venue the venue, as its file describes it
     * @param base the official layout the venue was measured against, which gives its tags their
     *     nominal poses
     * @param hash the hash of the file's contents, which identifies the venue
     */
    record Surveyed(Venue venue, Field base, String hash) implements Selection {}

    /**
     * A selection the robot cannot use: its file could not be read, is not a venue file, or
     * describes a venue that does not fit this game's field.
     *
     * @param problem why not
     */
    record Unusable(String problem) implements Selection {}
  }

  private static final Selection OFFICIAL = new Selection.Official();

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

  // What the disk said the last time it said something new, and what that came to. A venue file is
  // read and checked once, when it first appears, and not again every loop.

  private String understoodText = "";
  private String understoodProblem = "";
  private Selection understood = OFFICIAL;

  /** What the alert for an unusable selection says, for the selection last understood. */
  private String unusableText = unusableText("");

  /**
   * What identifies the field in use: a venue's hash, or the official layout's name, which the
   * robot starts with.
   */
  private String inUse;

  /** The name of the field in use, for the pit. */
  private String active;

  /** Whether another field is selected, and waits for the robot to be disabled. */
  private boolean pending;

  public Venues(Game game, VenueIO io) {
    this.game = game;
    this.io = io;
    inUse = game.official().getName();
    active = inUse;
  }

  /**
   * Starts a loop: reads the selection, raises what alerts it calls for, and, if it asks for
   * another field than the one in use and the robot is disabled, gives the field model to use from
   * now on.
   *
   * <p>The robot starts on the official layout, so a venue that is already selected on the disk
   * takes its place in the first loop.
   *
   * @param disabled whether the robot is disabled. The field never changes while it drives.
   * @param fieldAttached whether the field management system is attached
   * @return the field model to use from now on, in the loop the field changes
   */
  public Optional<FieldModel> update(boolean disabled, boolean fieldAttached) {
    io.updateInputs(inputs);
    Logger.processInputs("Venue", inputs);
    Selection selection = selection();
    unusable.setText(unusableText);
    unusable.set(!selection.problem().isEmpty());

    boolean isPracticeVenueOnTheField =
        selection instanceof Selection.Surveyed surveyed
            && surveyed.venue().kind() == Venue.Kind.PRACTICE
            && fieldAttached;
    practiceOnField.set(isPracticeVenueOnTheField);
    Selection wanted = OFFICIAL;
    if (selection instanceof Selection.Surveyed && !isPracticeVenueOnTheField) {
      wanted = selection;
    }
    notInGit.set(wanted instanceof Selection.Surveyed && !inputs.inGit);

    Optional<FieldModel> change = Optional.empty();
    String wantedKey = key(wanted);
    if (disabled && !wantedKey.equals(inUse)) {
      inUse = wantedKey;
      active = name(wanted);
      change = Optional.of(model(wanted));
    }
    pending = !wantedKey.equals(inUse);
    Logger.recordOutput("Venue/Active", active);
    Logger.recordOutput("Venue/Pending", pending);
    return change;
  }

  /**
   * What the selection the disk reports comes to. It is worked out afresh only when the disk
   * reports something new.
   */
  private Selection selection() {
    boolean isAsUnderstood =
        inputs.text.equals(understoodText) && inputs.problem.equals(understoodProblem);
    if (!isAsUnderstood) {
      understoodText = inputs.text;
      understoodProblem = inputs.problem;
      understood = understand(inputs.text, inputs.problem);
      unusableText = unusableText(understood.problem());
    }
    return understood;
  }

  /**
   * Works out what a selection comes to.
   *
   * @param text the selected venue's file, whole; empty when nothing is selected
   * @param problem why the selection could not be read from the disk; empty when it could
   */
  private Selection understand(String text, String problem) {
    if (!problem.isEmpty()) {
      return new Selection.Unusable(problem);
    }
    if (text.isEmpty()) {
      return OFFICIAL;
    }
    Venue venue;
    try {
      venue = VenueFile.read(text);
    } catch (IllegalArgumentException e) {
      return new Selection.Unusable(String.valueOf(e.getMessage()));
    }
    List<String> problems = problems(venue);
    if (!problems.isEmpty()) {
      return new Selection.Unusable(String.join("; ", problems));
    }
    return new Selection.Surveyed(venue, base(venue).orElseThrow(), venue.hash());
  }

  private static String unusableText(String problem) {
    return "The selected venue can't be used, so the robot uses the official layout: " + problem;
  }

  /** What identifies a field: a venue's hash, or the official layout's name. */
  private String key(Selection field) {
    if (field instanceof Selection.Surveyed surveyed) {
      return surveyed.hash();
    }
    return game.official().getName();
  }

  /** A field's name, for the pit: the venue's, or the official layout's. */
  private String name(Selection field) {
    if (field instanceof Selection.Surveyed surveyed) {
      return surveyed.venue().name();
    }
    return game.official().getName();
  }

  /** The field model for a field: the game's, with the venue's tags or the official layout's. */
  private FieldModel model(Selection field) {
    if (field instanceof Selection.Surveyed surveyed) {
      return game.model().apply(surveyed.venue().tagLayout(surveyed.base()));
    }
    return game.model().apply(TagLayout.of(game.official()));
  }

  /** What the robot believes it plays on: a venue's name, or the official layout's. */
  public String active() {
    return active;
  }

  /** What is wrong with a venue on this game's field. Empty when the robot can use it. */
  List<String> problems(Venue venue) {
    Optional<Field> base = base(venue);
    if (base.isEmpty()) {
      return List.of("it was measured against \"" + venue.base() + "\", not this game's field");
    }
    return VenueCheck.problems(venue, base.get(), game.symmetry());
  }

  /** The official layout a venue was measured against, if it is one of this game's. */
  private Optional<Field> base(Venue venue) {
    return game.variants().stream()
        .filter(field -> field.getName().equals(venue.base()))
        .findFirst();
  }

  /** Whether a choice waits for the robot to be disabled. */
  boolean pending() {
    return pending;
  }

  /** What the alert for a selection the robot cannot use says, while that alert is raised. */
  Optional<String> unusable() {
    if (!unusable.get()) {
      return Optional.empty();
    }
    return Optional.of(unusable.getText());
  }

  /** Whether a practice field's venue is set aside because the field is attached. */
  boolean practiceOnField() {
    return practiceOnField.get();
  }

  /** Whether the venue in use is one no build carries. */
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

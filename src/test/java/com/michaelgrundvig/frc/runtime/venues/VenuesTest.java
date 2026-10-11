package com.michaelgrundvig.frc.runtime.venues;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.Body;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class VenuesTest {
  private static final Body BODY = ExampleRobot.COMPETITION.definition().body();

  /** The season's welded layout, as WPILib ships it. */
  private static final String WELDED = resource("org/wpilib/fields/frc/2026-rebuilt-welded.json");

  private @Nullable Venues venues;

  private static String resource(String name) {
    try (InputStream in = VenuesTest.class.getClassLoader().getResourceAsStream(name)) {
      if (in == null) {
        throw new IllegalStateException("no resource " + name);
      }
      return new String(in.readAllBytes(), StandardCharsets.UTF_8);
    } catch (IOException e) {
      throw new IllegalStateException(e);
    }
  }

  /** A venue file over the welded layout, with tag 26 moved along x. */
  private static String venue(String kind, double tag26Shift) {
    String layout = WELDED;
    if (tag26Shift != 0) {
      int tag = layout.indexOf("\"ID\": 26");
      int x = layout.indexOf("\"x\": ", tag) + 5;
      int end = layout.indexOf(',', x);
      double moved = Double.parseDouble(layout.substring(x, end).strip()) + tag26Shift;
      layout = layout.substring(0, x) + moved + layout.substring(end);
    }
    return "{\"format\": 1, \"name\": \"Test venue\", \"kind\": \""
        + kind
        + "\", \"base\": \"2026 FRC Rebuilt Welded\", \"layout\": "
        + layout
        + "}";
  }

  /** What the disk says, as a test sets it. */
  private static final class Disk implements VenueIO {
    String text = "";
    String problem = "";
    boolean inGit = true;

    @Override
    public void updateInputs(VenueIOInputs inputs) {
      inputs.text = text;
      inputs.problem = problem;
      inputs.inGit = inGit;
    }
  }

  private Venues venues(VenueIO disk) {
    Venues made = new Venues(Rebuilt.game(BODY), disk);
    venues = made;
    return made;
  }

  @AfterEach
  void close() {
    if (venues != null) {
      venues.close();
    }
  }

  @Test
  void withNoSelectionTheRobotStaysOnTheOfficialLayout() {
    Venues venues = venues(new Disk());
    assertThat(venues.update(true, false)).isEmpty();
    assertThat(venues.active()).isEqualTo("2026 FRC Rebuilt Welded");
    assertThat(venues.unusable()).isEmpty();
  }

  @Test
  void aSelectedVenueTakesTheFieldsPlaceWhileDisabled() {
    Disk disk = new Disk();
    disk.text = venue("event", 0.03);
    Venues venues = venues(disk);
    Optional<FieldModel> field = venues.update(true, false);
    assertThat(field).isPresent();
    double official = Rebuilt.field(BODY).tag(26).orElseThrow().getX();
    assertThat(field.get().tag(26).orElseThrow().getX()).isEqualTo(official + 0.03);
    assertThat(venues.active()).isEqualTo("Test venue");
    assertThat(venues.update(true, false)).as("only once").isEmpty();
  }

  @Test
  void aSelectionWaitsUntilTheRobotIsDisabled() {
    Disk disk = new Disk();
    Venues venues = venues(disk);
    venues.update(true, false);
    disk.text = venue("event", 0.03);
    assertThat(venues.update(false, false)).as("enabled").isEmpty();
    assertThat(venues.active()).isEqualTo("2026 FRC Rebuilt Welded");
    assertThat(venues.update(true, false)).as("disabled again").isPresent();
  }

  @Test
  void aSelectionTheRobotCantUseFallsBackToTheOfficialLayoutSayingWhy() {
    Disk disk = new Disk();
    disk.text = venue("event", 0.5);
    Venues venues = venues(disk);
    assertThat(venues.update(true, false)).isEmpty();
    assertThat(venues.active()).isEqualTo("2026 FRC Rebuilt Welded");
    assertThat(venues.unusable().orElseThrow()).contains("tag 26 sits 0.500 m");

    disk.text = "";
    disk.problem = "the selected venue (abc) isn't on the robot";
    venues.update(true, false);
    assertThat(venues.unusable().orElseThrow()).contains("isn't on the robot");
  }

  @Test
  void aPracticeFieldsVenueGivesWayToTheOfficialLayoutWithTheFieldAttached() {
    Disk disk = new Disk();
    disk.text = venue("practice", 0.03);
    Venues venues = venues(disk);
    assertThat(venues.update(true, false)).isPresent();
    assertThat(venues.practiceOnField()).isFalse();

    Optional<FieldModel> field = venues.update(true, true);
    assertThat(venues.practiceOnField()).isTrue();
    assertThat(field).isPresent();
    assertThat(field.get().tag(26)).isEqualTo(Rebuilt.field(BODY).tag(26));
    assertThat(venues.active()).isEqualTo("2026 FRC Rebuilt Welded");
  }

  @Test
  void aVenueNotInGitSaysSo() {
    Disk disk = new Disk();
    disk.text = venue("event", 0.03);
    disk.inGit = false;
    Venues venues = venues(disk);
    venues.update(true, false);
    assertThat(venues.notInGit()).isTrue();
  }

  @Test
  void theStoreKeepsVenuesAndTheSelectionAcrossRestarts(@TempDir Path stored, @TempDir Path git)
      throws IOException {
    VenueStore store = new VenueStore(stored, git);
    assertThat(store.selectedText()).isEmpty();
    String text = venue("event", 0.03);
    String hash = store.save(text);
    store.select(hash);

    VenueStore restarted = new VenueStore(stored, git);
    assertThat(restarted.selectedText()).contains(text);
    assertThat(restarted.committed(text)).isFalse();
    Files.writeString(git.resolve("event.json"), text);
    assertThat(restarted.committed(text)).isTrue();

    restarted.selectOfficial();
    assertThat(restarted.selectedText()).isEmpty();
  }

  @Test
  void aSelectionNoFolderHasIsAnError(@TempDir Path stored, @TempDir Path git) throws IOException {
    VenueStore store = new VenueStore(stored, git);
    store.select("0123");
    assertThatThrownBy(store::selectedText).hasMessageContaining("isn't on the robot");
  }

  /**
   * The disk is read as the robot starts, and after a selection on the thread that made it; the
   * loop takes what was read, and never reads the disk: with the files gone, it still has them.
   */
  @Test
  void theDiskIsReadAtTheStartAndAfterASelectionNeverInTheLoop(
      @TempDir Path stored, @TempDir Path git) throws IOException {
    VenueStore store = new VenueStore(stored, git);
    VenueIODisk disk = new VenueIODisk(store);
    VenueIOInputsAutoLogged inputs = new VenueIOInputsAutoLogged();
    disk.updateInputs(inputs);
    assertThat(inputs.text).isEmpty();

    String text = venue("event", 0.03);
    store.select(store.save(text));
    disk.updateInputs(inputs);
    assertThat(inputs.text).as("not read again until asked").isEmpty();
    VenueIOInputsAutoLogged started = new VenueIOInputsAutoLogged();
    new VenueIODisk(store).updateInputs(started);
    assertThat(started.text).as("a robot started now reads it as it starts").isEqualTo(text);
    disk.reread();
    try (var files = Files.list(stored)) {
      for (Path file : files.toList()) {
        Files.delete(file);
      }
    }
    disk.updateInputs(inputs);
    assertThat(inputs.text).isEqualTo(text);
    assertThat(inputs.inGit).isFalse();
  }

  /**
   * The robot keeps up to {@value VenueStore#MOST_STORED} uploads: past that, an upload is refused,
   * saying what to do. One it keeps already takes no more room.
   */
  @Test
  void thePitKeepsAtMost32Venues(@TempDir Path stored, @TempDir Path git) throws IOException {
    VenueStore store = new VenueStore(stored, git);
    VenueIODisk disk = new VenueIODisk(store);
    VenuePit pit = new VenuePit(store, disk, venues(disk), Rebuilt.game(BODY));
    for (int i = 1; i <= VenueStore.MOST_STORED; i++) {
      store.save(venue("event", 0.0005 * i));
    }

    assertThatThrownBy(() -> pit.upload(venue("event", 0.02).getBytes(StandardCharsets.UTF_8)))
        .isInstanceOf(Devices.Refused.class)
        .hasMessage(
            "the robot keeps 32 uploaded venues already, the most it keeps: delete those you don't"
                + " need from "
                + stored
                + ", then upload again");
    assertThat(pit.upload(venue("event", 0.0005).getBytes(StandardCharsets.UTF_8)))
        .as("kept already")
        .contains("Saved");
  }

  @Test
  void thePitUploadsChoosesAndDownloadsAVenue(@TempDir Path stored, @TempDir Path git)
      throws IOException {
    VenueStore store = new VenueStore(stored, git);
    VenueIODisk disk = new VenueIODisk(store);
    Venues venues = venues(disk);
    VenuePit pit = new VenuePit(store, disk, venues, Rebuilt.game(BODY));
    assertThat(pit.card().choices().get(0).options())
        .extracting(Devices.Option::value)
        .containsExactly(VenuePit.OFFICIAL);

    assertThat(pit.upload(venue("event", 0.03).getBytes(StandardCharsets.UTF_8)))
        .contains("Saved \"Test venue\"");
    List<Devices.Option> options = pit.card().choices().get(0).options();
    assertThat(options).hasSize(2);
    assertThat(options.get(1).label()).isEqualTo("Test venue (event, not in Git)");

    assertThat(pit.choose(options.get(1).value())).contains("Chose Test venue");
    assertThat(venues.update(true, false)).as("taken at the next loop").isPresent();
    assertThat(pit.card().state()).isEqualTo("Test venue");
    assertThat(pit.card().lines()).extracting(Devices.Line::label).contains("Not in Git");
    try (Devices.File file = pit.download()) {
      assertThat(file.name()).isEqualTo("test-venue.json");
      assertThat(new String(file.body().readAllBytes(), StandardCharsets.UTF_8))
          .isEqualTo(venue("event", 0.03));
    }
  }

  @Test
  void thePitRefusesASurveyThatDoesntFitAndWrapsABareLayout(@TempDir Path stored, @TempDir Path git)
      throws IOException {
    VenueStore store = new VenueStore(stored, git);
    VenueIODisk disk = new VenueIODisk(store);
    VenuePit pit = new VenuePit(store, disk, venues(disk), Rebuilt.game(BODY));
    assertThatThrownBy(() -> pit.upload(venue("event", 0.5).getBytes(StandardCharsets.UTF_8)))
        .isInstanceOf(Devices.Refused.class)
        .hasMessageContaining("tag 26 sits 0.500 m");
    assertThat(pit.upload(WELDED.getBytes(StandardCharsets.UTF_8)))
        .contains("Saved \"2026 FRC Rebuilt Welded\"")
        .contains("bare WPILib layout");
    assertThatThrownBy(() -> pit.choose("nope")).hasMessageContaining("no venue nope");
    assertThatThrownBy(pit::download).hasMessageContaining("official layout");
  }
}

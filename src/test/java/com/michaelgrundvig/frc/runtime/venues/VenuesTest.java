package com.michaelgrundvig.frc.runtime.venues;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Venue;
import com.michaelgrundvig.frc.world.field.VenueCameras;
import com.michaelgrundvig.frc.world.field.VenueFile;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class VenuesTest {
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
    Venues made = new Venues(Rebuilt.GAME, disk);
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
    double official = Rebuilt.field().tag(26).orElseThrow().getX();
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

  @ParameterizedTest
  @ValueSource(strings = {"practice", "shop"})
  void aPracticeFieldsOrShopsVenueGivesWayToTheOfficialLayoutWithTheFieldAttached(String kind) {
    Disk disk = new Disk();
    disk.text = venue(kind, 0.03);
    Venues venues = venues(disk);
    assertThat(venues.update(true, false)).isPresent();
    assertThat(venues.practiceOnField()).isFalse();

    Optional<FieldModel> field = venues.update(true, true);
    assertThat(venues.practiceOnField()).isTrue();
    assertThat(field).isPresent();
    assertThat(field.get().tag(26)).isEqualTo(Rebuilt.field().tag(26));
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
    VenuePit pit = new VenuePit(store, disk, venues(disk), Rebuilt.GAME);
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
    VenuePit pit = new VenuePit(store, disk, venues, Rebuilt.GAME);
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
    VenuePit pit = new VenuePit(store, disk, venues(disk), Rebuilt.GAME);
    assertThatThrownBy(() -> pit.upload(venue("event", 0.5).getBytes(StandardCharsets.UTF_8)))
        .isInstanceOf(Devices.Refused.class)
        .hasMessageContaining("tag 26 sits 0.500 m");
    assertThat(pit.upload(WELDED.getBytes(StandardCharsets.UTF_8)))
        .contains("Saved \"2026 FRC Rebuilt Welded\"")
        .contains("bare WPILib layout");
    assertThatThrownBy(() -> pit.choose("nope")).hasMessageContaining("no venue nope");
    assertThatThrownBy(pit::download).hasMessageContaining("official layout");
  }

  private static VenueCameras.Settings gain(int gain) {
    return new VenueCameras.Settings(Optional.empty(), Optional.of(gain));
  }

  /**
   * The pit's way through an event with no survey: on the official layout there's nowhere to keep a
   * camera's settings; the card makes the event's venue, the official layout under its name, and
   * chooses it; a tuned gain is kept in it for every camera of a type, then another for one
   * station, each time in place of the venue's old file, so the robot keeps one file for the venue,
   * not in Git until it's committed; and the robot takes each, the cameras' settings with it, while
   * disabled.
   */
  @Test
  void thePitMakesAnEventVenueAndKeepsCameraSettingsInIt(@TempDir Path stored, @TempDir Path git)
      throws IOException {
    VenueStore store = new VenueStore(stored, git);
    VenueIODisk disk = new VenueIODisk(store);
    Venues venues = venues(disk);
    VenuePit pit = new VenuePit(store, disk, venues, Rebuilt.GAME);
    venues.update(true, false);

    assertThatThrownBy(
            () -> pit.keep("front", "ThriftyCam", gain(40), VenueKeeper.For.EVERY_OF_ITS_TYPE))
        .as("on the official layout")
        .isInstanceOf(Devices.Refused.class)
        .hasMessageContaining("official layout, which keeps nothing");
    assertThatThrownBy(() -> pit.makeEventVenue(" "))
        .isInstanceOf(Devices.Refused.class)
        .hasMessageContaining("give the event's");

    assertThat(pit.makeEventVenue("Week 1")).contains("Made \"Week 1\"").contains("chose it");
    assertThat(venues.update(true, false)).as("taken while disabled").isPresent();
    assertThat(venues.active()).isEqualTo("Week 1");
    assertThat(venues.cameras()).isEqualTo(VenueCameras.NONE);
    assertThatThrownBy(() ->
            pit.keep(
                "front",
                "ThriftyCam",
                VenueCameras.Settings.NONE,
                VenueKeeper.For.EVERY_OF_ITS_TYPE))
        .as("nothing tuned")
        .isInstanceOf(Devices.Refused.class)
        .hasMessageContaining("nothing tuned on front");

    record Step(
        VenueKeeper.For kept, String station, int gain, VenueCameras expected, String says) {}
    List<Step> steps =
        List.of(
            new Step(
                VenueKeeper.For.EVERY_OF_ITS_TYPE,
                "front",
                40,
                new VenueCameras(Map.of("ThriftyCam", gain(40)), Map.of()),
                "Kept front's gain 40 in \"Week 1\", for every ThriftyCam"),
            new Step(
                VenueKeeper.For.THIS_STATION,
                "back",
                52,
                new VenueCameras(
                    Map.of("ThriftyCam", gain(40)),
                    Map.of("back", new VenueCameras.StationSettings("ThriftyCam", gain(52)))),
                "Kept back's gain 52 in \"Week 1\", for back alone"));
    for (Step step : steps) {
      assertThat(pit.keep(step.station(), "ThriftyCam", gain(step.gain()), step.kept()))
          .as(step.says())
          .startsWith(step.says());
      assertThat(venues.update(false, false)).as("not while enabled").isEmpty();
      assertThat(venues.cameras()).as("not while enabled").isNotEqualTo(step.expected());
      assertThat(venues.update(true, false)).as(step.says()).isPresent();
      assertThat(venues.cameras()).as(step.says()).isEqualTo(step.expected());
      assertThat(venues.notInGit()).as(step.says()).isTrue();
      assertThat(store.files()).as("one file for the venue").hasSize(1);
    }
    try (Devices.File file = pit.download()) {
      String text = new String(file.body().readAllBytes(), StandardCharsets.UTF_8);
      assertThat(file.name()).isEqualTo("week-1.json");
      assertThat(VenueFile.read(text).cameras()).isEqualTo(steps.get(1).expected());
    }

    // Another venue chosen while enabled waits, and keeping waits with it: the values were tuned
    // on the venue in use, not the one chosen.
    assertThat(pit.upload(venue("event", 0.03).getBytes(StandardCharsets.UTF_8))).contains("Saved");
    Devices.Option other =
        pit.card().choices().get(0).options().stream()
            .filter(option -> option.label().startsWith("Test venue"))
            .findFirst()
            .orElseThrow();
    pit.choose(other.value());
    venues.update(false, false);
    assertThatThrownBy(
            () -> pit.keep("front", "ThriftyCam", gain(41), VenueKeeper.For.THIS_STATION))
        .as("another venue waits for the robot to be disabled")
        .isInstanceOf(Devices.Refused.class)
        .hasMessageContaining("another venue is chosen");
    pit.choose(VenuePit.OFFICIAL);
    assertThatThrownBy(
            () -> pit.keep("front", "ThriftyCam", gain(41), VenueKeeper.For.THIS_STATION))
        .as("the official layout waits, the event's venue still in use")
        .isInstanceOf(Devices.Refused.class)
        .hasMessageContaining("another venue is chosen");
    assertThat(venues.active()).isEqualTo("Week 1");
  }

  /** One step of the pit's: a loop of the robot's, or a keep and what it answers. */
  private record Step(String what, Runnable loop, String station, int gain, String answer) {}

  /**
   * Keeps follow one another without waiting on the robot: each writes a new file of the venue in
   * use, which the robot takes at its next disabled loop, and the next keep writes into that file,
   * enabled or not. A venue made, or chosen, is a choice the robot takes once disabled, and a keep
   * waits for it. A chosen venue the robot can't use says why, not that something waits.
   */
  @Test
  void keepsFollowOneAnotherAndOnlyAChoiceMakesOneWait(@TempDir Path stored, @TempDir Path git)
      throws IOException {
    VenueStore store = new VenueStore(stored, git);
    VenueIODisk disk = new VenueIODisk(store);
    Venues venues = venues(disk);
    VenuePit pit = new VenuePit(store, disk, venues, Rebuilt.GAME);
    venues.update(true, false);
    pit.makeEventVenue("Week 1");
    Runnable none = () -> {};
    Runnable disabled = () -> venues.update(true, false);
    Runnable enabled = () -> venues.update(false, false);
    List<Step> steps =
        List.of(
            new Step("made, before any loop", none, "front", 40, "another venue is chosen"),
            new Step("after a disabled loop", disabled, "front", 40, "Kept front's gain 40"),
            new Step("a second keep, before a loop", none, "back", 52, "Kept back's gain 52"),
            new Step("after an enabled loop", enabled, "side", 30, "Kept side's gain 30"),
            new Step("after a disabled loop", disabled, "front", 41, "Kept front's gain 41"));
    for (Step step : steps) {
      step.loop().run();
      String answer;
      try {
        answer =
            pit.keep(step.station(), "ThriftyCam", gain(step.gain()), VenueKeeper.For.THIS_STATION);
      } catch (Devices.Refused refused) {
        answer = refused.getMessage();
      }
      assertThat(answer).as(step.what()).startsWith(step.answer());
    }
    venues.update(true, false);
    VenueCameras kept = venues.cameras();
    assertThat(kept.forCamera("front", "ThriftyCam").gain()).contains(41);
    assertThat(kept.forCamera("back", "ThriftyCam").gain()).as("kept before a loop").contains(52);
    assertThat(kept.forCamera("side", "ThriftyCam").gain()).as("kept while enabled").contains(30);
    assertThat(store.files()).as("one file for the venue").hasSize(1);

    // A file the robot can't use: a station's own settings that don't say their camera type.
    String text = venues.inUseText().strip();
    String broken =
        text.substring(0, text.lastIndexOf('}')).replaceAll("(?s),\\s*\"cameras\".*$", "")
            + ", \"cameras\": {\"stations\": {\"side\": {\"gain\": 30}}}}";
    store.save(broken);
    store.select(Venue.hash(broken));
    disk.reread();
    venues.update(true, false);
    assertThatThrownBy(
            () -> pit.keep("front", "ThriftyCam", gain(44), VenueKeeper.For.THIS_STATION))
        .isInstanceOf(Devices.Refused.class)
        .hasMessageContaining("the chosen venue can't be used")
        .hasMessageContaining("\"type\": \"ThriftyCam\"");
  }
}

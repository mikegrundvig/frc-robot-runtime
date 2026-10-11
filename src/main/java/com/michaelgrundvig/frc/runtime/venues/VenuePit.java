package com.michaelgrundvig.frc.runtime.venues;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.world.field.Venue;
import com.michaelgrundvig.frc.world.field.VenueFile;
import com.michaelgrundvig.frc.world.game.Game;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

/**
 * The venue's card on Pitwall's Pit view. It shows the venue in use and lists the venues on the
 * robot to choose among. It takes an upload, which it checks as a survey before keeping it, and it
 * hands back the selected file to download and commit.
 *
 * <p>Uploads, choices and downloads run on Pitwall's server threads, never in the robot loop. The
 * robot takes a choice at the start of a loop while it is disabled ({@link Venues}). The card
 * itself is read in the robot loop, so it reads only what the server threads have published.
 */
public final class VenuePit {
  /** The value of the option that stands for the official layout. */
  static final String OFFICIAL = "official";

  private final VenueStore store;
  private final VenueIODisk disk;
  private final Venues venues;
  private final Game game;

  /**
   * The venues on the robot to choose among, the official layout first. {@link #refresh} keeps
   * this.
   */
  private volatile List<Devices.Option> options = List.of();

  /** What is selected on the disk, by its option's value. */
  private volatile String selected = OFFICIAL;

  VenuePit(VenueStore store, VenueIODisk disk, Venues venues, Game game) {
    this.store = store;
    this.disk = disk;
    this.venues = venues;
    this.game = game;
    refresh();
  }

  /** The venue's card, for {@code pitwall.devices().add(...)}. */
  public static Devices.Device device(
      VenueStore store, VenueIODisk disk, Venues venues, Game game) {
    VenuePit pit = new VenuePit(store, disk, venues, game);
    return Devices.Device.named("venue", "Field", pit::card)
        .upload("survey", "Upload venue", pit::upload)
        .choose("venue", pit::choose)
        .download("file", "Download venue file", pit::download);
  }

  /** The card as of now: the venue in use, anything wrong with the selection, and the choice. */
  Devices.Card card() {
    List<Devices.Line> lines = new ArrayList<>();
    lines.add(new Devices.Line("In use", venues.active(), Devices.Level.OK));
    Optional<String> unusable = venues.unusable();
    if (unusable.isPresent()) {
      lines.add(new Devices.Line("Problem", unusable.get(), Devices.Level.PROBLEM));
    }
    if (venues.practiceOnField()) {
      lines.add(
          new Devices.Line(
              "Field attached",
              "a practice field's venue is set aside: choose the event's",
              Devices.Level.WARNING));
    }
    if (venues.notInGit()) {
      lines.add(
          new Devices.Line(
              "Not in Git",
              "download it and commit it to src/main/deploy/venues/",
              Devices.Level.WARNING));
    }
    if (venues.pending()) {
      lines.add(
          new Devices.Line(
              "Chosen", "takes effect when the robot is disabled", Devices.Level.NOTE));
    }
    Devices.Choice choice = new Devices.Choice("venue", "Venue", options, selected);
    return new Devices.Card(venues.active(), cardLevel(), lines, List.of(choice));
  }

  /** How the card as a whole reads: as the worst of what it says about the selection. */
  private Devices.Level cardLevel() {
    if (venues.unusable().isPresent()) {
      return Devices.Level.PROBLEM;
    }
    if (venues.practiceOnField() || venues.notInGit()) {
      return Devices.Level.WARNING;
    }
    return Devices.Level.OK;
  }

  /**
   * Checks an uploaded survey and keeps it. The upload is a venue file, or a bare WPILib layout,
   * which is wrapped as a venue with default values.
   *
   * @return what the page says it did
   * @throws Devices.Refused saying everything that is wrong with the file, or that the robot has no
   *     room for it
   */
  String upload(byte[] body) throws IOException {
    String text = new String(body, StandardCharsets.UTF_8).strip();
    Venue venue = uploaded(text);
    List<String> problems = venues.problems(venue);
    if (!problems.isEmpty()) {
      throw new Devices.Refused(String.join("; ", problems));
    }
    synchronized (this) {
      Optional<String> noRoom = store.whyNoRoomFor(venue.text());
      if (noRoom.isPresent()) {
        throw new Devices.Refused(noRoom.get());
      }
      store.save(venue.text());
      refresh();
    }
    String saved = "Saved \"" + venue.name() + "\"";
    if (venue.wrapped()) {
      saved += ", a bare WPILib layout given the default sigma and method \"unknown\"";
    }
    return saved + ". Choose it to use it.";
  }

  /**
   * The venue that an upload's text describes.
   *
   * @throws Devices.Refused if the text is neither a venue file nor a WPILib layout
   */
  private Venue uploaded(String text) throws Devices.Refused {
    try {
      if (VenueFile.bare(text)) {
        return VenueFile.wrap(text, "", Venue.Kind.EVENT, game.official().getName());
      }
      return VenueFile.read(text);
    } catch (IllegalArgumentException e) {
      throw new Devices.Refused(String.valueOf(e.getMessage()));
    }
  }

  /**
   * Selects a venue on the robot, or the official layout, for the robot to take while it is
   * disabled. The venue's file is read here, on Pitwall's thread, and handed to the robot loop.
   *
   * @param value the value of one of the card's options
   * @return what the page says it did
   * @throws Devices.Refused if the card has no such option
   */
  String choose(String value) throws IOException {
    synchronized (this) {
      Optional<Devices.Option> option =
          options.stream().filter(each -> each.value().equals(value)).findFirst();
      if (option.isEmpty()) {
        throw new Devices.Refused("there's no venue " + value + " on the robot");
      }
      if (value.equals(OFFICIAL)) {
        store.selectOfficial();
      } else {
        store.select(value);
      }
      selected = value;
      disk.reread();
      return "Chose "
          + option.get().label()
          + ": the robot takes it at the start of a loop while disabled";
    }
  }

  /**
   * The selected venue's file, for the pit to commit.
   *
   * @throws Devices.Refused if no venue is selected
   */
  Devices.File download() throws IOException {
    Optional<String> text = store.selectedText();
    if (text.isEmpty()) {
      throw new Devices.Refused("the robot uses the official layout: there's no venue file");
    }
    String name = VenueFile.read(text.get()).name();
    return Devices.File.of(
        fileName(name) + ".json", "application/json", text.get().getBytes(StandardCharsets.UTF_8));
  }

  /**
   * A file name made from a venue's name: in lower case, with a hyphen for each run of anything
   * that is not a letter or a digit. A name that leaves nothing gives "venue".
   */
  private static String fileName(String venueName) {
    String file =
        venueName.toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9]+", "-").replaceAll("^-|-$", "");
    if (file.isEmpty()) {
      return "venue";
    }
    return file;
  }

  /** Lists the venues on the robot again, and reads again which one is selected. */
  private void refresh() {
    List<Devices.Option> listed = new ArrayList<>();
    listed.add(new Devices.Option(OFFICIAL, "Official layout: " + game.official().getName()));
    try {
      for (Path file : store.files()) {
        String text = Files.readString(file, StandardCharsets.UTF_8);
        String hash = Venue.hash(text);
        // A venue that is both stored and committed is two files with one hash, and is listed
        // once.
        boolean isListed = listed.stream().anyMatch(option -> option.value().equals(hash));
        if (!isListed) {
          listed.add(new Devices.Option(hash, label(file, text)));
        }
      }
      selected = store.selectedText().map(Venue::hash).orElse(OFFICIAL);
    } catch (IOException e) {
      selected = OFFICIAL;
    }
    options = List.copyOf(listed);
  }

  /**
   * How a venue file is listed: by its venue's name and kind, "Trace event (event, not in Git)". A
   * file that cannot be read as a venue is listed by its file name, as unreadable.
   */
  private String label(Path file, String text) {
    Venue venue;
    try {
      venue = VenueFile.read(text);
    } catch (IllegalArgumentException e) {
      return file.getFileName() + " (unreadable)";
    }
    String kind = venue.kind().name().toLowerCase(Locale.ROOT);
    if (store.committed(text)) {
      return venue.name() + " (" + kind + ")";
    }
    return venue.name() + " (" + kind + ", not in Git)";
  }
}

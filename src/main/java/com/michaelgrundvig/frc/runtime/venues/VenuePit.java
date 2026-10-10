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
 * The venue's card on Pitwall's Pit view: the venue in use, the venues on the robot to choose
 * among, an upload that checks a survey before keeping it, and the selected file to download and
 * commit.
 *
 * <p>Uploads, choices and downloads run on Pitwall's server threads, never in the robot loop; the
 * robot takes a choice at the start of a loop while disabled ({@link Venues}). The card is read in
 * the loop, so it reads only what the server threads published.
 */
public final class VenuePit {
  /** The option for the official layout, by its value. */
  static final String OFFICIAL = "official";

  private final VenueStore store;
  private final VenueIODisk disk;
  private final Venues venues;
  private final Game game;

  /**
   * The venues on the robot to choose among, the official layout first: kept by {@link #refresh}.
   */
  private volatile List<Devices.Option> options = List.of();

  /** What's selected on disk, by its option's value. */
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

  Devices.Card card() {
    List<Devices.Line> lines = new ArrayList<>();
    Devices.Level level = Devices.Level.OK;
    lines.add(new Devices.Line("In use", venues.active(), Devices.Level.OK));
    String unusable = venues.unusable();
    if (unusable != null) {
      lines.add(new Devices.Line("Problem", unusable, Devices.Level.PROBLEM));
      level = Devices.Level.PROBLEM;
    }
    if (venues.practiceOnField()) {
      lines.add(
          new Devices.Line(
              "Field attached",
              "a practice field's venue is set aside: choose the event's",
              Devices.Level.WARNING));
      level = worse(level, Devices.Level.WARNING);
    }
    if (venues.notInGit()) {
      lines.add(
          new Devices.Line(
              "Not in Git",
              "download it and commit it to src/main/deploy/venues/",
              Devices.Level.WARNING));
      level = worse(level, Devices.Level.WARNING);
    }
    if (venues.pending()) {
      lines.add(
          new Devices.Line(
              "Chosen", "takes effect when the robot is disabled", Devices.Level.NOTE));
    }
    return new Devices.Card(
        venues.active(),
        level,
        lines,
        List.of(new Devices.Choice("venue", "Venue", options, selected)));
  }

  /** Checks a survey and keeps it: a venue file, or a bare WPILib layout wrapped with defaults. */
  String upload(byte[] body) throws IOException {
    String text = new String(body, StandardCharsets.UTF_8).strip();
    Venue venue;
    try {
      venue =
          VenueFile.bare(text)
              ? VenueFile.wrap(text, "", Venue.Kind.EVENT, game.official().getName())
              : VenueFile.read(text);
    } catch (IllegalArgumentException e) {
      throw new Devices.Refused(String.valueOf(e.getMessage()));
    }
    List<String> problems = venues.problems(venue);
    if (!problems.isEmpty()) {
      throw new Devices.Refused(String.join("; ", problems));
    }
    synchronized (this) {
      String full = store.whyNoRoomFor(venue.text());
      if (full != null) {
        throw new Devices.Refused(full);
      }
      store.save(venue.text());
      refresh();
    }
    return "Saved \""
        + venue.name()
        + "\""
        + (venue.wrapped()
            ? ", a bare WPILib layout given the default sigma and method \"unknown\""
            : "")
        + ". Choose it to use it.";
  }

  /**
   * Selects a venue on the robot, or the official layout, for the robot to take while disabled: its
   * file is read here, on Pitwall's thread, and handed to the loop.
   */
  String choose(String value) throws IOException {
    synchronized (this) {
      Optional<Devices.Option> option =
          options.stream().filter(each -> each.value().equals(value)).findFirst();
      if (option.isEmpty()) {
        throw new Devices.Refused("there's no venue " + value + " on the robot");
      }
      store.select(value.equals(OFFICIAL) ? null : value);
      selected = value;
      disk.reread();
      return "Chose "
          + option.get().label()
          + ": the robot takes it at the start of a loop while disabled";
    }
  }

  /** The selected venue's file, to commit. */
  Devices.File download() throws IOException {
    Optional<String> text = store.selectedText();
    if (text.isEmpty()) {
      throw new Devices.Refused("the robot uses the official layout: there's no venue file");
    }
    String name = VenueFile.read(text.get()).name();
    String file =
        name.toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9]+", "-").replaceAll("^-|-$", "");
    return Devices.File.of(
        (file.isEmpty() ? "venue" : file) + ".json",
        "application/json",
        text.get().getBytes(StandardCharsets.UTF_8));
  }

  /** Lists the venues on the robot again, and what's selected. */
  private void refresh() {
    List<Devices.Option> listed = new ArrayList<>();
    listed.add(new Devices.Option(OFFICIAL, "Official layout: " + game.official().getName()));
    try {
      for (Path file : store.files()) {
        String text = Files.readString(file, StandardCharsets.UTF_8);
        String hash = Venue.hash(text);
        if (listed.stream().anyMatch(option -> option.value().equals(hash))) {
          continue; // the same file stored and committed
        }
        String label;
        try {
          Venue venue = VenueFile.read(text);
          label =
              venue.name()
                  + " ("
                  + venue.kind().name().toLowerCase(Locale.ROOT)
                  + (store.committed(text) ? "" : ", not in Git")
                  + ")";
        } catch (IllegalArgumentException e) {
          label = file.getFileName() + " (unreadable)";
        }
        listed.add(new Devices.Option(hash, label));
      }
      String chosen = store.selectedText().map(Venue::hash).orElse(OFFICIAL);
      selected = chosen;
    } catch (IOException e) {
      selected = OFFICIAL;
    }
    options = List.copyOf(listed);
  }

  private static Devices.Level worse(Devices.Level a, Devices.Level b) {
    return a.ordinal() >= b.ordinal() ? a : b;
  }
}

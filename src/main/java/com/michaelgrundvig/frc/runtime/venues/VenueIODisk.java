package com.michaelgrundvig.frc.runtime.venues;

import java.io.IOException;
import java.util.Optional;

/**
 * The selected venue on the robot's disk: read as the robot starts, and again on Pitwall's thread
 * after the pit selects another ({@link #reread}). The robot loop takes what was read last and
 * never reads the disk itself: a venue changes once an event.
 */
public final class VenueIODisk implements VenueIO {
  /** The selection as it was read: what the loop's inputs take. */
  private record Selection(String text, String problem, boolean inGit) {}

  private final VenueStore store;

  /** Written by the thread that read the disk, read by the robot loop. */
  private volatile Selection selection;

  /** Reads the selection now, as the robot starts. */
  public VenueIODisk(VenueStore store) {
    this.store = store;
    selection = read(store);
  }

  /** Reads the disk again, on the caller's thread: after a selection. The next loop takes it. */
  public void reread() {
    selection = read(store);
  }

  @Override
  public void updateInputs(VenueIOInputs inputs) {
    Selection read = selection;
    inputs.text = read.text();
    inputs.problem = read.problem();
    inputs.inGit = read.inGit();
  }

  private static Selection read(VenueStore store) {
    try {
      Optional<String> text = store.selectedText();
      return new Selection(text.orElse(""), "", text.map(store::committed).orElse(true));
    } catch (IOException | RuntimeException e) {
      return new Selection("", String.valueOf(e.getMessage()), true);
    }
  }
}

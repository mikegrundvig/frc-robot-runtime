package com.michaelgrundvig.frc.runtime.venues;

import java.io.IOException;
import java.util.Optional;

/**
 * The selected venue, read from the robot's disk. It is read as the robot starts, and again on
 * Pitwall's thread after the pit selects another ({@link #reread}). The robot loop only takes what
 * was read last and never reads the disk itself. That is enough, because a venue changes about once
 * an event.
 */
public final class VenueIODisk implements VenueIO {
  /**
   * The selection as it was read from the disk, in the shape the loop's inputs take it.
   *
   * @param text the selected venue's file, whole; empty for the official layout
   * @param problem why the selection could not be read; empty when it could
   * @param inGit whether a file with the same contents ships with the build
   */
  private record Selection(String text, String problem, boolean inGit) {}

  private final VenueStore store;

  /** Written by whichever thread read the disk, and read by the robot loop. */
  private volatile Selection selection;

  /** Reads the selection now, as the robot starts. */
  public VenueIODisk(VenueStore store) {
    this.store = store;
    selection = read(store);
  }

  /**
   * Reads the disk again, on the caller's thread. Called after the pit makes a selection, and the
   * next robot loop takes what it read.
   */
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
      if (text.isEmpty()) {
        // The official layout: there is no file, so nothing is missing from Git.
        return new Selection("", "", true);
      }
      return new Selection(text.get(), "", store.committed(text.get()));
    } catch (IOException | RuntimeException e) {
      return new Selection("", String.valueOf(e.getMessage()), true);
    }
  }
}

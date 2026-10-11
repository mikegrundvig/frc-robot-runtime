package com.michaelgrundvig.frc.runtime.venues;

import org.littletonrobotics.junction.AutoLog;

/**
 * The selected venue's file, as read from the robot's disk. It is logged as an input, so a replay
 * solves against exactly the venue the robot used, and never reads a disk.
 */
public interface VenueIO {
  @AutoLog
  class VenueIOInputs {
    /** The selected venue's file, whole; empty for the official layout. */
    public String text = "";

    /** Why the selection could not be read. Empty when it could. */
    public String problem = "";

    /** Whether a file with the same contents ships with the build, from the repository. */
    public boolean inGit = true;
  }

  default void updateInputs(VenueIOInputs inputs) {}
}

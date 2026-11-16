package com.michaelgrundvig.frc.runtime.venues;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.world.field.VenueCameras;
import java.io.IOException;

/**
 * Keeps a camera's tuned exposure and gain in the venue in use, on the robot, so they survive a
 * restart: the venue's card does ({@link VenuePit}). A camera's card asks it, from Pitwall's
 * threads, never the robot loop's.
 */
public interface VenueKeeper {
  /** Whom a kept value is for. */
  enum For {
    /**
     * Every camera of the type tuned: the venue's value for the type, which the station tuned at
     * gives up its own of.
     */
    EVERY_OF_ITS_TYPE,
    /** The station it was tuned at alone. */
    THIS_STATION
  }

  /** A program that keeps no venues: a replay, or a test's. */
  VenueKeeper NONE =
      (station, type, tuned, kept) -> {
        throw new Devices.Refused("this program keeps no venues, so it keeps no camera settings");
      };

  /**
   * Keeps values tuned at a station in the venue in use.
   *
   * @param station the camera's station, by its name
   * @param type the camera's type, by its name
   * @param tuned the values tuned there and not kept yet: an exposure, a gain, or both
   * @param kept whom they're for
   * @return what the page says it did
   * @throws Devices.Refused if nothing was tuned, or the robot plays on the official layout, which
   *     keeps nothing
   * @throws IOException if the venue's file couldn't be written
   */
  String keep(String station, String type, VenueCameras.Settings tuned, For kept)
      throws IOException;
}

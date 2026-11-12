package com.michaelgrundvig.frc.runtime.subsystems.vision;

import java.util.List;
import java.util.Optional;

/**
 * The robot's coprocessors as Visor's frames are checked against them: the boards its configuration
 * lists, which of them each Visor camera is plugged into, once a board has reported it, and which
 * configurations each board's frames may come from, once that's known.
 */
public interface VisorBoards {
  /**
   * Each listed board's address, as the robot's configuration gives it without its agent's port: an
   * IP address, or a name. Empty for a robot that lists none, whose frames may come from anyone.
   */
  List<String> listed();

  /**
   * The listed board a camera, by its index, is plugged into, as the boards last reported it; empty
   * until one has. Asked every loop, so it allocates nothing while nothing changed.
   */
  Optional<String> placed(int camera);

  /**
   * Whether a frame from a listed board, naming the configuration with this ID, is known to come
   * from another configuration than one the robot gave that board; false while the robot doesn't
   * know which it gave it. Asked for each frame, so it allocates nothing.
   *
   * @param board the board, as {@link #listed} gives it
   */
  boolean otherConfig(String board, int configId);

  /** No boards listed: a robot that runs Visor without them, whose frames may come from anyone. */
  static VisorBoards none() {
    return new VisorBoards() {
      @Override
      public List<String> listed() {
        return List.of();
      }

      @Override
      public Optional<String> placed(int camera) {
        return Optional.empty();
      }

      @Override
      public boolean otherConfig(String board, int configId) {
        return false;
      }
    };
  }
}

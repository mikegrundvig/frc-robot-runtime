package com.michaelgrundvig.frc.runtime.lib.phoenix;

import java.util.function.BooleanSupplier;

/**
 * Whether a Phoenix device has restarted since it was last asked.
 *
 * <p>A device that loses power and comes back sends a frame saying it has started, and Phoenix says
 * whether one has come since it was last asked ({@code ParentDevice.getResetOccurredChecker}). That
 * also reports the frame a device sent when it and the robot first started, which is not a restart,
 * so the first answer is thrown away.
 *
 * <p>This holds only what has been seen, as the REV helper's does ({@code RevRestarts}). It never
 * touches a device: the IO layer hands it Phoenix's checker, or a test's own in its place.
 */
public final class PhoenixRestarts {
  private final BooleanSupplier startFrameSeen;
  private boolean askedBefore;

  /**
   * @param startFrameSeen whether the device has sent the frame it sends as it starts, since this
   *     was last asked
   */
  public PhoenixRestarts(BooleanSupplier startFrameSeen) {
    this.startFrameSeen = startFrameSeen;
  }

  /** Whether the device restarted since this was last asked. Ask once a loop. */
  public boolean happened() {
    boolean seen = startFrameSeen.getAsBoolean();
    if (!askedBefore) {
      askedBefore = true;
      return false;
    }
    return seen;
  }
}

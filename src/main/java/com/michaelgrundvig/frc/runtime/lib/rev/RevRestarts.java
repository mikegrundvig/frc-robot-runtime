package com.michaelgrundvig.frc.runtime.lib.rev;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.util.Signal;

/**
 * Whether a SPARK has restarted since it was last asked, worked out from its sticky warnings.
 *
 * <p>A SPARK sets a sticky warning, "has reset", as it starts, and keeps it until its faults are
 * cleared. Whoever reads the SPARK clears its faults as the robot starts, so that what set the
 * warning before then is not taken for a restart, and again each time this reports one. A clear
 * takes a frame or more to show, so after one the warning is not believed until a frame has shown
 * it clear. Otherwise one restart would be reported for several loops.
 *
 * <p>This holds only what has been seen. It never touches the SPARK: the IO layer reads the
 * warnings and clears the faults ({@code JointIOSpark}, {@code ModuleIOSpark}).
 */
public final class RevRestarts {
  /** Whether a "has reset" warning would be a new restart, or the last one not yet cleared. */
  private enum Warning {
    /**
     * The SPARK's faults were cleared and no frame has shown the warning clear since. A warning
     * read now is the one that was cleared.
     */
    NOT_YET_SEEN_CLEAR,
    /** A frame has shown the warning clear, so the next one that shows it is a restart. */
    SEEN_CLEAR
  }

  private Warning warning = Warning.NOT_YET_SEEN_CLEAR;

  /**
   * Takes this loop's reading of the SPARK's sticky warnings.
   *
   * @param sticky the SPARK's sticky warnings, as its frame gave them this loop
   * @return whether the SPARK restarted. The caller clears the SPARK's faults when it did, so the
   *     next restart can show.
   */
  public boolean happened(Signal<SparkBase.Warnings> sticky) {
    if (!sticky.isValid()) {
      return false;
    }
    boolean hasReset = sticky.get().hasReset;
    if (warning == Warning.NOT_YET_SEEN_CLEAR) {
      if (!hasReset) {
        warning = Warning.SEEN_CLEAR;
      }
      return false;
    }
    if (!hasReset) {
      return false;
    }
    warning = Warning.NOT_YET_SEEN_CLEAR;
    return true;
  }
}

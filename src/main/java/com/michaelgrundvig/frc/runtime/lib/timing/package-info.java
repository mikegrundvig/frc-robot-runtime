/**
 * Where the robot loop's time goes. The loop never waits on a device or the network, and its time
 * is budgeted, not hoped for: every loop is timed section by section, and overruns are counted and
 * named, so a new source of delay shows up in the log before it shows up in a match.
 *
 * <p>Components here take timestamps as arguments, like the rest of the library; the robot code
 * reads the clock. Timings are for logs and alerts only: nothing that controls the robot may depend
 * on them, or a replay would no longer reproduce the match.
 */
package com.michaelgrundvig.frc.runtime.lib.timing;

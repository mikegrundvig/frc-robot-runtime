/**
 * How the robot's vision coprocessors and their cameras are doing, judged from what the robot
 * logged of them: each camera's frame rate, dropped and lost frames and detection time over the
 * last 10 s ({@link com.michaelgrundvig.frc.runtime.lib.health.CameraHealth}), and each board's
 * reach, reboots, throttling, heat and USB ({@link
 * com.michaelgrundvig.frc.runtime.lib.health.BoardHealth}). Each warning remembers its episodes
 * this session ({@link com.michaelgrundvig.frc.runtime.lib.health.Episodes}), so the pit can see
 * after a match what happened during it.
 *
 * <p>Plain functions of what they're handed, with the time as an argument: the same log gives the
 * same judgement in a replay.
 */
@NullMarked
package com.michaelgrundvig.frc.runtime.lib.health;

import org.jspecify.annotations.NullMarked;

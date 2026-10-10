/**
 * The robot's world on Pitwall's World view ({@link
 * com.michaelgrundvig.frc.runtime.views.world.WorldView}): the field, the robot where localization
 * believes it is, its cameras' sight lines and their verdicts, and in the simulator, the truth.
 * Data for Pitwall's scene, which draws it: nothing here draws.
 *
 * <p>It reads the robot's configuration, the field model, and the loop's localization outputs,
 * handed in by the robot's container; it reaches into no mechanism, and never into the simulator.
 */
package com.michaelgrundvig.frc.runtime.views.world;

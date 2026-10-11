/**
 * The robot's world on Pitwall's World view ({@link
 * com.michaelgrundvig.frc.runtime.views.world.WorldView}): the field, the robot where localization
 * believes it is, its cameras' sight lines and their verdicts, and in the simulator, the truth.
 * Everything here is data for Pitwall's scene. Pitwall draws it, and nothing here draws.
 *
 * <p>The package reads the robot's configuration, the field model, and the loop's localization
 * outputs, all handed in by the robot's container. It reaches into no mechanism, and never into the
 * simulator.
 */
package com.michaelgrundvig.frc.runtime.views.world;

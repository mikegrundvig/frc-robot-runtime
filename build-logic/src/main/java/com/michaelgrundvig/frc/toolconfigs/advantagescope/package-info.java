/**
 * AdvantageScope's robot assets, written from the robot's configuration so a value is set once, in
 * {@code Robots.java}, and AdvantageScope agrees: a schematic robot for its 3D field, and its
 * config ({@link com.michaelgrundvig.frc.toolconfigs.advantagescope.AdvantageScopeAssets}). The
 * writer is lossy: its Javadoc lists what it drops, and a test pins the list.
 *
 * <p>The build runs it, never the robot: the AdvantageScope plugin ({@link
 * com.michaelgrundvig.frc.toolconfigs.AdvantageScopeAssetsPlugin}) puts this jar beside the
 * compiled program and calls the writer's {@code main}, which finds the robots through the {@link
 * com.michaelgrundvig.frc.world.tools.RobotCatalog} the program names, builds their configuration
 * records, and writes files. Nothing here starts the robot program, reads a sensor, or reads
 * another tool's files. It runs on the robot model of the program it writes for, from the program's
 * classpath, and has no clock, unseeded draw, thread, or static state.
 */
@NullMarked
package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import org.jspecify.annotations.NullMarked;

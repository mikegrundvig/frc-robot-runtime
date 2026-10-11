/**
 * The REV helper: a mechanism's SPARK settings, translated from how the program drives its joint
 * ({@link com.michaelgrundvig.frc.world.robot.ControllerPlan}) by {@link
 * com.michaelgrundvig.frc.runtime.lib.rev.RevMechanism}. It is the same plan the Phoenix helper
 * says in Phoenix's terms, so a mechanism on REV hardware is described once, in the robot's
 * definition, exactly as one on CTRE hardware is.
 *
 * <p>Every function is pure: it builds configurations and returns them, and never touches a device.
 * The IO layer applies what it builds ({@code ArchitectureTest} keeps it so).
 */
package com.michaelgrundvig.frc.runtime.lib.rev;

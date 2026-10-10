/**
 * The REV helper: a mechanism's SPARK settings, translated from how the program drives its joint
 * ({@link com.michaelgrundvig.frc.world.robot.ControllerPlan}), the same plan the Phoenix helper
 * says in Phoenix's terms, so a mechanism on REV hardware is described once, as on CTRE's ({@link
 * com.michaelgrundvig.frc.runtime.lib.rev.RevMechanism}). Pure functions: it builds configurations
 * and never touches a device; the IO layer applies them ({@code ArchitectureTest}).
 */
package com.michaelgrundvig.frc.runtime.lib.rev;

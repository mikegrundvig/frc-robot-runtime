/**
 * The robot's coprocessors, through Spotter's agent on each ({@link
 * com.michaelgrundvig.frc.runtime.coprocessors.Coprocessors}): Spotter's manager watches them, and
 * the robot installs and configures Visor on each that has its cameras ({@link
 * com.michaelgrundvig.frc.runtime.coprocessors.VisorInstaller}); and each board's health is logged
 * and shown on Pitwall ({@link com.michaelgrundvig.frc.runtime.coprocessors.CoprocessorHealth}).
 */
@NullMarked
package com.michaelgrundvig.frc.runtime.coprocessors;

import org.jspecify.annotations.NullMarked;

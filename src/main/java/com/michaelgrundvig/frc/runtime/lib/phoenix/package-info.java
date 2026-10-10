/**
 * The Phoenix helper: a mechanism's Phoenix 6 settings, translated from how the program drives its
 * joint ({@link com.michaelgrundvig.frc.world.robot.ControllerPlan}), so a mechanism is described
 * once, in the robot's definition, and its TalonFX's and CANcoder's ratios, gains, gravity, limits
 * and followers come out of it ({@link
 * com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism}). Nothing is retyped by hand, so no
 * ratio is written as an integer division, no gain lands per rotor turn in a slot that reads per
 * mechanism turn, and no cruise asks more than the motors turn.
 *
 * <p>The plan has every value, from three inputs, each with one owner:
 *
 * <ul>
 *   <li>the robot's definition, through its {@code RobotMath}: the joint, the drive that moves it,
 *       its encoder, and what's derived from them (ratios, the encoder's wrap, gravity, the ideal
 *       gains and the free speed);
 *   <li>the mechanism's wiring ({@link com.michaelgrundvig.frc.world.robot.MechanismWiring}): CAN
 *       IDs, which way each motor turns, the CANcoder's offset and direction;
 *   <li>its control ({@link com.michaelgrundvig.frc.world.robot.MechanismControl}): gains, soft
 *       limits, current limits and neutral mode, in the joint's units.
 * </ul>
 *
 * <p>Its limit switches come from its sensors' wiring ({@link
 * com.michaelgrundvig.frc.world.robot.ControllerPins}): one on the TalonFX's own pins, or on a
 * CANdi it reads, which it stops on by itself.
 *
 * <p>This says them in Phoenix's units, the joint's rotations ({@link
 * com.michaelgrundvig.frc.world.robot.MechanismRotations}): a revolute joint's rotations, a
 * prismatic joint's drum's rotations (the ratio is the gearing alone, the drum a {@code
 * Transmission.Drum}'s or a combined transmission's term's), and gains, cruise and acceleration
 * follow. It refuses only what Phoenix can't hold, and notes what's doubtful beside the settings.
 * The REV helper ({@code com.michaelgrundvig.frc.runtime.lib.rev}) says the same plan in REVLib's
 * terms.
 *
 * <p>Pure functions: the helper builds configurations and never touches a device, so it's tested
 * without hardware; the IO layer applies what it builds ({@code ArchitectureTest}).
 */
package com.michaelgrundvig.frc.runtime.lib.phoenix;

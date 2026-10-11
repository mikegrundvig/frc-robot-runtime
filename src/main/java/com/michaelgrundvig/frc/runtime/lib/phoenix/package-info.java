/**
 * The Phoenix helper: a mechanism's Phoenix 6 settings, translated from how the program drives its
 * joint ({@link com.michaelgrundvig.frc.world.robot.ControllerPlan}) by {@link
 * com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism}.
 *
 * <p><b>Why it exists.</b> A mechanism is described once, in the robot's definition, and the
 * settings of its TalonFX and its CANcoder come out of that description: ratios, gains, gravity,
 * limits and followers. Nothing is retyped by hand. That removes the usual mistakes: a ratio
 * written as an integer division, a gain worked out per rotor turn and put in a slot that reads per
 * mechanism turn, a cruise speed faster than the motors can turn.
 *
 * <p><b>Where the values come from.</b> The plan holds every value. It gets them from three places,
 * and each value has one owner:
 *
 * <ul>
 *   <li>The robot's definition, through its {@code RobotMath}: the joint, the drive that moves it,
 *       its encoder, and what follows from them (ratios, where the encoder's reading wraps,
 *       gravity, the ideal gains and the free speed).
 *   <li>The mechanism's wiring ({@link com.michaelgrundvig.frc.world.robot.MechanismWiring}): CAN
 *       IDs, which way each motor turns, and the CANcoder's offset and direction.
 *   <li>Its control ({@link com.michaelgrundvig.frc.world.robot.MechanismControl}): gains, soft
 *       limits, current limits and neutral mode, in the joint's units.
 * </ul>
 *
 * <p>Its limit switches come from its sensors' wiring ({@link
 * com.michaelgrundvig.frc.world.robot.ControllerPins}): a switch on one of the TalonFX's own limit
 * pins, or on a CANdi that the TalonFX reads over CAN. The TalonFX stops on either by itself.
 *
 * <p><b>Units.</b> The helper says the plan in Phoenix's units, which are the joint's rotations
 * ({@link com.michaelgrundvig.frc.world.robot.MechanismRotations}): a turning joint's own
 * rotations, or a sliding joint's drum's rotations. For a sliding joint the ratio is then the
 * gearing alone, and the drum's radius turns rotations into meters. Gains, cruise speed and
 * acceleration are converted the same way.
 *
 * <p><b>Refusals and notes.</b> The helper refuses only what Phoenix can't hold, and notes what is
 * doubtful beside the settings. The REV helper ({@code com.michaelgrundvig.frc.runtime.lib.rev})
 * says the same plan in REVLib's terms.
 *
 * <p><b>No device is touched.</b> Every function is pure: it builds configurations and returns
 * them, so the helper is tested without hardware. The IO layer applies what it builds ({@code
 * ArchitectureTest} keeps it so).
 */
package com.michaelgrundvig.frc.runtime.lib.phoenix;

/**
 * The robot program's own components: its loop timing ({@code timing}), the model's wiring in the
 * hardware's own terms ({@code wiring}), and the vendor helpers, which turn how the program drives
 * a joint into its motor controllers' settings: Phoenix's ({@code phoenix}) for a TalonFX and its
 * CANcoder, REVLib's ({@code rev}) for a SPARK. The libraries are projects of their own, in {@code
 * lib/}: the world model ({@code com.michaelgrundvig.frc.world}: the validation kit, the field, and
 * the robot's definition, its wiring and control among it), localization ({@code
 * com.michaelgrundvig.frc.localization}), and the simulation ({@code
 * com.michaelgrundvig.frc.simulation}), which only the simulator's adapters reach. Gradle keeps
 * them from reaching the robot program.
 *
 * <p>Rules, enforced by the build ({@code ArchitectureTest}):
 *
 * <ul>
 *   <li>No device (a TalonFX, a SPARK), none of the rest of the robot program, and no AdvantageKit,
 *       NetworkTables, commands or Pitwall inside a component: what it needs comes in through
 *       parameters, so it's tested without a robot.
 *   <li>No clock reads and no static state: the same inputs give the same answer, in a replay too.
 * </ul>
 *
 * <p>Every component ships with its unit tests.
 */
package com.michaelgrundvig.frc.runtime.lib;

/**
 * The model's wiring in the hardware's own terms, for the robot's IO and the vendor helpers:
 * Systemcore's CAN ports as WPILib names them ({@link
 * com.michaelgrundvig.frc.runtime.lib.wiring.CanPorts}). What a joint's leading motor controller
 * reads as its limits is worked out in the model ({@link
 * com.michaelgrundvig.frc.world.robot.ControllerPins}), from the sensors' wiring.
 *
 * <p>The wiring itself (which CAN bus, which IDs, which way each motor turns, how each encoder and
 * switch reads) is part of the robot's definition, in the world model ({@code
 * com.michaelgrundvig.frc.world.robot}: {@code MechanismWiring}, {@code SwerveWiring}, {@code
 * SensorWiring}), written in each block in {@code Robots.java}. A robot that only runs in the
 * simulator has none.
 */
package com.michaelgrundvig.frc.runtime.lib.wiring;

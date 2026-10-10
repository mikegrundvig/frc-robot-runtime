/**
 * The simulation: the simulated world's truth, and how its sensors read it. The robot's code never
 * sees any of it directly; it reads its IO, and in the simulator the robot program's adapters
 * ({@code frc.robot.sim}) answer that IO from here, as hardware would.
 *
 * <p>Where to start reading:
 *
 * <ul>
 *   <li><b>Physics:</b> {@link com.michaelgrundvig.frc.simulation.Physics} is the physics apart
 *       from the sensors and faults: the {@link com.michaelgrundvig.frc.simulation.Actuators} drive
 *       it, sub-step by sub-step, each wheel through a {@link
 *       com.michaelgrundvig.frc.simulation.WheelDrive}; it keeps the {@link
 *       com.michaelgrundvig.frc.simulation.ChassisTruth} and each {@link
 *       com.michaelgrundvig.frc.simulation.WheelTruth}; and a {@link
 *       com.michaelgrundvig.frc.simulation.Disturbance} acts on it from outside. Its physics is
 *       {@link com.michaelgrundvig.frc.simulation.KinematicPhysics}: the robot moves as its wheels
 *       roll.
 *   <li><b>Joints:</b> {@link com.michaelgrundvig.frc.simulation.JointPlant} moves each driven
 *       joint in closed form, from the robot's definition: its motors through its gearing, held to
 *       their current limit ({@link com.michaelgrundvig.frc.simulation.CurrentLimitedMotor}, which
 *       turns a swerve module's steering too), its inertia, gravity and hard stops. Its motor
 *       controller is the robot program's adapters' to emulate.
 *   <li><b>Sensors:</b> {@link com.michaelgrundvig.frc.simulation.SensorErrors} says how the
 *       simulated robot's sensors and wheels differ from what its code believes; {@link
 *       com.michaelgrundvig.frc.simulation.GyroModel} and its {@link
 *       com.michaelgrundvig.frc.simulation.AccelerometerModel} read the chassis's truth as an IMU
 *       does. The robot program's adapters hand their readings to its IO.
 *   <li><b>Faults:</b> {@link com.michaelgrundvig.frc.simulation.FaultCatalog} lists what can go
 *       wrong with each part, as data; the robot program binds each to its part and declares it to
 *       Pitwall.
 *   <li><b>Seeds:</b> {@link com.michaelgrundvig.frc.simulation.Seeds} says where each random
 *       stream starts, so a run is a function of its seed.
 * </ul>
 *
 * <p>Conventions: no static state, no clock, and no unseeded randomness, so worlds run side by side
 * and a run repeats to the bit. Frames as in {@code com.michaelgrundvig.frc.world.field}: poses in
 * the field's frame, a robot's own motion in its frame; SI units throughout.
 */
@NullMarked
package com.michaelgrundvig.frc.simulation;

import org.jspecify.annotations.NullMarked;

package com.michaelgrundvig.frc.runtime.hardware;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.world.SimulatedParts;
import org.wpilib.math.geometry.Pose2d;

/**
 * The simulator's world, as the robot program steps it and asks it: where the robot really is, and
 * the hardware it reads and drives there. Only the simulator has one, made by its {@link
 * Simulation}. The robot program sees the simulator through this alone, so none of the simulator's
 * classes are on a robot's path.
 */
public interface World extends AutoCloseable {
  /** What the robot reads and drives in this world: its simulated devices. */
  Hardware hardware();

  /** The drive team puts the robot here. */
  void place(Pose2d pose);

  /**
   * One loop: the world moves on, before the robot reads its sensors, as the real world moves
   * between a robot's loops.
   */
  void step(long nowNanos);

  /** Logs where the robot really is, and how far the estimate is from it. */
  void log(Pose2d estimated);

  /** Where the robot really is. */
  Pose2d truePose();

  /**
   * Gives Pitwall's simulator controls the world's parts that can fail, and the scenarios that can
   * happen to this robot, for the page to apply.
   */
  void addControlsTo(Components components);

  /** The world's parts that can fail, by where they are on the robot: for Pitwall's World view. */
  SimulatedParts parts();

  /** Releases what it holds, such as its cameras' simulation. */
  @Override
  void close();
}

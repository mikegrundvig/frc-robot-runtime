package com.michaelgrundvig.frc.simulation;

import org.wpilib.math.geometry.Translation3d;

/**
 * One wheel as it truly is: how far it has turned and how fast, which way it points, how much of
 * the robot it carries, and where it meets the ground. Its encoders and its motor controller read
 * it; the robot's code sees it through them alone. A live view of its {@link Physics}: read it
 * before the next step or change.
 */
public interface WheelTruth {
  /** How far it has turned since the start, in radians: what its drive encoder counts. */
  double rotationRadians();

  /** How fast it turns, in radians a second. */
  double spinRadPerSec();

  /** Which way it truly points, counterclockwise from the robot's forward axis. */
  double steerRadians();

  /** Its true rolling radius, in meters: the configured one, less any wear. */
  double radiusMeters();

  /**
   * The current through its drive motor now, in amps: the motor's own, at its voltage and speed.
   */
  double driveCurrentAmps();

  /**
   * The voltage across its drive motor as the last sub-step ended: what its controller asked
   * ({@link WheelDrive}), held to its current limit and the battery.
   */
  double driveVolts();

  /**
   * Whether the drive's current limit held its motor back in the last sub-step: the controller
   * asked for more than the limit lets through, as a motor controller's stator-limit flag says.
   */
  boolean driveCurrentLimited();

  /**
   * What the floor pushes back on its steering with, about its steering axis, in N m: the scrub of
   * its tread turning on the carpet, for the steering's closed form to feel. Zero where the physics
   * models no scrub, as the template's doesn't; the robot program's steering doesn't read it until
   * a real module's scrub is measured.
   */
  double steerLoadNewtonMeters();

  /** How hard it presses on the ground, in newtons. */
  double loadNewtons();

  /** Whether it touches the ground. */
  boolean inContact();

  /**
   * Where its tread meets the ground, on the field, or where it would: on the plane the robot
   * stands on, under the wheel. Above the ground there, it hangs; below it, it's sunk in.
   */
  Translation3d contactPoint();
}

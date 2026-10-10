package com.michaelgrundvig.frc.runtime.examples;

import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveMove;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.Random;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Drives forward until its front bumper hits something, turns a quarter turn left or right, and
 * drives on: a robot exploring a room. If the bumper never trips, the robot stops.
 */
final class Explorer {
  /** Straight ahead at 1.5 m/s. */
  static final ChassisVelocities FORWARD = new ChassisVelocities(1.5, 0, 0);

  private final Drive drive;
  private final Gate bumped;
  private final Random random;

  Explorer(Drive drive, Joints joints, Sensor.Switch frontBumper, Random random) {
    this.drive = drive;
    this.random = random;
    bumped = joints.gate("bumped", frontBumper.tripped());
  }

  Command explore() {
    return Command.requiring(drive)
        .executing(
            co -> {
              while (true) {
                DriveMove forward = drive.runUntil(FORWARD, bumped).within(Seconds.of(15));
                co.await(forward);
                if (!forward.arrived()) {
                  return; // the bumper never tripped: the move says why, and the robot stops
                }
                co.await(drive.turnTo(nextQuarter()));
              }
            })
        .named("Explore");
  }

  /** A quarter turn left or right of the way the robot faces, square to the room. */
  private Rotation2d nextQuarter() {
    double facing = Math.round(drive.getPose().getRotation().getDegrees() / 90) * 90;
    return Rotation2d.fromDegrees(facing + (random.nextBoolean() ? 90 : -90));
  }
}

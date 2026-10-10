package com.michaelgrundvig.frc.runtime.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

/** The gyro's door to hardware. The gyro owns the robot's heading. */
public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw = Rotation2d.ZERO;
    public double yawVelocityRadPerSec = 0.0;

    /**
     * The robot's attitude: tilted on a bump, a ramp, or a wheel up on a game piece. WPILib's
     * convention: roll turns about the robot's forward axis and pitch about its left axis, each
     * counterclockwise seen from the axis's tip, so a positive pitch is nose down.
     */
    public double pitchRad = 0.0;

    public double rollRad = 0.0;

    /**
     * Acceleration along the robot's forward and left axes, as the IMU feels it: a jolt beyond what
     * driving explains is a collision, and a change of speed the wheels don't show is them sliding.
     * Gravity's share is in it too when the robot tilts, which localization takes out by the
     * attitude. Not a number from an IMU that can't say.
     */
    public double accelerationXMetersPerSecSq = 0.0;

    public double accelerationYMetersPerSecSq = 0.0;

    /** Yaw readings taken since the last loop, with the modules' readings (see ModuleIO). */
    public long[] odometryTimestampsNanos = new long[] {};

    public double[] odometryYawsRad = new double[] {};
  }

  default void updateInputs(GyroIOInputs inputs) {}
}

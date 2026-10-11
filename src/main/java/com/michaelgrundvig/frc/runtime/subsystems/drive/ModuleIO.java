package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

/**
 * One swerve module's door to hardware: a simulated module, a real one, or nothing (in replay,
 * where the inputs come from the log). Drive values are at the wheel, in radians, so the wheel
 * radius (which changes with wear) lives in one place: {@link Module}.
 *
 * <h2>The contract</h2>
 *
 * <p>What every module driver keeps, clause by clause, as the joints' drivers keep theirs ({@code
 * JointIO}). The conformance suite ({@code ModuleIOConformance}, in the test fixtures) checks the
 * clauses about units, signs and odometry readings against every driver here: M1, M3, M5 to M10,
 * M12 and M17. The rest are written down and not yet checked by it. Where a driver here is known to
 * differ from a clause, the clause says so: those are open questions, not leave to differ.
 *
 * <p><b>Readings</b>
 *
 * <ul>
 *   <li><b>M1.</b> The drive's position is the wheel's turning in radians, and its velocity in
 *       radians a second, through the drive's gearing. Positive rolls the robot forward with the
 *       module pointing straight ahead, whichever way the motor is mounted.
 *   <li><b>M2.</b> Where turning the module also turns the wheel through the module's gears, that
 *       share is left out of the wheel's position and velocity, and of each odometry reading.
 *   <li><b>M3.</b> The steering's angle is the module's, from its absolute encoder with the
 *       wiring's offset and direction: zero straight ahead, counterclockwise positive, within half
 *       a turn either way. It is right from the first reading, with no homing.
 *   <li><b>M4.</b> The steering's velocity is how fast that angle changes, in radians a second.
 *   <li><b>M8.</b> Each applied voltage is the voltage across that motor with its sign: positive
 *       rolls the wheel forward, or turns the module counterclockwise.
 *   <li><b>M9.</b> The drive's stator current is a size: never negative. Known difference: the CTRE
 *       module's is signed, negative while the motor sends current back.
 *   <li><b>M10.</b> The drive's supply current is a size: never negative. Known difference: the
 *       CTRE module's is signed.
 *   <li><b>M11.</b> The steering's stator current is the largest size it had since the last loop.
 *   <li><b>M12.</b> The odometry readings are three lists of one length, oldest first: when each
 *       was taken, on the robot's clock in nanoseconds and never after now, the wheel's position
 *       then in radians, and the module's angle then in radians. A driver that reads no faster than
 *       the loop gives one reading a loop.
 *   <li><b>M17.</b> {@link ModuleIOInputs#unconfigured} is empty when every device took its
 *       settings at startup.
 *   <li><b>M18.</b> {@link ModuleIOInputs#driveTorqueLimited} is true when the drive's stator limit
 *       held it since the last loop, where the device says so. A SPARK does not, and reads false.
 * </ul>
 *
 * <p><b>Requests</b>
 *
 * <ul>
 *   <li><b>M5.</b> {@link #setDriveVelocity} runs the wheel at a speed on the drive controller's
 *       loop: the drive's kP, in volts for each meter a second of the wheel's rim, times how far
 *       the wheel's speed reads from the speed asked, plus the feedforward volts given. A positive
 *       speed drives the wheel forward. Each request holds until the next.
 *   <li><b>M6.</b> {@link #setSteerAngle} holds the module at an angle on the steering controller's
 *       loop, which runs on the absolute encoder's angle with the steering's kP and kD, and goes
 *       the short way round.
 *   <li><b>M7.</b> {@link #stop} drives neither motor, and both brake.
 *   <li><b>M13.</b> {@link #configure} at startup sends each device its whole configuration and
 *       waits for it. Called again, it puts the gains and current limits in force without waiting.
 * </ul>
 *
 * <p><b>Silence, lost frames and restarts</b>
 *
 * <ul>
 *   <li><b>M14.</b> {@link ModuleIOInputs#driveConnected} and {@link ModuleIOInputs#steerConnected}
 *       are each false once that motor's controller has been silent for half a second, and true
 *       again with the first frame that comes. A shorter silence leaves them true.
 *   <li><b>M15.</b> In a loop a frame did not come, the readings stand as last read, and so does
 *       that loop's odometry reading: a lost frame is never a wheel at zero. Known difference: the
 *       CTRE module's odometry reading for a frame that did not come is not a number, and the
 *       drivetrain passes it over.
 *   <li><b>M16.</b> A controller that restarts has the gains and limits of the last {@link
 *       #configure} in force again by the next loop. Open: a restarted drive controller counts its
 *       wheel from zero, which reads as the wheel jumping back, and no driver says that it
 *       restarted. The inputs have nowhere to say it.
 * </ul>
 */
public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;

    /**
     * Whether the drive motor hit its stator current limit since the last loop: the torque it can
     * give is capped. Logged for tuning the limit; it isn't slip, which a limit set below the
     * tread's grip prevents.
     */
    public boolean driveTorqueLimited = false;

    public boolean steerConnected = false;

    /** From the absolute encoder: valid at power-on, no homing needed. */
    public Rotation2d steerAngle = Rotation2d.ZERO;

    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;

    /**
     * The most current through the steering motor since the last loop, either way: a turn draws its
     * peak for a few milliseconds as it starts and stops, which a reading once a loop would almost
     * always miss. Read at 250 Hz on a robot, with odometry; every millisecond in the simulator.
     */
    public double steerStatorCurrentAmps = 0.0;

    /**
     * Readings taken since the last loop, oldest first, faster than the loop runs (250 Hz on a
     * robot) so odometry sees every turn of the wheel: when each was taken, on the robot's clock,
     * and the wheel's position and angle then. Angles are plain radians, not Rotation2d: a replay
     * must hand the estimator the very same numbers, and a Rotation2d is rebuilt from its cosine
     * and sine, a bit off.
     */
    public long[] odometryTimestampsNanos = new long[] {};

    public double[] odometryDrivePositionsRad = new double[] {};
    public double[] odometrySteerAnglesRad = new double[] {};

    /**
     * The module's devices that didn't take their configuration at startup, such as "drive motor 3,
     * CANcoder 9", or empty: they run on their defaults, without the configured directions,
     * gearing, or current limits.
     */
    public String unconfigured = "";
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Sets the motor controllers' gains and current limits: at startup, and again whenever tuning
   * changes them.
   */
  default void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {}

  /** Closed-loop wheel velocity on the motor controller, plus robot-computed feedforward. */
  default void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {}

  /** Closed-loop steer angle on the motor controller. */
  default void setSteerAngle(Rotation2d angle) {}

  /** Stop both motors. */
  default void stop() {}
}

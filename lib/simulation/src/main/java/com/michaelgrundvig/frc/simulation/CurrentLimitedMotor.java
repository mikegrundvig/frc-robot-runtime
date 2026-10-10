package com.michaelgrundvig.frc.simulation;

import org.wpilib.math.system.DCMotor;

/**
 * A motor turning a load through its gearing, its motor controller holding the current through it
 * to a limit: a swerve module's steering, or one of the robot's joints ({@link JointPlant}), which
 * gravity pulls on too.
 *
 * <p>Its motion is worked out exactly rather than stepped, because a steering motor is quick: with
 * the simulated inertia, its speed closes on the speed its voltage gives with a time constant under
 * a millisecond, well inside one of the simulator's steps. WPILib's {@code DCMotorSim} holds a
 * voltage through each step, so a current limit set at the step's start would hold only at that
 * instant, and the motor would give far less torque than its limit allows. A motor controller holds
 * the limit the whole time. So here, at the limit, the motor gives a steady torque until its back
 * EMF leaves no more than the limit's worth of voltage across its windings; below the limit, the
 * load's speed closes on the speed the voltage gives, exponentially, exactly as in {@code
 * DCMotorSim}. The drive's limit is the physics' to hold ({@link WheelDrive}): its motor is about
 * sixty times slower, and today's physics holds the limit at each step's start, so it falls a
 * little short of the limit within each step (the truth suite's F = m a gap).
 *
 * <p>A torque from outside, gravity on a joint, is held steady through each step and adds to the
 * motor's: below the limit it moves the speed the load closes on; at the limit it adds to the
 * steady acceleration, and a motor held at its limit by more than it can lift stays there through
 * the step. Taken at each step's start, it's exact for steps short against how fast the load's
 * angle changes it: the simulator steps a joint every millisecond, as its motor controller runs.
 *
 * <p>Friction is left out: a real module's wheel scrubs on the carpet as it turns, and its gearbox
 * drags, so a real one coasts less than this one. Both need measuring on a real module before they
 * belong here. Once measured, add them in {@link #run} as a steady torque against the motion: it
 * takes {@code torque / (gearing * Kt)} amps of the motor's current, both at the limit and below
 * it, and the module stays put while the motor's torque can't beat it.
 */
public final class CurrentLimitedMotor {
  private final DCMotor motor;
  private final double gearing;
  private final double inertia;

  /** The load's acceleration per amp through the motor, in radians per second squared. */
  private final double accelerationPerAmp;

  /**
   * How quickly the load's speed closes on the speed its voltage gives: one over a time constant.
   */
  private final double settlingRate;

  private double positionRadians;
  private double velocityRadPerSec;
  private double appliedVolts;
  private boolean coasting;

  /**
   * @param motor the motor, or the motors turning the load together
   * @param inertiaKgM2 the load's moment of inertia, or its mass for a load that slides
   * @param gearing motor rotations per rotation of the load, or radians per meter of one that
   *     slides
   */
  public CurrentLimitedMotor(DCMotor motor, double inertiaKgM2, double gearing) {
    this.motor = motor;
    this.gearing = gearing;
    this.inertia = inertiaKgM2;
    accelerationPerAmp = gearing * motor.Kt / inertiaKgM2;
    settlingRate = gearing * gearing * motor.Kt / (motor.Kv * motor.R * inertiaKgM2);
  }

  /**
   * Runs the motor for a time, its controller asking for a voltage and holding the current to a
   * limit.
   *
   * @param volts the voltage the controller asks for
   * @param limitAmps the most current it lets through the motor, either way
   * @param seconds how long
   * @return the most current through the motor in that time, in amps: the current at the start,
   *     since it only falls as the motor's speed catches up with its voltage
   */
  public double run(double volts, double limitAmps, double seconds) {
    return run(volts, limitAmps, 0, seconds);
  }

  /**
   * Runs the motor for a time, as {@link #run(double, double, double)} does, with a torque from
   * outside on the load the whole time.
   *
   * @param loadTorque the torque from outside, gravity's, in newton meters (newtons for a load that
   *     slides), positive turning the load forward
   * @return the current through the motor at the start, held to the limit
   */
  public double run(double volts, double limitAmps, double loadTorque, double seconds) {
    coasting = false;
    double pulled = loadTorque / inertia;
    double amps = motor.getCurrent(velocityRadPerSec * gearing, volts);
    double remaining = seconds;
    if (Math.abs(amps) > limitAmps) {
      // At the limit: a steady torque, until the speed leaves no more than the limit's current.
      double limit = Math.copySign(limitAmps, amps);
      double acceleration = limit * accelerationPerAmp + pulled;
      double releaseSpeed = (volts - limit * motor.R) * motor.Kv / gearing;
      double toRelease = (releaseSpeed - velocityRadPerSec) / acceleration;
      double atLimit = toRelease >= 0 ? Math.min(toRelease, remaining) : remaining;
      positionRadians += (velocityRadPerSec + acceleration * atLimit / 2) * atLimit;
      velocityRadPerSec += acceleration * atLimit;
      remaining -= atLimit;
      if (remaining <= 0) {
        appliedVolts = velocityRadPerSec * gearing / motor.Kv + limit * motor.R;
        return limitAmps;
      }
    }
    // Below the limit: the speed closes on the voltage's free speed, shifted by the load, exactly.
    double freeSpeed = volts * motor.Kv / gearing + pulled / settlingRate;
    double decay = Math.exp(-settlingRate * remaining);
    positionRadians +=
        freeSpeed * remaining + (velocityRadPerSec - freeSpeed) * (1 - decay) / settlingRate;
    velocityRadPerSec = freeSpeed + (velocityRadPerSec - freeSpeed) * decay;
    appliedVolts = volts;
    return Math.min(Math.abs(amps), limitAmps);
  }

  /**
   * Lets the load turn freely for a time, no current through the motor, as a motor controller in
   * coast leaves it: only the torque from outside moves it.
   */
  public void coast(double loadTorque, double seconds) {
    double acceleration = loadTorque / inertia;
    positionRadians += (velocityRadPerSec + acceleration * seconds / 2) * seconds;
    velocityRadPerSec += acceleration * seconds;
    coasting = true;
    appliedVolts = 0;
  }

  /**
   * Holds the load still, as a hard stop does that it was driven into: its speed zero, and the
   * current its controller's voltage drives through the stalled motor, held to the limit. With no
   * speed there's no back EMF, so the voltage across the motor is what drives that current through
   * its windings: the controller's, or at the limit, the limit's worth.
   *
   * @param volts the voltage the controller asks for
   * @param limitAmps the most current it lets through the motor, either way
   */
  public void stall(double volts, double limitAmps) {
    velocityRadPerSec = 0;
    double amps = motor.getCurrent(0, volts);
    appliedVolts = Math.abs(amps) <= limitAmps ? volts : Math.copySign(limitAmps, amps) * motor.R;
    coasting = false;
  }

  /** Puts the load at a position and speed: against a hard stop, where it's held still. */
  public void place(double position, double velocity) {
    positionRadians = position;
    velocityRadPerSec = velocity;
  }

  /** The load's angle, in radians from where it started. */
  public double positionRadians() {
    return positionRadians;
  }

  /** The load's speed, in radians per second. */
  public double velocityRadPerSec() {
    return velocityRadPerSec;
  }

  /** The voltage across the motor now: less than the controller asks for, at the limit. */
  public double appliedVolts() {
    return appliedVolts;
  }

  /** The current through the motor now, in amps: none while it coasts. */
  public double currentAmps() {
    return coasting ? 0 : motor.getCurrent(velocityRadPerSec * gearing, appliedVolts);
  }
}

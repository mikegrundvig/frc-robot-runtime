package com.michaelgrundvig.frc.simulation;

import org.wpilib.math.system.DCMotor;

/**
 * A motor turning a load through its gearing, with its motor controller holding the current through
 * it to a limit: a swerve module's steering, or one of the robot's joints ({@link JointPlant}),
 * which gravity pulls on too.
 *
 * <p><b>The model.</b> A brushed DC motor's two equations, with V the voltage across the motor, I
 * its current, ω the load's speed, G the gearing and J the load's inertia:
 *
 * <pre>
 *   V = I·R + G·ω / Kv                    the voltage drives the current and overcomes the back EMF
 *   J · dω/dt = G · Kt · I + loadTorque   the current's torque, and any torque from outside
 * </pre>
 *
 * <p>The motion over a step is worked out exactly, in up to two stretches ({@link #run}):
 *
 * <ul>
 *   <li><b>At the limit.</b> While the voltage asked for would drive more current than the limit
 *       allows, the controller holds the current at the limit. A steady current is a steady torque,
 *       so the load's speed changes at a steady rate, until its back EMF has grown enough that the
 *       voltage asked for drives no more than the limit's current.
 *   <li><b>Below the limit.</b> The voltage is what was asked for. Taking the current out of the
 *       two equations leaves dω/dt = −k·(ω − ω<sub>free</sub>): the speed closes on the speed the
 *       voltage gives, exponentially, at the rate k = G²·Kt / (Kv·R·J). This is exactly what
 *       WPILib's {@code DCMotorSim} works out.
 * </ul>
 *
 * <p><b>Why it is not simply stepped.</b> A steering motor is quick: with the simulated inertia, k
 * is more than a thousand a second, so the speed settles well inside one of the simulator's steps.
 * {@code DCMotorSim} holds a voltage through each step, so a current limit applied at the step's
 * start would hold only at that instant, and the motor would give far less torque than its limit
 * allows. A real motor controller holds the limit the whole time, as the first stretch does here. A
 * swerve wheel's drive motor is about sixty times slower, and the physics holds its limit at each
 * step's start ({@link WheelMotor}).
 *
 * <p><b>A torque from outside</b>, such as gravity on a joint, is taken as steady through each
 * step. That is exact for steps that are short against how fast the load's angle changes the
 * torque: the simulator steps a joint every millisecond, as its motor controller runs.
 *
 * <p><b>What is left out.</b> Friction: a real module's wheel scrubs on the carpet as it turns, and
 * its gearbox drags, so a real one coasts less than this one. Both need measuring on a real module
 * before they belong here. Once measured, they go in {@link #run} as a steady torque against the
 * motion: it takes {@code torque / (gearing * Kt)} amps of the motor's current, both at the limit
 * and below it, and the module stays put while the motor's torque cannot beat it.
 */
public final class CurrentLimitedMotor {
  private final DCMotor motor;

  /** Motor turns for each turn of the load, or radians of motor for each meter of a slide. */
  private final double gearing;

  /** The load's moment of inertia, in kg m², or its mass in kg for a load that slides. */
  private final double inertia;

  /** The load's acceleration for each amp through the motor: G·Kt / J. */
  private final double accelerationPerAmp;

  /**
   * How quickly the load's speed closes on the speed its voltage gives, in 1/s: k = G²·Kt /
   * (Kv·R·J), one over the motor's mechanical time constant.
   */
  private final double settlingRate;

  private double positionRadians;
  private double velocityRadPerSec;
  private double appliedVolts;

  /** Whether the motor carries no current at all, as its controller in coast leaves it. */
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
    double loadAcceleration = loadTorque / inertia;
    double motorSpeedRadPerSec = velocityRadPerSec * gearing;
    double ampsAtStart = motor.getCurrent(motorSpeedRadPerSec, volts);
    boolean startsAtTheLimit = Math.abs(ampsAtStart) > limitAmps;
    double secondsBelowTheLimit = seconds;
    if (startsAtTheLimit) {
      double heldAmps = Math.copySign(limitAmps, ampsAtStart);
      secondsBelowTheLimit -= runAtTheLimit(volts, heldAmps, loadAcceleration, seconds);
      boolean stayedAtTheLimit = secondsBelowTheLimit <= 0;
      if (stayedAtTheLimit) {
        // The voltage that drives the held current at the speed the load ends at: back EMF + I·R.
        appliedVolts = velocityRadPerSec * gearing / motor.Kv + heldAmps * motor.R;
        return limitAmps;
      }
    }
    settleBelowTheLimit(volts, loadAcceleration, secondsBelowTheLimit);
    appliedVolts = volts;
    return Math.min(Math.abs(ampsAtStart), limitAmps);
  }

  /**
   * The first stretch of a step: the controller holds the current at its limit, so the motor gives
   * a steady torque and the load's speed changes at a steady rate. It lasts until the load reaches
   * the speed at which the voltage asked for drives exactly the limit's current, or until the step
   * ends.
   *
   * @param heldAmps the limit, with the sign of the current the voltage would drive
   * @param loadAcceleration what the torque from outside adds to the load's acceleration
   * @param seconds the whole step's length
   * @return how long the stretch lasted, in seconds: the whole step, if the limit never let go
   */
  private double runAtTheLimit(
      double volts, double heldAmps, double loadAcceleration, double seconds) {
    double acceleration = heldAmps * accelerationPerAmp + loadAcceleration;
    // From V = I·R + G·ω/Kv with I at the limit: the speed at which the limit lets go.
    double releaseSpeed = (volts - heldAmps * motor.R) * motor.Kv / gearing;
    double secondsToRelease = (releaseSpeed - velocityRadPerSec) / acceleration;
    // A negative time means the load is moving away from the release speed: the torque from
    // outside is more than the motor at its limit can hold, and the limit holds all step.
    boolean headedForRelease = secondsToRelease >= 0;
    double secondsAtLimit = seconds;
    if (headedForRelease) {
      secondsAtLimit = Math.min(secondsToRelease, seconds);
    }
    // Steady acceleration: x += (v + a·t/2)·t, then v += a·t.
    positionRadians += (velocityRadPerSec + acceleration * secondsAtLimit / 2) * secondsAtLimit;
    velocityRadPerSec += acceleration * secondsAtLimit;
    return secondsAtLimit;
  }

  /**
   * The second stretch of a step, or all of it: below the limit, the speed closes on the speed the
   * voltage gives, exponentially. With k the settling rate and ω<sub>free</sub> the speed it closes
   * on, the exact solution over a time t is
   *
   * <pre>
   *   ω(t) = ω_free + (ω₀ − ω_free)·e^(−k·t)
   *   x(t) = x₀ + ω_free·t + (ω₀ − ω_free)·(1 − e^(−k·t)) / k
   * </pre>
   *
   * <p>A steady torque from outside shifts ω<sub>free</sub> by its acceleration over k: the speed
   * at which the motor's torque balances it.
   */
  private void settleBelowTheLimit(double volts, double loadAcceleration, double seconds) {
    double freeSpeed = volts * motor.Kv / gearing + loadAcceleration / settlingRate;
    double decay = Math.exp(-settlingRate * seconds);
    positionRadians +=
        freeSpeed * seconds + (velocityRadPerSec - freeSpeed) * (1 - decay) / settlingRate;
    velocityRadPerSec = freeSpeed + (velocityRadPerSec - freeSpeed) * decay;
  }

  /**
   * Lets the load turn freely for a time, no current through the motor, as a motor controller in
   * coast leaves it: only the torque from outside moves it, at a steady acceleration.
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
    coasting = false;
    double stalledAmps = motor.getCurrent(0, volts);
    boolean withinTheLimit = Math.abs(stalledAmps) <= limitAmps;
    if (withinTheLimit) {
      appliedVolts = volts;
      return;
    }
    appliedVolts = Math.copySign(limitAmps, stalledAmps) * motor.R;
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
    if (coasting) {
      return 0;
    }
    return motor.getCurrent(velocityRadPerSec * gearing, appliedVolts);
  }
}

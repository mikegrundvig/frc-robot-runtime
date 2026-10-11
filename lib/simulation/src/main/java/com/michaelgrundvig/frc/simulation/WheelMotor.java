package com.michaelgrundvig.frc.simulation;

import java.util.Optional;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Nat;

/**
 * A swerve wheel's drive motor, turning the wheel and the share of the robot the wheel carries.
 *
 * <p><b>The model.</b> It is a brushed DC motor's two equations, exactly as WPILib's {@code
 * DCMotorSim} has them. With V the voltage across the motor, I its current, ω the wheel's speed, G
 * the gearing (motor turns for each turn of the wheel) and J the load's inertia at the wheel:
 *
 * <pre>
 *   V = I·R + G·ω / Kv          the voltage drives the current and overcomes the back EMF
 *   J · dω/dt = G · Kt · I      the current's torque, through the gearing, turns the load
 * </pre>
 *
 * <p>Taking the current out of the two gives a linear system in the wheel's angle and speed, driven
 * by the voltage:
 *
 * <pre>
 *   d/dt [angle]   [ 0        1          ] [angle]   [      0       ]
 *        [speed] = [ 0  −G²·Kt/(Kv·R·J) ] [speed] + [ G·Kt/(R·J)  ] · V
 * </pre>
 *
 * <p>The voltage is held through each step, so the step is taken exactly, with the system's
 * discrete form for a step of that length ({@link Discretized}).
 *
 * <p><b>The current limit.</b> The motor's controller holds the current to a limit. It is applied
 * here as a TalonFX applies it at the instant the step starts: the voltage is brought to within the
 * limit's worth of the back EMF. That voltage is then held through the step, and as the wheel
 * speeds up the current falls below the limit, so the robot accelerates a little less than F = m·a
 * at the limit would give. The truth suite keeps that as a known gap.
 *
 * <p><b>What is left out.</b> The velocity loop's gain within a step ({@link
 * WheelDrive#voltsPerRadPerSec}) is not used, because the voltage is held. Friction in the gearbox
 * and the tread's rolling resistance are not modeled.
 */
final class WheelMotor {
  /**
   * The motor's model made discrete for one length of step: over a step with the voltage held, the
   * state after is {@code a · state + b · volts}. Working it out takes a matrix exponential in
   * native code, so it is worked out once for each length of step, and today every step is the same
   * length.
   *
   * @param seconds the length of step it was worked out for
   * @param a how the state carries over a step: a 2 × 2 matrix
   * @param b what the held voltage adds over a step: a 2 × 1 matrix
   */
  private record Discretized(double seconds, Matrix<N2, N2> a, Matrix<N2, N1> b) {}

  private final DCMotor motor;

  /** The continuous system above. WPILib 2027 names this model for an arm with no gravity. */
  private final LinearSystem<N2, N1, N2> model;

  /**
   * The gearing as the drivetrain's definition gives it: motor turns for each turn of the wheel.
   */
  private final double configuredGearing;

  /**
   * The gearing as {@code DCMotorSim} works it back out of the model's own matrices. It is the same
   * number but for rounding, and the motor's current is worked out with it, as {@code DCMotorSim}
   * works it out, so that the current matches recorded runs to the bit.
   */
  private final double gearingFromModel;

  private Optional<Discretized> discretized = Optional.empty();

  /** The wheel's angle, in radians, and its speed, in radians a second, in that order. */
  private Matrix<N2, N1> state = new Matrix<>(Nat.N2(), Nat.N1());

  /** The voltage held across the motor through the last step. */
  private Matrix<N1, N1> heldVolts = new Matrix<>(Nat.N1(), Nat.N1());

  /** What the model reports of its state: the angle and the speed again, in the same order. */
  private Matrix<N2, N1> output = new Matrix<>(Nat.N2(), Nat.N1());

  private boolean currentLimited;

  /**
   * @param motor the drive motor, with the gearbox's losses already taken off
   * @param inertiaKgM2 the load at the wheel: the wheel's share of the robot and its own moving
   *     parts ({@link KinematicPhysics#driveInertiaKgM2})
   * @param gearing motor turns for each turn of the wheel
   */
  WheelMotor(DCMotor motor, double inertiaKgM2, double gearing) {
    this.motor = motor;
    configuredGearing = gearing;
    model = Models.singleJointedArmFromPhysicalConstants(motor, inertiaKgM2, gearing);
    // From the system above: A's speed entry is −G²·Kt/(Kv·R·J) and B's is G·Kt/(R·J), so
    // −Kv · A / B is G.
    double speedFromSpeed = model.getA(1, 1);
    double speedFromVolts = model.getB(1, 0);
    gearingFromModel = -motor.Kv * speedFromSpeed / speedFromVolts;
  }

  /**
   * Turns the wheel through one step.
   *
   * @param askedVolts what the motor's controller asks to put across the motor
   * @param currentLimitAmps the most current the controller lets through the motor, either way
   * @param batteryVolts the most voltage the controller can put across the motor, either way
   * @param seconds the step's length
   */
  void turn(double askedVolts, double currentLimitAmps, double batteryVolts, double seconds) {
    // At a current I the voltage across the motor is the back EMF plus I·R, so holding the current
    // within the limit means holding the voltage within the limit's I·R of the back EMF.
    double backEmfVolts = speedRadPerSec() * configuredGearing / motor.Kv;
    double limitVolts = currentLimitAmps * motor.R;
    double withinCurrentLimit =
        Math.clamp(askedVolts, backEmfVolts - limitVolts, backEmfVolts + limitVolts);
    currentLimited = withinCurrentLimit != askedVolts;
    // The battery caps the voltage too: scaled down to the battery's, never up.
    double batteryShare = Math.min(1.0, batteryVolts / Math.abs(withinCurrentLimit));
    heldVolts = VecBuilder.fill(withinCurrentLimit * batteryShare);

    Discretized step = discretizedFor(seconds);
    // The same arithmetic as WPILib's LinearSystem.calculateX, to the bit, with the discrete form
    // worked out once and not every step.
    state = step.a().times(state).plus(step.b().times(heldVolts));
    output = model.calculateY(state, heldVolts);
  }

  /** The model's discrete form for a step of this length, worked out if it has not been yet. */
  private Discretized discretizedFor(double seconds) {
    if (discretized.isPresent() && discretized.get().seconds() == seconds) {
      return discretized.get();
    }
    Discretized made = discretize(seconds);
    discretized = Optional.of(made);
    return made;
  }

  /**
   * Makes the model discrete for a step of one length, exactly as WPILib's {@code
   * Discretization.discretizeAB} does (which hands its result back in a type WPILib's math keeps to
   * itself). With A and B the continuous system's matrices, the matrix exponential
   *
   * <pre>
   *   exp( [ A  B ] · seconds )  =  [ a  b ]
   *        [ 0  0 ]                 [ 0  1 ]
   * </pre>
   *
   * <p>holds the discrete a in its top-left 2 × 2 block and the discrete b in the 2 × 1 block
   * beside it.
   */
  private Discretized discretize(double seconds) {
    Matrix<N3, N3> continuous = new Matrix<>(Nat.N3(), Nat.N3());
    continuous.assignBlock(0, 0, model.getA());
    continuous.assignBlock(0, 2, model.getB());
    Matrix<N3, N3> exponential = continuous.times(seconds).exp();
    Matrix<N2, N2> a = new Matrix<>(Nat.N2(), Nat.N2());
    a.extractFrom(0, 0, exponential);
    Matrix<N2, N1> b = new Matrix<>(Nat.N2(), Nat.N1());
    b.extractFrom(0, 2, exponential);
    return new Discretized(seconds, a, b);
  }

  /** How far the wheel has turned since the start, in radians. */
  double rotationRadians() {
    return output.get(0, 0);
  }

  /** How fast the wheel turns, in radians a second. */
  double speedRadPerSec() {
    return output.get(1, 0);
  }

  /** The voltage held across the motor through the last step. */
  double volts() {
    return heldVolts.get(0, 0);
  }

  /**
   * The current through the motor, as WPILib's {@code DCMotorSim} gives it: the motor's current at
   * its speed and voltage, given the sign of the voltage. So it reads no current at all at 0 V,
   * where a motor braking a spinning wheel truly carries current in its windings.
   */
  double currentAmps() {
    double volts = heldVolts.get(0, 0);
    double motorSpeedRadPerSec = state.get(1, 0) * gearingFromModel;
    return motor.getCurrent(motorSpeedRadPerSec, volts) * Math.signum(volts);
  }

  /** Whether the current limit held the voltage back in the last step. */
  boolean currentLimited() {
    return currentLimited;
  }
}

package com.michaelgrundvig.frc.simulation;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.Map;
import java.util.Optional;
import org.wpilib.math.system.DCMotor;

/**
 * One of the robot's joints as the simulator moves it, in closed form, from the robot's definition
 * alone: its motors through its transmission, with its losses, their current held to a limit
 * ({@link CurrentLimitedMotor}); the link and everything it carries, and its motors' rotors, as its
 * inertia ({@code RobotMath.inertia}), the rotors alone for links without mass; gravity ({@link
 * #gravityTorque}); and its hard stops, which stop it dead, its motors stalled against one they
 * drive it into: they draw what their voltage drives through them standing still, held to their
 * limit. Its motor controller isn't here: what the controller puts across the motors is its maker's
 * to work out, each millisecond, as the controller would, and hand to {@link #run}. The battery
 * holds 12 V.
 *
 * <p>Gravity is what the kG tuned in the joint's block holds, through its motors, since that's what
 * a team measures: its plan's gravity, one cosine about where its weight pulls hardest for an arm,
 * the same all along a slide ({@link ControllerPlan#gravity}). Where its block tunes none, gravity
 * is its links' weight ({@code RobotMath.gravityTorque}), with the other joints where they are, and
 * none for links without mass. Gravity is the definition's block's, whatever control its motor
 * controller runs. An arm's tuned gravity doesn't follow a parent that tilts it, as one cosine
 * doesn't.
 *
 * <p>Pure and deterministic: no clock, nothing random; it moves only as it's stepped. Joints move
 * independently: one joint's motion doesn't push on another's. Friction isn't modeled: it isn't
 * known until measured.
 */
public final class JointPlant {
  private final Joint joint;
  private final RobotMath math;

  /** Its motors driving it together: all of them, or those still driven ({@link #drivenBy}). */
  private CurrentLimitedMotor motor;

  /** How many motors turn it: its currents are each one's, as each motor's controller reads it. */
  private final int count;

  /** How many of them drive it now: all, until one is left neutral ({@link #drivenBy}). */
  private int driving;

  /**
   * One of its motors, through its transmission, its losses taken off; and its gearing, inertia.
   */
  private final DCMotor each;

  private final double gearing;
  private final double inertia;

  /**
   * Newton meters at the joint for each volt across its motors held still (newtons, for a slide):
   * their stall torque per volt, all of them, through its gearing, less its losses.
   */
  private final double torquePerVolt;

  /**
   * Gravity as the kG tuned in the joint's block holds it; empty where none is, and its links' mass
   * does.
   */
  private final Optional<ControllerPlan.Gravity> weight;

  private final double lower;
  private final double upper;

  /** Where it's jammed, a stop on both sides; NaN while it moves freely. */
  private double jammedAt = Double.NaN;

  private JointPlant(RobotMath math, Joint joint) {
    this.joint = joint;
    this.math = math;
    Drive drive = math.drivesOf(joint).get(0);
    Actuator.Motors motors = (Actuator.Motors) drive.actuator();
    DCMotor model = motors.model(drive.transmission().efficiency());
    gearing = Math.abs(drive.transmission().shaftRadiansPer(joint));
    inertia = math.inertia(joint, Map.of());
    motor = new CurrentLimitedMotor(model, inertia, gearing);
    torquePerVolt = gearing * model.stallTorque / model.nominalVoltage;
    count = motors.count();
    driving = count;
    each =
        new DCMotor(
            model.nominalVoltage,
            model.stallTorque / count,
            model.stallCurrent / count,
            model.freeCurrent / count,
            model.freeSpeed,
            1);
    // The definition's own joint, the same but perhaps for how it's driven: its block's kG.
    Joint defined = math.joints().get(math.joints().indexOf(joint));
    weight =
        math.gravityActsOn(joint)
                && defined.driven().control().flatMap(control -> control.gains().kG()).isPresent()
            ? Optional.of(ControllerPlan.of(math, defined).gravity())
            : Optional.empty();
    lower = joint.range().map(Range::lower).orElse(Double.NEGATIVE_INFINITY);
    upper = joint.range().map(Range::upper).orElse(Double.POSITIVE_INFINITY);
    motor.place(joint.restPosition(), 0);
  }

  /**
   * A driven joint of the robot's definition, still, where its definition rests it.
   *
   * @param math what the robot's definition derives: the joint's motors, inertia and gravity
   * @param joint one of its joints, moved by one drive of motors; as its block says, or driven
   *     otherwise, which changes nothing here
   */
  public static JointPlant of(RobotMath math, Joint joint) {
    return new JointPlant(math, joint);
  }

  /** The joint. */
  public Joint joint() {
    return joint;
  }

  /** How many motors turn it together. */
  public int motors() {
    return count;
  }

  /**
   * Drives it with only so many of its motors from now on, the rest left neutral, as a follower off
   * the CAN bus is: coasting with it, adding no torque and drawing nothing from the battery. The
   * drag a motor neutral in brake puts on the joint is left out. Gravity on it is unchanged.
   *
   * @param motors how many still drive it, from 1 to all of them
   * @throws IllegalArgumentException for none, or more than it has
   */
  public void drivenBy(int motors) {
    if (motors < 1 || motors > count) {
      throw new IllegalArgumentException(
          joint.label() + " has " + count + " motors to drive it, not " + motors);
    }
    if (motors == driving) {
      return;
    }
    double position = motor.positionRadians();
    double velocity = motor.velocityRadPerSec();
    motor =
        new CurrentLimitedMotor(
            new DCMotor(
                each.nominalVoltage,
                each.stallTorque,
                each.stallCurrent,
                each.freeCurrent,
                each.freeSpeed,
                motors),
            inertia,
            gearing);
    motor.place(position, velocity);
    driving = motors;
  }

  /**
   * Gravity's torque on the joint, positive turning it forward, with the joints where they stand:
   * newton meters, or newtons for a slide. What its block's tuned kG holds, through its motors;
   * where none is tuned, its links' weight.
   */
  public double gravityTorque(Map<Joint, Double> positions) {
    if (weight.isEmpty()) {
      return -math.gravityTorque(joint, positions);
    }
    return -weight.get().volts(position()) * torquePerVolt;
  }

  /**
   * Runs its motors for a time at a voltage, their controller holding the current through each to a
   * limit, gravity's torque steady through it; then its hard stops.
   *
   * @param volts what the controller puts across the motors, up to the battery's
   * @param limitAmps the most current it lets through each motor, either way: its stator limit
   * @param gravityTorque gravity's torque on the joint, positive turning it forward
   */
  public void run(double volts, double limitAmps, double gravityTorque, double seconds) {
    double limit = limitAmps * driving;
    motor.run(volts, limit, gravityTorque, seconds);
    if (stops()) {
      // Driven into a stop, its motors are held still: stalled, at their controller's voltage.
      motor.stall(volts, limit);
    }
  }

  /**
   * Lets it turn freely for a time, no current through its motors, as a motor controller in coast
   * leaves it: only gravity moves it, until a hard stop stops it.
   */
  public void coast(double gravityTorque, double seconds) {
    motor.coast(gravityTorque, seconds);
    stops();
  }

  /**
   * Holds it at a hard stop it went past, still, or moving away from it: whether the stop holds it
   * still.
   */
  private boolean stops() {
    double velocity = motor.velocityRadPerSec();
    double below = Double.isNaN(jammedAt) ? lower : jammedAt;
    double above = Double.isNaN(jammedAt) ? upper : jammedAt;
    if (motor.positionRadians() < below) {
      motor.place(below, Math.max(0, velocity));
      return velocity <= 0;
    }
    if (motor.positionRadians() > above) {
      motor.place(above, Math.min(0, velocity));
      return velocity >= 0;
    }
    return false;
  }

  /**
   * Jams it where it is, or frees it: a stop on both sides, as a game piece wedged in it or a bent
   * part leaves it, which its motors stall against as against a hard stop.
   */
  public void jam(boolean jammed) {
    jammedAt = jammed ? motor.positionRadians() : Double.NaN;
  }

  /** Puts it somewhere, still: where a test starts it. */
  public void place(double position) {
    motor.place(position, 0);
  }

  /** Where it is, in its units: radians, or meters for a slide. */
  public double position() {
    return motor.positionRadians();
  }

  /** How fast it moves, in its units a second. */
  public double velocity() {
    return motor.velocityRadPerSec();
  }

  /** The voltage across its motors now: less than its controller asks for, at the limit. */
  public double appliedVolts() {
    return motor.appliedVolts();
  }

  /**
   * The current through each of its motors that drive it, either way: what its stator limit holds.
   */
  public double statorAmps() {
    return Math.abs(motor.currentAmps()) / driving;
  }

  /**
   * The current each of its motors that drive it draws from the battery, in amps: what its
   * controller reads, and what its PDH channel does. Each draws alike, as they share the joint's
   * load; one left neutral draws nothing.
   */
  public double supplyAmps() {
    return Math.abs(motor.currentAmps() * motor.appliedVolts()) / Motor.NOMINAL_VOLTS / driving;
  }
}

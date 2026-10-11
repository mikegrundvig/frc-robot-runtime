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
import java.util.OptionalDouble;
import org.wpilib.math.system.DCMotor;

/**
 * One of the robot's joints as the simulator moves it, in closed form, from the robot's definition
 * alone. It is made of:
 *
 * <ul>
 *   <li><b>Its motors</b>, through its transmission with its losses, their current held to a limit
 *       ({@link CurrentLimitedMotor}).
 *   <li><b>Its inertia</b>: the link, everything the link carries, and the motors' rotors ({@code
 *       RobotMath.inertia}). For links without mass, it is the rotors alone.
 *   <li><b>Gravity</b> ({@link #gravityTorque}).
 *   <li><b>Its hard stops</b>, which stop it dead. Motors that drive the joint into a stop are
 *       stalled against it: they draw what their voltage drives through them standing still, held
 *       to their limit.
 * </ul>
 *
 * <p>Its motor controller is not here. What the controller puts across the motors is for whoever
 * runs this plant to work out, each millisecond, as the controller would, and hand to {@link #run}.
 * The battery holds 12 V.
 *
 * <p><b>Gravity</b> is what the kG tuned in the joint's block holds, through its motors, since that
 * is what a team measures: for an arm, one cosine about the angle where its weight pulls hardest,
 * and for a slide, the same all along it ({@link ControllerPlan#gravity}). Where the joint's block
 * tunes no kG, gravity is its links' weight ({@code RobotMath.gravityTorque}), with the other
 * joints where they are, and none for links without mass. Gravity belongs to the robot's
 * definition, whatever control the motor controller runs. An arm's tuned gravity does not follow a
 * parent joint that tilts it, since one cosine cannot.
 *
 * <p><b>What is left out.</b> Joints move independently: one joint's motion does not push on
 * another's. Friction is not modeled, because it is not known until measured.
 *
 * <p>Pure and deterministic: no clock and nothing random. It moves only as it is stepped.
 */
public final class JointPlant {
  private final Joint joint;
  private final RobotMath math;

  /** The motors that drive it, together: all of them, or those still driven ({@link #drivenBy}). */
  private CurrentLimitedMotor motor;

  /** How many motors turn it. Its currents are each one's, as each motor's controller reads it. */
  private final int motorCount;

  /** How many of them drive it now: all of them, until one is left neutral ({@link #drivenBy}). */
  private int drivingCount;

  /** One of its motors alone, through its transmission, with its losses taken off. */
  private final DCMotor oneMotor;

  private final double gearing;
  private final double inertia;

  /**
   * Newton meters at the joint for each volt across its motors while they are held still (newtons,
   * for a slide): the stall torque per volt of all of them together, through its gearing, less its
   * losses.
   */
  private final double torquePerVolt;

  /**
   * Gravity as the kG tuned in the joint's block holds it. Empty where the block tunes none, and
   * the links' mass gives gravity instead.
   */
  private final Optional<ControllerPlan.Gravity> tunedGravity;

  /** Its hard stops, in its units. A joint that turns without end has them at infinity. */
  private final double lowerStop;

  private final double upperStop;

  /** Where it is jammed, which is a stop on both sides. Empty while it moves freely. */
  private OptionalDouble jammedAt = OptionalDouble.empty();

  private JointPlant(RobotMath math, Joint joint) {
    this.joint = joint;
    this.math = math;
    Drive drive = math.drivesOf(joint).get(0);
    Actuator.Motors motors = (Actuator.Motors) drive.actuator();
    DCMotor allMotors = motors.model(drive.transmission().efficiency());
    gearing = Math.abs(drive.transmission().shaftRadiansPer(joint));
    inertia = math.inertia(joint, Map.of());
    motor = new CurrentLimitedMotor(allMotors, inertia, gearing);
    torquePerVolt = gearing * allMotors.stallTorque / allMotors.nominalVoltage;
    motorCount = motors.count();
    drivingCount = motorCount;
    oneMotor =
        new DCMotor(
            allMotors.nominalVoltage,
            allMotors.stallTorque / motorCount,
            allMotors.stallCurrent / motorCount,
            allMotors.freeCurrent / motorCount,
            allMotors.freeSpeed,
            1);
    tunedGravity = tunedGravity(math, joint);
    lowerStop = joint.range().map(Range::lower).orElse(Double.NEGATIVE_INFINITY);
    upperStop = joint.range().map(Range::upper).orElse(Double.POSITIVE_INFINITY);
    motor.place(joint.restPosition(), 0);
  }

  /**
   * Gravity as the kG tuned in the joint's block holds it, if gravity acts on the joint and its
   * block tunes a kG. The block read is the definition's own: the joint handed in may be the same
   * joint driven some other way, which changes nothing about its weight.
   */
  private static Optional<ControllerPlan.Gravity> tunedGravity(RobotMath math, Joint joint) {
    Joint asDefined = math.joints().get(math.joints().indexOf(joint));
    if (!math.gravityActsOn(joint)) {
      return Optional.empty();
    }
    boolean tunesKg =
        asDefined.driven().control().flatMap(control -> control.gains().kG()).isPresent();
    if (!tunesKg) {
      return Optional.empty();
    }
    return Optional.of(ControllerPlan.of(math, asDefined).gravity());
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
    return motorCount;
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
    if (motors < 1 || motors > motorCount) {
      throw new IllegalArgumentException(
          joint.label() + " has " + motorCount + " motors to drive it, not " + motors);
    }
    if (motors == drivingCount) {
      return;
    }
    // A new motor model for the motors that still drive it, carrying on from where the joint is.
    double position = motor.positionRadians();
    double velocity = motor.velocityRadPerSec();
    DCMotor stillDriving =
        new DCMotor(
            oneMotor.nominalVoltage,
            oneMotor.stallTorque,
            oneMotor.stallCurrent,
            oneMotor.freeCurrent,
            oneMotor.freeSpeed,
            motors);
    motor = new CurrentLimitedMotor(stillDriving, inertia, gearing);
    motor.place(position, velocity);
    drivingCount = motors;
  }

  /**
   * Gravity's torque on the joint, positive turning it forward, with the joints where they stand:
   * newton meters, or newtons for a slide. What its block's tuned kG holds, through its motors;
   * where none is tuned, its links' weight.
   */
  public double gravityTorque(Map<Joint, Double> positions) {
    if (tunedGravity.isEmpty()) {
      return -math.gravityTorque(joint, positions);
    }
    // A tuned kG is the volts that hold the joint against gravity, so gravity's torque is the
    // torque those volts give through the stalled motors, the other way.
    double holdingVolts = tunedGravity.get().volts(position());
    return -holdingVolts * torquePerVolt;
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
    // The motors that drive it are modeled as one motor, which carries all of their currents.
    double limitOfAllAmps = limitAmps * drivingCount;
    motor.run(volts, limitOfAllAmps, gravityTorque, seconds);
    boolean drivenIntoAStop = holdWithinItsStops();
    if (drivenIntoAStop) {
      // Its motors are held still: stalled, at their controller's voltage.
      motor.stall(volts, limitOfAllAmps);
    }
  }

  /**
   * Lets it turn freely for a time, no current through its motors, as a motor controller in coast
   * leaves it: only gravity moves it, until a hard stop stops it.
   */
  public void coast(double gravityTorque, double seconds) {
    motor.coast(gravityTorque, seconds);
    holdWithinItsStops();
  }

  /**
   * Puts the joint back at a stop it went past in the last step. A joint moving into the stop is
   * stopped dead. One already moving away from it keeps its speed.
   *
   * @return whether a stop holds it still: it was past the stop and not moving away from it
   */
  private boolean holdWithinItsStops() {
    double velocity = motor.velocityRadPerSec();
    // Jammed, it has a stop on both sides, right where it is.
    double lowest = jammedAt.orElse(lowerStop);
    double highest = jammedAt.orElse(upperStop);
    if (motor.positionRadians() < lowest) {
      double awayFromTheStop = Math.max(0, velocity);
      motor.place(lowest, awayFromTheStop);
      return velocity <= 0;
    }
    if (motor.positionRadians() > highest) {
      double awayFromTheStop = Math.min(0, velocity);
      motor.place(highest, awayFromTheStop);
      return velocity >= 0;
    }
    return false;
  }

  /**
   * Jams it where it is, or frees it: a stop on both sides, as a game piece wedged in it or a bent
   * part leaves it, which its motors stall against as against a hard stop.
   */
  public void jam(boolean jammed) {
    if (jammed) {
      jammedAt = OptionalDouble.of(motor.positionRadians());
    } else {
      jammedAt = OptionalDouble.empty();
    }
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
    return Math.abs(motor.currentAmps()) / drivingCount;
  }

  /**
   * The current each of its motors that drive it draws from the battery, in amps: what its
   * controller reads, and what its PDH channel does. Each draws alike, as they share the joint's
   * load; one left neutral draws nothing.
   */
  public double supplyAmps() {
    // Power in equals power out: the battery's 12 V times the supply current is the voltage
    // across the motors times the current through them.
    double wattsOfAll = Math.abs(motor.currentAmps() * motor.appliedVolts());
    return wattsOfAll / Motor.NOMINAL_VOLTS / drivingCount;
  }
}

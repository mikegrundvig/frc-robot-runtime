package com.michaelgrundvig.frc.simulation;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Twist2d;
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
 * Today's physics: the robot moves as its wheels roll, kinematically, as the best fit of their
 * travel, and each wheel carries an equal share of the robot's mass as inertia on its own drive
 * motor. It's the template's physics behind {@link Physics}: the simulator tests the robot's code,
 * not its traction or power, and the truth suite measures what that leaves out.
 *
 * <p>Each step is one sub-step. In it:
 *
 * <ul>
 *   <li>Each drive motor turns its wheel's share of the robot's configured mass ({@code
 *       Body.mass}), (m / N) r² at the configured radius, plus the wheel's own moving parts ({@code
 *       DrivetrainGeometry.ModuleType.driveInertia}), at the voltage its controller asks for, held
 *       to its current limit at the wheel's speed as the step starts and held through the step, as
 *       WPILib's {@code DCMotorSim} holds a voltage. So the robot speeds up as F = m a, short of it
 *       by the current falling within each step at the controller's limit.
 *   <li>The robot moves by the one rigid motion that best fits every wheel's travel (each wheel's
 *       turn at its true radius, at its true angle), scaled by the share of grip ({@link
 *       Disturbance.Grip}) and the slope it rolls along, and changes its velocity no faster than
 *       the tread's grip allows, μ g. Past that, every wheel skids alike.
 *   <li>It sits on the plane that best fits the ground under its four wheels, which gives its
 *       height and attitude, unless tilted by hand.
 * </ul>
 *
 * <p>What that leaves out, the truth suite keeps as known gaps (docs/simulator.md): the robot's yaw
 * inertia (it spins as if its mass sat at its wheels), weight transfer and each wheel's own load
 * and slip, wheels that disagree scrubbing (the fit just splits the difference), a wheel leaving
 * the ground, and gravity along a slope.
 */
public final class KinematicPhysics implements Physics {
  /**
   * How quickly a push gets the robot sliding, and lets it stop, in m/s²: about a third of g, as
   * one robot driving into another starts it moving.
   */
  public static final double PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;

  private final Wheel[] wheels;
  private final WheelDrive[] drives;
  private final Translation2d[] moduleLocations;
  private final RigidFit fit;
  private final double[] configuredRadiusMeters;

  /** Each wheel's load: an equal share of the robot's weight. */
  private final double loadNewtons;

  /** The most acceleration the floor's grip allows the robot: its wheels' friction times g. */
  private final double gripLimit;

  private final double[] planeFit = new double[9];

  /** The last step's rigid motion, in the robot's frame: forward, left, and turned. */
  private final double[] motion = new double[3];

  private final Chassis chassis = new Chassis();

  // The wheels' travel at the last step's end: each one's distance rolled and the angle it pointed.
  private double[] lastDistances;
  private double[] lastAngles;

  private Pose2d truePose = Pose2d.ZERO;
  private double yawTurnedRadians;
  private double yawRateRadiansPerSecond;
  private double elapsedSeconds;

  private double grip = 1;
  private double pitchRadians;
  private double rollRadians;

  // The ground under the robot: the terrain, and the plane through its wheels' contact points.
  private Terrain terrain = Terrain.FLAT;
  private boolean tiltedByHand;
  private double heightMeters;
  private double riseForward;
  private double riseLeft;

  /** The robot's velocity on the field over the last step, for what its IMUs feel. */
  private Translation2d lastVelocity = Translation2d.ZERO;

  /** A push the wheels don't see, in the field's frame: the velocity it's headed for, and now. */
  private Translation2d pushTarget = Translation2d.ZERO;

  private Translation2d pushVelocity = Translation2d.ZERO;

  /**
   * The robot's velocity on the field: what its wheels ask of it, followed no faster than the
   * floor's grip allows.
   */
  private Translation2d chassisVelocity = Translation2d.ZERO;

  /**
   * On flat ground until given its terrain ({@link Disturbance.Ground}), standing at the field's
   * origin.
   *
   * @param body the robot's mass, which the wheels carry
   * @param drivetrain the drivetrain as defined: where its wheels are, and each module's motors,
   *     gearing, and radius
   * @param wheelRadiusScale each wheel's true radius over the configured one, as built
   */
  public KinematicPhysics(Body body, DrivetrainGeometry drivetrain, List<Double> wheelRadiusScale) {
    moduleLocations = drivetrain.moduleTranslations();
    fit = new RigidFit(moduleLocations);
    planeFit(moduleLocations, planeFit);
    int count = moduleLocations.length;
    configuredRadiusMeters = new double[count];
    double carriedKilograms = body.mass().in(Kilograms) / count;
    wheels = new Wheel[count];
    drives = new WheelDrive[count];
    for (int i = 0; i < count; i++) {
      DrivetrainGeometry.ModuleType module = drivetrain.module(i);
      configuredRadiusMeters[i] = module.wheelRadius().in(Meters);
      double inertia =
          driveInertiaKgM2(
              carriedKilograms,
              configuredRadiusMeters[i],
              module.driveInertiaOrDefault().in(KilogramSquareMeters));
      wheels[i] =
          new Wheel(
              i,
              module.driveMotors().model(module.driveGearing().efficiency()),
              inertia,
              module.driveGearing().ratio(),
              configuredRadiusMeters[i] * wheelRadiusScale.get(i));
      drives[i] = new WheelDrive();
    }
    loadNewtons = body.mass().in(Kilograms) * STANDARD_GRAVITY / count;
    gripLimit = drivetrain.wheelCoefficientOfFriction() * STANDARD_GRAVITY;
    lastDistances = new double[count];
    lastAngles = new double[count];
    rollAlong();
  }

  /**
   * A drive's load, as inertia at the wheel, in kg m²: the mass the wheel moves, at the wheel's
   * radius, (m / N) r², plus the drive's own moving parts. A force F at the tread then accelerates
   * the wheel's share of the robot as F = (m / N) a, so the wheels together move the robot as F = m
   * a. A worn tread's true radius, a few percent short, is left out of it.
   *
   * @param carriedKilograms the share of the robot's mass the wheel moves
   * @param wheelRadiusMeters the wheel's configured radius
   * @param ownKgM2 the drive's own moving parts (wheel, gears, rotor), as inertia at the wheel
   */
  static double driveInertiaKgM2(
      double carriedKilograms, double wheelRadiusMeters, double ownKgM2) {
    return carriedKilograms * wheelRadiusMeters * wheelRadiusMeters + ownKgM2;
  }

  @Override
  public void step(double seconds, Actuators actuators) {
    if (!(seconds > 0)) {
      throw new IllegalArgumentException("a step takes time: " + seconds);
    }
    actuators.actuate(seconds, this, drives);
    for (int i = 0; i < wheels.length; i++) {
      wheels[i].turn(drives[i], seconds);
    }
    double[] distances = distances();
    double[] angles = angles();
    fit.motion(lastDistances, lastAngles, distances, angles, motion);
    double dx = motion[0];
    double dy = motion[1];
    double dtheta = motion[2];
    Translation2d before = truePose.getTranslation();
    // Only the gripping share of the wheels' rolling moves the robot, and rolling up or down a
    // slope moves it less across the field.
    double travel = Math.hypot(dx, dy);
    double rise = travel > 0 ? (riseForward * dx + riseLeft * dy) / travel : 0;
    double level = grip / Math.sqrt(1 + rise * rise);
    Pose2d asked = truePose.plus(new Twist2d(level * dx, level * dy, grip * dtheta).exp());
    // The robot goes where its wheels take it, as fast as the floor's grip lets it change course:
    // past that, they skid.
    chassisVelocity =
        toward(
            chassisVelocity,
            asked.getTranslation().minus(before).div(seconds),
            gripLimit * seconds);
    pushVelocity =
        toward(pushVelocity, pushTarget, PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED * seconds);
    truePose =
        new Pose2d(
            before.plus(chassisVelocity.times(seconds)).plus(pushVelocity.times(seconds)),
            asked.getRotation());
    settle();
    yawTurnedRadians += grip * dtheta;
    yawRateRadiansPerSecond = grip * dtheta / seconds;
    lastVelocity = truePose.getTranslation().minus(before).div(seconds);
    lastDistances = distances;
    lastAngles = angles;
    elapsedSeconds += seconds;
  }

  @Override
  public ChassisTruth chassis() {
    return chassis;
  }

  @Override
  public WheelTruth wheel(int index) {
    return wheels[index];
  }

  @Override
  public int wheels() {
    return wheels.length;
  }

  @Override
  public void apply(Disturbance disturbance) {
    switch (disturbance) {
      case Disturbance.Placed placed -> {
        Pose2d pose = placed.pose();
        yawTurnedRadians += pose.getRotation().minus(truePose.getRotation()).getRadians();
        truePose = pose;
        settle();
      }
      case Disturbance.Pushed pushed -> pushTarget = pushed.fieldVelocity();
      case Disturbance.TiltedByHand tilt -> {
        tiltedByHand = true;
        pitchRadians = tilt.pitchRadians();
        rollRadians = tilt.rollRadians();
      }
      case Disturbance.Ground ground -> {
        terrain = ground.terrain();
        tiltedByHand = false;
        settle();
      }
      case Disturbance.Grip slick -> grip = slick.share();
      case Disturbance.TreadWorn worn -> {
        wheels[worn.wheel()].radiusMeters = configuredRadiusMeters[worn.wheel()] * worn.scale();
        // The wheels' travel so far was at the old radius: start counting again from here.
        rollAlong();
      }
    }
  }

  /** Takes the wheels' travel as it stands for where the next step's starts. */
  private void rollAlong() {
    lastDistances = distances();
    lastAngles = angles();
  }

  /** How far each wheel has rolled the robot along, at its true radius. */
  private double[] distances() {
    double[] distances = new double[wheels.length];
    for (int i = 0; i < wheels.length; i++) {
      distances[i] = wheels[i].rotationRadians() * wheels[i].radiusMeters;
    }
    return distances;
  }

  /** Which way each wheel points. */
  private double[] angles() {
    double[] angles = new double[wheels.length];
    for (int i = 0; i < wheels.length; i++) {
      angles[i] = wheels[i].steerRadians;
    }
    return angles;
  }

  /** A velocity moved toward another by at most so much. */
  private static Translation2d toward(Translation2d from, Translation2d to, double most) {
    Translation2d change = to.minus(from);
    double size = change.getNorm();
    return size <= most ? to : from.plus(change.times(most / size));
  }

  /**
   * Sets the robot on the ground where it is: the plane through the ground under its wheels gives
   * its height and, unless tilted by hand, its attitude.
   */
  private void settle() {
    double sum = 0;
    double sumX = 0;
    double sumY = 0;
    for (Translation2d wheel : moduleLocations) {
      double height =
          terrain.heightAt(truePose.getTranslation().plus(wheel.rotateBy(truePose.getRotation())));
      sum += height;
      sumX += wheel.getX() * height;
      sumY += wheel.getY() * height;
    }
    // Least squares: height = a + (rise forward) x + (rise left) y, in the robot's frame.
    heightMeters = planeFit[0] * sum + planeFit[1] * sumX + planeFit[2] * sumY;
    riseForward = planeFit[3] * sum + planeFit[4] * sumX + planeFit[5] * sumY;
    riseLeft = planeFit[6] * sum + planeFit[7] * sumX + planeFit[8] * sumY;
    if (!tiltedByHand) {
      // WPILib's convention: nose up is a negative pitch, the left side up a positive roll.
      pitchRadians = -Math.atan(riseForward);
      rollRadians = Math.atan(riseLeft * Math.cos(pitchRadians));
    }
  }

  /** The inverse of the plane fit's normal equations for these wheels, row by row. */
  private static void planeFit(Translation2d[] wheels, double[] inverse) {
    double n = wheels.length;
    double sx = 0;
    double sy = 0;
    double sxx = 0;
    double sxy = 0;
    double syy = 0;
    for (Translation2d wheel : wheels) {
      sx += wheel.getX();
      sy += wheel.getY();
      sxx += wheel.getX() * wheel.getX();
      sxy += wheel.getX() * wheel.getY();
      syy += wheel.getY() * wheel.getY();
    }
    double[] m = {n, sx, sy, sx, sxx, sxy, sy, sxy, syy};
    double det =
        m[0] * (m[4] * m[8] - m[5] * m[7])
            - m[1] * (m[3] * m[8] - m[5] * m[6])
            + m[2] * (m[3] * m[7] - m[4] * m[6]);
    inverse[0] = (m[4] * m[8] - m[5] * m[7]) / det;
    inverse[1] = (m[2] * m[7] - m[1] * m[8]) / det;
    inverse[2] = (m[1] * m[5] - m[2] * m[4]) / det;
    inverse[3] = (m[5] * m[6] - m[3] * m[8]) / det;
    inverse[4] = (m[0] * m[8] - m[2] * m[6]) / det;
    inverse[5] = (m[2] * m[3] - m[0] * m[5]) / det;
    inverse[6] = (m[3] * m[7] - m[4] * m[6]) / det;
    inverse[7] = (m[1] * m[6] - m[0] * m[7]) / det;
    inverse[8] = (m[0] * m[4] - m[1] * m[3]) / det;
  }

  /** The chassis's truth: a view of this world. */
  private final class Chassis implements ChassisTruth {
    @Override
    public Pose2d pose() {
      return truePose;
    }

    @Override
    public Pose3d pose3d() {
      return new Pose3d(
          truePose.getX(),
          truePose.getY(),
          heightMeters,
          new Rotation3d(rollRadians, pitchRadians, truePose.getRotation().getRadians()));
    }

    @Override
    public double pitchRadians() {
      return pitchRadians;
    }

    @Override
    public double rollRadians() {
      return rollRadians;
    }

    @Override
    public Translation2d velocity() {
      return lastVelocity;
    }

    @Override
    public double yawRateRadiansPerSecond() {
      return yawRateRadiansPerSecond;
    }

    @Override
    public double yawTurnedRadians() {
      return yawTurnedRadians;
    }

    @Override
    public double seconds() {
      return elapsedSeconds;
    }

    @Override
    public Translation2d pointVelocity(Translation2d place) {
      Translation2d arm = place.rotateBy(truePose.getRotation());
      return lastVelocity.plus(
          new Translation2d(
              -yawRateRadiansPerSecond * arm.getY(), yawRateRadiansPerSecond * arm.getX()));
    }
  }

  /**
   * One wheel: its drive motor turning the wheel's share of the robot, exactly as WPILib's {@code
   * DCMotorSim} turns a load (its state-space model, discretized for the step, at the voltage held
   * through it), and the angle its steering was turned to.
   */
  private final class Wheel implements WheelTruth {
    private final int index;
    private final DCMotor motor;
    private final LinearSystem<N2, N1, N2> plant;

    /** The gearing as configured: motor turns per turn of the wheel. */
    private final double ratio;

    /** The gearing as {@code DCMotorSim} works it back out of its model. */
    private final double gearing;

    private Matrix<N2, N1> state = new Matrix<>(Nat.N2(), Nat.N1());
    private Matrix<N1, N1> input = new Matrix<>(Nat.N1(), Nat.N1());
    private Matrix<N2, N1> output = new Matrix<>(Nat.N2(), Nat.N1());
    private double radiusMeters;
    private double steerRadians;
    private boolean currentLimited;

    /**
     * The model discretized for a step this long: worked out once for each length of step, since
     * the matrix exponential it takes is native code, and today's steps are all one length.
     */
    private double discretizedSeconds = Double.NaN;

    private Matrix<N2, N2> discreteA = new Matrix<>(Nat.N2(), Nat.N2());
    private Matrix<N2, N1> discreteB = new Matrix<>(Nat.N2(), Nat.N1());

    Wheel(int index, DCMotor motor, double inertiaKgM2, double ratio, double radiusMeters) {
      this.index = index;
      this.motor = motor;
      this.ratio = ratio;
      // WPILib 2027 names this model for an arm; without gravity it's a motor turning a load.
      plant = Models.singleJointedArmFromPhysicalConstants(motor, inertiaKgM2, ratio);
      gearing = -motor.Kv * plant.getA(1, 1) / plant.getB(1, 0);
      this.radiusMeters = radiusMeters;
    }

    /**
     * Turns the wheel through a step, its motor at what its controller asks, held to the
     * controller's current limit as a TalonFX holds it at the step's start: no more than the
     * limit's worth of voltage past the motor's back EMF, and within the battery. Held through the
     * step, the current then falls as the wheel speeds up, short of the limit (the truth suite's F
     * = m a gap). The velocity loop's gain goes unused: the voltage is held.
     */
    void turn(WheelDrive drive, double seconds) {
      double backEmf = spinRadPerSec() * ratio / motor.Kv;
      double headroom = drive.currentLimitAmps() * motor.R;
      double limited = Math.clamp(drive.volts(), backEmf - headroom, backEmf + headroom);
      currentLimited = limited != drive.volts();
      input = VecBuilder.fill(limited * Math.min(1.0, drive.batteryVolts() / Math.abs(limited)));
      steerRadians = drive.steerRadians();
      if (seconds != discretizedSeconds) {
        discretize(seconds);
      }
      // As LinearSystem.calculateX works it, to the bit, but discretized once.
      state = discreteA.times(state).plus(discreteB.times(input));
      output = plant.calculateY(state, input);
    }

    /**
     * Discretizes the model for a step this long, exactly as WPILib's {@code
     * Discretization.discretizeAB} does (which hands the result back in a pair outside WPILib's
     * math): the exponential of [A B; 0 0] times the step, its blocks the discrete A and B.
     */
    private void discretize(double seconds) {
      Matrix<N3, N3> continuous = new Matrix<>(Nat.N3(), Nat.N3());
      continuous.assignBlock(0, 0, plant.getA());
      continuous.assignBlock(0, 2, plant.getB());
      Matrix<N3, N3> phi = continuous.times(seconds).exp();
      discreteA = new Matrix<>(Nat.N2(), Nat.N2());
      discreteA.extractFrom(0, 0, phi);
      discreteB = new Matrix<>(Nat.N2(), Nat.N1());
      discreteB.extractFrom(0, 2, phi);
      discretizedSeconds = seconds;
    }

    @Override
    public double rotationRadians() {
      return output.get(0, 0);
    }

    @Override
    public double spinRadPerSec() {
      return output.get(1, 0);
    }

    @Override
    public double steerRadians() {
      return steerRadians;
    }

    @Override
    public double radiusMeters() {
      return radiusMeters;
    }

    /**
     * As WPILib's {@code DCMotorSim} gives it: the motor's current, signed by its voltage, so none
     * at all at 0 V, where a motor braking a spinning wheel truly carries current in its windings.
     */
    @Override
    public double driveCurrentAmps() {
      double volts = input.get(0, 0);
      return motor.getCurrent(state.get(1, 0) * gearing, volts) * Math.signum(volts);
    }

    /** The voltage held through the last step. */
    @Override
    public double driveVolts() {
      return input.get(0, 0);
    }

    @Override
    public boolean driveCurrentLimited() {
      return currentLimited;
    }

    /** No scrub: the steering turns freely on the carpet. */
    @Override
    public double steerLoadNewtonMeters() {
      return 0;
    }

    /** An equal share of the robot's weight, however hard it speeds up or tilts. */
    @Override
    public double loadNewtons() {
      return loadNewtons;
    }

    /** Always: the robot stands on the plane through its four wheels' ground. */
    @Override
    public boolean inContact() {
      return true;
    }

    @Override
    public Translation3d contactPoint() {
      Translation2d location = moduleLocations[index];
      Translation2d onTheField =
          truePose.getTranslation().plus(location.rotateBy(truePose.getRotation()));
      double height = heightMeters + riseForward * location.getX() + riseLeft * location.getY();
      return new Translation3d(onTheField.getX(), onTheField.getY(), height);
    }
  }
}

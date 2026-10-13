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
import org.wpilib.math.system.DCMotor;

/**
 * Today's physics. The robot moves as its wheels roll: kinematically, by the one rigid motion that
 * best fits how far each wheel rolled and which way it pointed. Each wheel's drive motor turns an
 * equal share of the robot's mass as inertia. It is the template's physics behind {@link Physics}.
 * The simulator is there to test the robot's code, not its traction or its power, and the truth
 * suite measures what this leaves out.
 *
 * <p>Each call to {@link #step} is one sub-step, in five parts:
 *
 * <ol>
 *   <li><b>The motors turn the wheels</b> ({@link WheelMotor}). Each drive motor turns its wheel's
 *       share of the robot's mass, (m / N)·r² at the configured radius, plus the wheel's own moving
 *       parts, at the voltage its controller asks for, held to its current limit as the step
 *       starts.
 *   <li><b>The wheels' rolling moves the robot</b> ({@link RigidFit}). The robot is asked to move
 *       by the rigid motion that best fits every wheel's travel, at each wheel's true radius and
 *       true angle. Only the gripping share of that moves it ({@link Disturbance.Grip}), and
 *       rolling up or down a slope moves it less far across the field.
 *   <li><b>The floor's grip limits how fast its velocity changes.</b> The robot's velocity follows
 *       what the wheels ask for no faster than μ·g. Past that, every wheel skids alike. A push from
 *       outside ({@link Disturbance.Pushed}) adds a velocity of its own.
 *   <li><b>The robot stops at what it runs into</b> ({@link Contact}). Where its bumpers would be
 *       inside a wall or a field element it is moved straight back out, and loses the part of its
 *       velocity that went in, so it stops or slides along. From then on each wheel turns against
 *       the carpet's grip for the part of its rolling that goes into the surface: it stalls at its
 *       current limit if its motor pushes less than the grip, and breaks loose and spins under that
 *       load if it pushes more.
 *   <li><b>The robot settles on the ground</b> ({@link GroundPlane}). It sits on the plane that
 *       best fits the ground under its wheels, which gives its height and its attitude, unless it
 *       is tilted by hand.
 * </ol>
 *
 * <p><b>What is left out</b>, which the truth suite keeps as known gaps (docs/simulator.md): the
 * robot's yaw inertia (it spins as if its mass sat at its wheels); weight transfer, and each
 * wheel's own load and slip; scrub between wheels that disagree (the fit splits the difference); a
 * wheel leaving the ground; gravity along a slope; and, against a wall, the wall's own friction,
 * the bumpers' give, and a sliding tread's lower friction (one coefficient serves for holding and
 * for sliding). Away from a wall the wheels still never feel the floor: past the grip limit they
 * spin up as if they carried the robot, and a push from outside neither turns nor loads them.
 */
public final class KinematicPhysics implements Physics {
  /**
   * How quickly a push gets the robot sliding, and lets it stop, in m/s²: about a third of g, as
   * one robot driving into another starts it moving.
   */
  public static final double PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0;

  /** Where each wheel sits, from the robot's center, in the robot's frame. */
  private final Translation2d[] wheelPlaces;

  private final Wheel[] wheels;

  /** What drives each wheel over a step, filled in by the actuators as the step starts. */
  private final WheelDrive[] drives;

  /** Each wheel's radius as the drivetrain's definition gives it, before any wear. */
  private final double[] configuredRadiusMeters;

  private final RigidFit fit;
  private final GroundPlane ground;

  /** The robot's frame and bumpers: what touches a wall. */
  private final Body body;

  /** Each wheel's load: an equal share of the robot's weight. */
  private final double wheelLoadNewtons;

  /**
   * The wheels' coefficient of friction on the carpet, as the drivetrain's definition gives it. It
   * is used for a tread that holds and for one that slides alike: the definition has the one
   * number, and a sliding tread's lower friction is left out.
   */
  private final double frictionCoefficient;

  /**
   * A wheel rolling this nearly along a surface (the cosine of the angle between its rolling and
   * the way into the surface) isn't held back by it at all.
   */
  private static final double ALONG_THE_SURFACE = 1e-6;

  /**
   * The least a wheel's own moving parts count for, as inertia at the wheel, in kg m²: a drive
   * given none still has some, and a blocked wheel turns nothing else.
   */
  private static final double LEAST_OWN_INERTIA_KG_M2 = 1e-5;

  /**
   * The most the floor's grip can accelerate the robot, in m/s²: its wheels' coefficient of
   * friction times g.
   */
  private final double gripLimitMetersPerSecondSquared;

  /** How far the wheels' rolling asked the robot to move in the last step, in its own frame. */
  private final RigidMotion rolled = new RigidMotion();

  private final Chassis chassis = new Chassis();

  /** The wheels' travel as the last step ended, and as this one ends. They swap every step. */
  private WheelTravel travelBefore;

  private WheelTravel travelNow;

  private Pose2d truePose = Pose2d.ZERO;
  private double yawTurnedRadians;
  private double yawRateRadiansPerSecond;
  private double elapsedSeconds;

  /** The share of the wheels' rolling that moves the robot: 1 with full grip. */
  private double grip = 1;

  private double pitchRadians;
  private double rollRadians;

  private Terrain terrain = Terrain.FLAT;

  /** What the robot's bumpers run into: nothing, until it is given a field's walls and elements. */
  private Contact contact = Contact.openSpace();

  /**
   * How near a surface the robot counts as still against it, in meters: far less than a wheel rolls
   * in a step, and far more than rounding.
   */
  private static final double STILL_TOUCHING_METERS = 1e-6;

  /**
   * The way straight out of the surface the robot is against, as a unit vector on the field: both
   * zero while it touches nothing.
   */
  private double outOfSurfaceX;

  private double outOfSurfaceY;

  /**
   * Whether the robot's attitude is held by hand, whatever the ground says. Otherwise it follows
   * the plane under its wheels.
   */
  private boolean tiltedByHand;

  /** The robot's velocity on the field over the last step: what its IMUs feel. */
  private Translation2d lastVelocity = Translation2d.ZERO;

  /**
   * The share of the robot's velocity that comes from its wheels, on the field: what the wheels ask
   * of it, followed no faster than the floor's grip allows.
   */
  private Translation2d wheelDrivenVelocity = Translation2d.ZERO;

  /**
   * A push the wheels do not see, on the field: the velocity it is headed for, and the velocity it
   * has reached so far.
   */
  private Translation2d pushTarget = Translation2d.ZERO;

  private Translation2d pushVelocity = Translation2d.ZERO;

  /**
   * How far each wheel has rolled the robot along and which way it points, at one instant. The
   * arrays are in the wheels' order.
   */
  private static final class WheelTravel {
    final double[] distancesMeters;
    final double[] anglesRadians;

    WheelTravel(int wheelCount) {
      distancesMeters = new double[wheelCount];
      anglesRadians = new double[wheelCount];
    }

    /** Takes the wheels' travel as it stands now. A wheel rolls the robot at its true radius. */
    void read(Wheel[] wheels) {
      for (int i = 0; i < wheels.length; i++) {
        distancesMeters[i] = wheels[i].rotationRadians() * wheels[i].radiusMeters;
        anglesRadians[i] = wheels[i].steerRadians;
      }
    }
  }

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
    this.body = body;
    wheelPlaces = drivetrain.moduleTranslations();
    fit = new RigidFit(wheelPlaces);
    ground = new GroundPlane(wheelPlaces);
    int wheelCount = wheelPlaces.length;
    configuredRadiusMeters = new double[wheelCount];
    double carriedKilograms = body.mass().in(Kilograms) / wheelCount;
    wheels = new Wheel[wheelCount];
    drives = new WheelDrive[wheelCount];
    for (int i = 0; i < wheelCount; i++) {
      DrivetrainGeometry.ModuleType module = drivetrain.module(i);
      configuredRadiusMeters[i] = module.wheelRadius().in(Meters);
      double ownKgM2 = module.driveInertiaOrDefault().in(KilogramSquareMeters);
      double inertiaKgM2 = driveInertiaKgM2(carriedKilograms, configuredRadiusMeters[i], ownKgM2);
      DCMotor motor = module.driveMotors().model(module.driveGearing().efficiency());
      double trueRadiusMeters = configuredRadiusMeters[i] * wheelRadiusScale.get(i);
      wheels[i] =
          new Wheel(
              i,
              new WheelMotor(motor, inertiaKgM2, module.driveGearing().ratio()),
              trueRadiusMeters,
              inertiaKgM2 - ownKgM2,
              ownKgM2);
      drives[i] = new WheelDrive();
    }
    wheelLoadNewtons = body.mass().in(Kilograms) * STANDARD_GRAVITY / wheelCount;
    frictionCoefficient = drivetrain.wheelCoefficientOfFriction();
    gripLimitMetersPerSecondSquared = frictionCoefficient * STANDARD_GRAVITY;
    travelBefore = new WheelTravel(wheelCount);
    travelNow = new WheelTravel(wheelCount);
    countTravelFromHere();
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
      if (isAgainstSomething()) {
        wheels[i].turnAgainstTheSurface(drives[i], seconds);
      } else {
        wheels[i].turn(drives[i], seconds);
      }
    }
    travelNow.read(wheels);
    fit.motion(
        travelBefore.distancesMeters,
        travelBefore.anglesRadians,
        travelNow.distancesMeters,
        travelNow.anglesRadians,
        rolled);

    Translation2d placeBefore = truePose.getTranslation();
    Pose2d asked = whereTheWheelsAskItToBe();
    Translation2d askedVelocity = asked.getTranslation().minus(placeBefore).div(seconds);

    // Friction as a limit on acceleration: the robot's velocity follows what its wheels ask for,
    // but changes by no more than the grip's acceleration times the step. Past that, they skid.
    wheelDrivenVelocity =
        toward(wheelDrivenVelocity, askedVelocity, gripLimitMetersPerSecondSquared * seconds);
    // A push gets going, and stops, at its own steady acceleration, whatever the wheels do.
    pushVelocity =
        toward(pushVelocity, pushTarget, PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED * seconds);

    // The robot's place moves at the two velocities together. Its heading is the wheels' own: the
    // grip limit holds back how the robot's place changes, not its turn.
    Translation2d placeAfter =
        placeBefore.plus(wheelDrivenVelocity.times(seconds)).plus(pushVelocity.times(seconds));
    truePose = clearOfEverything(new Pose2d(placeAfter, asked.getRotation()));
    noteWhatItIsAgainst();
    settleOnTheGround();

    double turnedRadians = grip * rolled.turnRadians;
    yawTurnedRadians += turnedRadians;
    yawRateRadiansPerSecond = turnedRadians / seconds;
    lastVelocity = truePose.getTranslation().minus(placeBefore).div(seconds);
    elapsedSeconds += seconds;

    WheelTravel reused = travelBefore;
    travelBefore = travelNow;
    travelNow = reused;
  }

  /**
   * Where the wheels' rolling over the last step asks the robot to be: from where it is, moved by
   * the motion that best fits their travel. Only the gripping share of the rolling moves it, and
   * rolling along a slope moves it less far across the field, which is flat.
   */
  private Pose2d whereTheWheelsAskItToBe() {
    double forwardMeters = rolled.forwardMeters;
    double leftMeters = rolled.leftMeters;
    // The slope along the way the robot travels: how much the ground rises for each meter rolled
    // that way, from the plane under the wheels. A robot standing still is on no slope.
    double rolledMeters = Math.hypot(forwardMeters, leftMeters);
    double slopeAlongTravel = 0;
    if (rolledMeters > 0) {
      slopeAlongTravel =
          (ground.riseForward() * forwardMeters + ground.riseLeft() * leftMeters) / rolledMeters;
    }
    // A distance d rolled along a slope s covers d / sqrt(1 + s²) across the field: the cosine of
    // the slope's angle. The grip's share scales it too.
    double shareAcrossTheField = grip / Math.sqrt(1 + slopeAlongTravel * slopeAlongTravel);
    Twist2d moved =
        new Twist2d(
            shareAcrossTheField * forwardMeters,
            shareAcrossTheField * leftMeters,
            grip * rolled.turnRadians);
    return truePose.plus(moved.exp());
  }

  /**
   * A pose moved out of whatever its bumpers would be in there: the walls and the elements ({@link
   * Contact}). The robot keeps its heading, and loses the part of its velocity that went into the
   * surface, so it stops against it or slides along it. Its wheels feel it in their next step
   * ({@link #noteWhatItIsAgainst}): the part of their rolling the robot can't follow slides on the
   * carpet, against its grip.
   */
  private Pose2d clearOfEverything(Pose2d pose) {
    boolean pushed = contact.pushOut(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    if (!pushed) {
      return pose;
    }
    wheelDrivenVelocity = contact.alongTheSurface(wheelDrivenVelocity);
    pushVelocity = contact.alongTheSurface(pushVelocity);
    return new Pose2d(
        pose.getX() + contact.pushedXMeters(),
        pose.getY() + contact.pushedYMeters(),
        pose.getRotation());
  }

  private boolean isAgainstSomething() {
    return outOfSurfaceX != 0 || outOfSurfaceY != 0;
  }

  /**
   * Takes note of the surface the robot is against as the step ends, for the wheels' next step: the
   * way straight out of it. A robot that was just pushed back is against what pushed it. A robot
   * that was against something and was not pushed may only have stopped pushing, as stalled wheels
   * do, so it is asked whether a hair's breadth toward the surface would still be inside it.
   */
  private void noteWhatItIsAgainst() {
    double outX = contact.pushedXMeters();
    double outY = contact.pushedYMeters();
    boolean pushedBack = outX != 0 || outY != 0;
    if (!pushedBack && isAgainstSomething()) {
      contact.pushOut(
          truePose.getX() - STILL_TOUCHING_METERS * outOfSurfaceX,
          truePose.getY() - STILL_TOUCHING_METERS * outOfSurfaceY,
          truePose.getRotation().getRadians());
      outX = contact.pushedXMeters();
      outY = contact.pushedYMeters();
    }
    double pushedMeters = Math.hypot(outX, outY);
    if (pushedMeters == 0) {
      outOfSurfaceX = 0;
      outOfSurfaceY = 0;
      return;
    }
    outOfSurfaceX = outX / pushedMeters;
    outOfSurfaceY = outY / pushedMeters;
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
      case Disturbance.Placed placed -> place(placed.pose());
      case Disturbance.Pushed pushed -> pushTarget = pushed.fieldVelocity();
      case Disturbance.TiltedByHand tilt -> tiltByHand(tilt);
      case Disturbance.Ground onGround -> setDownOn(onGround.terrain());
      case Disturbance.Standing standing -> standAmong(standing);
      case Disturbance.Grip slick -> grip = slick.share();
      case Disturbance.TreadWorn worn -> wearTread(worn);
    }
  }

  /**
   * Sets the robot down at a pose. The turn from where it faced counts as one the robot truly made,
   * so its gyro feels it. Its velocity is kept, except what would carry it into a surface: a robot
   * can't be set down inside a wall or an element, so it is set down at the nearest spot clear of
   * them, and a shove into a wall ends against the wall.
   */
  private void place(Pose2d pose) {
    yawTurnedRadians += pose.getRotation().minus(truePose.getRotation()).getRadians();
    truePose = clearOfEverything(pose);
    settleOnTheGround();
  }

  /** Gives the robot what it runs into, and moves it out of anything it is standing in. */
  private void standAmong(Disturbance.Standing standing) {
    contact = new Contact(body, standing.walls(), standing.elements());
    truePose = clearOfEverything(truePose);
    settleOnTheGround();
  }

  private void tiltByHand(Disturbance.TiltedByHand tilt) {
    tiltedByHand = true;
    pitchRadians = tilt.pitchRadians();
    rollRadians = tilt.rollRadians();
  }

  /** Gives the robot its ground, and sets it back down on it, which undoes a tilt by hand. */
  private void setDownOn(Terrain terrain) {
    this.terrain = terrain;
    tiltedByHand = false;
    settleOnTheGround();
  }

  private void wearTread(Disturbance.TreadWorn worn) {
    wheels[worn.wheel()].radiusMeters = configuredRadiusMeters[worn.wheel()] * worn.scale();
    // The wheels' travel so far was at the old radius, and the same rotation at the new radius is
    // a different distance. Counting again from here keeps the change from moving the robot.
    countTravelFromHere();
  }

  /** Takes the wheels' travel as it stands for where the next step's travel is counted from. */
  private void countTravelFromHere() {
    travelBefore.read(wheels);
  }

  /**
   * A velocity moved toward another, by no more than so much: the whole way if the other is within
   * reach, and otherwise that far along the straight line to it.
   */
  private static Translation2d toward(Translation2d from, Translation2d to, double mostChange) {
    Translation2d change = to.minus(from);
    double changeSize = change.getNorm();
    if (changeSize <= mostChange) {
      return to;
    }
    return from.plus(change.times(mostChange / changeSize));
  }

  /**
   * Sets the robot on the ground where it is: the plane through the ground under its wheels gives
   * its height and, unless it is tilted by hand, its attitude.
   */
  private void settleOnTheGround() {
    ground.fitUnder(terrain, truePose);
    if (tiltedByHand) {
      return;
    }
    // The plane's two slopes as WPILib's angles: nose up is a negative pitch, and the left side up
    // is a positive roll. The roll is taken about the robot's forward axis after it has pitched,
    // which is where the pitch's cosine comes from.
    pitchRadians = -Math.atan(ground.riseForward());
    rollRadians = Math.atan(ground.riseLeft() * Math.cos(pitchRadians));
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
          ground.heightAtCenterMeters(),
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

    /**
     * A point fixed on a turning body moves at the body's velocity plus ω × r, where r is the arm
     * from the body's center to the point. In the plane, ω × r is (−ω·r.y, ω·r.x).
     */
    @Override
    public Translation2d pointVelocity(Translation2d place) {
      Translation2d arm = place.rotateBy(truePose.getRotation());
      return lastVelocity.plus(
          new Translation2d(
              -yawRateRadiansPerSecond * arm.getY(), yawRateRadiansPerSecond * arm.getX()));
    }
  }

  /**
   * One wheel: its drive motor turning it ({@link WheelMotor}), the angle its steering was turned
   * to, its true radius, and where it meets the ground.
   */
  private final class Wheel implements WheelTruth {
    private final int index;
    private final WheelMotor motor;
    private final double carriedInertiaKgM2;
    private final double ownInertiaKgM2;
    private double radiusMeters;
    private double steerRadians;

    /**
     * @param carriedInertiaKgM2 its share of the robot's mass, as inertia at the wheel
     * @param ownInertiaKgM2 its own moving parts, as inertia at the wheel
     */
    Wheel(
        int index,
        WheelMotor motor,
        double radiusMeters,
        double carriedInertiaKgM2,
        double ownInertiaKgM2) {
      this.index = index;
      this.motor = motor;
      this.radiusMeters = radiusMeters;
      this.carriedInertiaKgM2 = carriedInertiaKgM2;
      this.ownInertiaKgM2 = Math.max(ownInertiaKgM2, LEAST_OWN_INERTIA_KG_M2);
    }

    /** Turns the wheel through a step as its actuators say, and points it where they steered it. */
    void turn(WheelDrive drive, double seconds) {
      motor.turn(drive.volts(), drive.currentLimitAmps(), drive.batteryVolts(), seconds);
      steerRadians = drive.steerRadians();
    }

    /**
     * Turns the wheel through a step while the robot is against a surface. The share of the wheel's
     * rolling that goes into the surface slides on the carpet, against the carpet's grip on this
     * wheel: its coefficient of friction times the weight it carries, less on a slick floor. The
     * rest the robot still follows, so the wheel still turns that share of the robot's mass.
     */
    void turnAgainstTheSurface(WheelDrive drive, double seconds) {
      // Which way the wheel rolls across the field, as it is steered for this step.
      double rollsTowardRadians = truePose.getRotation().getRadians() + drive.steerRadians();
      double rollsIntoSurface =
          -(Math.cos(rollsTowardRadians) * outOfSurfaceX
              + Math.sin(rollsTowardRadians) * outOfSurfaceY);
      boolean rollsAlongIt = Math.abs(rollsIntoSurface) < ALONG_THE_SURFACE;
      if (rollsAlongIt) {
        turn(drive, seconds);
        return;
      }
      double shareTheRobotFollows = 1 - rollsIntoSurface * rollsIntoSurface;
      double blockedInertiaKgM2 = ownInertiaKgM2 + shareTheRobotFollows * carriedInertiaKgM2;
      motor.turnAgainst(
          drive.volts(),
          drive.currentLimitAmps(),
          drive.batteryVolts(),
          seconds,
          rollsIntoSurface,
          grip * frictionCoefficient * wheelLoadNewtons,
          radiusMeters,
          blockedInertiaKgM2);
      steerRadians = drive.steerRadians();
    }

    @Override
    public double rotationRadians() {
      return motor.rotationRadians();
    }

    @Override
    public double spinRadPerSec() {
      return motor.speedRadPerSec();
    }

    @Override
    public double steerRadians() {
      return steerRadians;
    }

    @Override
    public double radiusMeters() {
      return radiusMeters;
    }

    @Override
    public double driveCurrentAmps() {
      return motor.currentAmps();
    }

    @Override
    public double driveVolts() {
      return motor.volts();
    }

    @Override
    public boolean driveCurrentLimited() {
      return motor.currentLimited();
    }

    /** No scrub: the steering turns freely on the carpet. */
    @Override
    public double steerLoadNewtonMeters() {
      return 0;
    }

    /** An equal share of the robot's weight, however hard it speeds up or tilts. */
    @Override
    public double loadNewtons() {
      return wheelLoadNewtons;
    }

    /** Always: the robot stands on the plane through the ground under its wheels. */
    @Override
    public boolean inContact() {
      return true;
    }

    @Override
    public Translation3d contactPoint() {
      Translation2d place = wheelPlaces[index];
      Translation2d onTheField =
          truePose.getTranslation().plus(place.rotateBy(truePose.getRotation()));
      return new Translation3d(onTheField.getX(), onTheField.getY(), ground.heightUnder(place));
    }
  }
}

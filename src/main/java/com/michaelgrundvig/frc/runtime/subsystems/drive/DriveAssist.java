package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.runtime.behavior.Why;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.record.DriveMoveTask;
import com.michaelgrundvig.frc.world.record.MoveReason;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import java.util.Locale;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;

/**
 * Driving with the sticks while the robot holds a heading ({@link Drive#facing}): a v3 {@link
 * Command} that runs until it's interrupted. The translation is the driver's, exactly as {@link
 * Drive#driven} drives it. The turning is the drivetrain's, toward the {@link Heading} worked out
 * again every loop from where the robot is, the field and the alliance.
 *
 * <p>Each loop, the robot's turning rate is decided in this order:
 *
 * <ul>
 *   <li><b>Nothing to face.</b> If the heading can't be worked out (the alliance isn't known, or
 *       the robot is on the place it should face), the robot doesn't turn, and {@link #aimed} says
 *       why.
 *   <li><b>The heading law</b> ({@link #turnRateToward}). Otherwise the robot turns at the target's
 *       own rate (the bearing to a place turns as the robot moves past it) plus {@code headingKp}
 *       for each radian it is off, the nearer way round, up to the top turning rate. A computed
 *       rate isn't squared or deadbanded as a stick's is, so the robot settles on the target.
 *   <li><b>The spin-up limit</b> ({@link #spunUpNoFasterThanAMove}). The rate the law asks is
 *       reached no faster than a move would spin the robot up: half the model's turning
 *       acceleration. Slowing down isn't limited, since the law itself slows the robot as it closes
 *       on the target. A quarter turn goes 1.4° past in the simulator, not 3.3°.
 *   <li><b>The driver takes the heading back.</b> While the driver's turn stick is past its
 *       deadband, the robot turns as the stick says instead. Let go, and it returns to the target.
 * </ul>
 *
 * <p>The target and how far off the robot is log as {@code Drive/HeadingTargetDegrees} and {@code
 * Drive/HeadingErrorDegrees}, not a number while there's no target.
 */
@CheckReturnValue
public final class DriveAssist implements Command {
  private final Drive drive;

  /** Where the robot is to face. */
  private final Heading heading;

  /** How near the heading the robot must be for {@link #aimed} to open, in radians. */
  private final double toleranceRadians;

  /**
   * Why the assist cannot work as it was built, if it cannot: its tolerance is not a positive
   * angle. It then ends as it starts, and {@link #aimed} holds false.
   *
   * <p>Having no driver's sticks is not decided here. The assist asks the drive for them each time
   * it starts ({@link #whyNotNow}), so it does not matter whether the robot builds its assists
   * before or after it hands the drive the sticks ({@link Drive#takeDriver}).
   */
  private final Optional<String> refused;

  private final String name;

  /** Its {@link #aimed} gate, made the first time it is asked for. */
  private Optional<Gate> aimed = Optional.empty();

  /**
   * What {@link #aimed} last said of how far off the robot is, and the tenths of a degree it said
   * it for. The text is made again only when the tenths change, so a gate that is asked every loop
   * doesn't format a new string every loop.
   */
  private String offByShown = "";

  private OptionalInt offByShownForTenths = OptionalInt.empty();

  private DriveAssist(
      Drive drive, Heading heading, double toleranceRadians, Optional<String> whyNot) {
    this.drive = drive;
    this.heading = heading;
    this.toleranceRadians = toleranceRadians;
    this.name = "Drive facing " + heading.name();
    this.refused = drive.refusedAsBuilt(whyNot);
  }

  /**
   * Why the assist cannot work right now, if it cannot: the drive has not been handed the driver's
   * sticks, or the assist was refused as it was built. Having no sticks is the first thing to fix,
   * so it is the reason said.
   */
  private Optional<String> whyNotNow() {
    if (drive.driver().isEmpty()) {
      return Optional.of(Drive.NO_DRIVER);
    }
    return refused;
  }

  static DriveAssist facing(Drive drive, Heading heading) {
    return new DriveAssist(drive, heading, DriveMove.TURNED_RADIANS, Optional.empty());
  }

  /**
   * The same assist, {@link #aimed} within another tolerance, in place of the 2° a move arrives
   * within. A tolerance that is zero or less, or not a number, is refused as a joint's move to a
   * place it hasn't is ({@link Refusals}): thrown as the robot starts, and once it runs, an assist
   * that ends as it starts, with an {@link #aimed} that holds false saying why.
   *
   * @throws IllegalArgumentException for such a tolerance, as the robot starts
   */
  public DriveAssist within(Angle tolerance) {
    double radians = tolerance.in(Radians);
    boolean isAPositiveAngle = radians > 0;
    if (isAPositiveAngle) {
      return new DriveAssist(drive, heading, radians, Optional.empty());
    }
    String whyNot =
        name
            + ": aim within a positive angle, not "
            + String.format(Locale.ROOT, "%.1f°", Math.toDegrees(radians));
    return new DriveAssist(drive, heading, radians, Optional.of(whyNot));
  }

  // ---- Is it aimed? ----

  /**
   * A gate that's open while the robot is within the tolerance of the target heading (2° unless
   * {@link #within} says otherwise) and turning no faster or slower than the target does, within
   * {@code 0.2} rad/s: still, for a fixed heading, and keeping up with the bearing while driving
   * past a place. It's a fact about the robot, so it's true or false whether or not this command is
   * running. Closed, it says how far off: "aimed at the Hub is false: 3.4° off the Hub"; or
   * "alliance unknown", or "on the Hub". Until the drive is handed the driver's sticks, the field
   * and the alliance ({@link Drive#takeDriver}), it holds false and says so.
   */
  public Gate aimed() {
    if (aimed.isEmpty()) {
      aimed = Optional.of(aimedGate());
    }
    return aimed.get();
  }

  private Gate aimedGate() {
    String gateName = "aimed at " + heading.name();
    if (refused.isPresent()) {
      return Refusals.gate(drive.getRegisteredScheduler(), gateName, refused.get());
    }
    return new Gate(drive.getRegisteredScheduler(), gateName, this::isAimed);
  }

  /**
   * Whether the robot is aimed as of this loop: on the heading, and turning as the heading does.
   * The field and the alliance are the ones the drive has now, so a gate made before the drive was
   * handed them works once it is.
   *
   * @param whyNot takes each thing that keeps the robot from being aimed, in the gate's words
   */
  private boolean isAimed(Gate.Terms whyNot) {
    Optional<Driver> driver = drive.driver();
    if (driver.isEmpty()) {
      whyNot.add(Drive.NO_DRIVER);
      return false;
    }
    Driver sticks = driver.get();
    Pose2d pose = drive.getPose();
    FieldModel field = sticks.field().get();
    Optional<Alliance> alliance = sticks.alliance().get();
    Optional<Rotation2d> target = heading.resolve(pose, field, alliance);
    if (target.isEmpty()) {
      whyNot.add(alliance.isEmpty() ? "alliance unknown" : "on " + heading.name());
      return false;
    }
    double headingErrorRadians = target.get().minus(pose.getRotation()).getRadians();
    boolean offTheHeading = Math.abs(headingErrorRadians) > toleranceRadians;

    // The robot's turning is judged against the target's own. The bearing to a place turns as the
    // robot drives past it, so a robot that keeps up with the bearing is aimed, though it turns.
    ChassisVelocities moving = drive.getRobotRelativeVelocities();
    double targetTurnRate = targetTurnRate(pose, field, alliance, moving);
    boolean stillTurning =
        Math.abs(moving.omega - targetTurnRate) > DriveMove.STILL_RADIANS_PER_SECOND;

    if (offTheHeading) {
      whyNot.add(offBy(headingErrorRadians));
    }
    if (stillTurning) {
      whyNot.add("still turning");
    }
    return !offTheHeading && !stillTurning;
  }

  /** How far off the heading the robot is, as the gate says it: "3.4° off the Hub". */
  private String offBy(double headingErrorRadians) {
    int tenths = (int) Math.round(Math.abs(Math.toDegrees(headingErrorRadians)) / 0.1);
    boolean alreadyShown =
        offByShownForTenths.isPresent() && offByShownForTenths.getAsInt() == tenths;
    if (!alreadyShown) {
      offByShown = String.format(Locale.ROOT, "%.1f° off %s", tenths * 0.1, heading.name());
      offByShownForTenths = OptionalInt.of(tenths);
    }
    return offByShown;
  }

  // ---- The command ----

  @Override
  public String name() {
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }

  @Override
  public void run(Coroutine coroutine) {
    // The driver's sticks are read here, as the assist starts, and not when it was built.
    Optional<String> whyNot = whyNotNow();
    if (whyNot.isPresent()) {
      // It cannot work this time. It takes its loop all the same, and the drive's default command
      // goes on.
      boolean hasNoDriver = drive.driver().isEmpty();
      Why why =
          hasNoDriver
              ? Why.saying(MoveReason.NO_DRIVER, "")
              : Why.saying(MoveReason.REFUSED_AS_BUILT, whyNot.get());
      drive.recordRefused(name, DriveMoveTask.FACING_A_HEADING, why);
      coroutine.yield();
      return;
    }
    Driver sticks = drive.driver().orElseThrow();
    // The turning rate asked for in the loop before, which the spin-up limit starts from. As the
    // command starts, that is how fast the robot is turning already.
    double lastTurnRateAsked = drive.getRobotRelativeVelocities().omega;
    while (true) {
      DrivetrainControl control = drive.control();
      Pose2d pose = drive.getPose();
      FieldModel field = sticks.field().get();
      Optional<Alliance> alliance = sticks.alliance().get();
      double turnStick = sticks.turn().getAsDouble();
      Optional<Rotation2d> target = heading.resolve(pose, field, alliance);

      double turnRate = 0; // with nothing to face, the robot doesn't turn
      if (target.isPresent()) {
        double headingErrorRadians = target.get().minus(pose.getRotation()).getRadians();
        ChassisVelocities moving = drive.getRobotRelativeVelocities();
        double targetTurnRate = targetTurnRate(pose, field, alliance, moving);
        double wanted = turnRateToward(headingErrorRadians, targetTurnRate, control);
        turnRate = spunUpNoFasterThanAMove(wanted, lastTurnRateAsked);
        logHeading(target.get().getDegrees(), Math.toDegrees(headingErrorRadians));
      } else {
        logNoHeading();
      }
      if (Sticks.isPushed(turnStick)) {
        // The driver takes the heading back for as long as the stick is held.
        turnRate = drive.turnRateAskedBy(turnStick);
      }
      lastTurnRateAsked = turnRate;
      drive.driveField(drive.velocityAskedBy(sticks.translation().get()), turnRate);
      coroutine.yield();
    }
  }

  /**
   * How fast the target heading itself is turning, in radians a second. A fixed heading doesn't.
   * The bearing to a place does, as the robot moves past the place.
   *
   * @param moving how fast the robot is measured to move, relative to itself
   */
  private double targetTurnRate(
      Pose2d pose, FieldModel field, Optional<Alliance> alliance, ChassisVelocities moving) {
    ChassisVelocities onField = moving.toFieldRelative(pose.getRotation());
    Translation2d velocityOnField = new Translation2d(onField.vx, onField.vy);
    return heading.rate(pose, field, alliance, velocityOnField);
  }

  /**
   * The heading law: the turning rate that takes the robot onto the target heading and keeps it
   * there, in radians a second. It is the target's own turning rate, so that the robot keeps up
   * with a target that turns, plus a correction in proportion to how far off the robot is, and no
   * more than the top turning rate either way.
   *
   * @param headingErrorRadians how far the target is from the robot's heading, the nearer way
   *     round, counterclockwise positive
   * @param targetTurnRateRadiansPerSecond how fast the target heading itself turns
   */
  private static double turnRateToward(
      double headingErrorRadians,
      double targetTurnRateRadiansPerSecond,
      DrivetrainControl control) {
    double correction = control.gains().headingKp() * headingErrorRadians;
    double topTurnRate = control.limits().maxAngularSpeed().in(RadiansPerSecond);
    return Math.clamp(targetTurnRateRadiansPerSecond + correction, -topTurnRate, topTurnRate);
  }

  /**
   * The spin-up limit: the turning rate to ask this loop, given the rate the heading law wants and
   * the rate asked in the loop before.
   *
   * <p>Spinning up is limited to the turning acceleration a move asks of the robot ({@link
   * DriveMove#PROFILE_SHARE} of the model's). Faster than that, the robot reaches the part of the
   * turn where the law slows it spinning quicker than its wheels can stop it, and goes past the
   * heading: a quarter turn went 3.3° past without the limit, and goes 1.4° past with it.
   *
   * <p>Slowing down is not limited. The law slows the robot by itself as the robot closes on the
   * heading, and holding the robot to a slower stop would only carry it further past.
   *
   * @param wanted the turning rate the heading law wants, in radians a second
   * @param lastAsked the turning rate asked in the loop before, in radians a second
   */
  private double spunUpNoFasterThanAMove(double wanted, double lastAsked) {
    double mostChangeALoop =
        DriveMove.PROFILE_SHARE * drive.accelerations().angular() * RobotLoop.PERIOD_SECONDS;
    // A robot asked to turn back the other way first has to stop, which isn't limited, so its
    // spinning up the other way starts from standing still.
    boolean sameWayAsBefore = wanted * lastAsked >= 0;
    double startingFrom = sameWayAsBefore ? lastAsked : 0;
    boolean spinningUp = Math.abs(wanted) > Math.abs(startingFrom);
    if (!spinningUp) {
      return wanted;
    }
    double change = Math.clamp(wanted - startingFrom, -mostChangeALoop, mostChangeALoop);
    return startingFrom + change;
  }

  private static void logHeading(double targetDegrees, double errorDegrees) {
    Logger.recordOutput("Drive/HeadingTargetDegrees", targetDegrees);
    Logger.recordOutput("Drive/HeadingErrorDegrees", errorDegrees);
  }

  /** The log shows a gap, not a number, while there is no heading to face. */
  private static void logNoHeading() {
    logHeading(Double.NaN, Double.NaN);
  }

  @Override
  public void onCancel() {
    logNoHeading();
  }

  @Override
  public String toString() {
    return name;
  }
}

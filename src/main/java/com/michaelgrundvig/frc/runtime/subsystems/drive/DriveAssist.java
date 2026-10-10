package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
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
 * <p>Each loop:
 *
 * <ul>
 *   <li>While the driver's turn stick is past its deadband, the robot turns as the stick says: the
 *       driver takes the heading back. Let go, and it returns to the target.
 *   <li>Otherwise, with a target, the turning rate is the target's own rate (the bearing to a place
 *       turns as the robot moves past it) plus {@code headingKp} for each radian off, the nearer
 *       way round, up to the top turning rate. A computed rate isn't squared or deadbanded as a
 *       stick's is, so the robot settles on the target. The robot spins up no faster than a move
 *       would (half the model's turning acceleration); slowing down follows the same law, so a
 *       quarter turn goes 1.4° past in the simulator, not 3.3°.
 *   <li>With no target (the alliance isn't known, or the robot is on the place) it doesn't turn,
 *       and {@link #aimed} says why.
 * </ul>
 *
 * <p>The target and how far off the robot is log as {@code Drive/HeadingTargetDegrees} and {@code
 * Drive/HeadingErrorDegrees}, not a number while there's no target.
 */
@CheckReturnValue
public final class DriveAssist implements Command {
  private final Drive drive;
  private final Heading heading;
  private final double toleranceRadians;
  private final Drive.@Nullable Driver driver;

  /** Why it can't work, as it's built, or null. */
  private final @Nullable String refused;

  private final String name;
  private @Nullable Gate aimed;

  /** The off-by text last made, and the tenths of a degree it was made for. */
  private int shownTenths = Integer.MIN_VALUE;

  private String shownOff = "";

  private DriveAssist(Drive drive, Heading heading, double toleranceRadians, @Nullable String why) {
    this.drive = drive;
    this.heading = heading;
    this.toleranceRadians = toleranceRadians;
    this.driver = drive.driver();
    this.refused =
        Refusals.asBuilt(drive.getRegisteredScheduler(), driver == null ? Drive.NO_DRIVER : why);
    this.name = "Drive facing " + heading.name();
  }

  static DriveAssist facing(Drive drive, Heading heading) {
    return new DriveAssist(drive, heading, DriveMove.TURNED_RADIANS, null);
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
    return new DriveAssist(
        drive,
        heading,
        radians,
        radians > 0
            ? null
            : name
                + ": aim within a positive angle, not "
                + String.format(Locale.ROOT, "%.1f°", Math.toDegrees(radians)));
  }

  /**
   * A gate that's open while the robot is within the tolerance of the target heading (2° unless
   * {@link #within} says otherwise) and turning no faster or slower than the target does, within
   * {@code 0.2} rad/s: still, for a fixed heading, and keeping up with the bearing while driving
   * past a place. It's a fact about the robot, so it's true or false whether or not this command is
   * running. Closed, it says how far off: "aimed at the Hub is false: 3.4° off the Hub"; or
   * "alliance unknown", or "on the Hub".
   */
  public Gate aimed() {
    Gate gate = aimed;
    if (gate == null) {
      String gateName = "aimed at " + heading.name();
      Drive.Driver sticks = driver;
      gate =
          refused != null || sticks == null
              ? Refusals.gate(
                  drive.getRegisteredScheduler(),
                  gateName,
                  refused != null ? refused : Drive.NO_DRIVER)
              : new Gate(drive.getRegisteredScheduler(), gateName, why -> isAimed(sticks, why));
      aimed = gate;
    }
    return gate;
  }

  private boolean isAimed(Drive.Driver sticks, List<String> why) {
    Pose2d pose = drive.getPose();
    Optional<Alliance> alliance = sticks.alliance().get();
    Optional<Rotation2d> target = heading.resolve(pose, sticks.field().get(), alliance);
    if (target.isEmpty()) {
      why.add(alliance.isEmpty() ? "alliance unknown" : "on " + heading.name());
      return false;
    }
    double error = target.get().minus(pose.getRotation()).getRadians();
    boolean off = Math.abs(error) > toleranceRadians;
    // Still against the target, which turns as the robot moves past a place: a robot tracking
    // the bearing is aimed, though it turns.
    ChassisVelocities moving = drive.getRobotRelativeVelocities();
    ChassisVelocities onField = moving.toFieldRelative(pose.getRotation());
    double rate =
        heading.rate(
            pose, sticks.field().get(), alliance, new Translation2d(onField.vx, onField.vy));
    boolean turning = Math.abs(moving.omega - rate) > DriveMove.STILL_RADIANS_PER_SECOND;
    if (off) {
      why.add(off(error));
    }
    if (turning) {
      why.add("still turning");
    }
    return !off && !turning;
  }

  /** "3.4° off the Hub", made again only when the tenth of a degree changes. */
  private String off(double errorRadians) {
    int tenths = (int) Math.round(Math.abs(Math.toDegrees(errorRadians)) / 0.1);
    if (tenths != shownTenths) {
      shownTenths = tenths;
      shownOff = String.format(Locale.ROOT, "%.1f° off %s", tenths * 0.1, heading.name());
    }
    return shownOff;
  }

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
    Drive.Driver sticks = driver;
    if (refused != null || sticks == null) {
      // Never works: it takes its loop all the same, and the drive's default command goes on.
      Logger.recordOutput("Drive/Outcome", name + ": REFUSED: " + refused);
      coroutine.yield();
      return;
    }
    // The turning rate last asked for, which a computed rate may change only so fast.
    double asked = drive.getRobotRelativeVelocities().omega;
    while (true) {
      DrivetrainControl control = drive.control();
      Pose2d pose = drive.getPose();
      FieldModel field = sticks.field().get();
      Optional<Alliance> alliance = sticks.alliance().get();
      double stick = sticks.turn().getAsDouble();
      Optional<Rotation2d> facing = heading.resolve(pose, field, alliance);
      double omega = 0;
      double target = Double.NaN;
      double error = Double.NaN;
      if (facing.isPresent()) {
        double off = facing.get().minus(pose.getRotation()).getRadians();
        target = facing.get().getDegrees();
        error = Math.toDegrees(off);
        ChassisVelocities moving =
            drive.getRobotRelativeVelocities().toFieldRelative(pose.getRotation());
        double turn =
            heading.rate(pose, field, alliance, new Translation2d(moving.vx, moving.vy))
                + control.gains().headingKp() * off;
        double fastest = control.limits().maxAngularSpeed().in(RadiansPerSecond);
        double want = Math.clamp(turn, -fastest, fastest);
        // Spinning up is limited to the acceleration a move asks of the robot: faster, and it
        // reaches the heading's braking zone spinning quicker than the wheels can stop it (a 90°
        // snap overshot 3.3°, 1.4° with the limit). Slowing down follows the heading's own law.
        double step =
            DriveMove.PROFILE_SHARE * drive.accelerations().angular() * RobotLoop.PERIOD_SECONDS;
        double from = want * asked >= 0 ? asked : 0; // turning back starts from still
        omega =
            Math.abs(want) > Math.abs(from) ? from + Math.clamp(want - from, -step, step) : want;
      }
      if (Math.abs(stick) > Drive.STICK_DEADBAND) {
        omega = drive.turnFrom(stick);
      }
      asked = omega;
      Logger.recordOutput("Drive/HeadingTargetDegrees", target);
      Logger.recordOutput("Drive/HeadingErrorDegrees", error);
      drive.driveField(drive.translationFrom(sticks.translation().get()), omega);
      coroutine.yield();
    }
  }

  @Override
  public void onCancel() {
    Logger.recordOutput("Drive/HeadingTargetDegrees", Double.NaN);
    Logger.recordOutput("Drive/HeadingErrorDegrees", Double.NaN);
  }

  @Override
  public String toString() {
    return name;
  }
}

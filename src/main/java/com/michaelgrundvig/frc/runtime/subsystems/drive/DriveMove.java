package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.path.Keyframe;
import com.michaelgrundvig.frc.world.path.Trajectories;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalLong;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Time;

/**
 * A pre-built command on the drivetrain, as a joint's move is on its joint: a v3 {@link Command}
 * that records how it ended, one of four {@link Outcome}s, with a reason it logs ({@code
 * Drive/Outcome}).
 *
 * <ul>
 *   <li>{@link Drive#runUntil}: at a velocity, robot-relative, until a condition holds: {@link
 *       Outcome#ARRIVED}. It has no deadline of its own. Given one ({@link #within}), it ends
 *       {@link Outcome#TIMED_OUT} then, saying why a gate never opened ("bumped is false: front
 *       bumper lost").
 *   <li>{@link Drive#goTo}: to a field pose, in a straight line, turning on the way; {@link
 *       Drive#turnTo}, turning where it stands; and {@link Drive#follow}, a smooth path through
 *       keyframes. Each plans a timed path as it starts, from where the robot is, under the robot's
 *       own accelerations ({@link DrivetrainControl#accelerations}), and follows it: the wheels
 *       given the volts the path's acceleration takes, the robot pushed back onto the path each
 *       loop. {@link Outcome#ARRIVED} at its end within 2 cm and 2° (a turn, 2°), nearly still, for
 *       40 ms; or {@link Outcome#TIMED_OUT} after twice the path's time and half a second, saying
 *       where it got to; or {@link Outcome#REFUSED} if no path goes through its keyframes.
 * </ul>
 *
 * <p>{@link #within} sets a path's move another deadline, and {@link #passed} waits for a named
 * keyframe. Canceled, it's {@link Outcome#CANCELED}, and says so if it was because the robot was
 * disabled, which cancels every command. It stops the drivetrain as it ends; the drive's default
 * command takes over from the next loop. Each run takes a loop at least, one that ends as it starts
 * too, so a command that awaits moves in a loop goes round once a loop. Started while the robot is
 * disabled, it waits for it to be enabled, and its time and its deadline start then.
 *
 * <p><b>Inside</b>, what a move does is its {@link DriveTask}, one kind for each of the moves
 * above. This class is the command around the task: it starts a run, keeps the run's deadline, and
 * records how the run ended. How a path is followed from loop to loop is in {@link PathFollower}.
 */
@CheckReturnValue
public final class DriveMove implements Command {
  /** How near the path's last place a move must come to arrive, in meters. */
  static final double PLACED_METERS = 0.02;

  /** How near the path's last heading a move must come to arrive, in radians. */
  static final double TURNED_RADIANS = Math.toRadians(2);

  /** How slowly the robot must be moving for a move to arrive, in meters a second. */
  static final double STILL_METERS_PER_SECOND = 0.05;

  /** How slowly the robot must be turning for a move to arrive, in radians a second. */
  static final double STILL_RADIANS_PER_SECOND = 0.2;

  /**
   * The share of the robot's own accelerations, by its model, a move is profiled at. Half: the
   * model's yaw inertia is an estimate until the robot is characterized, and the simulator turns
   * the robot as if its mass sat at its wheels (10.8 kg m² for the template's robot against its
   * 6.7), reaching about 60% of the model's figure. At three quarters, a turn fell 15° behind.
   */
  static final double PROFILE_SHARE = 0.5;

  // What the move is. None of it changes: within() makes a new move.

  private final Drive drive;

  /** What the move does. */
  private final DriveTask task;

  /** The move's name, as the log and its outcome say it: "drive → 90.0°". */
  private final String name;

  /**
   * The deadline a team gave the move ({@link #within}), in seconds, or empty for the move's own:
   * for a path a deadline worked out from the path's time, and for a run-until none. One that was
   * given is a time above zero: {@link #within} refuses any other.
   */
  private final OptionalDouble within;

  /**
   * Why the move cannot run at all, if it cannot, whatever kind it is: it then ends refused as it
   * starts.
   */
  private final Optional<String> refused;

  /** The gates on its named keyframes that have been asked for, by keyframe name. */
  private final Map<String, Gate> passedGates = new HashMap<>();

  // How its latest run went, or is going.

  /** How the latest run ended. Empty while it runs, and before the move has first run. */
  private Optional<Outcome> outcome = Optional.empty();

  private String reason = "";
  private boolean running;

  /** Whether the latest run has waited for a loop yet. */
  private boolean tookALoop;

  private long startNanos;

  /**
   * When the latest run's deadline passes, on the robot's clock. Empty until it is known: a path's
   * deadline depends on the path, which is planned as the run starts. Empty for the whole run of a
   * run-until that was given none.
   */
  private OptionalLong deadlineNanos = OptionalLong.empty();

  /** Whether the path has passed each keyframe in the latest run, in the keyframes' order. */
  private final boolean[] passed;

  /** When the robot came to the path's end in the latest run, or empty while it is not there. */
  private OptionalLong atEndSince = OptionalLong.empty();

  private DriveMove(
      Drive drive, DriveTask task, String name, OptionalDouble within, Optional<String> refused) {
    this.drive = drive;
    this.task = task;
    this.name = name;
    this.within = within;
    this.refused = refused;
    passed = new boolean[task.keyframeNames().size()];
  }

  // ---- The kinds of move ----

  static DriveMove runUntil(Drive drive, ChassisVelocities velocity, BooleanSupplier until) {
    String name = "drive until";
    if (until instanceof Gate gate) {
      name = "drive until " + gate.name();
    }
    return new DriveMove(
        drive,
        new DriveTask.RunUntil(velocity, until),
        name,
        OptionalDouble.empty(),
        Optional.empty());
  }

  static DriveMove turnTo(Drive drive, Rotation2d heading) {
    String name = String.format(Locale.ROOT, "drive → %.1f°", heading.getDegrees());
    return new DriveMove(
        drive, new DriveTask.TurnTo(heading), name, OptionalDouble.empty(), Optional.empty());
  }

  static DriveMove goTo(Drive drive, Pose2d pose) {
    String name = "drive → " + shown(pose);
    return new DriveMove(
        drive, new DriveTask.ToPose(pose), name, OptionalDouble.empty(), Optional.empty());
  }

  /**
   * To a place, found for the robot's alliance as the move starts. Built before the drive has the
   * field to find it on, it is refused as a path to a place is ({@link Refusals}); started with the
   * alliance unknown, it ends {@link Outcome#REFUSED} as it starts.
   *
   * @throws IllegalArgumentException as the robot starts, if the drive has no field to find it on
   */
  static DriveMove goTo(Drive drive, Place place) {
    Optional<String> noField =
        drive.driver().isEmpty() ? Optional.of(Drive.NO_FIELD) : Optional.empty();
    Optional<String> refused = drive.refusedAsBuilt(noField);
    String name = "drive → " + place.name();
    return new DriveMove(
        drive, new DriveTask.ToPlace(place), name, OptionalDouble.empty(), refused);
  }

  /**
   * A path through keyframes. One with none, or two of one name, can't be, and is refused as a
   * joint's move to a place not its own is ({@link Refusals}): thrown as the robot starts, and once
   * it runs, it ends {@link Outcome#REFUSED} as it starts, saying why.
   *
   * @throws IllegalArgumentException with no keyframes, or two of one name, as the robot starts
   */
  static DriveMove follow(Drive drive, List<Keyframe> keyframes) {
    List<Keyframe> path = List.copyOf(keyframes);
    Optional<String> refused = drive.refusedAsBuilt(whyNotAPath(drive, path));
    String name = "drive → " + shownPath(path);
    return new DriveMove(drive, DriveTask.Through.of(path), name, OptionalDouble.empty(), refused);
  }

  /**
   * Why keyframes cannot be a path at all, whatever the robot is doing when the move starts, or
   * empty when they can. The first thing wrong is the one said.
   */
  private static Optional<String> whyNotAPath(Drive drive, List<Keyframe> path) {
    if (path.isEmpty()) {
      return Optional.of("a path needs a keyframe: give one, Keyframe.at(pose)");
    }
    List<String> namesSoFar = new ArrayList<>();
    boolean anyAtAPlace = false;
    for (Keyframe keyframe : path) {
      String keyframeName = keyframe.name();
      if (keyframeName != null && namesSoFar.contains(keyframeName)) {
        return Optional.of("two keyframes are named " + keyframeName + ": name each once");
      }
      if (keyframeName != null) {
        namesSoFar.add(keyframeName);
      }
      anyAtAPlace |= keyframe.place() != null;
    }
    if (anyAtAPlace && drive.driver().isEmpty()) {
      return Optional.of(Drive.NO_FIELD);
    }
    return Optional.empty();
  }

  /**
   * A path as a move's name shows it: its keyframes joined by arrows, and the heading it ends at,
   * if its last keyframe is at a pose. "(2.00, 1.00) → pickup → (7.00, 2.00) 180.0°".
   */
  private static String shownPath(List<Keyframe> path) {
    if (path.isEmpty()) {
      return "a path";
    }
    List<String> shown = new ArrayList<>();
    for (Keyframe keyframe : path) {
      shown.add(keyframe.toString());
    }
    String joined = String.join(" → ", shown);
    Pose2d end = path.get(path.size() - 1).pose();
    if (end == null) {
      return joined;
    }
    return joined + String.format(Locale.ROOT, " %.1f°", end.getRotation().getDegrees());
  }

  /** A pose as a move's name shows it: "(2.00, 1.00) 90.0°". */
  private static String shown(Pose2d pose) {
    return String.format(
        Locale.ROOT,
        "(%.2f, %.2f) %.1f°",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  // ---- Its options and its gates ----

  /**
   * The same move with another deadline: {@code drive.turnTo(heading).within(Seconds.of(1))}. For a
   * run-until it is the only deadline the move has. A deadline that is not a time above zero (none,
   * a negative one, one without end, or one that is not a number) is refused, whatever kind of move
   * it is, as a joint's move refuses an option that can't be ({@link Refusals}): thrown as the
   * robot starts, and once it runs, the move ends {@link Outcome#REFUSED} as it starts, saying why.
   *
   * @throws IllegalArgumentException with the move's name and why, for such a deadline, as the
   *     robot starts
   */
  public DriveMove within(Time deadline) {
    double seconds = deadline.in(Seconds);
    Optional<String> notADeadline = Outcome.whyNotADeadline(seconds);
    if (notADeadline.isPresent()) {
      return refusedFor(notADeadline.get());
    }
    return new DriveMove(drive, task, name, OptionalDouble.of(seconds), refused);
  }

  /**
   * The same move, refused for an option that can't be. As the robot starts, that is thrown after
   * the move's name. Once the robot runs, the move ends {@link Outcome#REFUSED} as it starts,
   * saying why it was refused first.
   *
   * @throws IllegalArgumentException with the move's name and why, as the robot starts
   */
  private DriveMove refusedFor(String why) {
    if (Refusals.starting(drive.getRegisteredScheduler())) {
      throw new IllegalArgumentException(name + ": " + why);
    }
    Optional<String> refusedFirst = refused.isPresent() ? refused : Optional.of(why);
    return new DriveMove(drive, task, name, within, refusedFirst);
  }

  /**
   * A gate that opens as this move's path passes a named keyframe, and stays open until the move
   * next starts: {@code co.waitUntil(leg.passed("pickup"))}. It's the path's time that passes it,
   * which the robot follows within centimeters. A name none of its keyframes has is refused as a
   * joint's gate on a place not its own is ({@link Refusals#gate}): thrown as the robot starts, and
   * once it runs, a gate that holds false saying why.
   *
   * <p>Asked for with no name at all, it is refused the same way: a keyframe without a name has no
   * gate.
   *
   * @throws IllegalArgumentException for a name none of its keyframes has, or none, as the robot
   *     starts
   */
  public Gate passed(String keyframe) {
    List<Optional<String>> names = task.keyframeNames();
    // A name is asked for, though the compiler can't stop a team's code passing none. Without this
    // check, no name would find the path's first keyframe that has none, and open as it passed.
    boolean namesAKeyframe = keyframe != null;
    if (!namesAKeyframe) {
      return Refusals.gate(
          drive.getRegisteredScheduler(), "passed a keyframe with no name", whyNoName(names));
    }
    int index = names.indexOf(Optional.of(keyframe));
    boolean hasThatKeyframe = index >= 0;
    if (!hasThatKeyframe) {
      return Refusals.gate(
          drive.getRegisteredScheduler(), "passed " + keyframe, whyNoKeyframe(keyframe, names));
    }
    return passedGates.computeIfAbsent(
        keyframe,
        asked -> Gate.of(drive.getRegisteredScheduler(), "passed " + asked, () -> passed[index]));
  }

  /** What a gate asked for with no keyframe's name says, with what to do about it. */
  private String whyNoName(List<Optional<String>> names) {
    List<String> named = namesGiven(names);
    String whatToDo = "its keyframes are " + String.join(", ", named);
    if (named.isEmpty()) {
      whatToDo = "name one, Keyframe.at(...).named(\"pickup\"), and ask for it by that name";
    }
    return name + ": passed() needs a keyframe's name, not null: " + whatToDo;
  }

  /** What a gate on a keyframe the move doesn't have says, with what to do about it. */
  private String whyNoKeyframe(String keyframe, List<Optional<String>> names) {
    List<String> named = namesGiven(names);
    String whatToDo = "its keyframes are " + String.join(", ", named);
    if (named.isEmpty()) {
      whatToDo = "name one, Keyframe.at(...).named(\"" + keyframe + "\")";
    }
    return name + " has no keyframe " + keyframe + ": " + whatToDo;
  }

  /** The names its keyframes were given, in the keyframes' order. */
  private static List<String> namesGiven(List<Optional<String>> names) {
    List<String> named = new ArrayList<>();
    for (Optional<String> each : names) {
      each.ifPresent(named::add);
    }
    return named;
  }

  // ---- How its latest run went ----

  /** Whether its latest run arrived: its condition held, or it got where it was going. */
  public boolean arrived() {
    return outcome.isPresent() && outcome.get() == Outcome.ARRIVED;
  }

  /** How its latest run ended: empty while it runs, or before it first has. */
  public Optional<Outcome> outcome() {
    return outcome;
  }

  /** Why its latest run ended as it did: "drive → 90.0°: ARRIVED in 0.84 s". */
  public String reason() {
    return reason;
  }

  /**
   * When its latest run's deadline passes, in nanoseconds on the robot's clock. Empty before its
   * path is planned, once the run has ended, and for a run-until that was given none. For tests
   * that check every move ends by its deadline.
   */
  OptionalLong deadlineNanos() {
    return running ? deadlineNanos : OptionalLong.empty();
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
  public String toString() {
    return name;
  }

  // ---- A run ----

  @Override
  public void run(Coroutine coroutine) {
    startRun();
    if (refused.isPresent()) {
      end(Outcome.REFUSED, "", refused);
    } else {
      waitToBeEnabled(coroutine);
      switch (task) {
        case DriveTask.RunUntil runUntil -> driveUntil(coroutine, runUntil);
        case DriveTask.AlongAPath path -> follow(coroutine, path);
      }
    }
    if (!tookALoop) {
      // The run ended in the loop it started, refused or already there. It takes a loop all the
      // same, so that a command that awaits moves in a loop goes round once a robot loop, and
      // never without end inside one.
      coroutine.yield();
    }
  }

  /** Forgets how the last run went, and notes when this one starts. */
  private void startRun() {
    running = true;
    outcome = Optional.empty();
    reason = "";
    Arrays.fill(passed, false);
    atEndSince = OptionalLong.empty();
    startNanos = RobotController.getTime();
    deadlineNanos = OptionalLong.empty();
    tookALoop = false;
  }

  /**
   * Waits for the robot to be enabled, if it is not. A disabled drivetrain is not driven, so a move
   * that began now would only run down its deadline. The run is timed from when the robot is
   * enabled, and its path is planned from where the robot is then.
   */
  private void waitToBeEnabled(Coroutine coroutine) {
    while (!drive.isEnabled()) {
      tookALoop = true;
      coroutine.yield();
    }
    startNanos = RobotController.getTime();
  }

  @Override
  public void onCancel() {
    if (running) {
      end(Outcome.CANCELED, "", drive.whyCanceledNow());
    }
  }

  /**
   * Drives at the task's velocity until its condition holds. It has no deadline unless its team
   * gave it one ({@link #within}): nothing says how long a condition takes to hold.
   */
  private void driveUntil(Coroutine coroutine, DriveTask.RunUntil runUntil) {
    OptionalLong deadline = deadlineItWasGiven();
    while (!runUntil.until().getAsBoolean()) {
      boolean pastItsDeadline =
          deadline.isPresent() && RobotController.getTime() >= deadline.getAsLong();
      if (pastItsDeadline) {
        end(Outcome.TIMED_OUT, "", runUntil.whyItNeverHeld());
        return;
      }
      drive.runVelocity(runUntil.velocity());
      tookALoop = true;
      coroutine.yield();
    }
    end(Outcome.ARRIVED);
  }

  /**
   * Plans the task's path from where the robot is, and follows it until the robot has arrived at
   * its end or the deadline passes.
   */
  private void follow(Coroutine coroutine, DriveTask.AlongAPath path) {
    DrivetrainControl control = drive.control();
    Trajectories.Plan plan;
    try {
      plan = planned(path, control);
    } catch (IllegalArgumentException e) {
      end(Outcome.REFUSED, "", Optional.ofNullable(e.getMessage()));
      return;
    }
    PathFollower follower = new PathFollower(drive, plan.trajectory(), path.arrival(), control);
    long deadline = deadlineAfter(deadlineSecondsFor(follower.pathSeconds()));

    while (true) {
      Pose2d pose = drive.getPose();
      ChassisVelocities measuredOnField =
          drive.getRobotRelativeVelocities().toFieldRelative(pose.getRotation());
      double elapsedSeconds = elapsedSeconds();
      notePassedKeyframes(elapsedSeconds, plan.passing());

      boolean atItsEnd = follower.isAtItsEnd(elapsedSeconds, pose, measuredOnField);
      long now = RobotController.getTime();
      if (hasStayedAtItsEnd(atItsEnd, now)) {
        end(Outcome.ARRIVED);
        return;
      }
      if (now >= deadline) {
        end(Outcome.TIMED_OUT, ", " + whereItGotTo(path.arrival(), pose), Optional.empty());
        return;
      }
      follower.drive(elapsedSeconds, pose, measuredOnField);
      tookALoop = true;
      coroutine.yield();
    }
  }

  /**
   * The task's path, planned from where the robot is and how it is moving now, under the top speeds
   * as tuned and a share of the accelerations the robot's model gives ({@link #PROFILE_SHARE}).
   *
   * @throws IllegalArgumentException saying why, if no path can be planned
   */
  private Trajectories.Plan planned(DriveTask.AlongAPath path, DrivetrainControl control) {
    DrivetrainControl.Accelerations most = drive.accelerations();
    Trajectories.Limits limits =
        new Trajectories.Limits(
            control.limits().maxLinearSpeed().in(MetersPerSecond),
            control.limits().maxAngularSpeed().in(RadiansPerSecond),
            PROFILE_SHARE * most.linear(),
            PROFILE_SHARE * most.angular());
    Pose2d start = drive.getPose();
    ChassisVelocities movingOnField =
        drive.getRobotRelativeVelocities().toFieldRelative(start.getRotation());
    return path.plan(drive, start, movingOnField, limits);
  }

  /**
   * How long a path's move has before it times out, in seconds: what a team gave it ({@link
   * #within}), or else the deadline every move has by its profile's time ({@link
   * Outcome#deadline}).
   */
  private double deadlineSecondsFor(double pathSeconds) {
    return within.orElse(Outcome.deadline(pathSeconds));
  }

  /** Sets this run's deadline to the one its team gave the move, if they gave one. */
  private OptionalLong deadlineItWasGiven() {
    if (within.isEmpty()) {
      return OptionalLong.empty();
    }
    return OptionalLong.of(deadlineAfter(within.getAsDouble()));
  }

  /** Sets this run's deadline to so many seconds after it started, and hands it back. */
  private long deadlineAfter(double seconds) {
    long deadline = startNanos + Math.round(seconds * 1e9);
    deadlineNanos = OptionalLong.of(deadline);
    return deadline;
  }

  /**
   * Opens the gate of each keyframe the path has passed by now. A keyframe is passed by the path's
   * timing, not by where the robot is.
   *
   * @param passingSeconds when the path passes each keyframe, in seconds after the move starts
   */
  private void notePassedKeyframes(double elapsedSeconds, double[] passingSeconds) {
    for (int keyframe = 0; keyframe < passingSeconds.length; keyframe++) {
      passed[keyframe] |= elapsedSeconds >= passingSeconds[keyframe];
    }
  }

  /**
   * Notes whether the robot is at the path's end as of this loop, and says whether it has been
   * there long enough to count as arrived. It must stay for {@link Outcome#DWELL_NANOS}, so that a
   * robot passing through its target is not taken to have arrived.
   */
  private boolean hasStayedAtItsEnd(boolean atItsEnd, long now) {
    if (!atItsEnd) {
      atEndSince = OptionalLong.empty();
      return false;
    }
    if (atEndSince.isEmpty()) {
      atEndSince = OptionalLong.of(now);
      return false;
    }
    return now - atEndSince.getAsLong() >= Outcome.DWELL_NANOS;
  }

  /** Where the robot got to, as a move that timed out says it. */
  private static String whereItGotTo(DriveTask.Arrival arrival, Pose2d pose) {
    return switch (arrival) {
      case FACING_ITS_HEADING ->
          String.format(Locale.ROOT, "heading at %.1f°", pose.getRotation().getDegrees());
      case AT_ITS_POSE -> "at " + shown(pose).replace(") ", ") facing ");
    };
  }

  /** How long this run has gone on, in seconds. */
  private double elapsedSeconds() {
    return (RobotController.getTime() - startNanos) / 1e9;
  }

  /** Ends its run with nothing to add to how it ended. */
  private void end(Outcome how) {
    end(how, "", Optional.empty());
  }

  /**
   * Ends its run: records how it ended and the reason it logs ({@link Outcome#reason}), and stops
   * the drivetrain.
   *
   * @param details where the robot got to, for a path that timed out, led by its own ", "; empty
   *     otherwise
   * @param why why it ended so, if there is a reason: a gate's terms, or a refusal
   */
  private void end(Outcome how, String details, Optional<String> why) {
    running = false;
    outcome = Optional.of(how);
    reason = how.reason(name, elapsedSeconds(), details, why);
    drive.runVelocity(new ChassisVelocities());
    Logger.recordOutput("Drive/Outcome", reason);
  }
}

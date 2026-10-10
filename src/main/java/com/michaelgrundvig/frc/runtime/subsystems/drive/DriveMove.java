package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.path.Keyframe;
import com.michaelgrundvig.frc.world.path.Trajectories;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Time;

/**
 * A pre-built command on the drivetrain, as a joint's move is on its joint: a v3 {@link Command}
 * that records how it ended, one of four {@link Outcome}s, with a reason it logs ({@code
 * Drive/Outcome}).
 *
 * <ul>
 *   <li>{@link Drive#runUntil}: at a velocity, robot-relative, until a condition holds: {@link
 *       Outcome#ARRIVED}; or {@link Outcome#TIMED_OUT} after 3 s, saying why a gate never opened
 *       ("bumped is false: front bumper lost").
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
 * <p>{@link #within} sets another deadline, and {@link #passed} waits for a named keyframe.
 * Canceled, it's {@link Outcome#CANCELED}. It stops the drivetrain as it ends; the drive's default
 * command takes over from the next loop. Each run takes a loop at least, one that ends as it starts
 * too, so a command that awaits moves in a loop goes round once a loop.
 */
@CheckReturnValue
public final class DriveMove implements Command {
  /**
   * How near its pose a move must come to arrive, and how still, for the dwell every move arrives
   * by ({@link Outcome#DWELL_NANOS}).
   */
  static final double PLACED_METERS = 0.02;

  static final double TURNED_RADIANS = Math.toRadians(2);

  static final double STILL_METERS_PER_SECOND = 0.05;

  static final double STILL_RADIANS_PER_SECOND = 0.2;

  /**
   * How much of the gap between the profile's speed and turning rate and the robot's measured ones
   * a move adds to what it asks. Even given the volts each acceleration takes, the drivetrain's
   * speed loop trails a little; damping it takes a quarter turn from 49 loops to 42 in the
   * simulator.
   */
  static final double RATE_DAMPING = 0.5;

  /**
   * The share of the robot's own accelerations, by its model, a move is profiled at. Half: the
   * model's yaw inertia is an estimate until the robot is characterized, and the simulator turns
   * the robot as if its mass sat at its wheels (10.8 kg m² for the template's robot against its
   * 6.7), reaching about 60% of the model's figure. At three quarters, a turn fell 15° behind.
   */
  static final double PROFILE_SHARE = 0.5;

  private enum Kind {
    RUN_UNTIL,
    FOLLOW
  }

  /** What a following move follows, worked out as it starts from where the robot is then. */
  @FunctionalInterface
  interface Planner {
    /**
     * @throws IllegalArgumentException with why it won't go
     */
    Trajectories.Plan plan(Pose2d start, ChassisVelocities moving, Trajectories.Limits limits);
  }

  private final Drive drive;
  private final Kind kind;
  private final String name;
  private final ChassisVelocities velocity;
  private final @Nullable BooleanSupplier until;
  private final @Nullable Planner planner;

  /** Whether it only turns: judged by its heading alone, and says where it's facing. */
  private final boolean turning;

  /** Its keyframes' names, in order, a null for each unnamed one. */
  private final List<@Nullable String> keyframes;

  private final double deadlineSeconds;

  private @Nullable Outcome outcome;
  private String reason = "";
  private boolean running;

  /** Whether its latest run has waited a loop yet. */
  private boolean yielded;

  private long startNanos;

  /** When its latest run's deadline passes, on the robot's clock: none until it's set. */
  private long deadlineNanos = Long.MAX_VALUE;

  private boolean[] passed = new boolean[0];
  private final Map<String, Gate> gates = new HashMap<>();

  private DriveMove(
      Drive drive,
      Kind kind,
      String name,
      ChassisVelocities velocity,
      @Nullable BooleanSupplier until,
      @Nullable Planner planner,
      boolean turning,
      List<@Nullable String> keyframes,
      double deadlineSeconds) {
    this.drive = drive;
    this.kind = kind;
    this.name = name;
    this.velocity = velocity;
    this.until = until;
    this.planner = planner;
    this.turning = turning;
    this.keyframes = keyframes;
    this.deadlineSeconds = deadlineSeconds;
  }

  static DriveMove runUntil(Drive drive, ChassisVelocities velocity, BooleanSupplier until) {
    String name = "drive until" + (until instanceof Gate gate ? " " + gate.name() : "");
    return new DriveMove(
        drive,
        Kind.RUN_UNTIL,
        name,
        velocity,
        until,
        null,
        false,
        List.of(),
        Outcome.RUN_UNTIL_SECONDS);
  }

  static DriveMove turnTo(Drive drive, Rotation2d heading) {
    return new DriveMove(
        drive,
        Kind.FOLLOW,
        String.format(Locale.ROOT, "drive → %.1f°", heading.getDegrees()),
        new ChassisVelocities(),
        null,
        (start, moving, limits) ->
            Trajectories.line(
                start, moving, start.getTranslation(), heading, limits, RobotLoop.PERIOD_SECONDS),
        true,
        List.of(),
        Double.NaN);
  }

  static DriveMove goTo(Drive drive, Pose2d pose) {
    return new DriveMove(
        drive,
        Kind.FOLLOW,
        "drive → " + shown(pose),
        new ChassisVelocities(),
        null,
        (start, moving, limits) ->
            Trajectories.line(
                start,
                moving,
                pose.getTranslation(),
                pose.getRotation(),
                limits,
                RobotLoop.PERIOD_SECONDS),
        false,
        List.of(),
        Double.NaN);
  }

  /**
   * To a place, found for the robot's alliance as the move starts. Built before the drive has the
   * field to find it on, it is refused as a path to a place is ({@link Refusals}); started with the
   * alliance unknown, it ends {@link Outcome#REFUSED} as it starts.
   *
   * @throws IllegalArgumentException as the robot starts, if the drive has no field to find it on
   */
  static DriveMove goTo(Drive drive, Place place) {
    String refused =
        Refusals.asBuilt(
            drive.getRegisteredScheduler(), drive.driver() == null ? Drive.NO_FIELD : null);
    return new DriveMove(
        drive,
        Kind.FOLLOW,
        "drive → " + place.name(),
        new ChassisVelocities(),
        null,
        (start, moving, limits) -> {
          if (refused != null) {
            throw new IllegalArgumentException(refused);
          }
          Pose2d pose = drive.found(place);
          return Trajectories.line(
              start,
              moving,
              pose.getTranslation(),
              pose.getRotation(),
              limits,
              RobotLoop.PERIOD_SECONDS);
        },
        false,
        List.of(),
        Double.NaN);
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
    List<@Nullable String> names = new ArrayList<>();
    List<String> shown = new ArrayList<>();
    String why = path.isEmpty() ? "a path needs a keyframe: give one, Keyframe.at(pose)" : null;
    boolean places = false;
    for (Keyframe keyframe : path) {
      if (why == null && keyframe.name() != null && names.contains(keyframe.name())) {
        why = "two keyframes are named " + keyframe.name() + ": name each once";
      }
      names.add(keyframe.name());
      shown.add(keyframe.toString());
      places |= keyframe.place() != null;
    }
    if (why == null && places && drive.driver() == null) {
      why = Drive.NO_FIELD;
    }
    String refused = Refusals.asBuilt(drive.getRegisteredScheduler(), why);
    if (!path.isEmpty() && path.get(path.size() - 1).pose() != null) {
      Pose2d end = Objects.requireNonNull(path.get(path.size() - 1).pose());
      shown.set(
          shown.size() - 1,
          shown.get(shown.size() - 1)
              + String.format(Locale.ROOT, " %.1f°", end.getRotation().getDegrees()));
    }
    return new DriveMove(
        drive,
        Kind.FOLLOW,
        "drive → " + (path.isEmpty() ? "a path" : String.join(" → ", shown)),
        new ChassisVelocities(),
        null,
        refused != null
            ? (start, moving, limits) -> {
              throw new IllegalArgumentException(refused);
            }
            : (start, moving, limits) ->
                Trajectories.through(
                    start, moving, drive.found(path), limits, RobotLoop.PERIOD_SECONDS),
        false,
        Collections.unmodifiableList(names),
        Double.NaN);
  }

  /** The same move with another deadline. */
  public DriveMove within(Time deadline) {
    return new DriveMove(
        drive, kind, name, velocity, until, planner, turning, keyframes, deadline.in(Seconds));
  }

  /**
   * A gate that opens as this move's path passes a named keyframe, and stays open until the move
   * next starts: {@code co.waitUntil(leg.passed("pickup"))}. It's the path's time that passes it,
   * which the robot follows within centimeters. A name none of its keyframes has is refused as a
   * joint's gate on a place not its own is ({@link Refusals#gate}): thrown as the robot starts, and
   * once it runs, a gate that holds false saying why.
   *
   * @throws IllegalArgumentException for a name none of its keyframes has, as the robot starts
   */
  public Gate passed(String keyframe) {
    int index = keyframes.indexOf(keyframe);
    if (index < 0) {
      List<String> named = new ArrayList<>();
      for (String each : keyframes) {
        if (each != null) {
          named.add(each);
        }
      }
      return Refusals.gate(
          drive.getRegisteredScheduler(),
          "passed " + keyframe,
          name
              + " has no keyframe "
              + keyframe
              + ": "
              + (named.isEmpty()
                  ? "name one, Keyframe.at(...).named(\"" + keyframe + "\")"
                  : "its keyframes are " + String.join(", ", named)));
    }
    return gates.computeIfAbsent(
        keyframe,
        k ->
            Gate.of(
                drive.getRegisteredScheduler(),
                "passed " + k,
                () -> index < passed.length && passed[index]));
  }

  /** Whether its latest run arrived: its condition held, or it got where it was going. */
  public boolean arrived() {
    return outcome == Outcome.ARRIVED;
  }

  /** How its latest run ended: empty while it runs, or before it first has. */
  public Optional<Outcome> outcome() {
    return Optional.ofNullable(outcome);
  }

  /** Why its latest run ended as it did: "drive → 90.0°: ARRIVED in 0.84 s". */
  public String reason() {
    return reason;
  }

  /**
   * When its latest run's deadline passes, in nanoseconds on the robot's clock: none ({@link
   * Long#MAX_VALUE}) before its path is planned and once it has ended. For tests that check every
   * move ends by its deadline.
   */
  long deadlineNanos() {
    return running ? deadlineNanos : Long.MAX_VALUE;
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
    running = true;
    outcome = null;
    reason = "";
    passed = new boolean[keyframes.size()];
    startNanos = RobotController.getTime();
    deadlineNanos = Long.MAX_VALUE;
    yielded = false;
    switch (kind) {
      case RUN_UNTIL -> runUntil(coroutine);
      case FOLLOW -> follow(coroutine, Objects.requireNonNull(planner));
    }
    if (!yielded) {
      // Ended as it started, refused or there already: it takes its loop all the same, so a loop
      // of moves goes round once a loop, never without end in one.
      coroutine.yield();
    }
  }

  @Override
  public void onCancel() {
    if (running) {
      end(Outcome.CANCELED, null);
    }
  }

  @Override
  public String toString() {
    return name;
  }

  private void runUntil(Coroutine coroutine) {
    BooleanSupplier condition = until;
    deadlineNanos = startNanos + Math.round(deadlineSeconds * 1e9);
    while (condition != null && !condition.getAsBoolean()) {
      if (RobotController.getTime() >= deadlineNanos) {
        end(Outcome.TIMED_OUT, condition instanceof Gate gate ? gate.why() : null);
        return;
      }
      drive.runVelocity(velocity);
      yielded = true;
      coroutine.yield();
    }
    end(Outcome.ARRIVED, null);
  }

  /**
   * Its path, planned from where the robot is: each loop, the robot's speed and turning rate asked
   * for are the path's where it's due at the loop's end, pushed back toward the path's place and
   * heading, and toward its speed; and the wheels given the volts the path's acceleration takes.
   */
  private void follow(Coroutine coroutine, Planner planner) {
    DrivetrainControl control = drive.control();
    DrivetrainControl.Accelerations most = drive.accelerations();
    double speed = control.limits().maxLinearSpeed().in(MetersPerSecond);
    double rate = control.limits().maxAngularSpeed().in(RadiansPerSecond);
    Pose2d start = drive.getPose();
    Trajectories.Plan plan;
    try {
      plan =
          planner.plan(
              start,
              drive.getRobotRelativeVelocities().toFieldRelative(start.getRotation()),
              new Trajectories.Limits(
                  speed, rate, PROFILE_SHARE * most.linear(), PROFILE_SHARE * most.angular()));
    } catch (IllegalArgumentException e) {
      end(Outcome.REFUSED, e.getMessage());
      return;
    }
    HolonomicTrajectory path = plan.trajectory();
    double[] passing = plan.passing();
    HolonomicSample last = path.end();
    double deadline =
        Double.isNaN(deadlineSeconds) ? Outcome.deadline(path.duration) : deadlineSeconds;
    deadlineNanos = startNanos + Math.round(deadline * 1e9);
    long arrivedSince = -1;
    while (true) {
      Pose2d pose = drive.getPose();
      ChassisVelocities measured =
          drive.getRobotRelativeVelocities().toFieldRelative(pose.getRotation());
      double t = seconds();
      for (int k = 0; k < passing.length; k++) {
        passed[k] |= t >= passing[k];
      }
      // A turn is judged by its heading alone: turning against a wall, the robot's corners push it
      // off its spot, which it only leans back toward.
      boolean arrived =
          t >= path.duration
              && (turning
                  || pose.getTranslation().getDistance(last.pose.getTranslation()) <= PLACED_METERS)
              && Math.abs(last.pose.getRotation().minus(pose.getRotation()).getRadians())
                  <= TURNED_RADIANS
              && Math.hypot(measured.vx, measured.vy) <= STILL_METERS_PER_SECOND
              && Math.abs(measured.omega) <= STILL_RADIANS_PER_SECOND;
      long now = RobotController.getTime();
      if (!arrived) {
        arrivedSince = -1;
      } else if (arrivedSince < 0) {
        arrivedSince = now;
      } else if (now - arrivedSince >= Outcome.DWELL_NANOS) {
        end(Outcome.ARRIVED, null);
        return;
      }
      if (now >= deadlineNanos) {
        end(Outcome.TIMED_OUT, ", " + where(pose), null);
        return;
      }
      HolonomicSample due = path.sampleAt(t + RobotLoop.PERIOD_SECONDS);
      ChassisVelocities was = path.sampleAt(t).velocity;
      Translation2d planned = new Translation2d(due.velocity.vx, due.velocity.vy);
      Translation2d ask =
          planned
              .plus(
                  due.pose
                      .getTranslation()
                      .minus(pose.getTranslation())
                      .times(control.gains().translationKp()))
              .plus(planned.minus(new Translation2d(measured.vx, measured.vy)).times(RATE_DAMPING));
      if (ask.getNorm() > speed) {
        ask = ask.times(speed / ask.getNorm());
      }
      double turn =
          due.velocity.omega
              + control.gains().headingKp()
                  * due.pose.getRotation().minus(pose.getRotation()).getRadians()
              + RATE_DAMPING * (due.velocity.omega - measured.omega);
      drive.runVelocity(
          new ChassisVelocities(ask.getX(), ask.getY(), Math.clamp(turn, -rate, rate))
              .toRobotRelative(pose.getRotation()),
          new ChassisAccelerations(
                  (due.velocity.vx - was.vx) / RobotLoop.PERIOD_SECONDS,
                  (due.velocity.vy - was.vy) / RobotLoop.PERIOD_SECONDS,
                  (due.velocity.omega - was.omega) / RobotLoop.PERIOD_SECONDS)
              .toRobotRelative(pose.getRotation()));
      yielded = true;
      coroutine.yield();
    }
  }

  /** Where the robot got to, as a timed-out move says. */
  private String where(Pose2d pose) {
    return turning
        ? String.format(Locale.ROOT, "heading at %.1f°", pose.getRotation().getDegrees())
        : "at " + shown(pose).replace(") ", ") facing ");
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

  private double seconds() {
    return (RobotController.getTime() - startNanos) / 1e9;
  }

  private void end(Outcome how, @Nullable String why) {
    end(how, "", why);
  }

  /**
   * Ends its run: records how, with what it says of it ({@link Outcome#reason}), stops the
   * drivetrain, and logs the reason.
   *
   * @param details where it got to, for a path timed out; empty otherwise
   */
  private void end(Outcome how, String details, @Nullable String why) {
    running = false;
    outcome = how;
    reason = how.reason(name, seconds(), details, why);
    drive.runVelocity(new ChassisVelocities());
    Logger.recordOutput("Drive/Outcome", reason);
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.path.Keyframe;
import com.michaelgrundvig.frc.world.path.Trajectories;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * What a {@link DriveMove} does. There are two sorts: driving at a velocity until a condition holds
 * ({@link RunUntil}), and following a path that is planned as the move starts ({@link AlongAPath}),
 * which is what a turn, a move to a pose or a place, and a path through keyframes all are. Each
 * kind carries only what it needs.
 */
sealed interface DriveTask permits DriveTask.RunUntil, DriveTask.AlongAPath {
  /**
   * The names of the keyframes the move passes, in order, with an empty entry for each keyframe
   * that has no name. Only a path through keyframes has any.
   */
  default List<Optional<String>> keyframeNames() {
    return List.of();
  }

  /**
   * Drives at a velocity until a condition holds.
   *
   * @param velocity the velocity, relative to the robot
   * @param until the condition the move ends on
   */
  record RunUntil(ChassisVelocities velocity, BooleanSupplier until) implements DriveTask {
    /**
     * Why the condition does not hold, for a move that timed out waiting for it. Only a {@link
     * Gate} can say. Any other condition gives no reason.
     */
    Optional<String> whyItNeverHeld() {
      if (until instanceof Gate gate) {
        return Optional.of(gate.why());
      }
      return Optional.empty();
    }
  }

  /** What a move along a path must reach before it has arrived. */
  enum Arrival {
    /** The path's last pose: its place and its heading. */
    AT_ITS_POSE,
    /**
     * The path's last heading alone. A turn is judged this way because a robot turning against a
     * wall is pushed off its spot by its own corners, and only leans back toward it.
     */
    FACING_ITS_HEADING
  }

  /** A task that plans a timed path as the move starts, and follows it to its end. */
  sealed interface AlongAPath extends DriveTask permits TurnTo, ToPose, ToPlace, Through {
    /**
     * Plans the path, from where the robot is as the move starts.
     *
     * @param drive the drivetrain, which finds a place for the robot's alliance
     * @param start where the robot is
     * @param moving how fast it is moving, on the field's axes
     * @param limits the speeds and accelerations the path is kept under
     * @throws IllegalArgumentException saying why, if no path can be planned: a place can't be
     *     found, or no smooth path goes through the keyframes
     */
    Trajectories.Plan plan(
        Drive drive, Pose2d start, ChassisVelocities moving, Trajectories.Limits limits);

    /** What the move must reach before it has arrived. */
    default Arrival arrival() {
      return Arrival.AT_ITS_POSE;
    }
  }

  /**
   * Turns where the robot stands to a heading on the field.
   *
   * @param heading the heading to end at
   */
  record TurnTo(Rotation2d heading) implements AlongAPath {
    @Override
    public Trajectories.Plan plan(
        Drive drive, Pose2d start, ChassisVelocities moving, Trajectories.Limits limits) {
      // A line from where the robot is to where it is: only the heading changes.
      return Trajectories.line(
          start, moving, start.getTranslation(), heading, limits, RobotLoop.PERIOD_SECONDS);
    }

    @Override
    public Arrival arrival() {
      return Arrival.FACING_ITS_HEADING;
    }
  }

  /**
   * Goes to a pose on the field in a straight line, turning on the way.
   *
   * @param pose the pose to end at, in field coordinates
   */
  record ToPose(Pose2d pose) implements AlongAPath {
    @Override
    public Trajectories.Plan plan(
        Drive drive, Pose2d start, ChassisVelocities moving, Trajectories.Limits limits) {
      return lineTo(pose, start, moving, limits);
    }
  }

  /**
   * Goes to a place in a straight line, turning on the way. The place is written once for blue, and
   * where it is for the robot's alliance is found as the move starts.
   *
   * @param place the place to end at
   */
  record ToPlace(Place place) implements AlongAPath {
    @Override
    public Trajectories.Plan plan(
        Drive drive, Pose2d start, ChassisVelocities moving, Trajectories.Limits limits) {
      return lineTo(drive.found(place), start, moving, limits);
    }
  }

  /**
   * Drives a smooth path through keyframes, stopping at the last. A keyframe at a place is found
   * for the robot's alliance as the move starts.
   *
   * @param keyframes the keyframes, in the order the path passes them
   * @param names each keyframe's name, in the same order, empty for one that has none
   */
  record Through(List<Keyframe> keyframes, List<Optional<String>> names) implements AlongAPath {
    static Through of(List<Keyframe> keyframes) {
      List<Optional<String>> names = new ArrayList<>();
      for (Keyframe keyframe : keyframes) {
        names.add(Optional.ofNullable(keyframe.name()));
      }
      return new Through(List.copyOf(keyframes), List.copyOf(names));
    }

    @Override
    public Trajectories.Plan plan(
        Drive drive, Pose2d start, ChassisVelocities moving, Trajectories.Limits limits) {
      return Trajectories.through(
          start, moving, drive.found(keyframes), limits, RobotLoop.PERIOD_SECONDS);
    }

    @Override
    public List<Optional<String>> keyframeNames() {
      return names;
    }
  }

  /** A straight line from where the robot is to a pose, turning on the way. */
  private static Trajectories.Plan lineTo(
      Pose2d end, Pose2d start, ChassisVelocities moving, Trajectories.Limits limits) {
    return Trajectories.line(
        start, moving, end.getTranslation(), end.getRotation(), limits, RobotLoop.PERIOD_SECONDS);
  }
}

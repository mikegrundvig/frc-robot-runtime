package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.behavior.Gate;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.units.Measure;

/**
 * What a {@link Move} is, apart from how its latest run went. A plan never changes: each of a
 * move's options ({@link Move#atSpeed}, {@link Move#within}, {@link Move#onlyIf}, {@link
 * Move#onlyWhile}) makes a new move from a new plan.
 *
 * @param task what the move does
 * @param name the move's name, as the log and its outcome say it: "pivot → out"
 * @param cruiseShare the share of each joint's cruise speed a move to a place is profiled at, above
 *     0 and at most 1
 * @param within the deadline a team gave the move, in seconds, or empty for the model's: one worked
 *     out from its profile for a move to a place, and none for any other. One that was given is a
 *     time above zero: {@link Move#within} refuses any other.
 * @param onlyIf a gate that must hold for the move to start, if it has one
 * @param onlyWhile a gate that must hold for the move to start and to carry on, if it has one
 * @param refused why the move cannot run at all, if it cannot: it then ends refused as it starts
 */
record MovePlan(
    Task task,
    String name,
    double cruiseShare,
    OptionalDouble within,
    Optional<Gate> onlyIf,
    Optional<Gate> onlyWhile,
    Optional<String> refused) {

  /** What a move does. */
  sealed interface Task {
    /**
     * Whether the move ends by itself, and so may be given a deadline ({@link Move#within}). One
     * that does not runs until it is canceled.
     */
    default boolean endsByItself() {
      return false;
    }
  }

  /** Goes to its targets, and ends when every joint is there. */
  record GoTo() implements Task {
    @Override
    public boolean endsByItself() {
      return true;
    }
  }

  /**
   * Stays at its targets, each joint at its output or its position, until it is canceled or a
   * condition holds.
   *
   * @param until the condition the move ends on, or empty for a move that runs until canceled
   */
  record Run(Optional<BooleanSupplier> until) implements Task {
    @Override
    public boolean endsByItself() {
      return until.isPresent();
    }
  }

  /**
   * Chases a target that may change every loop, until it is canceled.
   *
   * @param target gives the target each loop: a position or a speed
   */
  record Follow(Supplier<? extends Measure<?>> target) implements Task {}

  /** Holds the joint at the goal it has as the move starts, until it is canceled. */
  record Hold() implements Task {}

  /** Homes the joint again. How long that may take is the homing's business, not the move's. */
  record Home() implements Task {}

  /** A plan with no options: at the full cruise, the model's deadline, and no gates. */
  static MovePlan of(Task task, String name, Optional<String> refused) {
    return new MovePlan(
        task, name, 1, OptionalDouble.empty(), Optional.empty(), Optional.empty(), refused);
  }

  /** The same plan at a share of each joint's cruise speed, under a name that says so. */
  MovePlan atCruiseShare(double share, String name) {
    return new MovePlan(task, name, share, within, onlyIf, onlyWhile, refused);
  }

  /** The same plan with a deadline of its own, in seconds. */
  MovePlan within(double seconds) {
    return new MovePlan(
        task, name, cruiseShare, OptionalDouble.of(seconds), onlyIf, onlyWhile, refused);
  }

  /** The same plan, started only if a gate holds. */
  MovePlan onlyIf(Gate gate) {
    return new MovePlan(task, name, cruiseShare, within, Optional.of(gate), onlyWhile, refused);
  }

  /** The same plan, started and carried on only while a gate holds. */
  MovePlan onlyWhile(Gate gate) {
    return new MovePlan(task, name, cruiseShare, within, onlyIf, Optional.of(gate), refused);
  }

  /**
   * The same plan, refused. A plan that was refused already keeps its first reason, since that is
   * the mistake to fix first.
   */
  MovePlan refusedFor(String why) {
    if (refused.isPresent()) {
      return this;
    }
    return new MovePlan(task, name, cruiseShare, within, onlyIf, onlyWhile, Optional.of(why));
  }
}

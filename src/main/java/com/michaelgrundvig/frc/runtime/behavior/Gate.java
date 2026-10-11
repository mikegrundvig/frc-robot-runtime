package com.michaelgrundvig.frc.runtime.behavior;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Seconds;

import com.google.errorprone.annotations.CheckReturnValue;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Time;

/**
 * A named condition that says why it is false. It is a v3 {@link Trigger}, so it binds ({@code
 * operator.rightBumper().and(readyToShoot).onTrue(...)}) and can be waited on ({@code
 * co.waitUntil(readyToShoot)}), and it adds a name and named terms. Every joint's checks are gates
 * already ({@code shooter.isAtGoal()} is "shooter at goal"), and a team's code makes its own:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=gates}
 *
 * <p><b>The log.</b> Each time a gate is worked out it is logged: {@code Gates/ready to shoot =
 * false}, and {@code Gates/ready to shoot/Blocking = [aimed]}, the terms that are false. Terms go
 * one level deep. A gate is made of other gates, each one a lambda, and there is no expression
 * language.
 *
 * <p><b>When it is worked out.</b> A gate made as the robot starts is worked out once a loop from
 * then on, before the scheduler's bindings and commands read it. A gate made once the robot runs
 * (inside a command, or as an auto is built) is worked out each time it is read instead. A gate
 * that is waited on or bound is read every loop, so the two come to the same thing. The difference
 * is that the second kind costs nothing once nobody reads it, so a command can make a new gate on
 * every run without leaving anything behind.
 *
 * <p><b>A logic that throws.</b> The gate holds false while its logic throws, and its one term says
 * so ({@code aimed threw: ...}). The rest of the robot runs on. The first time, the gate reports a
 * Driver Station error with the stack, and an alert stays raised for as long as the logic throws.
 * Gates of the same name share that one alert, so a command that makes its gate on every run can
 * make one that throws every time.
 *
 * <p>What a gate has worked out, and the working out, is its {@link GateState}.
 */
@CheckReturnValue
public final class Gate extends Trigger {
  /** What a gate works out. */
  public interface Logic {
    /**
     * Whether the gate holds now.
     *
     * @param why where to add the name of each term that keeps the gate false
     */
    boolean holds(List<String> why);
  }

  /** What a gate with a band remembers between workings out: whether it held the last time. */
  private static final class Remembered {
    boolean held;
  }

  private final Scheduler scheduler;
  private final String name;
  private final GateState state;

  /**
   * A gate whose logic names what keeps it false. A mechanism's or a sensor's gate is built from
   * this.
   */
  public Gate(Scheduler scheduler, String name, Logic logic) {
    this(scheduler, name, new GateState(name, logic));
  }

  private Gate(Scheduler scheduler, String name, GateState state) {
    super(scheduler, state);
    this.scheduler = scheduler;
    this.name = name;
    this.state = state;
    state.workOut();
    if (Refusals.starting(scheduler)) {
      scheduler.addPeriodic(state::workOut);
      return;
    }
    // The scheduler keeps every periodic task for good. So a gate made once the robot runs is not
    // given one: it is worked out as it is read, and nothing of it is left running when the
    // command that made it is over. For the same reason its trigger is unbound here: a v3 trigger
    // polls from the moment it is made, and an unbound one starts polling again only when a
    // command is bound to it.
    state.workOutWhenRead();
    unbind();
  }

  /** A gate on any condition: {@code Gate.of(scheduler, "in range", () -> distance() <= 5)}. */
  public static Gate of(Scheduler scheduler, String name, BooleanSupplier condition) {
    return new Gate(scheduler, name, why -> condition.getAsBoolean());
  }

  /**
   * A gate on how far something is off, in either direction: {@code Gate.within(scheduler, "aimed",
   * () -> headingError, 2.0, 4.0)}. It becomes true once the error is within {@code enter} of zero,
   * and then stays true until the error is more than {@code stay} off. With one band, an error that
   * wavers at the edge would make the gate flicker.
   *
   * <p>Bands that cannot be are refused ({@link Refusals#gate}): as the robot starts that is a
   * throw, and once it runs it is a gate that holds false and says why.
   *
   * @throws IllegalArgumentException as the robot starts, for an {@code enter} that is not
   *     positive, or a {@code stay} below it
   */
  public static Gate within(
      Scheduler scheduler, String name, DoubleSupplier error, double enter, double stay) {
    boolean bandsCanBe = enter > 0 && stay >= enter;
    if (!bandsCanBe) {
      return Refusals.gate(
          scheduler,
          name,
          name
              + ": enter within a positive band, and stay within at least it, not "
              + enter
              + " and "
              + stay);
    }
    Remembered inside = new Remembered();
    return new Gate(
        scheduler,
        name,
        why -> {
          double off = Math.abs(error.getAsDouble());
          double band = inside.held ? stay : enter;
          inside.held = off <= band;
          return inside.held;
        });
  }

  /**
   * A gate that is true when all its terms are, and is blocked by the ones that are not: {@code
   * Gate.all("ready to shoot", shooter.isAtGoal(), aimed)}.
   *
   * @throws IllegalArgumentException with no terms, or with terms of different schedulers
   */
  public static Gate all(String name, Gate... terms) {
    if (terms.length == 0) {
      throw new IllegalArgumentException(name + " has no terms");
    }
    Scheduler scheduler = terms[0].scheduler;
    for (Gate term : terms) {
      if (term.scheduler != scheduler) {
        throw new IllegalArgumentException(possessive(name) + " terms run on different schedulers");
      }
    }
    List<Gate> all = List.of(terms);
    return new Gate(
        scheduler,
        name,
        why -> {
          boolean holds = true;
          // Every term is asked, even after one is false, so that all the false ones are named.
          for (Gate term : all) {
            if (!term.state.getAsBoolean()) {
              why.add(term.name);
              holds = false;
            }
          }
          return holds;
        });
  }

  /**
   * The same gate, debounced. On the falling edge it stays true until this gate has been false for
   * the time, which rides out a flywheel's dip as a shot leaves. On the rising edge it becomes true
   * only once this gate has been true for the time.
   *
   * <p>The debounced gate is logged under the same name, in place of this one. It holds nothing
   * until this gate has first held.
   */
  @Override
  public Gate debounce(Time time, Debouncer.DebounceType type) {
    Debouncer debouncer = new Debouncer(time.in(Seconds), type);
    GateState undebounced = state;
    undebounced.stopLogging();
    Remembered everHeld = new Remembered();
    return new Gate(
        scheduler,
        name,
        why -> {
          boolean holdsNow = undebounced.getAsBoolean();
          everHeld.held |= holdsNow;
          // The debouncer is asked every loop, whatever the answer, so that its timer keeps time.
          boolean debounced = debouncer.calculate(holdsNow);
          if (everHeld.held && debounced) {
            return true;
          }
          if (holdsNow) {
            why.add(name + " settling");
          } else {
            why.addAll(undebounced.blocking());
          }
          return false;
        });
  }

  /**
   * A trigger that is true while this gate and a condition both hold. When the condition is a
   * gamepad's button the result is the button's own kind of trigger, which records what is bound to
   * it: {@code readyToShoot.and(operator.rightBumper())} is "right bumper and ready to shoot", the
   * same as when it is written the other way round.
   */
  @Override
  public Trigger and(BooleanSupplier condition) {
    if (condition instanceof RecordingTrigger button) {
      return button.and(this);
    }
    return super.and(condition);
  }

  /**
   * A trigger that is true while this gate or a condition holds. With a gamepad's button it is the
   * button's own kind of trigger, as for {@link #and}.
   */
  @Override
  public Trigger or(BooleanSupplier condition) {
    if (condition instanceof RecordingTrigger button) {
      return button.or(this);
    }
    return super.or(condition);
  }

  /** Its name, as logged: "ready to shoot". */
  public String name() {
    return name;
  }

  /** Whether it holds this loop. */
  @Override
  public boolean getAsBoolean() {
    return state.getAsBoolean();
  }

  /**
   * The terms that keep it false this loop. Empty while it holds, and always empty for a gate
   * without terms.
   */
  public List<String> blocking() {
    state.bringUpToDate();
    return List.copyOf(state.blocking());
  }

  /**
   * Why it is false, as a reason reads: "ready to shoot is false: aimed", or just "ready to shoot
   * is false" for a gate without terms.
   */
  public String why() {
    state.bringUpToDate();
    List<String> blocking = state.blocking();
    if (blocking.isEmpty()) {
      return name + " is false";
    }
    return name + " is false: " + String.join(", ", blocking);
  }

  @Override
  public String toString() {
    return name;
  }
}

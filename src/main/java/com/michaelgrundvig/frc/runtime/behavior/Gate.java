package com.michaelgrundvig.frc.runtime.behavior;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Seconds;

import com.google.errorprone.annotations.CheckReturnValue;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Alert;

/**
 * A named condition that says why it's false: a v3 {@link Trigger}, so it binds ({@code
 * operator.rightBumper().and(readyToShoot).onTrue(...)}) and awaits ({@code
 * co.waitUntil(readyToShoot)}), with a name and named terms. Every joint's checks are gates already
 * ({@code shooter.isAtGoal()} is "shooter at goal"), and code makes its own:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=gates}
 *
 * <p>It's worked out once a loop, before the scheduler's bindings and commands read it, and logged
 * then: {@code Gates/ready to shoot = false}, and {@code Gates/ready to shoot/Blocking = [aimed]},
 * the terms that are false. Terms go one level deep: a gate of gates, each a lambda, never an
 * expression language.
 *
 * <p>A gate made as the robot starts is worked out and logged every loop from then on. One made
 * once the robot runs (inside a command, or as an auto is built) is worked out and logged when it's
 * read instead: waited on or bound, that's every loop all the same, and once nothing reads it any
 * more it costs nothing, so making one per run of a command leaves nothing behind.
 *
 * <p>A gate whose logic throws holds false while it does, its term saying so ({@code aimed threw:
 * ...}), and the rest of the robot runs on. The first time, it reports a Driver Station error with
 * the stack, and an alert stays raised while it throws.
 */
@CheckReturnValue
public final class Gate extends Trigger {
  /** What a gate works out each loop. */
  public interface Logic {
    /** Whether it holds now; when not, the names of the terms that don't, added to {@code why}. */
    boolean holds(List<String> why);
  }

  /** The gate's value this loop, which v3's trigger reads. */
  private static final class Value implements BooleanSupplier {
    boolean holds;

    /** The gate to work out as this is read, one made once the robot runs; else null. */
    @Nullable Gate asked;

    @Override
    public boolean getAsBoolean() {
      Gate gate = asked;
      if (gate != null) {
        gate.update();
      }
      return holds;
    }
  }

  private final Scheduler scheduler;
  private final String name;
  private final Logic logic;
  private final Value value;
  private final List<String> blocking = new ArrayList<>();
  private final String key;
  private final String blockingKey;

  /** Whether it logs: a gate debounced into another logs as that one, under the same name. */
  private boolean logged = true;

  private String[] shown = new String[0];

  /** Raised while its logic throws: made the first time it does. */
  private @Nullable Alert threw;

  /** What the alert says. */
  private String threwText = "";

  /** Whether its logic threw last loop. */
  private boolean throwing;

  /**
   * A gate whose logic names what keeps it false: what a mechanism's or a sensor's gate is built
   * from.
   */
  public Gate(Scheduler scheduler, String name, Logic logic) {
    this(scheduler, name, logic, new Value());
  }

  private Gate(Scheduler scheduler, String name, Logic logic, Value value) {
    super(scheduler, value);
    this.scheduler = scheduler;
    this.name = name;
    this.logic = logic;
    this.value = value;
    key = "Gates/" + name;
    blockingKey = key + "/Blocking";
    update();
    if (Refusals.starting(scheduler)) {
      scheduler.addPeriodic(this::update);
    } else {
      // The scheduler keeps every periodic for good, so one made once the robot runs works itself
      // out as it's read: its command's run over, nothing of it is left running. A trigger polls
      // from the moment it's made, too; unbound, it polls again once a command is bound to it.
      value.asked = this;
      unbind();
    }
  }

  /** A gate on any condition: {@code Gate.of(scheduler, "in range", () -> distance() <= 5)}. */
  public static Gate of(Scheduler scheduler, String name, BooleanSupplier condition) {
    return new Gate(scheduler, name, why -> condition.getAsBoolean());
  }

  /**
   * A gate on how far something is off, either way: it becomes true within {@code enter} of zero,
   * and stays true until it's more than {@code stay} off, so an error that wavers at the edge
   * doesn't flicker. {@code Gate.within(scheduler, "aimed", () -> headingError, 2.0, 4.0)}. Bands
   * that can't be are refused ({@link Refusals#gate}): thrown as the robot starts, and once it
   * runs, a gate that holds false saying why.
   *
   * @throws IllegalArgumentException for an {@code enter} that isn't positive, or a {@code stay}
   *     below it, as the robot starts
   */
  public static Gate within(
      Scheduler scheduler, String name, DoubleSupplier error, double enter, double stay) {
    if (!(enter > 0 && stay >= enter)) {
      return Refusals.gate(
          scheduler,
          name,
          name
              + ": enter within a positive band, and stay within at least it, not "
              + enter
              + " and "
              + stay);
    }
    boolean[] inside = {false};
    return new Gate(
        scheduler,
        name,
        why -> {
          double off = Math.abs(error.getAsDouble());
          inside[0] = inside[0] ? off <= stay : off <= enter;
          return inside[0];
        });
  }

  /**
   * A gate true when all its terms are, blocked by the ones that aren't: {@code Gate.all("ready to
   * shoot", shooter.isAtGoal(), aimed)}.
   *
   * @throws IllegalArgumentException with no terms, or terms of different schedulers
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
          for (Gate term : all) {
            if (!term.value.getAsBoolean()) {
              why.add(term.name);
              holds = false;
            }
          }
          return holds;
        });
  }

  /**
   * The same gate, debounced: on the falling edge, it stays true until it has been false for the
   * time, riding out a flywheel's dip as a shot leaves; on the rising edge, it becomes true only
   * once it has been for the time. It's logged under the same name, in place of this one. It holds
   * nothing until this gate first has.
   */
  @Override
  public Gate debounce(Time time, Debouncer.DebounceType type) {
    Debouncer debouncer = new Debouncer(time.in(Seconds), type);
    Gate inner = this;
    inner.logged = false;
    boolean[] seen = {false};
    return new Gate(
        scheduler,
        name,
        why -> {
          boolean now = inner.value.getAsBoolean();
          seen[0] |= now;
          boolean debounced = debouncer.calculate(now); // every loop, so its timer keeps time
          boolean holds = seen[0] && debounced;
          if (!holds) {
            if (now) {
              why.add(name + " settling");
            } else {
              why.addAll(inner.blocking);
            }
          }
          return holds;
        });
  }

  /**
   * A trigger true while this gate and a condition both hold. With a gamepad's button, it's the
   * button's, recording what's bound to it: {@code readyToShoot.and(operator.rightBumper())} is
   * "right bumper and ready to shoot", as the other way round.
   */
  @Override
  public Trigger and(BooleanSupplier condition) {
    return condition instanceof RecordingTrigger button ? button.and(this) : super.and(condition);
  }

  /**
   * A trigger true while this gate or a condition holds: with a gamepad's button, the button's, as
   * {@link #and} is.
   */
  @Override
  public Trigger or(BooleanSupplier condition) {
    return condition instanceof RecordingTrigger button ? button.or(this) : super.or(condition);
  }

  /** Its name, as logged: "ready to shoot". */
  public String name() {
    return name;
  }

  /** Whether it holds this loop. */
  @Override
  public boolean getAsBoolean() {
    return value.getAsBoolean();
  }

  /** The terms that keep it false this loop: empty while it holds, or for a gate without terms. */
  public List<String> blocking() {
    asked();
    return List.copyOf(blocking);
  }

  /** Why it's false, as a reason reads: "ready to shoot is false: aimed"; or "ready to shoot". */
  public String why() {
    asked();
    return blocking.isEmpty()
        ? name + " is false"
        : name + " is false: " + String.join(", ", blocking);
  }

  @Override
  public String toString() {
    return name;
  }

  /** Works it out now, if it's one worked out as it's read. */
  private void asked() {
    if (value.asked != null) {
      update();
    }
  }

  /** Works it out for this loop, and logs it. */
  private void update() {
    blocking.clear();
    boolean throwsNow = false;
    try {
      value.holds = logic.holds(blocking);
    } catch (RuntimeException e) {
      // Caught here, inside the scheduler's sideload: a sideload that throws is over, and the
      // scheduler would then throw on every loop after it.
      value.holds = false;
      blocking.clear();
      blocking.add(name + " threw: " + message(e));
      report(e);
      throwsNow = true;
    }
    Alert alert = threw;
    if (alert != null && throwsNow != throwing) {
      alert.set(throwsNow);
    }
    throwing = throwsNow;
    if (logged) {
      Logger.recordOutput(key, value.holds);
      if (shown.length != blocking.size() || !List.of(shown).equals(blocking)) {
        shown = blocking.toArray(String[]::new);
      }
      Logger.recordOutput(blockingKey, shown);
    }
  }

  /**
   * Reports its logic throwing: the first time, a Driver Station error with the stack; and the
   * alert, saying what it threw last.
   */
  private void report(RuntimeException e) {
    String text =
        blocking.get(0)
            + ". It holds false while it throws: fix its condition (the stack is in the"
            + " Driver Station's log)";
    Alert alert = threw;
    if (alert == null) {
      DriverStationErrors.reportError(text, e.getStackTrace());
      threw = new Alert("Gates", name, text, Alert.Level.HIGH);
    } else if (!text.equals(threwText)) {
      alert.setText(text);
    }
    threwText = text;
  }

  /** What it says, or what it is when it says nothing. */
  private static String message(RuntimeException e) {
    String message = e.getMessage();
    return message == null || message.isBlank() ? e.getClass().getSimpleName() : message;
  }
}

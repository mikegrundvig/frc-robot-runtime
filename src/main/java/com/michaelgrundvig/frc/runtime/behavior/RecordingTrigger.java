package com.michaelgrundvig.frc.runtime.behavior;

import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.event.EventLoop;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Time;

/**
 * A v3 {@link Trigger} on a drive team's gamepad that records each command bound to it ({@link
 * Bindings}). A gamepad's buttons are these ({@link RecordingGamepad}), so a binding is written as
 * on WPILib's gamepad, {@code operator.faceDown().onTrue(...)}, and is recorded with the button's
 * name.
 *
 * <p>A trigger made from one of these is one of these too, named for what it reads. {@code
 * operator.rightBumper().and(readyToShoot)} is "right bumper and ready to shoot": a gate goes by
 * its own name, and any other condition is "an unnamed condition".
 *
 * <p>Each press and each release it sees logs the bindings that start on it.
 */
public final class RecordingTrigger extends Trigger {
  /**
   * The trigger's condition as v3 polls it, once a loop. It watches for the condition changing, and
   * on each press and each release it logs the bindings that start then.
   */
  private static final class Presses implements BooleanSupplier {
    private final BooleanSupplier condition;
    private final Bindings bindings;

    /** What is bound to the trigger, as recorded. */
    private final List<Binding> bound = new ArrayList<>();

    private boolean hasBeenPolled;
    private boolean wasPressed;

    Presses(BooleanSupplier condition, Bindings bindings) {
      this.condition = condition;
      this.bindings = bindings;
    }

    @Override
    public boolean getAsBoolean() {
      boolean pressed = condition.getAsBoolean();
      // v3 takes a trigger's first poll as a change whichever way it reads, so a button that is
      // held as the robot starts does start what is bound to its press. The log says the same.
      boolean changed = !hasBeenPolled || pressed != wasPressed;
      if (changed) {
        logWhatStarts(pressed ? Binding.Edge.PRESS : Binding.Edge.RELEASE);
      }
      hasBeenPolled = true;
      wasPressed = pressed;
      return pressed;
    }

    private void logWhatStarts(Binding.Edge edge) {
      for (Binding binding : bound) {
        if (binding.how().startsOn() == edge) {
          bindings.fired(binding);
        }
      }
    }
  }

  /** How many things a trigger's name says it reads. */
  private enum Terms {
    /** One: a button, "right bumper". */
    ONE,
    /**
     * Several, joined: "right bumper and ready to shoot". Such a name is bracketed when it becomes
     * a term of a longer one, so that the longer name reads only one way.
     */
    SEVERAL
  }

  private final Scheduler scheduler;
  private final EventLoop loop;
  private final String gamepad;
  private final String button;
  private final Terms terms;
  private final Presses presses;
  private final Bindings bindings;

  /**
   * @param gamepad the gamepad's name: "driver"
   * @param button what it reads on that gamepad: "right trigger"
   * @param condition whether it is pressed
   */
  RecordingTrigger(
      Scheduler scheduler,
      EventLoop loop,
      String gamepad,
      String button,
      BooleanSupplier condition,
      Bindings bindings) {
    this(scheduler, loop, gamepad, button, Terms.ONE, new Presses(condition, bindings), bindings);
  }

  private RecordingTrigger(
      Scheduler scheduler,
      EventLoop loop,
      String gamepad,
      String button,
      Terms terms,
      Presses presses,
      Bindings bindings) {
    super(scheduler, loop, presses);
    this.scheduler = scheduler;
    this.loop = loop;
    this.gamepad = gamepad;
    this.button = button;
    this.terms = terms;
    this.presses = presses;
    this.bindings = bindings;
  }

  /** As its bindings name it: "operator right trigger". */
  @Override
  public String toString() {
    return gamepad + " " + button;
  }

  @Override
  public RecordingTrigger onTrue(Command command) {
    record(Binding.How.ON_PRESS, command);
    super.onTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger onFalse(Command command) {
    record(Binding.How.ON_RELEASE, command);
    super.onFalse(command);
    return this;
  }

  @Override
  public RecordingTrigger whileTrue(Command command) {
    record(Binding.How.WHILE_HELD, command);
    super.whileTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger whileFalse(Command command) {
    record(Binding.How.WHILE_RELEASED, command);
    super.whileFalse(command);
    return this;
  }

  @Override
  public RecordingTrigger toggleOnTrue(Command command) {
    record(Binding.How.TOGGLE_ON_PRESS, command);
    super.toggleOnTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger toggleOnFalse(Command command) {
    record(Binding.How.TOGGLE_ON_RELEASE, command);
    super.toggleOnFalse(command);
    return this;
  }

  @Override
  public RecordingTrigger retryWhileTrue(Command command) {
    record(Binding.How.RETRY_WHILE_HELD, command);
    super.retryWhileTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger retryWhileFalse(Command command) {
    record(Binding.How.RETRY_WHILE_RELEASED, command);
    super.retryWhileFalse(command);
    return this;
  }

  @Override
  public RecordingTrigger and(BooleanSupplier other) {
    return made(super.and(other), operand() + " and " + nameOf(other));
  }

  @Override
  public RecordingTrigger or(BooleanSupplier other) {
    return made(super.or(other), operand() + " or " + nameOf(other));
  }

  @Override
  public RecordingTrigger negate() {
    return made(super.negate(), "not " + operand());
  }

  /**
   * This trigger debounced, named for how. "right trigger for 0.1 s" becomes true once the button
   * has been held that long. "face down, and 0.1 s after" stays true until the button has been let
   * go for that long. "face down debounced 0.1 s" does both.
   *
   * <p>The debounced trigger is made here, as v3 makes its own, but with a debounce that keeps its
   * time in whole nanoseconds ({@link Debounce}): a time that is a whole number of loops ends on
   * the same loop every run.
   */
  @Override
  public RecordingTrigger debounce(Time duration, Debouncer.DebounceType type) {
    double seconds = duration.in(Seconds);
    String name =
        switch (type) {
          case RISING -> operand() + " for " + seconds + " s";
          case FALLING -> operand() + ", and " + seconds + " s after";
          case BOTH -> operand() + " debounced " + seconds + " s";
        };
    Debounce debounce = new Debounce(seconds, type);
    Trigger debounced = new Trigger(scheduler, loop, () -> debounce.calculate(getAsBoolean()));
    return made(debounced, name);
  }

  @Override
  public RecordingTrigger risingEdge() {
    return made(super.risingEdge(), operand() + " rising");
  }

  @Override
  public RecordingTrigger fallingEdge() {
    return made(super.fallingEdge(), operand() + " falling");
  }

  @Override
  public RecordingTrigger multiPress(int pressCount, Time duration) {
    return made(
        super.multiPress(pressCount, duration),
        operand() + " pressed " + pressCount + " times in " + duration.in(Seconds) + " s");
  }

  /**
   * A recording trigger made from this one, named for what it reads. It holds whatever the v3
   * trigger made from this one holds. v3 made that trigger first, so each loop it has been polled
   * just before this one reads it.
   */
  private RecordingTrigger made(Trigger made, String name) {
    Presses madePresses = new Presses(made, bindings);
    return new RecordingTrigger(
        scheduler, loop, gamepad, name, Terms.SEVERAL, madePresses, bindings);
  }

  /** Its name as a term of another trigger's name. */
  private String operand() {
    if (terms == Terms.SEVERAL) {
      return "(" + button + ")";
    }
    return button;
  }

  /**
   * The name of a condition this trigger is joined with. Another button goes by its own name, with
   * its gamepad's name in front if it is on another gamepad. A gate goes by its name. Anything else
   * has no name to give.
   */
  private String nameOf(BooleanSupplier other) {
    if (other instanceof RecordingTrigger trigger) {
      String name = trigger.operand();
      boolean isOnThisGamepad = trigger.gamepad.equals(gamepad);
      return isOnThisGamepad ? name : trigger.gamepad + " " + name;
    }
    if (other instanceof Gate gate) {
      return gate.name();
    }
    return "an unnamed condition";
  }

  /**
   * Records a binding as it is made. A binding made inside a running command is not recorded: it
   * lives only as long as that command runs.
   */
  private void record(Binding.How how, Command command) {
    boolean isMadeInsideACommand = scheduler.currentCommand() != null;
    if (isMadeInsideACommand) {
      return;
    }
    Binding binding = new Binding(gamepad, button, how, command.name());
    presses.bound.add(binding);
    bindings.add(binding);
  }
}

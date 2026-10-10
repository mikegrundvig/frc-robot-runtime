package com.michaelgrundvig.frc.runtime.behavior;

import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.event.EventLoop;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Time;

/**
 * A v3 {@link Trigger} on a drive team's gamepad that records each command bound to it ({@link
 * Bindings}): its gamepad's buttons are these ({@link RecordingGamepad}), so a binding is written
 * as on WPILib's, {@code operator.faceDown().onTrue(...)}, and recorded with the button's name. One
 * made from it is one too, named for what it reads: {@code
 * operator.rightBumper().and(readyToShoot)} is "right bumper and ready to shoot", a gate saying its
 * own name, any other condition "an unnamed condition".
 *
 * <p>Each press and release it sees logs the bindings it starts.
 */
public final class RecordingTrigger extends Trigger {
  /** The trigger's condition as v3 polls it, once a loop: it sees each press and release. */
  private static final class Signal implements BooleanSupplier {
    private final BooleanSupplier condition;
    private @Nullable RecordingTrigger trigger;
    private boolean polled;
    private boolean high;

    Signal(BooleanSupplier condition) {
      this.condition = condition;
    }

    @Override
    public boolean getAsBoolean() {
      boolean now = condition.getAsBoolean();
      RecordingTrigger owner = trigger;
      // v3 takes the first poll as a change either way, so a button held as the robot starts
      // starts what's bound to its press.
      if (owner != null && (!polled || now != high)) {
        owner.changed(now);
      }
      polled = true;
      high = now;
      return now;
    }
  }

  private final Scheduler scheduler;
  private final EventLoop loop;
  private final String gamepad;
  private final String button;

  /** Whether its name joins others' ("right bumper and ready to shoot"): it's bracketed in more. */
  private final boolean compound;

  private final Bindings bindings;

  /** What's bound to it, as recorded. */
  private final List<Binding> bound = new ArrayList<>();

  /**
   * @param gamepad the gamepad's name: "driver"
   * @param button what it reads on that gamepad: "right trigger"
   * @param condition whether it's pressed
   */
  RecordingTrigger(
      Scheduler scheduler,
      EventLoop loop,
      String gamepad,
      String button,
      BooleanSupplier condition,
      Bindings bindings) {
    this(scheduler, loop, gamepad, button, false, new Signal(condition), bindings);
  }

  private RecordingTrigger(
      Scheduler scheduler,
      EventLoop loop,
      String gamepad,
      String button,
      boolean compound,
      Signal signal,
      Bindings bindings) {
    super(scheduler, loop, signal);
    this.scheduler = scheduler;
    this.loop = loop;
    this.gamepad = gamepad;
    this.button = button;
    this.compound = compound;
    this.bindings = bindings;
    signal.trigger = this;
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
   * Debounced, named for how: "right trigger for 0.1 s" true once it's held that long, "face down,
   * and 0.1 s after" until it's been let go that long, or "face down debounced 0.1 s" both.
   */
  @Override
  public RecordingTrigger debounce(Time duration, Debouncer.DebounceType type) {
    double seconds = duration.in(Seconds);
    return made(
        super.debounce(duration, type),
        switch (type) {
          case RISING -> operand() + " for " + seconds + " s";
          case FALLING -> operand() + ", and " + seconds + " s after";
          case BOTH -> operand() + " debounced " + seconds + " s";
        });
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
   * One made from this, named for what it reads: it holds what v3's trigger made from this does,
   * read each loop just after v3 polls that one, since that one was made first.
   */
  private RecordingTrigger made(Trigger made, String name) {
    return new RecordingTrigger(scheduler, loop, gamepad, name, true, new Signal(made), bindings);
  }

  /** Its name as a term of another's, bracketed if it joins others' already. */
  private String operand() {
    return compound ? "(" + button + ")" : button;
  }

  /** A term's name: another button's, on another gamepad with its name; a gate's; or none. */
  private String nameOf(BooleanSupplier other) {
    if (other instanceof RecordingTrigger trigger) {
      String name = trigger.operand();
      return trigger.gamepad.equals(gamepad) ? name : trigger.gamepad + " " + name;
    }
    if (other instanceof Gate gate) {
      return gate.name();
    }
    return "an unnamed condition";
  }

  /**
   * Records a binding as it's made, unless it's made inside a running command, which it lives only
   * as long as.
   */
  private void record(Binding.How how, Command command) {
    if (scheduler.currentCommand() != null) {
      return;
    }
    Binding binding = new Binding(gamepad, button, how, command.name());
    bound.add(binding);
    bindings.add(binding);
  }

  /** A press or a release: logs the bindings it starts. */
  private void changed(boolean pressed) {
    for (Binding binding : bound) {
      if (binding.how().startsOnPress() == pressed) {
        bindings.fired(binding);
      }
    }
  }
}

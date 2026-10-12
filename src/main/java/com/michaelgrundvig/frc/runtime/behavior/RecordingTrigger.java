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
 * Bindings}). A gamepad's buttons are these ({@link RecordingGamepad}), so a binding is written as
 * on WPILib's gamepad, {@code operator.faceDown().onTrue(...)}, and is recorded with the button's
 * name.
 *
 * <p>A trigger made from one of these is one of these too, named for what it reads. {@code
 * operator.rightBumper().and(readyToShoot)} is "right bumper and ready to shoot": a gate goes by
 * its own name, and any other condition is "an unnamed condition".
 *
 * <p>Each press and each release that starts a binding logs the bindings it starts.
 *
 * <p><b>While the robot is disabled, a binding starts nothing.</b> A press made while the robot is
 * disabled is spent: it starts nothing then, and nothing as the robot is enabled with the button
 * still down. So is a press held from before the robot was disabled. The next press made while the
 * robot is enabled starts what is bound to it, as usual. A release is treated the same way. The one
 * exception is a binding made to start while disabled too ({@link #onTrueAlsoWhileDisabled}), for a
 * command that moves nothing.
 *
 * <p><b>Inside</b>, v3 starts a trigger's bindings as the trigger's condition changes, and has no
 * way to be told not to. So what v3 polls is not the button but one of its changes ({@link
 * Starts}), which holds only while the button may start something. The bindings that start on a
 * press are bound to this trigger, which polls the press. Those that start on a release are bound
 * to a v3 trigger of their own, which polls the release, and so are those that start while disabled
 * too.
 */
public final class RecordingTrigger extends Trigger {
  /** Whether a change of a trigger may start what is bound to it while the robot is disabled. */
  private enum WhileDisabled {
    /** Nothing starts, and a change made while disabled is spent. Every binding but one. */
    STARTS_NOTHING,
    /** It starts what is bound, as when enabled: for a command that moves nothing. */
    STARTS_ITS_BINDINGS
  }

  /** Whether a trigger that is at a change's level may start what is bound to that change. */
  private enum Change {
    /** It came to the level while the robot was enabled, or has yet to: it starts its bindings. */
    FRESH,
    /**
     * It was at the level while the robot was disabled. It starts nothing until it has left the
     * level and come back with the robot enabled.
     */
    SPENT
  }

  /**
   * One of a trigger's two changes, a press or a release, as v3 polls it once a loop. It holds
   * while the trigger is at the level the change comes to (pressed, for a press) and that may start
   * the bindings bound to the change. v3 starts them as this becomes true, and that is when the log
   * says they fired.
   */
  private static final class Starts implements BooleanSupplier {
    /** Whether the trigger is at this change's level: pressed for a press, let go for a release. */
    private final BooleanSupplier atItsLevel;

    private final WhileDisabled whileDisabled;
    private final Bindings bindings;

    /** What is bound to this change, as recorded. */
    private final List<Binding> bound = new ArrayList<>();

    private Change change = Change.FRESH;

    /** Whether the trigger was at this change's level when it was last polled. */
    private boolean wasAtItsLevel;

    /** Whether this held when it was last polled. */
    private boolean held;

    Starts(BooleanSupplier atItsLevel, WhileDisabled whileDisabled, Bindings bindings) {
      this.atItsLevel = atItsLevel;
      this.whileDisabled = whileDisabled;
      this.bindings = bindings;
    }

    @Override
    public boolean getAsBoolean() {
      wasAtItsLevel = atItsLevel.getAsBoolean();
      boolean holds = mayStartItsBindings(wasAtItsLevel);
      // v3 takes a trigger's first poll as a change, so a button that is held as an enabled robot
      // starts does start what is bound to its press. The log says the same.
      boolean becameTrue = holds && !held;
      if (becameTrue) {
        bound.forEach(bindings::fired);
      }
      held = holds;
      return holds;
    }

    /** Whether the trigger was at this change's level when v3 last polled it. */
    boolean wasAtItsLevel() {
      return wasAtItsLevel;
    }

    /**
     * Whether the trigger, at this change's level or not, may start what is bound to the change
     * now. This also keeps track of whether the change is spent.
     */
    private boolean mayStartItsBindings(boolean isAtItsLevel) {
      if (whileDisabled == WhileDisabled.STARTS_ITS_BINDINGS) {
        return isAtItsLevel;
      }
      if (!bindings.robotIsEnabled()) {
        // Disabled means disabled: nothing starts. And a button that is down now is spent, so
        // that it starts nothing as the robot is enabled, on a press made before anyone was ready.
        change = isAtItsLevel ? Change.SPENT : Change.FRESH;
        return false;
      }
      if (!isAtItsLevel) {
        change = Change.FRESH;
        return false;
      }
      return change == Change.FRESH;
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
  private final Bindings bindings;

  /** Its press, which this trigger polls: what starts on a press is bound to this trigger. */
  private final Starts presses;

  /** Its release, and the v3 trigger that polls it, made when something is first bound to it. */
  private final Starts releases;

  private @Nullable Trigger polledForReleases;

  /**
   * Its press whether or not the robot is enabled, and the v3 trigger that polls it, made when
   * something is first bound to it.
   */
  private final Starts pressesAlsoWhileDisabled;

  private @Nullable Trigger polledAlsoWhileDisabled;

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
    this(
        scheduler,
        loop,
        gamepad,
        button,
        Terms.ONE,
        new Starts(condition, WhileDisabled.STARTS_NOTHING, bindings),
        bindings);
  }

  private RecordingTrigger(
      Scheduler scheduler,
      EventLoop loop,
      String gamepad,
      String button,
      Terms terms,
      Starts presses,
      Bindings bindings) {
    super(scheduler, loop, presses);
    this.scheduler = scheduler;
    this.loop = loop;
    this.gamepad = gamepad;
    this.button = button;
    this.terms = terms;
    this.presses = presses;
    this.bindings = bindings;
    // Each reads the button for itself, so each works whether or not the other triggers are
    // polled: v3 stops polling a trigger that was made inside a command when that command ends.
    BooleanSupplier pressed = presses.atItsLevel;
    releases = new Starts(() -> !pressed.getAsBoolean(), WhileDisabled.STARTS_NOTHING, bindings);
    pressesAlsoWhileDisabled = new Starts(pressed, WhileDisabled.STARTS_ITS_BINDINGS, bindings);
  }

  /**
   * Whether it is pressed, as of the scheduler's last poll, whether or not the robot is enabled:
   * what a command that waits on a button reads. Only what it starts depends on the robot being
   * enabled.
   */
  @Override
  public boolean getAsBoolean() {
    return presses.wasAtItsLevel();
  }

  /** The v3 trigger that polls its release, to bind what starts on a release to. */
  private Trigger onRelease() {
    Trigger polled = polledForReleases;
    if (polled == null) {
      polled = new Trigger(scheduler, loop, releases);
      polledForReleases = polled;
    }
    return polled;
  }

  /** The v3 trigger that polls its press whether or not the robot is enabled. */
  private Trigger onPressAlsoWhileDisabled() {
    Trigger polled = polledAlsoWhileDisabled;
    if (polled == null) {
      polled = new Trigger(scheduler, loop, pressesAlsoWhileDisabled);
      polledAlsoWhileDisabled = polled;
    }
    return polled;
  }

  /** As its bindings name it: "operator right trigger". */
  @Override
  public String toString() {
    return gamepad + " " + button;
  }

  @Override
  public RecordingTrigger onTrue(Command command) {
    record(presses, Binding.How.ON_PRESS, command);
    super.onTrue(command);
    return this;
  }

  /**
   * Starts a command on each press, whether or not the robot is enabled. It is for a command that
   * moves nothing and that a drive team needs before the match, such as the template's heading
   * reset. Every other binding starts nothing while the robot is disabled, and a command that moves
   * a mechanism belongs on one of those: started while disabled, it would move the robot as it is
   * enabled.
   *
   * <p>Commands v3 has no way for a command to say it runs while disabled, so this is the
   * template's own, and it is the only one.
   */
  public RecordingTrigger onTrueAlsoWhileDisabled(Command command) {
    record(pressesAlsoWhileDisabled, Binding.How.ON_PRESS, command);
    onPressAlsoWhileDisabled().onTrue(command);
    return this;
  }

  // What starts on a release is bound to the trigger that polls the release, by the binding that
  // starts on a press there: a release is that trigger's press.

  @Override
  public RecordingTrigger onFalse(Command command) {
    record(releases, Binding.How.ON_RELEASE, command);
    onRelease().onTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger whileTrue(Command command) {
    record(presses, Binding.How.WHILE_HELD, command);
    super.whileTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger whileFalse(Command command) {
    record(releases, Binding.How.WHILE_RELEASED, command);
    onRelease().whileTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger toggleOnTrue(Command command) {
    record(presses, Binding.How.TOGGLE_ON_PRESS, command);
    super.toggleOnTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger toggleOnFalse(Command command) {
    record(releases, Binding.How.TOGGLE_ON_RELEASE, command);
    onRelease().toggleOnTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger retryWhileTrue(Command command) {
    record(presses, Binding.How.RETRY_WHILE_HELD, command);
    super.retryWhileTrue(command);
    return this;
  }

  @Override
  public RecordingTrigger retryWhileFalse(Command command) {
    record(releases, Binding.How.RETRY_WHILE_RELEASED, command);
    onRelease().retryWhileTrue(command);
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
    Starts madePresses = new Starts(made, WhileDisabled.STARTS_NOTHING, bindings);
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
   *
   * @param starts the change of this trigger that starts the command, which logs it as it fires
   */
  private void record(Starts starts, Binding.How how, Command command) {
    boolean isMadeInsideACommand = scheduler.currentCommand() != null;
    if (isMadeInsideACommand) {
      return;
    }
    Binding binding = new Binding(gamepad, button, how, command.name());
    starts.bound.add(binding);
    bindings.add(binding);
  }
}

package com.michaelgrundvig.frc.runtime.autos;

import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.world.field.Place;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent;
import org.wpilib.system.RobotController;
import org.wpilib.util.Alert;

/**
 * The robot's autos: the list a team declares in its {@code Controls} ({@link #add}), the one the
 * pit chose, and the template's running of it. Nothing a team writes starts or stops an auto: when
 * autonomous becomes enabled the chosen auto's command is made fresh and scheduled ({@link
 * #autonomousStarted}), and when autonomous ends it is canceled if still running ({@link
 * #autonomousEnded}).
 *
 * <ul>
 *   <li><b>There is always a choice.</b> "Do nothing" is first, added here, and is what the robot
 *       starts with: the choice is never null or empty, and a robot with no autos declared has only
 *       that.
 *   <li><b>The pit chooses as it chooses a venue.</b> A card on Pitwall's Pit view ({@link
 *       AutoPit}) calls {@link #choose} on a server thread; the robot takes the choice at the start
 *       of a loop while disabled ({@link #update}), never while it drives. The selection is logged
 *       as an input ({@code Auto/Selected}), so a replay runs the auto the robot did, and what is
 *       in use as {@code Auto/Chosen}.
 *   <li><b>A fresh command every run</b>, built when autonomous starts. One that throws as it is
 *       built is caught here: nothing runs, {@code Auto/Outcome} says why, and an alert is raised.
 *       One that throws as it runs is the command scheduler's to handle, as any command's: the loop
 *       carries on past it on a robot, and fails the test in the simulator ({@code Robot.loop}); it
 *       is reported here too.
 *   <li><b>The start.</b> The chosen auto's start is where the robot is placed before the match,
 *       and what its placement is checked against.
 * </ul>
 *
 * <p>Two autos of one name, or a blank name, are refused as {@link Refusals} refuses a mistake:
 * thrown as the robot starts. Declared once the robot runs, the auto is left out and an alert says
 * so.
 */
public final class Autos implements AutoCloseable {
  /** The auto that is always there, and the robot's default: it moves nothing. */
  public static final String DO_NOTHING = "Do nothing";

  /** How a run ended. */
  public enum Ending {
    /** Its command finished. */
    COMPLETED,
    /** Autonomous ended while it was still running: how an auto that shoots until the end ends. */
    CUT_OFF,
    /** Something else canceled it. */
    CANCELED,
    /** Its command threw, as it was built or as it ran. */
    FAILED
  }

  /**
   * One run of an auto.
   *
   * @param auto its name
   * @param ending how it ended
   * @param seconds how long it ran
   * @param why what threw, for {@link Ending#FAILED}; empty otherwise
   */
  public record Run(String auto, Ending ending, double seconds, String why) {
    /** As the log and the pit say it: "Left Double: COMPLETED in 9.80 s". */
    public String describe() {
      String how =
          switch (ending) {
            case COMPLETED -> String.format(Locale.ROOT, "COMPLETED in %.2f s", seconds);
            case CUT_OFF ->
                String.format(Locale.ROOT, "CUT OFF after %.2f s, as autonomous ended", seconds);
            case CANCELED -> String.format(Locale.ROOT, "CANCELED after %.2f s", seconds);
            case FAILED -> String.format(Locale.ROOT, "FAILED after %.2f s", seconds);
          };
      return auto + ": " + how + (why.isEmpty() ? "" : ": " + why);
    }
  }

  /** The pit's selection, logged as an input so a replay chooses what the robot did. */
  private static final class Selection implements LoggableInputs {
    String name = DO_NOTHING;

    @Override
    public void toLog(LogTable table) {
      table.put("Selected", name);
    }

    @Override
    public void fromLog(LogTable table) {
      name = table.get("Selected", name);
    }
  }

  private final Scheduler scheduler;

  /** Every auto, "Do nothing" first. Declared as the robot starts; read by the pit's threads. */
  private final List<Auto> declared = new CopyOnWriteArrayList<>();

  /** What the pit asked for last: any thread sets it, the loop takes it while disabled. */
  private volatile String requested = DO_NOTHING;

  private final Selection selection = new Selection();
  private Auto chosen;
  private boolean pending;

  /** The command scheduled for this autonomous, and when it started. */
  private @Nullable Command running;

  private long startedNanos;

  /** Whether the end of autonomous is what is canceling the running command. */
  private boolean cuttingOff;

  private @Nullable Run lastRun;

  private final Alert failed = new Alert("Auto", "failed", "", Alert.Level.HIGH);
  private final Alert refused = new Alert("Auto", "refused", "", Alert.Level.MEDIUM);

  /**
   * @param scheduler the robot's command scheduler, which runs the chosen auto and which says
   *     whether the robot is starting
   */
  public Autos(Scheduler scheduler) {
    this.scheduler = scheduler;
    Auto nothing =
        new Auto(DO_NOTHING, null, () -> Command.noRequirements(coroutine -> {}).named(DO_NOTHING));
    declared.add(nothing);
    chosen = nothing;
    scheduler.addEventListener(this::heard);
  }

  /**
   * Declares an auto. Its command is made fresh each time autonomous starts with it chosen, so the
   * factory builds everything the run composes: a mechanism's or a gate's built once, in the
   * constructor, is shared.
   *
   * @param name what the pit and the log call it; not blank, and not another auto's
   * @param start where the robot starts, as a place written for blue
   * @param command makes the auto's command, a new one on every call
   * @throws IllegalArgumentException as the robot starts, for a blank name or two autos of one name
   */
  public void add(String name, Place start, Supplier<Command> command) {
    String why = null;
    if (name.isBlank()) {
      why = "an auto needs a name";
    } else if (find(name) != null) {
      why = "two autos are named " + name + ": name each once";
    }
    String refusal = Refusals.asBuilt(scheduler, why);
    if (refusal != null) {
      refused.setText("An auto was left out: " + refusal);
      refused.set(true);
      return;
    }
    declared.add(new Auto(name, Objects.requireNonNull(start), command));
  }

  private @Nullable Auto find(String name) {
    for (Auto auto : declared) {
      if (auto.name().equals(name)) {
        return auto;
      }
    }
    return null;
  }

  /** Every auto's name in the order declared, "Do nothing" first: what the pit chooses among. */
  public List<String> names() {
    return declared.stream().map(Auto::name).toList();
  }

  /**
   * The pit's choice, for the robot to take at the start of a loop while disabled. Any thread.
   *
   * @throws IllegalArgumentException saying which autos there are, if none has this name
   */
  public void choose(String name) {
    if (find(name) == null) {
      throw new IllegalArgumentException(
          "there's no auto " + name + " on the robot: it has " + String.join(", ", names()));
    }
    requested = name;
  }

  /** What the pit last asked for, taken or not. */
  public String requested() {
    return requested;
  }

  /** The auto chosen as of this loop. Never null: "Do nothing" until the pit chooses another. */
  public Auto chosen() {
    return chosen;
  }

  /** Whether a choice waits for the robot to be disabled. */
  public boolean pending() {
    return pending;
  }

  /** The chosen auto's name, and that another waits for the robot to be disabled. */
  public String describe() {
    return chosen.name() + (pending ? " (" + requested + " when disabled)" : "");
  }

  /**
   * At the start of a loop: takes the pit's choice when the robot is disabled, and logs it. A
   * replay's log names the choice; one this code has no auto for leaves "Do nothing".
   *
   * @param disabled whether the robot is disabled: the choice never changes while it drives
   */
  public void update(boolean disabled) {
    selection.name = requested;
    Logger.processInputs("Auto", selection);
    String wanted = selection.name;
    Auto auto = find(wanted);
    if (disabled && !wanted.equals(chosen.name())) {
      chosen = auto != null ? auto : Objects.requireNonNull(find(DO_NOTHING));
    }
    pending = !wanted.equals(chosen.name());
    Logger.recordOutput("Auto/Chosen", chosen.name());
    Logger.recordOutput("Auto/Pending", pending);
  }

  /**
   * Autonomous became enabled: makes the chosen auto's command, fresh, and schedules it. One that
   * throws as it is built runs nothing and is said ({@link #lastRun}, {@code Auto/Outcome}, an
   * alert); the robot carries on.
   */
  public void autonomousStarted() {
    stopRunning(false);
    failed.set(false);
    Auto auto = chosen;
    startedNanos = RobotController.getTime();
    Logger.recordOutput("Auto/Outcome", auto.name() + ": running");
    try {
      Command command = auto.command().get();
      running = command;
      scheduler.schedule(command);
    } catch (RuntimeException e) {
      ended(auto.name(), Ending.FAILED, 0, "it couldn't be built: " + shown(e));
    }
  }

  /** Autonomous ended, disabled or into teleop: cancels the auto if it is still running. */
  public void autonomousEnded() {
    stopRunning(true);
  }

  /** What the latest run failed of, as its alert says, while it is raised; null otherwise. */
  public @Nullable String failure() {
    return failed.get() ? failed.getText() : null;
  }

  /** Why an auto declared once the robot ran was left out, while its alert is raised. */
  public @Nullable String refusal() {
    return refused.get() ? refused.getText() : null;
  }

  /** The latest run's outcome, if an auto has run. */
  public Optional<Run> lastRun() {
    return Optional.ofNullable(lastRun);
  }

  private void stopRunning(boolean autonomousEnded) {
    Command command = running;
    if (command != null && scheduler.isScheduledOrRunning(command)) {
      cuttingOff = autonomousEnded;
      scheduler.cancel(command);
      cuttingOff = false;
    }
  }

  private void heard(SchedulerEvent event) {
    Command command = running;
    if (command == null) {
      return;
    }
    double seconds = (event.timestampNanos() - startedNanos) / 1e9;
    // The choice can't change while autonomous is enabled, so the run is of the chosen auto.
    String auto = chosen.name();
    switch (event) {
      case SchedulerEvent.Completed done when done.command() == command ->
          ended(auto, Ending.COMPLETED, seconds, "");
      case SchedulerEvent.Canceled canceled when canceled.command() == command ->
          ended(auto, cuttingOff ? Ending.CUT_OFF : Ending.CANCELED, seconds, "");
      case SchedulerEvent.CompletedWithError error when error.command() == command ->
          ended(auto, Ending.FAILED, seconds, shown(error.error()));
      default -> {}
    }
  }

  private void ended(String auto, Ending ending, double seconds, String why) {
    running = null;
    Run run = new Run(auto, ending, seconds, why);
    lastRun = run;
    Logger.recordOutput("Auto/Outcome", run.describe());
    if (ending == Ending.FAILED) {
      failed.setText("Autonomous \"" + auto + "\" failed: " + why);
      failed.set(true);
    }
  }

  private static String shown(Throwable e) {
    String message = e.getMessage();
    return e.getClass().getSimpleName() + (message == null ? "" : ": " + message);
  }

  @Override
  public void close() {
    failed.close();
    refused.close();
  }
}

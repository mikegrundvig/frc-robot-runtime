package com.michaelgrundvig.frc.runtime.autos;

import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.world.field.Place;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;
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
 * pit chose, and the running of it. Nothing a team writes starts or stops an auto. When autonomous
 * becomes enabled, the chosen auto's command is made fresh and scheduled ({@link
 * #autonomousStarted}). When autonomous ends, that command is canceled if it is still running
 * ({@link #autonomousEnded}).
 *
 * <ul>
 *   <li><b>There is always a choice.</b> "Do nothing" is added here, first in the list, and is what
 *       the robot starts with. So the choice is never missing, and a robot with no autos declared
 *       has only that one.
 *   <li><b>The pit chooses an auto as it chooses a venue.</b> A card on Pitwall's Pit view ({@link
 *       AutoPit}) calls {@link #choose} on a server thread. The robot takes the choice at the start
 *       of a loop while it is disabled ({@link #update}), and never while it drives. What the pit
 *       selected is logged as an input ({@code Auto/Selected}), so a replay runs the auto the robot
 *       ran. The auto in use is logged as {@code Auto/Chosen}.
 *   <li><b>Every run gets a fresh command</b>, built when autonomous starts. A factory that throws
 *       as it builds one is caught here: nothing runs, {@code Auto/Outcome} says why, and an alert
 *       is raised. A command that throws as it runs is the command scheduler's to handle, like any
 *       other command's throw: on a robot the loop carries on past it, and in the simulator it
 *       fails the test ({@code Robot.loop}). It is reported here as well.
 *   <li><b>The start.</b> The chosen auto's start is where the robot is placed before the match,
 *       and what its placement is checked against.
 * </ul>
 *
 * <p>An auto with a blank name, or with the name of another auto, is refused as {@link Refusals}
 * refuses any mistake: as the robot starts, that is a throw. If it is declared once the robot runs,
 * the auto is left out and an alert says so.
 */
public final class Autos implements AutoCloseable {
  /** The auto that is always there, and that the robot starts with. It moves nothing. */
  public static final String DO_NOTHING = "Do nothing";

  /** How a run ended. */
  public enum Ending {
    /** Its command finished. */
    COMPLETED,
    /**
     * Autonomous ended while it was still running. An auto that shoots until the end ends this way.
     */
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
      if (why.isEmpty()) {
        return auto + ": " + how;
      }
      return auto + ": " + how + ": " + why;
    }
  }

  /** The pit's selection, logged as an input, so that a replay chooses what the robot chose. */
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

  /**
   * The command scheduled for this autonomous.
   *
   * @param command the chosen auto's command, as made for this run
   * @param startedNanos when it was scheduled, on the robot's clock
   */
  private record Running(Command command, long startedNanos) {}

  private final Scheduler scheduler;

  /**
   * Every auto, "Do nothing" first. Declared as the robot starts, and read by the pit's threads.
   */
  private final List<Auto> declared = new CopyOnWriteArrayList<>();

  /** The auto that is always there. */
  private final Auto nothing;

  /**
   * The name the pit asked for last. Any thread sets it, and the robot loop takes it while the
   * robot is disabled.
   */
  private volatile String requested = DO_NOTHING;

  private final Selection selection = new Selection();

  /** The auto in use. */
  private Auto chosen;

  /** Whether the pit has asked for another auto, which waits for the robot to be disabled. */
  private boolean pending;

  /** The command running for this autonomous, if one is. */
  private Optional<Running> running = Optional.empty();

  /**
   * How the run is recorded as ending if the running command is canceled now. A cancel is {@link
   * Ending#CANCELED} unless this class is making it because autonomous ended, and for the length of
   * that one call this says {@link Ending#CUT_OFF}.
   */
  private Ending endingIfCanceled = Ending.CANCELED;

  private Optional<Run> lastRun = Optional.empty();

  private final Alert failed = new Alert("Auto", "failed", "", Alert.Level.HIGH);
  private final Alert refused = new Alert("Auto", "refused", "", Alert.Level.MEDIUM);

  /**
   * @param scheduler the robot's command scheduler, which runs the chosen auto and which says
   *     whether the robot is starting
   */
  public Autos(Scheduler scheduler) {
    this.scheduler = scheduler;
    nothing =
        new Auto(
            DO_NOTHING,
            Optional.empty(),
            () -> Command.noRequirements(coroutine -> {}).named(DO_NOTHING));
    declared.add(nothing);
    chosen = nothing;
    scheduler.addEventListener(this::heard);
  }

  // ---- Declaring and choosing ----

  /**
   * Declares an auto. Its command is made fresh each time autonomous starts with it chosen, so the
   * factory must build everything that the run composes. Whatever is built once, in the team's
   * constructor, such as a mechanism or a gate, is shared between runs.
   *
   * @param name what the pit and the log call it. Not blank, and not the name of another auto.
   * @param start where the robot starts, as a place written for blue
   * @param command makes the auto's command, a new one on every call
   * @throws IllegalArgumentException as the robot starts, for a blank name or two autos of one name
   */
  public void add(String name, Place start, Supplier<Command> command) {
    Optional<String> refusal = Refusals.whenBuilt(scheduler, whyNotDeclared(name));
    if (refusal.isPresent()) {
      refused.setText("An auto was left out: " + refusal.get());
      refused.set(true);
      return;
    }
    declared.add(new Auto(name, Optional.of(start), command));
  }

  /** Why an auto of this name cannot be declared, if it cannot. */
  private Optional<String> whyNotDeclared(String name) {
    if (name.isBlank()) {
      return Optional.of("an auto needs a name");
    }
    if (find(name).isPresent()) {
      return Optional.of("two autos are named " + name + ": name each once");
    }
    return Optional.empty();
  }

  private Optional<Auto> find(String name) {
    for (Auto auto : declared) {
      if (auto.name().equals(name)) {
        return Optional.of(auto);
      }
    }
    return Optional.empty();
  }

  /**
   * Every auto's name in the order declared, "Do nothing" first. This is what the pit chooses
   * among.
   */
  public List<String> names() {
    return declared.stream().map(Auto::name).toList();
  }

  /**
   * Takes the pit's choice, for the robot to put into use at the start of a loop while it is
   * disabled. Any thread may call this.
   *
   * @throws IllegalArgumentException saying which autos there are, if none has this name
   */
  public void choose(String name) {
    if (find(name).isEmpty()) {
      throw new IllegalArgumentException(
          "there's no auto " + name + " on the robot: it has " + String.join(", ", names()));
    }
    requested = name;
  }

  /** The name the pit last asked for, whether or not the robot has taken it yet. */
  public String requested() {
    return requested;
  }

  /** The auto in use as of this loop: "Do nothing" until the pit chooses another. */
  public Auto chosen() {
    return chosen;
  }

  /** Whether a choice waits for the robot to be disabled. */
  public boolean pending() {
    return pending;
  }

  /** The chosen auto's name, and, if another waits for the robot to be disabled, that too. */
  public String describe() {
    if (pending) {
      return chosen.name() + " (" + requested + " when disabled)";
    }
    return chosen.name();
  }

  // ---- Each loop ----

  /**
   * Starts a loop: puts the pit's choice into use if the robot is disabled, and logs it.
   *
   * <p>In a replay the log names the choice. If this code has no auto of that name, "Do nothing" is
   * used.
   *
   * @param disabled whether the robot is disabled. The auto in use never changes while it drives.
   */
  public void update(boolean disabled) {
    selection.name = requested;
    Logger.processInputs("Auto", selection);
    String wanted = selection.name;
    boolean isInUse = wanted.equals(chosen.name());
    if (disabled && !isInUse) {
      chosen = find(wanted).orElse(nothing);
    }
    pending = !wanted.equals(chosen.name());
    Logger.recordOutput("Auto/Chosen", chosen.name());
    Logger.recordOutput("Auto/Pending", pending);
  }

  // ---- Running the chosen auto ----

  /**
   * Autonomous became enabled: makes the chosen auto's command, fresh, and schedules it. If the
   * factory throws as it builds the command, nothing runs and the failure is said ({@link
   * #lastRun}, {@code Auto/Outcome}, and an alert). The robot carries on.
   */
  public void autonomousStarted() {
    // A command left running from an earlier autonomous would run alongside this one.
    cancelRunning(Ending.CANCELED);
    failed.set(false);
    Auto auto = chosen;
    long startedNanos = RobotController.getTime();
    Logger.recordOutput("Auto/Outcome", auto.name() + ": running");
    try {
      Command command = auto.command().get();
      running = Optional.of(new Running(command, startedNanos));
      scheduler.schedule(command);
    } catch (RuntimeException e) {
      ended(auto.name(), Ending.FAILED, 0, "it couldn't be built: " + shown(e));
    }
  }

  /**
   * Autonomous ended, because the robot was disabled or went into teleop: cancels the auto if it is
   * still running.
   */
  public void autonomousEnded() {
    cancelRunning(Ending.CUT_OFF);
  }

  /**
   * Cancels the running command, if it is still scheduled or running.
   *
   * @param recordedAs how the run is recorded as ending, if this cancels it
   */
  private void cancelRunning(Ending recordedAs) {
    if (running.isEmpty()) {
      return;
    }
    Command command = running.get().command();
    if (!scheduler.isScheduledOrRunning(command)) {
      return;
    }
    // The scheduler tells this class of the cancel before cancel() returns (heard), so what the
    // run is recorded as is set only for the length of the call.
    endingIfCanceled = recordedAs;
    scheduler.cancel(command);
    endingIfCanceled = Ending.CANCELED;
  }

  /**
   * Hears everything the scheduler does, and records the run when what it did was end the running
   * auto's command.
   */
  private void heard(SchedulerEvent event) {
    if (running.isEmpty()) {
      return;
    }
    Command command = running.get().command();
    double seconds = (event.timestampNanos() - running.get().startedNanos()) / 1e9;
    // The auto in use cannot change while autonomous is enabled, so the run is of the chosen auto.
    String auto = chosen.name();
    switch (event) {
      case SchedulerEvent.Completed done when done.command() == command ->
          ended(auto, Ending.COMPLETED, seconds, "");
      case SchedulerEvent.Canceled canceled when canceled.command() == command ->
          ended(auto, endingIfCanceled, seconds, "");
      case SchedulerEvent.CompletedWithError error when error.command() == command ->
          ended(auto, Ending.FAILED, seconds, shown(error.error()));
      default -> {}
    }
  }

  /** Records how a run ended, and raises the alert if it failed. */
  private void ended(String auto, Ending ending, double seconds, String why) {
    running = Optional.empty();
    Run run = new Run(auto, ending, seconds, why);
    lastRun = Optional.of(run);
    Logger.recordOutput("Auto/Outcome", run.describe());
    if (ending == Ending.FAILED) {
      failed.setText("Autonomous \"" + auto + "\" failed: " + why);
      failed.set(true);
    }
  }

  /** What was thrown, by its kind and what it says: "IllegalStateException: no path". */
  private static String shown(Throwable thrown) {
    String message = thrown.getMessage();
    String kind = thrown.getClass().getSimpleName();
    if (message == null) {
      return kind;
    }
    return kind + ": " + message;
  }

  // ---- What the pit reads ----

  /** What the latest run failed of, as its alert says it, while that alert is raised. */
  public Optional<String> failure() {
    if (!failed.get()) {
      return Optional.empty();
    }
    return Optional.of(failed.getText());
  }

  /**
   * Why an auto that was declared once the robot ran was left out, as its alert says it, while that
   * alert is raised.
   */
  public Optional<String> refusal() {
    if (!refused.get()) {
      return Optional.empty();
    }
    return Optional.of(refused.getText());
  }

  /** The latest run's outcome, if an auto has run. */
  public Optional<Run> lastRun() {
    return lastRun;
  }

  @Override
  public void close() {
    failed.close();
    refused.close();
  }
}

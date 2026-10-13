package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.record.BindingFired;
import com.michaelgrundvig.frc.world.record.BindingStart;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;

/**
 * Every command bound to the drive team's gamepads, recorded as each binding is made. This is the
 * one place the robot's bindings are listed, so a controller map (the documentation's, or
 * Pitwall's) comes from the bindings the robot really makes and never from a second list that could
 * disagree.
 *
 * <p>The robot's container makes this, and makes its gamepads from it ({@link #gamepad}). A team
 * then binds exactly as it would on WPILib's gamepad: {@code
 * operator.rightTrigger().whileTrue(intake())} is recorded as "operator right trigger: while held:
 * Intake".
 *
 * <p>The record is logged as it grows ({@code Controls/Bindings}), each binding in its own words,
 * which are a team's: its gamepad's name, its button's and its command's. Each loop, the bindings
 * that a press or a release started are logged as facts ({@code Controls/Fired}): which binding of
 * that list, on which gamepad's port, on what kind of control, and how it starts ({@link
 * BindingFired}), with the name of the command each started beside them ({@code
 * Controls/FiredCommands}). While the robot is disabled a binding starts nothing, and none is
 * logged as fired ({@link RecordingTrigger}). Two commands bound to one button in the same way
 * start together, so each such button is a doubt the robot says at startup ({@link #doubts}).
 *
 * <p>A binding made inside a running command lives only while that command runs, as v3's bindings
 * do, and is not recorded. The record is of the robot's standing bindings.
 */
public final class Bindings {
  private static final String BINDINGS_KEY = "Controls/Bindings";
  private static final String FIRED_KEY = "Controls/Fired";
  private static final String FIRED_COMMANDS_KEY = "Controls/FiredCommands";

  /** What is logged while there is nothing to list. */
  private static final String[] NONE = new String[0];

  /** What is logged in a loop in which no binding fired. */
  private static final BindingFired[] NONE_FIRED = new BindingFired[0];

  /**
   * One of the robot's standing bindings, with what the log says of it each time it fires.
   *
   * @param binding the binding, in its own words
   * @param fired the facts of it: where it is in the list of bindings, its gamepad's port, the
   *     control it is on, and how it starts. It is made once, as the binding is, so a firing makes
   *     none.
   */
  record Recorded(Binding binding, BindingFired fired) {}

  /**
   * A button and a way of binding a command to it. Two bindings that share both start together.
   *
   * @param button the button with its gamepad's name: "operator right trigger"
   * @param how when a command bound this way starts
   */
  private record Start(String button, BindingStart how) {}

  private final Scheduler scheduler;

  /** Whether the robot is enabled, as of this loop. */
  private final BooleanSupplier robotIsEnabled;

  private final List<Binding> all = new ArrayList<>();

  /** The bindings a press or a release started this loop. */
  private final List<Recorded> fired = new ArrayList<>();

  /**
   * @param scheduler the command scheduler its gamepads bind commands on
   * @param robotIsEnabled whether the robot is enabled, asked as the scheduler polls the gamepads:
   *     {@code RobotState::isEnabled}. While it is not, the bindings start nothing.
   */
  public Bindings(Scheduler scheduler, BooleanSupplier robotIsEnabled) {
    this.scheduler = scheduler;
    this.robotIsEnabled = robotIsEnabled;
    Logger.recordOutput(BINDINGS_KEY, NONE);
    logFired();
    // The scheduler runs this before it polls its triggers. So each loop the last loop's firings
    // are cleared first, and then this loop's are added as the triggers are polled.
    scheduler.addPeriodic(this::forgetLastLoopsFirings);
  }

  /**
   * A gamepad on a Driver Station port, whose bindings are recorded here.
   *
   * @param name how the record names it: "driver", "operator"
   * @param port its Driver Station port: 0 for the driver, 1 for the operator
   */
  public RecordingGamepad gamepad(String name, int port) {
    return new RecordingGamepad(scheduler, port, name, this);
  }

  /** Every binding, in the order they were made. */
  public List<Binding> all() {
    return List.copyOf(all);
  }

  /**
   * The bindings a press or a release started this loop, as the scheduler polled the gamepads.
   * Empty in a loop that started none.
   */
  public List<Binding> fired() {
    List<Binding> bindings = new ArrayList<>();
    for (Recorded each : fired) {
      bindings.add(each.binding());
    }
    return List.copyOf(bindings);
  }

  /**
   * A doubt for each button that has two commands bound to it in the same way, which therefore
   * start together. It is a doubt and not a refusal because it is sometimes meant: "operator right
   * trigger starts Intake and Outtake while held: one press runs them together. If that's meant,
   * make them one command; if not, bind one to another button".
   */
  public List<Finding> doubts() {
    List<Finding> doubts = new ArrayList<>();
    for (List<Binding> together : bindingsByStart().values()) {
      if (together.size() >= 2) {
        doubts.add(doubt(together));
      }
    }
    return doubts;
  }

  /** The bindings that start together, grouped, in the order each group's first was made. */
  private Map<Start, List<Binding>> bindingsByStart() {
    Map<Start, List<Binding>> byStart = new LinkedHashMap<>();
    for (Binding binding : all) {
      Start start = new Start(binding.gamepad() + " " + binding.button(), binding.how());
      byStart.computeIfAbsent(start, each -> new ArrayList<>()).add(binding);
    }
    return byStart;
  }

  /** The doubt about two or more bindings that start together. */
  private static Finding doubt(List<Binding> together) {
    Binding first = together.get(0);
    String button = first.gamepad() + " " + first.button();
    Set<String> commands = new LinkedHashSet<>();
    for (Binding binding : together) {
      commands.add(binding.command());
    }
    boolean isOneCommandBoundTwice = commands.size() == 1;
    if (isOneCommandBoundTwice) {
      return Finding.doubt(
          button,
          button
              + " starts "
              + first.command()
              + " "
              + first.how().words()
              + " twice: bind it once");
    }
    String edge = first.how().startsOnPress() ? "press" : "release";
    return Finding.doubt(
        button,
        button
            + " starts "
            + listed(List.copyOf(commands))
            + " "
            + first.how().words()
            + ": one "
            + edge
            + " runs them together. If that's meant, make them one command; if not,"
            + " bind one to another button");
  }

  /** Two or more names as a list in a sentence: "A and B", or "A, B and C". */
  private static String listed(List<String> names) {
    int last = names.size() - 1;
    return String.join(", ", names.subList(0, last)) + " and " + names.get(last);
  }

  /** Whether the robot is enabled now. While it is not, a binding starts nothing. */
  boolean robotIsEnabled() {
    return robotIsEnabled.getAsBoolean();
  }

  /**
   * Records a binding as it is made.
   *
   * @param port the Driver Station port of the gamepad it is on
   * @param control what kind of control on the gamepad it is made on
   * @param button the button's index on the gamepad, for a binding on one button; none otherwise
   * @return the binding as recorded, for the trigger it is on to say when it fires
   */
  Recorded add(Binding binding, int port, BindingFired.Control control, int button) {
    int placeInTheList = all.size();
    all.add(binding);
    Logger.recordOutput(BINDINGS_KEY, shown(all));
    return new Recorded(
        binding, new BindingFired(placeInTheList, port, control, button, binding.how()));
  }

  /** Notes, and logs, a binding that a press or a release started this loop. */
  void fired(Recorded binding) {
    fired.add(binding);
    logFired();
  }

  private void forgetLastLoopsFirings() {
    if (fired.isEmpty()) {
      return;
    }
    fired.clear();
    logFired();
  }

  /**
   * Logs the bindings that fired this loop as facts, with the name of the command each started
   * beside them, in the same order.
   */
  private void logFired() {
    if (fired.isEmpty()) {
      Logger.recordOutput(FIRED_KEY, NONE_FIRED);
      Logger.recordOutput(FIRED_COMMANDS_KEY, NONE);
      return;
    }
    BindingFired[] facts = new BindingFired[fired.size()];
    String[] commands = new String[fired.size()];
    for (int i = 0; i < facts.length; i++) {
      facts[i] = fired.get(i).fired();
      commands[i] = fired.get(i).binding().command();
    }
    Logger.recordOutput(FIRED_KEY, facts);
    Logger.recordOutput(FIRED_COMMANDS_KEY, commands);
  }

  /** Bindings as the log lists them, each in its own words. */
  private static String[] shown(List<Binding> bindings) {
    String[] shown = new String[bindings.size()];
    for (int i = 0; i < shown.length; i++) {
      shown[i] = bindings.get(i).toString();
    }
    return shown;
  }
}

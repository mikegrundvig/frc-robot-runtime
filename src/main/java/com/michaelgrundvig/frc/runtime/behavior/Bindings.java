package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;

/**
 * Every command bound to the drive team's gamepads, recorded as each binding is made: the one place
 * the robot's bindings are listed, so a controller map, the documentation's or Pitwall's, comes
 * from the bindings the robot makes and never from a second list. The robot's container makes it,
 * and its gamepads ({@link #gamepad}), so a team binds exactly as it would on WPILib's: {@code
 * operator.rightTrigger().whileTrue(intake())} is recorded as "operator right trigger: while held:
 * Intake".
 *
 * <p>The record is logged as it grows ({@code Controls/Bindings}), and each loop, the bindings a
 * press or a release started ({@code Controls/Fired}). Two commands bound to one button the same
 * way start together, so each such button is a doubt the robot says at startup ({@link #doubts}).
 *
 * <p>A binding made inside a running command lives only while that command runs, as v3's do, and
 * isn't recorded: the record is the robot's standing bindings.
 */
public final class Bindings {
  private static final String BINDINGS_KEY = "Controls/Bindings";
  private static final String FIRED_KEY = "Controls/Fired";

  private final Scheduler scheduler;
  private final List<Binding> all = new ArrayList<>();

  /** The bindings a press or a release started this loop. */
  private final List<Binding> fired = new ArrayList<>();

  /**
   * @param scheduler the command scheduler its gamepads bind commands on
   */
  public Bindings(Scheduler scheduler) {
    this.scheduler = scheduler;
    Logger.recordOutput(BINDINGS_KEY, new String[0]);
    Logger.recordOutput(FIRED_KEY, new String[0]);
    // Before the scheduler polls its triggers: last loop's firings are cleared, this loop's added.
    scheduler.addPeriodic(this::nextLoop);
  }

  /**
   * A gamepad on a Driver Station port, its bindings recorded here.
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
   * Each button with two commands bound to it the same way, which then start together: a doubt,
   * since it's sometimes meant. "operator right trigger starts Intake and Outtake while held: one
   * press runs them together. If that's meant, make them one command; if not, bind one to another
   * button".
   */
  public List<Finding> doubts() {
    Map<String, List<Binding>> byButton = new LinkedHashMap<>();
    for (Binding binding : all) {
      byButton
          .computeIfAbsent(
              binding.gamepad() + " " + binding.button() + "\n" + binding.how(),
              key -> new ArrayList<>())
          .add(binding);
    }
    List<Finding> doubts = new ArrayList<>();
    for (List<Binding> same : byButton.values()) {
      if (same.size() < 2) {
        continue;
      }
      Binding first = same.get(0);
      String button = first.gamepad() + " " + first.button();
      Set<String> commands = new LinkedHashSet<>();
      same.forEach(binding -> commands.add(binding.command()));
      doubts.add(
          Finding.doubt(
              button,
              commands.size() == 1
                  ? button
                      + " starts "
                      + first.command()
                      + " "
                      + first.how()
                      + " twice: bind it once"
                  : button
                      + " starts "
                      + and(List.copyOf(commands))
                      + " "
                      + first.how()
                      + ": one "
                      + (first.how().startsOnPress() ? "press" : "release")
                      + " runs them together. If that's meant, make them one command; if not,"
                      + " bind one to another button"));
    }
    return doubts;
  }

  /** "A and B", or "A, B and C". */
  private static String and(List<String> names) {
    int last = names.size() - 1;
    return String.join(", ", names.subList(0, last)) + " and " + names.get(last);
  }

  /** Records a binding as it's made. */
  void add(Binding binding) {
    all.add(binding);
    Logger.recordOutput(BINDINGS_KEY, all.stream().map(Binding::toString).toArray(String[]::new));
  }

  /** Logs a binding a press or a release started, this loop. */
  void fired(Binding binding) {
    fired.add(binding);
    Logger.recordOutput(FIRED_KEY, fired.stream().map(Binding::toString).toArray(String[]::new));
  }

  /**
   * The bindings a press or a release started this loop, as the scheduler polled the gamepads:
   * empty in a loop that started none.
   */
  public List<Binding> fired() {
    return List.copyOf(fired);
  }

  private void nextLoop() {
    if (!fired.isEmpty()) {
      fired.clear();
      Logger.recordOutput(FIRED_KEY, new String[0]);
    }
  }
}

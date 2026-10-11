package com.michaelgrundvig.frc.runtime.behavior;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.util.Alert;

/**
 * What one {@link Gate} has worked out: whether it holds, and the terms that keep it false. It is
 * also the working out itself: it runs the gate's logic, logs the answer, and reports a logic that
 * throws. A gate's v3 trigger reads its value from here.
 *
 * <p>A gate is kept up to date in one of two ways ({@link Upkeep}), which its {@link Gate} chooses
 * as it is made.
 */
final class GateState implements BooleanSupplier {
  /** When a gate is worked out. */
  private enum Upkeep {
    /** Once a loop, by the scheduler, before its bindings and commands read the gate. */
    EVERY_LOOP,
    /** Each time the gate is read. Nothing runs for a gate that nobody reads any more. */
    WHEN_READ
  }

  /** What is logged for a gate with nothing blocking it. */
  private static final String[] NOTHING_BLOCKING = new String[0];

  private final String name;
  private final Gate.Logic logic;
  private final String key;
  private final String blockingKey;

  private Upkeep upkeep = Upkeep.EVERY_LOOP;

  /** Whether the gate held the last time it was worked out. */
  private boolean holds;

  /** The terms that kept it false the last time it was worked out. */
  private final List<String> blocking = new ArrayList<>();

  /**
   * Whether the gate is logged. A gate that was debounced into another is not: the debounced gate
   * is logged under the same name in its place.
   */
  private boolean logged = true;

  /**
   * The blocking terms as they were last logged. The same array is logged again while the terms
   * stay the same, so that a steady gate allocates nothing.
   */
  private String[] shown = NOTHING_BLOCKING;

  /** The alert that is raised while the logic throws. It is made the first time the logic does. */
  private Optional<Alert> threwAlert = Optional.empty();

  /** What the alert says. */
  private String threwText = "";

  /** Whether the logic threw the last time the gate was worked out. */
  private boolean throwing;

  /**
   * @param name the gate's name, as logged: "ready to shoot"
   * @param logic what the gate works out
   */
  GateState(String name, Gate.Logic logic) {
    this.name = name;
    this.logic = logic;
    key = "Gates/" + name;
    blockingKey = key + "/Blocking";
  }

  /** From now on the gate is worked out each time it is read, and never by the scheduler. */
  void workOutWhenRead() {
    upkeep = Upkeep.WHEN_READ;
  }

  /** Stops logging the gate, because another gate is logged under its name in its place. */
  void stopLogging() {
    logged = false;
  }

  /** Whether the gate holds, as of this loop. */
  @Override
  public boolean getAsBoolean() {
    bringUpToDate();
    return holds;
  }

  /** Works the gate out now, if it is one that is worked out as it is read. */
  void bringUpToDate() {
    if (upkeep == Upkeep.WHEN_READ) {
      workOut();
    }
  }

  /**
   * The terms that kept the gate false the last time it was worked out. This is the list itself,
   * which the next working out changes.
   */
  List<String> blocking() {
    return blocking;
  }

  /**
   * Works the gate out, and logs it.
   *
   * <p>A logic that throws is caught here. The scheduler runs this as one of its periodic tasks,
   * and a periodic task that throws is over for good: the scheduler would then throw on every loop
   * after it.
   */
  void workOut() {
    boolean threw = false;
    blocking.clear();
    try {
      holds = logic.holds(blocking);
    } catch (RuntimeException e) {
      holdFalseAfter(e);
      threw = true;
    }
    showWhetherItThrows(threw);
    if (logged) {
      log();
    }
  }

  /** Holds the gate false because its logic threw, with what it threw as its only term. */
  private void holdFalseAfter(RuntimeException thrown) {
    holds = false;
    blocking.clear();
    String term = name + " threw: " + said(thrown);
    blocking.add(term);
    report(term, thrown);
  }

  /**
   * Reports a logic that threw. The first time, that is a Driver Station error with the stack, and
   * the alert is made. After that only the alert's text follows what was thrown last.
   */
  private void report(String term, RuntimeException thrown) {
    String text =
        term
            + ". It holds false while it throws: fix its condition (the stack is in the"
            + " Driver Station's log)";
    if (threwAlert.isEmpty()) {
      DriverStationErrors.reportError(text, thrown.getStackTrace());
      threwAlert = Optional.of(new Alert("Gates", name, text, Alert.Level.HIGH));
    } else if (!text.equals(threwText)) {
      threwAlert.get().setText(text);
    }
    threwText = text;
  }

  /** Raises the alert as the logic starts to throw, and lowers it as the logic stops. */
  private void showWhetherItThrows(boolean throwsNow) {
    boolean changed = throwsNow != throwing;
    if (threwAlert.isPresent() && changed) {
      threwAlert.get().set(throwsNow);
    }
    throwing = throwsNow;
  }

  private void log() {
    Logger.recordOutput(key, holds);
    if (!isShown(blocking)) {
      shown = blocking.toArray(String[]::new);
    }
    Logger.recordOutput(blockingKey, shown);
  }

  /** Whether these terms are the ones last logged, in the same order. */
  private boolean isShown(List<String> terms) {
    if (shown.length != terms.size()) {
      return false;
    }
    for (int i = 0; i < shown.length; i++) {
      if (!shown[i].equals(terms.get(i))) {
        return false;
      }
    }
    return true;
  }

  /** What an exception says, or its kind when it says nothing. */
  private static String said(RuntimeException thrown) {
    String message = thrown.getMessage();
    if (message == null || message.isBlank()) {
      return thrown.getClass().getSimpleName();
    }
    return message;
  }
}

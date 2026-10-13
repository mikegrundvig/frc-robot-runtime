package com.michaelgrundvig.frc.runtime.behavior;

import com.michaelgrundvig.frc.world.record.GateTerm;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
    /**
     * Once a loop, by the scheduler, before its bindings and commands read the gate. It is also
     * worked out again when it is read after its question has changed ({@link
     * Gate.Logic#question}).
     */
    EVERY_LOOP,
    /** Each time the gate is read. Nothing runs for a gate that nobody reads any more. */
    WHEN_READ
  }

  /** What is logged for a gate with nothing blocking it. */
  private static final GateTerm[] NOTHING_BLOCKING = new GateTerm[0];

  /** What is logged beside the terms of a gate none of whose terms is text. */
  private static final String[] NO_TEXT = new String[0];

  private final String name;
  private final Gate.Logic logic;
  private final String key;
  private final String blockingKey;
  private final String blockingTextKey;

  private Upkeep upkeep = Upkeep.EVERY_LOOP;

  /** Whether the gate held the last time it was worked out. */
  private boolean holds;

  /** The terms that kept it false the last time it was worked out. */
  private final Gate.Terms blocking = new Gate.Terms();

  /** The question its logic was answering the last time it was worked out. */
  private long answered;

  /**
   * Whether the gate is logged. A gate that was debounced into another is not: the debounced gate
   * is logged under the same name in its place.
   */
  private boolean logged = true;

  /**
   * How each blocking term read when the terms were last logged. The terms are logged again only
   * when they read otherwise, so that a steady gate logs, and allocates, nothing.
   */
  private final List<String> shown = new ArrayList<>();

  /** Whether the terms have been logged yet. A gate logs them the first time whatever they are. */
  private boolean termsLogged;

  /** What this gate last logged of whether it holds. */
  private enum Answer {
    NONE_YET,
    HOLDS,
    DOES_NOT_HOLD
  }

  /**
   * The answer as it was last logged. It is logged again only when it is another: the log keeps a
   * value until another replaces it, and each write makes a new value and builds the key's full
   * name, for a gate that is worked out every loop, or every time it is read.
   */
  private Answer answerLogged = Answer.NONE_YET;

  /**
   * The alert of every gate name whose logic has thrown, by that name. Gates of one name share one
   * alert, because an alert's name can be taken only once, and a command may make a gate of the
   * same name on every run. The alert is kept for good, as the gates of a robot's start are: a gate
   * made by a command has no moment at which it is done, so there is none to close it at.
   */
  private static final Map<String, Alert> THREW_ALERTS = new HashMap<>();

  /**
   * The alert that is raised while the logic throws, which is its name's shared one. This gate
   * takes it the first time its own logic throws.
   */
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
    blockingTextKey = key + "/BlockingText";
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

  /**
   * Works the gate out now, if what it last worked out is not this moment's answer. That is so for
   * a gate that is worked out as it is read, and for any gate whose question has changed since: a
   * command took the joint the gate is about, say, after the scheduler worked the gate out for the
   * loop.
   */
  void bringUpToDate() {
    if (upkeep == Upkeep.WHEN_READ || question() != answered) {
      workOut();
    }
  }

  /**
   * Which question the gate's logic is answering now ({@link Gate.Logic#question}). A logic that
   * throws as it is asked is answering the question it last answered: its gate is worked out again
   * at the next loop, where the throw is caught and reported.
   */
  long question() {
    try {
      return logic.question();
    } catch (RuntimeException e) {
      return answered;
    }
  }

  /**
   * The terms that kept the gate false the last time it was worked out. These are the terms
   * themselves, which the next working out changes.
   */
  Gate.Terms blocking() {
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
      // The question is read first, so that a question that changes while the logic runs leaves
      // the gate to be worked out again the next time it is read.
      answered = logic.question();
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
    String term = blocking.addThrew(name, said(thrown));
    report(term, thrown);
  }

  /**
   * Reports a logic that threw. The first time, that is a Driver Station error with the stack, and
   * the gate takes its name's alert. After that only the alert's text follows what was thrown last.
   */
  private void report(String term, RuntimeException thrown) {
    String text =
        term
            + ". It holds false while it throws: fix its condition (the stack is in the"
            + " Driver Station's log)";
    if (threwAlert.isEmpty()) {
      DriverStationErrors.reportError(text, thrown.getStackTrace());
      threwAlert = Optional.of(sharedAlert(name, text));
    } else if (!text.equals(threwText)) {
      threwAlert.get().setText(text);
    }
    threwText = text;
  }

  /**
   * The one alert of the gates with this name, saying this text. It is made for the first gate of
   * the name that throws, and every later one is handed the same alert.
   */
  private static Alert sharedAlert(String name, String text) {
    Alert made = THREW_ALERTS.get(name);
    if (made == null) {
      made = new Alert("Gates", name, text, Alert.Level.HIGH);
      THREW_ALERTS.put(name, made);
      return made;
    }
    made.setText(text);
    return made;
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
    Answer answer = holds ? Answer.HOLDS : Answer.DOES_NOT_HOLD;
    if (answer != answerLogged) {
      answerLogged = answer;
      Logger.recordOutput(key, holds);
    }
    List<String> reads = blocking.shown();
    if (termsLogged && shown.equals(reads)) {
      return;
    }
    termsLogged = true;
    shown.clear();
    shown.addAll(reads);
    logTerms();
  }

  /**
   * What the log holds of the terms that keep a gate false.
   *
   * @param terms each term as a fact, in the order the gate's logic named them
   * @param text the text of the terms that have some, in the same order. A term in a team's own
   *     words, and the term of a logic that threw, each point at their text by where it is here.
   */
  record Recorded(GateTerm[] terms, String[] text) {
    static final Recorded NOTHING = new Recorded(NOTHING_BLOCKING, NO_TEXT);
  }

  /** The terms that kept the gate false the last time it was worked out, as the log holds them. */
  Recorded recorded() {
    if (blocking.size() == 0) {
      return Recorded.NOTHING;
    }
    GateTerm[] terms = new GateTerm[blocking.size()];
    List<String> text = new ArrayList<>();
    for (int index = 0; index < terms.length; index++) {
      terms[index] = blocking.logged(index, text.size());
      if (blocking.hasText(index)) {
        text.add(blocking.text(index));
      }
    }
    return new Recorded(terms, text.toArray(NO_TEXT));
  }

  /** Logs the terms that keep the gate false: each as a fact, and beside them their text. */
  private void logTerms() {
    Recorded recorded = recorded();
    Logger.recordOutput(blockingKey, recorded.terms());
    Logger.recordOutput(blockingTextKey, recorded.text());
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

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.record.JointOwner;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.util.Alert;

/**
 * What commands ask of one joint, loop by loop.
 *
 * <p>A request holds for the loop it is made in and no longer. A command that wants a joint to keep
 * doing something asks again every loop, and in a loop in which nothing asks, the joint does what
 * it does idle. So a command that ends, or is canceled, needs no way to take its request back.
 */
final class JointRequests {
  /**
   * What a command asked of the joint.
   *
   * @param demand what it asked, with nothing yet holding the joint short of it
   * @param name the name of what it asked for ("out", "intaking"), or empty for a computed position
   *     or output, which has none
   * @param by the command that asked, all the way out: the one that was bound or scheduled
   *     ("Intake"). Empty when code outside any command asked.
   */
  record Request(Demand demand, Optional<String> name, Optional<Command> by) {
    /**
     * Whether this request is for exactly the same thing, by the same name, from the same asker.
     */
    boolean isFor(JointAsk ask, Optional<String> name, Optional<Command> by) {
      return demand.ask().equals(ask) && this.name.equals(name) && this.by.equals(by);
    }
  }

  private final String label;
  private final Scheduler scheduler;

  /** How near one request must be to the next to be the same request going on. */
  private final AskBands bands;

  /** Whose joint it is, which is told of each request: a new one is a new question for it. */
  private final Tenure tenure;

  /** How many loops the joint has started. The first is loop 1. */
  private long loop;

  /**
   * What a command last asked of the joint, or empty before any has. It is this loop's request only
   * if it was made in this loop ({@link #requestedInLoop}).
   */
  private Optional<Request> request = Optional.empty();

  /** The loop in which a command last asked something of the joint. */
  private long requestedInLoop;

  /**
   * The last request that was refused for not being a number, as logged in the loop it was made.
   */
  private String rejected = "";

  /** The loop in which that request was made. */
  private long rejectedInLoop;

  /**
   * The alert raised the first time the joint is asked for something that is not a number. That is
   * a bug in the code that asked, so the alert stays raised.
   */
  private Optional<Alert> rejecting = Optional.empty();

  /**
   * @param label the joint's label, which names it in what is logged
   * @param scheduler the command scheduler, which says which command is running
   * @param bands how near one request must be to the next to be the same request going on
   * @param tenure whose joint it is, which is told of each request
   */
  JointRequests(String label, Scheduler scheduler, AskBands bands, Tenure tenure) {
    this.label = label;
    this.scheduler = scheduler;
    this.bands = bands;
    this.tenure = tenure;
  }

  /** A new loop starts: nothing has been asked in it yet. */
  void newLoop() {
    loop++;
  }

  /**
   * Takes a request for this loop from the command that is running.
   *
   * <p>A request whose number is not a number (NaN, or infinite, as a computed target's division by
   * zero gives) is no request. The joint idles this loop, beneath its interlocks as ever, so it is
   * neither driven at such a number nor let past a keep-out by one. The request is logged ({@code
   * Joints/<label>/Rejected}) with who asked, and the first one raises an alert that stays raised.
   *
   * <p>A command that asks for the same thing loop after loop, as most do, keeps the request it
   * made before, so asking again allocates nothing.
   *
   * @param name the name of what is asked for ("out", "intaking"), or empty for something computed
   */
  void ask(JointAsk ask, Optional<String> name) {
    Optional<Command> by = outermostCommandRunning();
    if (!Double.isFinite(Asks.number(ask))) {
      reject(ask, by);
      return;
    }
    tenure.asked(goesOn(ask, name, by));
    boolean askedBefore = request.isPresent() && request.get().isFor(ask, name, by);
    if (!askedBefore) {
      request = Optional.of(new Request(Demand.of(ask), name, by));
    }
    requestedInLoop = loop;
  }

  /**
   * Whether a request is the last one going on: asked by the same command, by the same name, in
   * this loop or the one after the last, for the same thing or near enough ({@link AskBands}). A
   * command that follows a target asks for a little more or less every loop, and that is one
   * request. Anything else is a new request, which the joint has not yet been judged against
   * ({@link Tenure}).
   */
  private boolean goesOn(JointAsk ask, Optional<String> name, Optional<Command> by) {
    if (request.isEmpty()) {
      return false;
    }
    Request last = request.get();
    boolean followsStraightOn = loop - requestedInLoop <= 1;
    boolean sameAsker = last.by().equals(by) && last.name().equals(name);
    return followsStraightOn && sameAsker && bands.goesOn(last.demand().ask(), ask);
  }

  /** What a command asked of the joint in this loop, if one did. */
  Optional<Request> thisLoop() {
    if (requestedInLoop != loop) {
      return Optional.empty();
    }
    return request;
  }

  /** Who asked something of the joint this loop, as the log's {@code Owner} says it. */
  JointOwner owner() {
    Optional<Request> requested = thisLoop();
    if (requested.isEmpty()) {
      return JointOwner.IDLE;
    }
    boolean aCommandAsked = requested.get().by().isPresent();
    return aCommandAsked ? JointOwner.COMMAND : JointOwner.OUTSIDE_ANY_COMMAND;
  }

  /**
   * The name of the command that asked something of the joint this loop, as the log's {@code
   * Command} says it. Empty when no command asked.
   */
  String commandName() {
    Optional<Request> requested = thisLoop();
    if (requested.isEmpty() || requested.get().by().isEmpty()) {
      return "";
    }
    return requested.get().by().get().name();
  }

  /** The name of what a command asked for this loop, as the log's {@code GoalName} says it. */
  String nameShown() {
    Optional<Request> requested = thisLoop();
    if (requested.isEmpty()) {
      return "";
    }
    return requested.get().name().orElse("");
  }

  /**
   * What the joint was asked this loop that is not a number, and by whom, as logged. Empty when
   * nothing was refused this loop.
   */
  String rejected() {
    return rejectedInLoop == loop ? rejected : "";
  }

  /**
   * Refuses a request that is not a number: logs it this loop, and raises an alert the first time.
   */
  private void reject(JointAsk ask, Optional<Command> by) {
    String who = by.isPresent() ? by.get().name() : "code outside any command";
    rejected =
        who
            + " asked "
            + label
            + " for "
            + Asks.numberShown(ask)
            + " of "
            + Asks.number(ask)
            + ", which isn't a number: "
            + label
            + " idles instead; check how "
            + who
            + " works it out";
    rejectedInLoop = loop;
    if (rejecting.isEmpty()) {
      Alert alert = new Alert("Joints", label + " rejected", rejected, Alert.Level.HIGH);
      alert.set(true);
      rejecting = Optional.of(alert);
    }
  }

  /** Releases the alert, so that a test can build another robot. */
  void close() {
    rejecting.ifPresent(Alert::close);
    rejecting = Optional.empty();
  }

  /**
   * The command that is running, all the way out: not a command inside another, but the one that
   * was bound or scheduled ("Intake"). Empty when the code running is outside any command.
   */
  private Optional<Command> outermostCommandRunning() {
    Command outer = scheduler.currentCommand();
    for (Command up = outer; up != null; up = scheduler.getParentOf(up)) {
      outer = up;
    }
    return Optional.ofNullable(outer);
  }
}

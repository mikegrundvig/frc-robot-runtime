package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Value;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MoveLeg.AtOutput;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MoveLeg.AtPlace;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MoveLeg.Unplanned;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MovePlan.Follow;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MovePlan.GoTo;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MovePlan.Hold;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MovePlan.Home;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MovePlan.Run;
import com.michaelgrundvig.frc.world.config.Rules;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.Output;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Time;

/**
 * A pre-built command on the robot's joints: a v3 {@link Command} that records how it ended, as one
 * of four outcomes, each with a reason it logs under each of its joints ({@code
 * Joints/pivot/Outcome}).
 *
 * <ul>
 *   <li>{@link Outcome#ARRIVED}: inside the joint's band for 40 ms, every joint for a move to a
 *       configuration; for {@code runUntil}, its condition held; for {@code home}, it's homed.
 *   <li>{@link Outcome#TIMED_OUT}: its deadline passed. The reason says where it was, and what
 *       interlock held it.
 *   <li>{@link Outcome#CANCELED}: another command took the joint, its binding released, or its
 *       {@link #onlyWhile} gate fell.
 *   <li>{@link Outcome#REFUSED}: it wouldn't start or can't go on: a target outside the joint's
 *       limits, a place not the joint's own or an option that can't be, a joint not answering or
 *       faulted, or its {@link #onlyIf} gate false.
 * </ul>
 *
 * <p>A move that ends on its own has a deadline, from the model: twice its profile's time from
 * where it starts, plus half a second, or 3 s for {@code runUntil}; {@link #within} overrides it.
 * One that never ends on its own, {@code run}, {@code follow} or {@code hold}, is for "while held"
 * and for background work. A move on a joint that's homing waits for it, and its deadline starts
 * once it has; one on a joint whose homing was canceled homes it again first, if the robot is
 * enabled, so it waits no longer than the homing may take, and a homing that faults refuses it.
 * Recovery is an {@code if}:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=recovery}
 *
 * <p>A move can be bound to a trigger and run again and again: each run starts afresh, and its
 * outcome is its latest run's. Each run takes a loop at least, one that ends as it starts too,
 * refused or there already, so a command that awaits moves in a loop goes round once a loop. Its
 * requirements are exactly the joints it moves.
 */
@CheckReturnValue
public final class Move implements Command {
  /** Whether a run goes on after it was begun or stepped, or has ended. */
  private enum Progress {
    GOES_ON,
    ENDED
  }

  /** A deadline that never passes, in seconds: for a move that runs until it is canceled. */
  private static final double NO_DEADLINE = Double.POSITIVE_INFINITY;

  /** The command scheduler its joints are registered with, which says whether the robot starts. */
  private final Scheduler scheduler;

  private final MovePlan plan;
  private final MoveLeg[] legs;
  private final Set<Mechanism> requirements;

  // How its latest run went.

  /**
   * When the latest run started, in nanoseconds on the robot's clock: when it began, having waited
   * for its joints to home, or, until it begins, when it started to wait.
   */
  private long startNanos;

  /** When the latest run's deadline passes, or {@link Long#MAX_VALUE} for a run with none. */
  private long deadlineNanos;

  private boolean running;

  /** Whether the latest run got past its start, and so has driven its joints. */
  private boolean started;

  /** How the latest run ended, or empty before any run has ended. */
  private Optional<Outcome> outcome = Optional.empty();

  private String reason = "";

  /** When each joint got there in the latest run, in seconds from its start, as the run ended. */
  private Map<Joint, Double> arrivals = Map.of();

  private Move(Scheduler scheduler, MovePlan plan, MoveLeg[] legs) {
    this.scheduler = scheduler;
    this.plan = plan;
    this.legs = legs;
    Set<Mechanism> required = new LinkedHashSet<>();
    for (MoveLeg leg : legs) {
      required.add(leg.joint);
    }
    requirements = Set.copyOf(required);
  }

  /** A move of one joint. */
  private static Move of(MovePlan plan, MoveLeg leg) {
    return new Move(leg.joint.getRegisteredScheduler(), plan, new MoveLeg[] {leg});
  }

  // ---- Made by the joints ----

  /**
   * A move of one joint to a position.
   *
   * @param name the position's name on the joint, or empty for a computed one
   * @param where where it goes, as the move's name shows it: "out", "118°"
   * @param refused why it cannot run, if it cannot: a position the joint has no name for
   */
  static Move goTo(
      JointMechanism joint,
      double position,
      Optional<String> name,
      String where,
      Optional<String> refused) {
    MovePlan plan = MovePlan.of(new GoTo(), joint.getName() + " → " + where, refused);
    return of(plan, new MoveLeg(joint, new AtPlace(position, name)));
  }

  /**
   * A move of one joint to somewhere it cannot go, which is refused whenever it would start: a
   * place that is not the joint's own, or an angle for a joint that slides. It has nowhere to send
   * the joint, so its leg plans nothing.
   *
   * @param where where it was meant to go, as the move's name shows it
   * @param refused why it cannot run
   */
  static Move refusedGoTo(JointMechanism joint, String where, Optional<String> refused) {
    MovePlan plan = MovePlan.of(new GoTo(), joint.getName() + " → " + where, refused);
    return of(plan, new MoveLeg(joint, new Unplanned()));
  }

  /**
   * A move that runs one joint at one of its outputs.
   *
   * @param until the condition it ends on, or empty for one that runs until canceled
   * @param refused why it cannot run, if it cannot: an output the joint has no name for
   */
  static Move run(
      JointMechanism joint,
      Output output,
      Optional<BooleanSupplier> until,
      Optional<String> refused) {
    String name = joint.getName() + ": " + output.name() + untilShown(until);
    return of(MovePlan.of(new Run(until), name, refused), new MoveLeg(joint, AtOutput.of(output)));
  }

  /** The end of a move's name that says what it runs until. A gate has a name, and is named. */
  private static String untilShown(Optional<BooleanSupplier> until) {
    if (until.isEmpty()) {
      return "";
    }
    if (until.get() instanceof Gate gate) {
      return " until " + gate;
    }
    return " until";
  }

  /**
   * A move that has one joint chase a target.
   *
   * @param refused why it cannot run, if it cannot
   */
  static Move follow(
      JointMechanism joint, Supplier<? extends Measure<?>> target, Optional<String> refused) {
    MovePlan plan = MovePlan.of(new Follow(target), joint.getName() + " follows", refused);
    return of(plan, new MoveLeg(joint, new Unplanned()));
  }

  /**
   * A move that holds one joint at its goal.
   *
   * @param refused why it cannot run, if it cannot
   */
  static Move hold(JointMechanism joint, Optional<String> refused) {
    MovePlan plan = MovePlan.of(new Hold(), joint.getName() + " holds", refused);
    return of(plan, new MoveLeg(joint, new Unplanned()));
  }

  /**
   * A move that homes one joint again.
   *
   * @param refused why it cannot run, if it cannot
   */
  static Move home(JointMechanism joint, Optional<String> refused) {
    MovePlan plan = MovePlan.of(new Home(), joint.getName() + " homes", refused);
    return of(plan, new MoveLeg(joint, new Unplanned()));
  }

  /**
   * A move of a configuration's joints, each to its named position or at its named output, as the
   * running robot's joint has it. It either goes to the configuration, and ends when every joint is
   * there, or stays at it until canceled.
   *
   * <p>A configuration that names a joint the template does not drive makes a move that is refused
   * whenever it would start, saying why, so that the mistake does not stop the robot when the move
   * is made.
   *
   * @param joints the mechanism of each target's joint, in the configuration's order, or empty for
   *     a joint the template does not drive
   * @param task whether it goes to the configuration or stays at it
   * @param refused why it cannot run, if it cannot
   * @param scheduler the command scheduler the joints are registered with
   */
  static Move configuration(
      Configuration configuration,
      List<Optional<JointMechanism>> joints,
      MovePlan.Task task,
      Optional<String> refused,
      Scheduler scheduler) {
    List<MoveLeg> legs = new ArrayList<>();
    for (int i = 0; i < joints.size(); i++) {
      var target = configuration.targets().get(i);
      if (joints.get(i).isEmpty()) {
        continue;
      }
      JointMechanism joint = joints.get(i).get();
      if (refused.isPresent()) {
        // A refused move goes nowhere, and a place that is not the joint's own may not exist.
        legs.add(new MoveLeg(joint, new Unplanned()));
      } else if (target.isOutput()) {
        legs.add(new MoveLeg(joint, AtOutput.of(joint.joint().output(target.name()))));
      } else {
        double position = joint.joint().position(target.name()).value();
        legs.add(new MoveLeg(joint, new AtPlace(position, Optional.of(target.name()))));
      }
    }
    String name = (task instanceof GoTo ? "→ " : "at ") + configuration.name();
    return new Move(scheduler, MovePlan.of(task, name, refused), legs.toArray(MoveLeg[]::new));
  }

  // ---- Its options ----

  /**
   * The same move at a share of each joint's cruise speed: {@code
   * goTo(PIVOT_OUT).atSpeed(Percent.of(50))}, at half. Its default deadline grows to match. A share
   * past none or all of it, or a move that isn't to a place, is refused as a place not the joint's
   * own is ({@link JointMechanism#goTo(Position)}): thrown as the robot starts, and once it runs,
   * the move ends {@link Outcome#REFUSED} as it starts, saying why.
   *
   * @param share of its cruise: above 0%, at most 100%
   * @throws IllegalArgumentException for a share outside that, or a move with no profile, as the
   *     robot starts
   */
  public Move atSpeed(Dimensionless share) {
    double cruiseShare = share.in(Value);
    boolean isAShare = cruiseShare > 0 && cruiseShare <= 1;
    if (!isAShare) {
      return refused(
          String.format(
              Locale.ROOT,
              "a share of its cruise speed is above 0%% and at most 100%%, not %s%%",
              Rules.shown(cruiseShare * 100)));
    }
    if (!(plan.task() instanceof GoTo)) {
      return refused("it isn't a move to a place, so it has no profile to run at a share of");
    }
    String name = plan.name() + String.format(Locale.ROOT, " at %.0f%%", cruiseShare * 100);
    return with(plan.atCruiseShare(cruiseShare, name));
  }

  /**
   * The same move with a deadline of its own, tighter or looser than the model's: {@code
   * goTo(PIVOT_OUT).within(Seconds.of(1))}. A move that never ends on its own, which v3's {@code
   * withTimeout} cancels instead, is refused as {@link #atSpeed} refuses what can't be.
   *
   * @throws IllegalArgumentException for a move that never ends on its own, as the robot starts
   */
  public Move within(Time deadline) {
    if (!plan.task().endsByItself()) {
      return refused("it never ends on its own, so it has no deadline: use withTimeout");
    }
    return with(plan.within(deadline.in(Seconds)));
  }

  /**
   * The same move, started only if a gate holds as it would start; otherwise it's {@link
   * Outcome#REFUSED}, with the gate's reason: {@code pivot.goTo(PIVOT_OUT).onlyIf(intakeClear)}.
   */
  public Move onlyIf(Gate gate) {
    return with(plan.onlyIf(gate));
  }

  /**
   * The same move, for only as long as a gate holds: checked every loop, it ends {@link
   * Outcome#CANCELED} when the gate falls, with the gate's reason, and is refused if it doesn't
   * hold to start with. {@code joints.run(FEEDING).onlyWhile(readyToShoot)}.
   */
  public Move onlyWhile(Gate gate) {
    return with(plan.onlyWhile(gate));
  }

  /**
   * The same move, refused for what an option can't do ({@link Refusals}): thrown, after the move's
   * name, as the robot starts; once it runs, the move ends {@link Outcome#REFUSED} as it starts,
   * saying why it was refused first.
   *
   * @throws IllegalArgumentException with the move's name and why, as the robot starts
   */
  private Move refused(String why) {
    if (Refusals.starting(scheduler)) {
      throw new IllegalArgumentException(plan.name() + ": " + why);
    }
    return with(plan.refusedFor(why));
  }

  /** A move of the same joints by another plan, with no run behind it. */
  private Move with(MovePlan plan) {
    MoveLeg[] copies = new MoveLeg[legs.length];
    for (int i = 0; i < legs.length; i++) {
      copies[i] = legs[i].copy();
    }
    return new Move(scheduler, plan, copies);
  }

  // ---- How it ended ----

  /** Whether its latest run arrived: got there, or its condition held. */
  public boolean arrived() {
    return outcome.isPresent() && outcome.get() == Outcome.ARRIVED;
  }

  /** How its latest run ended; empty before one has, and while one runs. */
  public Optional<Outcome> outcome() {
    if (running) {
      return Optional.empty();
    }
    return outcome;
  }

  /** Why its latest run ended as it did, as logged: "pivot → out: ARRIVED in 0.41 s". */
  public String reason() {
    return reason;
  }

  /**
   * When each joint got there in its latest run, in seconds from its start: the joints inside their
   * bands as it ended, each since it came inside for good. For a configuration that arrived, every
   * joint, so the slowest one, the one to speed up, is plain; for one that timed out, those that
   * made it.
   */
  public Map<Joint, Double> arrivals() {
    return arrivals;
  }

  /**
   * When its latest run's deadline passes, in nanoseconds on the robot's clock: none ({@link
   * Long#MAX_VALUE}) while it waits for its joints to home, once it has ended, and for a move that
   * never ends on its own. For tests that check every move ends by its deadline.
   */
  long deadlineNanos() {
    return running && started ? deadlineNanos : Long.MAX_VALUE;
  }

  // ---- The command ----

  @Override
  public String name() {
    return plan.name();
  }

  @Override
  public Set<Mechanism> requirements() {
    return requirements;
  }

  /**
   * One run of the move, from start to end. It waits for its joints to home, begins, and then steps
   * once a loop until a step ends it.
   */
  @Override
  public void run(Coroutine coroutine) {
    running = true;
    started = false;
    // Until the run begins, it is timed from here: a run canceled while it still waits for its
    // joints to home then says how long it waited, and not how long ago an earlier run began.
    startNanos = now();
    outcome = Optional.empty();
    reason = "";
    boolean yielded = false;
    boolean homesTheJoint = plan.task() instanceof Home;
    if (!homesTheJoint) {
      while (waitsForHoming()) {
        coroutine.yield();
        yielded = true;
      }
    }
    Progress progress = begin();
    while (progress == Progress.GOES_ON) {
      progress = step();
      if (progress == Progress.GOES_ON) {
        coroutine.yield();
        yielded = true;
      }
    }
    if (!yielded) {
      // A run that ended as it started, refused or there already, takes its loop all the same.
      // A command that makes moves in a loop then goes round once a loop, and never without end
      // inside one.
      coroutine.yield();
    }
  }

  @Override
  public void onCancel() {
    if (!running) {
      return;
    }
    if (plan.task() instanceof Home) {
      legs[0].joint.stopHoming();
    }
    end(Outcome.CANCELED, Optional.empty());
  }

  @Override
  public String toString() {
    return plan.name();
  }

  /**
   * Whether the move must wait because one of its joints is homing, or will. Every joint is asked,
   * and none is skipped once one says yes, because asking is also what starts a canceled homing
   * again ({@link JointMechanism#homesFirst}): all of them start at once.
   */
  private boolean waitsForHoming() {
    boolean waits = false;
    for (MoveLeg leg : legs) {
      waits |= leg.joint.homesFirst();
    }
    return waits;
  }

  // ---- Beginning a run ----

  /** Begins a run. A run that is refused, or has nothing to do, ends at once. */
  private Progress begin() {
    startNanos = now();
    Optional<String> refused = whyRefused();
    if (refused.isPresent()) {
      return ended(Outcome.REFUSED, refused);
    }
    if (plan.task() instanceof Home) {
      return beginHoming();
    }
    boolean goesToItsTargets = plan.task() instanceof GoTo;
    for (MoveLeg leg : legs) {
      leg.begin(goesToItsTargets, plan.cruiseShare());
      if (!leg.runsAtAnOutput()) {
        leg.joint.beginMove();
      }
    }
    if (plan.task() instanceof Hold) {
      legs[0].beginHolding(legs[0].joint.goal());
    }
    double seconds = plan.within().orElse(modelDeadlineSeconds());
    if (Double.isInfinite(seconds)) {
      deadlineNanos = Long.MAX_VALUE;
    } else {
      deadlineNanos = startNanos + Math.round(seconds * 1e9);
    }
    started = true;
    return Progress.GOES_ON;
  }

  /**
   * Why the move cannot start, if it cannot, in the order the reasons are checked: the plan was
   * refused as it was made; a joint is faulted, or cannot reach where it is sent; or a gate the
   * move depends on does not hold.
   */
  private Optional<String> whyRefused() {
    if (plan.refused().isPresent()) {
      return plan.refused();
    }
    for (MoveLeg leg : legs) {
      Optional<String> cannot = whyRefused(leg);
      if (cannot.isPresent()) {
        return cannot;
      }
    }
    if (plan.onlyIf().isPresent() && !plan.onlyIf().get().getAsBoolean()) {
      return Optional.of(plan.onlyIf().get().why());
    }
    if (plan.onlyWhile().isPresent() && !plan.onlyWhile().get().getAsBoolean()) {
      return Optional.of(plan.onlyWhile().get().why());
    }
    return Optional.empty();
  }

  /** Why one joint's part of the move cannot start, if it cannot. */
  private Optional<String> whyRefused(MoveLeg leg) {
    if (plan.task() instanceof Home) {
      // A faulted joint may be homed again: that is what clears most faults.
      return Optional.empty();
    }
    Optional<String> fault = leg.joint.fault();
    if (fault.isPresent()) {
      return fault;
    }
    if (plan.task() instanceof GoTo && leg.part instanceof AtPlace atPlace) {
      return leg.joint.whyUnreachable(atPlace.position());
    }
    return Optional.empty();
  }

  /**
   * Begins a homing. A joint with nothing to home by is home already. The run has no deadline of
   * its own: a homing that takes too long faults the joint, and that ends the run.
   */
  private Progress beginHoming() {
    JointMechanism joint = legs[0].joint;
    boolean hasAWayToHome = joint.homesToAStop() || joint.homesByEncoder();
    if (!hasAWayToHome) {
      return ended(Outcome.ARRIVED, Optional.empty());
    }
    joint.startHoming();
    deadlineNanos = Long.MAX_VALUE;
    started = true;
    return Progress.GOES_ON;
  }

  /**
   * The deadline the model gives a run whose team gave it none, in seconds from its start: for a
   * move to a place, twice the time its slowest joint needs and half a second more ({@link
   * Outcome#deadline}); for one that runs until a condition holds, {@link
   * Outcome#RUN_UNTIL_SECONDS}; and none for one that runs until it is canceled.
   */
  private double modelDeadlineSeconds() {
    return switch (plan.task()) {
      case GoTo goTo -> Outcome.deadline(longestLegSeconds());
      case Run run -> run.endsByItself() ? Outcome.RUN_UNTIL_SECONDS : NO_DEADLINE;
      case Follow follow -> NO_DEADLINE;
      case Hold hold -> NO_DEADLINE;
      case Home home -> NO_DEADLINE;
    };
  }

  /** How long the slowest joint of a move to a place needs, in seconds, from the model. */
  private double longestLegSeconds() {
    double longest = Double.NaN;
    for (MoveLeg leg : legs) {
      double needed = leg.secondsNeeded(plan.cruiseShare());
      // The first leg has none before it to be compared with.
      longest = Double.isNaN(longest) ? needed : Math.max(longest, needed);
    }
    return longest;
  }

  // ---- Each loop of a run ----

  /**
   * One loop of a run. It first judges, on what the joints read this loop, whether the run has
   * ended. If it has not, it asks each joint for its part this loop. A run that ends asks nothing
   * more, so in the loop it ends its joints are already doing what they do idle.
   */
  private Progress step() {
    if (plan.task() instanceof Home) {
      return stepHoming();
    }
    long now = now();
    for (MoveLeg leg : legs) {
      Optional<String> fault = leg.joint.fault();
      if (fault.isPresent()) {
        return ended(Outcome.REFUSED, Optional.of("can't go on: " + fault.get()));
      }
    }
    if (plan.onlyWhile().isPresent() && !plan.onlyWhile().get().getAsBoolean()) {
      return ended(Outcome.CANCELED, Optional.of(plan.onlyWhile().get().why()));
    }
    if (plan.task() instanceof GoTo && everyLegHasArrived(now)) {
      return ended(Outcome.ARRIVED, Optional.empty());
    }
    Optional<BooleanSupplier> until = runsUntil();
    if (until.isPresent() && until.get().getAsBoolean()) {
      return ended(Outcome.ARRIVED, Optional.empty());
    }
    if (now >= deadlineNanos) {
      // A gate that never opened says why: "coral in claw is false: claw beam lost".
      Optional<String> why =
          until.isPresent() && until.get() instanceof Gate gate
              ? Optional.of(gate.why())
              : Optional.empty();
      return ended(Outcome.TIMED_OUT, why);
    }
    if (plan.task() instanceof Follow follow) {
      return askForTarget(legs[0], follow);
    }
    for (MoveLeg leg : legs) {
      leg.ask();
    }
    return Progress.GOES_ON;
  }

  /** One loop of a homing: it ends when the joint knows where it is, or is faulted. */
  private Progress stepHoming() {
    JointMechanism joint = legs[0].joint;
    if (joint.state() == JointMechanism.State.READY) {
      return ended(Outcome.ARRIVED, Optional.empty());
    }
    Optional<String> fault = joint.fault();
    if (fault.isPresent()) {
      return ended(Outcome.REFUSED, fault);
    }
    return Progress.GOES_ON;
  }

  /** The condition the move runs until, if it is one that does. */
  private Optional<BooleanSupplier> runsUntil() {
    if (plan.task() instanceof Run run) {
      return run.until();
    }
    return Optional.empty();
  }

  /**
   * Asks a joint to chase this loop's target: a position, which it takes inside its limits, or a
   * speed. A target that is neither, for this joint, ends the run refused.
   */
  private Progress askForTarget(MoveLeg leg, Follow follow) {
    JointMechanism joint = leg.joint;
    Optional<Measure<?>> target = Optional.ofNullable(follow.target().get());
    if (target.isPresent() && joint.isPosition(target.get())) {
      double position = joint.reachable(target.get().baseUnitMagnitude());
      joint.ask(new JointAsk.ToPosition(position, JointAsk.FULL_CRUISE), Optional.empty());
      return Progress.GOES_ON;
    }
    if (target.isPresent() && joint.isSpeed(target.get())) {
      joint.ask(new JointAsk.AtVelocity(target.get().baseUnitMagnitude()), Optional.empty());
      return Progress.GOES_ON;
    }
    String given = target.isPresent() ? target.get().toLongString() : "nothing";
    String takes =
        joint.joint() instanceof Joint.Revolute
            ? "an angle or an angular velocity"
            : "a distance or a linear velocity";
    return ended(
        Outcome.REFUSED,
        Optional.of(joint.getName() + " can't follow " + given + ": it takes " + takes));
  }

  /**
   * Whether every joint has arrived, as of now. Every leg is asked, and none is skipped once one
   * says no, because asking is also what notes when each joint came inside its band.
   */
  private boolean everyLegHasArrived(long now) {
    boolean all = true;
    for (MoveLeg leg : legs) {
      all &= leg.hasArrived(now);
    }
    return all;
  }

  // ---- Ending a run ----

  /**
   * Ends the run: records how it ended, logs that under each joint, and leaves each joint to what
   * it does idle.
   *
   * @param why what made it end this way, where the outcome alone does not say: a refusal's reason,
   *     or the gate that fell or never opened
   */
  private void end(Outcome how, Optional<String> why) {
    running = false;
    outcome = Optional.of(how);
    double seconds = (now() - startNanos) / 1e9;
    Map<Joint, Double> came = arrivalsAsItEnded();
    arrivals = Collections.unmodifiableMap(came);
    reason = how.reason(plan.name(), seconds, details(how, came), why);

    boolean homedTheJoint = plan.task() instanceof Home;
    boolean arrivedAtItsPlaces = how == Outcome.ARRIVED && plan.task() instanceof GoTo;
    for (MoveLeg leg : legs) {
      boolean positionedItsJoint = started && !leg.runsAtAnOutput() && !homedTheJoint;
      if (positionedItsJoint && arrivedAtItsPlaces) {
        leg.joint.moveArrived(leg.target());
      } else if (positionedItsJoint) {
        leg.joint.moveEndedShort();
      }
      leg.joint.recordOutcome(reason);
    }
  }

  /** Ends the run ({@link #end}), and says that it has ended. */
  private Progress ended(Outcome how, Optional<String> why) {
    end(how, why);
    return Progress.ENDED;
  }

  /**
   * When each joint of a move to a place got there, in seconds from the run's start: only the
   * joints that are inside their bands as the run ends.
   */
  private Map<Joint, Double> arrivalsAsItEnded() {
    Map<Joint, Double> came = new LinkedHashMap<>();
    if (!started || !(plan.task() instanceof GoTo)) {
      return came;
    }
    for (MoveLeg leg : legs) {
      if (leg.insideSince().isPresent()) {
        came.put(leg.joint.joint(), (leg.insideSince().getAsLong() - startNanos) / 1e9);
      }
    }
    return came;
  }

  /**
   * What the outcome's reason says beyond how the run ended and how long it took.
   *
   * <ul>
   *   <li>Arrived, a move of several joints says when each got there: " (pivot 0.41 s, elevator
   *       0.72 s)".
   *   <li>Timed out or canceled, it says where each joint it positioned was: ", pivot at 62°".
   *       Timed out, it also says what held each joint: ", held: pivot keeps out of 40° to 80°".
   *   <li>Refused, the refusal's own reason says it all.
   * </ul>
   */
  private String details(Outcome how, Map<Joint, Double> came) {
    StringBuilder text = new StringBuilder();
    switch (how) {
      case ARRIVED -> {
        if (legs.length > 1) {
          List<String> each = new ArrayList<>();
          came.forEach(
              (joint, at) -> each.add(String.format(Locale.ROOT, "%s %.2f s", joint.label(), at)));
          text.append(" (").append(String.join(", ", each)).append(")");
        }
      }
      case TIMED_OUT, CANCELED -> {
        boolean homedTheJoint = plan.task() instanceof Home;
        for (MoveLeg leg : legs) {
          if (!leg.runsAtAnOutput() && !homedTheJoint) {
            text.append(", ")
                .append(leg.joint.getName())
                .append(" at ")
                .append(leg.joint.joint().shown(leg.joint.position()));
          }
        }
        if (how == Outcome.TIMED_OUT) {
          for (MoveLeg leg : legs) {
            if (!leg.joint.heldBy().isEmpty()) {
              text.append(", held: ").append(leg.joint.heldBy());
            }
          }
        }
      }
      case REFUSED -> {}
    }
    return text.toString();
  }

  /**
   * The robot's time this loop, as the move's joints read it. A move of no joints, which is always
   * refused, has no joint to ask, and its time does not matter.
   */
  private long now() {
    if (legs.length == 0) {
      return 0;
    }
    return legs[0].joint.now();
  }
}

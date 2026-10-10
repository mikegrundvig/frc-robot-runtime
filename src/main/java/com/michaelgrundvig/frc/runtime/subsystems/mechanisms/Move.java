package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Value;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.world.config.Rules;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Joint;
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
import org.jspecify.annotations.Nullable;
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
  /** What a move does. */
  private enum Kind {
    /** To targets, ending when every joint is there. */
    GO_TO,
    /** At outputs or positions until canceled, or until a condition holds. */
    RUN,
    /** After a target that changes every loop. */
    FOLLOW,
    /** At the joint's goal. */
    HOLD,
    /** Homes the joint by its recipe. */
    HOME
  }

  /**
   * One joint's part in a move: a position to go to, or an output to run at. A position with no
   * name is a computed one, which a turret takes as a heading.
   */
  private static final class Leg {
    final JointMechanism joint;
    final @Nullable Output output;
    final double position;
    final @Nullable String name;

    /**
     * Where its latest run sends the joint: its position, or for a computed angle on a turret, the
     * turn of it chosen as the run starts ({@link JointMechanism#turnNear}).
     */
    double target;

    /** When the joint last came inside its band, in nanoseconds: -1 while it's outside. */
    long insideSince = -1;

    Leg(JointMechanism joint, @Nullable Output output, double position, @Nullable String name) {
      this.joint = joint;
      this.output = output;
      this.position = position;
      this.name = name;
      target = position;
    }

    Leg copy() {
      return new Leg(joint, output, position, name);
    }
  }

  /** What a move is, apart from how its latest run went: each option makes a new one. */
  private record Plan(
      Kind kind,
      String name,
      double fraction,
      double within,
      @Nullable BooleanSupplier until,
      @Nullable Supplier<? extends Measure<?>> target,
      @Nullable Gate onlyIf,
      @Nullable Gate onlyWhile,
      @Nullable String refused) {}

  /** The command scheduler its joints are registered with, which says whether the robot starts. */
  private final Scheduler scheduler;

  private final Plan plan;
  private final Kind kind;
  private final String name;
  private final Leg[] legs;
  private final Set<Mechanism> requirements;

  // How its latest run went.
  private long startNanos;
  private long deadlineNanos;
  private boolean running;

  /** Whether its latest run got past its start, and has driven its joints. */
  private boolean started;

  private @Nullable Outcome outcome;
  private String reason = "";

  /** When each joint got there in its latest run, in seconds from its start, as it ended. */
  private Map<Joint, Double> arrivals = Map.of();

  /** Where a hold holds: the joint's goal as it starts. */
  private double held;

  private Move(Scheduler scheduler, Plan plan, Leg[] legs) {
    this.scheduler = scheduler;
    this.plan = plan;
    this.kind = plan.kind();
    this.name = plan.name();
    this.legs = legs;
    Set<Mechanism> required = new LinkedHashSet<>();
    for (Leg leg : legs) {
      required.add(leg.joint);
    }
    requirements = Set.copyOf(required);
  }

  /** A move of one leg, refused as it starts if it can't be. */
  private static Move of(Kind kind, String name, @Nullable String refused, Leg leg) {
    return new Move(
        leg.joint.getRegisteredScheduler(),
        new Plan(kind, name, 1, Double.NaN, null, null, null, null, refused),
        new Leg[] {leg});
  }

  // ---- Made by the joints ----

  /**
   * @param named the name of the joint's position it goes to; null for a computed one
   * @param where where it goes, as its name shows it: "out", "118°"
   * @param refused why it can't run, or null: a position the joint has no name for
   */
  static Move goTo(
      JointMechanism joint,
      double position,
      @Nullable String named,
      String where,
      @Nullable String refused) {
    return of(
        Kind.GO_TO,
        joint.getName() + " → " + where,
        refused,
        new Leg(joint, null, position, named));
  }

  /**
   * @param refused why it can't run, or null: an output the joint has no name for
   */
  static Move run(
      JointMechanism joint,
      Output output,
      @Nullable BooleanSupplier until,
      @Nullable String refused) {
    Plan plan =
        new Plan(
            Kind.RUN,
            joint.getName()
                + ": "
                + output.name()
                + (until == null ? "" : until instanceof Gate gate ? " until " + gate : " until"),
            1,
            Double.NaN,
            until,
            null,
            null,
            null,
            refused);
    return new Move(
        joint.getRegisteredScheduler(),
        plan,
        new Leg[] {new Leg(joint, output, Double.NaN, output.name())});
  }

  /**
   * @param refused why it can't run, or null
   */
  static Move follow(
      JointMechanism joint, Supplier<? extends Measure<?>> target, @Nullable String refused) {
    Plan plan =
        new Plan(
            Kind.FOLLOW,
            joint.getName() + " follows",
            1,
            Double.NaN,
            null,
            target,
            null,
            null,
            refused);
    return new Move(
        joint.getRegisteredScheduler(), plan, new Leg[] {new Leg(joint, null, Double.NaN, null)});
  }

  /**
   * @param refused why it can't run, or null
   */
  static Move hold(JointMechanism joint, @Nullable String refused) {
    return of(
        Kind.HOLD, joint.getName() + " holds", refused, new Leg(joint, null, Double.NaN, null));
  }

  /**
   * @param refused why it can't run, or null
   */
  static Move home(JointMechanism joint, @Nullable String refused) {
    return of(
        Kind.HOME, joint.getName() + " homes", refused, new Leg(joint, null, Double.NaN, null));
  }

  /**
   * A configuration's joints, each to its named position or at its named output, where the running
   * robot's joint puts it: to it, ending when every joint is there; or held at it, until canceled.
   * One that names a joint the template doesn't drive is refused whenever it would start, saying
   * why, rather than when it's made.
   *
   * @param joints each target's joint's mechanism, in the configuration's order; null for a joint
   *     the template doesn't drive
   * @param refused why it can't run, or null
   * @param scheduler the command scheduler the joints are registered with
   */
  static Move configuration(
      Configuration configuration,
      List<@Nullable JointMechanism> joints,
      boolean toIt,
      @Nullable String refused,
      Scheduler scheduler) {
    List<Leg> legs = new ArrayList<>();
    for (int i = 0; i < joints.size(); i++) {
      var at = configuration.targets().get(i);
      JointMechanism joint = joints.get(i);
      if (joint != null) {
        legs.add(
            refused != null
                // Refused as it starts, it goes nowhere: a place not the joint's own may be none.
                ? new Leg(joint, null, Double.NaN, at.name())
                : at.isOutput()
                    ? new Leg(joint, joint.joint().output(at.name()), Double.NaN, at.name())
                    : new Leg(joint, null, joint.joint().position(at.name()).value(), at.name()));
      }
    }
    Kind kind = toIt ? Kind.GO_TO : Kind.RUN;
    String name = (toIt ? "→ " : "at ") + configuration.name();
    return new Move(
        scheduler,
        new Plan(kind, name, 1, Double.NaN, null, null, null, null, refused),
        legs.toArray(Leg[]::new));
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
    double fraction = share.in(Value);
    if (!(fraction > 0 && fraction <= 1)) {
      return refused(
          String.format(
              Locale.ROOT,
              "a share of its cruise speed is above 0%% and at most 100%%, not %s%%",
              Rules.shown(fraction * 100)));
    }
    if (kind != Kind.GO_TO) {
      return refused("it isn't a move to a place, so it has no profile to run at a share of");
    }
    return new Move(
        scheduler,
        new Plan(
            kind,
            name + String.format(Locale.ROOT, " at %.0f%%", fraction * 100),
            fraction,
            plan.within(),
            plan.until(),
            plan.target(),
            plan.onlyIf(),
            plan.onlyWhile(),
            plan.refused()),
        copies());
  }

  /**
   * The same move with a deadline of its own, tighter or looser than the model's: {@code
   * goTo(PIVOT_OUT).within(Seconds.of(1))}. A move that never ends on its own, which v3's {@code
   * withTimeout} cancels instead, is refused as {@link #atSpeed} refuses what can't be.
   *
   * @throws IllegalArgumentException for a move that never ends on its own, as the robot starts
   */
  public Move within(Time deadline) {
    if (kind != Kind.GO_TO && plan.until() == null) {
      return refused("it never ends on its own, so it has no deadline: use withTimeout");
    }
    return new Move(
        scheduler,
        new Plan(
            kind,
            name,
            plan.fraction(),
            deadline.in(Seconds),
            plan.until(),
            plan.target(),
            plan.onlyIf(),
            plan.onlyWhile(),
            plan.refused()),
        copies());
  }

  /**
   * The same move, started only if a gate holds as it would start; otherwise it's {@link
   * Outcome#REFUSED}, with the gate's reason: {@code pivot.goTo(PIVOT_OUT).onlyIf(intakeClear)}.
   */
  public Move onlyIf(Gate gate) {
    return new Move(
        scheduler,
        new Plan(
            kind,
            name,
            plan.fraction(),
            plan.within(),
            plan.until(),
            plan.target(),
            gate,
            plan.onlyWhile(),
            plan.refused()),
        copies());
  }

  /**
   * The same move, for only as long as a gate holds: checked every loop, it ends {@link
   * Outcome#CANCELED} when the gate falls, with the gate's reason, and is refused if it doesn't
   * hold to start with. {@code joints.run(FEEDING).onlyWhile(readyToShoot)}.
   */
  public Move onlyWhile(Gate gate) {
    return new Move(
        scheduler,
        new Plan(
            kind,
            name,
            plan.fraction(),
            plan.within(),
            plan.until(),
            plan.target(),
            plan.onlyIf(),
            gate,
            plan.refused()),
        copies());
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
      throw new IllegalArgumentException(name + ": " + why);
    }
    String first = plan.refused();
    return new Move(
        scheduler,
        new Plan(
            kind,
            name,
            plan.fraction(),
            plan.within(),
            plan.until(),
            plan.target(),
            plan.onlyIf(),
            plan.onlyWhile(),
            first != null ? first : why),
        copies());
  }

  private Leg[] copies() {
    Leg[] copies = new Leg[legs.length];
    for (int i = 0; i < legs.length; i++) {
      copies[i] = legs[i].copy();
    }
    return copies;
  }

  // ---- How it ended ----

  /** Whether its latest run arrived: got there, or its condition held. */
  public boolean arrived() {
    return outcome == Outcome.ARRIVED;
  }

  /** How its latest run ended; empty before one has, and while one runs. */
  public Optional<Outcome> outcome() {
    return running ? Optional.empty() : Optional.ofNullable(outcome);
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
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return requirements;
  }

  @Override
  public void run(Coroutine coroutine) {
    running = true;
    started = false;
    outcome = null;
    reason = "";
    boolean yielded = false;
    if (kind != Kind.HOME) {
      while (waitsForHoming()) {
        coroutine.yield();
        yielded = true;
      }
    }
    if (begin()) {
      while (!step()) {
        coroutine.yield();
        yielded = true;
      }
    }
    if (!yielded) {
      // Ended as it started, refused or there already: it takes its loop all the same, so a loop
      // of moves goes round once a loop, never without end in one.
      coroutine.yield();
    }
  }

  @Override
  public void onCancel() {
    if (running) {
      if (kind == Kind.HOME) {
        legs[0].joint.stopHoming();
      }
      end(Outcome.CANCELED, null);
    }
  }

  @Override
  public String toString() {
    return name;
  }

  /**
   * Whether one of its joints is homing, or will: it waits for them. A joint whose homing was
   * canceled homes again now, if the robot is enabled ({@link JointMechanism#homesFirst}), each of
   * them at once.
   */
  private boolean waitsForHoming() {
    boolean waits = false;
    for (Leg leg : legs) {
      waits |= leg.joint.homesFirst();
    }
    return waits;
  }

  /** Starts a run: false if it's refused, or done at once, which ends it. */
  boolean begin() {
    startNanos = now();
    String cannot = plan.refused();
    if (cannot != null) {
      end(Outcome.REFUSED, cannot);
      return false;
    }
    for (Leg leg : legs) {
      String refused = kind == Kind.HOME ? null : leg.joint.fault();
      if (refused == null && leg.output == null && kind == Kind.GO_TO) {
        refused = leg.joint.whyUnreachable(leg.position);
      }
      if (refused != null) {
        end(Outcome.REFUSED, refused);
        return false;
      }
    }
    Gate onlyIf = plan.onlyIf();
    if (onlyIf != null && !onlyIf.getAsBoolean()) {
      end(Outcome.REFUSED, onlyIf.why());
      return false;
    }
    Gate onlyWhile = plan.onlyWhile();
    if (onlyWhile != null && !onlyWhile.getAsBoolean()) {
      end(Outcome.REFUSED, onlyWhile.why());
      return false;
    }
    if (kind == Kind.HOME) {
      JointMechanism joint = legs[0].joint;
      if (!joint.homesToAStop() && !joint.homesByEncoder()) {
        end(Outcome.ARRIVED, null);
        return false;
      }
      joint.startHoming();
      deadlineNanos = Long.MAX_VALUE; // its recipe's timeout faults it instead
      started = true;
      return true;
    }
    double seconds = Double.NaN;
    for (Leg leg : legs) {
      leg.insideSince = -1;
      // A named position is exact: a team named that very angle. A computed one, on a turret, is
      // the turn of it nearest where the joint is now.
      leg.target =
          kind == Kind.GO_TO && leg.output == null && leg.name == null
              ? leg.joint.turnNear(leg.position, leg.joint.position())
              : leg.position;
      if (leg.output == null) {
        leg.joint.beginMove();
      }
      if (kind == Kind.GO_TO) {
        double leg_ =
            leg.output == null
                ? leg.joint.secondsTo(leg.target, plan.fraction())
                : leg.output.kind() == Output.Kind.VELOCITY ? leg.joint.secondsToSpeed() : 0;
        seconds = Double.isNaN(seconds) ? leg_ : Math.max(seconds, leg_);
      }
    }
    held = kind == Kind.HOLD ? legs[0].joint.goal() : Double.NaN;
    double deadline =
        !Double.isNaN(plan.within())
            ? plan.within()
            : kind == Kind.GO_TO
                ? Outcome.deadline(seconds)
                : plan.until() != null ? Outcome.RUN_UNTIL_SECONDS : Double.POSITIVE_INFINITY;
    deadlineNanos =
        Double.isInfinite(deadline) ? Long.MAX_VALUE : startNanos + Math.round(deadline * 1e9);
    started = true;
    return true;
  }

  /**
   * One loop: whether it has ended, judged on what the joints read this loop; if not, asks them for
   * this loop. A move that ends asks nothing more, so its last loop is already its joints' idle.
   */
  boolean step() {
    long now = now();
    if (kind == Kind.HOME) {
      JointMechanism joint = legs[0].joint;
      if (joint.state() == JointMechanism.State.READY) {
        end(Outcome.ARRIVED, null);
        return true;
      }
      String fault = joint.fault();
      if (fault != null) {
        end(Outcome.REFUSED, fault);
        return true;
      }
      return false;
    }
    for (Leg leg : legs) {
      String fault = leg.joint.fault();
      if (fault != null) {
        end(Outcome.REFUSED, "can't go on: " + fault);
        return true;
      }
    }
    Gate onlyWhile = plan.onlyWhile();
    if (onlyWhile != null && !onlyWhile.getAsBoolean()) {
      end(Outcome.CANCELED, onlyWhile.why());
      return true;
    }
    if (kind == Kind.GO_TO && everyLegThere(now)) {
      end(Outcome.ARRIVED, null);
      return true;
    }
    BooleanSupplier condition = plan.until();
    if (condition != null && condition.getAsBoolean()) {
      end(Outcome.ARRIVED, null);
      return true;
    }
    if (now >= deadlineNanos) {
      // A gate that never opened says why: "coral in claw is false: claw beam lost".
      end(Outcome.TIMED_OUT, condition instanceof Gate gate ? gate.why() : null);
      return true;
    }
    for (Leg leg : legs) {
      if (!ask(leg)) {
        return true;
      }
    }
    return false;
  }

  /** Asks a joint for its part this loop: false if the move can't, which has ended it. */
  private boolean ask(Leg leg) {
    JointMechanism joint = leg.joint;
    switch (kind) {
      case GO_TO, RUN -> {
        Output output = leg.output;
        if (output != null) {
          joint.askOutput(output);
        } else {
          joint.askPosition(leg.target, plan.fraction(), leg.name);
        }
      }
      case HOLD -> joint.askPosition(held, 1, null);
      case FOLLOW -> {
        Supplier<? extends Measure<?>> chased = plan.target();
        Measure<?> wanted = chased == null ? null : chased.get();
        if (wanted != null && joint.isPosition(wanted)) {
          joint.askPosition(joint.reachable(wanted.baseUnitMagnitude()), 1, null);
        } else if (wanted != null && joint.isSpeed(wanted)) {
          joint.askVelocity(wanted.baseUnitMagnitude());
        } else {
          end(
              Outcome.REFUSED,
              joint.getName()
                  + " can't follow "
                  + (wanted == null ? "nothing" : wanted.toLongString())
                  + ": it takes "
                  + (joint.joint() instanceof Joint.Revolute
                      ? "an angle or an angular velocity"
                      : "a distance or a linear velocity"));
          return false;
        }
      }
      case HOME -> {}
    }
    return true;
  }

  /** Whether every joint has been there for the dwell, as of now. */
  private boolean everyLegThere(long now) {
    boolean all = true;
    for (Leg leg : legs) {
      Output output = leg.output;
      boolean inside =
          output == null
              ? leg.joint.settledAt(leg.target)
              : output.kind() != Output.Kind.VELOCITY || leg.joint.atSpeed(output.value());
      if (!inside) {
        leg.insideSince = -1;
        all = false;
      } else if (leg.insideSince < 0) {
        leg.insideSince = now;
        all &= output != null && output.kind() != Output.Kind.VELOCITY;
      } else {
        all &= now - leg.insideSince >= Outcome.DWELL_NANOS;
      }
    }
    return all;
  }

  /** Ends its run: records how, logs it under each joint, and leaves each to its idle. */
  private void end(Outcome how, @Nullable String why) {
    running = false;
    outcome = how;
    double seconds = (now() - startNanos) / 1e9;
    Map<Joint, Double> came = new LinkedHashMap<>();
    if (started && kind == Kind.GO_TO) {
      for (Leg leg : legs) {
        if (leg.insideSince >= 0) {
          came.put(leg.joint.joint(), (leg.insideSince - startNanos) / 1e9);
        }
      }
    }
    arrivals = Collections.unmodifiableMap(came);
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
        for (Leg leg : legs) {
          if (leg.output == null && kind != Kind.HOME) {
            text.append(", ")
                .append(leg.joint.getName())
                .append(" at ")
                .append(leg.joint.joint().shown(leg.joint.position()));
          }
        }
        if (how == Outcome.TIMED_OUT) {
          for (Leg leg : legs) {
            if (!leg.joint.heldBy().isEmpty()) {
              text.append(", held: ").append(leg.joint.heldBy());
            }
          }
        }
      }
      case REFUSED -> {}
    }
    reason = how.reason(name, seconds, text.toString(), why);
    for (Leg leg : legs) {
      if (started && leg.output == null && kind != Kind.HOME) {
        leg.joint.moveEnded(how == Outcome.ARRIVED && kind == Kind.GO_TO, leg.target);
      }
      leg.joint.recordOutcome(reason);
    }
  }

  /** The robot's time this loop, as its joints read it: none for a move of no joints, refused. */
  private long now() {
    return legs.length == 0 ? 0 : legs[0].joint.now();
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut.Side;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Predicate;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent;

/**
 * What must hold of a robot's joints every loop, whatever its drive team presses and whatever
 * fails: checked after each loop by {@code Monkey}, each a small named check that says what it
 * found wrong, in the joint's own words, or nothing.
 *
 * <ul>
 *   <li><b>numbers only</b>: nothing that isn't a number reaches a joint's IO;
 *   <li><b>known before driven</b>: a joint held at positions that doesn't know where it is is
 *       stopped, pushed by its homing, or held where its motor says, never run to a position;
 *   <li><b>limit switches stop it</b>: nothing drives a joint on toward a limit switch that reads
 *       tripped;
 *   <li><b>kept out</b>: no joint goes into a region it's kept out of while the region is closed,
 *       past its band, apart from braking from where it was going as the region closed;
 *   <li><b>moves end</b>: every move ends with an outcome, by its deadline, and nothing a move
 *       waits on waits longer than its homing may take;
 *   <li><b>jams stall</b>: a jammed joint run at an output asking it to move is stalled, by its
 *       gate, within the gate's time;
 *   <li><b>dead channels flagged</b>: a motor that draws nothing, its PDH breaker tripped or a
 *       follower off the CAN bus, is flagged once its joint has worked its control's time without
 *       it, and none is flagged while it draws.
 * </ul>
 *
 * <p>Each judges as the robot can know, from what its joints measure and its sensors read: a fault
 * that fools the robot (an encoder slipped on its shaft and seeded again) moves where the robot
 * believes a joint is, and nothing here faults the robot for believing it. A fault on a joint
 * starts its checks afresh from where it then is ({@link #touched}).
 */
public final class JointInvariants {

  /**
   * One check: its name, and what it found wrong this loop, or null for nothing; a loop it had
   * something to judge, it says so ({@link #judging}).
   */
  private interface Check {
    String name();

    @Nullable String broken(long now, boolean enabled);
  }

  private final Joints joints;
  private final RobotMath math;
  private final Map<Joint, WatchedJoint> watched;
  private final Predicate<Joint> jammed;
  private final Predicate<CanDevice> drawsNothing;
  private final List<Check> checks = new ArrayList<>();

  /** The joints a fault reached since the last loop's checks. */
  private final Set<Joint> touched = new LinkedHashSet<>();

  /** How many loops each check had something to judge, by its name; and those this loop. */
  private final Map<String, Integer> judged = new LinkedHashMap<>();

  private final Set<String> judgedNow = new LinkedHashSet<>();

  /**
   * The checks on a robot's joints, watching its scheduler from now on.
   *
   * @param watched each driven joint's IO, as the robot's hardware handed it through
   * @param jammed whether a joint is jammed, as the simulated world has it
   * @param drawsNothing whether a motor draws nothing from the battery, as the simulated world has
   *     it: its PDH breaker tripped, or a follower off the CAN bus, neutral
   */
  public JointInvariants(
      RobotDefinition definition,
      Joints joints,
      Map<Joint, WatchedJoint> watched,
      Scheduler scheduler,
      Predicate<Joint> jammed,
      Predicate<CanDevice> drawsNothing) {
    this.joints = joints;
    this.math = definition.math();
    this.watched = Map.copyOf(watched);
    this.jammed = jammed;
    this.drawsNothing = drawsNothing;
    checks.add(new NumbersOnly());
    checks.add(new KnownBeforeDriven());
    checks.add(new LimitSwitchesStop());
    for (Interlock interlock : definition.everyInterlock()) {
      if (interlock instanceof Interlock.KeepsOut keeps) {
        checks.add(new KeptOut(keeps));
      }
    }
    MovesEnd moves = new MovesEnd();
    scheduler.addEventListener(moves::heard);
    checks.add(moves);
    checks.add(new JamsStall());
    checks.add(new DeadChannelsFlagged());
    for (Check check : checks) {
      judged.put(check.name(), 0);
    }
  }

  /**
   * How long a joint is let be after its motor controllers all answer again, or its leading one
   * restarts: what catching it takes, a joint that fell meanwhile still falling as they come back.
   */
  static final long SETTLE_NANOS = 250_000_000L;

  /** A fault reached a joint before this loop: its checks start afresh from where it is. */
  public void touched(Joint joint) {
    touched.add(joint);
  }

  /**
   * Checks the loop just run: the first thing found wrong, led by its check's name ("kept out: arm
   * ..."), or null when everything holds.
   *
   * @param now the loop's time on the robot's clock, in nanoseconds
   * @param enabled whether the robot was enabled this loop
   */
  public @Nullable String broken(long now, boolean enabled) {
    String found = null;
    for (Check check : checks) {
      String why = check.broken(now, enabled);
      if (why != null && found == null) {
        found = check.name() + ": " + why;
      }
    }
    judgedNow.forEach(name -> judged.merge(name, 1, Integer::sum));
    judgedNow.clear();
    touched.clear();
    watched.values().forEach(WatchedJoint::cleared);
    return found;
  }

  /**
   * How many loops each check had something to judge, by its name, in the order they're checked: a
   * check that never fires may never have had the chance. Every check is counted, from 0.
   */
  public Map<String, Integer> judged() {
    return new LinkedHashMap<>(judged);
  }

  /** A check had something to judge this loop: counted once a loop, however many it judged. */
  private void judging(Check check) {
    judgedNow.add(check.name());
  }

  private WatchedJoint io(Joint joint) {
    WatchedJoint io = watched.get(joint);
    if (io == null) {
      throw new IllegalArgumentException(joint.label() + "'s IO isn't watched");
    }
    return io;
  }

  /** A joint's travel, as its mechanism profiles it: its limits, and how hard it brakes. */
  private Travel travel(Joint joint) {
    return Travel.of(math, joint, ControllerSettings.control(joint));
  }

  /** How far a joint may stray from where it's held: its control's stay band. */
  private static double band(Joint joint) {
    MechanismControl.Bands kind =
        joint instanceof Joint.Revolute ? JointMechanism.ANGLE_BANDS : JointMechanism.LENGTH_BANDS;
    return ControllerSettings.control(joint).bands().orElse(kind).stay();
  }

  // ---- The checks ----

  /** Nothing that isn't a number reaches a joint's IO. */
  private final class NumbersOnly implements Check {
    @Override
    public String name() {
      return "numbers only";
    }

    @Override
    public @Nullable String broken(long now, boolean enabled) {
      for (JointMechanism joint : joints.all()) {
        for (WatchedJoint.Told told : io(joint.joint()).told()) {
          judging(this);
          if (!told.numbers()) {
            return joint.getName() + "'s IO was told " + told;
          }
        }
      }
      return null;
    }
  }

  /**
   * A joint held at positions that doesn't know where it is (unhomed, homing or faulted) is
   * stopped, pushed by its homing while it homes, or held where its motor says: a position with no
   * speed and no added volts, where its motor read it as it stopped knowing, give or take how far
   * it took to stop and its band. Never run to a position, at a speed, or at volts on a guess. A
   * later fault on the joint starts it afresh from where the joint then is, still closing on its
   * hold, perhaps, or stuck short of it: the hold it allowed stays allowed, its mechanism's own,
   * until the robot is enabled again, when it holds wherever it is.
   */
  private final class KnownBeforeDriven implements Check {
    /** Where each joint's motor read it, and how fast, as it last stopped knowing where it is. */
    private final Map<Joint, double[]> lost = new LinkedHashMap<>();

    /** Where each joint was held as it allowed, since it stopped knowing or was last enabled. */
    private final Map<Joint, Double> held = new LinkedHashMap<>();

    private boolean wasEnabled;

    @Override
    public String name() {
      return "known before driven";
    }

    @Override
    public @Nullable String broken(long now, boolean enabled) {
      String found = null;
      for (JointMechanism mechanism : joints.all()) {
        Joint joint = mechanism.joint();
        if (!joint.holdsPositions()) {
          continue;
        }
        WatchedJoint io = io(joint);
        JointMechanism.State state = mechanism.state();
        if (state == JointMechanism.State.READY) {
          lost.remove(joint);
          held.remove(joint);
          continue;
        }
        boolean afresh = !lost.containsKey(joint) || (enabled && !wasEnabled);
        if (afresh || touched.contains(joint)) {
          // Disabled meanwhile, it holds wherever it is as it's enabled.
          lost.put(joint, new double[] {io.onMotor(), io.velocity()});
        }
        if (afresh) {
          held.remove(joint);
        }
        double[] from = Objects.requireNonNull(lost.get(joint));
        Double holds = held.get(joint);
        Travel travel = travel(joint);
        double reach =
            Math.abs(travel.stoppingPoint(from[0], from[1]) - travel.clamp(from[0])) + band(joint);
        for (WatchedJoint.Told told : io.told()) {
          judging(this);
          String why =
              switch (told.kind()) {
                case "stop", "reset" -> null;
                case "home" ->
                    state == JointMechanism.State.HOMING ? null : "pushed by homing while " + state;
                case "position" -> {
                  double point = told.values()[0];
                  boolean still = told.values()[1] == 0 && told.values()[2] == 0;
                  if (still
                      && (Math.abs(point - from[0]) <= reach || Objects.equals(holds, point))) {
                    held.put(joint, point);
                    yield null;
                  }
                  yield String.format(
                      Locale.ROOT,
                      "run to a position while %s, its motor at %s as it stopped knowing",
                      mechanism.stateShown(),
                      joint.shown(from[0]));
                }
                default -> "run at a " + told.kind() + " while " + mechanism.stateShown();
              };
          if (why != null && found == null) {
            found = joint.label() + " was " + why + ": its IO was told " + told;
          }
        }
      }
      wasEnabled = enabled;
      return found;
    }
  }

  /**
   * Nothing drives a joint on toward a limit switch that reads tripped, as the robot reads it: no
   * output toward its end, no homing push toward it, and no goal past where the joint was as it
   * tripped. Leaving it is free.
   */
  private final class LimitSwitchesStop implements Check {
    /** Each tripped switch, and where its joint was as it tripped. */
    private final Map<Sensor.Switch, Double> trippedAt = new LinkedHashMap<>();

    @Override
    public String name() {
      return "limit switches stop it";
    }

    @Override
    public @Nullable String broken(long now, boolean enabled) {
      String found = null;
      for (Sensor sensor : math.sensors()) {
        if (!(sensor instanceof Sensor.Switch limit)
            || !(limit.kind() instanceof Sensor.Switch.Limit kind)
            || !watched.containsKey(kind.joint())) {
          continue;
        }
        if (!joints.sensors().holds(limit.tripped())) {
          trippedAt.remove(limit);
          continue;
        }
        JointMechanism mechanism = joints.of(kind.joint());
        double at = mechanism.position();
        double stop = trippedAt.computeIfAbsent(limit, each -> at);
        double toward = kind.end() == Sensor.Switch.Limit.End.LOWER ? -1 : 1;
        double band = band(kind.joint());
        for (WatchedJoint.Told told : io(kind.joint()).told()) {
          if (!told.kind().equals("stop") && !told.kind().equals("reset")) {
            judging(this); // driven, its switch tripped
          }
          boolean on =
              switch (told.kind()) {
                case "velocity", "voltage", "duty cycle", "home" -> told.values()[0] * toward > 0;
                case "position" ->
                    (mechanism.goal() - stop) * toward > band
                        && (mechanism.goal() - at) * toward > band;
                default -> false;
              };
          if (on && found == null) {
            found =
                kind.joint().label()
                    + " was driven on toward "
                    + limit.label()
                    + ", tripped, its goal "
                    + kind.joint().shown(mechanism.goal())
                    + ": its IO was told "
                    + told;
          }
        }
      }
      return found;
    }
  }

  /**
   * A joint doesn't go into a region it's kept out of while the region is closed, as the robot
   * judges it (the joint the condition reads unknown counts as closed), past its band. As the
   * region closes, or the joint comes to know where it is inside it, it may still go as far as
   * braking from how fast it was going takes it; once it's been outside, no further than braking
   * from how fast it goes there takes it, and once it's slowed, its band. One braking would take
   * through the region may carry on out the far side, as its interlock lets it, and is kept out
   * from there. Judged only while the robot is enabled, the joint knows where it is, and its motor
   * controllers all answer and its leading one hasn't restarted, for a settle since ({@link
   * #SETTLE_NANOS}): disabled, or with its leading controller off the bus, nothing holds a joint
   * up; with a follower off it, its motors hold less than its control counts on, so a weighed joint
   * sags under its weight, further than its band where its gains are soft.
   */
  private final class KeptOut implements Check {
    private final Interlock.KeepsOut keeps;
    private final JointMechanism joint;
    private final @Nullable JointMechanism other;
    private final Travel travel;
    private final double band;

    /** The side of the region the joint was last outside on, or empty before it ever was. */
    private Optional<Side> lastOutside = Optional.empty();

    /** Whether it was judged last loop, the region closed. */
    private boolean judging;

    /**
     * Until when it's let be, in nanoseconds: one of its motor controllers silent, or its leading
     * one restarting, and a settle after ({@link #SETTLE_NANOS}).
     */
    private long letBeUntil = Long.MIN_VALUE;

    /** Which side it's kept to, and how far past that edge it may go, while it's judged. */
    private Side keptTo = Side.ABOVE;

    private double allowed;

    KeptOut(Interlock.KeepsOut keeps) {
      this.keeps = keeps;
      joint = joints.of(keeps.joint());
      other = keeps.other().map(joints::of).orElse(null);
      travel = travel(keeps.joint());
      band = band(keeps.joint());
    }

    @Override
    public String name() {
      return "kept out";
    }

    /** Whether the region is closed, as the joint's interlock judges it this loop. */
    private boolean closed() {
      JointMechanism read = other;
      return switch (keeps.condition()) {
        case Condition.OnJoint onJoint ->
            read == null || !read.view().knowsWhere() || onJoint.holdsAt(read.view().position());
        case Condition.UpToSpeed upToSpeed ->
            read == null || !read.view().answers() || upToSpeed.holdsAt(read.view().velocity());
        case Condition.OnSensor onSensor ->
            !joints.sensors().reads(onSensor.sensor()) || joints.sensors().holds(onSensor);
      };
    }

    /** How far past the edge it's kept to a position is: negative outside. */
    private double past(double position) {
      return keeps.distancePastEdge(keptTo, position);
    }

    @Override
    public @Nullable String broken(long now, boolean enabled) {
      boolean knows = joint.view().knowsWhere();
      double at = joint.position();
      if (knows && !keeps.isInside(at)) {
        lastOutside = Optional.of(keeps.sideKeptTo(at));
      }
      WatchedJoint io = io(keeps.joint());
      if (!io.answeredThisLoop() || !io.followersConnected() || io.restarted()) {
        letBeUntil = now + SETTLE_NANOS;
      }
      boolean judged = enabled && knows && now >= letBeUntil && closed();
      if (judged) {
        judging(this);
      }
      String found = null;
      if (judged && (!judging || touched.contains(keeps.joint()))) {
        keptTo = keeps.sideKeptTo(at, lastOutside);
        double stops = past(travel.stoppingPoint(at, joint.velocity()));
        allowed = Math.max(0, Math.max(past(at), stops)) + band;
      } else if (judged) {
        if (!keeps.isInside(at) && keeps.sideKeptTo(at) != keptTo) {
          // Out the far side: too fast to stop inside as the region closed, it carried on through,
          // and is kept out from there.
          keptTo = keeps.sideKeptTo(at);
          allowed = Math.max(0, past(travel.stoppingPoint(at, joint.velocity()))) + band;
        }
        if (past(at) <= 0) {
          // Outside, it may still go as far as braking from how fast it goes takes it, and as it
          // slows, that's its band: a joint the region closed on a little short of its edge, too
          // fast to stop there, is still braking outside it the loop after.
          double stops = Math.max(0, past(travel.stoppingPoint(at, joint.velocity())));
          allowed = Math.min(allowed, stops + band);
        } else if (past(at) > allowed) {
          found =
              String.format(
                  Locale.ROOT,
                  "%s is at %s, %s past the edge it's kept to, where %s is allowed: %s",
                  joint.getName(),
                  keeps.joint().shown(at),
                  keeps.joint().shown(past(at)),
                  keeps.joint().shown(allowed),
                  keeps);
        }
      }
      judging = judged;
      return found;
    }
  }

  /**
   * Every move ends with an outcome (arrived, refused, timed out, or canceled by what took its
   * joints, its binding or its gate) and none runs on past its deadline; nothing a move waits on, a
   * joint finding where it is, takes longer while the robot is enabled than its homing may: its
   * recipe's time, or the wait for its absolute encoder's first reading.
   */
  private final class MovesEnd implements Check {
    /** The moves running now, in the order they started. */
    private final Set<Move> running = new LinkedHashSet<>();

    /** A move that ended without an outcome, said as it ended. */
    private @Nullable String endedWithout;

    /** How long each joint a move waits on has waited, enabled, in loops. */
    private final Map<Joint, Integer> waited = new LinkedHashMap<>();

    @Override
    public String name() {
      return "moves end";
    }

    void heard(SchedulerEvent event) {
      switch (event) {
        case SchedulerEvent.Mounted mounted when mounted.command() instanceof Move move ->
            running.add(move);
        case SchedulerEvent.Completed completed when completed.command() instanceof Move move -> {
          running.remove(move);
          if (move.outcome().isEmpty() && endedWithout == null) {
            endedWithout = move.name() + " ended with no outcome";
          }
        }
        case SchedulerEvent.Canceled canceled when canceled.command() instanceof Move move ->
            running.remove(move);
        case SchedulerEvent.CompletedWithError error when error.command() instanceof Move move ->
            running.remove(move);
        default -> {}
      }
    }

    @Override
    public @Nullable String broken(long now, boolean enabled) {
      String found = endedWithout;
      endedWithout = null;
      if (found != null || !running.isEmpty()) {
        judging(this);
      }
      Set<Joint> waitedOn = new LinkedHashSet<>();
      for (Move move : running) {
        long deadline = move.deadlineNanos();
        if (deadline != Long.MAX_VALUE && now >= deadline && found == null) {
          found =
              String.format(
                  Locale.ROOT,
                  "%s is still running %.2f s past its deadline",
                  move.name(),
                  (now - deadline) / 1e9);
        }
        for (var mechanism : move.requirements()) {
          if (mechanism instanceof JointMechanism joint
              && joint.state() != JointMechanism.State.READY
              && joint.state() != JointMechanism.State.FAULTED) {
            waitedOn.add(joint.joint());
          }
        }
      }
      waited.keySet().retainAll(waitedOn);
      for (Joint joint : waitedOn) {
        int loops = enabled && !touched.contains(joint) ? waited.getOrDefault(joint, 0) + 1 : 0;
        waited.put(joint, loops);
        double seconds = loops * RobotLoop.PERIOD_SECONDS;
        double allowed = homingSeconds(joint) + 2 * RobotLoop.PERIOD_SECONDS;
        if (seconds > allowed && found == null) {
          found =
              String.format(
                  Locale.ROOT,
                  "%s has been %s for %.2f s with a move waiting on it, past the %.2f s its"
                      + " homing may take",
                  joint.label(),
                  joints.of(joint).stateShown(),
                  seconds,
                  allowed);
        }
      }
      return found;
    }

    /**
     * How long a joint may take to find where it is: its homing recipe's time, or the wait for its
     * absolute encoder's first reading.
     */
    private double homingSeconds(Joint joint) {
      return switch (ControllerSettings.control(joint).homing().orElse(null)) {
        case MechanismControl.Homing.ToHardStop toStop -> toStop.secondsOn(math, joint);
        case MechanismControl.Homing.ToSwitch toSwitch -> toSwitch.secondsOn(math, joint);
        case null, default -> JointHoming.ABSOLUTE_WAIT_NANOS / 1e9;
      };
    }
  }

  /**
   * A jammed joint, run at an output asking it to move while it answers and the robot is enabled,
   * is stalled by its gate ({@link JointMechanism#isStalled}) once it has been for the gate's time,
   * and a loop's slack for the gate's own clock. Its gate reads what the joint applied the loop
   * before, so the run counts from the loop after its IO was first told the output.
   */
  private final class JamsStall implements Check {
    private final Map<Joint, Gate> stalled = new LinkedHashMap<>();

    /** Whether each joint's IO was told an output asking it to move last loop. */
    private final Map<Joint, Boolean> ran = new LinkedHashMap<>();

    /** How many loops in a row each has been jammed and run so. */
    private final Map<Joint, Integer> loops = new LinkedHashMap<>();

    JamsStall() {
      for (JointMechanism mechanism : joints.all()) {
        stalled.put(mechanism.joint(), mechanism.isStalled());
      }
    }

    @Override
    public String name() {
      return "jams stall";
    }

    @Override
    public @Nullable String broken(long now, boolean enabled) {
      String found = null;
      for (JointMechanism mechanism : joints.all()) {
        Joint joint = mechanism.joint();
        WatchedJoint io = io(joint);
        boolean run = ran.getOrDefault(joint, false) && enabled && io.answeredThisLoop();
        boolean jammedAndRun = run && jammed.test(joint);
        if (jammedAndRun) {
          judging(this);
        }
        int count = jammedAndRun ? loops.getOrDefault(joint, 0) + 1 : 0;
        loops.put(joint, count);
        ran.put(joint, asksToMove(joint, io.told()));
        double seconds = count * RobotLoop.PERIOD_SECONDS;
        Gate gate = Objects.requireNonNull(stalled.get(joint));
        if (found == null
            && seconds >= JointMechanism.STALL_SECONDS + RobotLoop.PERIOD_SECONDS
            && !gate.getAsBoolean()) {
          found =
              String.format(
                  Locale.ROOT,
                  "%s has been jammed and run at an output for %.2f s, and %s",
                  joint.label(),
                  seconds,
                  gate.why());
        }
      }
      return found;
    }

    /**
     * Whether a joint's IO was last told an output asking it to move: a speed, or volts or a share
     * of the battery, but not the volts a joint whose loop closes on its port's encoder is driven
     * to a position with.
     */
    private boolean asksToMove(Joint joint, List<WatchedJoint.Told> told) {
      if (told.isEmpty()) {
        return false;
      }
      WatchedJoint.Told last = told.get(told.size() - 1);
      boolean closesOnPort =
          joint.holdsPositions()
              && joint
                  .driven()
                  .absoluteEncoder()
                  .flatMap(Driven.AbsoluteEncoder::port)
                  .map(MechanismWiring.PortEncoder::closedLoop)
                  .orElse(false);
      return switch (last.kind()) {
        case "velocity", "duty cycle" -> last.values()[0] != 0;
        case "voltage" -> !closesOnPort && last.values()[0] != 0;
        default -> false;
      };
    }
  }

  /**
   * A motor that draws nothing from the battery, its PDH breaker tripped or a follower off the CAN
   * bus, neutral, is flagged ({@link JointMechanism#deadChannels}) once its joint has worked for
   * its control's time without it, as the robot reads it: the PDH read, its leading controller
   * answering and drawing more than its control's working current. And none is flagged while it
   * draws: in the simulator, each of a joint's motors that drives it draws alike.
   */
  private final class DeadChannelsFlagged implements Check {
    /** Since when each fed motor has drawn nothing while its joint worked. */
    private final Map<CanDevice, Long> since = new LinkedHashMap<>();

    @Override
    public String name() {
      return "dead channels flagged";
    }

    @Override
    public @Nullable String broken(long now, boolean enabled) {
      String found = null;
      Power power = joints.power();
      for (JointMechanism mechanism : joints.all()) {
        Joint joint = mechanism.joint();
        WatchedJoint io = io(joint);
        MechanismControl.DeadChannel dead = ControllerSettings.control(joint).deadChannel();
        boolean working =
            power.reads() && io.connected() && Math.abs(io.supplyAmps()) > dead.drawing().in(Amps);
        long allowed = Math.round(dead.time().in(Seconds) * 1e9);
        List<String> flagged = mechanism.deadChannels();
        for (CanDevice device : joint.devices()) {
          if (device.feed().isEmpty()) {
            continue;
          }
          int channel = device.feed().get().channel();
          boolean named = flagged.stream().anyMatch(each -> each.startsWith(device.name() + " "));
          boolean off = drawsNothing.test(device);
          long from = off && working ? since.getOrDefault(device, now) : -1;
          if (from < 0) {
            since.remove(device);
          } else {
            since.put(device, from);
          }
          if (from >= 0 || named) {
            judging(this); // drawing nothing while its joint works, or flagged
          }
          String why = null;
          if (from >= 0 && now - from >= allowed && !named) {
            why =
                String.format(
                    Locale.ROOT,
                    "%s, on channel %d, has drawn nothing for %.2f s while %s worked, and"
                        + " nothing flags it",
                    device.name(),
                    channel,
                    (now - from) / 1e9,
                    joint.label());
          } else if (!off && named) {
            why = device.name() + " is flagged while it draws: " + flagged;
          }
          if (why != null && found == null) {
            found = why;
          }
        }
      }
      return found;
    }
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.ConditionReading;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.InterlockEnforcement;
import com.michaelgrundvig.frc.world.robot.InterlockVerdict;
import com.michaelgrundvig.frc.world.robot.InterlockWorkspace;
import com.michaelgrundvig.frc.world.robot.JointMotion;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * The interlocks on one joint, enforced beneath every command.
 *
 * <p>The rules themselves are the model's: {@link InterlockEnforcement} decides what the interlocks
 * allow, as a function of plain values. This class is the part of the job that the model cannot do,
 * because the model holds no state and reads no hardware. Each loop ({@link #apply}) it gathers
 * what the conditions read, asks the model for its verdict, keeps that verdict for the next loop
 * (it carries what the interlocks remember), and writes it into the joint's {@link Demand}.
 *
 * <p>This runs for every joint with an interlock, every loop, and in nearly every loop nothing has
 * changed since the loop before. Such a loop makes no new objects. The model hands back the last
 * loop's verdict itself when its answer is the same, and everything this class hands the model each
 * loop is its own and used again: the list of readings, how the joint is moving, and the model's
 * scratch space.
 *
 * <p>A condition on another joint is read through what that joint publishes ({@link JointView}),
 * never through its mechanism. A condition that cannot be read is reported to the model as unknown,
 * and the model then restricts the joint: see {@link ConditionReading}.
 */
final class Interlocks {
  /**
   * One interlock on the joint, with what is needed to read its condition and to report it.
   *
   * @param interlock the rule
   * @param other what the joint its condition is about publishes, or empty for a condition on a
   *     sensor
   * @param shown the rule in words, as the log names what holds a joint. It is worked out once
   *     here, because building it every loop would be wasteful.
   */
  private record Enforced(Interlock interlock, Optional<JointView> other, String shown) {}

  /**
   * How the joint is moving this loop, as the model reads it. There is one of these for the joint,
   * given this loop's numbers before the model is asked, so that none is made each loop.
   */
  private static final class Motion implements JointMotion {
    private double position;
    private double velocity;
    private double acceleration;

    void is(double position, double velocity, double acceleration) {
      this.position = position;
      this.velocity = velocity;
      this.acceleration = acceleration;
    }

    @Override
    public double position() {
      return position;
    }

    @Override
    public double velocity() {
      return velocity;
    }

    @Override
    public double acceleration() {
      return acceleration;
    }
  }

  /**
   * A demand the interlocks changed, kept so that the same change is not built again every loop.
   *
   * @param asked the demand that came in
   * @param verdict what the model decided about it
   * @param left the demand that went out
   */
  private record Changed(Demand asked, InterlockVerdict verdict, Demand left) {
    /**
     * Whether this is the change for a demand and a verdict: it is when both are the very objects
     * it was built from. Neither ever changes once made, so the same two give the same demand.
     */
    boolean isFor(Demand asked, InterlockVerdict verdict) {
      return this.asked == asked && this.verdict == verdict;
    }
  }

  private final Travel travel;
  private final List<Enforced> enforced = new ArrayList<>();

  /** The model's decision for this joint's interlocks, rebuilt whenever one is added. */
  private InterlockEnforcement enforcement;

  /**
   * The model's verdict in the last loop, which carries what the interlocks remember. The model
   * hands this same object back for as long as nothing changes.
   */
  private InterlockVerdict last;

  /** What each interlock's condition reads this loop, in the order the interlocks were added. */
  private final List<ConditionReading> conditions = new ArrayList<>();

  private final Motion motion = new Motion();

  /** The model's scratch space for this joint's decisions. */
  private final InterlockWorkspace workspace = new InterlockWorkspace();

  /** The last demand the interlocks changed, or empty until they change one. */
  private Optional<Changed> lastChanged = Optional.empty();

  /**
   * @param travel where the joint is driven, and where it comes to a stop
   */
  Interlocks(Travel travel) {
    this.travel = travel;
    enforcement = new InterlockEnforcement(travel, List.of(), RobotLoop.PERIOD_SECONDS);
    last = InterlockVerdict.startingFrom(enforcement.initialState());
  }

  /**
   * Adds an interlock on the joint. Interlocks are added while the robot is being built, before its
   * first loop, so starting their memory afresh here loses nothing.
   *
   * @param other what the joint its condition is about publishes, or empty for a condition on a
   *     sensor
   */
  void add(Interlock interlock, Optional<JointView> other) {
    enforced.add(new Enforced(interlock, other, interlock.toString()));
    List<Interlock> interlocks = enforced.stream().map(Enforced::interlock).toList();
    enforcement = new InterlockEnforcement(travel, interlocks, RobotLoop.PERIOD_SECONDS);
    last = InterlockVerdict.startingFrom(enforcement.initialState());
  }

  /**
   * What the interlocks leave of what the joint is asked this loop. A demand they do not change is
   * handed back as the very same object. One they do change comes back as a position to hold or as
   * neutral, saying which interlocks hold it ({@link Demand#heldBy}), and asking for the joint's
   * profile to restart when the model says it must.
   *
   * @param demand what the joint is asked, or what it does idle
   * @param at where the joint is
   * @param velocity how fast it moves
   * @param acceleration how fast its speed changed over the last loop, a second squared
   * @param sensors the robot's sensors, as read this loop
   */
  Demand apply(Demand demand, double at, double velocity, double acceleration, Sensors sensors) {
    if (enforced.isEmpty()) {
      return demand;
    }
    readConditions(sensors);
    motion.is(at, velocity, acceleration);
    InterlockVerdict verdict =
        enforcement.decide(last, demand.ask(), motion, conditions, workspace);
    last = verdict;

    boolean unchanged = verdict.heldBy().isEmpty() && !verdict.restartProfile();
    if (unchanged) {
      return demand;
    }
    // A hold lasts many loops, each with the same demand and the same verdict, so the demand it
    // leaves is built in its first loop and handed out again in the rest.
    if (lastChanged.isPresent() && lastChanged.get().isFor(demand, verdict)) {
      return lastChanged.get().left();
    }
    Demand left = changed(demand, verdict);
    lastChanged = Optional.of(new Changed(demand, verdict, left));
    return left;
  }

  /** Reads each interlock's condition for this loop, into the list that is used every loop. */
  private void readConditions(Sensors sensors) {
    conditions.clear();
    for (int index = 0; index < enforced.size(); index++) {
      Enforced each = enforced.get(index);
      conditions.add(read(each.interlock().condition(), each.other(), sensors));
    }
  }

  /**
   * What one condition reads this loop. It is unknown when the thing it is about cannot be read: a
   * joint that does not know where it is, a joint whose motor controllers are silent, or a sensor
   * that is lost.
   */
  private static ConditionReading read(
      Condition condition, Optional<JointView> other, Sensors sensors) {
    switch (condition) {
      case Condition.OnJoint onJoint -> {
        if (other.isEmpty() || !other.get().knowsWhere()) {
          return ConditionReading.UNKNOWN;
        }
        return ConditionReading.of(onJoint.holdsAt(other.get().position()));
      }
      case Condition.UpToSpeed upToSpeed -> {
        // A joint's speed comes from its motors, so it is known whether or not the joint is homed.
        if (other.isEmpty() || !other.get().answers()) {
          return ConditionReading.UNKNOWN;
        }
        return ConditionReading.of(upToSpeed.holdsAt(other.get().velocity()));
      }
      case Condition.OnSensor onSensor -> {
        if (!sensors.reads(onSensor.sensor())) {
          return ConditionReading.UNKNOWN;
        }
        return ConditionReading.of(sensors.holds(onSensor));
      }
    }
  }

  /**
   * The demand as the verdict leaves it: doing what the model allows, and naming the interlocks
   * that hold the joint after whatever held it already.
   */
  private Demand changed(Demand demand, InterlockVerdict verdict) {
    boolean restartProfile = demand.restartProfile() || verdict.restartProfile();
    String heldBy = demand.heldBy();
    for (Interlock holding : verdict.heldBy()) {
      heldBy = Demand.joined(heldBy, shown(holding));
    }
    return new Demand(verdict.allowed(), restartProfile, heldBy);
  }

  /** An interlock in words, as it was worked out when the interlock was added. */
  private String shown(Interlock interlock) {
    for (Enforced each : enforced) {
      if (each.interlock().equals(interlock)) {
        return each.shown();
      }
    }
    throw new IllegalStateException(interlock + " is not one of this joint's interlocks");
  }
}

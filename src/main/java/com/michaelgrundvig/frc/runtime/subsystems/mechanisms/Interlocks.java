package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.ConditionReading;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.InterlockEnforcement;
import com.michaelgrundvig.frc.world.robot.InterlockReadings;
import com.michaelgrundvig.frc.world.robot.InterlockState;
import com.michaelgrundvig.frc.world.robot.InterlockVerdict;
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
 * what the conditions read, asks the model for its verdict, keeps what the interlocks remember for
 * the next loop, and writes the verdict into the joint's {@link Demand}.
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

  private final Travel travel;
  private final List<Enforced> enforced = new ArrayList<>();

  /** The model's decision for this joint's interlocks, rebuilt whenever one is added. */
  private InterlockEnforcement enforcement;

  /** What the interlocks remember from the last loop. */
  private InterlockState state;

  /**
   * @param travel where the joint is driven, and where it comes to a stop
   */
  Interlocks(Travel travel) {
    this.travel = travel;
    enforcement = new InterlockEnforcement(travel, List.of(), RobotLoop.PERIOD_SECONDS);
    state = enforcement.initialState();
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
    state = enforcement.initialState();
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
    InterlockReadings readings =
        new InterlockReadings(at, velocity, acceleration, conditionReadings(sensors));
    InterlockVerdict verdict = enforcement.decide(state, demand.ask(), readings);
    state = verdict.state();

    boolean unchanged = verdict.heldBy().isEmpty() && !verdict.restartProfile();
    if (unchanged) {
      return demand;
    }
    return changed(demand, verdict);
  }

  /** What each interlock's condition reads this loop, in the order the interlocks were added. */
  private List<ConditionReading> conditionReadings(Sensors sensors) {
    List<ConditionReading> readings = new ArrayList<>(enforced.size());
    for (Enforced each : enforced) {
      readings.add(read(each.interlock().condition(), each.other(), sensors));
    }
    return readings;
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

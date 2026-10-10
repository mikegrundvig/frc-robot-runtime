package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.ConditionReading;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.InterlockEnforcement;
import com.michaelgrundvig.frc.world.robot.InterlockReadings;
import com.michaelgrundvig.frc.world.robot.InterlockState;
import com.michaelgrundvig.frc.world.robot.InterlockVerdict;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * The interlocks on one joint, enforced beneath every command.
 *
 * <p>The rules themselves are the model's: {@link InterlockEnforcement} decides what the interlocks
 * allow, as a function of plain values. This class is the part of the job that the model cannot do,
 * because the model holds no state and reads no hardware. Each loop ({@link #apply}) it gathers
 * what the conditions read, asks the model for its verdict, keeps what the interlocks remember for
 * the next loop, and turns the verdict back into the joint's {@link Demand}.
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
   * @param other what the joint its condition is about publishes, or null for a sensor's condition
   * @param shown the rule in words, as the log names what holds a joint. It is worked out once
   *     here, because building it every loop would be wasteful.
   */
  private record Enforced(Interlock interlock, @Nullable JointView other, String shown) {}

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
   * @param other what the joint its condition is about publishes, or null for a sensor's condition
   */
  void add(Interlock interlock, @Nullable JointView other) {
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
   * @param demand what the joint is asked, or its idle
   * @param at where the joint is
   * @param velocity how fast it moves
   * @param accelerating how fast its speed changed over the last loop, a second squared
   * @param sensors the robot's sensors, as read this loop
   */
  Demand apply(
      Demand demand, double at, double velocity, double accelerating, @Nullable Sensors sensors) {
    if (enforced.isEmpty()) {
      return demand;
    }
    InterlockReadings readings =
        new InterlockReadings(at, velocity, accelerating, conditionReadings(sensors));
    InterlockVerdict verdict = enforcement.decide(state, asked(demand), readings);
    state = verdict.state();

    boolean unchanged = verdict.heldBy().isEmpty() && !verdict.restartProfile();
    if (unchanged) {
      return demand;
    }
    return changed(demand, verdict);
  }

  /** What each interlock's condition reads this loop, in the order the interlocks were added. */
  private List<ConditionReading> conditionReadings(@Nullable Sensors sensors) {
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
      Condition condition, @Nullable JointView other, @Nullable Sensors sensors) {
    switch (condition) {
      case Condition.OnJoint onJoint -> {
        if (other == null || !other.knowsWhere()) {
          return ConditionReading.UNKNOWN;
        }
        return ConditionReading.of(onJoint.holdsAt(other.position()));
      }
      case Condition.UpToSpeed upToSpeed -> {
        // A joint's speed comes from its motors, so it is known whether or not the joint is homed.
        if (other == null || !other.answers()) {
          return ConditionReading.UNKNOWN;
        }
        return ConditionReading.of(upToSpeed.holdsAt(other.velocity()));
      }
      case Condition.OnSensor onSensor -> {
        if (sensors == null || !sensors.reads(onSensor.sensor())) {
          return ConditionReading.UNKNOWN;
        }
        return ConditionReading.of(sensors.holds(onSensor));
      }
    }
  }

  /**
   * A demand in the model's terms. By the time a demand reaches the interlocks, the joint's idle
   * has turned "nothing asked" into a position to hold or into neutral, and homing's push never
   * comes this way, so neither of those has a counterpart in the model.
   */
  private static JointAsk asked(Demand demand) {
    return switch (demand.kind()) {
      case POSITION -> new JointAsk.ToPosition(demand.value(), demand.fraction());
      case VELOCITY -> new JointAsk.AtVelocity(demand.value());
      case VOLTAGE -> new JointAsk.AtVoltage(demand.value());
      case DUTY_CYCLE -> new JointAsk.AtDutyCycle(demand.value());
      case NEUTRAL -> JointAsk.NEUTRAL;
      case NONE, HOMING ->
          throw new IllegalStateException(
              "interlocks are applied to what a joint is driven at, and "
                  + demand.kind()
                  + " is not that");
    };
  }

  /** The demand as the verdict leaves it, naming the interlocks that hold the joint. */
  private Demand changed(Demand demand, InterlockVerdict verdict) {
    boolean restart = demand.restart() || verdict.restartProfile();
    String heldBy = demand.heldBy();
    for (Interlock holding : verdict.heldBy()) {
      heldBy = Demand.joined(heldBy, shown(holding));
    }
    return switch (verdict.allowed()) {
      case JointAsk.ToPosition toPosition ->
          new Demand(
              Request.POSITION, toPosition.position(), toPosition.cruiseShare(), restart, heldBy);
      // Neutral drives nothing, so nothing reads the value and the share it carries: they stay as
      // they were.
      case JointAsk.Neutral neutral ->
          new Demand(Request.NEUTRAL, demand.value(), demand.fraction(), restart, heldBy);
      // No interlock ever turns a demand into an output, so an output in the verdict is the one
      // that was asked, with only its profile restarted.
      case JointAsk.AtOutput output ->
          new Demand(demand.kind(), demand.value(), demand.fraction(), restart, heldBy);
    };
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

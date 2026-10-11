package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Sensor.Switch.Limit.End;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * A joint's limit switches, applied beneath every command and every interlock: while a switch is
 * tripped, nothing drives the joint on toward that switch's end of its travel.
 *
 * <ul>
 *   <li>A position beyond where the joint was when the switch stopped it is replaced by that point,
 *       which the joint holds.
 *   <li>An output that pushes toward the switch's end is replaced by neutral.
 *   <li>Anything that takes the joint away from the switch is left alone, so leaving a tripped
 *       switch is always free.
 * </ul>
 *
 * <p>Where the joint's motor controller reads the switch itself, the controller stops the joint
 * too, and this is a second stop beside it. Where the controller does not (a switch on a Systemcore
 * port, or on a CANdi beside a SPARK), this is the only stop.
 */
final class LimitStops {
  /**
   * One limit switch, with what is worked out about it once so that nothing is built each loop.
   *
   * @param limit the switch
   * @param tripped the condition that holds while the switch is tripped
   * @param end the end of the joint's travel the switch is at
   * @param shown what the log says while the switch holds the joint: "pivot upper limit tripped"
   */
  private record Stop(Sensor.Switch limit, Condition.Tripped tripped, End end, String shown) {
    static Stop at(Sensor.Switch limit) {
      End end = ((Sensor.Switch.Limit) limit.kind()).end();
      return new Stop(limit, limit.tripped(), end, limit.label() + " tripped");
    }

    /** Whether a value lies beyond another, toward this switch's end of the joint's travel. */
    boolean isBeyond(double value, double other) {
      return switch (end) {
        case LOWER -> value < other;
        case UPPER -> value > other;
      };
    }

    /** Whether an output, by its sign, drives the joint toward this switch's end. */
    boolean isPushedTowardBy(JointAsk.AtOutput output) {
      return isBeyond(output.value(), 0);
    }
  }

  private final List<Stop> stops;

  /**
   * Where the joint was when a tripped switch first met a position the joint was asked for, or
   * empty while no switch holds the joint. A joint held at a switch is held at this one point for
   * as long as the hold lasts: if the point followed the joint, a joint pushed past its switch
   * would be held ever further past it.
   */
  private OptionalDouble stoppedAt = OptionalDouble.empty();

  /**
   * @param switches the joint's limit switches
   */
  LimitStops(List<Sensor.Switch> switches) {
    List<Stop> each = new ArrayList<>();
    for (Sensor.Switch limit : switches) {
      each.add(Stop.at(limit));
    }
    stops = List.copyOf(each);
  }

  /**
   * What the joint's tripped limit switches leave of what it is asked this loop. Each switch that
   * holds the joint says so ("pivot upper limit tripped"). A demand that no switch changes is
   * handed back as the very same object.
   *
   * @param demand what the joint is asked, as its interlocks left it
   * @param at where the joint is
   * @param sensors the robot's sensors, as read this loop
   */
  Demand apply(Demand demand, double at, Sensors sensors) {
    Demand left = demand;
    for (Stop stop : stops) {
      if (sensors.holds(stop.tripped())) {
        left = stoppedBy(stop, left, at);
      }
    }
    // Every switch that stops the joint makes a new demand, so the same object means none did.
    boolean nothingHoldsIt = left == demand;
    if (nothingHoldsIt) {
      stoppedAt = OptionalDouble.empty();
    }
    return left;
  }

  /** What one tripped switch leaves of a demand. */
  private Demand stoppedBy(Stop stop, Demand demand, double at) {
    return switch (demand.ask()) {
      case JointAsk.ToPosition toPosition -> positionStoppedBy(stop, demand, toPosition, at);
      case JointAsk.AtOutput output -> outputStoppedBy(stop, demand, output);
      case JointAsk.Neutral neutral -> demand;
    };
  }

  /**
   * A position, with a switch tripped. If the position lies beyond where the joint was stopped,
   * toward the switch's end, the joint holds where it was stopped. The joint's profile starts again
   * from where the joint is in every loop of such a hold.
   */
  private Demand positionStoppedBy(
      Stop stop, Demand demand, JointAsk.ToPosition toPosition, double at) {
    if (stoppedAt.isEmpty()) {
      stoppedAt = OptionalDouble.of(at);
    }
    double stopped = stoppedAt.getAsDouble();
    if (!stop.isBeyond(toPosition.position(), stopped)) {
      return demand;
    }
    JointAsk hold = new JointAsk.ToPosition(stopped, toPosition.cruiseShare());
    return new Demand(hold, true, Demand.joined(demand.heldBy(), stop.shown()));
  }

  /** An output, with a switch tripped: neutral if the output pushes toward the switch's end. */
  private static Demand outputStoppedBy(Stop stop, Demand demand, JointAsk.AtOutput output) {
    if (!stop.isPushedTowardBy(output)) {
      return demand;
    }
    return demand.heldTo(JointAsk.NEUTRAL, stop.shown());
  }

  /**
   * The joint's switch at one end of its travel, if it has one there whose position is known.
   * Homing to a switch finds the joint by such a switch.
   */
  Optional<Sensor.Switch> at(End end) {
    for (Stop stop : stops) {
      Sensor.Switch.Limit kind = (Sensor.Switch.Limit) stop.limit().kind();
      if (kind.end() == end && kind.position().isPresent()) {
        return Optional.of(stop.limit());
      }
    }
    return Optional.empty();
  }
}

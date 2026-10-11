package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotMath;
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

  /**
   * A joint held where a switch stopped it, short of a position it was asked for. One hold lasts
   * for as long as the same switch stops the same demand. A different position asked for, or
   * something else coming to hold the joint above this stage, begins a new hold.
   *
   * @param by the switch that holds the joint
   * @param asked the demand that came in as the hold began
   * @param asItBegins the demand that goes out in the loop the hold begins, which restarts the
   *     joint's profile from where the joint is
   * @param asItLasts the demand that goes out in the loops after, which leaves the profile alone
   */
  private record Hold(Stop by, Demand asked, Demand asItBegins, Demand asItLasts) {
    /**
     * @param toPosition what the demand asks, a position beyond where the joint was stopped
     * @param stopped where the joint was stopped, which it holds
     */
    static Hold beginning(Stop by, Demand asked, JointAsk.ToPosition toPosition, double stopped) {
      JointAsk held = new JointAsk.ToPosition(stopped, toPosition.cruiseShare());
      String heldBy = Demand.joined(asked.heldBy(), by.shown());
      return new Hold(by, asked, new Demand(held, true, heldBy), new Demand(held, false, heldBy));
    }

    /**
     * Whether this is the hold of a demand by a switch: the same switch, the same thing asked, and
     * the same things already holding the joint. Whether the demand restarts the joint's profile is
     * left out, because that is true for a single loop and does not make the hold another one.
     */
    boolean isOf(Stop stop, Demand demand) {
      return by == stop
          && asked.ask().equals(demand.ask())
          && asked.heldBy().equals(demand.heldBy());
    }

    /**
     * The demand that goes out in a loop after the one the hold began in. It restarts the joint's
     * profile only if the demand that came in this loop does.
     */
    Demand carryingOn(Demand demand) {
      return demand.restartProfile() ? asItBegins : asItLasts;
    }
  }

  /** The joint's switches. An array, because going through a list makes an iterator each loop. */
  private final Stop[] stops;

  /**
   * Where the joint was when a tripped switch first met a position the joint was asked for, or
   * empty while no switch holds the joint. A joint held at a switch is held at this one point for
   * as long as the hold lasts: if the point followed the joint, a joint pushed past its switch
   * would be held ever further past it.
   */
  private OptionalDouble stoppedAt = OptionalDouble.empty();

  /**
   * The hold a switch has the joint in, or empty while no switch holds the joint at a position. It
   * is built in the loop the hold begins and used in every loop after that, so that a joint held on
   * a switch allocates nothing.
   */
  private Optional<Hold> hold = Optional.empty();

  /**
   * @param switches the joint's limit switches
   */
  LimitStops(List<Sensor.Switch> switches) {
    stops = new Stop[switches.size()];
    for (int index = 0; index < stops.length; index++) {
      stops[index] = Stop.at(switches.get(index));
    }
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
      hold = Optional.empty();
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
   * from where the joint is in the loop such a hold begins, and only in that loop: from then on the
   * profile carries on to the stopping point and rests there, and the joint's position loop keeps
   * the joint at it.
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
    if (hold.isPresent() && hold.get().isOf(stop, demand)) {
      return hold.get().carryingOn(demand);
    }
    Hold begun = Hold.beginning(stop, demand, toPosition, stopped);
    hold = Optional.of(begun);
    return begun.asItBegins();
  }

  /** An output, with a switch tripped: neutral if the output pushes toward the switch's end. */
  private static Demand outputStoppedBy(Stop stop, Demand demand, JointAsk.AtOutput output) {
    if (!stop.isPushedTowardBy(output)) {
      return demand;
    }
    return demand.heldTo(JointAsk.NEUTRAL, stop.shown());
  }

  /** The robot's limit switches that are one joint's. */
  static List<Sensor.Switch> switchesOf(RobotMath math, Joint joint) {
    List<Sensor.Switch> limits = new ArrayList<>();
    for (Sensor sensor : math.sensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.joint().equals(joint)) {
        limits.add(onSwitch);
      }
    }
    return limits;
  }

  /** The limit switch the joint homes to, if its homing recipe homes it to one and it has it. */
  Optional<Sensor.Switch> homingSwitch(Optional<MechanismControl.Homing> recipe) {
    if (recipe.isPresent() && recipe.get() instanceof MechanismControl.Homing.ToSwitch toSwitch) {
      return at(toSwitch.end());
    }
    return Optional.empty();
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

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import java.util.OptionalDouble;
import org.jspecify.annotations.Nullable;

/**
 * A joint's limit switches, beneath every command and interlock: while one is tripped, nothing
 * drives the joint on toward its end. A position past it holds where the joint was as it tripped;
 * an output toward it goes neutral; leaving it is free. Beside the controller's own stop where its
 * controller reads the switch, and the only stop where it doesn't (a Systemcore port, a CANdi
 * beside a SPARK).
 */
final class LimitStops {
  private final List<Sensor.Switch> switches;

  /**
   * Where the joint was as a switch it's sent toward tripped: held there while it's tripped; none
   * while none holds it.
   */
  private OptionalDouble stoppedAt = OptionalDouble.empty();

  /**
   * @param switches the joint's limit switches
   */
  LimitStops(List<Sensor.Switch> switches) {
    this.switches = List.copyOf(switches);
  }

  /**
   * What the joint's tripped limit switches leave of what it's asked this loop, each saying it
   * holds the joint ("pivot lower limit tripped"), its profile restarted from where it is as one
   * first does. A demand they leave alone is handed on as it is.
   *
   * @param demand what the joint is asked, beneath its interlocks
   * @param at where the joint is
   * @param sensors the robot's sensors, as read this loop
   */
  Demand apply(Demand demand, double at, @Nullable Sensors sensors) {
    Request asked = demand.kind();
    double target = demand.value();
    boolean restart = demand.restart();
    String heldBy = demand.heldBy();
    boolean stopped = false;
    for (Sensor.Switch limit : switches) {
      if (sensors == null || !sensors.holds(limit.tripped())) {
        continue;
      }
      double toward =
          ((Sensor.Switch.Limit) limit.kind()).end() == Sensor.Switch.Limit.End.LOWER ? -1 : 1;
      if (asked == Request.POSITION) {
        if (stoppedAt.isEmpty()) {
          stoppedAt = OptionalDouble.of(at);
        }
        double stop = stoppedAt.getAsDouble();
        if ((target - stop) * toward > 0) {
          if (!stopped && target != stop) {
            restart = true;
          }
          target = stop;
          stopped = true;
          heldBy = Demand.joined(heldBy, limit.label() + " tripped");
        }
      } else if ((asked == Request.VELOCITY
              || asked == Request.VOLTAGE
              || asked == Request.DUTY_CYCLE)
          && target * toward > 0) {
        asked = Request.NEUTRAL;
        stopped = true;
        heldBy = Demand.joined(heldBy, limit.label() + " tripped");
      }
    }
    if (!stopped) {
      stoppedAt = OptionalDouble.empty();
      return demand;
    }
    return new Demand(asked, target, demand.fraction(), restart, heldBy);
  }

  /** Its switch at an end whose position is known, which homing can find it by; or null. */
  Sensor.@Nullable Switch at(Sensor.Switch.Limit.End end) {
    for (Sensor.Switch limit : switches) {
      if (limit.kind() instanceof Sensor.Switch.Limit kind
          && kind.end() == end
          && kind.position().isPresent()) {
        return limit;
      }
    }
    return null;
  }
}

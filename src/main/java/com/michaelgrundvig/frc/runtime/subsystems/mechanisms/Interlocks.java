package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import org.jspecify.annotations.Nullable;

/**
 * The interlocks on one joint, from the robot's definition, beneath every command: what they leave
 * of what the joint is asked, each loop ({@link #apply}). While a {@code runsOnlyWhen} condition
 * doesn't hold, the joint holds where it comes to a stop, or stops; while a {@code keepsOut} region
 * is closed, its target is held at the region's edge on the side it's on, so its profile stops it
 * there. A condition on a joint that doesn't know where it is (unhomed, faulted, silent), or on a
 * sensor that's lost, fails safe: it restricts. The command keeps its goal, and carries on when the
 * interlock lets it.
 *
 * <p>Another joint is read through what it publishes ({@link JointView}), never its mechanism. What
 * each interlock remembers is its own: for a region, the side the joint was last on and where it's
 * held inside; and for the joint, where a {@code runsOnlyWhen} holds it.
 */
final class Interlocks {
  /**
   * An interlock on the joint, the joint its condition reads (none for a sensor's), and for a
   * region it keeps the joint out of, the side the joint was last on: the only way it may leave the
   * region, should it ever be inside; where it's held inside, once; and whether an output is held
   * at its edge.
   */
  private static final class Restriction {
    final Interlock interlock;
    final @Nullable JointView other;
    final String shown;

    /** Below the region (-1) or above it (1) when last outside it; 0 before it ever was. */
    int side;

    /**
     * Where the joint is held inside the region, worked out once as the hold starts; none while it
     * isn't: worked out again each loop from how fast it's still going, the point would run ahead
     * of a joint its loop hasn't yet slowed, and the joint after it through the region.
     */
    OptionalDouble inside = OptionalDouble.empty();

    /**
     * Whether the joint, run at an output, is held at the region's edge: from the loop it could no
     * longer stop short of it, for as long as the output pushes it toward the region. Judged again
     * each loop, a joint come to rest at the edge would be let go, pushed on, and across.
     */
    boolean atEdge;

    Restriction(Interlock interlock, @Nullable JointView other) {
      this.interlock = interlock;
      this.other = other;
      shown = interlock.toString();
    }
  }

  private final Travel travel;

  /** Whether the joint holds positions, so a {@code runsOnlyWhen} holds it rather than stops it. */
  private final boolean positional;

  private final List<Restriction> restrictions = new ArrayList<>();

  /**
   * Where a {@code runsOnlyWhen} holds the joint, worked out once as the hold starts; none while
   * none does.
   */
  private OptionalDouble heldAt = OptionalDouble.empty();

  /**
   * @param travel where the joint is driven, and where it comes to a stop
   * @param positional whether it holds positions
   */
  Interlocks(Travel travel, boolean positional) {
    this.travel = travel;
    this.positional = positional;
  }

  /**
   * An interlock on the joint.
   *
   * @param other what the joint its condition reads publishes; null for a sensor's condition
   */
  void add(Interlock interlock, @Nullable JointView other) {
    restrictions.add(new Restriction(interlock, other));
  }

  /**
   * What the interlocks leave of what the joint is asked this loop, each in turn: held, stopped or
   * kept out, saying which hold it ({@link Demand#heldBy}), its profile restarted from where it is
   * as a {@code runsOnlyWhen} or a region it's inside holds it, and as a {@code runsOnlyWhen} lets
   * go. A demand they leave alone is handed on as it is.
   *
   * @param demand what the joint is asked, or its idle
   * @param at where the joint is
   * @param velocity how fast it moves
   * @param accelerating how fast its speed changed over the last loop, a second squared
   * @param sensors the robot's sensors, as read this loop
   */
  Demand apply(
      Demand demand, double at, double velocity, double accelerating, @Nullable Sensors sensors) {
    Request asked = demand.kind();
    double target = demand.value();
    double fraction = demand.fraction();
    String heldBy = demand.heldBy();
    boolean frozen = false;
    boolean holdStarts = false;
    for (Restriction restriction : restrictions) {
      Interlock interlock = restriction.interlock;
      JointView other = restriction.other;
      if (interlock instanceof Interlock.KeepsOut keeps) {
        // Which side of the region the joint is on, whether the interlock is closed or not. An
        // edge at the range's end has no side past it: a joint there is inside.
        if (!keeps.keepsOut(at)) {
          restriction.side = keeps.keptBelow(at, 0) ? -1 : 1;
        }
      }
      // Whether what it speaks of is known, and holds: a joint that knows where it is, or a
      // sensor that answers. Unknown, it restricts: kept out, or held.
      boolean known;
      boolean condition;
      switch (interlock.condition()) {
        case Condition.OnJoint onJoint -> {
          known = other != null && other.knowsWhere();
          condition = known && other != null && onJoint.holdsAt(other.position());
        }
        case Condition.UpToSpeed upToSpeed -> {
          // How fast it runs is known from its motors, homed or not.
          known = other != null && other.answers();
          condition = known && other != null && upToSpeed.holdsAt(other.velocity());
        }
        case Condition.OnSensor onSensor -> {
          known = sensors != null && sensors.reads(onSensor.sensor());
          condition = known && sensors != null && sensors.holds(onSensor);
        }
      }
      switch (interlock) {
        case Interlock.RunsOnlyWhen runs -> {
          if (!condition) {
            if (positional) {
              if (heldAt.isEmpty()) {
                heldAt = OptionalDouble.of(travel.stoppingPoint(at, velocity));
                holdStarts = true;
              }
              asked = Request.POSITION;
              target = heldAt.getAsDouble();
              fraction = 1;
              frozen = true;
            } else {
              asked = Request.NEUTRAL;
            }
            heldBy = Demand.joined(heldBy, restriction.shown);
          }
        }
        case Interlock.KeepsOut keeps -> {
          if (!known || condition) {
            boolean heldInside = restriction.inside.isPresent();
            double kept = keptOut(keeps, restriction, asked, target, at, velocity, accelerating);
            holdStarts |= !heldInside && restriction.inside.isPresent();
            // Only a region that moves its target holds it, at its cruise; a target it leaves
            // alone keeps its command's share of the cruise.
            if (!Double.isNaN(kept) && (asked != Request.POSITION || kept != target)) {
              heldBy = Demand.joined(heldBy, restriction.shown);
              asked = Request.POSITION;
              target = kept;
              fraction = 1;
            }
          } else {
            restriction.inside = OptionalDouble.empty();
            restriction.atEdge = false;
          }
        }
      }
    }
    // Let go: the command's goal again, its profile from where the joint is.
    boolean letGo = !frozen && heldAt.isPresent();
    if (letGo) {
      heldAt = OptionalDouble.empty();
    }
    // As a hold starts, its profile starts from where the joint is, braking it to where it stops:
    // from where it had the joint, ahead of one that lagged it, it would carry the joint on past.
    boolean restart = letGo || holdStarts;
    if (asked == demand.kind()
        && target == demand.value()
        && fraction == demand.fraction()
        && heldBy.equals(demand.heldBy())
        && !restart) {
      return demand;
    }
    return new Demand(asked, target, fraction, demand.restart() || restart, heldBy);
  }

  /**
   * Where a {@code keepsOut} interlock that's closed holds the joint, or NaN when it doesn't: a
   * position at the region's edge on the side the joint is on, which its profile stops at. A joint
   * inside the region, one that overshot an edge or was there when it closed, may leave it only the
   * way it came, toward a goal outside on the side it was last on (the nearer side, if it was never
   * outside), and otherwise holds where it would stop as the hold starts, braking at its profile's
   * acceleration. A joint the region closes on while it's moving too fast to stop inside it carries
   * on out the far side and is then kept out from there: it never stops inside the region. Run at
   * an output, it's held at the edge, a position its profile brakes it to from where it is, from
   * the loop before it could no longer stop short of it: once where it would stop, held a loop on,
   * would reach the region ({@link Travel#stoppingPointLoopOn}), going on as it goes or at the
   * speed the output asks for, which a light joint reaches within a loop. Held then, its profile
   * stops it at the edge at its acceleration, as every move of its brakes; and it's held there for
   * as long as the output pushes it toward the region.
   *
   * @param restriction the interlock's: the side the joint was last on, and where it's held inside
   * @param accelerating how fast its speed changed over the last loop, a second squared
   */
  private double keptOut(
      Interlock.KeepsOut keeps,
      Restriction restriction,
      Request asked,
      double target,
      double at,
      double velocity,
      double accelerating) {
    double from = keeps.region().lower();
    double to = keeps.region().upper();
    // Out of the region, it's held on its side; inside, it may only leave, by the side it's kept
    // to. A position is judged where its profile takes it: the short way round from where the
    // joint is, for one that turns freely, so -170° from 90° is 190°.
    boolean inside = keeps.keepsOut(at);
    boolean below = keeps.keptBelow(at, restriction.side);
    double goal = asked == Request.POSITION ? travel.goalNear(target, at) : target;
    boolean leaving = asked == Request.POSITION && (below ? goal <= from : goal >= to);
    if (!inside || leaving || asked == Request.NEUTRAL) {
      restriction.inside = OptionalDouble.empty();
    }
    if (inside || asked == Request.POSITION || asked == Request.NEUTRAL) {
      restriction.atEdge = false;
    }
    if (asked == Request.POSITION) {
      if (!inside) {
        boolean past = below ? goal > from : goal < to;
        return past ? (below ? from : to) : target;
      }
      return leaving ? target : heldInside(restriction, at, velocity);
    }
    if (asked == Request.NEUTRAL) {
      return Double.NaN;
    }
    if (!inside) {
      // Where it would stop, held a loop on: going on as it goes now, or at the speed the output
      // asks for, which a light joint's motors reach within the loop, from rest as from moving.
      double stopping =
          travel.stoppingPointLoopOn(at, velocity, accelerating, RobotLoop.PERIOD_SECONDS);
      double asks =
          travel.stoppingPointLoopOn(at, speedAsked(asked, target), 0, RobotLoop.PERIOD_SECONDS);
      boolean reaches = below ? Math.max(stopping, asks) >= from : Math.min(stopping, asks) <= to;
      // A speed, volts or a share of the battery pushes it toward the region by its sign.
      boolean toward = below ? target > 0 : target < 0;
      restriction.atEdge = reaches || (restriction.atEdge && toward);
      if (!restriction.atEdge) {
        return Double.NaN;
      }
      return below ? from : to;
    }
    return heldInside(restriction, at, velocity);
  }

  /**
   * The speed an output asks for, in the joint's units a second: a speed's own, or what a voltage
   * or a share of the battery runs the joint at, free; none for anything else.
   */
  private double speedAsked(Request asked, double target) {
    return switch (asked) {
      case VELOCITY -> target;
      case VOLTAGE -> target / Motor.NOMINAL_VOLTS * travel.freeSpeed();
      case DUTY_CYCLE -> target * travel.freeSpeed();
      default -> 0;
    };
  }

  /**
   * Where a joint inside a region it's kept out of is held: where it would stop, worked out once as
   * the hold starts.
   */
  private double heldInside(Restriction restriction, double at, double velocity) {
    if (restriction.inside.isEmpty()) {
      restriction.inside = OptionalDouble.of(travel.stoppingPoint(at, velocity));
    }
    return restriction.inside.getAsDouble();
  }
}

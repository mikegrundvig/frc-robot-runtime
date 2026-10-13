package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.world.record.Identity;
import com.michaelgrundvig.frc.world.record.MoveTarget;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Position;
import java.util.List;
import java.util.Optional;
import java.util.OptionalLong;

/**
 * One joint's part in a {@link Move}: the joint, what the move has it do, and how that is going in
 * the move's latest run.
 */
final class MoveLeg {
  /** What a move has one joint do. */
  sealed interface Part {}

  /**
   * Be at a position.
   *
   * @param position the position, in the joint's units
   * @param name the position's name on the joint, or empty for a computed one. A named position is
   *     exact, because a team named that very angle. A computed one is taken by a turret as a
   *     heading, and the joint goes to whichever turn of it is nearest.
   */
  record AtPlace(double position, Optional<String> name) implements Part {}

  /**
   * Run at one of the joint's named outputs.
   *
   * @param output the output
   * @param ask the output as the joint is asked for it, worked out once
   * @param name the output's name, as the joint is told what it is asked for
   */
  record AtOutput(Output output, JointAsk ask, Optional<String> name) implements Part {
    static AtOutput of(Output output) {
      return new AtOutput(output, Asks.running(output), Optional.of(output.name()));
    }

    /** Whether the output is a speed, which takes time to reach. Any other is reached at once. */
    boolean isASpeed() {
      return output.kind() == Output.Kind.VELOCITY;
    }
  }

  /**
   * Nothing fixed in advance. The move works out each loop what to ask of the joint (a follow, a
   * hold), or asks nothing of it (a homing, or a move that is refused as it starts).
   */
  record Unplanned() implements Part {}

  /**
   * What a move sent a joint to or ran it at, as the move's outcome records it.
   *
   * @param kind what kind of thing it is
   * @param index for a named position or a named output, where it is in the joint's own list of
   *     them; {@link Identity#NONE} for anything else
   * @param number the target as a number, in the units its kind has, or NaN where it has none
   */
  record Target(MoveTarget kind, int index, double number) {
    /** Nothing the outcome can say: a hold, a homing, or a move refused for where it would go. */
    static final Target NONE = new Target(MoveTarget.NONE, Identity.NONE, Double.NaN);

    /** Stopped, the output every joint has. */
    static final Target STOPPED = new Target(MoveTarget.STOPPED, Identity.NONE, 0);
  }

  final JointMechanism joint;
  final Part part;

  /**
   * Where the leg's named position or named output is in the joint's own list of them, or {@link
   * Identity#NONE} for a leg that has neither. It is looked up once, as the leg is made.
   */
  private final int indexOfItsName;

  // How the latest run is going.

  /**
   * Where this run sends the joint, for a leg that goes to a place: the place's position, or, for a
   * computed angle on a turret, the turn of it that was nearest as the run started.
   */
  private double target;

  /** What this run asks of the joint each loop, for a leg that asks the same thing every loop. */
  private JointAsk ask = JointAsk.NEUTRAL;

  /** When the joint last came inside its band, in nanoseconds, or empty while it is outside. */
  private OptionalLong insideSince = OptionalLong.empty();

  MoveLeg(JointMechanism joint, Part part) {
    this.joint = joint;
    this.part = part;
    target = part instanceof AtPlace atPlace ? atPlace.position() : Double.NaN;
    indexOfItsName = indexOfItsName(joint, part);
  }

  /** Where a part's named position or named output is in the joint's own list of them. */
  private static int indexOfItsName(JointMechanism joint, Part part) {
    return switch (part) {
      case AtPlace atPlace -> {
        List<Position> positions = joint.joint().positions();
        for (int index = 0; index < positions.size(); index++) {
          boolean isItsName =
              atPlace.name().isPresent()
                  && positions.get(index).name().equals(atPlace.name().get());
          if (isItsName) {
            yield index;
          }
        }
        yield Identity.NONE;
      }
      case AtOutput atOutput -> joint.joint().outputs().indexOf(atOutput.output());
      case Unplanned unplanned -> Identity.NONE;
    };
  }

  /** What this run sent the joint to or ran it at, as the move's outcome records it. */
  Target recordedTarget() {
    return switch (part) {
      case AtPlace atPlace -> {
        MoveTarget kind = atPlace.name().isPresent() ? MoveTarget.PLACE : MoveTarget.NUMBER;
        yield new Target(kind, indexOfItsName, target);
      }
      case AtOutput atOutput -> {
        boolean isOneOfItsOwn = Identity.isSome(indexOfItsName);
        if (!isOneOfItsOwn) {
          // The only output a joint runs at that is not in its list is the one every joint has.
          yield Target.STOPPED;
        }
        yield new Target(MoveTarget.OUTPUT, indexOfItsName, atOutput.output().value());
      }
      case Unplanned unplanned -> Target.NONE;
    };
  }

  /** The same leg with no run behind it, for another move made from the same plan. */
  MoveLeg copy() {
    return new MoveLeg(joint, part);
  }

  /** Whether the leg runs its joint at an output, where every other leg positions its joint. */
  boolean runsAtAnOutput() {
    return part instanceof AtOutput;
  }

  /** Where this run sends the joint, for a leg that goes to a place. */
  double target() {
    return target;
  }

  /** When the joint last came inside its band in this run, or empty while it is outside. */
  OptionalLong insideSince() {
    return insideSince;
  }

  /**
   * Starts a run of a leg that is at a place or at an output: settles where the joint is sent and
   * what it is asked each loop.
   *
   * @param nearestTurn whether a computed angle is taken as a heading, at the turn of it nearest
   *     where the joint is now: so for a move that goes to it, and not for one that stays at it
   * @param cruiseShare the share of the joint's cruise speed it is profiled at
   */
  void begin(boolean nearestTurn, double cruiseShare) {
    insideSince = OptionalLong.empty();
    switch (part) {
      case AtPlace atPlace -> {
        boolean computed = atPlace.name().isEmpty();
        if (nearestTurn && computed) {
          target = joint.turnNear(atPlace.position(), joint.position());
        } else {
          target = atPlace.position();
        }
        ask = new JointAsk.ToPosition(target, cruiseShare);
      }
      case AtOutput atOutput -> ask = atOutput.ask();
      case Unplanned unplanned -> {}
    }
  }

  /** Starts a run of a leg that holds its joint at a position for the whole run. */
  void beginHolding(double position) {
    ask = new JointAsk.ToPosition(position, JointAsk.FULL_CRUISE);
  }

  /**
   * How long the joint needs to get where a move to a place sends it, in seconds, from the model:
   * its profile's time to a position, or the time its velocity loop takes to reach a speed. A
   * voltage or a share of the battery is there at once.
   */
  double secondsNeeded(double cruiseShare) {
    return switch (part) {
      case AtPlace atPlace -> joint.secondsTo(target, cruiseShare);
      case AtOutput atOutput -> atOutput.isASpeed() ? joint.secondsToSpeed() : 0;
      case Unplanned unplanned -> 0;
    };
  }

  /** Asks the joint, for this loop, for what this run has it do. */
  void ask() {
    switch (part) {
      case AtPlace atPlace -> joint.ask(ask, atPlace.name());
      case AtOutput atOutput -> joint.ask(ask, atOutput.name());
      case Unplanned unplanned -> joint.ask(ask, Optional.empty());
    }
  }

  /**
   * Notes whether the joint is where the move sends it as of this loop, and says whether it has
   * been there long enough to count as arrived.
   *
   * <p>A joint must stay inside its band for {@link Outcome#DWELL_NANOS}, so that one passing
   * through its target is not taken to have arrived. A voltage or a share of the battery has no
   * band to settle into, so a joint run at one has arrived in its first loop.
   */
  boolean hasArrived(long now) {
    if (!isThere()) {
      insideSince = OptionalLong.empty();
      return false;
    }
    if (insideSince.isEmpty()) {
      insideSince = OptionalLong.of(now);
      return part instanceof AtOutput atOutput && !atOutput.isASpeed();
    }
    return now - insideSince.getAsLong() >= Outcome.DWELL_NANOS;
  }

  /** Whether the joint is where the move sends it, as of this loop. */
  private boolean isThere() {
    return switch (part) {
      case AtPlace atPlace -> joint.settledAt(target);
      case AtOutput atOutput -> !atOutput.isASpeed() || joint.atSpeed(atOutput.output().value());
      case Unplanned unplanned -> false;
    };
  }
}

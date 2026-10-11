package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SwitchDoubt;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.OptionalLong;

/**
 * Checks each of a joint's limit switches against where the joint is, once the joint knows where it
 * is. The model says when the two disagree ({@link SwitchDoubt}): a switch that reads tripped far
 * from where it trips, say, as one with a cut wire does. A switch that has disagreed for a quarter
 * of a second is doubted.
 *
 * <p>A doubt is only said, in the log and to the drivers. It is never acted on: a doubted switch
 * still stops the joint, because stopping is the safe way to be wrong.
 */
final class SwitchDoubts {
  /** How long a switch must disagree with where the joint is before it is doubted. */
  static final long DOUBT_NANOS = 250_000_000L;

  private final Joint joint;
  private final List<Sensor.Switch> switches;

  /** The condition that holds while each switch is tripped, in the switches' order. */
  private final List<Condition.Tripped> tripped;

  /**
   * What each switch is doubted for, in the switches' order, as the log shows it: an empty text for
   * a switch that is not doubted.
   */
  private final String[] doubts;

  /** Since when each switch has disagreed, in nanoseconds, or empty while it agrees. */
  private final OptionalLong[] disagreesSince;

  /**
   * @param joint the joint, which says where it is in its own units
   * @param switches the joint's limit switches
   */
  SwitchDoubts(Joint joint, List<Sensor.Switch> switches) {
    this.joint = joint;
    this.switches = List.copyOf(switches);
    tripped = switches.stream().map(Sensor.Switch::tripped).toList();
    doubts = new String[switches.size()];
    Arrays.fill(doubts, "");
    disagreesSince = new OptionalLong[switches.size()];
    Arrays.fill(disagreesSince, OptionalLong.empty());
  }

  /**
   * Checks each switch for this loop.
   *
   * @param now this loop's time, in nanoseconds
   * @param knowsWhere whether the joint knows where it is and its motor controllers answer. While
   *     it does not, there is nothing to check a switch against, and no switch is doubted.
   * @param at where the joint is
   * @param sensors the robot's sensors, as read this loop
   */
  void check(long now, boolean knowsWhere, double at, Sensors sensors) {
    for (int i = 0; i < switches.size(); i++) {
      Optional<String> disagreement = disagreement(i, knowsWhere, at, sensors);
      if (disagreement.isEmpty()) {
        disagreesSince[i] = OptionalLong.empty();
        doubts[i] = "";
        continue;
      }
      if (disagreesSince[i].isEmpty()) {
        disagreesSince[i] = OptionalLong.of(now);
        continue;
      }
      boolean disagreedLongEnough = now - disagreesSince[i].getAsLong() >= DOUBT_NANOS;
      if (disagreedLongEnough) {
        doubts[i] = disagreement.get();
      }
    }
  }

  /**
   * How a switch disagrees with where the joint is this loop, in the model's words, if it does. A
   * switch that is lost reads nothing, and so cannot disagree.
   */
  private Optional<String> disagreement(int index, boolean knowsWhere, double at, Sensors sensors) {
    Sensor.Switch limit = switches.get(index);
    if (!knowsWhere || !sensors.reads(limit)) {
      return Optional.empty();
    }
    boolean isTripped = sensors.holds(tripped.get(index));
    return Optional.ofNullable(SwitchDoubt.of(joint, limit, at, isTripped));
  }

  /**
   * What each switch is doubted for, in the switches' order, as logged: empty for one that is not.
   */
  String[] logged() {
    return doubts;
  }

  /** What the joint's switches are doubted for as of this loop: none while every one agrees. */
  List<String> doubts() {
    List<String> doubted = new ArrayList<>();
    for (String doubt : doubts) {
      if (!doubt.isEmpty()) {
        doubted.add(doubt);
      }
    }
    return doubted;
  }
}

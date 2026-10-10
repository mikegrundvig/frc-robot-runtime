package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SwitchDoubt;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * Each of a joint's limit switches checked against where the joint is, once it knows ({@link
 * SwitchDoubt}): doubted once it has disagreed for a quarter second. Said, never acted on: a
 * doubted switch still stops the joint, the safe way to be wrong.
 */
final class SwitchDoubts {
  /** How long a limit switch must disagree with where the joint is before it's doubted. */
  static final long DOUBT_NANOS = 250_000_000L;

  private final Joint joint;
  private final List<Sensor.Switch> switches;

  /** What each switch is doubted for, in their order: empty while it agrees. */
  private final String[] doubts;

  /** Since when each has disagreed, in nanoseconds: -1 while it agrees. */
  private final long[] since;

  /**
   * @param joint the joint, which says where it is in its units
   * @param switches its limit switches
   */
  SwitchDoubts(Joint joint, List<Sensor.Switch> switches) {
    this.joint = joint;
    this.switches = List.copyOf(switches);
    doubts = new String[switches.size()];
    Arrays.fill(doubts, "");
    since = new long[switches.size()];
    Arrays.fill(since, -1);
  }

  /**
   * Checks each switch this loop.
   *
   * @param now this loop's time, in nanoseconds
   * @param knowsWhere whether the joint knows where it is, and its motor controllers answer
   * @param at where it is
   * @param sensors the robot's sensors, as read this loop
   */
  void check(long now, boolean knowsWhere, double at, @Nullable Sensors sensors) {
    for (int i = 0; i < switches.size(); i++) {
      Sensor.Switch limit = switches.get(i);
      String doubt = null;
      if (sensors != null && knowsWhere && sensors.reads(limit)) {
        doubt = SwitchDoubt.of(joint, limit, at, sensors.holds(limit.tripped()));
      }
      if (doubt == null) {
        since[i] = -1;
        doubts[i] = "";
      } else if (since[i] < 0) {
        since[i] = now;
      } else if (now - since[i] >= DOUBT_NANOS) {
        doubts[i] = doubt;
      }
    }
  }

  /** What each switch is doubted for, in their order, as logged: empty for one that agrees. */
  String[] logged() {
    return doubts;
  }

  /** What its switches are doubted for, as of this loop: none while they agree. */
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

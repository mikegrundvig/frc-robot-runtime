package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalLong;

/**
 * Notices a motor controller that draws nothing on its PDH channel while its joint works.
 *
 * <p>That is a follower whose breaker tripped, so that its leader drives the joint alone and
 * nothing else says so, or a controller whose block puts it on the wrong channel. While the joint's
 * leading controller draws current from the battery, each of the joint's motors draws about as much
 * on its own channel. A controller whose channel reads under the joint's control's dead current for
 * long enough ({@link MechanismControl.DeadChannel}) is flagged, until it draws again.
 *
 * <p>A joint whose block names no PDH channel has nothing to check.
 */
final class DeadChannels {
  private final String label;

  /** The joint's motor controllers that the PDH feeds, as its block says them. */
  private final List<CanDevice> fed;

  /** The PDH channel that feeds each, in the same order. */
  private final int[] channels;

  /**
   * The leading controller must draw more than this, in amps, for the joint to count as working.
   */
  private final double workingAmps;

  /** A channel that reads under this, in amps, while the joint works, is drawing nothing. */
  private final double deadAmps;

  /** How long a channel must draw nothing before its controller is flagged, in nanoseconds. */
  private final long allowedNanos;

  /**
   * Since when each controller has drawn nothing while the joint works, in nanoseconds, or empty
   * while it draws.
   */
  private final OptionalLong[] deadSince;

  /**
   * What each controller is flagged with when it draws nothing: what is wrong and what to check.
   */
  private final String[] flags;

  /**
   * What each controller is flagged for now, in their order, as the log shows it: an empty text for
   * one that draws as it should.
   */
  private final String[] dead;

  /**
   * @param deadChannel when a controller counts as drawing nothing, from the joint's control
   */
  DeadChannels(Joint joint, MechanismControl.DeadChannel deadChannel) {
    label = joint.label();
    fed = joint.devices().stream().filter(device -> device.feed().isPresent()).toList();
    channels = new int[fed.size()];
    for (int i = 0; i < fed.size(); i++) {
      channels[i] = fed.get(i).feed().orElseThrow().channel();
    }
    workingAmps = deadChannel.drawing().in(Amps);
    deadAmps = deadChannel.dead().in(Amps);
    allowedNanos = Math.round(deadChannel.time().in(Seconds) * 1e9);
    deadSince = new OptionalLong[fed.size()];
    Arrays.fill(deadSince, OptionalLong.empty());
    flags = new String[fed.size()];
    for (int i = 0; i < fed.size(); i++) {
      flags[i] =
          fed.get(i).name()
              + " draws nothing on channel "
              + channels[i]
              + " while "
              + possessive(label)
              + " motors work: its breaker tripped, its power is unplugged, or its block"
              + " names the wrong channel";
    }
    dead = new String[fed.size()];
    Arrays.fill(dead, "");
  }

  /** Whether the PDH feeds any of the joint's motor controllers, as its block says. */
  boolean any() {
    return !fed.isEmpty();
  }

  /**
   * Checks each controller's channel for this loop.
   *
   * @param now this loop's time, in nanoseconds
   * @param power the PDH, as read this loop
   * @param answers whether the joint's leading controller answers
   * @param supplyAmps what the leading controller draws from the battery, by its own reading
   * @return the current the joint's motors draw together on their channels, in amps. It is NaN
   *     while the PDH is not read, as each channel's current then is.
   */
  double check(long now, Power power, boolean answers, double supplyAmps) {
    // The joint works while its leading controller answers and draws from the battery. Each of
    // its motors then draws too.
    boolean working = power.reads() && answers && Math.abs(supplyAmps) > workingAmps;
    double togetherAmps = 0;
    for (int i = 0; i < fed.size(); i++) {
      double amps = power.amps(channels[i]);
      togetherAmps += amps;
      // A channel whose current is not a number is not drawing enough either.
      boolean drawsEnough = amps >= deadAmps;
      boolean drawsNothing = working && !drawsEnough;
      if (!drawsNothing) {
        deadSince[i] = OptionalLong.empty();
        dead[i] = "";
        continue;
      }
      if (deadSince[i].isEmpty()) {
        deadSince[i] = OptionalLong.of(now);
      }
      boolean deadLongEnough = now - deadSince[i].getAsLong() >= allowedNanos;
      if (deadLongEnough) {
        dead[i] = flags[i];
      }
    }
    return togetherAmps;
  }

  /** What each controller is flagged for, in their order, as logged: empty for one that draws. */
  String[] logged() {
    return dead;
  }

  /** The controllers flagged as of this loop, each with what to check: none while each draws. */
  List<String> flagged() {
    List<String> flagged = new ArrayList<>();
    for (String each : dead) {
      if (!each.isEmpty()) {
        flagged.add(each);
      }
    }
    return flagged;
  }
}

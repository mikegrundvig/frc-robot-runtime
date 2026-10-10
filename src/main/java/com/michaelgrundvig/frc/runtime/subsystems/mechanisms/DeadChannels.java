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

/**
 * A joint's motor controllers that read as drawing nothing on their PDH channels while the joint
 * works: a follower whose breaker tripped, so its leader drives the joint alone and nothing else
 * says so, or a controller its block puts on the wrong channel. While its leading controller draws
 * from the battery, each of its motors draws about as much on its own channel; one whose channel
 * reads under its control's dead current for its time ({@link MechanismControl.DeadChannel}) is
 * flagged until it draws again. Nothing for a joint whose block says no channel.
 */
final class DeadChannels {
  private final String label;

  /** Its motor controllers the PDH feeds, each with its channel, as its block says them. */
  private final List<CanDevice> fed;

  /** When one of them reads as drawing nothing while the joint works, from its control. */
  private final MechanismControl.DeadChannel deadChannel;

  /** Since when each has read as drawing nothing while the joint works, in nanoseconds: -1 not. */
  private final long[] since;

  /** What each is flagged for, in their order: empty while it draws as it should. */
  private final String[] dead;

  DeadChannels(Joint joint, MechanismControl.DeadChannel deadChannel) {
    label = joint.label();
    fed = joint.devices().stream().filter(device -> device.feed().isPresent()).toList();
    this.deadChannel = deadChannel;
    since = new long[fed.size()];
    Arrays.fill(since, -1);
    dead = new String[fed.size()];
    Arrays.fill(dead, "");
  }

  /** Whether the PDH feeds any of its motor controllers, as its block says. */
  boolean any() {
    return !fed.isEmpty();
  }

  /**
   * Checks each this loop.
   *
   * @param now this loop's time, in nanoseconds
   * @param power the PDH, as read this loop
   * @param answers whether its leading controller answers
   * @param supplyAmps what its leading controller draws from the battery, by its own reading
   * @return the current its motors draw together on their channels, in amps: NaN while the PDH
   *     isn't read
   */
  double check(long now, Power power, boolean answers, double supplyAmps) {
    // Working: its leading controller answers, and draws from the battery, so each motor does.
    boolean working =
        power.reads() && answers && Math.abs(supplyAmps) > deadChannel.drawing().in(Amps);
    long allowed = Math.round(deadChannel.time().in(Seconds) * 1e9);
    double together = 0;
    for (int i = 0; i < fed.size(); i++) {
      int channel = fed.get(i).feed().orElseThrow().channel();
      double amps = power.amps(channel);
      together += amps;
      if (!working || amps >= deadChannel.dead().in(Amps)) {
        since[i] = -1;
        dead[i] = "";
        continue;
      }
      if (since[i] < 0) {
        since[i] = now;
      }
      if (now - since[i] >= allowed) {
        dead[i] =
            fed.get(i).name()
                + " draws nothing on channel "
                + channel
                + " while "
                + possessive(label)
                + " motors work: its breaker tripped, its power is unplugged, or its block"
                + " names the wrong channel";
      }
    }
    return together;
  }

  /** What each is flagged for, in their order, as logged: empty for one that draws as it should. */
  String[] logged() {
    return dead;
  }

  /** Those flagged as of this loop, each said with what to check; empty while each draws. */
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

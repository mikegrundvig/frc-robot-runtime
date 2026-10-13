package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.record.Holder;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;

/**
 * What holds a joint, for a test to check against the robot's own interlocks and switches and never
 * against a sentence: {@code assertThat(Held.interlocksHolding(roller)).containsExactly(RULE)}. A
 * joint says what holds it by where each thing is in the robot's definition ({@link Holder}), and
 * this looks each one up there.
 */
public final class Held {
  private Held() {}

  /** The interlocks that hold a joint this loop, in the order they are applied. */
  public static List<Interlock> interlocksHolding(JointMechanism joint) {
    return interlocks(joint.definition(), joint.holders());
  }

  /** The limit switches that hold a joint this loop, in the order they are applied. */
  public static List<Sensor> switchesHolding(JointMechanism joint) {
    return switches(joint.definition(), joint.holders());
  }

  /** The interlocks that held a move's joints as it timed out, joint by joint. */
  public static List<Interlock> interlocksThatHeld(MoveEnding ending) {
    return interlocks(ending.definition(), ending.heldAsItEnded());
  }

  /** The limit switches that held a move's joints as it timed out, joint by joint. */
  public static List<Sensor> switchesThatHeld(MoveEnding ending) {
    return switches(ending.definition(), ending.heldAsItEnded());
  }

  /** Every interlock the robot puts on a joint, in the definition's order. */
  public static List<Interlock> interlocksOn(JointMechanism joint) {
    List<Interlock> on = new ArrayList<>();
    for (Interlock interlock : joint.definition().everyInterlock()) {
      if (interlock.joint().equals(joint.joint())) {
        on.add(interlock);
      }
    }
    return on;
  }

  private static List<Interlock> interlocks(RobotDefinition definition, List<Holder> holders) {
    List<Interlock> interlocks = new ArrayList<>();
    for (Holder holder : holders) {
      if (holder.kind() == Holder.Kind.INTERLOCK) {
        interlocks.add(definition.everyInterlock().get(holder.index()));
      }
    }
    return interlocks;
  }

  private static List<Sensor> switches(RobotDefinition definition, List<Holder> holders) {
    List<Sensor> switches = new ArrayList<>();
    for (Holder holder : holders) {
      if (holder.kind() == Holder.Kind.LIMIT_SWITCH) {
        switches.add(definition.everySensor().get(holder.index()));
      }
    }
    return switches;
  }
}

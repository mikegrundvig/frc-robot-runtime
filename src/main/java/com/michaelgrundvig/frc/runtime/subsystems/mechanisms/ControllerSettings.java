package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.lib.rev.RevMechanism;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.EncoderLoop;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * What a driven joint's motor controllers are told, from its block: how the program drives it
 * ({@link ControllerPlan}), said by the helper for the controllers its block names, a TalonFX's or
 * a TalonFXS's by the Phoenix helper ({@link PhoenixMechanism}), a SPARK's by the REV helper
 * ({@link RevMechanism}), with the definition's absolute encoder its block wires to them. Pure, as
 * they are: no device is touched, so they're worked out as the robot's configuration is checked
 * ({@link Joints#problems}), in the simulator and the tests as on a robot, and what the robot would
 * refuse at startup is refused there first, with the robot's own message; what's doubtful the robot
 * says at startup, wherever it runs ({@link #findings}). On a robot, the joint's IO applies these
 * same settings ({@link JointIOTalon}, {@link JointIOSpark}).
 *
 * <p>A joint whose motors aren't wired runs in the simulator alone, on the controller its motor
 * runs on, its plan's stand-in ({@link MechanismWiring.Controller#standIn}): it's checked on that,
 * as the simulator runs it, so a NEO Vortex isn't refused for what only a TalonFX can't take.
 */
public final class ControllerSettings {
  private ControllerSettings() {}

  /**
   * What a joint's motor controllers would make of its settings, from its block's control, worked
   * out for the controllers its block names, and for its motor's where it names none: what they'd
   * refuse at startup, as one refusal in the robot's own words, or what's doubtful though they'd
   * take it, each a doubt about the joint. The robot's configuration refuses the first ({@link
   * Joints#problems}), and the robot says the rest at startup, in the simulator as on the robot.
   *
   * @param joint one the template drives, with a control ({@link Joints#driven})
   */
  public static List<Finding> findings(RobotMath math, Joint joint) {
    List<String> notes;
    try {
      notes = notes(math, joint);
    } catch (IllegalArgumentException e) {
      // The plan and the helpers refuse by throwing, as the robot does at startup: said here,
      // once, as found.
      return List.of(Finding.refused(joint.label(), String.valueOf(e.getMessage())));
    }
    return notes.stream().map(note -> Finding.doubt(joint.label(), note)).toList();
  }

  /**
   * What's doubtful in a joint's settings, though allowed, each a sentence: the plan's, then the
   * helper's for the controllers its block names; for a joint whose motors aren't wired, for its
   * motor's.
   *
   * @throws IllegalArgumentException for what the robot would refuse at startup, saying what and
   *     why as it would
   */
  private static List<String> notes(RobotMath math, Joint joint) {
    MechanismControl control = control(joint);
    return switch (plan(math, joint, control).controllers().get(0).controller()) {
      case TALON_FX, TALON_FXS -> talon(math, joint, control).notes();
      case SPARK_MAX, SPARK_FLEX -> spark(math, joint, control).notes();
    };
  }

  /**
   * How the program drives a joint, for a control, its block's or as the pit tunes it: with the
   * controllers its block names, or its motor's where it names none ({@link
   * ControllerPlan#controllers}).
   *
   * @throws IllegalArgumentException for what no motor controller could do
   */
  static ControllerPlan plan(RobotMath math, Joint joint, MechanismControl control) {
    return ControllerPlan.of(math, joint, control);
  }

  /**
   * A TalonFX's or TalonFXS's settings for a joint, from its block and a control: its block's, or
   * as the pit tunes it. A CANcoder its block names is the definition's absolute encoder on it.
   *
   * @throws IllegalArgumentException for an encoder on a TalonFXS's data port, which the template
   *     doesn't read yet, and what the plan or the Phoenix helper refuses
   */
  static PhoenixMechanism.Settings talon(RobotMath math, Joint joint, MechanismControl control) {
    ControllerPlan plan = plan(math, joint, control);
    Optional<Driven.AbsoluteEncoder> encoder = joint.driven().absoluteEncoder();
    if (encoder.flatMap(Driven.AbsoluteEncoder::dataPort).isPresent()
        && plan.controllers().get(0).controller() == MechanismWiring.Controller.TALON_FXS) {
      throw new IllegalArgumentException(
          possessive(joint.label())
              + " wiring names an encoder on a TalonFXS's data port, which the template doesn't"
              + " read yet: wire it as a CANcoder");
    }
    Optional<MechanismWiring.Cancoder> cancoder = encoder.flatMap(Driven.AbsoluteEncoder::cancoder);
    if (cancoder.isEmpty()) {
      return PhoenixMechanism.configure(plan);
    }
    return PhoenixMechanism.configure(
        math, plan, Objects.requireNonNull(math.absoluteEncoder(joint)), cancoder.get());
  }

  /**
   * A SPARK MAX's or SPARK Flex's settings for a joint, from its block and a control. An encoder on
   * its data port is the definition's absolute encoder on it.
   *
   * @throws IllegalArgumentException for what the plan or the REV helper refuses
   */
  static RevMechanism.Settings spark(RobotMath math, Joint joint, MechanismControl control) {
    ControllerPlan plan = plan(math, joint, control);
    Optional<MechanismWiring.DataPortEncoder> port =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::dataPort);
    if (port.isEmpty()) {
      return RevMechanism.configure(plan);
    }
    return RevMechanism.configure(
        math, plan, Objects.requireNonNull(math.absoluteEncoder(joint)), port.get());
  }

  /**
   * The loop a joint closes itself on its absolute encoder on a Systemcore port, for a control: the
   * plan's kP, kD and feedforward law, worked out on the robot. Its motor controller's own settings
   * are worked out beside it: they run the joint while the encoder is lost.
   *
   * @throws IllegalArgumentException for what the plan refuses
   */
  static EncoderLoop encoderLoop(RobotMath math, Joint joint, MechanismControl control) {
    return new EncoderLoop(plan(math, joint, control));
  }

  /**
   * A joint's control, from its block.
   *
   * @throws IllegalArgumentException for a joint with none, which the template doesn't drive
   */
  public static MechanismControl control(Joint joint) {
    return joint
        .driven()
        .control()
        .orElseThrow(() -> new IllegalArgumentException(joint.label() + " has no control"));
  }
}

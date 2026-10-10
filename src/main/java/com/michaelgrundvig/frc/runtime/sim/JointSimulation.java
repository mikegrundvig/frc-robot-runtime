package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The robot's driven joints in the simulator, each a {@link JointIOSim} the joints' mechanisms read
 * and drive as they would real motor controllers, and the absolute encoders the robot reads itself.
 * Built from the robot's definition, every one at once, whatever the robot asks for first. Stepped
 * once a loop, before the robot reads its sensors, as the drivetrain's physics is ({@link
 * DriveSimulation}): gravity on each joint is worked out from where every joint really is as the
 * step starts, so a wrist feels its arm's angle through its links' weight (a tuned kG holds it as
 * one cosine, {@code JointPlant}).
 *
 * <p>Closed form, from the robot's definition: no joint pushes on another as it moves.
 */
public final class JointSimulation {
  private final RobotDefinition definition;
  private final RobotMath math;
  private final List<JointIOSim> joints = new ArrayList<>();
  private final Map<Joint, AbsoluteEncoderIOSim> encoders = new HashMap<>();

  /** Where each joint really is, as the step starts: what gravity is worked out from. */
  private final Map<Joint, Double> truth = new LinkedHashMap<>();

  /**
   * Every joint the robot drives ({@code Joints.driven}), its motor controller running the control
   * its block gives it, gravity on it the definition's ({@link JointIOSim}), and what's on its
   * controller's limits as the robot's IO is told ({@link ControllerPins#of}); and each absolute
   * encoder the robot reads itself, on a Systemcore port or unwired ({@code Joints.readsAbsolute}).
   *
   * @param definition what the robot is: each joint's inertia, gravity, motors and wiring
   */
  public JointSimulation(RobotDefinition definition) {
    this.definition = definition;
    math = definition.math();
    for (Joint joint : Joints.driven(definition)) {
      JointIOSim simulated = new JointIOSim(math, joint, ControllerPins.of(definition, joint));
      joints.add(simulated);
      if (Joints.readsAbsolute(joint) != null) {
        encoders.put(
            joint,
            new AbsoluteEncoderIOSim(
                simulated,
                Objects.requireNonNull(math.absoluteEncoder(joint)),
                joint
                    .driven()
                    .absoluteEncoder()
                    .flatMap(Driven.AbsoluteEncoder::port)
                    .orElse(null)));
      }
    }
  }

  /** A driven joint's motor controller, simulated, for its mechanism to read and drive. */
  public JointIO io(Joint joint) {
    return sim(joint);
  }

  /**
   * A joint's absolute encoder, simulated, where the robot reads it itself.
   *
   * @throws IllegalArgumentException for a joint whose encoder the robot doesn't read itself
   */
  public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
    AbsoluteEncoderIOSim encoder = encoders.get(joint);
    if (encoder == null) {
      throw new IllegalArgumentException(
          joint.label() + " has no absolute encoder the robot reads itself");
    }
    return encoder;
  }

  /**
   * Each joint's motor controller reads the switches on its limits from the simulated world's
   * sensors, as they read: once they're simulated.
   */
  void readsLimits(SensorSimulation sensors) {
    for (JointIOSim joint : joints) {
      joint.readsLimits(
          limit(sensors, joint.joint(), SensorWiring.Direction.FORWARD),
          limit(sensors, joint.joint(), SensorWiring.Direction.REVERSE));
    }
  }

  /** Whether the switch on one of a joint's controller's limits reads tripped: never for none. */
  private BooleanSupplier limit(
      SensorSimulation sensors, Joint joint, SensorWiring.Direction direction) {
    Sensor.Switch on = ControllerPins.onLimit(definition, joint, direction);
    return on == null ? () -> false : sensors.tripped(on);
  }

  /**
   * Silences a joint's motor controllers, or brings them back: off the CAN bus, as a loose
   * connector leaves them, they answer nothing and hear nothing, so they go neutral.
   */
  public void silence(Joint joint, boolean silent) {
    sim(joint).silence(silent);
  }

  /**
   * Restarts a joint's leading motor controller, as a breaker that resets or a power connector that
   * comes loose does: it says so the next time it's read, goes neutral until it's asked again, and
   * counts from 0, or reads where the joint is from the absolute encoder its block has it read.
   */
  public void restart(Joint joint) {
    sim(joint).restart();
  }

  /**
   * Takes one of a joint's followers off the CAN bus, or brings it back, as a loose connector does:
   * it goes neutral, drawing nothing from the battery, and its leader drives the joint without it.
   *
   * @param follower which, from 0 for the first after the leader
   * @throws IllegalArgumentException for a follower the joint hasn't
   */
  public void silenceFollower(Joint joint, int follower, boolean silent) {
    sim(joint).silenceFollower(follower, silent);
  }

  /**
   * Jams a joint where it is, or frees it: it stays still whatever its motors do, as a game piece
   * wedged in it or a bent part leaves it, its motors stalled against what holds it, drawing their
   * stall current up to their limit.
   */
  public void jam(Joint joint, boolean jammed) {
    sim(joint).jam(jammed);
  }

  /**
   * Freezes a joint's absolute encoder, or thaws it: its reading stuck where it was, as a frozen
   * encoder's is, though it answers. The robot's, or one its motor controller reads.
   *
   * @throws IllegalArgumentException for a joint without one
   */
  public void freeze(Joint joint, boolean frozen) {
    AbsoluteEncoderIOSim encoder = encoders.get(joint);
    if (encoder != null) {
      encoder.frozen = frozen;
    } else {
      withEncoder(joint).encoderFrozen = frozen;
    }
  }

  /**
   * Slips a joint's absolute encoder on its shaft, as a loose magnet does: it reads the joint this
   * far off from now on, in the joint's units, radians or meters; 0 puts it back. The robot's, or
   * one its motor controller reads.
   *
   * @throws IllegalArgumentException for a joint without one
   */
  public void slip(Joint joint, double off) {
    AbsoluteEncoderIOSim encoder = encoders.get(joint);
    if (encoder != null) {
      encoder.slip = off;
    } else {
      withEncoder(joint).encoderSlip = off;
    }
  }

  /** A joint whose motor controller reads its absolute encoder, as the robot's block has it. */
  private JointIOSim withEncoder(Joint joint) {
    JointIOSim simulated = sim(joint);
    if (simulated
        .joint()
        .driven()
        .absoluteEncoder()
        .map(Driven.AbsoluteEncoder::readByController)
        .orElse(false)) {
      return simulated;
    }
    throw new IllegalArgumentException(joint.label() + " has no absolute encoder to fail");
  }

  /** Unplugs a joint's absolute encoder, or plugs it back in: it reads nothing while unplugged. */
  public void unplug(Joint joint, boolean unplugged) {
    Objects.requireNonNull(encoders.get(joint), joint.label() + " has no simulated encoder")
            .connected =
        !unplugged;
  }

  /** One loop: every joint runs for a time, with gravity where the joints stand as it starts. */
  public void step(double seconds) {
    truth.clear();
    for (JointIOSim joint : joints) {
      truth.put(joint.joint(), joint.position());
    }
    for (JointIOSim joint : joints) {
      joint.step(seconds, joint.gravityTorque(truth));
    }
  }

  /** Where a camera on a joint really is in the robot's frame, the joints as they really stand. */
  public Transform3d cameraMount(CameraConfig camera) {
    Map<Joint, Double> now = new LinkedHashMap<>();
    for (JointIOSim joint : joints) {
      now.put(joint.joint(), joint.position());
    }
    Pose3d pose = math.cameraPose(camera, now);
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /** Whether a joint is simulated: one the robot drives. */
  public boolean simulates(Joint joint) {
    for (JointIOSim each : joints) {
      if (each.joint().equals(joint)) {
        return true;
      }
    }
    return false;
  }

  /** Where a joint really is, in its units. */
  public double position(Joint joint) {
    return sim(joint).position();
  }

  /**
   * The current each of a joint's motors draws from the battery, in amps: what its PDH channel
   * reads.
   */
  public double supplyAmps(Joint joint) {
    return sim(joint).supplyAmps();
  }

  /**
   * The current one of a joint's motor controllers draws from the battery, in amps: what its PDH
   * channel reads; nothing for a follower off the bus.
   *
   * @param controller which, from 0 for the leader
   */
  public double supplyAmps(Joint joint, int controller) {
    return sim(joint).supplyAmps(controller);
  }

  /** How fast a joint really moves, in its units a second. */
  public double velocity(Joint joint) {
    return sim(joint).velocity();
  }

  /** Puts a joint somewhere, still: where a test starts it. */
  public void place(Joint joint, double position) {
    sim(joint).place(position);
  }

  /**
   * Powers a joint on somewhere other than where it rests, still: its motor controller reads it
   * where it rests until it's homed, as a joint read by its motor alone does.
   */
  public void powerOnAt(Joint joint, double position) {
    sim(joint).powerOnAt(position);
  }

  private JointIOSim sim(Joint joint) {
    for (JointIOSim each : joints) {
      if (each.joint().equals(joint)) {
        return each;
      }
    }
    throw new IllegalArgumentException(
        joint.label() + " isn't simulated: the template doesn't drive it");
  }
}

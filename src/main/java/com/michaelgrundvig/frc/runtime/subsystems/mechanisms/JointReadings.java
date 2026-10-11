package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalInt;
import org.littletonrobotics.junction.Logger;

/**
 * What a joint reads in one loop: its motor controller's readings, its absolute encoder's where the
 * robot reads that encoder itself, and what follows directly from the two, such as where the joint
 * is and how fast its speed is changing.
 *
 * <p>Both sets of readings are AdvantageKit inputs and are logged as they are read, so a replay of
 * a log hands the joint exactly what it read on the robot, and everything the joint works out from
 * them comes out the same.
 */
final class JointReadings {
  /**
   * The absolute encoder of a joint, when the robot reads it itself.
   *
   * @param io the encoder's hardware
   * @param model the encoder as the robot's definition has it, which turns its reading into where
   *     the joint is. Empty if the definition has no encoder for the joint alone, and then its
   *     reading is logged but says nothing of where the joint is.
   * @param port the Systemcore port it is wired to, whose offset and direction its raw reading is
   *     corrected by, if it is on one
   * @param closesLoop whether the joint's loop closes on this encoder, so that the encoder's word
   *     is where the joint is, and not the motor's count
   */
  record RobotReadEncoder(
      AbsoluteEncoderIO io,
      Optional<Sensor.Encoder> model,
      Optional<MechanismWiring.PortEncoder> port,
      boolean closesLoop) {
    /** A joint's absolute encoder as the robot reads it, wired as the joint's block says. */
    static RobotReadEncoder of(RobotMath math, Joint joint, AbsoluteEncoderIO io) {
      Optional<MechanismWiring.PortEncoder> port =
          joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port);
      boolean closesLoop = port.isPresent() && port.get().closedLoop();
      Optional<Sensor.Encoder> model = Optional.ofNullable(math.absoluteEncoder(joint));
      return new RobotReadEncoder(io, model, port, closesLoop);
    }

    /** What the joint's homing needs to know of this encoder. */
    JointHoming.ReadEncoder forHoming() {
      OptionalInt onPort =
          port.isPresent() ? OptionalInt.of(port.get().port()) : OptionalInt.empty();
      return new JointHoming.ReadEncoder(closesLoop, onPort);
    }
  }

  /**
   * The switches wired to the limit inputs of the joint's leading motor controller. The joint reads
   * them from its controller with the rest of its readings, and hands them to the robot's sensors
   * each loop.
   *
   * @param forward the switch on the forward limit input, if there is one
   * @param reverse the switch on the reverse limit input, if there is one
   */
  record PinnedSwitches(Optional<Sensor.Switch> forward, Optional<Sensor.Switch> reverse) {
    /** The robot's switches that are wired to a joint's motor controller's limit inputs. */
    static PinnedSwitches of(RobotMath math, Joint joint) {
      Optional<Sensor.Switch> forward = Optional.empty();
      Optional<Sensor.Switch> reverse = Optional.empty();
      for (Sensor sensor : math.sensors()) {
        if (!(sensor instanceof Sensor.Switch onSwitch)) {
          continue;
        }
        if (!(ControllerPins.onPin(onSwitch) instanceof SensorWiring.ControllerLimit pin)) {
          continue;
        }
        if (!pin.joint().equals(joint.label())) {
          continue;
        }
        if (pin.direction() == SensorWiring.Direction.FORWARD) {
          forward = Optional.of(onSwitch);
        } else {
          reverse = Optional.of(onSwitch);
        }
      }
      return new PinnedSwitches(forward, reverse);
    }
  }

  private final RobotMath math;
  private final JointIO io;
  private final JointIOInputsAutoLogged inputs = new JointIOInputsAutoLogged();
  private final Optional<RobotReadEncoder> encoder;
  private final AbsoluteEncoderIOInputsAutoLogged encoderInputs =
      new AbsoluteEncoderIOInputsAutoLogged();
  private final PinnedSwitches pinned;
  private final Sensors sensors;

  /** Whether the joint's leading motor controller reads a CANcoder, which then is a device too. */
  private final boolean hasCancoder;

  private final String inputsKey;
  private final String encoderKey;
  private final String restartsKey;

  // This loop.

  /** How many loops have been read. The first is loop 1. */
  private long loop;

  private double position;
  private double acceleration;

  /** Whether the encoder the robot reads says where the joint is this loop. */
  private boolean encoderSaysWhere;

  /** Where that encoder says the joint is. It means something only while the encoder says where. */
  private double positionByEncoder;

  /**
   * How many times each of the joint's devices has restarted since the robot started: the leader,
   * then each follower, then its CANcoder if it has one.
   */
  private int[] restarts = new int[0];

  /**
   * @param key where the joint logs, {@code Joints/<label>}
   * @param hasCancoder whether the joint's leading motor controller reads a CANcoder
   */
  JointReadings(
      RobotMath math,
      String key,
      JointIO io,
      Optional<RobotReadEncoder> encoder,
      PinnedSwitches pinned,
      Sensors sensors,
      boolean hasCancoder) {
    this.math = math;
    this.io = io;
    this.encoder = encoder;
    this.pinned = pinned;
    this.sensors = sensors;
    this.hasCancoder = hasCancoder;
    inputsKey = key;
    encoderKey = key + "/Absolute";
    restartsKey = key + "/Restarts";
  }

  /** Reads the joint's hardware for a new loop. */
  void read() {
    loop++;
    double velocityBefore = inputs.velocity;
    io.updateInputs(inputs);
    Logger.processInputs(inputsKey, inputs);
    handPinnedSwitchesToTheSensors();
    // The first loop has no loop before it to have changed speed since.
    acceleration = loop == 1 ? 0 : (inputs.velocity - velocityBefore) / RobotLoop.PERIOD_SECONDS;
    readEncoder();
    boolean encoderIsWhereItIs =
        encoderSaysWhere && encoder.isPresent() && encoder.get().closesLoop();
    position = encoderIsWhereItIs ? positionByEncoder : inputs.position;
    countRestarts();
  }

  /**
   * Hands the switches on the controller's limit inputs to the robot's sensors, as the controller
   * read them. A controller that does not answer has lost its switches with it.
   */
  private void handPinnedSwitchesToTheSensors() {
    if (pinned.forward().isPresent()) {
      sensors.pinned(pinned.forward().get(), inputs.connected, inputs.forwardLimit);
    }
    if (pinned.reverse().isPresent()) {
      sensors.pinned(pinned.reverse().get(), inputs.connected, inputs.reverseLimit);
    }
  }

  /**
   * Reads the absolute encoder the robot reads itself, and works out where it says the joint is.
   */
  private void readEncoder() {
    encoderSaysWhere = false;
    if (encoder.isEmpty()) {
      return;
    }
    RobotReadEncoder read = encoder.get();
    read.io().updateInputs(encoderInputs);
    Logger.processInputs(encoderKey, encoderInputs);
    if (read.model().isEmpty() || !encoderInputs.connected) {
      return;
    }
    // The port's wiring says how the encoder is mounted: its offset and its direction.
    double share = encoderInputs.share;
    double corrected = read.port().isPresent() ? read.port().get().read(share) : share;
    positionByEncoder = math.fromAbsolute(read.model().get(), corrected);
    encoderSaysWhere = true;
  }

  /**
   * Counts each device's restarts, and logs the counts with the devices' health, a few times a
   * second ({@link JointIO#HEALTH_LOOPS}). The devices are in the order the joint's inputs number
   * their faults: the leader, each follower, then its CANcoder.
   */
  private void countRestarts() {
    int followers = inputs.followersRestarted.length;
    int devices = 1 + followers + (hasCancoder ? 1 : 0);
    if (restarts.length != devices) {
      restarts = Arrays.copyOf(restarts, devices);
    }
    restarts[0] += inputs.restarted ? 1 : 0;
    for (int i = 0; i < followers; i++) {
      restarts[1 + i] += inputs.followersRestarted[i] ? 1 : 0;
    }
    if (hasCancoder) {
      restarts[devices - 1] += inputs.encoderRestarted ? 1 : 0;
    }
    if (loop % JointIO.HEALTH_LOOPS == 1) {
      Logger.recordOutput(restartsKey, restarts);
    }
  }

  // ---- Where the joint is ----

  /** Whether this is the first loop the joint has read. */
  boolean isFirstLoop() {
    return loop == 1;
  }

  /**
   * Where the joint is, in its units: what its motor controller reads, or, for a joint whose loop
   * closes on an absolute encoder the robot reads, what that encoder reads while it answers.
   */
  double position() {
    return position;
  }

  /** How fast the joint moves, in its units a second. */
  double velocity() {
    return inputs.velocity;
  }

  /** How fast its speed changed since the loop before, in its units a second squared. */
  double acceleration() {
    return acceleration;
  }

  /** Where the motor controller says the joint is, whatever an encoder the robot reads says. */
  double positionByController() {
    return inputs.position;
  }

  /**
   * Where the motor's own count says the joint is. For most joints that is the controller's
   * position. For a joint whose controller closes its loop on an absolute encoder, the controller's
   * position is the encoder's word, and the controller reports the motor's count beside it.
   */
  double positionByMotor() {
    return Double.isNaN(inputs.motorPosition) ? inputs.position : inputs.motorPosition;
  }

  // ---- The absolute encoder ----

  /**
   * Whether the joint has an absolute encoder the robot reads itself with a model to read it by.
   */
  boolean hasRobotReadEncoder() {
    return encoder.isPresent() && encoder.get().model().isPresent();
  }

  /** Whether the absolute encoder the robot reads itself gave a reading this loop. */
  boolean encoderAnswers() {
    return encoderInputs.connected;
  }

  /** Whether the encoder the robot reads says where the joint is this loop. */
  boolean encoderSaysWhere() {
    return encoderSaysWhere;
  }

  /**
   * Where the encoder the robot reads says the joint is. Ask {@link #encoderSaysWhere} first: while
   * it does not, this is where it last did, and means nothing.
   */
  double positionByEncoder() {
    return positionByEncoder;
  }

  /**
   * Where the joint's absolute encoder says the joint is, whoever reads the encoder: the robot, or
   * the joint's motor controller. It is NaN in a loop with no reading, and for a joint with no such
   * encoder, because that is how the model's {@code EncoderGap} is told a reading is missing.
   */
  double positionByAnyEncoderOrNaN() {
    if (!hasRobotReadEncoder()) {
      return inputs.absolutePosition;
    }
    return encoderSaysWhere ? positionByEncoder : Double.NaN;
  }

  /** Where the encoder the joint's motor controller reads says the joint is, or NaN for none. */
  double positionByControllersEncoderOrNaN() {
    return inputs.absolutePosition;
  }

  /** Whether the CANcoder the controller reads says its magnet is too close or too far. */
  boolean magnetIsBad() {
    return inputs.magnetBad;
  }

  // ---- The motor controllers ----

  /** Whether the joint's motor controllers answered lately. */
  boolean answers() {
    return inputs.connected;
  }

  /**
   * Whether the joint's position and speed are this loop's. A reading that is not fresh, because a
   * status frame did not come, is treated as no reading by the interlocks that read this joint:
   * they hold at once, without waiting for the controller to be called silent.
   */
  boolean answersThisLoop() {
    return inputs.connected && inputs.fresh;
  }

  /** Whether the joint's leading motor controller restarted since the last loop. */
  boolean controllerRestarted() {
    return inputs.restarted;
  }

  /** The joint's devices that did not take their settings at startup, or empty when all did. */
  String unconfigured() {
    return inputs.unconfigured;
  }

  /** The voltage across the leading motor. */
  double appliedVolts() {
    return inputs.appliedVolts;
  }

  /** The current through the leading motor's windings. */
  double statorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  /** The current the leading motor draws from the battery. */
  double supplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  /** How many times each device has restarted since the robot started, as logged. */
  int[] restarts() {
    return restarts.clone();
  }
}

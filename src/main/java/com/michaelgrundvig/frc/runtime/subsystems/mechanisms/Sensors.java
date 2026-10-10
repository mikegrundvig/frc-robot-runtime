package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;

/**
 * The robot's switches and rangefinders, as its program reads them: the definition says what each
 * is and where, its wiring how it's connected on a robot. Each loop, after the joints, it reads
 * each through its IO ({@link SensorIO}), logged under {@code Sensors/<label>}; a switch on a
 * joint's motor controller's limit input is read by that joint, with the rest of it, from its
 * controller ({@link JointIO.JointIOInputs#forwardLimit}), and logged here the same. Behavior reads
 * them through the conditions the model names, as gates ({@link Joints#gate}):
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=sensor-gates}
 *
 * <p>A gate on a sensor waits, guards and binds as any gate does:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=intake-one}
 *
 * <p>A sensor that's lost (a CAN device silent, a joint's controller silent for a switch on its
 * limit input, or one with no wiring on a robot) holds no condition either way: neither tripped nor
 * clear, neither nearer nor farther. A gate on it never opens, and says so ("hopper range lost"); a
 * move that waits on it times out.
 */
public final class Sensors {
  /**
   * One sensor, as read: through its IO, or, for a switch on a joint's limit input, by its joint.
   */
  private static final class Reading {
    final Sensor.Placed sensor;
    final @Nullable SensorIO io;
    final SensorIOInputsAutoLogged inputs = new SensorIOInputsAutoLogged();
    final String key;

    Reading(Sensor.Placed sensor, @Nullable SensorIO io) {
      this.sensor = sensor;
      this.io = io;
      key = "Sensors/" + sensor.label();
    }
  }

  private final Scheduler scheduler;
  private final Reading[] readings;
  private final Map<Sensor, Reading> bySensor = new HashMap<>();

  /**
   * @param definition what the robot is: its switches and rangefinders
   * @param io each one's hardware: a port or a CAN device, the simulator's, or none; never asked
   *     for a switch on a joint's motor controller's limit input, which its joint reads ({@link
   *     #pinned})
   * @param scheduler the command scheduler its gates are registered with
   */
  Sensors(RobotDefinition definition, Function<Sensor.Placed, SensorIO> io, Scheduler scheduler) {
    this.scheduler = scheduler;
    List<Reading> built = new ArrayList<>();
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Placed placed) {
        Reading reading =
            new Reading(placed, ControllerPins.onPin(placed) != null ? null : io.apply(placed));
        built.add(reading);
        bySensor.put(placed, reading);
      }
    }
    readings = built.toArray(Reading[]::new);
  }

  /**
   * A switch on a joint's motor controller's limit input, as its joint read it from its controller
   * this loop: before the sensors are read, as the joints are.
   *
   * @param connected whether the controller answers: the switch is lost with it
   * @param tripped whether the controller reads it tripped, its polarity told to the controller
   */
  void pinned(Sensor.Switch sensor, boolean connected, boolean tripped) {
    SensorIO.SensorIOInputs inputs = reading(sensor).inputs;
    inputs.connected = connected;
    inputs.tripped = tripped;
  }

  /** Reads every sensor: once a loop, after the joints and before the command scheduler. */
  void read() {
    for (Reading reading : readings) {
      SensorIO io = reading.io;
      if (io != null) {
        io.updateInputs(reading.inputs);
      }
      Logger.processInputs(reading.key, reading.inputs);
    }
  }

  /**
   * Whether a condition on one of the robot's sensors holds this loop: never for a sensor that's
   * lost.
   *
   * @throws IllegalArgumentException for a sensor the robot doesn't have
   */
  public boolean holds(Condition.OnSensor condition) {
    SensorIO.SensorIOInputs inputs = reading(condition.sensor()).inputs;
    if (!inputs.connected) {
      return false;
    }
    return switch (condition) {
      case Condition.Tripped tripped -> inputs.tripped;
      case Condition.Clear clear -> !inputs.tripped;
      case Condition.Nearer nearer -> inputs.meters < nearer.distance().in(Meters);
      case Condition.Farther farther -> inputs.meters > farther.distance().in(Meters);
    };
  }

  /** Whether a sensor answers this loop: false for one that's lost. */
  public boolean reads(Sensor sensor) {
    return reading(sensor).inputs.connected;
  }

  /**
   * A gate on a condition, named: {@link Joints#gate(String, Condition)}'s for a sensor. One on a
   * sensor the robot doesn't have is refused as a joint's gate on a place not its own is ({@link
   * Refusals#gate}): thrown as the robot starts, and once it runs, a gate that holds false saying
   * why.
   *
   * @throws IllegalArgumentException for a sensor the robot doesn't have, as the robot starts
   */
  Gate gate(String name, Condition.OnSensor condition) {
    Reading reading = bySensor.get(condition.sensor());
    if (reading == null) {
      return Refusals.gate(scheduler, name, notOurs(condition.sensor()));
    }
    return new Gate(
        scheduler,
        name,
        why -> {
          boolean holds = holds(condition);
          if (!holds && !reading.inputs.connected) {
            why.add(reading.sensor.label() + " lost");
          }
          return holds;
        });
  }

  /** The sensors that are lost this loop, by label: what an alert names. */
  public List<String> lost() {
    List<String> lost = new ArrayList<>();
    for (Reading reading : readings) {
      if (!reading.inputs.connected) {
        lost.add(reading.sensor.label());
      }
    }
    return lost;
  }

  private Reading reading(Sensor sensor) {
    Reading reading = bySensor.get(sensor);
    if (reading == null) {
      throw new IllegalArgumentException(notOurs(sensor));
    }
    return reading;
  }

  private static String notOurs(Sensor sensor) {
    return sensor.label() + " isn't one of the robot's sensors";
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.world.record.ControlSentences;
import com.michaelgrundvig.frc.world.record.GateTerm;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
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
   * One sensor, and what it read this loop.
   *
   * <p>Most sensors are read here, through their IO. A switch on a joint's motor controller's limit
   * input has no IO of its own: its joint reads it from the controller and hands the reading over
   * ({@link #pinned}).
   */
  private static final class Reading {
    final Sensor.Placed sensor;

    /** The sensor's hardware, or empty for a switch that its joint reads from its controller. */
    final Optional<SensorIO> io;

    final SensorIOInputsAutoLogged inputs = new SensorIOInputsAutoLogged();

    /** Where the sensor logs, {@code Sensors/<label>}. */
    final String key;

    Reading(Sensor.Placed sensor, Optional<SensorIO> io) {
      this.sensor = sensor;
      this.io = io;
      key = "Sensors/" + sensor.label();
    }
  }

  private final Scheduler scheduler;

  /** The robot, whose names a gate's term about a sensor is said in. */
  private final RobotDefinition definition;

  /** The definition's sensors. A logged fact names a sensor by where it is among them. */
  private final List<Sensor> everySensor;

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
    this.definition = definition;
    everySensor = definition.everySensor();
    List<Reading> built = new ArrayList<>();
    for (Sensor sensor : everySensor) {
      if (sensor instanceof Sensor.Placed placed) {
        boolean isReadByItsJoint = ControllerPins.onPin(placed) != null;
        Optional<SensorIO> own =
            isReadByItsJoint ? Optional.empty() : Optional.of(io.apply(placed));
        Reading reading = new Reading(placed, own);
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
      if (reading.io.isPresent()) {
        reading.io.get().updateInputs(reading.inputs);
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
    GateTerm lost = new GateTerm(GateTerm.Kind.SENSOR_LOST, everySensor.indexOf(reading.sensor));
    String lostShown = ControlSentences.gateTerm(definition, name, lost, List.of());
    return new Gate(
        scheduler,
        name,
        why -> {
          boolean holds = holds(condition);
          if (!holds && !reading.inputs.connected) {
            why.add(lost, lostShown);
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

  /**
   * What one of the robot's sensors read. Conditions ask this several times a loop, and nearly
   * always with the very object the robot's definition holds, so that is looked for first, among
   * the few sensors a robot has. Finding a sensor by its value hashes everything the sensor is made
   * of, its joint and its pose included, which allocates about a kilobyte each time; that is left
   * for a sensor that is equal to one of the robot's without being the same object.
   */
  private Reading reading(Sensor sensor) {
    for (Reading reading : readings) {
      if (reading.sensor == sensor) {
        return reading;
      }
    }
    Reading equal = bySensor.get(sensor);
    if (equal == null) {
      throw new IllegalArgumentException(notOurs(sensor));
    }
    return equal;
  }

  private static String notOurs(Sensor sensor) {
    return sensor.label() + " isn't one of the robot's sensors";
  }
}

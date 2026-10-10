package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIO;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Distance;

/**
 * The robot's switches and rangefinders in the simulator, each its definition has, built at once. A
 * limit switch trips from where its joint really is; what a beam, a contact switch or a rangefinder
 * sees is a test's or a scenario's to say ({@link #trip}, {@link #see}), game pieces being sensor
 * states, never physics. Until it's said, a switch is clear and a rangefinder sees nothing within
 * its range. A switch on a joint's motor controller's limit input is read by that joint's simulated
 * controller, as a robot's is, which stops on it there ({@link JointIOSim}).
 */
public final class SensorSimulation {
  /** One simulated sensor: its device's reading, as a test leaves it. */
  private static final class Simulated implements SensorIO {
    final Sensor.Placed sensor;
    final JointSimulation joints;
    boolean tripped;
    double meters;
    boolean silent;

    /** A cut wire's reading, whatever trips it: null while its wire is whole. */
    @Nullable Boolean cut;

    Simulated(Sensor.Placed sensor, JointSimulation joints) {
      this.sensor = sensor;
      this.joints = joints;
      meters =
          sensor instanceof Sensor.Rangefinder rangefinder ? rangefinder.range().in(Meters) : 0;
    }

    /**
     * Whether a switch reads tripped now: as its cut wire reads, else where its joint really is for
     * a limit switch on a simulated joint, else as a test left it.
     */
    boolean tripped() {
      Boolean stuck = cut;
      if (stuck != null) {
        return stuck;
      }
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && joints.simulates(limit.joint())) {
        return limit.trippedAt(joints.position(limit.joint()));
      }
      return tripped;
    }

    @Override
    public void updateInputs(SensorIOInputs inputs) {
      inputs.connected = !silent;
      if (silent) {
        return;
      }
      inputs.tripped = tripped();
      inputs.meters = meters;
    }
  }

  private final JointSimulation joints;
  private final Map<Sensor.Placed, Simulated> sensors = new LinkedHashMap<>();

  /**
   * Every switch and rangefinder the robot's definition has; and each joint's motor controller,
   * from now on, reads the switches on its limits from here ({@link JointSimulation#readsLimits}).
   *
   * @param definition what the robot is: its sensors
   * @param joints the joints' physics, which a limit switch reads its joint's position from
   */
  public SensorSimulation(RobotDefinition definition, JointSimulation joints) {
    this.joints = joints;
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Placed placed) {
        sensors.put(placed, new Simulated(placed, joints));
      }
    }
    joints.readsLimits(this);
  }

  /**
   * A sensor's device, simulated, for the robot to read.
   *
   * @throws IllegalArgumentException for a sensor that isn't the robot's, or a switch on a joint's
   *     motor controller's limit input, which its joint reads from its controller
   */
  public SensorIO io(Sensor.Placed sensor) {
    SensorWiring.ControllerLimit pin = ControllerPins.onPin(sensor);
    if (pin != null) {
      throw new IllegalArgumentException(
          sensor.label()
              + " is on "
              + possessive(pin.joint())
              + " controller's limit input: "
              + pin.joint()
              + " reads it from its controller");
    }
    return simulated(sensor);
  }

  /** Whether a switch reads tripped, as it reads each time it's asked: for a joint's controller. */
  BooleanSupplier tripped(Sensor.Switch sensor) {
    Simulated simulated = simulated(sensor);
    return simulated::tripped;
  }

  /**
   * Trips a beam or a contact switch, or clears it: something in its beam, or nothing. A limit
   * switch follows its joint instead.
   *
   * @throws IllegalArgumentException for a limit switch on a simulated joint, or a sensor that
   *     isn't simulated
   */
  public void trip(Sensor.Switch sensor, boolean tripped) {
    if (sensor.kind() instanceof Sensor.Switch.Limit limit && joints.simulates(limit.joint())) {
      throw new IllegalArgumentException(
          sensor.label() + " trips where " + limit.joint().label() + " really is: move it there");
    }
    simulated(sensor).tripped = tripped;
  }

  /**
   * What a rangefinder sees: the distance to the nearest thing along its axis.
   *
   * @throws IllegalArgumentException for a sensor that isn't simulated
   */
  public void see(Sensor.Rangefinder sensor, Distance distance) {
    simulated(sensor).meters = distance.in(Meters);
  }

  /**
   * Cuts a switch's wire: it reads as its polarity reads an open circuit, whatever trips it, a
   * normally closed one tripped and a normally open one clear. Null mends it.
   */
  public void cut(Sensor.Switch sensor, SensorWiring.@Nullable Polarity polarity) {
    simulated(sensor).cut =
        polarity == null ? null : polarity == SensorWiring.Polarity.NORMALLY_CLOSED;
  }

  /**
   * Silences a sensor, or brings it back: a CAN device that stops answering, as a loose connector
   * does.
   *
   * @throws IllegalArgumentException for a switch on a joint's motor controller's limit input,
   *     which is silent only with its controller
   */
  public void silence(Sensor.Placed sensor, boolean silent) {
    SensorWiring.ControllerLimit pin = ControllerPins.onPin(sensor);
    if (pin != null) {
      throw new IllegalArgumentException(
          sensor.label()
              + " is read through "
              + possessive(pin.joint())
              + " motor controller: it's lost only with it");
    }
    simulated(sensor).silent = silent;
  }

  private Simulated simulated(Sensor.Placed sensor) {
    Simulated simulated = sensors.get(sensor);
    if (simulated == null) {
      throw new IllegalArgumentException(sensor.label() + " isn't one of the robot's sensors");
    }
    return simulated;
  }
}

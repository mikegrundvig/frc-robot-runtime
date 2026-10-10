package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.command3.Scheduler;

/**
 * The fixtures' robot with some switches, each reading what a test says, for a stage's tests
 * without the simulator: {@code switches.trip(TURRET_RIGHT, true)}, then {@code switches.sensors()}
 * as read this loop.
 */
final class SwitchReadings {
  private final Map<Sensor, SensorIO.SensorIOInputs> said = new HashMap<>();
  private final Sensors sensors;

  /** The switches, each answering and clear until a test says otherwise. */
  SwitchReadings(Sensor.Switch... switches) {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(switches)).build();
    for (Sensor.Switch each : switches) {
      SensorIO.SensorIOInputs inputs = new SensorIO.SensorIOInputs();
      inputs.connected = true;
      said.put(each, inputs);
    }
    sensors =
        new Sensors(
            robot,
            placed ->
                new SensorIO() {
                  @Override
                  public void updateInputs(SensorIOInputs inputs) {
                    SensorIO.SensorIOInputs each = said.get(placed);
                    if (each != null) {
                      inputs.connected = each.connected;
                      inputs.tripped = each.tripped;
                    }
                  }
                },
            Scheduler.createIndependentScheduler());
  }

  /** Trips a switch, or clears it. */
  SwitchReadings trip(Sensor.Switch sensor, boolean tripped) {
    reading(sensor).tripped = tripped;
    return this;
  }

  /** Loses a switch, or finds it again. */
  SwitchReadings lose(Sensor.Switch sensor, boolean lost) {
    reading(sensor).connected = !lost;
    return this;
  }

  /** The robot's sensors, as read now. */
  Sensors sensors() {
    sensors.read();
    return sensors;
  }

  private SensorIO.SensorIOInputs reading(Sensor.Switch sensor) {
    SensorIO.SensorIOInputs reading = said.get(sensor);
    if (reading == null) {
      throw new IllegalArgumentException(sensor.label() + " isn't one of these switches");
    }
    return reading;
  }
}

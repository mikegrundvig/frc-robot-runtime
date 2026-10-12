package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.michaelgrundvig.frc.runtime.behavior.Debounce;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.Distance;

/**
 * A CANrange, set from the definition: its long-range mode where the definition's range is past the
 * 1 m its short-range mode reads (CTRE's CANrange User's Guide: 1 m short, 3 m long), and its cone
 * the definition's field of view, within the 6.75° to 27° it takes, the same across and up and
 * down.
 */
public final class SensorIOCanRange implements SensorIO {
  /** How far its short-range mode reads, in meters. */
  static final double SHORT_RANGE_METERS = 1.0;

  /** Its cone's narrowest and widest, in degrees. */
  static final double NARROWEST_DEGREES = 6.75;

  static final double WIDEST_DEGREES = 27;

  /** How long to wait for the CANrange to answer at startup, each try. */
  private static final double STARTUP_TIMEOUT_SECONDS = 0.25;

  private static final int TRIES = 5;

  private final StatusSignal<Distance> distance;

  /**
   * Whether it answered within the last half second, kept in whole nanoseconds ({@link Debounce})
   * so that every driver calls a device silent in the same loop.
   */
  private final Debounce connected = new Debounce(0.5, DebounceType.FALLING);

  /**
   * @param rangefinder what the definition says it is: its range and cone
   * @param input its CAN ID and bus
   */
  public SensorIOCanRange(Sensor.Rangefinder rangefinder, SensorWiring.CanRange input) {
    CANrange device = new CANrange(input.id(), PhoenixMechanism.bus(input.bus()));
    CANrangeConfiguration config = configuration(rangefinder);
    for (int i = 0; i < TRIES; i++) {
      if (device.getConfigurator().apply(config, STARTUP_TIMEOUT_SECONDS).isOK()) {
        break;
      }
    }
    distance = device.getDistance();
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, distance);
  }

  /** Its settings, from the definition's range and cone. */
  static CANrangeConfiguration configuration(Sensor.Rangefinder rangefinder) {
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ToFParams.UpdateMode =
        rangefinder.range().in(Meters) > SHORT_RANGE_METERS
            ? UpdateModeValue.LongRangeUserFreq
            : UpdateModeValue.ShortRange100Hz;
    config.ToFParams.UpdateFrequency = RobotLoop.FREQUENCY_HZ;
    double cone =
        Math.max(
            NARROWEST_DEGREES, Math.min(WIDEST_DEGREES, rangefinder.fieldOfView().in(Degrees)));
    config.FovParams.FOVRangeX = cone;
    config.FovParams.FOVRangeY = cone;
    return config;
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = connected.calculate(distance.refresh().getStatus().isOK());
    inputs.meters = distance.getValueAsDouble();
  }
}

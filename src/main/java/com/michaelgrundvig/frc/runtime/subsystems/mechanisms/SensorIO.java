package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import org.littletonrobotics.junction.AutoLog;

/**
 * One switch's or rangefinder's door to hardware: a Systemcore port ({@link SensorIOPort}), a
 * CANdi's input ({@link SensorIOCandi}), a CANrange ({@link SensorIOCanRange}), the simulator's
 * ({@code com.michaelgrundvig.frc.runtime.sim.SensorSimulation}), or nothing, in replay, where its
 * inputs come from the log. A switch on a joint's motor controller's limit input has none of its
 * own: its joint reads it from its controller ({@link JointIO.JointIOInputs#forwardLimit}), and
 * hands it to the sensors with these same inputs.
 *
 * <p>Its wiring's polarity stays behind the door: a switch's reading is tripped or clear, never
 * high or low, wherever the device can be told which is which (a CANdi), and in the IO where it
 * can't (a Systemcore port).
 */
public interface SensorIO {
  @AutoLog
  class SensorIOInputs {
    /**
     * Whether it answered lately: a CAN device heard within half a second, or a port read. One that
     * isn't holds no condition either way.
     */
    public boolean connected = false;

    /** A switch's reading: whether it's tripped (pressed, its beam blocked). */
    public boolean tripped = false;

    /** A rangefinder's reading: the distance to the nearest thing along its axis, in meters. */
    public double meters = Double.NaN;
  }

  default void updateInputs(SensorIOInputs inputs) {}
}

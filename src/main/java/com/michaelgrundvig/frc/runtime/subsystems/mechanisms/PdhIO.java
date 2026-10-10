package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import org.littletonrobotics.junction.AutoLog;

/**
 * The PDH's door to hardware: REV's Power Distribution Hub on the robot ({@link PdhIORev}), the
 * simulator's ({@code com.michaelgrundvig.frc.runtime.sim.PdhIOSim}), or nothing, in replay, where
 * its inputs come from the log, and on a robot that declares no PDH.
 */
public interface PdhIO {
  @AutoLog
  class PdhIOInputs {
    /** Whether it answered lately: false after half a second of silence. */
    public boolean connected = false;

    /** The current each channel draws, in amps, by its number: 0 to 23 on a REV PDH. */
    public double[] channelCurrentAmps = new double[0];

    /** The current its channels draw together, in amps. */
    public double totalCurrentAmps = 0.0;

    /** The battery's voltage, as it reads it at its input. */
    public double volts = 0.0;
  }

  default void updateInputs(PdhIOInputs inputs) {}
}

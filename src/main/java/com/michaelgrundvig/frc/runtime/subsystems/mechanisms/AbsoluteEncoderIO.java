package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import org.littletonrobotics.junction.AutoLog;

/**
 * A joint's absolute encoder where the robot reads it itself: on one of Systemcore's ports ({@link
 * AbsoluteEncoderIOPort}), the simulator's ({@code
 * com.michaelgrundvig.frc.runtime.sim.AbsoluteEncoderIOSim}), or nothing, in replay, where its
 * inputs come from the log. A CANcoder or a SPARK's data port is read by the joint's motor
 * controller instead ({@link JointIO}).
 *
 * <p>Its reading is raw, as the port reads it: the joint's mechanism applies the encoder's offset
 * and direction from the robot's definition ({@code MechanismWiring.PortEncoder#read}), so a replay
 * with a corrected offset seeds where the robot should have.
 */
public interface AbsoluteEncoderIO {
  @AutoLog
  class AbsoluteEncoderIOInputs {
    /** Whether it gave a reading this loop: a duty-cycle encoder's pulses arriving. */
    public boolean connected = false;

    /** Its signal's share of its range, from 0 to 1: its pulse's, or its voltage's. */
    public double share = 0.0;
  }

  default void updateInputs(AbsoluteEncoderIOInputs inputs) {}
}

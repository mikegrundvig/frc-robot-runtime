package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import org.jspecify.annotations.Nullable;
import org.wpilib.hardware.rotation.AnalogEncoder;
import org.wpilib.hardware.rotation.DutyCycleEncoder;

/**
 * An absolute encoder on one of Systemcore's ports, through WPILib: a duty-cycle encoder, such as a
 * REV Through Bore's absolute output, or an analog encoder or potentiometer. Read raw, its full
 * range from 0 to 1: the offset and direction are the joint's mechanism's to apply.
 */
public final class AbsoluteEncoderIOPort implements AbsoluteEncoderIO, AutoCloseable {
  private final @Nullable DutyCycleEncoder dutyCycle;
  private final @Nullable AnalogEncoder analog;

  public AbsoluteEncoderIOPort(MechanismWiring.PortEncoder port) {
    if (port.signal() == MechanismWiring.PortEncoder.Signal.DUTY_CYCLE) {
      dutyCycle = new DutyCycleEncoder(port.port(), 1, 0);
      analog = null;
    } else {
      dutyCycle = null;
      analog = new AnalogEncoder(port.port(), 1, 0);
    }
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    DutyCycleEncoder pulses = dutyCycle;
    AnalogEncoder voltage = analog;
    if (pulses != null) {
      inputs.connected = pulses.isConnected();
      inputs.share = pulses.get();
    } else if (voltage != null) {
      // A voltage can't say it's unplugged: a loose wire reads as somewhere.
      inputs.connected = true;
      inputs.share = voltage.get();
    }
  }

  /** Frees its port. */
  @Override
  public void close() {
    if (dutyCycle != null) {
      dutyCycle.close();
    }
    if (analog != null) {
      analog.close();
    }
  }
}

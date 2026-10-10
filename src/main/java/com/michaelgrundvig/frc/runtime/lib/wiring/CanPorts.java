package com.michaelgrundvig.frc.runtime.lib.wiring;

import com.michaelgrundvig.frc.world.robot.CanBus;
import org.wpilib.hardware.bus.CANPort;

/** Systemcore's CAN ports, from the model's names to WPILib's. */
public final class CanPorts {
  private CanPorts() {}

  /** WPILib's port for one of Systemcore's CAN ports. */
  public static CANPort of(CanBus.Systemcore port) {
    return CANPort.valueOf(port.name());
  }
}

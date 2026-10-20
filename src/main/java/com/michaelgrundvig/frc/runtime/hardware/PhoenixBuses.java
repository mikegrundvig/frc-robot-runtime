package com.michaelgrundvig.frc.runtime.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.michaelgrundvig.frc.world.robot.CanBus;

/**
 * What Phoenix says of a CANivore: whether this computer sees it. A laptop driving real devices
 * says it as it starts, so a CANivore left unplugged, or one Windows hasn't found, shows at once
 * rather than as twelve devices that don't answer.
 */
public final class PhoenixBuses {
  private PhoenixBuses() {}

  /**
   * One line about a CANivore: "the CANivore canivore: seen, 4% of the bus in use", or "the
   * CANivore canivore: not seen (CANNotFound: ...)", Phoenix's own words for why.
   */
  public static String status(CanBus.Canivore canivore) {
    CANBus.CANBusStatus status = new CANBus(canivore.name()).getStatus();
    StatusCode code = status.Status;
    if (code.isOK()) {
      return canivore.shown()
          + ": seen, "
          + Math.round(status.BusUtilization * 100)
          + "% of the bus in use";
    }
    return canivore.shown() + ": not seen (" + code.getName() + ": " + code.getDescription() + ")";
  }
}

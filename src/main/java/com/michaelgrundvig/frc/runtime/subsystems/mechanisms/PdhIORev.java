package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.behavior.Debounce;
import com.michaelgrundvig.frc.runtime.lib.wiring.CanPorts;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Pdh;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.math.filter.Debouncer.DebounceType;

/**
 * REV's Power Distribution Hub, read through WPILib's {@code PowerDistribution} on Systemcore's CAN
 * port at its ID, as the robot's definition declares it. A PDH that doesn't answer reads no
 * battery, which a powered one always does (it runs from 4.7 V up, REV's specifications), so its
 * voltage reading says whether it answered.
 */
public final class PdhIORev implements PdhIO {
  private final PowerDistribution pdh;

  /**
   * Whether it answered within the last half second, kept in whole nanoseconds ({@link Debounce})
   * so that every driver calls a device silent in the same loop.
   */
  private final Debounce connected = new Debounce(0.5, DebounceType.FALLING);

  /**
   * @param pdh the robot's PDH, as its definition declares it: on Systemcore's own CAN, which the
   *     definition holds it to
   */
  public PdhIORev(Pdh pdh) {
    CanBus.Port port = (CanBus.Port) pdh.bus();
    this.pdh =
        new PowerDistribution(CanPorts.of(port.port()), pdh.id(), PowerDistribution.ModuleType.REV);
  }

  @Override
  public void updateInputs(PdhIOInputs inputs) {
    double volts = pdh.getVoltage();
    inputs.connected = connected.calculate(volts > 0);
    inputs.volts = volts;
    inputs.channelCurrentAmps = pdh.getAllCurrents();
    inputs.totalCurrentAmps = pdh.getTotalCurrent();
  }
}

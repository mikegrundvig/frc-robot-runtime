package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.SensorWiring;
import org.wpilib.hardware.discrete.DigitalInput;

/**
 * A switch on one of Systemcore's own I/O ports, read as a digital input. Systemcore pulls its
 * inputs down (alpha spec: 4.7 kΩ), so a switch wired from the port to its supply reads high when
 * closed: a normally open one reads high as it trips, a normally closed one low. What a port reads
 * with nothing connected is open question S17.
 */
public final class SensorIOPort implements SensorIO, AutoCloseable {
  private final DigitalInput input;
  private final boolean trippedHigh;

  /**
   * @param port 0 to 5
   * @param polarity which raw signal means tripped
   */
  public SensorIOPort(int port, SensorWiring.Polarity polarity) {
    input = new DigitalInput(port);
    trippedHigh = polarity == SensorWiring.Polarity.NORMALLY_OPEN;
  }

  /** Frees its port: for a test, which makes several in one run. */
  @Override
  public void close() {
    input.close();
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = true;
    inputs.tripped = input.get() == trippedHigh;
  }
}

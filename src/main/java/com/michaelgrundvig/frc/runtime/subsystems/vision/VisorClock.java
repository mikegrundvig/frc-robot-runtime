package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.visor.NtpResponder;
import java.util.function.LongSupplier;

/**
 * The robot's clock as Visor's coprocessors sync to it: Visor's own responder, answering on a UDP
 * port, so each frame's capture time is stamped on the robot's clock. Once for all of the robot's
 * cameras on Visor.
 */
public final class VisorClock implements AutoCloseable {
  private final NtpResponder responder;

  private VisorClock(NtpResponder responder) {
    this.responder = responder;
  }

  /**
   * Starts answering the coprocessors' clock sync.
   *
   * @param port where they sync ({@link VisorPorts#clock}), 0 for one the system picks
   * @param robotNanos the robot's clock
   */
  public static VisorClock start(int port, LongSupplier robotNanos) {
    return new VisorClock(NtpResponder.start(port, robotNanos));
  }

  @Override
  public void close() {
    responder.close();
  }
}

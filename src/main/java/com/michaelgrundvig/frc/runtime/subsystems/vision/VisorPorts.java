package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.visor.NtpResponder;
import com.michaelgrundvig.frc.visor.VisorReceiver;

/**
 * The UDP ports a robot listens on for Visor: the one its coprocessors send frames to, and the one
 * they sync their clocks with. A robot listens on Visor's own; a test listens on ports the system
 * picks as each is bound, which no other program on the computer can take.
 *
 * @param frames where Visor's frames arrive, 0 for one the system picks
 * @param clock where the coprocessors sync their clocks ({@link NtpResponder}), 0 for one the
 *     system picks
 */
public record VisorPorts(int frames, int clock) {
  /** Visor's own ports, which a robot's coprocessors send to. */
  public static final VisorPorts VISORS =
      new VisorPorts(VisorReceiver.DEFAULT_PORT, NtpResponder.DEFAULT_PORT);

  /** Ports the system picks as each is bound: a test's. */
  public static final VisorPorts ANY_FREE = new VisorPorts(0, 0);
}

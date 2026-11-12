package com.michaelgrundvig.frc.runtime.lib.phoenix;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.michaelgrundvig.frc.world.robot.CanBus;
import java.util.function.Supplier;

/**
 * How the IO layer sends a Phoenix device its settings as the robot starts: how many times, how
 * long each try waits, and whether the device confirmed.
 *
 * <p>A device on a bus that is seen is tried a few times, since a busy bus can drop one answer. A
 * device on a CANivore this computer doesn't see can't answer at all, so it is tried once: five
 * tries of a quarter second each, for every device, kept a robot with its CANivore unplugged from
 * starting its loop for fifteen seconds or more.
 *
 * <p>{@link #busSeen} is the one call here that asks Phoenix anything: whether it sees a CANivore.
 * It touches no device. The rest are plain functions of what they're handed.
 */
public final class PhoenixStartup {
  /** How many times a device on a bus that is seen is sent its settings, until it confirms. */
  public static final int TRIES_ON_A_BUS_SEEN = 5;

  /** How many times a device on a CANivore that isn't seen is sent its settings. */
  public static final int TRIES_ON_A_BUS_NOT_SEEN = 1;

  /** How long each try waits for the device to confirm. Startup can afford the wait. */
  public static final double TIMEOUT_SECONDS = 0.25;

  private PhoenixStartup() {}

  /**
   * Whether this computer sees the bus. A CANivore is asked about. One of Systemcore's own CAN
   * ports is taken as seen: it is part of the controller, and what Phoenix reports for one isn't
   * confirmed yet.
   */
  public static boolean busSeen(CanBus bus) {
    return switch (bus) {
      case CanBus.Port _ -> true;
      case CanBus.Canivore canivore -> new CANBus(canivore.name()).getStatus().Status.isOK();
    };
  }

  /** How many times each device on a bus is sent its settings at startup. */
  public static int tries(boolean busSeen) {
    if (busSeen) {
      return TRIES_ON_A_BUS_SEEN;
    }
    return TRIES_ON_A_BUS_NOT_SEEN;
  }

  /**
   * Sends a device something up to so many times, and says whether it confirmed. Only for startup,
   * where waiting is fine.
   *
   * @param attempt one send, which waits for the device's answer and returns it
   */
  public static boolean confirmed(int tries, Supplier<StatusCode> attempt) {
    for (int i = 0; i < tries; i++) {
      if (attempt.get().isOK()) {
        return true;
      }
    }
    return false;
  }
}

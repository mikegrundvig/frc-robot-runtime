package com.michaelgrundvig.frc.runtime.hardware;

import com.michaelgrundvig.frc.world.robot.CanBus;
import org.wpilib.framework.RobotBase;

/**
 * The computer the robot program runs on: a robot's Systemcore, or a desktop or laptop.
 *
 * <p>Most of the time this follows from the program's mode: a robot runs on Systemcore, and the
 * simulator and a replay run on a laptop. One combination needs it said apart: a laptop driving
 * real CTRE devices through CTRE's hardware-attached simulation ({@code ./gradlew run -PhwSim}).
 * Its devices are real, so it runs as a robot does, but the laptop has none of Systemcore's own
 * parts. So its hardware is only what it can reach: the CTRE devices on a CANivore plugged into it.
 */
public enum RunsOn {
  /** A robot's Systemcore, with its own CAN ports, I/O ports and IMU. */
  SYSTEMCORE,

  /**
   * A desktop or laptop. With real devices, it reaches only CTRE's devices on a CANivore plugged
   * into it: not Systemcore's CAN ports or I/O ports, not its IMU, not a PDH, and not REV's SPARKs,
   * which have no way to real hardware from a laptop's program.
   */
  DESKTOP;

  /** The computer this program runs on, as WPILib says it. */
  public static RunsOn current() {
    if (RobotBase.isReal()) {
      return SYSTEMCORE;
    }
    return DESKTOP;
  }

  /**
   * Whether this computer can reach devices on a CAN bus: Systemcore reaches every bus, a laptop
   * only a CANivore plugged into it.
   */
  public boolean reaches(CanBus bus) {
    if (this == SYSTEMCORE) {
      return true;
    }
    return bus instanceof CanBus.Canivore;
  }

  /** Whether this computer has Systemcore's own parts: its I/O ports, its IMU, and the PDH. */
  public boolean hasSystemcoresOwnParts() {
    return this == SYSTEMCORE;
  }
}

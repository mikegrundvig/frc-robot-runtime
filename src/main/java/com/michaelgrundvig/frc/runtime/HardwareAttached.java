package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.RobotKind;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;

/**
 * A laptop driving a robot's real CTRE devices: CTRE's hardware-attached simulation, {@code
 * ./gradlew run -PhwSim} (docs/bench.md). The program runs in WPILib's simulation, as the simulator
 * does, but in {@link RunMode#REAL}, as on a robot: the devices are real, there's no simulated
 * world, and Pitwall's page only shows. The robot starts disabled, and enables from WPILib's
 * simulator GUI or a Driver Station.
 *
 * <p>This says what the program decides for each mode and computer, in one place, and prints the
 * banner a mentor sees as such a program starts.
 */
final class HardwareAttached {
  private HardwareAttached() {}

  /** The banner's first line, which says what this is before anything else is printed. */
  static final String WARNING = "HARDWARE-ATTACHED: real motors will move when enabled";

  /** Whether this is a laptop driving real devices: real devices, and no robot to run on. */
  static boolean applies(RunMode mode, RunsOn runsOn) {
    return mode == RunMode.REAL && runsOn == RunsOn.DESKTOP;
  }

  /**
   * What the program drives, as Pitwall's page needs to know it: real motors on a robot and on a
   * laptop driving real devices, which the page never enables; a simulated robot otherwise. A
   * replay serves no page at all.
   */
  static RobotKind pitwallKind(RunMode mode) {
    if (mode == RunMode.REAL) {
      return RobotKind.REAL;
    }
    return RobotKind.SIMULATED;
  }

  /**
   * The banner a laptop driving real devices prints as it starts: the warning, the devices it will
   * open, those it can't reach, and how it's enabled and stopped.
   */
  static List<String> banner(RobotDefinition definition) {
    List<String> lines = new ArrayList<>();
    String rule = "=".repeat(78);
    lines.add(rule);
    lines.add(WARNING);
    lines.add(
        "Robot \""
            + definition.name()
            + "\", driven from this computer through CTRE's hardware-attached simulation.");
    List<CanDevice> opened = new ArrayList<>();
    List<CanDevice> notOpened = new ArrayList<>();
    for (CanDevice device : definition.devices()) {
      if (RunsOn.DESKTOP.reaches(device.bus())) {
        opened.add(device);
      } else {
        notOpened.add(device);
      }
    }
    if (opened.isEmpty()) {
      lines.add("It opens no devices: nothing in this robot's wiring is on a CANivore.");
    } else {
      lines.add("It opens these devices:");
      opened.forEach(device -> lines.add("  " + shown(device)));
    }
    if (!notOpened.isEmpty()) {
      lines.add("Not opened, since only a robot reaches them:");
      notOpened.forEach(device -> lines.add("  " + shown(device)));
    }
    lines.add(
        "Nor is anything on Systemcore's I/O ports, or its IMU: the alert Robot/outOfReach names"
            + " each one this robot has.");
    lines.add(
        "It starts disabled. Enable and disable it in WPILib's simulator GUI (Robot State) or a"
            + " Driver Station. Pitwall's page only shows, and tuning is off until switched on"
            + " there.");
    lines.add("To stop at once: Disabled in the GUI, Ctrl+C here, or the robot's main breaker.");
    lines.add(rule);
    return List.copyOf(lines);
  }

  /** Each CANivore the robot's devices are on, once each, in the order the devices list them. */
  static List<CanBus.Canivore> canivores(RobotDefinition definition) {
    List<CanBus.Canivore> canivores = new ArrayList<>();
    for (CanDevice device : definition.devices()) {
      if (device.bus() instanceof CanBus.Canivore canivore && !canivores.contains(canivore)) {
        canivores.add(canivore);
      }
    }
    return List.copyOf(canivores);
  }

  /** A device as the banner lists it: "front-left drive: TalonFX 23 on the CANivore canivore". */
  private static String shown(CanDevice device) {
    return device.name() + ": " + device.kind() + " " + device.id() + " on " + device.bus().shown();
  }
}

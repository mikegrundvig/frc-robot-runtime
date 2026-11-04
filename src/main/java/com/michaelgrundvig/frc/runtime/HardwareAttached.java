package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.RobotKind;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import org.wpilib.system.Filesystem;

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

  /** Set by {@code ./gradlew run -PhwSim}: the team number the project deploys with. */
  static final String TEAM_PROPERTY = "frc.team";

  /**
   * Set by {@code ./gradlew run -PhwSim}: the deploy folder the build put together as a robot's is,
   * the project's {@code src/main/deploy} with Visor's release and pack beside it.
   */
  static final String DEPLOY_PROPERTY = "frc.deploy";

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

  /**
   * The team number a laptop driving real devices goes by, as Systemcore gives a robot its own: the
   * one the project deploys with, which {@code ./gradlew run -PhwSim} passes. WPILib's simulation
   * has none, and the coprocessors find the robot by it. Empty for none, or 0.
   *
   * @param asked {@link #TEAM_PROPERTY}, if it's set
   * @throws IllegalArgumentException if it isn't a number
   */
  static OptionalInt team(Optional<String> asked) {
    if (asked.isEmpty() || asked.get().isBlank()) {
      return OptionalInt.empty();
    }
    int team;
    try {
      team = Integer.parseInt(asked.get().strip());
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException(
          TEAM_PROPERTY + " \"" + asked.get() + "\" isn't a team number", e);
    }
    return team > 0 ? OptionalInt.of(team) : OptionalInt.empty();
  }

  /**
   * What a laptop driving real devices says of its coprocessors as it starts: the address it has to
   * take for them, or that it can't install Visor without a team number. Nothing for a robot with
   * no coprocessors.
   */
  static List<String> coprocessorLines(int boards, OptionalInt team) {
    if (boards == 0) {
      return List.of();
    }
    if (team.isEmpty()) {
      return List.of(
          "No team number, so Visor isn't installed or configured on the coprocessors: store it"
              + " with ./gradlew setTeamNumber -PteamNumber=NNNN, and run again.");
    }
    return List.of(
        "Team "
            + team.getAsInt()
            + ": the coprocessors send their frames to, and ask the time of, "
            + robotAddress(team.getAsInt())
            + ". Give this computer that address on the robot's network, and let Java receive UDP"
            + " through its firewall.");
  }

  /** The robot's own address for a team, 10.TE.AM.2, as Visor works it out. */
  static String robotAddress(int team) {
    return "10." + team / 100 + "." + team % 100 + ".2";
  }

  /**
   * The deploy folder the program reads its coprocessors' packs, Visor's release and its venues
   * from: the one {@code ./gradlew run -PhwSim} put together, as a robot's is, or else WPILib's.
   * WPILib's is {@code src/main/deploy} on a laptop, which lacks what the build adds to a deploy.
   *
   * @param asked {@link #DEPLOY_PROPERTY}, if it's set
   */
  static Path deployDirectory(Optional<String> asked) {
    if (asked.isPresent() && !asked.get().isBlank()) {
      return Path.of(asked.get());
    }
    return Filesystem.getDeployDirectory().toPath();
  }

  /** The deploy folder this program reads ({@link #deployDirectory(Optional)}). */
  static Path deployDirectory() {
    return deployDirectory(Optional.ofNullable(System.getProperty(DEPLOY_PROPERTY)));
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

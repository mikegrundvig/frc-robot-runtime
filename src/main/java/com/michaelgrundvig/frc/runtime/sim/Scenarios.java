package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.simulation.FaultCatalog.BACKUP_IMU;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.CLOCK_OFFSET;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.DRIFT;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.ERRORS;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.EXTRA;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.FORWARD;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.FROZEN;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.GRIP;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.GYRO;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.HIT;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.LATE;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.LEFT;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.MOVED;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.NOISE;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.OFFSET;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.PITCH;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.RESET;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.ROBOT;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.ROLL;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.SCALE;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.SHOVED;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.SLIPPING;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.SLIPS;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.STALE;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.TILTED;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.TO_A_BUMP;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.TURN;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.TURNED;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.TURNS;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.UNPLUGGED;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.UP;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.WORN;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.YAW;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * The simulator's scenarios, by name: {@code ./gradlew run -Pscenario=<name>} starts the simulator
 * in one, the Pitwall page applies one to a running simulation, and {@code ./gradlew scenarios}
 * lists them. Each is a set of the simulated world's faults ({@link Faults}), written for any game:
 * a camera names the robot's first, and a bump is the field's nearest.
 *
 * <p>To add one, add it to {@link #ALL}, made of the faults it applies. A fault the simulated world
 * can't make yet starts in {@link Faults}; docs/simulator.md lists those.
 */
public final class Scenarios {
  /** How far a collision or a push moves the robot: to its left. */
  static final double KNOCKED_LEFT_METERS = 0.6;

  /** Every scenario: faults in the robot's sensors first, then what can happen to it. */
  public static final List<Scenario> ALL =
      List.of(
          Scenario.builder()
              .name("camera-low")
              .description("The robot's first camera knocked 4 in low on its mount.")
              .sensorFaults(
                  world ->
                      List.of(
                          Setting.of(
                              firstCamera(world),
                              MOVED,
                              Map.of(FORWARD, 0.0, LEFT, 0.0, UP, -4.0))))
              .events(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("camera-turned")
              .description("The robot's first camera turned 2° on its mount.")
              .sensorFaults(
                  world ->
                      List.of(
                          Setting.of(
                              firstCamera(world), TURNED, Map.of(ROLL, 0.0, PITCH, 0.0, YAW, 2.0))))
              .events(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("camera-late")
              .description(
                  "The robot's first camera 150 ms later than its configuration says: its images"
                      + " arrive past the margin allowed, and are left out.")
              .sensorFaults(
                  world -> List.of(Setting.of(firstCamera(world), LATE, Map.of(EXTRA, 150.0))))
              .events(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("camera-clock-off")
              .description(
                  "The robot's first camera's clock 20 ms off the robot's: its images arrive on"
                      + " time but are stamped 20 ms after they were taken, so each shows the robot"
                      + " where it was a loop before its stamp says.")
              .sensorFaults(
                  world ->
                      List.of(Setting.of(firstCamera(world), CLOCK_OFFSET, Map.of(OFFSET, 20.0))))
              .events(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("gyro-scale")
              .description(
                  "The gyro reads 15% more turn than the robot makes; the backup IMU doesn't.")
              .sensorFaults(Scenario.faults(Setting.of(GYRO, ERRORS, Map.of(SCALE, 1.15))))
              .events(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("worn-tread")
              .description(
                  "Every wheel's tread worn 3%: the robot travels 3% less than odometry says.")
              .sensorFaults(Scenario.faults(everyModule(WORN, Map.of(SCALE, 0.97))))
              .events(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("module-zero-off")
              .description(
                  "The front left module's absolute encoder zeroed 5° off: it points 5° from where"
                      + " the code thinks.")
              .sensorFaults(
                  Scenario.faults(
                      Setting.of(
                          FaultCatalog.module(0), FaultCatalog.ZEROED_OFF, Map.of(OFFSET, 5.0))))
              .events(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("exact-sensors")
              .description(
                  "Wheels and sensors exactly as configured, not a season-worn robot's: odometry is"
                      + " the truth.")
              .sensorFaults(Scenario.faults(exactSensors()))
              .events(Scenario.NOTHING)
              .build(),
          Scenario.builder()
              .name("collision")
              .description(
                  "Another robot hits it: knocked 0.6 m to its left, a jolt its IMU feels.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(
                  Scenario.faults(
                      Setting.of(ROBOT, HIT, Map.of(FORWARD, 0.0, LEFT, KNOCKED_LEFT_METERS))))
              .build(),
          Scenario.builder()
              .name("shove")
              .description(
                  "Pushed 0.6 m to its left without a jolt: neither its wheels nor its IMU notice.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(
                  Scenario.faults(
                      Setting.of(
                          ROBOT,
                          SHOVED,
                          Map.of(FORWARD, 0.0, LEFT, KNOCKED_LEFT_METERS, TURN, 0.0))))
              .build(),
          Scenario.builder()
              .name("wheel-slip")
              .description(
                  "The front left wheel spins half a turn without moving the robot: its encoder"
                      + " counts travel the floor never saw.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(
                  Scenario.faults(Setting.of(FaultCatalog.module(0), SLIPS, Map.of(TURNS, 0.5))))
              .build(),
          // An approximation: the simulator can tilt the robot and scale how far all its wheels
          // move it, but can't lift one wheel. A lifted wheel would spin free and disagree with the
          // others; these all slip alike, and the robot isn't raised.
          Scenario.builder()
              .name("wheel-lifted")
              .description(
                  "Its front left corner up on a game piece: tilted 5° nose up and 5° left side up,"
                      + " moving three quarters as far as its wheels turn, for the rest of the run"
                      + " (the tilt holds over any bump). Approximate: every wheel slips alike, so"
                      + " they never disagree as a lifted one would; the robot isn't raised.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(
                  // WPILib's convention: nose up is a negative pitch, left side up a positive roll.
                  Scenario.faults(
                      Setting.of(ROBOT, TILTED, Map.of(PITCH, -5.0, ROLL, 5.0)),
                      Setting.of(ROBOT, SLIPPING, Map.of(GRIP, 0.75))))
              .build(),
          Scenario.builder()
              .name("gyro-unplugged")
              .description("The gyro unplugged: the backup IMU has to keep the heading.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(Scenario.faults(Setting.of(GYRO, UNPLUGGED)))
              .build(),
          Scenario.builder()
              .name("gyro-frozen")
              .description("The gyro freezes: it keeps reporting, the same yaw every time.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(Scenario.faults(Setting.of(GYRO, FROZEN)))
              .build(),
          Scenario.builder()
              .name("gyro-reset")
              .description(
                  "The gyro resets to zero mid-run, as one that browns out and restarts does: its"
                      + " heading jumps.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(Scenario.faults(Setting.of(GYRO, RESET)))
              .build(),
          Scenario.builder()
              .name("imus-unplugged")
              .description(
                  "The gyro and the backup IMU both unplugged: the heading rests on the wheels"
                      + " alone.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(
                  Scenario.faults(Setting.of(GYRO, UNPLUGGED), Setting.of(BACKUP_IMU, UNPLUGGED)))
              .build(),
          Scenario.builder()
              .name("encoder-stale")
              .description(
                  "The front left module's encoders go stale: their readings stop changing while"
                      + " the wheel turns on, so the wheels disagree.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(Scenario.faults(Setting.of(FaultCatalog.module(0), STALE)))
              .build(),
          Scenario.builder()
              .name("camera-frozen")
              .description(
                  "The robot's first camera freezes: it keeps sending its last image's tags as"
                      + " new ones.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(world -> List.of(Setting.of(firstCamera(world), FROZEN)))
              .build(),
          Scenario.builder()
              .name("bump-crossing")
              .description(
                  "Carried to 1 m short of the nearest bump's ramp, facing across it, to drive"
                      + " over. Its gyro feels the turn but its wheels see no move, so only its"
                      + " cameras can say where it is. Needs a field with bumps.")
              .sensorFaults(Scenario.NO_FAULTS)
              .events(
                  world -> {
                    if (!Faults.hasBumps(world)) {
                      throw new IllegalArgumentException(
                          "this scenario carries the robot to a bump, and the field has none");
                    }
                    return List.of(Setting.of(ROBOT, TO_A_BUMP));
                  })
              .build());

  private Scenarios() {}

  /**
   * The scenario with this name.
   *
   * @throws IllegalArgumentException naming every scenario there is, when none has this name
   */
  public static Scenario named(String name) {
    for (Scenario scenario : ALL) {
      if (scenario.name().equals(name)) {
        return scenario;
      }
    }
    throw new IllegalArgumentException(
        "No simulator scenario is named \""
            + name
            + "\". The scenarios: "
            + ALL.stream().map(Scenario::name).collect(Collectors.joining(", ")));
  }

  /** Every scenario, one a line: its name, then what it does. */
  static String listing() {
    return ALL.stream().map(Scenarios::line).collect(Collectors.joining());
  }

  /** A scenario's line in the list. */
  static String line(Scenario scenario) {
    return String.format(Locale.ROOT, "%-16s %s%n", scenario.name(), scenario.description());
  }

  /**
   * For the build: with no arguments, lists the scenarios ({@code ./gradlew scenarios}); with a
   * name, gives that scenario's line, or stops with the list when there is none by that name, so
   * {@code ./gradlew run -Pscenario=<name>} stops at a misspelled name before the simulator starts.
   */
  public static void main(String... args) {
    try {
      System.out.print(args.length == 0 ? listing() : line(named(args[0])));
    } catch (IllegalArgumentException e) {
      System.err.println(e.getMessage());
      System.exit(1);
    }
  }

  /**
   * Declares every scenario that can happen to this world's robot to Pitwall's simulator controls,
   * where the page applies them. One that needs what the robot or its field lacks (a camera, a
   * bump) is left out.
   */
  public static void addTo(Components components, SimulatedWorld world) {
    for (Scenario scenario : ALL) {
      List<Setting> faults;
      try {
        faults = scenario.faultsIn(world);
      } catch (IllegalArgumentException needsAPartItLacks) {
        continue;
      }
      components.scenario(scenario.name(), scenario.description(), faults);
    }
  }

  /** The robot's first camera, which the camera scenarios knock. */
  private static String firstCamera(SimulatedWorld world) {
    List<String> cameras = world.cameraNames();
    if (cameras.isEmpty()) {
      throw new IllegalArgumentException("this scenario knocks a camera, and the robot has none");
    }
    return FaultCatalog.camera(cameras.get(0));
  }

  /** The same fault on every module. */
  private static Setting[] everyModule(String fault, Map<String, Double> values) {
    List<Setting> settings = new ArrayList<>();
    for (int i = 0; i < FaultCatalog.MODULES.size(); i++) {
      settings.add(Setting.of(FaultCatalog.module(i), fault, values));
    }
    return settings.toArray(Setting[]::new);
  }

  /** Every wheel and sensor exactly as configured. */
  private static Setting[] exactSensors() {
    List<Setting> settings = new ArrayList<>();
    for (int i = 0; i < FaultCatalog.MODULES.size(); i++) {
      settings.add(Setting.of(FaultCatalog.module(i), WORN, Map.of(SCALE, 1.0)));
      settings.add(
          Setting.of(FaultCatalog.module(i), FaultCatalog.ZEROED_OFF, Map.of(OFFSET, 0.0)));
    }
    for (String imu : List.of(GYRO, BACKUP_IMU)) {
      settings.add(Setting.of(imu, ERRORS, Map.of(SCALE, 1.0, DRIFT, 0.0, NOISE, 0.0)));
    }
    return settings.toArray(Setting[]::new);
  }
}

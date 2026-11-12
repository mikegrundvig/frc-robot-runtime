package com.michaelgrundvig.frc.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.dashboard.DashboardStream;
import com.michaelgrundvig.frc.runtime.hardware.Simulation;
import com.michaelgrundvig.frc.runtime.hardware.World;
import com.michaelgrundvig.frc.runtime.sim.Scenario;
import com.michaelgrundvig.frc.runtime.sim.Scenarios;
import com.michaelgrundvig.frc.runtime.sim.SimulatedWorld;
import com.michaelgrundvig.frc.runtime.sim.SimulatorWiring;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.api.io.TempDir;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;

/**
 * The robot program itself. {@code ./gradlew run} starts the simulator ready to drive, in teleop on
 * a station, so a first-time driver is one press from driving; only a physics simulation that asked
 * for it may enable itself, since replay gets its enable state from the log.
 */
@ExtendWith(RobotSim.class)
class RobotTest {
  /**
   * Pitwall's Robot view is drawn from a session of the robot: its definition, and five minutes of
   * its log. There is one wherever the page is served, and none in a replay, which serves no page.
   */
  @Test
  void pitwallKeepsASessionOfTheRobotWhereverItsPageIsServed() {
    RobotConfig robot = ExampleRobot.COMPETITION;

    assertSame(robot.definition(), Robot.sessionFor(RunMode.SIM, robot).orElseThrow().definition());
    assertEquals(
        15_000,
        Robot.sessionFor(RunMode.REAL, robot).orElseThrow().capacity(),
        "five minutes of loops");
    assertTrue(Robot.sessionFor(RunMode.REPLAY, robot).isEmpty(), "a replay serves no page");
  }

  @Test
  void withoutTheGuiTheSimulatorStartsReadyButDisabled() {
    asARealRobotStarts();

    Robot.startAsAsked(RunMode.SIM, Robot.SimStart.READY);

    assertFalse(DriverStationSim.getEnabled(), "disabled: the page enables it");
    assertTrue(DriverStationSim.getDsAttached(), "driver station attached");
    assertEquals(RobotMode.TELEOPERATED, DriverStationSim.getRobotMode(), "in teleop");
    assertEquals(Optional.of(Alliance.BLUE), MatchState.getAlliance(), "on an alliance");
    assertEquals(OptionalInt.of(2), MatchState.getLocation(), "at the center start's station");
  }

  @Test
  void withTheGuiTheSimulatorStartsEnabled() {
    asARealRobotStarts();

    Robot.startAsAsked(RunMode.SIM, Robot.SimStart.ENABLED);

    assertTrue(DriverStationSim.getEnabled(), "enabled");
    assertTrue(DriverStationSim.getDsAttached(), "driver station attached");
    assertEquals(OptionalInt.of(2), MatchState.getLocation(), "at the center start's station");
  }

  @Test
  void aStationChosenInTheSimulatorIsKept() {
    DriverStationSim.setAllianceStationId(AllianceStationID.RED_3);
    DriverStationSim.notifyNewData();

    Robot.startAsAsked(RunMode.SIM, Robot.SimStart.READY);

    assertEquals(Optional.of(Alliance.RED), MatchState.getAlliance());
    assertEquals(OptionalInt.of(3), MatchState.getLocation());
  }

  @Test
  void startDisabledLeavesItAsARealRobotStarts() {
    asARealRobotStarts();

    Robot.startAsAsked(RunMode.SIM, Robot.SimStart.DISABLED);

    assertFalse(DriverStationSim.getEnabled());
    assertEquals(Optional.empty(), MatchState.getAlliance(), "no station");
  }

  @Test
  void aStartIsReadByNameAndAnyOtherStartsAsARealRobot() {
    assertEquals(Robot.SimStart.DISABLED, Robot.SimStart.named(Optional.empty()));
    assertEquals(Robot.SimStart.DISABLED, Robot.SimStart.named(Optional.of("sideways")));
    assertEquals(Robot.SimStart.READY, Robot.SimStart.named(Optional.of("ready")));
    assertEquals(Robot.SimStart.ENABLED, Robot.SimStart.named(Optional.of("ENABLED")));
  }

  @Test
  void replayNeverEnablesItself() {
    asARealRobotStarts();

    Robot.startAsAsked(RunMode.REPLAY, Robot.SimStart.ENABLED);

    assertFalse(DriverStationSim.getEnabled());
    assertEquals(Optional.empty(), MatchState.getAlliance(), "no station");
  }

  /**
   * The loop carries on past what the commands throw on a robot, and in a replay, which follows the
   * robot's path; the simulator stops at once, so a bug fails its test.
   */
  @Test
  void aRobotAndItsReplayCarryOnPastAThrowAndTheSimulatorStops() {
    assertEquals(Robot.AfterAThrow.CARRIES_ON, Robot.afterAThrow(RunMode.REAL), "on a robot");
    assertEquals(Robot.AfterAThrow.CARRIES_ON, Robot.afterAThrow(RunMode.REPLAY), "replaying one");
    assertEquals(
        Robot.AfterAThrow.ENDS_THE_PROGRAM, Robot.afterAThrow(RunMode.SIM), "in the simulator");
  }

  /** Disabled with no station, as a robot starts: not the tests' own default, blue 2. */
  private static void asARealRobotStarts() {
    DriverStationSim.setAllianceStationId(AllianceStationID.UNKNOWN);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void theSimulatorStartsInTheScenarioAskedFor() {
    Scenario collision = Scenarios.named("collision");
    assertEquals(
        Optional.of(new Simulation.ScenarioAsked(collision.name(), collision.description())),
        Robot.simulation(RunMode.SIM, Optional.of("collision")).orElseThrow().scenario());
    assertEquals(
        Optional.empty(),
        Robot.simulation(RunMode.SIM, Optional.empty()).orElseThrow().scenario(),
        "the simulator, in no scenario");
  }

  /** Only the simulator's mode has the simulator: a replay runs what its log recorded. */
  @Test
  void aReplayRunsWhatItsLogRecordedNeverAScenario() {
    assertEquals(Optional.empty(), Robot.simulation(RunMode.REPLAY, Optional.of("collision")));
    assertEquals(Optional.empty(), Robot.simulation(RunMode.REAL, Optional.of("collision")));
  }

  @Test
  void aMisspelledScenarioStopsTheProgramWithTheList() {
    IllegalArgumentException refused =
        assertThrows(
            IllegalArgumentException.class,
            () -> Robot.simulation(RunMode.SIM, Optional.of("colision")));

    String message = String.valueOf(refused.getMessage());
    assertTrue(message.contains("camera-low, camera-turned"), message);
  }

  /** As ./gradlew run starts it: on the first loop the robot is enabled, then never again. */
  @Test
  void aScenarioStartsOnceTheRobotIsEnabled() {
    try (SimulatedWorld world =
        new SimulatedWorld(
            RobotSim.FIELD,
            RobotSim.WITHOUT_CAMERAS.definition(),
            RobotSim.massOf(RobotSim.WITHOUT_CAMERAS.definition()),
            SensorErrors.NONE)) {
      Components components = new Components(said -> {});
      world.addTo(components);
      List<Setting> collision = Scenarios.named("collision").eventsIn(world);
      Pose2d start = world.truePose();

      Optional<List<Setting>> waiting = Optional.of(collision);
      Optional<Components> controls = Optional.of(components);
      assertSame(
          collision,
          Robot.startWhenEnabled(waiting, controls, false).orElseThrow(),
          "disabled: waits");
      assertEquals(start, world.truePose());
      assertSame(
          collision,
          Robot.startWhenEnabled(waiting, Optional.empty(), true).orElseThrow(),
          "no simulator: waits");
      assertEquals(
          Optional.empty(), Robot.startWhenEnabled(waiting, controls, true), "enabled: started");
      assertEquals(0.6, world.truePose().getY() - start.getY(), 1e-9);
      assertEquals(
          Optional.empty(),
          Robot.startWhenEnabled(Optional.empty(), controls, true),
          "nothing more to start");
    }
  }

  /** Asked for by name, a scenario stops a program that has nothing to apply it through. */
  @Test
  void aScenarioWithoutTheSimulatorsControlsStopsTheProgram() {
    SimulatorWiring simulator = SimulatorWiring.start(SensorErrors.NONE, Optional.of("collision"));
    try (World world = madeWithoutCameras(simulator)) {
      IllegalStateException refused =
          assertThrows(
              IllegalStateException.class,
              () -> Robot.startScenario(Optional.of(simulator), Optional.empty()));

      assertTrue(
          String.valueOf(refused.getMessage()).contains("needs Pitwall's simulator controls"),
          refused.getMessage());
    }
    assertEquals(
        Optional.empty(),
        Robot.startScenario(Optional.empty(), Optional.empty()),
        "no simulator: nothing to do");
    assertEquals(
        Optional.empty(),
        Robot.startScenario(Optional.of(SimulatorWiring.of(SensorErrors.NONE)), Optional.empty()),
        "none asked for: nothing to do");
  }

  /** The world the simulator makes for the robot without cameras. */
  private static World madeWithoutCameras(SimulatorWiring simulator) {
    return simulator.world(
        RobotSim.FIELD,
        RobotSim.WITHOUT_CAMERAS.definition(),
        RobotSim.massOf(RobotSim.WITHOUT_CAMERAS.definition()));
  }

  /**
   * Started as ./gradlew run starts it: its sensor faults now, on a season-worn robot, even where
   * one leaves a part as it was built; what happens in it, later.
   */
  @Test
  void aScenarioStartsWithItsSensorFaults() {
    SimulatorWiring simulator =
        SimulatorWiring.start(SensorErrors.TYPICAL, Optional.of("exact-sensors"));
    try (World world = madeWithoutCameras(simulator)) {
      Components components = new Components(said -> {});
      world.addControlsTo(components);

      List<Setting> events =
          Robot.startScenario(Optional.of(simulator), Optional.of(components)).orElseThrow();

      assertEquals(List.of(), events);
      assertTrue(components.on().contains("Gyro: Errors"), components.on().toString());
    }
  }

  /** A scenario asked for by name that can't happen stops the program, saying why. */
  @Test
  void aScenarioThatCantHappenStopsTheProgram() {
    Components components = new Components(said -> {});

    IllegalStateException refused =
        assertThrows(
            IllegalStateException.class,
            () -> Robot.apply(List.of(Setting.of("Gyro", "Unplugged")), components));

    assertTrue(
        String.valueOf(refused.getMessage()).contains("no component named Gyro"),
        refused.getMessage());
  }

  /**
   * The dashboards' stream is guarded wherever it streams, so a throw in NetworkTables' publisher
   * stops the stream and not the log: thinned on a robot's field, every loop in the simulator.
   */
  @Test
  void theDashboardsStreamIsGuardedOnARobotAndInTheSimulator(@TempDir Path logs) {
    for (RunMode mode : List.of(RunMode.REAL, RunMode.SIM)) {
      List<LogDataReceiver> receivers = Robot.logReceivers(mode, logs);
      assertEquals(
          List.of(WPILOGWriter.class, DashboardStream.class),
          receivers.stream().map(LogDataReceiver::getClass).toList(),
          mode + ": the log's file, and the guarded stream");
    }
  }

  @Test
  void aDebugDeployIsKnownByItsDebuggerArgument() {
    assertTrue(
        Robot.startedForDebugger(
            List.of(
                "-Xmx100M", "-agentlib:jdwp=transport=dt_socket,address=0.0.0.0:8349,server=y")));
    assertFalse(Robot.startedForDebugger(List.of("-Xmx100M", "-XX:+UseSerialGC")));
  }
}

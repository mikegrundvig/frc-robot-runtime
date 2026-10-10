package com.michaelgrundvig.frc.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.sim.Scenarios;
import com.michaelgrundvig.frc.runtime.sim.SimulatedWorld;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
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
    assertEquals(Robot.SimStart.DISABLED, Robot.SimStart.named(null));
    assertEquals(Robot.SimStart.DISABLED, Robot.SimStart.named("sideways"));
    assertEquals(Robot.SimStart.READY, Robot.SimStart.named("ready"));
    assertEquals(Robot.SimStart.ENABLED, Robot.SimStart.named("ENABLED"));
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
    assertTrue(Robot.carriesOn(RunMode.REAL), "on a robot");
    assertTrue(Robot.carriesOn(RunMode.REPLAY), "replaying one");
    assertFalse(Robot.carriesOn(RunMode.SIM), "in the simulator");
  }

  /** Disabled with no station, as a robot starts: not the tests' own default, blue 2. */
  private static void asARealRobotStarts() {
    DriverStationSim.setAllianceStationId(AllianceStationID.UNKNOWN);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void theSimulatorStartsInTheScenarioAskedFor() {
    assertSame(Scenarios.named("collision"), Robot.scenarioAskedFor(RunMode.SIM, "collision"));
    assertNull(Robot.scenarioAskedFor(RunMode.SIM, null));
  }

  @Test
  void aReplayRunsWhatItsLogRecordedNeverAScenario() {
    assertNull(Robot.scenarioAskedFor(RunMode.REPLAY, "collision"));
    assertNull(Robot.scenarioAskedFor(RunMode.REAL, "collision"));
  }

  @Test
  void aMisspelledScenarioStopsTheProgramWithTheList() {
    IllegalArgumentException refused =
        assertThrows(
            IllegalArgumentException.class, () -> Robot.scenarioAskedFor(RunMode.SIM, "colision"));

    String message = String.valueOf(refused.getMessage());
    assertTrue(message.contains("camera-low, camera-turned"), message);
  }

  /** As ./gradlew run starts it: on the first loop the robot is enabled, then never again. */
  @Test
  void aScenarioStartsOnceTheRobotIsEnabled() {
    try (SimulatedWorld world =
        new SimulatedWorld(
            RobotSim.FIELD, RobotSim.WITHOUT_CAMERAS.definition(), SensorErrors.NONE)) {
      Components components = new Components(said -> {});
      world.addTo(components);
      List<Setting> collision = Scenarios.named("collision").eventsIn(world);
      Pose2d start = world.truePose();

      assertSame(
          collision, Robot.startWhenEnabled(collision, components, false), "disabled: waits");
      assertEquals(start, world.truePose());
      assertSame(collision, Robot.startWhenEnabled(collision, null, true), "no simulator: waits");
      assertNull(Robot.startWhenEnabled(collision, components, true), "enabled: started");
      assertEquals(0.6, world.truePose().getY() - start.getY(), 1e-9);
      assertNull(Robot.startWhenEnabled(null, components, true), "nothing more to start");
    }
  }

  /** Asked for by name, a scenario stops a program that has nothing to apply it through. */
  @Test
  void aScenarioWithoutTheSimulatorsControlsStopsTheProgram() {
    IllegalStateException refused =
        assertThrows(
            IllegalStateException.class,
            () -> Robot.startScenario(Scenarios.named("collision"), null, null));

    assertTrue(
        String.valueOf(refused.getMessage()).contains("needs the simulated world"),
        refused.getMessage());
    assertNull(Robot.startScenario(null, null, null), "none asked for: nothing to do");
  }

  /**
   * Started as ./gradlew run starts it: its sensor faults now, on a season-worn robot, even where
   * one leaves a part as it was built; what happens in it, later.
   */
  @Test
  void aScenarioStartsWithItsSensorFaults() {
    try (SimulatedWorld world =
        new SimulatedWorld(
            RobotSim.FIELD, RobotSim.WITHOUT_CAMERAS.definition(), SensorErrors.TYPICAL)) {
      Components components = new Components(said -> {});
      world.addTo(components);

      List<Setting> events =
          Robot.startScenario(Scenarios.named("exact-sensors"), world, components);

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

  @Test
  void aDebugDeployIsKnownByItsDebuggerArgument() {
    assertTrue(
        Robot.startedForDebugger(
            List.of(
                "-Xmx100M", "-agentlib:jdwp=transport=dt_socket,address=0.0.0.0:8349,server=y")));
    assertFalse(Robot.startedForDebugger(List.of("-Xmx100M", "-XX:+UseSerialGC")));
  }
}

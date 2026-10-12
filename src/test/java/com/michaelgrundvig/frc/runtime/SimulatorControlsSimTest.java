package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.localization.YawSource;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.system.RobotController;

/**
 * The simulator's controls on the template's robot, through Pitwall's page the way the Sim view
 * drives them, in WPILib's own simulation: the Driver Station, driving from the page's keyboard or
 * gamepad, the field lock the panel itself is exempt from, faults the robot's code has to find out
 * about, and the clock.
 */
@SimulationTest
class SimulatorControlsSimTest {
  private static final String JSON = "application/json";

  @RegisterExtension final RobotSim sim = new RobotSim();

  private final HttpClient client = HttpClient.newHttpClient();

  /**
   * The page enables the robot in utility mode: the robot program sees it from the next loop on, as
   * it sees a real Driver Station's change, all of it at once.
   */
  @Test
  void thePageEnablesTheRobotAndPicksItsStation() throws Exception {
    sim.robot();
    RobotSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();

    var answer =
        request(
            "PATCH",
            "/api/simulator",
            "{\"allianceStation\": \"RED_1\", \"mode\": \"UTILITY\", \"enabled\": true}");
    // Loop by loop until the loop that applies it (the Driver Station's own data is enabled), not
    // until the answer arrives: under load it can arrive a loop or more later.
    loopUntil(DriverStationSim::getEnabled);
    boolean sameLoop = RobotState.isEnabled();
    sim.loop();

    assertThat(answer.get(10, TimeUnit.SECONDS).statusCode()).isEqualTo(200);
    assertThat(sameLoop).as("not partway through the loop that applied it").isFalse();
    assertThat(RobotState.isEnabled()).isTrue();
    assertThat(RobotState.getRobotMode()).isEqualTo(RobotMode.UTILITY);
    assertThat(RobotState.isDSAttached()).isTrue();
    assertThat(MatchState.getLocation()).hasValue(1);
    assertThat(sim.robot().matchField().allianceStation()).isEqualTo("Red 1");
  }

  /**
   * In the simulator the panel is the field: attaching the field system locks tuning as a real
   * field does, while the panel stays usable to detach it again.
   */
  @Test
  void theFieldLocksTuningButNotThePanel() throws Exception {
    sim.robot();

    assertThat(send("PATCH", "/api/simulator", "{\"fmsAttached\": true}").statusCode())
        .isEqualTo(200);
    sim.loop(); // seen as the next loop begins
    var tuned = send("PATCH", "/api/values", "{\"/Drive/config/gains/steerKp\": 18}");
    var detached = send("PATCH", "/api/simulator", "{\"fmsAttached\": false}");
    sim.loop();

    assertThat(tuned.statusCode()).isEqualTo(409);
    assertThat(detached.statusCode()).isEqualTo(200);
    assertThat(RobotState.isFMSAttached()).isFalse();
  }

  /**
   * The page's input drives the robot, through the same joystick the drive team's controller is;
   * when the page goes quiet, the deadman centers it and the robot stops.
   */
  @Test
  void thePageDrivesTheRobotAndItsDeadmanStopsIt() throws Exception {
    Pose2d start = placedAndEnabled();

    // Left stick pushed forward, as the page's W key or a gamepad sends it: up is negative.
    String forward = "{\"0\": {\"axes\": [0, -0.5, 0, 0, 0, 0]}}";
    for (int i = 0; i < 10; i++) { // 1 s, repeating every 100 ms as the page does
      assertThat(send("PUT", "/api/simulator/joysticks", forward).statusCode()).isEqualTo(204);
      sim.loops(5);
    }
    double driven = travelled(start);
    sim.run(1.5); // quiet: centered after 500 ms, and stopped
    Pose2d stopped = sim.simulation().truePose();
    sim.run(0.5);

    assertThat(driven).as("driven forward").isGreaterThan(0.3);
    assertThat(sim.simulation().truePose().getTranslation().getDistance(stopped.getTranslation()))
        .as("stopped")
        .isLessThan(0.01);
  }

  /**
   * The page's input drives nothing while the robot is disabled: the stick reaches the robot's
   * program, which asks the wheels to drive, and the drivetrain refuses. Enabled with the stick
   * still pushed, the robot drives.
   */
  @Test
  void thePagesDrivingMovesNothingWhileTheRobotIsDisabled() throws Exception {
    Pose2d start = placedAndEnabled();
    RobotSim.setEnabled(false);
    sim.run(0.2);

    String forward = "{\"0\": {\"axes\": [0, -0.5, 0, 0, 0, 0]}}";
    for (int i = 0; i < 10; i++) { // 1 s, repeating every 100 ms as the page does
      assertThat(send("PUT", "/api/simulator/joysticks", forward).statusCode()).isEqualTo(204);
      sim.loops(5);
    }
    double stickAsRead = sim.robot().driver().getLeftY();
    double drivenDisabled = travelled(start);
    RobotSim.setEnabled(true);
    for (int i = 0; i < 10; i++) {
      assertThat(send("PUT", "/api/simulator/joysticks", forward).statusCode()).isEqualTo(204);
      sim.loops(5);
    }

    assertThat(stickAsRead).as("the robot's program reads the stick pushed").isLessThan(-0.4);
    assertThat(drivenDisabled).as("meters driven while disabled").isLessThan(0.001);
    assertThat(travelled(start)).as("meters driven once enabled").isGreaterThan(0.3);
  }

  /** A fault from the page: the robot's code finds out from its sensors, and back again. */
  @Test
  void aFaultFromThePageIsFoundByTheRobotsCode() throws Exception {
    placedAndEnabled();

    var unplugged =
        send(
            "POST",
            "/api/simulator/faults",
            "{\"component\": \""
                + FaultCatalog.GYRO
                + "\", \"fault\": \""
                + FaultCatalog.UNPLUGGED
                + "\"}");
    sim.run(0.5);
    YawSource without = sim.robot().localization().yawSource().orElseThrow();
    var pluggedIn =
        send(
            "POST",
            "/api/simulator/faults",
            "{\"component\": \"Gyro\", \"fault\": \"Unplugged\", \"undo\": true}");
    sim.run(1.0);

    assertThat(unplugged.statusCode()).isEqualTo(200);
    assertThat(without).isEqualTo(YawSource.BACKUP);
    assertThat(pluggedIn.statusCode()).isEqualTo(200);
    assertThat(sim.robot().localization().yawSource()).contains(YawSource.GYRO);
  }

  /** A scenario from the page happens now, to the running robot. */
  @Test
  void aScenarioFromThePageHappensToTheRunningRobot() throws Exception {
    Pose2d start = placedAndEnabled();

    var collision = send("POST", "/api/simulator/scenarios", "{\"apply\": \"collision\"}");

    assertThat(collision.statusCode()).isEqualTo(200);
    assertThat(travelled(start)).isGreaterThan(0.5);
  }

  /** With the clock paused, a step moves it on to the next loop's time, and no further. */
  @Test
  void aStepMovesThePausedClockOnOneLoop() {
    Simulator simulator = sim.pitwall().simulator().orElseThrow();
    long before = RobotController.getTime();

    CompletableFuture<?> stepped = simulator.step(); // tests run with the clock paused
    long after = RobotController.getTime();
    sim.loop();

    assertThat(after - before).isPositive().isLessThanOrEqualTo(20_000_000L);
    assertThat(stepped).isDone();
  }

  /** On its start, enabled in teleop, cameras settled. */
  private Pose2d placedAndEnabled() {
    sim.robot();
    RobotSim.setEnabled(true);
    sim.run(1.0);
    return sim.simulation().truePose();
  }

  private double travelled(Pose2d from) {
    return sim.simulation().truePose().getTranslation().getDistance(from.getTranslation());
  }

  /**
   * Sends the page's request and runs the robot loop until it's answered, as the robot does: the
   * server waits for the loop.
   */
  private HttpResponse<String> send(String method, String path, @Nullable String json)
      throws Exception {
    var answer = request(method, path, json);
    loopUntil(answer::isDone);
    return answer.get();
  }

  /** Sends the page's request, without running the loop that answers it. */
  private CompletableFuture<HttpResponse<String>> request(
      String method, String path, @Nullable String json) {
    var request =
        HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + sim.pitwall().port() + path));
    if (json != null) {
      request.header("Content-Type", JSON);
    }
    request.header("Pitwall-Stream", "1"); // as the page names its event stream
    return client.sendAsync(
        request
            .method(
                method,
                json == null
                    ? HttpRequest.BodyPublishers.noBody()
                    : HttpRequest.BodyPublishers.ofString(json))
            .build(),
        HttpResponse.BodyHandlers.ofString());
  }

  /**
   * Runs the robot loop, a loop at a time, until {@code done}: what the network brings arrives in
   * its own time, so it gives up after 10 s.
   */
  private void loopUntil(BooleanSupplier done) {
    long end = System.nanoTime() + Duration.ofSeconds(10).toNanos();
    while (!done.getAsBoolean() && System.nanoTime() < end) {
      sim.loop();
    }
  }
}

package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.Outcome;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.Status;
import com.michaelgrundvig.frc.pitwall.TunableTee;
import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Profiles;
import com.michaelgrundvig.frc.pitwall.experiment.Rig;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.experiment.StepResponse;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import io.avaje.jsonb.Jsonb;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HALUtil;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.AlertDataJNI;

/** Pitwall in the simulated robot program: its standard status, and actions that run commands. */
@ExtendWith(RobotSim.class)
class PitwallRobotTest {
  private Pitwall pitwall;
  private LiveConfig<DrivetrainControl> drivetrain;

  @BeforeEach
  void setUp() {
    pitwall = Pitwall.start(ExampleRobot.SIM.name(), 0);
    // The drivetrain's tuning, published as the robot program publishes it.
    drivetrain = RobotContainer.tunings(ExampleRobot.SIM).drivetrain();
    TunableRegistry.publish("Drive/config", drivetrain);
  }

  @AfterEach
  void tearDown() {
    pitwall.close();
    Scheduler.getDefault().cancelAll();
  }

  @Test
  void theStatusShowsTheBatteryAndTheBarItsNameAndMode() throws Exception {
    pitwall.status().requestRead(); // as the page's GET does: nothing is read unless asked
    pitwall.periodic();

    // Which code is running comes from the team's generated BuildConstants, which the runtime's
    // tests don't have: the team's tests check it (PitwallCodeTest).
    assertThat(pitwall.status().view())
        .extracting(Status.Item::name)
        .containsExactly("Robot/Battery");
    assertThat(value("Robot/Battery")).matches("\\d+\\.\\d V");
    assertThat(robotSection())
        .containsEntry("name", "sim")
        .containsEntry("mode", "Teleop, disabled");
    assertThat(pitwall.port()).as("the page is served").isPositive();
  }

  @Test
  void everyWpilibTunableGoesThroughPitwallAndTheConfigurationIsOnNetworkTablesWithItsLimits() {
    TunableDouble gain = TunableDouble.create(1.0);
    TunableRegistry.publish("Shooter/gain", gain);
    TunableRegistry.update();

    assertThat(pitwall.tunables().view())
        .extracting(TunableTee.Listed::path)
        .containsExactly("/Shooter/gain");
    try (var steerKp =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Tunables/Drive/config/gains/steerKp")
            .subscribe(-1)) {
      assertThat(steerKp.get()).as("dashboards see the configuration").isEqualTo(16.0);
      assertThat(steerKp.getTopic().getProperties()).as("and its limits").contains("\"max\":100");
    }
  }

  @Test
  void theModeSaysWhetherTheRobotIsEnabledAndOnTheField() throws Exception {
    assertThat(modeAfterALoop()).isEqualTo("Teleop, disabled");

    RobotSim.setEnabled(true);
    assertThat(modeAfterALoop()).isEqualTo("Teleop, enabled");

    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();
    assertThat(modeAfterALoop()).isEqualTo("Autonomous, enabled, on the field");

    DriverStationSim.setFmsAttached(false);
    setMode(RobotMode.UTILITY);
    assertThat(modeAfterALoop()).isEqualTo("Utility, enabled");

    setMode(RobotMode.UNKNOWN);
    assertThat(modeAfterALoop()).startsWith("No mode");
  }

  @Test
  void anActionRunsItsCommandOnlyInUtilityModeAndShowsWhenItEnds() {
    pitwall
        .actions()
        .add(
            "Wait a moment",
            () -> Command.noRequirements(co -> co.wait(Seconds.of(0.1))).named("Wait"));

    var locked = pitwall.actions().requestStart("Wait a moment");
    loops(1);
    assertThat(locked.getNow(null).result()).as("teleop").isEqualTo(Outcome.Result.LOCKED);

    RobotSim.setEnabled(true);
    setMode(RobotMode.UTILITY);
    var started = pitwall.actions().requestStart("Wait a moment");
    loops(1);
    assertThat(started.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    assertThat(phase()).isEqualTo(Actions.Phase.RUNNING);

    loops(10);
    assertThat(phase()).as("its 0.1 s wait is over").isEqualTo(Actions.Phase.ENDED);
  }

  /**
   * On a robot, Pitwall has no simulator controls: nothing answers at their routes, so nothing can
   * write the Driver Station's state, and the page's event stream has none of their sections. The
   * Driver Station is the robot's safety switch.
   */
  @Test
  void aRealRobotHasNoSimulatorControls() throws Exception {
    pitwall.close();
    SimHooks.setHALRuntimeType(HALUtil.RUNTIME_SYSTEMCORE);
    try {
      pitwall = Pitwall.start(ExampleRobot.SIM.name(), 0);

      var enable = patch("/api/simulator", "{\"enabled\": true}");
      for (int i = 0; i < 5; i++) {
        pitwall.periodic(); // loops that would apply it, if anything had queued it
        DriverStationSim.notifyNewData();
      }
      String snapshot = snapshot();

      assertThat(pitwall.simulator()).isEmpty();
      assertThat(enable.statusCode()).isEqualTo(404);
      assertThat(DriverStationSim.getEnabled()).isFalse();
      assertThat(snapshot)
          .contains("\"robot\":{")
          .doesNotContain("\"simulator\":{")
          .doesNotContain("\"components\":{")
          .doesNotContain("\"scenarios\":{");
    } finally {
      SimHooks.setHALRuntimeType(HALUtil.RUNTIME_SIMULATION);
    }
  }

  /** In the simulator, the same page has them. */
  @Test
  void theSimulatorHasSimulatorControls() throws Exception {
    assertThat(pitwall.simulator()).isPresent();
    assertThat(snapshot()).contains("\"simulator\":{").contains("\"components\":{");
  }

  /** The page's event stream's first event, its snapshot, as the robot loop sends it. */
  private String snapshot() throws Exception {
    try (HttpClient client = HttpClient.newHttpClient()) {
      var opened =
          client.sendAsync(
              HttpRequest.newBuilder(
                      URI.create("http://127.0.0.1:" + pitwall.port() + "/api/events"))
                  .build(),
              HttpResponse.BodyHandlers.ofLines());
      List<String> lines = new CopyOnWriteArrayList<>();
      Thread reader =
          Thread.ofVirtual()
              .start(
                  () -> {
                    try {
                      opened.get().body().forEach(lines::add);
                    } catch (Exception closed) {
                      // The test is over.
                    }
                  });
      long end = System.nanoTime() + TimeUnit.SECONDS.toNanos(10);
      while (lines.stream().noneMatch(line -> line.startsWith("data: "))
          && System.nanoTime() < end) {
        pitwall.periodic();
        Thread.sleep(5); // waiting on the network, not the robot's clock
      }
      reader.interrupt();
      return lines.stream().filter(line -> line.startsWith("data: ")).findFirst().orElseThrow();
    }
  }

  /**
   * The field lock on the running robot program: with the field system connected, no tunable
   * changes, whether the page or a dashboard asks. Pitwall's headline safety promise.
   */
  @Test
  void theFieldLocksEveryTunableFromThePageAndFromDashboards() throws Exception {
    TunableDouble gain = TunableDouble.create(1.0);
    TunableRegistry.publish("Shooter/gain", gain);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();
    pitwall.periodic();

    var page = patch("/api/wpilib-tunables", "{\"Shooter/gain\": 2.0}");
    var config = patch("/api/values", "{\"/Drive/config/gains/steerKp\": 18}");
    try (var dashboard =
        NetworkTableInstance.getDefault().getDoubleTopic("/Tunables/Shooter/gain").publish()) {
      dashboard.set(3.0);
      TunableRegistry.update();

      assertThat(page.statusCode()).isEqualTo(409);
      assertThat(page.body()).contains("the field system is connected");
      assertThat(config.statusCode()).isEqualTo(409);
      assertThat(gain.get()).as("neither the page nor a dashboard").isEqualTo(1.0);
      assertThat(drivetrain.get().gains().steerKp()).isEqualTo(16.0);

      // The same dashboard write, off the field with tuning switched back on (the field switched it
      // off): it gets through, so the lock is what held it.
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.notifyNewData();
      assertThat(patch("/api/tuning", "{\"enabled\": true}").statusCode()).isEqualTo(200);
      dashboard.set(4.0);
      TunableRegistry.update();
      assertThat(gain.get()).isEqualTo(4.0);
    }
  }

  @Test
  void tuningSwitchedOffLocksEveryTunable() throws Exception {
    TunableDouble gain = TunableDouble.create(1.0);
    TunableRegistry.publish("Shooter/gain", gain);

    var off = patch("/api/tuning", "{\"enabled\": false}");
    var page = patch("/api/wpilib-tunables", "{\"Shooter/gain\": 2.0}");

    assertThat(off.body()).contains("\"enabled\":false");
    assertThat(page.statusCode()).isEqualTo(409);
    assertThat(page.body()).contains("tuning is off");
    assertThat(gain.get()).isEqualTo(1.0);
  }

  /**
   * When the robot disables, what tuning changed is printed as the builder calls that commit it.
   */
  @Test
  void disablingPrintsWhatTuningChanged() throws Exception {
    RobotSim.setEnabled(true);
    pitwall.periodic();
    patch("/api/values", "{\"/Drive/config/gains/steerKp\": 18}");
    PrintStream console = System.out;
    ByteArrayOutputStream printed = new ByteArrayOutputStream();
    System.setOut(new PrintStream(printed, true, StandardCharsets.UTF_8));
    try {
      RobotSim.setEnabled(false);
      pitwall.periodic();
    } finally {
      System.setOut(console);
    }

    assertThat(printed.toString(StandardCharsets.UTF_8))
        .contains("gains.steerKp: 16.0 -> 18.0")
        .contains(".steerKp(18.0)");
  }

  /**
   * When the field connects, a match runs the committed configuration: tuned gains go back (printed
   * first, so they can be committed), while a switch left changed (a camera off) keeps its setting
   * and raises an alert naming it.
   */
  @Test
  void theFieldConnectingRevertsTunedValuesAndNamesChangedSwitches() throws Exception {
    TunableBoolean camera = TunableBoolean.create(true);
    TunableRegistry.publish("Vision/front/enabled", camera);
    patch("/api/values", "{\"/Drive/config/gains/steerKp\": 18}");
    patch("/api/wpilib-tunables", "{\"Vision/front/enabled\": false}");
    PrintStream console = System.out;
    ByteArrayOutputStream printed = new ByteArrayOutputStream();
    System.setOut(new PrintStream(printed, true, StandardCharsets.UTF_8));
    try {
      DriverStationSim.setFmsAttached(true);
      DriverStationSim.notifyNewData();
      pitwall.periodic();
    } finally {
      System.setOut(console);
    }

    assertThat(drivetrain.get().gains().steerKp()).as("reverted").isEqualTo(16.0);
    assertThat(printed.toString(StandardCharsets.UTF_8))
        .contains("tuned values return to the committed configuration")
        .contains(".steerKp(18.0)");
    assertThat(camera.get()).as("the switch keeps its setting").isFalse();
    assertThat(activeAlerts())
        .anySatisfy(
            text ->
                assertThat(text)
                    .isEqualTo(
                        "On the field with settings changed since startup:"
                            + " /Vision/front/enabled: false (was true)"));
  }

  /**
   * An experiment runs as a Commands v3 command on its rig's own scheduler, through the robot loop,
   * and is scored. The mechanism is a first-order lag whose time constant is 1/steerKp, so its rise
   * is known: ln 9 / 16 s.
   */
  @Test
  void anExperimentRunsAsACommandOnItsRigsSchedulerAndIsScored() {
    Lag lag = new Lag();
    RobotSim.enable(RobotMode.UTILITY, true);
    var started = pitwall.experiments().requestStart("Step", null);

    run(lag, 110);

    assertThat(started.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    Run run = Objects.requireNonNull(pitwall.experiments().run());
    assertThat(run.state().phase()).isEqualTo(Run.Phase.ENDED);
    assertThat(run.gains()).containsEntry("gains.steerKp", 16.0);
    var steps = Objects.requireNonNull(run.state().scores()).steps();
    StepResponse up = Objects.requireNonNull(steps.get("Angle")).get(0).orElseThrow();
    assertThat(up.riseSeconds().getAsDouble()).isCloseTo(Math.log(9) / 16, within(0.003));
    assertThat(up.overshoot()).isZero();
  }

  /** Disabling cancels the command, and its cleanup ends the run, saying why. */
  @Test
  void disablingCancelsAnExperimentsCommand() {
    Lag lag = new Lag();
    RobotSim.enable(RobotMode.UTILITY, true);
    pitwall.experiments().requestStart("Step", null);
    run(lag, 20);

    RobotSim.enable(RobotMode.UTILITY, false);
    run(lag, 1);

    Run run = Objects.requireNonNull(pitwall.experiments().run());
    assertThat(run.state().phase()).isEqualTo(Run.Phase.CANCELED);
    assertThat(run.state().detail())
        .isEqualTo("Canceled: experiments run only with the robot enabled in utility mode");
    assertThat(lag.scheduler.getRunningCommands()).isEmpty();
  }

  /** A mechanism on its own scheduler that closes on its setpoint as a first-order lag. */
  private final class Lag implements Mechanism {
    final Scheduler scheduler = Scheduler.createIndependentScheduler();
    double setpoint;
    double angle;

    Lag() {
      pitwall
          .experiments()
          .add(
              new Experiment(
                  "Step",
                  Rig.builder()
                      .name("Lag")
                      .gains(drivetrain, "gains.steerKp")
                      .setpoint(Radians, radians -> setpoint = radians)
                      .limits(Degrees.of(-90), Degrees.of(90))
                      .measurement("Angle", Degrees, () -> Math.toDegrees(angle))
                      .requires(this)
                      .build(),
                  Profiles.stepAndBack()
                      .from(Degrees.of(0))
                      .to(Degrees.of(45))
                      .hold(Seconds.of(0.8))));
    }

    @Override
    public Scheduler getRegisteredScheduler() {
      return scheduler;
    }

    /** Moves one loop toward the setpoint, exactly as a lag does. */
    void step() {
      double kP = drivetrain.get().gains().steerKp();
      angle = setpoint + (angle - setpoint) * Math.exp(-kP * 0.02);
    }
  }

  /** Robot loops with an experiment's page in touch: Pitwall, the mechanism, then its commands. */
  private void run(Lag lag, int loops) {
    for (int i = 0; i < loops; i++) {
      RobotSim.stepClock();
      pitwall.experiments().heard(null);
      pitwall.periodic();
      lag.step();
      lag.scheduler.run();
    }
  }

  private static List<String> activeAlerts() {
    List<String> active = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.activeStartTime != 0) {
        active.add(alert.text);
      }
    }
    return active;
  }

  /**
   * Sends the page's request and runs the robot loop until it's answered, as the robot does: the
   * server waits for the loop to apply a change.
   */
  private HttpResponse<String> patch(String path, String json) throws Exception {
    try (HttpClient client = HttpClient.newHttpClient()) {
      var answer =
          client.sendAsync(
              HttpRequest.newBuilder(URI.create("http://localhost:" + pitwall.port() + path))
                  .header("Content-Type", "application/merge-patch+json")
                  .method("PATCH", HttpRequest.BodyPublishers.ofString(json))
                  .build(),
              HttpResponse.BodyHandlers.ofString());
      for (int i = 0; i < 400 && !answer.isDone(); i++) {
        pitwall.periodic();
        TunableRegistry.update();
        Thread.sleep(5);
      }
      return answer.get(1, TimeUnit.SECONDS);
    }
  }

  private String value(String name) {
    return pitwall.status().view().stream()
        .filter(item -> item.name().equals(name))
        .findFirst()
        .orElseThrow()
        .value()
        .toString();
  }

  /** The mode as the page's bar shows it: the snapshot's robot section, a loop after a change. */
  private String modeAfterALoop() throws Exception {
    pitwall.periodic();
    return (String) Objects.requireNonNull(robotSection().get("mode"));
  }

  /** The snapshot's robot section: its name, code, and mode, as the page's bar shows them. */
  @SuppressWarnings("unchecked")
  private Map<String, Object> robotSection() throws Exception {
    Map<String, Object> snapshot =
        (Map<String, Object>)
            Jsonb.instance().type(Object.class).fromJson(snapshot().substring("data: ".length()));
    Map<String, Object> document =
        (Map<String, Object>) Objects.requireNonNull(snapshot.get("document"));
    return (Map<String, Object>) Objects.requireNonNull(document.get("robot"));
  }

  private Actions.Phase phase() {
    return pitwall.actions().view().get(0).phase();
  }

  private void loops(int count) {
    for (int i = 0; i < count; i++) {
      RobotSim.stepClock();
      pitwall.periodic();
      Scheduler.getDefault().run();
    }
  }

  private static void setMode(RobotMode mode) {
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.notifyNewData();
  }
}

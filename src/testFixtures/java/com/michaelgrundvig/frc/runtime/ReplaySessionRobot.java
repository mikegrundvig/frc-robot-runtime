package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.simulator.JoystickInput;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Joystick;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;

/**
 * The template's robot program with a scripted session on top, for {@code ReplayEndToEndTest} and
 * the team's layout checks ({@link SessionRecording}). In the simulator it changes things the way
 * people do in the pit, through Pitwall's page: it picks the red alliance's station 1 (which puts
 * the robot on that start) and enables utility mode from the simulator's controls, pushes the
 * operator's stick from the page, changes a configuration gain (over HTTP), changes a WPILib
 * tunable, runs an action that drives the robot under a trench, and runs the steering's experiment,
 * keeping it going as the page would. It records what those affect, then stops. Replayed, it only
 * records, and the replay must match.
 *
 * <p>The session runs on a clock of its own ({@link SessionClock}), which moves a loop at a time,
 * so it is the same session on any computer, however busy: two recordings of it hold the same log.
 * Everything the session does, it does in a loop, before the robot's code, and what it sends over
 * the network it waits for there, until the request has arrived for that loop to apply.
 */
final class ReplaySessionRobot extends Robot {
  /** How many loops the recorded session runs, about five and a half seconds. */
  static final int LOOPS = 280;

  static final String GAIN = "Session/gain";

  private final TunableDouble gain = TunableDouble.create(1.0);
  private final HttpClient http = HttpClient.newHttpClient();

  /** The operator's joystick, which the page drives for a moment. */
  private final Joystick operator = new Joystick(1);

  /** The clock the recorded session runs on. A replay has none: its log says the time. */
  private final Optional<SessionClock> clock;

  private int loop;
  private int utilityInits;

  /**
   * @param clock the session's clock, stopped before this program was made; none in a replay
   */
  ReplaySessionRobot(Optional<SessionClock> clock) {
    super(
        new RobotProgram(
            () -> ExampleRobot.COMPETITION, Rebuilt.GAME, ControlsMaker.NONE, BuildInfo.UNKNOWN));
    this.clock = clock;
    if (clock.isPresent()) {
      // A loop follows the last at once: the session's clock says the time, not the computer's.
      setUseTiming(false);
    }
    TunableRegistry.publish(GAIN, gain);
  }

  /** Runs the session robot: {@code java com.michaelgrundvig.frc.runtime.ReplaySessionRobot}. */
  public static void main(String... args) {
    SessionRecording.start(ReplaySessionRobot::new);
  }

  @Override
  public void robotPeriodic() {
    // The script runs before the robot's code, the way Driver Station data arrives between loops:
    // AdvantageKit records it at the end of each loop, so a change made after the robot's code had
    // read it would be logged a loop before the robot saw it, and the replay would run a loop
    // early.
    if (mode() == RunMode.SIM) {
      script(++loop);
      pitwall().experiments().heard(null); // the page keeping its experiment going
    }
    super.robotPeriodic();
    Logger.recordOutput("Session/Gain", gain.get());
    Logger.recordOutput(
        "Session/Operator", operator.getHID().getAxesAvailable() > 0 ? operator.getRawAxis(0) : 0);
    Logger.recordOutput("Session/SteerKp", container().drivetrainTuning().get().gains().steerKp());
    Logger.recordOutput("Session/Action", pitwall().actions().view().get(0).phase().name());
    Run run = pitwall().experiments().run();
    Logger.recordOutput("Session/Experiment", run == null ? "NONE" : run.state().phase().name());
    Run.@Nullable Scores scores = run == null ? null : run.state().scores();
    Logger.recordOutput(
        "Session/ExperimentRise",
        scores == null
            ? 0.0
            : Objects.requireNonNull(scores.steps().get("Front left"))
                .get(0)
                .orElseThrow()
                .riseSeconds()
                .orElseThrow());
  }

  /**
   * WPILib calls this on the first loop it sees the robot enabled in utility mode, from the Driver
   * Station's data as that loop began: the replay must call it on the same loop.
   */
  @Override
  public void utilityInit() {
    Logger.recordOutput("Session/UtilityInits", (double) ++utilityInits);
  }

  /**
   * How many loops the session has run, which AdvantageKit logs after every loop, and with it the
   * moment the session's clock moves ({@link SessionClock#loopIsOver}). AdvantageKit reads an
   * output like this one when a loop is over and before it gives the next loop its time, and no
   * other code of a robot program's runs between two loops. Inside a loop the clock must stand
   * still: moved there, WPILib's watchdog would take every loop for a late one, and say so.
   */
  @AutoLogOutput(key = "Session/Loops")
  int loopsRunAndTheClockMoves() {
    clock.ifPresent(SessionClock::loopIsOver);
    return loop;
  }

  private Simulator simulator() {
    return pitwall().simulator().orElseThrow();
  }

  /** The session, by loop: in a replay none of this runs, and the log supplies it instead. */
  private void script(int loop) {
    switch (loop) {
      // The simulator's controls, as the page's Sim view uses them: applied by this loop's Pitwall.
      case 5 ->
          simulator()
              .requestDriverStation(
                  station -> station.withAllianceStation(AllianceStationID.RED_1));
      case 10 ->
          simulator()
              .requestDriverStation(
                  station -> station.withMode(RobotMode.UTILITY).withEnabled(true));
      case 15 ->
          simulator().joystick(1, new JoystickInput(List.of(0.5), List.of(), List.of()), null);
      case 20 -> patchTheSteeringGain();
      case 25 -> simulator().joystick(1, null, null); // let go
      case 30 -> pitwall().tunables().requestChanges(Map.of(GAIN, 2.5));
      case 40 -> pitwall().actions().requestStart(RobotContainer.DRIVE_CHECK);
      case 150 -> pitwall().experiments().requestStart(RobotContainer.STEER_STEP, null);
      case LOOPS -> {
        Logger.end(); // closes the log, once it has written every loop it was handed
        System.exit(0);
      }
      default -> {
        // nothing this loop
      }
    }
  }

  /**
   * Changes a configuration gain as the page does, over HTTP, and waits until the request has
   * arrived. Pitwall's server takes it on a thread of its own and keeps it for the robot's loop,
   * which applies it as this loop ends, when the tunables update. Sent and not waited for, it would
   * arrive whenever the server's thread got to it, a loop later or many on a busy computer, and the
   * gain would change in another loop from one recording to the next.
   */
  private void patchTheSteeringGain() {
    http.sendAsync(
        HttpRequest.newBuilder(URI.create("http://localhost:" + pitwall().port() + "/api/values"))
            .header("Content-Type", "application/merge-patch+json")
            .method(
                "PATCH",
                HttpRequest.BodyPublishers.ofString("{\"/Drive/config/gains/steerKp\": 18.5}"))
            .build(),
        HttpResponse.BodyHandlers.discarding());
    SessionClock.await(
        "the page's change to arrive at Pitwall", () -> pitwall().tunables().waiting() > 0);
  }
}

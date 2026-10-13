package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveMove;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Placement;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.SchedulerEvent;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;

/**
 * The template's own test of a robot's autos, the one to run before any match: every auto its
 * {@code Controls} declares (and "Do nothing"), on each alliance, in the simulator, from its own
 * start, for the whole autonomous period. A fresh robot runs each, chosen and started as the pit
 * and the Driver Station do, so what is checked is what the robot does, not a stand-in for it.
 *
 * <p>Each run fails, saying why, if:
 *
 * <ul>
 *   <li>the auto can't start: the alliance is unknown, or the robot placed on its start doesn't
 *       read as standing on it (the placement check the drive team reads);
 *   <li>its command threw, as it was built or as it ran;
 *   <li>any drive or joint move in it ended {@code REFUSED}: a place the joint hasn't, a path with
 *       no way through its keyframes, the alliance unknown;
 *   <li>the robot left the field.
 * </ul>
 *
 * <p>Whatever the outcome, each run reports where the robot ended and each move's outcome line, so
 * a failure reads without a debugger.
 *
 * <pre>{@code
 * AutoCheck.check(sim, BetaRobot.CONFIG, BetaControls::new);
 * }</pre>
 */
public final class AutoCheck {
  private AutoCheck() {}

  /**
   * The length of the autonomous period, 15 s. The game model (Rebuilt, SeasonLimits) says none, so
   * this is the template's; a game whose autonomous differs sets it where the game's rules are.
   */
  public static final double AUTONOMOUS_SECONDS = 15;

  /**
   * One auto on one alliance.
   *
   * @param auto its name
   * @param alliance the alliance it ran on
   * @param ended where the robot really ended, if it started
   * @param run how the auto ended, if it started
   * @param moves how each drive and joint move ended, in the order they ended: ask each with the
   *     {@link Ended} checks, or print it for its sentence
   * @param problems everything wrong, in words; empty when it passed
   */
  public record Result(
      String auto,
      Alliance alliance,
      Optional<Pose2d> ended,
      Optional<Autos.Run> run,
      List<Ended> moves,
      List<String> problems) {
    /** Whether nothing was wrong. */
    public boolean passed() {
      return problems.isEmpty();
    }

    /** The auto, the alliance, where it ended, how, each move, and what was wrong. */
    public String describe() {
      StringBuilder text = new StringBuilder(auto + " on " + alliance + ": ");
      text.append(
          ended
              .map(
                  pose ->
                      String.format(
                          Locale.ROOT,
                          "ended at (%.2f, %.2f) %.1f°",
                          pose.getX(),
                          pose.getY(),
                          pose.getRotation().getDegrees()))
              .orElse("did not start"));
      run.ifPresent(each -> text.append(", ").append(each.describe()));
      for (Ended move : moves) {
        text.append("\n    ").append(move);
      }
      for (String problem : problems) {
        text.append("\n    PROBLEM: ").append(problem);
      }
      return text.toString();
    }
  }

  /**
   * Runs every auto the controls declare on each alliance, and fails, with every run reported, if
   * any has a problem.
   *
   * @param robot the robot to build; its controls declare the autos
   * @param controls the team's {@code Controls}' constructor, as {@link RobotSim#controls} takes it
   * @return every run, passed or not
   * @throws AssertionError listing each run that failed
   */
  public static List<Result> check(RobotSim sim, RobotConfig robot, ControlsMaker<?> controls) {
    List<Result> results = run(sim, robot, controls);
    List<Result> failed = results.stream().filter(result -> !result.passed()).toList();
    if (!failed.isEmpty()) {
      StringBuilder message =
          new StringBuilder(failed.size() + " of " + results.size() + " autos failed:\n");
      results.forEach(result -> message.append(result.describe()).append('\n'));
      throw new AssertionError(message.toString());
    }
    return results;
  }

  /** Runs every auto the controls declare on each alliance, and reports each run. */
  public static List<Result> run(RobotSim sim, RobotConfig robot, ControlsMaker<?> controls) {
    sim.build(robot, SensorErrors.NONE);
    sim.controls(controls);
    List<String> names = sim.robot().autos().names();
    List<Result> results = new ArrayList<>();
    for (String name : names) {
      for (Alliance alliance : Alliance.values()) {
        results.add(runOne(sim, robot, controls, name, alliance));
      }
    }
    return results;
  }

  private static Result runOne(
      RobotSim sim, RobotConfig robot, ControlsMaker<?> controls, String name, Alliance alliance) {
    RobotSim.setEnabled(false);
    sim.build(robot, SensorErrors.NONE);
    sim.controls(controls);
    List<Ended> moves = new ArrayList<>();
    List<String> problems = new ArrayList<>();
    sim.scheduler().addEventListener(event -> heard(event, moves, problems));
    Autos autos = sim.robot().autos();

    // The drive team's setup: the auto chosen, the alliance known, the robot placed on its start.
    autos.choose(name);
    RobotSim.station(alliance == Alliance.RED ? AllianceStationID.RED_2 : AllianceStationID.BLUE_2);
    sim.loops(5);
    Placement placement = sim.placement();
    if (!autos.chosen().name().equals(name)) {
      problems.add("it can't start: the robot chose " + autos.chosen().name() + " instead");
    } else if (placement.status() != Placement.Status.READY) {
      problems.add("it can't start: " + placement.describe());
    }
    if (!problems.isEmpty()) {
      return new Result(name, alliance, Optional.empty(), Optional.empty(), moves, problems);
    }

    // Autonomous, for its whole period.
    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    try {
      for (int i = 0; i < RobotLoop.loops(AUTONOMOUS_SECONDS); i++) {
        sim.loop();
        Pose2d now = sim.simulation().truePose();
        if (!sim.robot().matchField().field().contains(now.getTranslation())
            && problems.stream().noneMatch(problem -> problem.startsWith("the robot left"))) {
          problems.add(
              String.format(
                  Locale.ROOT,
                  "the robot left the field at %.1f s, at (%.2f, %.2f)",
                  i * RobotLoop.PERIOD_SECONDS,
                  now.getX(),
                  now.getY()));
        }
      }
      RobotSim.enable(RobotMode.AUTONOMOUS, false);
      sim.loops(3);
    } catch (RuntimeException e) {
      problems.add("it threw: " + e);
    }
    Optional<Autos.Run> ran = autos.lastRun();
    ran.filter(each -> each.ending() == Autos.Ending.FAILED)
        .ifPresent(each -> problems.add("it failed: " + each.describe()));
    if (ran.isEmpty()) {
      problems.add("it never ended: no outcome was logged");
    }
    return new Result(
        name, alliance, Optional.of(sim.simulation().truePose()), ran, moves, problems);
  }

  /** Records each drive and joint move as it ends, and a refused one as a problem. */
  private static void heard(SchedulerEvent event, List<Ended> moves, List<String> problems) {
    Command command = null;
    if (event instanceof SchedulerEvent.Completed done) {
      command = done.command();
    } else if (event instanceof SchedulerEvent.Canceled canceled) {
      command = canceled.command();
    }
    if (command instanceof DriveMove move && move.ending().isPresent()) {
      ended(move.outcome(), Ended.ended(move), moves, problems);
    } else if (command instanceof Move move && move.ending().isPresent()) {
      ended(move.outcome(), Ended.ended(move), moves, problems);
    }
  }

  private static void ended(
      Optional<Outcome> outcome, Ended ended, List<Ended> moves, List<String> problems) {
    moves.add(ended);
    if (outcome.isPresent() && outcome.get() == Outcome.REFUSED) {
      problems.add("a move was refused: " + ended);
    }
  }
}

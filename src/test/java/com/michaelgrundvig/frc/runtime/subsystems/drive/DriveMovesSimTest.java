package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.world.path.Keyframe;
import com.michaelgrundvig.frc.world.record.DriveOutcome;
import com.michaelgrundvig.frc.world.record.MoveReason;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The drivetrain's moves, as a joint's are: each a command that ends on its own, with a deadline,
 * an outcome, and a reason. Turning is profiled under what the robot's own model says it can do.
 */
@ExtendWith(RobotSim.class)
class DriveMovesSimTest {
  private final DriveSimHarness harness = new DriveSimHarness(ExampleRobot.SIM);
  private final Drive drive = harness.drive;

  @AfterEach
  void tearDown() {
    harness.close();
  }

  /** Schedules a move and runs loops until it ends, at most {@code seconds}: how long it took. */
  private double runToEnd(DriveMove move, double seconds) {
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(move);
    for (int i = 1; i <= RobotLoop.loops(seconds); i++) {
      harness.loop();
      if (move.outcome().isPresent()) {
        return i * RobotLoop.PERIOD_SECONDS;
      }
    }
    return Double.NaN;
  }

  private double heading() {
    return drive.getPose().getRotation().getDegrees();
  }

  /**
   * A quarter turn, profiled at half the robot's own turning acceleration, each wheel given the
   * volts its share of that takes: it follows its profile and arrives inside 2° and nearly still in
   * about the profile's 0.83 s, and says so.
   */
  @Test
  void aTurnArrivesAtItsHeading() {
    DriveMove left = drive.turnTo(Rotation2d.fromDegrees(90));

    double took = runToEnd(left, 3);

    assertThat(left.arrived()).isTrue();
    ended(left).named("drive → 90.0°").arrived().inBetween(0, 1);
    assertThat(heading()).isCloseTo(90, within(2.0));
    assertThat(took).as("its 0.83 s profile, tracked").isLessThan(0.95);
  }

  /** A turn to -170° from 0° goes the nearer way, clockwise, never through 180° the long way. */
  @Test
  void aTurnGoesTheNearerWay() {
    DriveMove around = drive.turnTo(Rotation2d.fromDegrees(-170));
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(around);
    double highest = 0;
    for (int i = 0; i < RobotLoop.loops(3) && around.outcome().isEmpty(); i++) {
      harness.loop();
      highest = Math.max(highest, heading());
    }

    assertThat(around.arrived()).isTrue();
    assertThat(highest).as("never turned left").isLessThan(1);
  }

  /**
   * To a pose 2 m ahead and 1 m left, turning a quarter turn on the way: it arrives within 2 cm and
   * 2°, and never strays more than a few centimeters from the straight line there.
   */
  @Test
  void itGoesToAPoseInAStraightLine() {
    Pose2d target = new Pose2d(2, 1, Rotation2d.fromDegrees(90));
    DriveMove there = drive.goTo(target);
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(there);
    double strayed = 0;
    int loops = 0;
    for (; loops < RobotLoop.loops(5) && there.outcome().isEmpty(); loops++) {
      harness.loop();
      Translation2d at = harness.simulation.truePose().getTranslation();
      // Distance from the line from the start (0, 0) to (2, 1).
      strayed = Math.max(strayed, Math.abs(at.getX() * 1 - at.getY() * 2) / Math.sqrt(5));
    }

    assertThat(there.arrived()).isTrue();
    ended(there).named("drive → (2.00, 1.00) 90.0°").arrived();
    Pose2d truth = harness.simulation.truePose();
    assertThat(truth.getTranslation().getDistance(target.getTranslation())).isLessThan(0.03);
    assertThat(truth.getRotation().getDegrees()).isCloseTo(90, within(2.5));
    assertThat(strayed).as("off the straight line, at most").isLessThan(0.05);
    assertThat(loops * RobotLoop.PERIOD_SECONDS).as("about its profile's time").isLessThan(2.5);
  }

  /**
   * A half turn, past where the heading wraps from 180° to -180°, arrives, and holds the robot
   * where it stood: it moves less than 2 cm.
   */
  @Test
  void aTurnHoldsItsSpot() {
    DriveMove around = drive.turnTo(Rotation2d.fromDegrees(180));

    RobotSim.setEnabled(true);
    harness.scheduler.schedule(around);
    for (int i = 0; i < RobotLoop.loops(3) && around.outcome().isEmpty(); i++) {
      harness.loop();
    }

    assertThat(around.arrived()).isTrue();
    assertThat(harness.simulation.truePose().getTranslation().getNorm()).isLessThan(0.02);
  }

  /**
   * A move that ends as it starts, its condition holding already, still takes its loop, so a loop
   * of moves goes round once a loop. Until 2026-10-07 it ended without a loop, and such a loop went
   * round without end in one, the robot's loop never finishing.
   */
  @Test
  void aMoveThatEndsAsItStartsStillTakesItsLoop() {
    int[] arrived = {0};
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(
        Command.noRequirements(
                co -> {
                  while (arrived[0] < 100) {
                    arrived[0]++;
                    co.await(drive.runUntil(new ChassisVelocities(1, 0, 0), () -> true));
                  }
                })
            .named("There, again and again"));

    harness.loop();
    int inALoop = arrived[0];
    harness.loop();

    assertThat(inALoop).isEqualTo(1);
    assertThat(arrived[0]).as("again, the next loop").isEqualTo(2);
  }

  /** A move that can't get there in time says where it got to. */
  @Test
  void aMoveThatTimesOutSaysWhereItGot() {
    DriveMove far = drive.goTo(new Pose2d(8, 0, Rotation2d.ZERO)).within(Seconds.of(0.5));

    runToEnd(far, 1);

    assertThat(far.outcome()).contains(Outcome.TIMED_OUT);
    ended(far).named("drive → (8.00, 0.00) 0.0°").timedOut().after(0.50).forNoReason();
    DriveOutcome gotTo = far.ending().orElseThrow();
    assertThat(gotTo.x()).as("part of the first meter").isBetween(0.0, 1.0);
    assertThat(gotTo.y()).isCloseTo(0, within(0.005));
    assertThat(Math.toDegrees(gotTo.headingRadians())).isCloseTo(0, within(0.05));
  }

  /** Driving until a gate opens ends the loop it does, having driven that far. */
  @Test
  void runUntilEndsTheLoopItsConditionHolds() {
    Gate far = Gate.of(harness.scheduler, "a meter out", () -> drive.getPose().getX() >= 1);
    DriveMove out = drive.runUntil(new ChassisVelocities(1.5, 0, 0), far);

    double took = runToEnd(out, 3);

    assertThat(out.name()).isEqualTo("drive until a meter out");
    assertThat(out.arrived()).isTrue();
    assertThat(took).as("speeding up, then 1.5 m/s").isBetween(0.6, 1.2);
    assertThat(drive.getPose().getX()).isCloseTo(1.0, within(0.1));
  }

  /**
   * Driving until a gate opens has no deadline of its own: with a gate that never opens it drives
   * on until it is canceled. Until 2026-10-11 it timed out after 3 s.
   */
  @Test
  void runUntilHasNoDeadlineOfItsOwn() {
    Gate never = Gate.of(harness.scheduler, "bumped", () -> false);
    DriveMove out = drive.runUntil(new ChassisVelocities(0.2, 0, 0), never);

    double took = runToEnd(out, 6);
    harness.scheduler.cancel(out);

    assertThat(took).as("it never ended in 6 s").isNaN();
    assertThat(drive.getPose().getX()).as("still driving at 0.2 m/s").isGreaterThan(1.0);
    ended(out).named("drive until bumped").canceled().after(5.98).forNoReason();
  }

  /** Given a deadline, a gate that never opens times out the move at it, saying why. */
  @Test
  void runUntilTimesOutAtTheDeadlineItIsGivenSayingWhy() {
    Gate never = Gate.of(harness.scheduler, "bumped", () -> false);
    DriveMove brief = drive.runUntil(new ChassisVelocities(0.5, 0, 0), never).within(Seconds.of(1));

    runToEnd(brief, 2);

    assertThat(brief.outcome()).contains(Outcome.TIMED_OUT);
    ended(brief)
        .named("drive until bumped")
        .timedOut()
        .after(1.00)
        .because(MoveReason.CONDITION_NEVER_HELD)
        .saying("bumped is false");
  }

  /** One of each kind of move, by name, made new each time it's asked for. */
  private Map<String, Supplier<DriveMove>> everyKindOfMove() {
    Map<String, Supplier<DriveMove>> kinds = new LinkedHashMap<>();
    kinds.put("drive until", () -> drive.runUntil(new ChassisVelocities(0.5, 0, 0), () -> false));
    kinds.put("drive → 90.0°", () -> drive.turnTo(Rotation2d.fromDegrees(90)));
    kinds.put("drive → (1.00, 0.00) 0.0°", () -> drive.goTo(new Pose2d(1, 0, Rotation2d.ZERO)));
    kinds.put(
        "drive → (1.00, 0.00) → (2.00, 0.00) 0.0°",
        () ->
            drive.follow(
                Keyframe.at(new Pose2d(1, 0, Rotation2d.ZERO)),
                Keyframe.at(new Pose2d(2, 0, Rotation2d.ZERO))));
    return kinds;
  }

  /** Deadlines that aren't a time above zero, each with how a refusal shows it. */
  private static final Map<Double, String> NOT_DEADLINES = notDeadlines();

  private static Map<Double, String> notDeadlines() {
    Map<Double, String> shown = new LinkedHashMap<>();
    shown.put(Double.NaN, "NaN");
    shown.put(Double.POSITIVE_INFINITY, "Infinity");
    shown.put(Double.NEGATIVE_INFINITY, "-Infinity");
    shown.put(0.0, "0");
    shown.put(-1.5, "-1.5");
    return shown;
  }

  /**
   * A deadline that isn't a time above zero is refused as the move is made, the same for every kind
   * of move: made as the robot starts, it throws, so the robot doesn't start. Until 2026-10-11 a
   * path took a deadline that wasn't a number as none given, and a run-until timed out in its first
   * loop.
   */
  @Test
  void aDeadlineThatIsntATimeAboveZeroMadeAsTheRobotStartsThrows() {
    Map<String, String> thrown = new LinkedHashMap<>();
    Map<String, String> expected = new LinkedHashMap<>();

    everyKindOfMove()
        .forEach(
            (name, move) ->
                NOT_DEADLINES.forEach(
                    (seconds, shown) -> {
                      String row = name + " within " + seconds;
                      try {
                        DriveMove made = move.get().within(Seconds.of(seconds));
                        thrown.put(row, "made: " + made);
                      } catch (IllegalArgumentException e) {
                        thrown.put(row, e.getMessage());
                      }
                      expected.put(
                          row,
                          name
                              + ": a deadline is a time above 0 s, not "
                              + shown
                              + " s: give within() how long the move may take");
                    }));

    assertThat(thrown).containsExactlyEntriesOf(expected);
  }

  /** What a refused move is named and says. */
  private record Refused(String name, String why) {}

  /**
   * Made once the robot runs, inside a command, the same deadlines make moves that end REFUSED as
   * they start, saying the same, every kind alike: the robot stays put and the command runs on.
   */
  @Test
  void aDeadlineThatIsntATimeAboveZeroMadeWhileTheRobotRunsIsRefused() {
    Map<String, DriveMove> moves = new LinkedHashMap<>();
    Map<String, Refused> expected = new LinkedHashMap<>();
    boolean[] ended = {false};
    Command wrong =
        Command.requiring(drive)
            .executing(
                co -> {
                  everyKindOfMove()
                      .forEach(
                          (name, move) ->
                              NOT_DEADLINES.forEach(
                                  (seconds, shown) -> {
                                    String row = name + " within " + seconds;
                                    moves.put(row, move.get().within(Seconds.of(seconds)));
                                    expected.put(
                                        row,
                                        new Refused(
                                            name,
                                            "a deadline is a time above 0 s, not "
                                                + shown
                                                + " s: give within() how long the move may"
                                                + " take"));
                                  }));
                  for (DriveMove move : moves.values()) {
                    co.await(move);
                  }
                  ended[0] = true;
                })
            .named("Bad deadlines");
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(wrong);
    int loops = 0;
    while (!ended[0] && loops < 100) {
      harness.loop();
      loops++;
    }

    moves.forEach(
        (row, move) -> {
          Refused refused = Objects.requireNonNull(expected.get(row), row);
          ended(move)
              .named(refused.name())
              .refused()
              .because(MoveReason.REFUSED_AS_BUILT)
              .saying(refused.why());
        });
    assertThat(moves.values())
        .allSatisfy(move -> assertThat(move.outcome()).contains(Outcome.REFUSED));
    assertThat(ended[0]).as("the command ran on to its end").isTrue();
    assertThat(loops)
        .as("a loop for each refused move, and one to end")
        .isEqualTo(moves.size() + 1);
    assertThat(harness.simulation.truePose().getTranslation().getNorm()).isLessThan(0.01);
  }

  /** Canceled, a move says so, and stops the drivetrain. */
  @Test
  void aCanceledMoveSaysSo() {
    DriveMove out =
        drive.runUntil(
            new ChassisVelocities(1, 0, 0), Gate.of(harness.scheduler, "never", () -> false));
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(out);
    for (int i = 0; i < 10; i++) {
      harness.loop();
    }

    harness.scheduler.cancel(out);

    assertThat(out.outcome()).contains(Outcome.CANCELED);
    ended(out).named("drive until never").canceled().inBetween(0, 1);
  }
}

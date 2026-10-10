package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.world.path.Keyframe;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/** Following a path through keyframes, in the simulator. */
@ExtendWith(RobotSim.class)
class DriveFollowSimTest {
  private final DriveSimHarness harness = new DriveSimHarness(ExampleRobot.SIM);
  private final Drive drive = harness.drive;

  @AfterEach
  void tearDown() {
    harness.close();
  }

  private static Keyframe at(double x, double y, double degrees) {
    return Keyframe.at(new Pose2d(x, y, Rotation2d.fromDegrees(degrees)));
  }

  private Pose2d truth() {
    return harness.simulation.truePose();
  }

  /**
   * Through three keyframes, turning a half turn on the way: it passes within a few centimeters of
   * each, facing it (2 cm and 2° in the simulator), and arrives at the last within 2 cm and 2°,
   * moments after its path's 4.58 s.
   */
  @Test
  void itFollowsAPathThroughKeyframes() {
    Keyframe[] keyframes = {at(2, 1, 0), at(4, 0, 90), at(6, 1.5, 180)};
    DriveMove path = drive.follow(keyframes);
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(path);
    double[] nearest = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
    double[] facingThere = new double[2];
    int loops = 0;
    for (; loops < RobotLoop.loops(10) && path.outcome().isEmpty(); loops++) {
      harness.loop();
      for (int k = 0; k < 2; k++) {
        Pose2d keyframe = Objects.requireNonNull(keyframes[k].pose());
        double off = truth().getTranslation().getDistance(keyframe.getTranslation());
        if (off < nearest[k]) {
          nearest[k] = off;
          facingThere[k] = truth().getRotation().minus(keyframe.getRotation()).getDegrees();
        }
      }
    }

    assertThat(path.reason())
        .startsWith("drive → (2.00, 1.00) → (4.00, 0.00) → (6.00, 1.50) 180.0°: ARRIVED in ");
    for (int k = 0; k < 2; k++) {
      assertThat(nearest[k]).as("keyframe %d passed within", k).isLessThan(0.05);
      assertThat(facingThere[k]).as("keyframe %d faced to", k).isCloseTo(0, within(3.0));
    }
    assertThat(truth().getTranslation().getDistance(new Translation2d(6, 1.5))).isLessThan(0.03);
    assertThat(truth().getRotation().minus(Rotation2d.PI).getDegrees()).isCloseTo(0, within(2.5));
    assertThat(loops * RobotLoop.PERIOD_SECONDS).as("its path's 4.58 s, tracked").isLessThan(4.9);
  }

  /**
   * A named keyframe's gate opens as the robot passes it, within a few centimeters, and not before;
   * it stays open once the move has ended.
   */
  @Test
  void aNamedKeyframeOpensAGateAsItsPassed() {
    DriveMove path = drive.follow(at(1.5, 0.5, 0).named("pickup"), at(3, 0, 0));
    Gate pickup = path.passed("pickup");
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(path);
    double openedAt = Double.NaN;
    for (int i = 0; i < RobotLoop.loops(6) && path.outcome().isEmpty(); i++) {
      harness.loop();
      if (Double.isNaN(openedAt) && pickup.getAsBoolean()) {
        openedAt = truth().getTranslation().getDistance(new Translation2d(1.5, 0.5));
      }
    }

    assertThat(pickup.name()).isEqualTo("passed pickup");
    assertThat(path.arrived()).isTrue();
    assertThat(openedAt).as("from pickup as it opened").isLessThan(0.1);
    assertThat(pickup.getAsBoolean()).isTrue();
  }

  /**
   * A command that forks the path, waits for a named keyframe, and does something there, as the
   * driving guide shows: it acts as the robot passes the keyframe, while the path drives on.
   */
  @Test
  void aCommandActsAtANamedKeyframe() {
    double[] acted = {Double.NaN};
    Command pickUp =
        Command.requiring(drive)
            .executing(
                co -> {
                  DriveMove leg = drive.follow(at(1.5, 0.5, 0).named("pickup"), at(3, 0, 0));
                  co.fork(leg);
                  co.waitUntil(leg.passed("pickup"));
                  acted[0] = truth().getTranslation().getDistance(new Translation2d(1.5, 0.5));
                  co.await(leg);
                })
            .named("Pick up");
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(pickUp);
    for (int i = 0; i < RobotLoop.loops(6) && harness.scheduler.isScheduledOrRunning(pickUp); i++) {
      harness.loop();
    }

    assertThat(harness.scheduler.isScheduledOrRunning(pickUp))
        .as("it drove the path to its end")
        .isFalse();
    assertThat(acted[0]).as("from pickup as it acted").isLessThan(0.1);
    assertThat(truth().getTranslation().getDistance(new Translation2d(3, 0))).isLessThan(0.03);
  }

  /**
   * A path back to where it starts drives it, rather than arriving at once: it arrives once its
   * path's 4.45 s are up.
   */
  @Test
  void aLoopDrivesBeforeItArrives() {
    DriveMove loop = drive.follow(at(1.5, 0.75, 90), at(0, 1.5, 180), at(0, 0, 0));
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(loop);
    double farthest = 0;
    int loops = 0;
    for (; loops < RobotLoop.loops(12) && loop.outcome().isEmpty(); loops++) {
      harness.loop();
      farthest = Math.max(farthest, truth().getTranslation().getNorm());
    }

    assertThat(loop.arrived()).isTrue();
    assertThat(loops * RobotLoop.PERIOD_SECONDS).isBetween(4.45, 5.2);
    assertThat(farthest).isGreaterThan(1.4);
    assertThat(truth().getTranslation().getNorm()).isLessThan(0.03);
  }

  /** Keyframes with no path through them are refused, saying why, and the robot stays put. */
  @Test
  void keyframesWithNoPathAreRefused() {
    DriveMove nowhere = drive.follow(at(0.02, 0, 90));
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(nowhere);
    harness.loop();

    assertThat(nowhere.outcome()).contains(Outcome.REFUSED);
    assertThat(nowhere.reason())
        .isEqualTo(
            "drive → (0.02, 0.00) 90.0°: REFUSED: (0.02, 0.00) is within 5 cm of where the robot"
                + " starts");
  }

  /**
   * Keyframes are checked as the move is made: none, two of one name, or a name it hasn't. Made as
   * the robot starts, as its autos are, it throws, so the robot doesn't start.
   */
  @Test
  void badKeyframesMadeAsTheRobotStartsThrow() {
    assertThatThrownBy(() -> drive.follow())
        .hasMessage("a path needs a keyframe: give one, Keyframe.at(pose)");
    assertThatThrownBy(() -> drive.follow(at(1, 0, 0).named("a"), at(2, 0, 0).named("a")))
        .hasMessage("two keyframes are named a: name each once");
    assertThatThrownBy(() -> drive.follow(at(1, 0, 0)).passed("pickup"))
        .hasMessage(
            "drive → (1.00, 0.00) 0.0° has no keyframe pickup: name one,"
                + " Keyframe.at(...).named(\"pickup\")");
    assertThatThrownBy(() -> at(1, 0, 0).named(" ")).hasMessage("a keyframe's name can't be blank");
  }

  /**
   * Made once the robot runs, inside a command, the same are refused, saying the same: the moves
   * end REFUSED as they start, the gate on a name it hasn't holds false, and the command runs on.
   */
  @Test
  void badKeyframesMadeWhileTheRobotRunsAreRefused() {
    List<DriveMove> moves = new ArrayList<>();
    List<Gate> gates = new ArrayList<>();
    boolean[] ended = {false};
    Command wrong =
        Command.requiring(drive)
            .executing(
                co -> {
                  moves.add(drive.follow());
                  moves.add(drive.follow(at(1, 0, 0).named("a"), at(2, 0, 0).named("a")));
                  gates.add(drive.follow(at(1, 0, 0)).passed("pickup"));
                  for (DriveMove move : moves) {
                    co.await(move);
                  }
                  ended[0] = true;
                })
            .named("Bad keyframes");
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(wrong);
    for (int i = 0; i < 5; i++) {
      harness.loop();
    }

    assertThat(moves.get(0).outcome()).contains(Outcome.REFUSED);
    assertThat(moves.get(0).reason())
        .isEqualTo("drive → a path: REFUSED: a path needs a keyframe: give one, Keyframe.at(pose)");
    assertThat(moves.get(1).reason())
        .isEqualTo("drive → a → a 0.0°: REFUSED: two keyframes are named a: name each once");
    assertThat(gates.get(0).getAsBoolean()).isFalse();
    assertThat(gates.get(0).why())
        .isEqualTo(
            "passed pickup is false: drive → (1.00, 0.00) 0.0° has no keyframe pickup: name one,"
                + " Keyframe.at(...).named(\"pickup\")");
    assertThat(ended[0]).as("the command ran on to its end").isTrue();
    assertThat(truth().getTranslation().getNorm()).isLessThan(0.01);
  }
}

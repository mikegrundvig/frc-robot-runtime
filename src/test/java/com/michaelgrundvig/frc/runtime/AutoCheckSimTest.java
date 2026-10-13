package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.path.Keyframe;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The template's check of a robot's autos fails each way an auto can be wrong, saying which and
 * where, and passes one that is not.
 */
class AutoCheckSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  private static final Place LEFT = Rebuilt.starts(ExampleRobot.SIM.definition().body()).left();

  private List<AutoCheck.Result> run(ControlsMaker<?> controls) {
    return AutoCheck.run(sim, ExampleRobot.SIM, controls);
  }

  private static AutoCheck.Result of(List<AutoCheck.Result> results, String auto, Alliance side) {
    return results.stream()
        .filter(result -> result.auto().equals(auto) && result.alliance() == side)
        .findFirst()
        .orElseThrow();
  }

  @Test
  void anAutoThatDrivesToAPlaceOnEachAlliancePasses() {
    List<AutoCheck.Result> results =
        AutoCheck.check(
            sim,
            ExampleRobot.SIM,
            (scheduler, robot, joints, drive, driver, operator, autos) -> {
              autos.add(
                  "To the hub",
                  LEFT,
                  () ->
                      Command.noRequirements(co -> co.await(drive.goTo(Rebuilt.HUB))).named("Hub"));
              return autos;
            });

    assertThat(results).hasSize(4);
    for (AutoCheck.Result result : results) {
      assertThat(result.passed()).isTrue();
    }
    AutoCheck.Result onRed = of(results, "To the hub", Alliance.RED);
    assertThat(onRed.ended()).as("the robot started, and ended somewhere").isPresent();
    assertThat(onRed.moves())
        .singleElement()
        .satisfies(move -> move.named("drive → Hub").arrived());
  }

  @Test
  void aRefusedMoveFailsItSayingWhichAndWhy() {
    List<AutoCheck.Result> results =
        run(
            (scheduler, robot, joints, drive, driver, operator, autos) -> {
              autos.add(
                  "Nowhere",
                  LEFT,
                  () ->
                      Command.noRequirements(co -> co.await(drive.follow(Keyframe.at(LEFT))))
                          .named("Nowhere"));
              return autos;
            });

    AutoCheck.Result blue = of(results, "Nowhere", Alliance.BLUE);
    assertThat(blue.passed()).isFalse();
    assertThat(blue.problems())
        .singleElement()
        .asString()
        .startsWith(
            "a move was refused: drive → Left: REFUSED: Left is within 5 cm of where the robot starts");
  }

  @Test
  void anAutoThatThrowsFails() {
    List<AutoCheck.Result> results =
        run(
            (scheduler, robot, joints, drive, driver, operator, autos) -> {
              autos.add(
                  "Throws",
                  LEFT,
                  () -> {
                    throw new IllegalStateException("no such path");
                  });
              return autos;
            });

    assertThat(of(results, "Throws", Alliance.RED).problems())
        .anyMatch(problem -> problem.contains("it failed: Throws: FAILED"))
        .anyMatch(problem -> problem.contains("no such path"));
  }

  @Test
  void anAutoThatLeavesTheFieldFails() {
    List<AutoCheck.Result> results =
        run(
            (scheduler, robot, joints, drive, driver, operator, autos) -> {
              autos.add(
                  "Away",
                  LEFT,
                  () ->
                      Command.noRequirements(
                              co ->
                                  co.await(
                                      drive
                                          .runUntil(new ChassisVelocities(3, 0, 0), () -> false)
                                          .within(Seconds.of(12))))
                          .named("Away"));
              return autos;
            });

    assertThat(of(results, "Away", Alliance.BLUE).problems())
        .anyMatch(problem -> problem.startsWith("the robot left the field at "));
  }

  /** A start needn't be one of the field's: the placement check reads an auto's own too. */
  @Test
  void anAutoStartsFromAStartOfItsOwn() {
    Place elsewhere = Place.at("Elsewhere", new Pose2d(6, 3, Rotation2d.ZERO));
    List<AutoCheck.Result> results =
        run(
            (scheduler, robot, joints, drive, driver, operator, autos) -> {
              autos.add("Elsewhere", elsewhere, () -> Command.noRequirements(co -> {}).named("-"));
              return autos;
            });

    for (Alliance alliance : Alliance.values()) {
      AutoCheck.Result result = of(results, "Elsewhere", alliance);
      assertThat(result.problems()).isEmpty();
      assertThat(result.ended()).isPresent();
    }
  }

  @Test
  void theCheckFailsWithEveryRunReported() {
    assertThatThrownBy(
            () ->
                AutoCheck.check(
                    sim,
                    ExampleRobot.SIM,
                    (scheduler, robot, joints, drive, driver, operator, autos) -> {
                      autos.add(
                          "Throws",
                          LEFT,
                          () -> {
                            throw new IllegalStateException("no such path");
                          });
                      return autos;
                    }))
        .isInstanceOf(AssertionError.class)
        .hasMessageContaining("2 of 4 autos failed")
        .hasMessageContaining("Do nothing on BLUE: ended at (")
        .hasMessageContaining("PROBLEM: it failed: Throws: FAILED");
  }
}

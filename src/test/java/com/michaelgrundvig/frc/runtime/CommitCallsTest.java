package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import java.util.List;
import java.util.Map;
import java.util.function.UnaryOperator;
import org.junit.jupiter.api.Test;

/**
 * Each value the pit tunes prints the call that commits it, and that call is code that compiles and
 * commits it: each row pairs the printed call with the same call written here, which the compiler
 * checks against the API, applied where the configuration is written.
 */
class CommitCallsTest {
  /**
   * A tuned value, the call Pitwall prints for it, and that call as code, applied to the committed
   * configuration as pasting it into the block would.
   */
  private record Row<T>(String path, double tuned, String printed, UnaryOperator<T> pasted) {}

  /** A pivot's control, written from the factory it starts from: pasting a kP replaces it. */
  private static MechanismControl pivot(MechanismControl factory) {
    return factory.withKd(1).withKs(0.2).withStatorLimit(Amps.of(60)).withSupplyLimit(Amps.of(50));
  }

  /** A joint's kP commits by its control's factory, the rest by their withers. */
  @Test
  void aJointsTunedValuesCommitInItsBlock() {
    commits(
        JointMechanism.tuning(JointFixtures.PIVOT, pivot(MechanismControl.position(30))),
        List.of(
            new Row<>(
                "gains.kP",
                31.5,
                "MechanismControl.position(31.5)",
                control -> pivot(MechanismControl.position(31.5))),
            new Row<>("gains.kD", 1.5, ".withKd(1.5)", control -> control.withKd(1.5)),
            new Row<>("gains.kS", 0.25, ".withKs(0.25)", control -> control.withKs(0.25)),
            new Row<>(
                "currentLimits.stator",
                70,
                ".withStatorLimit(Amps.of(70.0))",
                control -> control.withStatorLimit(Amps.of(70.0))),
            new Row<>(
                "currentLimits.supply",
                45,
                ".withSupplyLimit(Amps.of(45.0))",
                control -> control.withSupplyLimit(Amps.of(45.0)))));
  }

  /** A roller's control starts from the velocity factory, so its kP commits there. */
  @Test
  void aRollersKpCommitsByItsFactory() {
    commits(
        JointMechanism.tuning(JointFixtures.ROLLER, MechanismControl.velocity(0.02)),
        List.of(
            new Row<>(
                "gains.kP",
                0.025,
                "MechanismControl.velocity(0.025)",
                control -> MechanismControl.velocity(0.025)),
            new Row<>("gains.kD", 0.5, ".withKd(0.5)", control -> control.withKd(0.5)),
            new Row<>("gains.kS", 0.1, ".withKs(0.1)", control -> control.withKs(0.1)),
            new Row<>(
                "currentLimits.stator",
                30,
                ".withStatorLimit(Amps.of(30.0))",
                control -> control.withStatorLimit(Amps.of(30.0))),
            new Row<>(
                "currentLimits.supply",
                30,
                ".withSupplyLimit(Amps.of(30.0))",
                control -> control.withSupplyLimit(Amps.of(30.0)))));
  }

  /**
   * The drivetrain's control is written with its records' builders in {@code Robots.java}, so each
   * value commits by its own builder step, a measure in the unit it was written in.
   */
  @Test
  void theDrivetrainsTunedValuesCommitByTheirBuilderSteps() {
    commits(
        RobotContainer.tunings(ExampleRobot.SIM).drivetrain(),
        List.of(
            new Row<>("gains.driveKs", 0.15, ".driveKs(0.15)", gains(b -> b.driveKs(0.15))),
            new Row<>("gains.driveKv", 2.2, ".driveKv(2.2)", gains(b -> b.driveKv(2.2))),
            new Row<>("gains.driveKp", 2.5, ".driveKp(2.5)", gains(b -> b.driveKp(2.5))),
            new Row<>("gains.steerKp", 18.5, ".steerKp(18.5)", gains(b -> b.steerKp(18.5))),
            new Row<>("gains.steerKd", 0.1, ".steerKd(0.1)", gains(b -> b.steerKd(0.1))),
            new Row<>(
                "gains.skewLookahead",
                0.05,
                ".skewLookahead(Seconds.of(0.05))",
                gains(b -> b.skewLookahead(Seconds.of(0.05)))),
            new Row<>(
                "limits.driveStatorCurrent",
                35,
                ".driveStatorCurrent(Amps.of(35.0))",
                limits(b -> b.driveStatorCurrent(Amps.of(35.0)))),
            new Row<>(
                "limits.driveSupplyCurrent",
                55,
                ".driveSupplyCurrent(Amps.of(55.0))",
                limits(b -> b.driveSupplyCurrent(Amps.of(55.0)))),
            new Row<>(
                "limits.driveSupplyLowerCurrent",
                35,
                ".driveSupplyLowerCurrent(Amps.of(35.0))",
                limits(b -> b.driveSupplyLowerCurrent(Amps.of(35.0)))),
            new Row<>(
                "limits.driveSupplyLowerTime",
                0.5,
                ".driveSupplyLowerTime(Seconds.of(0.5))",
                limits(b -> b.driveSupplyLowerTime(Seconds.of(0.5)))),
            new Row<>(
                "limits.steerStatorCurrent",
                35,
                ".steerStatorCurrent(Amps.of(35.0))",
                limits(b -> b.steerStatorCurrent(Amps.of(35.0)))),
            new Row<>(
                "limits.maxLinearSpeed",
                4.0,
                ".maxLinearSpeed(MetersPerSecond.of(4.0))",
                limits(b -> b.maxLinearSpeed(MetersPerSecond.of(4.0)))),
            new Row<>(
                "limits.maxAngularSpeed",
                1.5,
                ".maxAngularSpeed(RotationsPerSecond.of(1.5))",
                limits(b -> b.maxAngularSpeed(RotationsPerSecond.of(1.5))))));
  }

  /**
   * Every value the tuning has, a row each, in order: each tuned alone prints its row's call, and
   * that call, pasted, makes the configuration the tuning has.
   */
  private static <T extends Record> void commits(LiveConfig<T> tuning, List<Row<T>> rows) {
    assertThat(rows)
        .extracting(Row::path)
        .as("a row for every value " + tuning.name() + " tunes")
        .containsExactlyElementsOf(tuning.knobs().stream().map(LiveConfig.Knob::path).toList());
    for (Row<T> row : rows) {
      tuning.revert();
      assertThat(tuning.set(Map.of(row.path(), row.tuned()))).as(row.path()).isTrue();
      assertThat(tuning.changes())
          .singleElement()
          .extracting(LiveConfig.Change::builderCall)
          .as(row.path())
          .isEqualTo(row.printed());
      assertThat(row.pasted().apply(tuning.committed())).as(row.printed()).isEqualTo(tuning.get());
    }
  }

  /** A call pasted into the drivetrain's gains' builder. */
  private static UnaryOperator<DrivetrainControl> gains(
      UnaryOperator<DrivetrainControlGainsBuilder> step) {
    return control ->
        DrivetrainControlBuilder.builder(control)
            .gains(step.apply(DrivetrainControlGainsBuilder.builder(control.gains())).build())
            .build();
  }

  /** A call pasted into the drivetrain's limits' builder. */
  private static UnaryOperator<DrivetrainControl> limits(
      UnaryOperator<DrivetrainControlLimitsBuilder> step) {
    return control ->
        DrivetrainControlBuilder.builder(control)
            .limits(step.apply(DrivetrainControlLimitsBuilder.builder(control.limits())).build())
            .build();
  }
}

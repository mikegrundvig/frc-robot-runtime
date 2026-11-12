package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.world.field.TagFamily;
import com.michaelgrundvig.frc.world.game.Game;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import java.util.List;
import java.util.Optional;
import java.util.function.UnaryOperator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/**
 * A robot whose cameras on Visor Visor couldn't run stops as the program starts, built as {@link
 * Robot} builds it, saying which camera and why: a control the exposure or gain sets, in Visor's
 * words, or a field whose tags Visor doesn't detect. The same robot on the season's field, its
 * controls Visor's own, starts.
 */
class VisorStartupTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  /** The season's game, its tags printed 16h5 as 2023's were. */
  private static final Game SIXTEEN_H_FIVE =
      new Game(
          Rebuilt.GAME.official(),
          Rebuilt.GAME.variants(),
          Rebuilt.GAME.symmetry(),
          TagFamily.TAG_16H5,
          Inches.of(6),
          Rebuilt::field,
          Rebuilt.GAME.limits());

  /** The simulator's robot with its first camera on Visor, naming this control. */
  private static RobotConfig withControl(String control) {
    CameraConfig first = ExampleRobot.SIM.definition().cameras().get(0);
    Software.Visor visor = Software.visor().controls(List.of(Software.Control.of(control, 1)));
    return ExampleRobot.SIM.with(first.withSoftware(visor));
  }

  private record Row(String what, RobotConfig robot, Game game, String refused) {}

  @Test
  void aRobotVisorCantRunStopsAsItStarts() {
    String camera = ExampleRobot.SIM.definition().cameras().get(0).name();
    List<Row> rows =
        List.of(
            new Row(
                "its own control, on the season's field",
                withControl("brightness"),
                Rebuilt.GAME,
                ""),
            new Row(
                "the exposure as a control",
                withControl("auto_exposure"),
                Rebuilt.GAME,
                camera + ": Visor refuses it: control auto_exposure is set by exposureMicros(..)"),
            new Row(
                "the gain as a control",
                withControl("gain"),
                Rebuilt.GAME,
                camera + ": Visor refuses it: control gain is set by gain(..)"),
            new Row(
                "a field of 16h5 tags",
                withControl("brightness"),
                SIXTEEN_H_FIVE,
                camera + " is on Visor, which detects TAG_36H11 tags only"));
    for (Row row : rows) {
      if (row.refused().isEmpty()) {
        assertThatCode(() -> start(row)).as(row.what()).doesNotThrowAnyException();
        continue;
      }
      assertThatThrownBy(() -> start(row))
          .as(row.what())
          .isInstanceOf(IllegalArgumentException.class)
          .hasMessageContaining(row.refused());
    }
  }

  /**
   * Builds the robot as the program does, in a replay, and closes it. A robot refused as it starts
   * never takes the loop's health to close it, so this does.
   */
  private void start(Row row) {
    LoopHealth health = new LoopHealth();
    try (Pitwall page = Pitwall.start(row.robot().name(), 0)) {
      RobotContainer built;
      try {
        built =
            Robot.assemble(
                RunMode.REPLAY,
                RunsOn.DESKTOP,
                sim.scheduler(),
                row.robot(),
                row.game(),
                ControlsMaker.NONE,
                page,
                Optional.empty(),
                health,
                Optional.empty(),
                UnaryOperator.identity());
      } catch (RuntimeException refused) {
        health.close();
        throw refused;
      }
      built.close();
    }
  }
}

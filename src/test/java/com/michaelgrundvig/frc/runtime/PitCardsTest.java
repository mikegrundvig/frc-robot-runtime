package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.runtime.hardware.RunsOn;
import com.michaelgrundvig.frc.runtime.venues.VenueStore;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.function.UnaryOperator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.api.io.TempDir;

/**
 * The cards the robot's configuration checks the names of ({@link PitCards}) are the cards the
 * robot program puts on Pitwall's Pit view: built as {@link Robot} builds it, with its venues and
 * its coprocessors (a replay's, which watches the boards its configuration lists), the page has
 * those cards and no others. A card added to the page and not to {@link PitCards} fails here, so
 * its name can't clash unchecked.
 */
class PitCardsTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @TempDir Path folder;

  /** A robot, and the coprocessors its configuration lists. */
  private record Listed(String name, RobotConfig robot, List<String> boards) {}

  @Test
  void theCardsCheckedAreTheCardsOnThePage() throws Exception {
    List<Listed> rows =
        List.of(
            new Listed(
                "the simulator's robot, two boards",
                ExampleRobot.SIM,
                List.of("10.12.34.11", "10.12.34.12")),
            new Listed(
                "the competition robot, one board",
                ExampleRobot.COMPETITION,
                List.of("10.12.34.11")),
            new Listed("no boards", ExampleRobot.SIM, List.of()));
    for (Listed row : rows) {
      RobotConfig robot =
          RobotConfigBuilder.builder(row.robot()).coprocessors(row.boards()).build();
      Path venues = Files.createTempDirectory(folder, "venues");
      VenueStore store =
          new VenueStore(
              Files.createDirectories(venues.resolve("stored")),
              Files.createDirectories(venues.resolve("committed")));
      List<String> checked =
          PitCards.of(robot.definition(), robot.coprocessors()).stream()
              .map(PitCards.Card::name)
              .toList();
      try (Pitwall page = Pitwall.start(robot.name(), 0)) {
        RobotContainer built =
            Robot.assemble(
                RunMode.REPLAY,
                RunsOn.DESKTOP,
                sim.scheduler(),
                robot,
                Rebuilt.GAME,
                ControlsMaker.NONE,
                page,
                Optional.empty(),
                new LoopHealth(),
                Optional.of(store),
                UnaryOperator.identity());
        List<String> onThePage = page.devices().all().stream().map(Devices.Device::name).toList();
        built.close();
        assertThat(onThePage).as(row.name()).containsExactlyInAnyOrderElementsOf(checked);
      }
    }
  }
}

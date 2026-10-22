package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.offset;
import static org.assertj.core.api.Assertions.tuple;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.pitwall.robot.RobotSession;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Scenes;
import com.michaelgrundvig.frc.pitwall.world.SimulatedParts;
import com.michaelgrundvig.frc.pitwall.world.WorldScene;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.Faults;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * The starting positions on the World view's 2026 field, for the example robot: where each alliance
 * starts, the robot standing on each by its bumpers, and what a click on one offers in the
 * simulator and on a robot.
 */
class WorldStartsTest {
  private static final RobotDefinition ROBOT = ExampleRobot.COMPETITION.definition();
  private static final FieldModel FIELD = Rebuilt.field();
  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  /** The 2026 field's three starts, left to right as the drivers see them. */
  private static final List<String> NAMES = List.of("Left", "Center", "Right");

  private final Scenes scenes = new Scenes(System::nanoTime, System.out::println);
  private final RobotSession session =
      RobotSession.of(
          ROBOT, RobotLoop.PERIOD_SECONDS, RobotSession.REMEMBERED_SECONDS, Vision.sightingWords());

  private void onAPage(SimulatedParts parts) {
    WorldScene world =
        WorldScene.of(
            ROBOT,
            () -> FIELD,
            WorldScene.Truth.IN_THE_RECORD,
            Vision.sightingWords(),
            camera -> Optional.empty(),
            parts);
    scenes.add(world.scene(session));
  }

  private Inspection click(String shape) {
    return scenes.inspect(WorldScene.ID, shape).orElseThrow();
  }

  private static String where(double x, double y) {
    return String.format(Locale.ROOT, "x %.3f m, y %.3f m", x, y);
  }

  /**
   * The scene has the field's three starts for each alliance, blue's with their bumpers on the
   * starting line and red's a half turn from them, across the field. A click on each says which it
   * is, whose, and where.
   */
  @Test
  void theSceneCarriesTheFieldsStartsForBothAlliancesAndAClickNamesEach() {
    onAPage(SimulatedParts.NONE);
    String described = JSON.toJson(scenes.descriptions());
    double lineX = Rebuilt.STARTS.left().on(FIELD, Alliance.BLUE, ROBOT.body().bumpers()).getX();
    double length = FIELD.length();
    double width = FIELD.width();

    assertThat(described).contains("\"starts\":{\"title\":\"Starting positions\",\"shown\":true");
    // The front bumper is on the starting line, 4.022 m from blue's wall.
    assertThat(lineX + ROBOT.body().bumperLength().in(Meters) / 2).isEqualTo(4.022, offset(1e-9));
    for (int index = 0; index < NAMES.size(); index++) {
      String name = NAMES.get(index);
      double blueY =
          FIELD.starts().get(index).on(FIELD, Alliance.BLUE, ROBOT.body().bumpers()).getY();
      assertThat(described)
          .contains("\"text\":\"Blue " + name.toLowerCase(Locale.ROOT) + "\"")
          .contains("\"text\":\"Red " + name.toLowerCase(Locale.ROOT) + "\"");
      assertThat(click("start.blue." + index + ".outline"))
          .satisfies(
              start -> {
                assertThat(start.title())
                    .isEqualTo("Blue " + name.toLowerCase(Locale.ROOT) + " start");
                assertThat(start.details())
                    .extracting(Inspection.Detail::label, Inspection.Detail::value)
                    .contains(
                        tuple("Name", name),
                        tuple("Alliance", "Blue"),
                        tuple("Where", where(lineX, blueY)),
                        tuple("Heading", "0.0°"));
              });
      assertThat(click("start.red." + index + ".outline"))
          .satisfies(
              start -> {
                assertThat(start.title())
                    .isEqualTo("Red " + name.toLowerCase(Locale.ROOT) + " start");
                assertThat(start.details())
                    .extracting(Inspection.Detail::label, Inspection.Detail::value)
                    .contains(
                        tuple("Alliance", "Red"),
                        tuple("Where", where(length - lineX, width - blueY)),
                        tuple("Heading", "180.0°"));
              });
    }
  }

  /**
   * In the simulator a click on a start offers the robot's own Placed fault with the start's pose,
   * so one press sets the robot down on it. On a robot it offers nothing.
   */
  @Test
  void inTheSimulatorAStartOffersToPlaceTheRobotThereAndOnARobotItDoesNot() {
    onAPage(Faults.parts());
    Inspection red = click("start.red.0.outline");
    Map<String, Double> values = red.offers().getFirst().values();

    assertThat(red.offers()).hasSize(1);
    assertThat(red.offers().getFirst().component()).isEqualTo("Robot");
    assertThat(red.offers().getFirst().fault()).isEqualTo("Placed");
    assertThat(values).containsOnlyKeys("X", "Y", "Heading");
    assertThat(values.get("Heading")).isEqualTo(180.0);

    Scenes onARobot = new Scenes(System::nanoTime, System.out::println);
    onARobot.add(
        WorldScene.of(
                ROBOT,
                () -> FIELD,
                WorldScene.Truth.NOT_KNOWN,
                Vision.sightingWords(),
                camera -> Optional.empty())
            .scene(session));
    assertThat(onARobot.inspect(WorldScene.ID, "start.blue.1.outline").orElseThrow().offers())
        .isEmpty();
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ControlsMaker;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * What stands on the field hides tags from the simulated cameras. The template's robot is taken
 * across the far side of blue's hub, facing away from blue's wall, so that its back-right camera
 * looks back at the two tags on the tower's wall: past the hub on one side, into the hub, and past
 * it on the other. On the season's field with nothing standing on it, the same camera sees both
 * tags the whole way.
 */
@SimulationTest
class HiddenTagsSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  private static final List<CameraConfig> CAMERAS = ExampleRobot.SIM.definition().cameras();

  /** The season's own field, with its hub, trenches, towers and outposts standing. */
  private static final FieldModel SEASON = Rebuilt.field(ExampleRobot.SIM.definition().body());

  /** The camera that looks back at the tower past the hub, and the tower's two tags. */
  private static final CameraConfig BACK_RIGHT =
      CAMERAS.stream()
          .filter(camera -> camera.name().equals("back-right"))
          .findFirst()
          .orElseThrow();

  private static final Set<Integer> TOWER_TAGS = Set.of(31, 32);

  /** How far from blue's wall the robot crosses: two meters beyond the hub's far face. */
  private static final double CROSSING_X = 6.6;

  /** The tags one camera reports from a pose, with exact cameras, once its images have come. */
  private static Set<Integer> seen(FieldModel field, CameraConfig config, Pose2d robot) {
    Set<Integer> ids = new TreeSet<>();
    try (VisionSim vision =
        new VisionSim(field, Map.of(), camera -> Seeds.camera(1, camera), VisionSim.Noise.NONE)) {
      PhotonCamera camera = ((CameraIOPhotonVision) vision.camera(config)).camera();
      for (int loop = 0; loop < 10; loop++) {
        RobotSim.stepClock();
        vision.update(new Pose3d(robot));
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
          ids.clear();
          result.getTargets().forEach(target -> ids.add(target.getFiducialId()));
        }
      }
      camera.close();
    }
    return ids;
  }

  /** Whether the hub is between the camera and the tower's tags at a place along the crossing. */
  enum TowerTags {
    IN_SIGHT,
    BEHIND_THE_HUB
  }

  static Stream<Arguments> alongTheCrossing() {
    return Stream.of(
        Arguments.argumentSet(
            "before the hub, to the drivers' right of it", 2.5, TowerTags.IN_SIGHT),
        Arguments.argumentSet("behind the hub", 4.0, TowerTags.BEHIND_THE_HUB),
        Arguments.argumentSet("still behind it", 4.5, TowerTags.BEHIND_THE_HUB),
        Arguments.argumentSet("past the hub, to the drivers' left of it", 6.0, TowerTags.IN_SIGHT));
  }

  @ParameterizedTest
  @MethodSource("alongTheCrossing")
  void theHubHidesTheTowersTagsWhileItIsBetweenThemAndTheCamera(double y, TowerTags towerTags) {
    Pose2d robot = new Pose2d(CROSSING_X, y, Rotation2d.ZERO);

    assertThat(seen(RobotSim.FIELD, BACK_RIGHT, robot))
        .as("with nothing standing on the field")
        .containsAll(TOWER_TAGS);
    Set<Integer> reported = seen(SEASON, BACK_RIGHT, robot);
    switch (towerTags) {
      case IN_SIGHT -> assertThat(reported).containsAll(TOWER_TAGS);
      case BEHIND_THE_HUB -> assertThat(reported).doesNotContainAnyElementsOf(TOWER_TAGS);
    }
  }

  /**
   * What each camera reports from each of the season's starts, on blue: the same as on the open
   * floor but for one tag. From the Center start, the tower's own upright stands between the
   * back-left camera and tag 32 on the tower's wall.
   */
  @Test
  void fromTheStartsOnlyTheTowersOwnUprightHidesATag() {
    for (Place start : Rebuilt.starts(ExampleRobot.SIM.definition().body()).all()) {
      Pose2d robot = start.on(SEASON, Alliance.BLUE);
      for (CameraConfig camera : CAMERAS) {
        Set<Integer> open = seen(RobotSim.FIELD, camera, robot);
        Set<Integer> real = seen(SEASON, camera, robot);
        boolean theOneHidden = start.name().equals("Center") && camera.name().equals("back-left");
        if (theOneHidden) {
          assertThat(open).containsExactly(31, 32);
          assertThat(real).containsExactly(31);
        } else {
          assertThat(real).as(start.name() + ", " + camera.name()).isEqualTo(open);
        }
      }
    }
  }

  /**
   * The whole robot program driven along the crossing on the season's field, sensors erring as a
   * real robot's do. The tower's tags go behind the hub and come back, other tags come and go, and
   * the estimate stays with the truth the whole way.
   */
  @Test
  void drivenAcrossBehindTheHubTheEstimateStaysWithTheTruth() {
    Pose2d start = new Pose2d(CROSSING_X, 2.5, Rotation2d.ZERO);
    sim.build(ExampleRobot.SIM, Rebuilt::game, ControlsMaker.NONE, SensorErrors.TYPICAL);
    sim.loops(1);
    sim.placeAt(start);
    sim.loops(5);
    RobotSim.setEnabled(true);
    // The left stick pushed left drives toward the blue drivers' left: along the crossing.
    sim.driver().setLeftX(-0.5);
    sim.driver().notifyNewData();

    double worstErrorMeters = 0;
    for (int loop = 0; loop < 400 && sim.simulation().truePose().getY() < 6.0; loop++) {
      sim.loops(1);
      Pose2d truth = sim.simulation().truePose();
      Pose2d estimate = sim.drive().getPose();
      worstErrorMeters =
          Math.max(worstErrorMeters, truth.getTranslation().getDistance(estimate.getTranslation()));
    }
    assertThat(sim.simulation().truePose().getY()).as("it crossed").isGreaterThanOrEqualTo(6.0);
    assertThat(worstErrorMeters).as("the estimate's worst error on the way").isLessThan(0.05);
  }
}

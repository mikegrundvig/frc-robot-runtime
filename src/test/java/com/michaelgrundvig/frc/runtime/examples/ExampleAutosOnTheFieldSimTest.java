package com.michaelgrundvig.frc.runtime.examples;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.AutoCheck;
import com.michaelgrundvig.frc.runtime.Clearance;
import com.michaelgrundvig.frc.runtime.ExampleControls;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.Body;
import java.util.stream.Stream;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The guide's autos run on the season's own field, with the hub, the trenches, the towers and the
 * walls standing, where the other tests of them drive an open floor. Each must run clean: end where
 * its last move ends, and never touch anything on the way.
 */
class ExampleAutosOnTheFieldSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  private static final Body BODY = ExampleRobot.COMPETITION.definition().body();
  private static final FieldModel FIELD = Rebuilt.field();

  /** How near its last place an auto ends, in meters. */
  private static final double ENDS_WITHIN = 0.08;

  /**
   * The least room the bumpers must keep once the robot has left its start, in meters. On the
   * Center start itself they stand 7 mm from the hub's face, as the field's start has them.
   */
  private static final double STAYS_CLEAR_BY = 0.05;

  /** How far from its start the robot counts as having left it, in meters. */
  private static final double LEFT_ITS_START = 0.3;

  static Stream<Arguments> autos() {
    Translation2d pickup = new Translation2d(2, 6);
    Translation2d shot = new Translation2d(3, 3);
    return Stream.of(
        Arguments.argumentSet("Grab one on blue", "Grab one", Alliance.BLUE, pickup),
        Arguments.argumentSet("Grab one on red", "Grab one", Alliance.RED, pickup),
        Arguments.argumentSet("Grab and shoot on blue", "Grab and shoot", Alliance.BLUE, shot),
        Arguments.argumentSet("Grab and shoot on red", "Grab and shoot", Alliance.RED, shot));
  }

  @ParameterizedTest
  @MethodSource("autos")
  void theAutoRunsCleanAmongWhatStandsOnTheField(
      String auto, Alliance alliance, Translation2d lastForBlue) {
    RobotSim.setEnabled(false);
    sim.build(ExampleRobot.COMPETITION, Rebuilt.GAME, ExampleControls::new, SensorErrors.NONE);
    RobotSim.station(alliance == Alliance.RED ? AllianceStationID.RED_2 : AllianceStationID.BLUE_2);
    sim.autos().choose(auto);
    sim.loops(5);
    Clearance clearance = new Clearance(FIELD, BODY);
    Pose2d start = sim.simulation().truePose();

    double leastRoom = Double.POSITIVE_INFINITY;
    double leastRoomOffTheStart = Double.POSITIVE_INFINITY;
    Pose2d tightest = start;
    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    for (int loop = 0; loop < RobotLoop.loops(AutoCheck.AUTONOMOUS_SECONDS); loop++) {
      sim.loop();
      Pose2d truth = sim.simulation().truePose();
      double room = clearance.at(truth);
      leastRoom = Math.min(leastRoom, room);
      boolean offTheStart =
          truth.getTranslation().getDistance(start.getTranslation()) > LEFT_ITS_START;
      if (offTheStart && room < leastRoomOffTheStart) {
        leastRoomOffTheStart = room;
        tightest = truth;
      }
    }
    Pose2d ended = sim.simulation().truePose();
    RobotSim.enable(RobotMode.AUTONOMOUS, false);
    sim.loops(3);

    assertThat(leastRoom).as("meters of room at the least: it never touched").isPositive();
    assertThat(leastRoomOffTheStart)
        .as("meters of room at the tightest spot off its start, %s", tightest)
        .isGreaterThan(STAYS_CLEAR_BY);
    Translation2d expected = FIELD.forAlliance(alliance, lastForBlue);
    assertThat(ended.getTranslation().getDistance(expected))
        .as("meters from its last place %s", expected)
        .isLessThan(ENDS_WITHIN);
  }
}

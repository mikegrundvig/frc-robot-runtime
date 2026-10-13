package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.path.Keyframe;
import com.michaelgrundvig.frc.world.record.MoveReason;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Moves to a place, for the robot's alliance: one move, written once for blue, is right on both,
 * because the place is found when the move starts and never when it's built.
 */
@ExtendWith(RobotSim.class)
class DrivePlacesSimTest {
  private final DriveSimHarness harness = new DriveSimHarness(ExampleRobot.SIM);
  private final Drive drive = harness.drive;

  /** Beside the middle of the field, so blue's and red's images are a short drive apart. */
  private static final Pose2d MIDDLE = new Pose2d(8.27, 4.03, Rotation2d.ZERO);

  private static final Place MARK = Place.at("Mark", new Pose2d(6.8, 3.2, Rotation2d.PI));
  private static final Place AHEAD = Place.at("Ahead", new Pose2d(7.4, 3.2, Rotation2d.PI));

  @AfterEach
  void tearDown() {
    harness.close();
  }

  /** The robot is exactly here, and the drive knows it. */
  private void startAt(Pose2d pose) {
    harness.simulation.place(pose);
    harness.loop();
    harness.localization.resetPose(pose);
    harness.loop();
  }

  private Pose2d truth() {
    return harness.simulation.truePose();
  }

  private void run(DriveMove move) {
    RobotSim.setEnabled(true);
    harness.scheduler.schedule(move);
    for (int i = 0; i < RobotLoop.loops(10) && move.outcome().isEmpty(); i++) {
      harness.loop();
    }
  }

  private void assertAt(Pose2d expected) {
    assertThat(truth().getTranslation().getDistance(expected.getTranslation())).isLessThan(0.03);
    assertThat(truth().getRotation().minus(expected.getRotation()).getDegrees())
        .isCloseTo(0, within(2.5));
  }

  /**
   * The same move arrives at the place as it is for each alliance: red's is blue's turned round.
   */
  @Test
  void goingToAPlaceArrivesAtItsPoseForTheAlliance() {
    for (Alliance alliance : Alliance.values()) {
      harness.alliance(Optional.of(alliance));
      startAt(MIDDLE);
      DriveMove go = drive.goTo(MARK);

      run(go);

      ended(go).named("drive → Mark").arrived();
      assertAt(MARK.on(RobotSim.FIELD, alliance));
    }
    assertThat(MARK.on(RobotSim.FIELD, Alliance.RED).getX())
        .as("red's Mark is across the field from blue's")
        .isGreaterThan(9.7);
  }

  /** Made before the alliance is known, it is the alliance at the move's start that counts. */
  @Test
  void theAllianceIsReadWhenTheMoveStartsNotWhenItsBuilt() {
    harness.alliance(Optional.empty());
    DriveMove go = drive.goTo(MARK);
    harness.alliance(Optional.of(Alliance.BLUE));
    startAt(MIDDLE);
    run(go);
    Pose2d onBlue = truth();
    harness.alliance(Optional.of(Alliance.RED));
    harness.run(go, 6); // the first run's outcome stands until the move runs again

    assertThat(onBlue.getX()).as("started as blue").isCloseTo(6.8, within(0.03));
    assertThat(truth().getX()).as("the same move again, started as red").isGreaterThan(9.7);
    ended(go).named("drive → Mark").arrived();
  }

  /** With the alliance unknown when it starts, it is refused, saying so, and the robot stays. */
  @Test
  void aMoveStartedWithTheAllianceUnknownIsRefused() {
    harness.alliance(Optional.empty());
    startAt(MIDDLE);
    DriveMove go = drive.goTo(MARK);
    DriveMove path = drive.follow(Keyframe.at(AHEAD), Keyframe.at(MARK));

    run(go);
    run(path);

    assertThat(go.outcome()).contains(Outcome.REFUSED);
    ended(go)
        .named("drive → Mark")
        .refused()
        .because(MoveReason.PATH_NOT_PLANNED)
        .saying("alliance unknown");
    assertThat(path.outcome()).contains(Outcome.REFUSED);
    ended(path)
        .named("drive → Ahead → Mark")
        .refused()
        .because(MoveReason.PATH_NOT_PLANNED)
        .saying("alliance unknown");
    assertThat(truth().getTranslation().getDistance(MIDDLE.getTranslation())).isLessThan(0.01);
  }

  /** A path of places is right on both alliances with no other change. */
  @Test
  void aPathOfPlacesIsRightOnBothAlliances() {
    for (Alliance alliance : Alliance.values()) {
      harness.alliance(Optional.of(alliance));
      startAt(MIDDLE);
      DriveMove path =
          drive.follow(Keyframe.at(AHEAD).named("ahead"), Keyframe.at(MARK).named("mark"));

      run(path);

      ended(path).named("drive → ahead → mark").arrived();
      assertAt(MARK.on(RobotSim.FIELD, alliance));
    }
  }
}

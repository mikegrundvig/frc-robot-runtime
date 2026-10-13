package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER_FEEDING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleControls;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.MoveEnding;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.record.JointPlace;
import com.michaelgrundvig.frc.world.record.MoveReason;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The order of the guide's aimed shot, on the guide's own robot: nothing is fed before the shooter
 * is at its speed.
 *
 * <p>The shooter's flywheel is what lets this show. It takes the shooter about half a second to
 * reach 3000 rpm, so a feed that started even one loop early would start with the shooter far below
 * its speed.
 */
class AimedShotOrderSimTest {
  private static final double RPM = 2 * Math.PI / 60;

  /** How near its speed the shooter stays once there: the band its "at goal" may stray within. */
  private static final double STAYS_WITHIN = 0.06 * 3000 * RPM;

  @RegisterExtension final RobotSim sim = new RobotSim();

  /** What one loop of the shot showed. */
  private record Loop(boolean feederToldFeeding, double shooterSpeed) {}

  /**
   * The driver presses Shoot with the robot already aimed and the shooter idle and stopped. Before
   * the press both of the feed's conditions read true: the robot is aimed, and a stopped shooter
   * that nobody has asked anything of is where it is asked to be. The press asks the shooter for
   * 3000 rpm, and from then on the feed waits for that.
   */
  @Test
  void pressedWhileAimedWithTheShooterStoppedNothingIsFedUntilTheShooterIsAtSpeed() {
    sim.build(ExampleRobot.COMPETITION, SensorErrors.NONE);
    sim.controls(ExampleControls::new);
    RobotSim.setEnabled(true);
    Translation2d hub = Rebuilt.HUB.on(RobotSim.FIELD, Alliance.BLUE).getTranslation();
    Translation2d from = new Translation2d(3, 2.5);
    Rotation2d towardTheHub =
        new Rotation2d(Math.atan2(hub.getY() - from.getY(), hub.getX() - from.getX()));
    sim.placeAt(new Pose2d(from, towardTheHub));
    sim.run(0.5);
    JointSimulation truth = Objects.requireNonNull(sim.world().joints());
    JointMechanism feeder = sim.joints().of(FEEDER);
    JointPlace feeding = JointPlace.running(FEEDER.outputs().indexOf(FEEDER_FEEDING));
    boolean aimedBeforeThePress =
        sim.drive().facing(Heading.toward(Rebuilt.HUB)).aimed().getAsBoolean();
    double shooterBeforeThePress = truth.velocity(SHOOTER);

    sim.driver().setRightTriggerAxis(1);
    sim.driver().notifyNewData();
    List<Loop> loops = new ArrayList<>();
    List<MoveEnding> feedsThatEnded = new ArrayList<>();
    Optional<MoveEnding> lastEnding = feeder.lastEnding();
    for (int loop = 0; loop < 200; loop++) {
      sim.loop();
      loops.add(new Loop(feeder.place().equals(feeding), truth.velocity(SHOOTER)));
      if (!feeder.lastEnding().equals(lastEnding)) {
        lastEnding = feeder.lastEnding();
        feedsThatEnded.add(lastEnding.orElseThrow());
      }
    }

    assertThat(aimedBeforeThePress).as("aimed before the press").isTrue();
    assertThat(shooterBeforeThePress).as("the shooter is stopped before the press").isZero();
    List<Integer> fedIn = new ArrayList<>();
    List<Integer> fedBelowSpeedIn = new ArrayList<>();
    for (int loop = 0; loop < loops.size(); loop++) {
      Loop seen = loops.get(loop);
      if (seen.feederToldFeeding()) {
        fedIn.add(loop);
      }
      boolean belowSpeed = Math.abs(seen.shooterSpeed() - 3000 * RPM) > STAYS_WITHIN;
      if (seen.feederToldFeeding() && belowSpeed) {
        fedBelowSpeedIn.add(loop);
      }
    }
    String whatHappened =
        "the feeder was told to feed in loops "
            + fedIn
            + " after the press, and the feeds that ended were "
            + feedsThatEnded.stream().map(MoveEnding::sentence).toList();
    assertThat(fedBelowSpeedIn)
        .as("the loops in which the feeder was told to feed below speed; %s", whatHappened)
        .isEmpty();
    assertThat(fedIn).as("the feed did start, once the shooter was up").isNotEmpty();
    assertThat(fedIn.getFirst())
        .as("the shooter takes some loops to spin up, or the shot shows nothing")
        .isGreaterThan(5);
    assertThat(feedsThatEnded)
        .as("no feed ended while the trigger was held: none was cut short by its gate")
        .extracting(MoveEnding::reason)
        .doesNotContain(MoveReason.GATE_FELL)
        .isEmpty();
  }
}

package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER_FEEDING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.runtime.ExampleControls;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
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
 * The order of the guide's aimed shot, on a robot whose shooter takes time to spin up: nothing is
 * fed before the shooter is at its speed.
 *
 * <p>The guide's own robot cannot show this. Its shooter has no flywheel to turn, so the simulator
 * has it at 3000 rpm one loop after it is asked, and a feed that started a loop early would look
 * the same as one that waited. The robot here is the guide's with a shooter that is slow to spin
 * up.
 */
class AimedShotOrderSimTest {
  private static final double RPM = 2 * Math.PI / 60;

  /** How near its speed the shooter stays once there: the band its "at goal" may stray within. */
  private static final double STAYS_WITHIN = 0.06 * 3000 * RPM;

  /**
   * The guide's robot with its shooter's motors held to one amp each, which takes the shooter well
   * over a second to bring to speed. A flywheel would do the same and be truer to a real shooter,
   * but a joint's link is part of what makes it the same joint on a robot derived from another
   * ({@code RobotDefinition.with}), and the guide's controls ask for the guide's shooter.
   */
  private static final RobotConfig SLOW_TO_SPIN_UP =
      ExampleRobot.COMPETITION
          .named("slow to spin up")
          .with(SHOOTER.retuned(control -> control.withStatorLimit(Amps.of(1))));

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
    sim.build(SLOW_TO_SPIN_UP, SensorErrors.NONE);
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
        .as("the shooter takes some loops to spin up, or this robot shows nothing")
        .isGreaterThan(5);
    assertThat(feedsThatEnded)
        .as("no feed ended while the trigger was held: none was cut short by its gate")
        .extracting(MoveEnding::reason)
        .doesNotContain(MoveReason.GATE_FELL)
        .isEmpty();
  }
}

package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.CLAW_BEAM;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INDEXER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INTAKING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveAssist;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * What the guide's robot ({@link ExampleRobot}) does, and the drive team's buttons for it, written
 * as a team writes its own {@code Controls}: an intake that ends when its beam sees a piece, a
 * deploy that backs off if it doesn't arrive, an aimed shot that feeds only while the robot is
 * aimed and the shooter is at speed, and two autos.
 *
 * <p>It is beside the guide's robot so that a program running that robot can run its controls too:
 * the template gives the guide's robots these, so the buttons the guide describes work in the
 * simulator from the start. docs/guide.md walks through all of it, docs/mechanisms.md's "Where
 * behavior goes" and docs/autos.md quote it (DocExamplesTest), and ControlsSimTest runs each
 * behavior in the simulator.
 */
public final class ExampleControls {
  // @start region=constructor
  private static final Place PICKUP = Place.at("Pickup", new Pose2d(6, 3, Rotation2d.ZERO));
  private static final Place SHOT = Place.at("Shot", new Pose2d(3, 3, Rotation2d.ZERO));

  private final JointMechanism pivot, roller, shooter, feeder, indexer;
  private final Joints joints;
  private final Drive drive;
  private final Gate pieceIn;

  public ExampleControls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos) {
    this.joints = joints;
    this.drive = drive;
    pivot = joints.of(PIVOT);
    roller = joints.of(ROLLER);
    shooter = joints.of(SHOOTER);
    feeder = joints.of(FEEDER);
    indexer = joints.of(INDEXER);
    pieceIn = joints.gate("piece in", CLAW_BEAM.tripped());

    operator.rightTrigger().whileTrue(intake());
    operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));
    operator.faceDown().onTrue(deploy());
    driver.rightTrigger().whileTrue(shoot());

    autos.add("Grab one", Rebuilt.starts(robot.body()).center(), this::grabOne);
    autos.add("Grab and shoot", Rebuilt.starts(robot.body()).center(), this::grabAndShoot);
  }

  // @end

  // @start region=intake
  /**
   * Deploy, and roll until a piece is in. The interlock holds the roller until the pivot passes
   * 110°.
   */
  Command intake() {
    return Command.requiring(pivot, roller)
        .executing(
            co -> {
              co.fork(pivot.goTo(PIVOT_OUT));
              co.await(roller.runUntil(INTAKING, pieceIn).within(Seconds.of(8)));
            })
        .named("Intake");
  }

  // @end

  // @start region=deploy
  /** Deploys the intake without rolling, and brings it back in if it doesn't get there. */
  Command deploy() {
    return Command.requiring(pivot)
        .executing(
            co -> {
              var out = pivot.goTo(PIVOT_OUT);
              co.await(out);
              if (!out.arrived()) {
                co.await(pivot.goTo(PIVOT_IN)); // back off
              }
            })
        .named("Deploy");
  }

  // @end

  // @start region=shoot
  /** Aims at the hub with the sticks driving, and feeds only while aimed and the shooter is up. */
  Command shoot() {
    DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB));
    Gate ready = Gate.all("ready to feed", aim.aimed(), shooter.isAtGoal());
    return Command.requiring(drive, shooter, feeder, indexer)
        .executing(
            co -> {
              co.fork(shooter.follow(() -> RPM.of(3000)));
              co.fork(aim); // the sticks drive, and the robot holds its aim...
              while (true) {
                co.waitUntil(ready); // ...and the feed runs only while it is aimed
                co.await(joints.run(FEEDING).onlyWhile(ready));
              }
            })
        .named("Shoot");
  }

  // @end

  // @start region=auto
  /** Drives to the pickup with the intake running, then brings the intake in. */
  Command grabOne() {
    return Command.noRequirements(
            co -> {
              co.awaitAny(drive.goTo(PICKUP), intake());
              co.await(pivot.goTo(PIVOT_IN));
            })
        .named("Grab one");
  }

  // @end

  // @start region=two-legs
  /** Out to the pickup with the intake running, back to the shot, and three seconds of shooting. */
  Command grabAndShoot() {
    return Command.noRequirements(
            co -> {
              co.awaitAny(drive.goTo(PICKUP), intake()); // the first leg ends the intake
              co.fork(pivot.goTo(PIVOT_IN)); // the intake comes in on the way back
              co.await(drive.goTo(SHOT));
              co.await(shoot().withTimeout(Seconds.of(3)));
            })
        .named("Grab and shoot");
  }

  // @end
}

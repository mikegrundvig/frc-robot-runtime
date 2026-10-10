package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.INTAKING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * What the robot of docs/mechanisms.md does, and the operator's buttons for it, in the class a team
 * writes its behavior in (src/main/java/frc/robot/Controls.java), as the guide's "Where behavior
 * goes" writes it (DocExamplesTest), and its one auto, as docs/autos.md does.
 */
public final class Controls {
  // @start region=constructor
  private static final Place PICKUP = Place.at("Pickup", new Pose2d(6, 3, Rotation2d.ZERO));

  private final JointMechanism pivot, roller;
  private final Drive drive;

  Controls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos) {
    this.drive = drive;
    pivot = joints.of(PIVOT);
    roller = joints.of(ROLLER);

    operator.rightTrigger().whileTrue(intake());
    operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));

    autos.add("Grab one", Rebuilt.starts(robot.body()).center(), this::grabOne);
  }

  // @end

  /** Deploy and roll. The interlock holds the roller until the pivot passes 110°. */
  Command intake() {
    return Command.requiring(pivot, roller)
        .executing(
            co -> {
              co.fork(pivot.goTo(PIVOT_OUT));
              co.await(roller.run(INTAKING));
            })
        .named("Intake");
  }

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
}

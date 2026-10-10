package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_OUT;

import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveMove;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.path.Keyframe;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * A path through three keyframes, the intake swung out as the path passes the one named "pickup",
 * as docs/driving.md writes it (DocExamplesTest).
 */
final class PickUp {
  private final Drive drive;
  private final JointMechanism pivot;

  PickUp(Drive drive, Joints joints) {
    this.drive = drive;
    pivot = joints.of(PIVOT);
  }

  /** Drives the path, swinging the intake out as it passes the pickup, and on to its end. */
  Command pickUp() {
    return Command.requiring(drive, pivot)
        .executing(
            co -> {
              // @start region=path
              DriveMove leg =
                  drive.follow(
                      Keyframe.at(new Pose2d(3, 1, Rotation2d.ZERO)),
                      Keyframe.at(new Pose2d(5, 2, Rotation2d.CCW_90DEG)).named("pickup"),
                      Keyframe.at(new Pose2d(7, 2, Rotation2d.PI)));
              co.fork(leg); // drive the path...
              co.waitUntil(leg.passed("pickup")); // ...and as it passes pickup,
              co.await(pivot.goTo(PIVOT_OUT)); // swing the intake out
              co.await(leg); // then wait for the path's end
              // @end
            })
        .named("Pick up");
  }
}

package com.michaelgrundvig.frc.runtime.examples;

import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveMove;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.path.Keyframe;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Moves to places, written once for blue and right on both alliances, as docs/driving.md writes
 * them (DocExamplesTest).
 */
@SuppressWarnings("unused") // the moves are the example: nothing here runs them
final class PlaceMoves {
  private PlaceMoves() {}

  // @start region=places
  static final Place PICKUP = Place.at("Pickup", new Pose2d(5, 2, Rotation2d.CCW_90DEG));

  static void moves(Drive drive) {
    DriveMove toTower = drive.goTo(Rebuilt.TOWER);
    DriveMove leg = drive.follow(Keyframe.at(PICKUP), Keyframe.at(Rebuilt.TOWER));
  }

  // @end
}

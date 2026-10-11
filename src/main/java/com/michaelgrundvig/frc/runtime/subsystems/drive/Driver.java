package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.path.Keyframe;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * What the robot hands the drivetrain once, as it builds it ({@link Drive#takeDriver}): the
 * driver's sticks, and the field and alliance the robot plays on. The sticks drive the robot. The
 * field and the alliance say where a {@link Place} is, since a place is written once for blue and
 * red's is somewhere else.
 *
 * <p>Each is a supplier, read when it is needed and never kept: the sticks move, a venue can
 * replace the field, and the alliance is not known until the Driver Station or the field says.
 *
 * @param translation the driver's translation stick, turned to the field's axes
 * @param turn the driver's turn stick, counterclockwise positive
 * @param field the field the robot plays on
 * @param alliance the robot's alliance, once it is known
 */
record Driver(
    Supplier<Translation2d> translation,
    DoubleSupplier turn,
    Supplier<FieldModel> field,
    Supplier<Optional<Alliance>> alliance) {

  /**
   * Why a move to a place will not start before the Driver Station or the field names the alliance.
   */
  static final String ALLIANCE_UNKNOWN = "alliance unknown";

  /**
   * Where a place is for the robot's alliance, as of now. A move asks as it starts, so that a move
   * built before the alliance was known still goes to the right side of the field.
   *
   * @throws IllegalArgumentException saying so, if the alliance is not known yet
   */
  Pose2d found(Place place) {
    Optional<Alliance> ours = alliance.get();
    if (ours.isEmpty()) {
      throw new IllegalArgumentException(ALLIANCE_UNKNOWN);
    }
    return place.on(field.get(), ours.get());
  }

  /**
   * A path's keyframes, each at a pose. A keyframe at a place becomes one at the pose that place
   * has for the robot's alliance now, and keeps its name, or takes the place's name if it had none.
   * A keyframe already at a pose is left as it is.
   *
   * @throws IllegalArgumentException as {@link #found(Place)} does
   */
  List<Keyframe> found(List<Keyframe> keyframes) {
    List<Keyframe> atPoses = new ArrayList<>();
    for (Keyframe keyframe : keyframes) {
      Place place = keyframe.place();
      if (place == null) {
        atPoses.add(keyframe);
        continue;
      }
      String ownName = keyframe.name();
      String name = ownName != null ? ownName : place.name();
      atPoses.add(Keyframe.at(found(place)).named(name));
    }
    return atPoses;
  }
}

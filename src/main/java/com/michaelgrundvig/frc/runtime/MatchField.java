package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Placement;
import com.michaelgrundvig.frc.world.field.Zone;
import com.michaelgrundvig.frc.world.field.ZoneTrigger;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;

/**
 * The field as this match sets it, as of each loop: the robot's alliance and station, from the
 * Driver Station or the field system; the start chosen for the match, and where it is for the
 * alliance; whether the robot stands on it before the match (its placement); the zones it's in or
 * nearing; and, in the simulator, the field view. The field itself is the official layout, or the
 * venue the pit selected.
 */
final class MatchField implements AutoCloseable {
  /** How far from a start position, and how far turned from it, still counts as on it. */
  static final double START_TOLERANCE_METERS = 0.10;

  static final double START_TOLERANCE_DEGREES = 5;

  /** Zone triggers look this far ahead along the robot's path, and turn off this far outside. */
  static final double ZONE_LOOKAHEAD_SECONDS = 0.4;

  static final double ZONE_EXIT_MARGIN_METERS = 0.1;

  /** The field the robot believes it plays on: the official layout, or the venue selected. */
  private FieldModel field;

  private final Map<Zone, ZoneTrigger> zones = new LinkedHashMap<>();

  /** The autos, whose chosen one names the start. */
  private final Autos autos;

  /** The simulator's field view: the robot, its start, and the field's places. Null in a robot. */
  private final @Nullable Field2d view;

  private final Alert allianceUnknown =
      new Alert(
          "Field",
          "allianceUnknown",
          "Alliance unknown: driving assumes the blue drivers' side of the field",
          Alert.Level.MEDIUM);
  private final Alert offField =
      new Alert("Field", "offField", "Odometry puts the robot off the field", Alert.Level.MEDIUM);

  /** This loop's alliance and station (1, 2, 3), from the Driver Station or the field system. */
  private Optional<Alliance> alliance = Optional.empty();

  private OptionalInt location = OptionalInt.empty();

  /** The start chosen for the match, and where it is for the alliance, as of this loop. */
  private Optional<Place> start = Optional.empty();

  private Optional<Pose2d> startPose = Optional.empty();
  private boolean enabledSinceBoot;
  private Placement placement = new Placement(Placement.Status.ALLIANCE_UNKNOWN, null, 0, 0);

  /**
   * @param field the field the robot plays on, with its official layout
   * @param simulated whether it's the simulator, which shows the field view
   * @param autos the robot's autos, whose chosen one names the start
   */
  MatchField(FieldModel field, boolean simulated, Autos autos) {
    this.field = field;
    this.autos = autos;
    for (Zone zone : field.zones()) {
      zones.put(zone, new ZoneTrigger(zone, ZONE_LOOKAHEAD_SECONDS, ZONE_EXIT_MARGIN_METERS));
    }
    view = simulated ? new Field2d() : null;
  }

  /**
   * Reads this loop's alliance, station and chosen start, before the robot reads its sensors. The
   * robot stands where the drive team put it: on the chosen start. It's put there whenever the
   * choice changes before the match or while disabled (so changing the alliance station in the
   * simulator moves it), never while it drives.
   *
   * @return where the drive team put the robot this loop, if they did
   */
  Optional<Pose2d> read(boolean enabled) {
    alliance = MatchState.getAlliance().map(MatchField::alliance);
    location = MatchState.getLocation();
    Optional<Place> chosen = chosenStart();
    Optional<Pose2d> chosenPose = alliance.flatMap(ours -> chosen.map(s -> s.on(field, ours)));
    Optional<Pose2d> placed =
        !chosenPose.equals(startPose) && (!enabled || !enabledSinceBoot)
            ? chosenPose
            : Optional.empty();
    start = chosen;
    startPose = chosenPose;
    enabledSinceBoot |= enabled;
    return placed;
  }

  /** What localization is told the robot was placed on: "placed on Center", or "its start". */
  String placedOn() {
    return "placed on " + start.map(Place::name).orElse("its start");
  }

  /**
   * Judges where the robot is, once localization has it this loop: its placement on its start, the
   * alerts, the zones, the logs, and the field view.
   *
   * @param pose where localization puts the robot
   * @param moving how fast the robot moves, robot-relative, for the zones' look ahead
   * @param truth where the simulator really has it, for the field view; null on a robot
   */
  void update(Pose2d pose, ChassisVelocities moving, @Nullable Pose2d truth) {
    placement =
        Placement.check(
            field,
            alliance,
            pose,
            start.orElse(null),
            START_TOLERANCE_METERS,
            START_TOLERANCE_DEGREES);
    allianceUnknown.set(alliance.isEmpty());
    offField.set(!field.contains(pose.getTranslation()));
    Translation2d velocity = new Translation2d(moving.vx, moving.vy).rotateBy(pose.getRotation());
    for (ZoneTrigger trigger : zones.values()) {
      boolean active =
          alliance.isPresent()
              && trigger.update(field, alliance.get(), pose.getTranslation(), velocity);
      Logger.recordOutput("Field/Zones/" + trigger.zone().name(), active);
    }
    show(pose, truth);
  }

  /** Logs the robot's place on the field, and shows it in the simulator's field view. */
  private void show(Pose2d pose, @Nullable Pose2d truth) {
    Logger.recordOutput("Field/Alliance", allianceStation());
    Logger.recordOutput("Field/Start", startPose.stream().toArray(Pose2d[]::new));
    Logger.recordOutput("Field/Placement", placement.describe());
    for (Place place : field.places()) {
      Logger.recordOutput(
          "Field/Places/" + place.name(),
          alliance.stream().map(ours -> place.on(field, ours)).toArray(Pose2d[]::new));
    }
    Field2d shown = view;
    if (shown != null) {
      shown.setRobotPose(pose);
      if (truth != null) {
        shown.getObject("Truth").setPose(truth);
      }
      shown.getObject("Start").setPoses(startPose.stream().toList());
      for (Place place : field.places()) {
        shown
            .getObject(place.name())
            .setPoses(alliance.stream().map(ours -> place.on(field, ours)).toList());
      }
      Telemetry.log("Field", shown);
    }
  }

  /**
   * The start chosen for the match: the chosen auto's own, unless it's "Do nothing", which starts
   * where the Driver Station's location says: stations 1, 2, and 3 choose the field's first,
   * second, and third start.
   */
  private Optional<Place> chosenStart() {
    Place autoStart = autos.chosen().start();
    if (autoStart != null) {
      return Optional.of(autoStart);
    }
    List<Place> starts = field.starts();
    if (location.isEmpty() || location.getAsInt() < 1 || location.getAsInt() > starts.size()) {
      return Optional.empty();
    }
    return Optional.of(starts.get(location.getAsInt() - 1));
  }

  /**
   * The Driver Station's alliance, as the field model names it: the model has its own, so that
   * nothing reading it needs WPILib's Driver Station.
   */
  static Alliance alliance(org.wpilib.driverstation.Alliance driverStation) {
    return switch (driverStation) {
      case BLUE -> Alliance.BLUE;
      case RED -> Alliance.RED;
    };
  }

  /** Plays on a venue from now on: the places derived from tags move with them. */
  void venue(FieldModel venue) {
    field = venue;
  }

  /** The field the robot believes it plays on: the official layout, or the venue selected. */
  FieldModel field() {
    return field;
  }

  /** This loop's alliance, once the Driver Station or the field says. */
  Optional<Alliance> alliance() {
    return alliance;
  }

  /**
   * The way the robot's drivers face: the field's forward for their alliance, blue's until known.
   */
  Rotation2d driverForward() {
    return FieldModel.driverForward(alliance.orElse(Alliance.BLUE));
  }

  /** The alliance and station, such as "Red 2", or "Unknown". */
  String allianceStation() {
    if (alliance.isEmpty()) {
      return "Unknown";
    }
    String name = alliance.get() == Alliance.RED ? "Red" : "Blue";
    return location.isPresent() ? name + " " + location.getAsInt() : name;
  }

  /** The pre-match placement check, as of the last loop. */
  Placement placement() {
    return placement;
  }

  /**
   * Whether a zone's trigger is on: the robot is in the zone, or will be within {@link
   * #ZONE_LOOKAHEAD_SECONDS}. Bind mechanisms to it, such as stowing before a low bar.
   */
  boolean inZone(Zone zone) {
    ZoneTrigger trigger = zones.get(zone);
    return alliance.isPresent() && trigger != null && trigger.isActive();
  }

  @Override
  public void close() {
    allianceUnknown.close();
    offField.close();
    Field2d shown = view;
    if (shown != null) {
      shown.close();
    }
  }
}

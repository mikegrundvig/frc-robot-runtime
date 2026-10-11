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
 * The field as this match sets it, kept up to date each loop:
 *
 * <ul>
 *   <li>the robot's alliance and station, from the Driver Station or the field system;
 *   <li>the start chosen for the match, and where that start is for the alliance;
 *   <li>whether the robot stands on its start before the match (its {@link Placement});
 *   <li>the zones the robot is in or is nearing;
 *   <li>in the simulator, the field view.
 * </ul>
 *
 * <p>The field itself is the official layout, or the venue the pit selected.
 *
 * <p>A loop uses this in two steps. Before the robot reads its sensors, {@link #read} takes the
 * alliance, the station and the chosen start. Once localization has the robot's pose, {@link
 * #update} judges where the robot is.
 */
final class MatchField implements AutoCloseable {
  /** How far from its start the robot may stand and still count as on it. */
  static final double START_TOLERANCE_METERS = 0.10;

  /** How far the robot may be turned from its start's heading and still count as on it. */
  static final double START_TOLERANCE_DEGREES = 5;

  /** How far ahead along the robot's path a zone's trigger looks. */
  static final double ZONE_LOOKAHEAD_SECONDS = 0.4;

  /** How far outside a zone the robot must be before the zone's trigger turns off. */
  static final double ZONE_EXIT_MARGIN_METERS = 0.1;

  /** The field the robot believes it plays on: the official layout, or the venue selected. */
  private FieldModel field;

  private final Map<Zone, ZoneTrigger> zones = new LinkedHashMap<>();

  /** The robot's autos. The chosen one names the start. */
  private final Autos autos;

  /**
   * The simulator's field view, which shows the robot, its start and the field's places. A real
   * robot has none.
   */
  private final Optional<Field2d> view;

  private final Alert allianceUnknown =
      new Alert(
          "Field",
          "allianceUnknown",
          "Alliance unknown: driving assumes the blue drivers' side of the field",
          Alert.Level.MEDIUM);
  private final Alert offField =
      new Alert("Field", "offField", "Odometry puts the robot off the field", Alert.Level.MEDIUM);

  // As of this loop.

  /** The robot's alliance, once the Driver Station or the field system says. */
  private Optional<Alliance> alliance = Optional.empty();

  /** The robot's station on its alliance's wall: 1, 2 or 3. */
  private OptionalInt station = OptionalInt.empty();

  /** The start chosen for the match. */
  private Optional<Place> start = Optional.empty();

  /** Where the chosen start is on the field for the robot's alliance. */
  private Optional<Pose2d> startPose = Optional.empty();

  /** Whether the robot has been enabled since the program started. */
  private boolean hasBeenEnabled;

  private Placement placement = new Placement(Placement.Status.ALLIANCE_UNKNOWN, null, 0, 0);

  /**
   * @param field the field the robot plays on, with its official layout
   * @param mode what the program runs as. Only the simulator shows the field view.
   * @param autos the robot's autos, whose chosen one names the start
   */
  MatchField(FieldModel field, RunMode mode, Autos autos) {
    this.field = field;
    this.autos = autos;
    for (Zone zone : field.zones()) {
      zones.put(zone, new ZoneTrigger(zone, ZONE_LOOKAHEAD_SECONDS, ZONE_EXIT_MARGIN_METERS));
    }
    view = mode == RunMode.SIM ? Optional.of(new Field2d()) : Optional.empty();
  }

  // ---- Each loop, before the sensors are read ----

  /**
   * Reads this loop's alliance, station and chosen start, and says whether the drive team has put
   * the robot on that start.
   *
   * <p>The robot is taken to stand where the drive team put it, which is on the chosen start. So
   * whenever where the start is changes, the robot is put there, as long as that happens before the
   * robot is first enabled or while it is disabled. That is what moves the simulated robot when the
   * alliance station is changed in the simulator. The robot is never moved while it drives.
   *
   * @return where the drive team put the robot this loop, if they did
   */
  Optional<Pose2d> read(boolean enabled) {
    alliance = MatchState.getAlliance().map(MatchField::alliance);
    station = MatchState.getLocation();
    Optional<Place> chosen = chosenStart();
    Optional<Pose2d> chosenPose = alliance.flatMap(ours -> chosen.map(s -> s.on(field, ours)));

    boolean startMoved = !chosenPose.equals(startPose);
    boolean mayBePlaced = !enabled || !hasBeenEnabled;
    start = chosen;
    startPose = chosenPose;
    hasBeenEnabled |= enabled;
    if (startMoved && mayBePlaced) {
      return chosenPose;
    }
    return Optional.empty();
  }

  /**
   * The start chosen for the match. It is the chosen auto's own start. "Do nothing" has none, and
   * then the Driver Station's station chooses: stations 1, 2 and 3 are the field's first, second
   * and third start.
   */
  private Optional<Place> chosenStart() {
    Optional<Place> autosStart = autos.chosen().start();
    if (autosStart.isPresent()) {
      return autosStart;
    }
    if (station.isEmpty()) {
      return Optional.empty();
    }
    List<Place> starts = field.starts();
    int number = station.getAsInt();
    boolean hasAStart = number >= 1 && number <= starts.size();
    if (!hasAStart) {
      return Optional.empty();
    }
    return Optional.of(starts.get(number - 1));
  }

  /**
   * What localization is told the robot was placed on: "placed on Center", or "placed on its
   * start".
   */
  String placedOn() {
    return "placed on " + start.map(Place::name).orElse("its start");
  }

  // ---- Each loop, once localization has the pose ----

  /**
   * Judges where the robot is: its placement on its start, the alerts, and the zones. Then logs all
   * of it, and shows it in the simulator's field view.
   *
   * @param pose where localization puts the robot
   * @param moving how fast the robot moves, relative to itself, for the zones to look ahead with
   * @param truth where the simulator really has the robot, for the field view. A real robot has no
   *     such thing.
   */
  void update(Pose2d pose, ChassisVelocities moving, Optional<Pose2d> truth) {
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
    updateZones(pose, moving);
    log();
    if (view.isPresent()) {
      show(view.get(), pose, truth);
    }
  }

  /** Works out, and logs, whether the robot is in or nearing each zone. */
  private void updateZones(Pose2d pose, ChassisVelocities moving) {
    // The zones look ahead along the robot's path on the field, so its velocity is turned from
    // the robot's axes to the field's.
    Translation2d velocity = new Translation2d(moving.vx, moving.vy).rotateBy(pose.getRotation());
    for (ZoneTrigger trigger : zones.values()) {
      // A zone is written for one alliance, so it means nothing until the alliance is known.
      boolean active =
          alliance.isPresent()
              && trigger.update(field, alliance.get(), pose.getTranslation(), velocity);
      Logger.recordOutput("Field/Zones/" + trigger.zone().name(), active);
    }
  }

  /** Logs the alliance, the start, the placement, and each of the field's places. */
  private void log() {
    Logger.recordOutput("Field/Alliance", allianceStation());
    Logger.recordOutput("Field/Start", startPose.stream().toArray(Pose2d[]::new));
    Logger.recordOutput("Field/Placement", placement.describe());
    for (Place place : field.places()) {
      Logger.recordOutput(
          "Field/Places/" + place.name(),
          alliance.stream().map(ours -> place.on(field, ours)).toArray(Pose2d[]::new));
    }
  }

  /** Shows the robot, the truth, the start and the field's places in the simulator's view. */
  private void show(Field2d shown, Pose2d pose, Optional<Pose2d> truth) {
    shown.setRobotPose(pose);
    if (truth.isPresent()) {
      shown.getObject("Truth").setPose(truth.get());
    }
    shown.getObject("Start").setPoses(startPose.stream().toList());
    for (Place place : field.places()) {
      shown
          .getObject(place.name())
          .setPoses(alliance.stream().map(ours -> place.on(field, ours)).toList());
    }
    Telemetry.log("Field", shown);
  }

  // ---- What the rest of the robot asks ----

  /**
   * The Driver Station's alliance, as the field model names it. The model has an alliance type of
   * its own, so that nothing which reads the model needs WPILib's Driver Station.
   */
  static Alliance alliance(org.wpilib.driverstation.Alliance driverStation) {
    return switch (driverStation) {
      case BLUE -> Alliance.BLUE;
      case RED -> Alliance.RED;
    };
  }

  /**
   * Plays on a venue from now on. The places that are derived from tags move with the venue's tags.
   */
  void venue(FieldModel venue) {
    field = venue;
  }

  /** The field the robot believes it plays on: the official layout, or the venue selected. */
  FieldModel field() {
    return field;
  }

  /** This loop's alliance, once the Driver Station or the field system says. */
  Optional<Alliance> alliance() {
    return alliance;
  }

  /**
   * The way the robot's drivers face: the field's forward for their alliance. Until the alliance is
   * known, it is blue's.
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
    if (station.isEmpty()) {
      return name;
    }
    return name + " " + station.getAsInt();
  }

  /** The check of the robot's placement before the match, as of the last loop. */
  Placement placement() {
    return placement;
  }

  /**
   * Whether a zone's trigger is on: the robot is in the zone, or will be within {@link
   * #ZONE_LOOKAHEAD_SECONDS}. Mechanisms are bound to it, for example to stow before a low bar.
   */
  boolean inZone(Zone zone) {
    ZoneTrigger trigger = zones.get(zone);
    return alliance.isPresent() && trigger != null && trigger.isActive();
  }

  @Override
  public void close() {
    allianceUnknown.close();
    offField.close();
    if (view.isPresent()) {
      view.get().close();
    }
  }
}

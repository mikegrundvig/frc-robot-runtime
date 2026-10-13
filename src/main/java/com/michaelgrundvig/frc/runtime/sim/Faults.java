package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.simulation.FaultCatalog.BACKUP_IMU;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.DRIFT;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.EXTRA;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.FIELD;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.FORWARD;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.GRIP;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.GYRO;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.HEADING;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.LEFT;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.MODULES;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.NOISE;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.OFFSET;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.PITCH;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.ROBOT;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.ROLL;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.SCALE;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.SPEED;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.TAG;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.TURN;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.TURNS;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.UP;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.X;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.Y;
import static com.michaelgrundvig.frc.simulation.FaultCatalog.YAW;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.simulator.Component;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Fault;
import com.michaelgrundvig.frc.pitwall.simulator.Values;
import com.michaelgrundvig.frc.pitwall.world.SimulatedParts;
import com.michaelgrundvig.frc.runtime.subsystems.vision.VisionSim;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Crossing;
import com.michaelgrundvig.frc.world.field.Surface;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The simulated world's faults, declared to Pitwall's simulator controls: each fault in the
 * simulation's catalog ({@link FaultCatalog}, which says what each is and what it takes) bound to
 * the part it fails, here, where the parts are: each camera, the gyro, the backup IMU, each module,
 * the robot itself, and the field's tags. The page lists them, applies them, and undoes them; the
 * scenarios ({@link Scenarios}) are made of them. The robot's code is never told: it has to find
 * out from its sensors.
 *
 * <p>Whether a fault lasts or happens once is the catalog's to say ({@link FaultCatalog.Lifetime}):
 * a binding here that was written for the other is refused as the world is built.
 *
 * <p>A fault that lasts (a camera unplugged) says whether it's on by asking the world, and undoing
 * it puts the part back as the world was built: a season-worn robot's sensors ({@link
 * SensorErrors}), not perfect ones. An event (a hit) happens once; nothing undoes it, and each says
 * how the robot recovers. What the world can't make fail yet is listed in docs/simulator.md.
 */
public final class Faults {
  /** How far short of a bump's ramp the robot is set down, from its center: clear of the ramp. */
  static final double BUMP_CLEARANCE_METERS = 1.0;

  /**
   * How far a module's absolute encoder's zero may sit from where it was built and still count as
   * not knocked off, in radians: far below any offset a fault sets, and above the rounding of a
   * value that went to degrees and back.
   */
  private static final double SAME_ZERO_RADIANS = 1e-9;

  private Faults() {}

  /** Declares every part of the world that can fail. */
  static void declare(SimulatedWorld world, Components components) {
    for (int i = 0; i < world.cameraCount(); i++) {
      components.add(camera(world.camera(i)));
    }
    DriveSimulation drivetrain = world.drivetrain();
    components.add(imu(GYRO, FaultCatalog.Imu.GYRO_KIND, drivetrain.gyroSim()));
    components.add(imu(BACKUP_IMU, FaultCatalog.Imu.BACKUP_KIND, drivetrain.backupGyroSim()));
    for (int i = 0; i < MODULES.size(); i++) {
      components.add(module(drivetrain, i));
    }
    components.add(robot(world));
    if (!world.vision().tagIds().isEmpty()) {
      components.add(field(world.vision()));
    }
  }

  /**
   * Which of these parts each part of the robot is, by the names they are declared under here:
   * Pitwall's World view offers a part's faults where the part is clicked.
   */
  public static SimulatedParts parts() {
    return new SimulatedParts() {
      @Override
      public Optional<String> camera(String name) {
        return Optional.of(FaultCatalog.camera(name));
      }

      @Override
      public Optional<String> gyro() {
        return Optional.of(GYRO);
      }

      @Override
      public Optional<String> backupImu() {
        return Optional.of(BACKUP_IMU);
      }

      @Override
      public Optional<String> module(int index) {
        if (index < 0 || index >= MODULES.size()) {
          return Optional.empty();
        }
        return Optional.of(FaultCatalog.module(index));
      }

      @Override
      public Optional<String> robot() {
        return Optional.of(ROBOT);
      }
    };
  }

  /**
   * A fault that lasts, as the catalog has it, bound to its part: how the world says it is on, how
   * it is applied, and how it is undone.
   *
   * @throws IllegalArgumentException for a fault the catalog says happens once
   */
  private static Fault lasting(
      FaultCatalog.Fault fault, BooleanSupplier isOn, Consumer<Values> apply, Runnable undo) {
    return declared(fault, FaultCatalog.Lifetime.LASTS_UNTIL_UNDONE)
        .isOn(isOn)
        .apply(apply)
        .undo(fault.undone(), undo)
        .build();
  }

  /**
   * A fault that happens once, as the catalog has it, bound to its part: how it is applied.
   *
   * @throws IllegalArgumentException for a fault the catalog says lasts until it is undone
   */
  private static Fault event(FaultCatalog.Fault fault, Consumer<Values> apply) {
    return declared(fault, FaultCatalog.Lifetime.HAPPENS_ONCE)
        .apply(apply)
        .undoneBy(fault.undone())
        .build();
  }

  /**
   * Starts a fault's declaration from the catalog: its name, what it does, its parameters, and
   * whether it lasts or happens once. Only the catalog says which ({@link FaultCatalog.Lifetime}).
   * A binding is written for one or the other, with a way to undo the fault or without, so it says
   * here which it was written for, and one written for the other is refused as the world is built.
   *
   * @param bindsItAs the lifetime the binding was written for
   * @throws IllegalArgumentException if the catalog gives the fault the other lifetime
   */
  static Fault.Builder declared(FaultCatalog.Fault fault, FaultCatalog.Lifetime bindsItAs) {
    FaultCatalog.Lifetime lifetime = fault.lifetime();
    if (lifetime != bindsItAs) {
      throw new IllegalArgumentException(whyBoundWrongly(fault));
    }
    Fault.Builder builder =
        switch (lifetime) {
          case LASTS_UNTIL_UNDONE -> Fault.lasting(fault.name());
          case HAPPENS_ONCE -> Fault.event(fault.name());
        };
    builder.does(fault.does());
    for (FaultCatalog.Parameter parameter : fault.parameters()) {
      builder.parameter(
          parameter.name(),
          parameter.unit(),
          Limits.between(parameter.min(), parameter.max()),
          parameter.initial());
    }
    return builder;
  }

  /** What a fault bound as the other lifetime than the catalog's is refused with. */
  private static String whyBoundWrongly(FaultCatalog.Fault fault) {
    String name = "the fault catalog's \"" + fault.name() + "\" ";
    return switch (fault.lifetime()) {
      case LASTS_UNTIL_UNDONE ->
          name
              + "lasts until it is undone, but it was bound as one that happens once: bind it"
              + " with lasting(), saying how it is undone, or change its lifetime in FaultCatalog";
      case HAPPENS_ONCE ->
          name
              + "happens once, but it was bound as one that lasts: bind it with event(), or"
              + " change its lifetime in FaultCatalog";
    };
  }

  private static Component camera(CameraIOSim camera) {
    return Component.of(
        FaultCatalog.camera(camera.name()),
        FaultCatalog.Camera.KIND,
        lasting(
            FaultCatalog.Camera.UNPLUGGED,
            camera::isUnplugged,
            values -> camera.unplug(true),
            () -> camera.unplug(false)),
        lasting(
            FaultCatalog.Camera.FROZEN,
            camera::isFrozen,
            values -> camera.freeze(true),
            () -> camera.freeze(false)),
        lasting(
            FaultCatalog.Camera.MOVED,
            camera::isMoved,
            values ->
                camera.move(
                    new Translation3d(
                        Inches.of(values.get(FORWARD)),
                        Inches.of(values.get(LEFT)),
                        Inches.of(values.get(UP)))),
            () -> camera.move(Translation3d.ZERO)),
        lasting(
            FaultCatalog.Camera.TURNED,
            camera::isTurned,
            values ->
                camera.turn(
                    new Rotation3d(
                        Degrees.of(values.get(ROLL)),
                        Degrees.of(values.get(PITCH)),
                        Degrees.of(values.get(YAW)))),
            () -> camera.turn(Rotation3d.ZERO)),
        lasting(
            FaultCatalog.Camera.LATE,
            camera::isLate,
            values -> camera.delay(values.get(EXTRA)),
            () -> camera.delay(0)),
        lasting(
            FaultCatalog.Camera.CLOCK_OFFSET,
            camera::hasClockOffset,
            values -> camera.offsetClock(values.get(OFFSET)),
            () -> camera.offsetClock(0)));
  }

  /** The gyro, or the backup IMU: they fail alike. */
  private static Component imu(String name, String kind, GyroIOSim imu) {
    SensorErrors.GyroErrors built = imu.built();
    return Component.of(
        name,
        kind,
        lasting(
            FaultCatalog.Imu.UNPLUGGED,
            () -> !imu.isConnected(),
            values -> imu.setConnected(false),
            () -> imu.setConnected(true)),
        lasting(
            FaultCatalog.Imu.FROZEN,
            imu::isFrozen,
            values -> imu.setFrozen(true),
            () -> imu.setFrozen(false)),
        lasting(
            FaultCatalog.Imu.errors(built),
            () -> !imu.errors().equals(built),
            values ->
                imu.setErrors(
                    SensorErrors.GyroErrors.builder()
                        .scale(values.get(SCALE))
                        .driftDegreesPerMinute(values.get(DRIFT))
                        .noiseDegrees(values.get(NOISE))
                        .build()),
            () -> imu.setErrors(built)),
        event(FaultCatalog.Imu.RESET, values -> imu.reset()));
  }

  private static Component module(DriveSimulation drivetrain, int index) {
    ModuleIOSim module = drivetrain.moduleSim(index);
    double builtOffset = Math.toDegrees(module.builtSteerOffsetRadians());
    double builtTread = drivetrain.builtTreadScale(index);
    return Component.of(
        FaultCatalog.module(index),
        FaultCatalog.Module.KIND,
        lasting(
            FaultCatalog.Module.STALE,
            module::isStale,
            values -> module.setStale(true),
            () -> module.setStale(false)),
        lasting(
            FaultCatalog.Module.zeroedOff(builtOffset),
            () ->
                Math.abs(module.steerOffsetRadians() - module.builtSteerOffsetRadians())
                    > SAME_ZERO_RADIANS,
            values -> module.setSteerOffset(Math.toRadians(values.get(OFFSET))),
            () -> module.setSteerOffset(module.builtSteerOffsetRadians())),
        lasting(
            FaultCatalog.Module.worn(builtTread),
            () -> drivetrain.treadScale(index) != builtTread,
            values -> drivetrain.wearTread(index, values.get(SCALE)),
            () -> drivetrain.wearTread(index, builtTread)),
        event(
            FaultCatalog.Module.SLIPS,
            values -> drivetrain.spin(index, 2 * Math.PI * values.get(TURNS))),
        lasting(
            FaultCatalog.Module.SPINNING,
            () -> module.spinMetersPerSecond() != 0,
            values -> drivetrain.spinAt(index, values.get(SPEED)),
            () -> drivetrain.spinAt(index, 0)));
  }

  private static Component robot(SimulatedWorld world) {
    DriveSimulation drivetrain = world.drivetrain();
    List<Fault> faults = new ArrayList<>();
    faults.add(
        event(
            FaultCatalog.Robot.SHOVED,
            values ->
                drivetrain.shove(
                    new Transform2d(
                        values.get(FORWARD),
                        values.get(LEFT),
                        Rotation2d.fromDegrees(values.get(TURN))))));
    faults.add(
        lasting(
            FaultCatalog.Robot.PUSHED,
            drivetrain::isPushed,
            values -> drivetrain.push(new Translation2d(values.get(FORWARD), values.get(LEFT))),
            () -> drivetrain.push(Translation2d.ZERO)));
    faults.add(
        event(
            FaultCatalog.Robot.HIT,
            values ->
                drivetrain.hit(
                    new Transform2d(values.get(FORWARD), values.get(LEFT), Rotation2d.ZERO))));
    faults.add(
        lasting(
            FaultCatalog.Robot.TILTED,
            drivetrain::isTiltedByHand,
            values ->
                drivetrain.tilt(
                    Math.toRadians(values.get(PITCH)), Math.toRadians(values.get(ROLL))),
            () -> drivetrain.setTerrain(drivetrain.terrain())));
    faults.add(
        lasting(
            FaultCatalog.Robot.SLIPPING,
            () -> drivetrain.grip() < 1,
            values -> drivetrain.grip(values.get(GRIP)),
            () -> drivetrain.grip(1)));
    // Its X and Y are a place on the field, which Pitwall's World view can take from a click.
    FaultCatalog.Fault placed =
        FaultCatalog.Robot.placed(world.field().length(), world.field().width());
    faults.add(
        declared(placed, FaultCatalog.Lifetime.HAPPENS_ONCE)
            .point(X, Y)
            .apply(values -> world.place(placed(values)))
            .undoneBy(placed.undone())
            .build());
    if (hasBumps(world)) {
      faults.add(event(FaultCatalog.Robot.TO_A_BUMP, values -> beforeTheNearestBump(world)));
    }
    return new Component(ROBOT, FaultCatalog.Robot.KIND, faults);
  }

  /** The field's tags: one can be knocked off where the field was built. */
  private static Component field(VisionSim vision) {
    List<Integer> ids = vision.tagIds();
    int lowest = ids.stream().mapToInt(Integer::intValue).min().orElseThrow();
    int highest = ids.stream().mapToInt(Integer::intValue).max().orElseThrow();
    return Component.of(
        FIELD,
        FaultCatalog.Field.KIND,
        lasting(
            FaultCatalog.Field.tagMoved(lowest, highest, ids.get(0)),
            vision::isTagMoved,
            values ->
                vision.moveTag(
                    tagId(values.get(TAG)),
                    new Translation3d(
                        Inches.of(values.get(X)),
                        Inches.of(values.get(Y)),
                        Inches.of(values.get(UP)))),
            vision::putTagsBack));
  }

  /** A tag's ID, given as a number. */
  private static int tagId(double id) {
    if (id != Math.rint(id)) {
      throw new IllegalArgumentException("a tag's ID is a whole number, not " + id);
    }
    return (int) id;
  }

  /** Whether the field the world was built on has bumps (crests), to carry the robot to. */
  static boolean hasBumps(SimulatedWorld world) {
    return !crests(world).isEmpty();
  }

  private static Pose2d placed(Values values) {
    return new Pose2d(values.get(X), values.get(Y), Rotation2d.fromDegrees(values.get(HEADING)));
  }

  /**
   * Puts the robot down short of the bump nearest it, on the side it's on, facing across the crest:
   * the drive team carrying it there. Its wheels don't turn, so odometry doesn't see the move; its
   * gyro feels any turn. A bump is any surface's crest, where the ground falls away on both sides.
   *
   * @throws IllegalStateException on a field without bumps
   */
  static void beforeTheNearestBump(SimulatedWorld world) {
    Translation2d robot = world.truePose().getTranslation();
    Crest nearest =
        crests(world).stream()
            .min(Comparator.comparingDouble(each -> each.line().middle().getDistance(robot)))
            .orElseThrow(
                () ->
                    new IllegalStateException(
                        "carrying the robot to a bump needs a field with bumps, and this one has"
                            + " none"));
    Crossing crest = nearest.line();
    // A crest has a direction across it. The robot is on the side that direction points to, or on
    // the other, and it stays on its own side.
    Rotation2d across = crest.across();
    Rotation2d backAcross = across.plus(Rotation2d.PI);
    boolean onTheAcrossSide = crest.acrossOffset(robot) >= 0;
    Rotation2d fromTheCrestToTheRobot = onTheAcrossSide ? across : backAcross;
    Rotation2d facingTheCrest = onTheAcrossSide ? backAcross : across;
    // From the crest's middle to its surface's edge on the robot's side, then clear of it.
    double toTheEdgeMeters =
        nearest.surface().footprint().reach(crest.middle(), fromTheCrestToTheRobot);
    double outMeters = toTheEdgeMeters + BUMP_CLEARANCE_METERS;
    double alongAcrossMeters = onTheAcrossSide ? outMeters : -outMeters;
    Translation2d spot = crest.middle().plus(new Translation2d(alongAcrossMeters, across));
    world.place(new Pose2d(spot, facingTheCrest));
  }

  /** A crest, and the surface it's on. */
  private record Crest(Surface surface, Crossing line) {}

  /** Every crest on the field the world was built on. */
  private static List<Crest> crests(SimulatedWorld world) {
    List<Crest> crests = new ArrayList<>();
    for (Surface surface : world.drivetrain().terrain().surfaces()) {
      for (Crossing crossing : surface.crossings()) {
        if (crossing.kind() == Crossing.Kind.CREST) {
          crests.add(new Crest(surface, crossing));
        }
      }
    }
    return crests;
  }
}

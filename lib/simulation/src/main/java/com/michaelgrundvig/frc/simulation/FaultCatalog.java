package com.michaelgrundvig.frc.simulation;

import java.util.List;

/**
 * What can go wrong with the simulated world's parts, as data: each part's name and kind, and each
 * fault's name, whether it lasts or happens once, what it does, the numbers it takes, and how it's
 * undone. The robot program's simulator controls ({@code frc.robot.sim.Faults}) bind each fault to
 * the part it fails and declare it to Pitwall; the scenarios name them; docs/simulator.md lists
 * them. Where a fault's numbers depend on the world (the field's size, how a part was built), its
 * part's factory method takes what they depend on.
 *
 * <p>A fault that lasts (a camera unplugged) can be undone, and undoing it puts the part back as
 * the world was built. An event (a hit) happens once; nothing undoes it, and its {@link
 * Fault#undone} says how the robot recovers.
 */
public final class FaultCatalog {
  private FaultCatalog() {}

  // The parts.
  public static final String GYRO = "Gyro";
  public static final String BACKUP_IMU = "Backup IMU";
  public static final String ROBOT = "Robot";
  public static final String FIELD = "Field";

  /** The modules' names, front left first, in the drivetrain's order. */
  public static final List<String> MODULES =
      List.of("Front left module", "Front right module", "Back left module", "Back right module");

  // What can go wrong with them.
  public static final String UNPLUGGED = "Unplugged";
  public static final String FROZEN = "Frozen";
  public static final String MOVED = "Moved on its mount";
  public static final String TURNED = "Turned on its mount";
  public static final String LATE = "Late";
  public static final String CLOCK_OFFSET = "Clock offset";
  public static final String ERRORS = "Errors";
  public static final String RESET = "Reset";
  public static final String STALE = "Stale encoder";
  public static final String ZEROED_OFF = "Zeroed off";
  public static final String WORN = "Worn tread";
  public static final String SLIPS = "Slips";
  public static final String SPINNING = "Spinning";
  public static final String SHOVED = "Shoved";
  public static final String PUSHED = "Pushed";
  public static final String HIT = "Hit";
  public static final String TILTED = "Tilted";
  public static final String SLIPPING = "Slipping";
  public static final String PLACED = "Placed";
  public static final String TO_A_BUMP = "Carried to a bump";
  public static final String TAG_MOVED = "Tag moved";

  // Their parameters, as the scenarios give them.
  public static final String FORWARD = "Forward";
  public static final String LEFT = "Left";
  public static final String UP = "Up";
  public static final String YAW = "Yaw";
  public static final String PITCH = "Pitch";
  public static final String ROLL = "Roll";
  public static final String SCALE = "Scale";
  public static final String DRIFT = "Drift";
  public static final String NOISE = "Noise";
  public static final String OFFSET = "Offset";
  public static final String TURNS = "Turns";
  public static final String GRIP = "Grip";
  public static final String SPEED = "Speed";
  public static final String EXTRA = "Extra";
  public static final String TURN = "Turn";
  public static final String X = "X";
  public static final String Y = "Y";
  public static final String HEADING = "Heading";
  public static final String TAG = "Tag";

  /** The farthest a camera may be knocked, or a tag moved, in one go, in inches. */
  private static final double MOUNT_INCHES = 12;

  /** The farthest a camera may be turned, or the robot tilted, in one go, in degrees. */
  private static final double MOUNT_DEGREES = 30;

  /** The farthest the robot may be shoved in one go, in meters. */
  private static final double SHOVE_METERS = 3;

  /** The fastest a wheel may spin past the floor, or the robot be pushed, in m/s. */
  private static final double SLIDE_METERS_PER_SECOND = 3;

  /** The farthest a camera's clock may be off the robot's, either way, in ms. */
  private static final double CLOCK_OFFSET_MILLIS = 100;

  private static final String RECOVERS =
      "Nothing undoes it: place the robot again, or let its cameras find it";

  /** A camera's name: {@code "Camera front-left"}. */
  public static String camera(String name) {
    return "Camera " + name;
  }

  /**
   * A module's name: {@code "Front left module"}.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public static String module(int module) {
    return MODULES.get(module);
  }

  /**
   * One thing that can go wrong with a part.
   *
   * @param name what it's called, on the page and in the scenarios
   * @param lasting whether it lasts until undone; otherwise it happens once
   * @param does what it does to the part, for the page
   * @param parameters the numbers it takes, in the order the page shows them
   * @param undone for a fault that lasts, what undoing it does; for one that happens once, how the
   *     robot recovers, since nothing undoes it
   */
  public record Fault(
      String name, boolean lasting, String does, List<Parameter> parameters, String undone) {
    public Fault {
      parameters = List.copyOf(parameters);
    }

    /** The parameter with this name. */
    public Parameter parameter(String name) {
      return parameters.stream()
          .filter(parameter -> parameter.name().equals(name))
          .findFirst()
          .orElseThrow(() -> new IllegalArgumentException(this.name + " takes no " + name));
    }
  }

  /**
   * A number a fault takes.
   *
   * @param name what it's called
   * @param unit its unit, as the page shows it; empty for none
   * @param min the least it may be
   * @param max the most it may be
   * @param initial what the page offers first
   */
  public record Parameter(String name, String unit, double min, double max, double initial) {
    public Parameter {
      if (!(min <= initial && initial <= max)) {
        throw new IllegalArgumentException(
            name + ": " + initial + " must be between " + min + " and " + max);
      }
    }

    /**
     * The same, its range widened to hold what the part was built with, however far out that is,
     * and starting there.
     */
    public Parameter around(double built) {
      return new Parameter(name, unit, Math.min(min, built), Math.max(max, built), built);
    }
  }

  private static Fault lasting(String name, String does, String undone, Parameter... parameters) {
    return new Fault(name, true, does, List.of(parameters), undone);
  }

  private static Fault event(String name, String does, String undone, Parameter... parameters) {
    return new Fault(name, false, does, List.of(parameters), undone);
  }

  private static Parameter parameter(
      String name, String unit, double min, double max, double initial) {
    return new Parameter(name, unit, min, max, initial);
  }

  /** A camera's faults. */
  public static final class Camera {
    private Camera() {}

    /** The part's kind, on the page. */
    public static final String KIND = "camera";

    public static final Fault UNPLUGGED =
        lasting(
            FaultCatalog.UNPLUGGED,
            "No results at all: the robot's code sees it disconnected.",
            "Plugs it back in.");

    public static final Fault FROZEN =
        lasting(
            FaultCatalog.FROZEN,
            "Keeps sending its last image's tags as new ones, at its frame rate: to the robot's"
                + " code, the robot seems not to move.",
            "Sends new images again.");

    public static final Fault MOVED =
        lasting(
            FaultCatalog.MOVED,
            "Knocked this far from where it's mounted, in the robot's frame (forward, left, up);"
                + " the robot's code still believes its configured mount.",
            "Puts it back where it was mounted.",
            parameter(FORWARD, "in", -MOUNT_INCHES, MOUNT_INCHES, 0),
            parameter(LEFT, "in", -MOUNT_INCHES, MOUNT_INCHES, 0),
            parameter(UP, "in", -MOUNT_INCHES, MOUNT_INCHES, -4));

    public static final Fault TURNED =
        lasting(
            FaultCatalog.TURNED,
            "Turned this far from how it's mounted, in the robot's frame; the robot's code still"
                + " believes its configured mount.",
            "Turns it back as it was mounted.",
            parameter(ROLL, "°", -MOUNT_DEGREES, MOUNT_DEGREES, 0),
            parameter(PITCH, "°", -MOUNT_DEGREES, MOUNT_DEGREES, 0),
            parameter(YAW, "°", -MOUNT_DEGREES, MOUNT_DEGREES, 2));

    public static final Fault LATE =
        lasting(
            FaultCatalog.LATE,
            "Its images arrive this much later than its configuration says, still carrying when"
                + " they were taken; past its frame time, it sends fewer of them.",
            "Back to its usual latency.",
            parameter(EXTRA, "ms", 0, 1000, 100));

    public static final Fault CLOCK_OFFSET =
        lasting(
            FaultCatalog.CLOCK_OFFSET,
            "Its clock is this far off the robot's, as a coprocessor whose time sync failed: each"
                + " image arrives on time, but is stamped this much later than it was taken"
                + " (earlier, if negative). Unlike Late, the stamps are wrong.",
            "Its clock agrees with the robot's again.",
            parameter(OFFSET, "ms", -CLOCK_OFFSET_MILLIS, CLOCK_OFFSET_MILLIS, 10));

    /** All of them, in the page's order. */
    public static final List<Fault> ALL =
        List.of(UNPLUGGED, FROZEN, MOVED, TURNED, LATE, CLOCK_OFFSET);
  }

  /** The gyro's faults, and the backup IMU's: they fail alike. */
  public static final class Imu {
    private Imu() {}

    /** The gyro's kind, on the page. */
    public static final String GYRO_KIND = "gyro";

    /** The backup IMU's kind, on the page. */
    public static final String BACKUP_KIND = "backup IMU";

    public static final Fault UNPLUGGED =
        lasting(
            FaultCatalog.UNPLUGGED,
            "No readings at all: the robot's code sees it disconnected.",
            "Plugs it back in.");

    public static final Fault FROZEN =
        lasting(
            FaultCatalog.FROZEN,
            "Keeps reporting, the same yaw every time, however the robot turns.",
            "Reports the robot's turns again.");

    public static final Fault RESET =
        event(
            FaultCatalog.RESET,
            "Its yaw resets to zero wherever the robot points, as a gyro that browns out and"
                + " restarts does.",
            "Nothing undoes it: the robot's code has to notice the jump.");

    /**
     * Errs another way from now on, starting from how it was built, its range wide enough to hold
     * that.
     */
    public static Fault errors(SensorErrors.GyroErrors built) {
      return lasting(
          ERRORS,
          "Errs this way from now on: its reading per true turn (1.15 reads 15% too much), a"
              + " steady drift, and noise on each reading. It carries on from what it reads now.",
          "Errs as it did as the robot was built.",
          parameter(SCALE, "", 0.5, 1.5, 1).around(built.scale()),
          parameter(DRIFT, "°/min", -10, 10, 0).around(built.driftDegreesPerMinute()),
          parameter(NOISE, "°", 0, 2, 0).around(built.noiseDegrees()));
    }

    /** All of them, for an IMU built this way, in the page's order. */
    public static List<Fault> all(SensorErrors.GyroErrors built) {
      return List.of(UNPLUGGED, FROZEN, errors(built), RESET);
    }
  }

  /** A swerve module's faults. */
  public static final class Module {
    private Module() {}

    /** The part's kind, on the page. */
    public static final String KIND = "module";

    public static final Fault STALE =
        lasting(
            FaultCatalog.STALE,
            "Its encoders' readings stop changing while the wheel turns on, as signals that stop"
                + " arriving do: odometry thinks the wheel stands still, pointing where it last"
                + " did.",
            "Its readings catch up with the wheel.");

    public static final Fault SLIPS =
        event(
            FaultCatalog.SLIPS,
            "Spins this many turns without moving the robot: its encoder counts travel the floor"
                + " never saw.",
            "Nothing undoes it: odometry has to notice the wheels disagreeing, and the cameras"
                + " correct what it missed.",
            parameter(TURNS, "turns", -10, 10, 0.5));

    public static final Fault SPINNING =
        lasting(
            FaultCatalog.SPINNING,
            "Its wheel keeps spinning this much faster than the floor moves it, as a wheel losing"
                + " its grip does: its encoder counts travel the floor never saw, and the other"
                + " wheels and the gyro disagree with it.",
            "It grips again.",
            parameter(SPEED, "m/s", -SLIDE_METERS_PER_SECOND, SLIDE_METERS_PER_SECOND, 0.5));

    /** Its absolute encoder's zero off, starting from where it was built: in degrees. */
    public static Fault zeroedOff(double builtDegrees) {
      return lasting(
          ZEROED_OFF,
          "Its absolute encoder's zero this far off: the module points that far from where the"
              + " robot's code thinks.",
          "Back to its zero as the robot was built.",
          parameter(OFFSET, "°", -45, 45, 0).around(builtDegrees));
    }

    /** Its tread worn, starting from how it was built: its true radius over the configured one. */
    public static Fault worn(double builtScale) {
      return lasting(
          WORN,
          "Its wheel's true radius over the configured one: 0.97 is tread worn 3%, and the robot"
              + " goes 3% less far than this wheel's encoder counts.",
          "Back to its tread as the robot was built.",
          parameter(SCALE, "", 0.8, 1.1, 1).around(builtScale));
    }

    /** All of them, for a module built this way, in the page's order. */
    public static List<Fault> all(double builtOffsetDegrees, double builtScale) {
      return List.of(STALE, zeroedOff(builtOffsetDegrees), worn(builtScale), SLIPS, SPINNING);
    }
  }

  /** What can happen to the robot itself. */
  public static final class Robot {
    private Robot() {}

    /** The part's kind, on the page. */
    public static final String KIND = "robot";

    public static final Fault SHOVED =
        event(
            FaultCatalog.SHOVED,
            "Pushed this far in its own frame without its wheels turning, as another robot pushes"
                + " it: its gyro feels a turn, and nothing feels the move.",
            RECOVERS,
            parameter(FORWARD, "m", -SHOVE_METERS, SHOVE_METERS, 0),
            parameter(LEFT, "m", -SHOVE_METERS, SHOVE_METERS, 0.6),
            parameter(TURN, "°", -180, 180, 0));

    public static final Fault PUSHED =
        lasting(
            FaultCatalog.PUSHED,
            "Pushed at this speed, in its own frame as it faces when the push starts, as another"
                + " robot pushes it: its wheels skid and never see it, and its IMU feels the push"
                + " get going, and stop, at 3 m/s².",
            "The push stops: the robot slows to its wheels' pace, and nothing undoes how far it"
                + " slid.",
            parameter(FORWARD, "m/s", -SLIDE_METERS_PER_SECOND, SLIDE_METERS_PER_SECOND, 0),
            parameter(LEFT, "m/s", -SLIDE_METERS_PER_SECOND, SLIDE_METERS_PER_SECOND, 1));

    public static final Fault HIT =
        event(
            FaultCatalog.HIT,
            "Knocked this far in its own frame, as a collision knocks it, with a 4 g jolt its IMU"
                + " feels.",
            RECOVERS,
            parameter(FORWARD, "m", -SHOVE_METERS, SHOVE_METERS, 0),
            parameter(LEFT, "m", -SHOVE_METERS, SHOVE_METERS, 0.6));

    public static final Fault TILTED =
        lasting(
            FaultCatalog.TILTED,
            "Tilted by hand, whatever the ground under it, as a wheel up on a game piece tilts it:"
                + " nose up is a negative pitch, left side up a positive roll. Its IMU reads the"
                + " tilt.",
            "Sets it back on the ground, which tilts it as the terrain does.",
            parameter(PITCH, "°", -MOUNT_DEGREES, MOUNT_DEGREES, -5),
            parameter(ROLL, "°", -MOUNT_DEGREES, MOUNT_DEGREES, 5));

    public static final Fault SLIPPING =
        lasting(
            FaultCatalog.SLIPPING,
            "Only this share of its wheels' rolling moves it, as with wheels in the air or on a"
                + " slick floor; the encoders count all of it, alike, so the wheels never"
                + " disagree.",
            "Full grip again.",
            parameter(GRIP, "", 0, 1, 0.75));

    public static final Fault TO_A_BUMP =
        event(
            FaultCatalog.TO_A_BUMP,
            "Carried to 1 m short of the nearest bump's ramp, facing across it, to drive over. Its"
                + " gyro feels the turn but its wheels see no move, so only its cameras can say"
                + " where it is.",
            "Place it again.");

    /** Carried somewhere on a field this size, in meters, starting at its middle. */
    public static Fault placed(double fieldLength, double fieldWidth) {
      return event(
          PLACED,
          "Carried here by the drive team, facing this way (from the blue end): its wheels don't"
              + " turn, and its gyro feels any turn.",
          "Place it again.",
          parameter(X, "m", 0, fieldLength, fieldLength / 2),
          parameter(Y, "m", 0, fieldWidth, fieldWidth / 2),
          parameter(HEADING, "°", -180, 180, 0));
    }
  }

  /** The field's faults: its tags. */
  public static final class Field {
    private Field() {}

    /** The part's kind, on the page. */
    public static final String KIND = "field";

    /** One tag moved, of tags numbered from {@code lowest} to {@code highest}, starting at one. */
    public static Fault tagMoved(int lowest, int highest, int first) {
      return lasting(
          TAG_MOVED,
          "One tag moved this far from where the field was built, in the field's frame (x away"
              + " from the blue wall, y to its left, up), as a tag knocked at an event is; moving"
              + " another puts this one back. The robot's code, and the cameras' multi-tag solves,"
              + " still believe the field's layout.",
          "Puts it back where the field was built.",
          parameter(TAG, "", lowest, highest, first),
          parameter(X, "in", -MOUNT_INCHES, MOUNT_INCHES, 4),
          parameter(Y, "in", -MOUNT_INCHES, MOUNT_INCHES, 0),
          parameter(UP, "in", -MOUNT_INCHES, MOUNT_INCHES, 0));
    }
  }
}

package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.world.game.SeasonLimits;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Driven.AbsoluteEncoder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The robot docs/mechanisms.md describes, and the one the template runs until a team writes its own
 * ({@code Robots.current()}): an intake (a pivot that swings out, with a roller on its end), a
 * hood, an elevator with an arm on it, and a shooter fed by a feeder and an indexer, on the
 * template's drivetrain and cameras. So the simulator, and Pitwall's status, tuning and
 * experiments, show real mechanisms from the start. The guide's blocks, and Robots.java's example,
 * are this code (DocExamplesTest), and ExampleRobotSimTest builds it as the robot does, so a robot
 * the guide describes is one the robot would start.
 */
public final class ExampleRobot {
  private ExampleRobot() {}

  // @start region=intake
  public static final Position PIVOT_IN = Position.degrees("in", 1.5);
  public static final Position PIVOT_OUT = Position.degrees("out", 130);

  public static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
          .range(Range.degrees(0, 135))
          .rest(PIVOT_IN) // its hard stops, and where it rests
          .carries(
              Link.box(
                  "intake arm", new Translation3d(0, 0, 0.2), new Translation3d(0.05, 0.55, 0.36)))
          .positions(PIVOT_IN, PIVOT_OUT)
          .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32).pdh(4, Amps.of(40)))
          .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))
          .control(
              MechanismControl.position(30)
                  .withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700)));

  public static final Output INTAKING = Output.rpm("intaking", 2000);
  public static final Output OUTTAKING = Output.volts("outtaking", -4);

  public static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35)) // it rides on the pivot's end
          .outputs(INTAKING, OUTTAKING)
          .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31).pdh(5, Amps.of(40)))
          // @start region=runs-only-when
          // The roller runs only once the pivot is out of the frame.
          .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
          // @end region=runs-only-when
          .control(MechanismControl.velocity(0.02));

  public static final Configuration INTAKE =
      Configuration.of("intake", PIVOT.at(PIVOT_OUT), ROLLER.at(INTAKING));

  // @end region=intake

  /** The claw's beam break, on the pivot's arm, and a rangefinder looking into the hopper. */
  public static final Sensor.Switch CLAW_BEAM =
      Sensor.Switch.beam("claw beam", PIVOT).on(CANDI.can(30).s1().normallyOpen());

  public static final Sensor.Rangefinder HOPPER =
      Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
          .on(CANRANGE.can(31));

  public static final Position HOOD_DOWN = Position.degrees("down", 0);

  /** A lob's angle: aimed shots follow a computed angle, and this gives the hood a step to test. */
  public static final Position HOOD_LOB = Position.degrees("lob", 30);

  /** A hood zeroed level, raised as its angle grows: its kG holds it, along one cosine. */
  public static final Joint.Revolute HOOD =
      Joint.pivot("hood", Part.CHASSIS, new Translation3d(-0.2, 0, 0.55), Joint.PITCH_UP)
          .range(Range.degrees(0, 40))
          .rest(HOOD_DOWN)
          .positions(HOOD_DOWN, HOOD_LOB)
          .motor(Motor.KRAKEN_X44, Gearing.of(50), TALON_FX.can(40).pdh(6, Amps.of(30)))
          .absoluteEncoder(CANCODER.can(41).offset(Rotations.of(-0.3)))
          .control(MechanismControl.position(30).withKg(0.35));

  public static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));
  public static final Position ELEVATOR_CLEAR = Position.of("clear", Inches.of(32));

  /**
   * An elevator whose CANcoder is geared to its drum, so it knows where it is at startup: its 1.2 m
   * of travel turn its 0.75 in drum ten times, and the CANcoder less than once.
   */
  public static final Joint.Prismatic ELEVATOR =
      Joint.elevator("elevator", Part.CHASSIS, new Translation3d(0, 0, 0.1))
          .range(Range.meters(0, 1.2))
          .rest(ELEVATOR_DOWN)
          .positions(ELEVATOR_DOWN, ELEVATOR_CLEAR)
          .motors(
              Motor.KRAKEN_X60,
              Gearing.of(5),
              Inches.of(0.75),
              TALON_FX.can(20).pdh(0, Amps.of(40)),
              TALON_FX.can(21).clockwise().pdh(1, Amps.of(40)))
          // Its CANcoder turns once for twelve turns of the drum: less than once over its travel.
          .absoluteEncoder(
              AbsoluteEncoder.of().ratio(1.0 / 12).on(CANCODER.can(24).offset(Rotations.of(0.31))))
          .control(MechanismControl.position(30).withKg(0.35));

  public static final Position ARM_STOWED = Position.degrees("stowed", -45);
  public static final Position ARM_SCORING = Position.degrees("scoring", 120);

  /**
   * An arm on the elevator, standing straight up at its zero and swinging back as its angle grows.
   */
  public static final Joint.Revolute ARM =
      Joint.pivot("arm", ELEVATOR, new Translation3d(0.1, 0, 0.3), Joint.PITCH_UP)
          .range(Range.degrees(-60, 150))
          .rest(ARM_STOWED)
          .positions(ARM_STOWED, ARM_SCORING)
          .motor(Motor.KRAKEN_X60, Gearing.of(60), TALON_FX.can(22).pdh(2, Amps.of(40)))
          .absoluteEncoder(CANCODER.can(23).offset(Rotations.of(0.12)))
          // @start region=keeps-out
          // The arm keeps out of the bumper's band while the elevator is too low to clear it.
          .keepsOut(Range.degrees(-10, 30))
          .when(ELEVATOR.atMost(Inches.of(32)))
          // @end
          .control(MechanismControl.position(30).withKg(0.35, Degrees.of(-90)));

  public static final Output FEEDER_FEEDING = Output.rpm("feeding", 1000);
  public static final Output INDEXER_FEEDING = Output.rpm("feeding", 1500);

  public static final Joint.Revolute FEEDER =
      Joint.roller("feeder", Part.CHASSIS, new Translation3d(-0.1, 0, 0.35))
          .outputs(FEEDER_FEEDING)
          .motor(Motor.KRAKEN_X60, Gearing.of(4), TALON_FX.can(50).pdh(7, Amps.of(40)))
          .control(MechanismControl.velocity(0.02));

  public static final Joint.Revolute INDEXER =
      Joint.roller("indexer", Part.CHASSIS, new Translation3d(0.05, 0, 0.15))
          .outputs(INDEXER_FEEDING)
          .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(51).pdh(8, Amps.of(40)))
          .control(MechanismControl.velocity(0.02));

  /** A typical shot's speed: aimed shots follow a computed one, and this gives a step to test. */
  public static final Output SHOOTING = Output.rpm("shooting", 3000);

  /** A flywheel on two motors, the second turned the other way: it coasts when told nothing. */
  public static final Joint.Revolute SHOOTER =
      Joint.roller("shooter", Part.CHASSIS, new Translation3d(-0.2, 0, 0.45))
          .outputs(SHOOTING)
          .motors(
              Motor.KRAKEN_X60,
              Gearing.of(1),
              TALON_FX.can(52).pdh(9, Amps.of(40)),
              TALON_FX.can(53).clockwise().pdh(10, Amps.of(40)))
          .control(MechanismControl.velocity(0.02).coasting());

  // @start region=configurations
  public static final Configuration FEEDING =
      Configuration.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING));

  public static final Configuration STOWED =
      Configuration.of(
          "stowed", PIVOT.at(PIVOT_IN), ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_STOWED));

  // @end

  /**
   * What 2026's manual allows a robot: a 110 in frame perimeter and 30 in tall at the start,
   * bumpers within 2.5 to 5.75 in of the floor, and 135 lb with bumpers and without its battery, a
   * battery with its leads weighing about 14.5 lb. The guide's robot is built to them.
   */
  private static final SeasonLimits SEASON =
      SeasonLimits.builder()
          .framePerimeter(Inches.of(110))
          .startingHeight(Inches.of(30))
          .extension(Inches.of(12))
          .bumperZoneBottom(Inches.of(2.5))
          .bumperZoneTop(Inches.of(5.75))
          .weight(Pounds.of(135))
          .battery(Pounds.of(14.5))
          .build();

  /**
   * Not weighed: the most the season allows, 135 lb of robot and bumpers (R408) plus a 14.5 lb
   * battery (R601). Weigh the robot and write what the scale says ({@code
   * Body.Weight.weighed(Pounds.of(131.2))}).
   */
  private static final Body.Weight WEIGHT = SEASON.unweighed();

  /**
   * Frame 27.5 x 27.5 in, the largest square within 2026's 110 in frame perimeter (R104), with
   * bumpers 4 in beyond it (R403) filling the bumper zone, 2.5 to 5.75 in up (R405), and 30 in
   * tall, 2026's starting height limit (R104). The 6.7 kg m^2 yaw inertia is an estimate, not a
   * measurement; pitch and roll are the model's box, provisional until CAD or a tilt test gives
   * them. The center of gravity is centered and low, the best case: only the simulator reads it,
   * and the drive's current limit (below) is estimated from it. Left out, it's the frame's center a
   * third of the robot's height up.
   */
  private static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(27.5))
          .frameWidth(Inches.of(27.5))
          .height(Inches.of(30))
          .bumperEnds(Inches.of(4))
          .bumperSides(Inches.of(4))
          .bumperBottom(Inches.of(2.5))
          .bumperTop(Inches.of(5.75))
          .weight(WEIGHT)
          .inertia(Body.Turning.yaw(KilogramSquareMeters.of(6.7)))
          .cg(new Translation3d(0, 0, 0.15))
          .build();

  /**
   * SDS MK4i modules with L2 gearing (6.75:1 drive, 150/7:1 steer) and 4 in wheels, driven by
   * Kraken X60s without FOC: it needs a Phoenix Pro license, and under this drivetrain's 30 A limit
   * it adds nothing. Free speed: 4.73 m/s. The gearing's losses aren't modeled yet (efficiency 1).
   * The tread's grip and the drive's and steering's inertia aren't measured, so they're left out
   * and the module's defaults stand in: PathPlanner's 1.2 for the grip, which the robot says at
   * startup, and estimates for the inertias, which only the simulator reads, and notes. Measure the
   * grip with a pull test (docs/characterization.md). The coupling, 50/14 drive motor turns per
   * turn of the module, is Tuner X's for the MK4i, as a team's 2024 code had it.
   */
  private static final DrivetrainGeometry.ModuleType MODULES =
      DrivetrainGeometry.ModuleType.builder()
          .driveMotor(Motor.KRAKEN_X60)
          .steerMotor(Motor.KRAKEN_X60)
          .wheelRadius(Inches.of(2))
          .driveGearing(Gearing.builder().ratio(6.75).efficiency(1.0).build())
          .steerGearing(Gearing.builder().ratio(150.0 / 7.0).efficiency(1.0).build())
          .couplingRatio(50.0 / 14.0)
          .build();

  /**
   * Drive stator limit 30 A. Under hard acceleration, weight shifts off the front wheels, and by a
   * simple weight-transfer estimate they start to slip near 47 A with this tread (friction 1.2;
   * about 50 A at 1.3, new) and this low center of gravity, or near 31 A with worn tread (friction
   * 1.0) and a center of gravity at 0.3 m. More current than that only spins the wheels and drains
   * the battery. Steering needs little torque: 40 A, the value Phoenix Tuner X's swerve generator
   * writes, as the 2026 code kept it.
   *
   * <p>Supply limit 60 A per drive motor, stepping down to 40 A once a motor has been held at 60 A
   * for a second, until it draws less than 40 A again. That's Phoenix's own step-down, which it
   * applies unless told otherwise, written out here so nobody is surprised by it: it keeps a 40 A
   * breaker from tripping in a long push (one can trip in 3.9 s at 60 A), and no sprint holds 60 A
   * for anywhere near a second. With the 30 A stator limit, each drive motor draws at most its 30 A
   * from the battery, so neither comes into play until the stator limit is raised past them.
   *
   * <p>Top speed 4.4 m/s leaves the velocity control headroom under the free speed; spinning is
   * capped at a driver-friendly 1 rotation per second. kV is the ideal value for these motors and
   * gearing; kS and the drive's kP are starting values from simulation, and the steering's kP and
   * kD are the 2026 code's (100 V per rotation and 0.5, converted). Measure and tune them all on
   * the real robot.
   */
  private static final DrivetrainControl DRIVING =
      DrivetrainControl.builder()
          .limits(
              DrivetrainControl.Limits.builder()
                  .driveStatorCurrent(Amps.of(30))
                  .driveSupplyCurrent(Amps.of(60))
                  .driveSupplyLowerCurrent(Amps.of(40))
                  .driveSupplyLowerTime(Seconds.of(1))
                  .steerStatorCurrent(Amps.of(40))
                  .maxLinearSpeed(MetersPerSecond.of(4.4))
                  .maxAngularSpeed(RotationsPerSecond.of(1))
                  .build())
          .gains(
              DrivetrainControl.Gains.builder()
                  .driveKs(0.0)
                  .driveKv(MODULES.idealDriveKv())
                  .driveKp(2.0)
                  .steerKp(16.0)
                  .steerKd(0.08)
                  .skewLookahead(Seconds.of(0.03))
                  .headingKp(5.0)
                  .translationKp(3.0)
                  .build())
          .build();

  /**
   * The modules sit on the corners, where MK4i wheels are 2.625 in in from the frame's edges,
   * driven as above. No wiring: on real hardware the drivetrain stays still and says so.
   */
  private static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.inset(MODULES, BODY, Inches.of(2.625)).control(DRIVING);

  /**
   * A 1280 x 800 global-shutter camera with an 80 degree diagonal field of view, typical for
   * AprilTags. The frame rate and latency are guesses: measure yours (the Vision log's {@code
   * LatencySeconds} for each camera). The latency matters on the robot, not only in the simulator,
   * which runs its cameras with it: an image arriving more than {@code
   * VisionConfig.latencyMarginSeconds} later than it is left out, so a guess too low drops images.
   * The 7 m sight range is a guess too, the simulator's all along: measure where yours stops
   * finding a tag (docs/characterization.md, camera noise).
   */
  private static final CameraConfig.Sensor APRILTAG_CAMERA =
      CameraConfig.Sensor.builder()
          .widthPixels(1280)
          .heightPixels(800)
          .fieldOfView(Degrees.of(80))
          .fieldOfViewAcross(CameraConfig.Across.DIAGONAL)
          .frameRate(Hertz.of(30))
          .latency(Milliseconds.of(30))
          .sightRange(Meters.of(7))
          .build();

  /**
   * Four cameras as a 2026 competition robot mounted them: two facing forward and out at 23
   * degrees, two facing backward, all near the frame's sides about 21 in up and tilted up 8 to 12.5
   * degrees. Positions as that team's 2026 code gives them, from the robot's origin on the floor.
   */
  private static final List<CameraConfig> CAMERAS =
      List.of(
          CameraConfig.builder()
              .name("front-left")
              .forward(Inches.of(-7.75))
              .left(Inches.of(12.25))
              .up(Inches.of(21))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(10))
              .yaw(Degrees.of(23.162))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .build(),
          CameraConfig.builder()
              .name("front-right")
              .forward(Inches.of(-8.625))
              .left(Inches.of(-12.25))
              .up(Inches.of(20.875))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(8.2))
              .yaw(Degrees.of(-23.162))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .build(),
          CameraConfig.builder()
              .name("back-left")
              .forward(Inches.of(-10.5))
              .left(Inches.of(12))
              .up(Inches.of(21))
              .roll(Degrees.of(-2))
              .tiltUp(Degrees.of(10))
              .yaw(Degrees.of(180.985))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .build(),
          CameraConfig.builder()
              .name("back-right")
              .forward(Inches.of(-11.6875))
              .left(Inches.of(-12))
              .up(Inches.of(20.705))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(12.5))
              .yaw(Degrees.of(180.985))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .build());

  /**
   * The simulator's IMUs: at the center, 3 in up on the belly pan, level with x forward, and
   * checked (they read along their axes by construction).
   */
  private static final ImuMount SIM_IMU =
      ImuMount.builder()
          .forward(Meters.of(0))
          .left(Meters.of(0))
          .up(Inches.of(3))
          .orientation(Rotation3d.ZERO)
          .accelerometerChecked(true)
          .build();

  /**
   * The template's drivetrain and cameras, with no mechanisms: what the guide's robot builds on
   * ({@code ExampleRobot}), and a robot of its own for whatever needs no joints. It has no wiring:
   * on real hardware its drivetrain stays still and says so. Give a real robot its own entry, with
   * its wiring from Phoenix Tuner X's swerve generator.
   */
  public static final RobotConfig SIM =
      RobotConfig.builder()
          .definition(
              RobotDefinition.builder()
                  .name("sim")
                  .body(BODY)
                  .drivetrain(DRIVETRAIN)
                  // The simulator's IMUs sit at the center on the belly pan, level, x forward, and
                  // read along their axes by construction. A real robot starts from
                  // Imus.UNCHECKED, its IMUs measured where they're mounted, until each passes the
                  // accelerometer check (docs/calibration.md).
                  .imus(Imus.builder().gyro(SIM_IMU).backup(SIM_IMU).build())
                  .cameras(CAMERAS)
                  // No PDH: the template's robot has no wiring. A real robot declares its own,
                  // .pdh(Pdh.can(1)), and each motor controller its channel where it's wired,
                  // TALON_FX.can(32).pdh(4, Amps.of(40)).
                  .build())
          .vision(VisionConfig.DEFAULTS)
          .estimator(EstimatorConfig.DEFAULTS)
          .build();

  /** The template's robot's back-left camera, as {@link #SIM} mounts it, 21 in up. */
  public static final CameraConfig BACK_LEFT = ExampleRobot.SIM.definition().cameras().get(2);

  /** The competition robot: the template's, with these mechanisms, each motor fed by the PDH. */
  public static final RobotConfig COMPETITION =
      RobotConfigBuilder.builder(ExampleRobot.SIM)
          .definition(
              RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                  .name("example")
                  .pdh(Pdh.can(1))
                  .joints(List.of(PIVOT, ROLLER, HOOD, ELEVATOR, ARM, FEEDER, INDEXER, SHOOTER))
                  .sensors(List.of(CLAW_BEAM, HOPPER))
                  .configurations(List.of(FEEDING, STOWED, INTAKE))
                  .build())
          .build();

  /**
   * A practice robot of the same design: its pivot's motor on another CAN ID and its pivot out at
   * 128°, its arm's CANcoder offset for its own magnet, its arm lighter and stowed at -30°, its
   * claw's beam on another CANdi, and its back-left camera half an inch lower. Each keeps the rest
   * of what it changes, so a regear of the competition robot's pivot reaches it, and the same code
   * sends its pivot to its own out.
   */
  // @start region=practice
  public static final RobotConfig PRACTICE =
      COMPETITION
          .named("practice")
          .with(
              PIVOT
                  .motorsOn(TALON_FX.can(33).pdh(4, Amps.of(40))) // another CAN ID
                  .replaced(PIVOT_OUT, Degrees.of(128)), // it swings out to 128°
              ARM.encoderOffset(Rotations.of(0.47)) // its magnet's own offset
                  .retuned(control -> control.withKg(0.3)) // a lighter arm
                  .rest(Degrees.of(-30))) // it stows at -30°
          .with(CLAW_BEAM.on(CANDI.can(36).s1().normallyOpen())) // a beam on another CANdi
          .with(CameraConfigBuilder.builder(BACK_LEFT).up(Inches.of(20.5)).build()); // lower

  /** Every robot of the design, as Robots.all() lists them. */
  public static List<RobotConfig> all() {
    return List.of(COMPETITION, PRACTICE);
  }
  // @end
}

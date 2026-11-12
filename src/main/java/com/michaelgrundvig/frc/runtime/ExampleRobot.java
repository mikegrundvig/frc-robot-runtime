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
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.Driven.AbsoluteEncoder;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Preset;
import com.michaelgrundvig.frc.world.robot.PrismaticJoint;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RevoluteJoint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Mass;

/**
 * The robot that docs/mechanisms.md describes, and the one the template runs until a team writes
 * its own ({@code Robots.current()}). On the template's drivetrain and cameras it has an intake (a
 * pivot that swings out, with a roller on its end), a hood, an elevator with an arm on it, and a
 * shooter fed by a feeder and an indexer. With it, the simulator and Pitwall's status, tuning and
 * experiments show real mechanisms from the start. What it does, and its buttons, are in {@link
 * ExampleControls}.
 *
 * <p>The guide's code blocks, and the example in Robots.java, are copied from this file, and
 * DocExamplesTest keeps them the same. ExampleRobotSimTest builds this robot the way the robot
 * program does. So the robot the guide describes is one that would really start.
 */
public final class ExampleRobot {
  private ExampleRobot() {}

  // @start region=intake
  public static final Position PIVOT_IN = Position.degrees("in", 1.5);
  public static final Position PIVOT_OUT = Position.degrees("out", 130);

  public static final RevoluteJoint PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, Joint.PITCH_DOWN)
          .at(Meters.of(0.25), Meters.of(0), Meters.of(0.32))
          .range(Range.degrees(0, 135))
          .rest(PIVOT_IN) // its hard stops, and where it rests
          .carries(
              Link.box(
                  "intake arm",
                  new Translation3d(0, 0, 0.2),
                  Meters.of(0.05),
                  Meters.of(0.55),
                  Meters.of(0.36)))
          .positions(PIVOT_IN, PIVOT_OUT)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(32).pdh(4, Amps.of(40))))
          .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))
          .control(
              MechanismControl.position(30)
                  .withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700)));

  public static final Output INTAKING = Output.rpm("intaking", 2000);
  public static final Output OUTTAKING = Output.volts("outtaking", -4);

  public static final RevoluteJoint ROLLER =
      Joint.roller("roller", PIVOT)
          .at(Meters.of(0), Meters.of(0), Meters.of(0.35)) // it rides on the pivot's end
          .outputs(INTAKING, OUTTAKING)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(3)).on(TALON_FX.can(31).pdh(5, Amps.of(40))))
          // @start region=runs-only-when
          // The roller runs only once the pivot is out of the frame.
          .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
          // @end region=runs-only-when
          .control(MechanismControl.velocity(0.02));

  public static final Preset INTAKE = Preset.of("intake", PIVOT.at(PIVOT_OUT), ROLLER.at(INTAKING));

  // @end region=intake

  /** The claw's beam break, on the pivot's arm, and a rangefinder looking into the hopper. */
  public static final Sensor.Switch CLAW_BEAM =
      Sensor.Switch.beam("claw beam", PIVOT).on(CANDI.can(30).s1().normallyOpen());

  public static final Sensor.Rangefinder HOPPER =
      Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
          .on(CANRANGE.can(31));

  public static final Position HOOD_DOWN = Position.degrees("down", 0);

  /**
   * The hood's angle for a lob. Aimed shots follow a computed angle and need no named one, so this
   * is here to give the hood a step to test with.
   */
  public static final Position HOOD_LOB = Position.degrees("lob", 30);

  /**
   * A hood whose zero is level, and which rises as its angle grows. Its kG holds it up against
   * gravity, scaled by the cosine of its angle.
   */
  public static final RevoluteJoint HOOD =
      Joint.pivot("hood", Part.CHASSIS, Joint.PITCH_UP)
          .at(Meters.of(-0.2), Meters.of(0), Meters.of(0.55))
          .range(Range.degrees(0, 40))
          .rest(HOOD_DOWN)
          .positions(HOOD_DOWN, HOOD_LOB)
          .motors(Motor.KRAKEN_X44.through(Gearing.of(50)).on(TALON_FX.can(40).pdh(6, Amps.of(30))))
          .absoluteEncoder(CANCODER.can(41).offset(Rotations.of(-0.3)))
          // Its motor's 30 A breaker: it draws no more than that, so a long push doesn't trip it.
          .control(MechanismControl.position(30).withKg(0.35).withSupplyLimit(Amps.of(30)));

  public static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));
  public static final Position ELEVATOR_CLEAR = Position.of("clear", Inches.of(32));

  /**
   * An elevator whose CANcoder is geared to its drum, so it knows where it is at startup: its 1.2 m
   * of travel turn its 0.75 in drum ten times, and the CANcoder less than once.
   */
  public static final PrismaticJoint ELEVATOR =
      Joint.elevator("elevator", Part.CHASSIS)
          .at(Meters.of(0), Meters.of(0), Meters.of(0.1))
          .range(Range.meters(0, 1.2))
          .rest(ELEVATOR_DOWN)
          .positions(ELEVATOR_DOWN, ELEVATOR_CLEAR)
          .motors(
              Motor.KRAKEN_X60
                  .through(Gearing.of(5))
                  .drum(Inches.of(0.75))
                  .on(
                      TALON_FX.can(20).pdh(0, Amps.of(40)),
                      TALON_FX.can(21).clockwise().pdh(1, Amps.of(40))))
          // Its CANcoder turns once for twelve turns of the drum: less than once over its travel.
          .absoluteEncoder(
              AbsoluteEncoder.of().ratio(1.0 / 12).on(CANCODER.can(24).offset(Rotations.of(0.31))))
          .control(MechanismControl.position(30).withKg(0.35));

  public static final Position ARM_STOWED = Position.degrees("stowed", -45);
  public static final Position ARM_SCORING = Position.degrees("scoring", 120);

  /**
   * An arm on the elevator, standing straight up at its zero and swinging back as its angle grows.
   */
  public static final RevoluteJoint ARM =
      Joint.pivot("arm", ELEVATOR, Joint.PITCH_UP)
          .at(Meters.of(0.1), Meters.of(0), Meters.of(0.3))
          .range(Range.degrees(-60, 150))
          .rest(ARM_STOWED)
          .positions(ARM_STOWED, ARM_SCORING)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(60)).on(TALON_FX.can(22).pdh(2, Amps.of(40))))
          .absoluteEncoder(CANCODER.can(23).offset(Rotations.of(0.12)))
          // @start region=keeps-out
          // The arm keeps out of the bumper's band while the elevator is too low to clear it.
          .keepsOut(Range.degrees(-10, 30))
          .when(ELEVATOR.atMost(Inches.of(32)))
          // @end
          .control(MechanismControl.position(30).withKg(0.35, Degrees.of(-90)));

  public static final Output FEEDER_FEEDING = Output.rpm("feeding", 1000);
  public static final Output INDEXER_FEEDING = Output.rpm("feeding", 1500);

  public static final RevoluteJoint FEEDER =
      Joint.roller("feeder", Part.CHASSIS)
          .at(Meters.of(-0.1), Meters.of(0), Meters.of(0.35))
          .outputs(FEEDER_FEEDING)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(4)).on(TALON_FX.can(50).pdh(7, Amps.of(40))))
          .control(MechanismControl.velocity(0.02));

  public static final RevoluteJoint INDEXER =
      Joint.roller("indexer", Part.CHASSIS)
          .at(Meters.of(0.05), Meters.of(0), Meters.of(0.15))
          .outputs(INDEXER_FEEDING)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(3)).on(TALON_FX.can(51).pdh(8, Amps.of(40))))
          .control(MechanismControl.velocity(0.02));

  /**
   * The speed of a typical shot. Aimed shots follow a computed speed and need no named one, so this
   * is here to give the shooter a step to test with.
   */
  public static final Output SHOOTING = Output.rpm("shooting", 3000);

  /** The flywheel's mass, 1.8 kg, its radius, 2 in, and its shaft's length, 0.5 m. */
  private static final Mass FLYWHEEL_MASS = Kilograms.of(1.8);

  private static final double FLYWHEEL_RADIUS = Inches.of(2).in(Meters);
  private static final double FLYWHEEL_LENGTH = 0.5;

  /** A solid cylinder's inertia about its shaft: half its mass times its radius squared. */
  private static final double FLYWHEEL_ABOUT_ITS_SHAFT =
      FLYWHEEL_MASS.in(Kilograms) * FLYWHEEL_RADIUS * FLYWHEEL_RADIUS / 2;

  /**
   * A solid cylinder's inertia across its shaft: a twelfth of its mass times the sum of three times
   * its radius squared and its length squared. Nothing reads it yet.
   */
  private static final double FLYWHEEL_ACROSS_ITS_SHAFT =
      FLYWHEEL_MASS.in(Kilograms)
          * (3 * FLYWHEEL_RADIUS * FLYWHEEL_RADIUS + FLYWHEEL_LENGTH * FLYWHEEL_LENGTH)
          / 12;

  /**
   * What the shooter's motors turn: four 4 in wheels of 0.25 kg each on one shaft 0.5 m long, with
   * a 4 in steel disk a quarter of an inch thick at each end of it to carry the wheels' speed
   * through a shot (0.4 kg each, from steel's 7850 kg/m^3). That is 1.8 kg in all. None of it has
   * been weighed: these are estimates of a typical shooter, so weigh yours, or take the numbers
   * from CAD's mass-properties report.
   *
   * <p>Every part is a disk 4 in across, so together they are as hard to spin as one solid cylinder
   * of their mass: 1/2 x 1.8 kg x (0.0508 m)^2 = 0.00232 kg m^2 about the shaft, ninety times the
   * two rotors' own 0.000026. The shaft lies along the shooter's axis, the robot's y, through the
   * joint, so gravity does not turn it.
   *
   * <p>The mass is what makes the shooter take time to spin up. Under their 40 A stator limit the
   * two Kraken X60s push with 2 x 40 A x 0.0194 N m/A = 1.55 N m, so 3000 rpm (314 rad/s) takes
   * about inertia x speed / torque = 0.00235 kg m^2 x 314 / 1.55 = 0.48 s, and a little longer to
   * settle, since the current falls below its limit over the last 300 rpm. The simulator's shooter
   * is within its band of 3000 rpm 0.50 s after it is asked. Without this link the simulator would
   * have it there within one loop, as no real shooter is, and a shot that fed too early would look
   * the same as one that waited.
   */
  private static final Link FLYWHEEL =
      Link.of(
          "flywheel",
          MassProperties.of(
              FLYWHEEL_MASS,
              Translation3d.ZERO,
              Inertia.builder()
                  .ixx(FLYWHEEL_ACROSS_ITS_SHAFT)
                  .iyy(FLYWHEEL_ABOUT_ITS_SHAFT)
                  .izz(FLYWHEEL_ACROSS_ITS_SHAFT)
                  .ixy(0)
                  .ixz(0)
                  .iyz(0)
                  .build()));

  /**
   * A flywheel on two motors, the second mounted facing the other way. It coasts when nothing is
   * asked of it.
   */
  public static final RevoluteJoint SHOOTER =
      Joint.roller("shooter", Part.CHASSIS)
          .at(Meters.of(-0.2), Meters.of(0), Meters.of(0.45))
          .carries(FLYWHEEL)
          .outputs(SHOOTING)
          .motors(
              Motor.KRAKEN_X60
                  .through(Gearing.of(1))
                  .on(
                      TALON_FX.can(52).pdh(9, Amps.of(40)),
                      TALON_FX.can(53).clockwise().pdh(10, Amps.of(40))))
          .control(MechanismControl.velocity(0.02).coasting());

  // @start region=presets
  public static final Preset FEEDING =
      Preset.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING));

  public static final Preset STOWED =
      Preset.of("stowed", PIVOT.at(PIVOT_IN), ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_STOWED));

  // @end

  /**
   * Frame 27.5 x 27.5 in, the largest square within 2026's 110 in frame perimeter (R104), with
   * bumpers 4 in beyond it (R403) filling the bumper zone, 2.5 to 5.75 in up (R405), and 30 in
   * tall, 2026's starting height limit (R104). It isn't weighed: until it is, the program takes it
   * at the season's maximum and says so at startup. Weigh it and write what the scale says ({@code
   * .weight(Pounds.of(131.2))}). The 6.7 kg m^2 yaw inertia is an estimate, not a measurement. The
   * pitch and roll inertias are those of a box, which the model works out, and they stand until CAD
   * or a tilt test gives better ones. The center of gravity is written as centered and low, which
   * is the best case. Only the simulator reads it, and the drive's current limit (below) is
   * estimated from it. If it were left out, it would be taken as the frame's center, a third of the
   * robot's overall height up.
   */
  private static final Body BODY =
      Body.frame(Inches.of(27.5), Inches.of(27.5), Inches.of(30))
          .bumpers(Inches.of(4), Inches.of(2.5), Inches.of(5.75))
          .inertia(Body.Turning.yaw(KilogramSquareMeters.of(6.7)))
          .cg(new Translation3d(0, 0, 0.15));

  /**
   * SDS MK4i modules with L2 gearing (6.75:1 drive, 150/7:1 steer) and 4 in wheels, driven by
   * Kraken X60s without FOC. FOC needs a Phoenix Pro license, and under this drivetrain's 30 A
   * limit it adds nothing. The free speed is 4.73 m/s. The gearing's losses aren't measured, so
   * they're left out ({@code Gearing.of(6.75)} is efficiency 1; {@code .efficiency(0.97)} would say
   * 3% is lost).
   *
   * <p>The tread's grip and the inertias of the drive and of the steering have not been measured.
   * They are left out here, so the module's defaults stand in for them. For the grip that is
   * PathPlanner's 1.2, and the robot says so at startup. For the inertias they are estimates, which
   * only the simulator reads, and which it notes. Measure the grip with a pull test
   * (docs/characterization.md).
   *
   * <p>The coupling, 50/14 turns of the drive motor for each turn of the module, is Tuner X's value
   * for the MK4i, as one team's 2024 code had it.
   */
  private static final Drivetrain.Module MODULES =
      Drivetrain.Module.builder()
          .driveMotor(Motor.KRAKEN_X60)
          .steerMotor(Motor.KRAKEN_X60)
          .wheelRadius(Inches.of(2))
          .driveGearing(Gearing.of(6.75))
          .steerGearing(Gearing.of(150.0 / 7.0))
          .couplingRatio(50.0 / 14.0)
          .build();

  /**
   * The template's drivetrain control, its top speed raised to 4.4 m/s: the headroom velocity
   * control needs under the 4.73 m/s free speed. The rest are the defaults ({@link
   * DrivetrainControl#defaults()}), each of which values.md gives a reason for.
   *
   * <p>Drive stator limit 30 A. Under hard acceleration, weight shifts off the front wheels, and by
   * a simple weight-transfer estimate they start to slip near 47 A with this tread (friction 1.2;
   * about 50 A at 1.3, new) and this low center of gravity, or near 31 A with worn tread (friction
   * 1.0) and a center of gravity at 0.3 m. More current than that only spins the wheels and drains
   * the battery. Steering needs little torque: 40 A, the value Phoenix Tuner X's swerve generator
   * writes.
   *
   * <p>Supply limit 60 A per drive motor, stepping down to 40 A once a motor has been held at 60 A
   * for a second, until it draws less than 40 A again. That's Phoenix's own step-down, which it
   * applies unless told otherwise, set where it shows so nobody is surprised by it: it keeps a 40 A
   * breaker from tripping in a long push (one can trip in 3.9 s at 60 A), and no sprint holds 60 A
   * for anywhere near a second. With the 30 A stator limit, each drive motor draws at most its 30 A
   * from the battery, so neither comes into play until the stator limit is raised past them.
   *
   * <p>Spinning is capped at a driver-friendly 1 rotation per second. kV is seeded with the ideal
   * value for these motors and gearing; kS and the drive's kP are starting values from simulation,
   * and the steering's kP and kD are Tuner X's stock (100 V per rotation and 0.5, to two figures).
   * Measure and tune them all on the real robot.
   */
  private static final DrivetrainControl DRIVING =
      DrivetrainControl.defaults().withMaxSpeed(MetersPerSecond.of(4.4));

  /**
   * The modules sit on the corners, where MK4i wheels are 2.625 in in from the frame's edges,
   * driven as above. No wiring: on real hardware the drivetrain stays still and says so.
   */
  private static final Drivetrain DRIVETRAIN =
      Drivetrain.inset(MODULES, BODY, Inches.of(2.625)).control(DRIVING);

  /**
   * A 1280 x 800 global-shutter camera with an 80 degree diagonal field of view, typical for
   * AprilTags. The frame rate and the latency are guesses, so measure yours (the Vision log has
   * {@code LatencySeconds} for each camera). The latency matters on the robot, and not only in the
   * simulator, which runs its cameras with that latency. An image that arrives more than {@code
   * VisionConfig.latencyMarginSeconds} later than the latency says is left out, so a guess that is
   * too low drops images. The 7 m sight range is a guess as well, and has always been the
   * simulator's. Measure where your camera stops finding a tag (docs/characterization.md, camera
   * noise).
   */
  private static final CameraConfig.ImageSensor APRILTAG_CAMERA =
      CameraConfig.ImageSensor.of(1280, 800, Hertz.of(30))
          .fieldOfView(Degrees.of(80), CameraConfig.Across.DIAGONAL)
          .latency(Milliseconds.of(30))
          .sightRange(Meters.of(7));

  /**
   * Four cameras, one at each corner of the frame, each facing diagonally outward: 45 degrees from
   * straight ahead at the front corners, 135 at the back ones. Between them they look all the way
   * round the robot, so whichever way it faces, some camera has the field's tags in front of it,
   * and each one looks out past its own corner, away from the rest of the robot. Each lens sits
   * 11.75 in from the center both ways, 2 in inside the 27.5 in frame's edges, where a camera can
   * bolt to the frame, 21 in up, level, and tilted up 10 degrees towards the tags. The simulator
   * mounts each exactly where it says, so each mount counts as measured and checked.
   */
  private static final List<CameraConfig> CAMERAS =
      List.of(
          CameraConfig.of("front-left", APRILTAG_CAMERA)
              .at(Inches.of(11.75), Inches.of(11.75), Inches.of(21))
              .facing(Degrees.of(10), Degrees.of(45))
              .measured(),
          CameraConfig.of("front-right", APRILTAG_CAMERA)
              .at(Inches.of(11.75), Inches.of(-11.75), Inches.of(21))
              .facing(Degrees.of(10), Degrees.of(-45))
              .measured(),
          CameraConfig.of("back-left", APRILTAG_CAMERA)
              .at(Inches.of(-11.75), Inches.of(11.75), Inches.of(21))
              .facing(Degrees.of(10), Degrees.of(135))
              .measured(),
          CameraConfig.of("back-right", APRILTAG_CAMERA)
              .at(Inches.of(-11.75), Inches.of(-11.75), Inches.of(21))
              .facing(Degrees.of(10), Degrees.of(-135))
              .measured());

  /**
   * The simulator's IMUs: at the center, 3 in up on the belly pan, level with x forward, and
   * checked (they read along their axes by construction).
   */
  private static final ImuMount SIM_IMU =
      ImuMount.at(Meters.of(0), Meters.of(0), Inches.of(3)).accelerometerCheckPassed();

  /**
   * The template's drivetrain and cameras, with no mechanisms: what the guide's robot builds on
   * ({@code ExampleRobot}), and a robot of its own for whatever needs no joints. It has no wiring:
   * on real hardware its drivetrain stays still and says so. Give a real robot its own entry, with
   * its wiring from Phoenix Tuner X's swerve generator.
   */
  public static final RobotConfig SIM =
      RobotConfig.of(
          RobotDefinition.of("sim", BODY, DRIVETRAIN)
              // The simulator's IMUs sit at the center on the belly pan, level, x forward, and
              // read along their axes by construction. The drivetrain has no wiring, so no
              // Pigeon of its own: the simulator stands its gyro in beside this backup IMU. A
              // real robot mounts its gyro on its drivetrain's, PIGEON_2.can(CANIVORE,
              // 1).at(mount), and its backup IMU here, each from ImuMount.UNCHECKED, measured
              // where it sits, until each passes the accelerometer check
              // (docs/calibration.md).
              .withBackupImu(SIM_IMU)
              // No PDH: the template's robot has no wiring. A real robot declares its own,
              // .withPdh(Pdh.can(1)), and each motor controller its channel where it's wired,
              // TALON_FX.can(32).pdh(4, Amps.of(40)).
              .withCameras(CAMERAS.toArray(CameraConfig[]::new)));

  /** The template's robot's back-left camera, as {@link #SIM} mounts it, 21 in up. */
  public static final CameraConfig BACK_LEFT = ExampleRobot.SIM.definition().cameras().get(2);

  /** The competition robot: the template's, with these mechanisms, each motor fed by the PDH. */
  public static final RobotConfig COMPETITION =
      RobotConfig.of(
          ExampleRobot.SIM
              .definition()
              .named("example")
              .withPdh(Pdh.can(1))
              .withJoints(PIVOT, ROLLER, HOOD, ELEVATOR, ARM, FEEDER, INDEXER, SHOOTER)
              .withSensors(CLAW_BEAM, HOPPER)
              .withPresets(FEEDING, STOWED, INTAKE));

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
              // The pivot's motor is at another CAN ID, and it swings out to 128°.
              PIVOT
                  .motors(motors -> motors.on(TALON_FX.can(33).pdh(4, Amps.of(40))))
                  .replaced(PIVOT_OUT, Degrees.of(128)),
              // The arm's magnet has its own offset, and the arm is lighter.
              ARM.absoluteEncoder(encoder -> encoder.offset(Rotations.of(0.47)))
                  .retuned(control -> control.withKg(0.3))
                  .rest(Degrees.of(-30))) // it stows at -30°
          .with(CLAW_BEAM.on(CANDI.can(36).s1().normallyOpen())) // a beam on another CANdi
          .with(CameraConfigBuilder.builder(BACK_LEFT).up(Inches.of(20.5)).build()); // lower

  /** Every robot of the design, as Robots.all() lists them. */
  public static List<RobotConfig> all() {
    return List.of(COMPETITION, PRACTICE);
  }
  // @end
}

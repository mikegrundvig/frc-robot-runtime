package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.DrivenBuilder;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Transmission;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

/**
 * A robot with the joints the runtime's tests drive, on the template's: an intake's pivot with a
 * roller on its end, an elevator, and a turret that turns freely, each turned by its block's
 * motors, unwired, so it runs in the simulator. Numbers are chosen to read, not taken from a robot.
 * A test varies a joint through its block, {@code ROBOT.with(PIVOT.retuned(control -> ...))}, or
 * wires it, {@code PIVOT.motorsOn(TALON_FX.can(32))}; one that turns it with other motors takes
 * them off first, {@code unmotored(PIVOT).motor(Motor.NEO, gearing(25), SPARK_MAX.can(32))}.
 */
public final class JointFixtures {
  private JointFixtures() {}

  /** Turning about this, a joint whose zero points forward raises it as its position grows. */
  static final Vector3 PITCH_UP = new Vector3(0, -1, 0);

  /** The pivot's places, and its push. */
  public static final Position PIVOT_IN = Position.degrees("in", 1.5);

  public static final Position PIVOT_RUN = Position.degrees("run", 110);
  public static final Position PIVOT_OUT = Position.degrees("out", 130);
  public static final Output PIVOT_PUSH = Output.volts("push", 3);

  /** The roller's outputs. */
  public static final Output ROLLER_INTAKING = Output.rpm("intaking", 1500);

  public static final Output ROLLER_OUTTAKING = Output.volts("outtaking", -6);
  public static final Output ROLLER_EJECTING = Output.dutyCycle("ejecting", -0.5);

  /** The elevator's places. */
  public static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));

  public static final Position ELEVATOR_CLEAR = Position.of("clear", Inches.of(0.4 / 0.0254));
  public static final Position ELEVATOR_TOP = Position.of("top", Inches.of(1.1 / 0.0254));

  /** The turret's places. */
  public static final Position TURRET_FRONT = Position.degrees("front", 0);

  public static final Position TURRET_BACK = Position.degrees("back", 180);

  /**
   * The intake's pivot: 2 kg, 0.3 m out along its zero, which points forward, level; stowed at
   * 1.5°, its roller run from 110°, out at 130°, its stops at 0° and 135°.
   */
  public static final Joint.Revolute PIVOT =
      Joint.Revolute.builder()
          .label("pivot")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(new Translation3d(0.2, 0, 0.3), Rotation3d.ZERO))
          .axis(PITCH_UP)
          .link(link("intake arm", 2, new Translation3d(0.3, 0, 0)))
          .positions(List.of(PIVOT_IN, PIVOT_RUN, PIVOT_OUT))
          .outputs(List.of(PIVOT_PUSH))
          .range(Range.degrees(0, 135))
          .rest(Math.toRadians(1.5))
          .build()
          .motor(Motor.KRAKEN_X60, gearing(25))
          .control(assumed(control(30, 1, profile(8, 30))));

  /** Its roller, through 3:1: intaking at 1500 rpm, outtaking at 6 V back, ejecting at half. */
  public static final Joint.Revolute ROLLER =
      Joint.Revolute.builder()
          .label("roller")
          .parent(PIVOT)
          .origin(new Transform3d(new Translation3d(0.35, 0, 0), Rotation3d.ZERO))
          .axis(PITCH_UP)
          .link(link("roller", 0.5, new Translation3d(0, 0, 0)))
          .outputs(List.of(ROLLER_INTAKING, ROLLER_OUTTAKING, ROLLER_EJECTING))
          .build()
          .motor(Motor.KRAKEN_X60, gearing(3))
          .control(MechanismControl.velocity(0.02));

  /** An elevator, 0 to 1.2 m, carrying 5 kg: down, clear at 0.4 m, and at the top. */
  public static final Joint.Prismatic ELEVATOR =
      Joint.Prismatic.builder()
          .label("elevator")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(new Translation3d(-0.2, 0, 0.1), Rotation3d.ZERO))
          .axis(Vector3.UP)
          .link(link("carriage", 5, new Translation3d(0, 0, 0.1)))
          .positions(List.of(ELEVATOR_DOWN, ELEVATOR_CLEAR, ELEVATOR_TOP))
          .range(Range.meters(0, 1.2))
          .build()
          .motors(Motor.KRAKEN_X60, gearing(10), Inches.of(1), 2)
          .control(assumed(control(60, 2, profile(1.2, 5))));

  /** A turret that turns freely about the vertical, facing forward at 0°, back at 180°. */
  public static final Joint.Revolute TURRET =
      Joint.Revolute.builder()
          .label("turret")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(new Translation3d(0, 0, 0.5), Rotation3d.ZERO))
          .axis(Vector3.UP)
          .link(link("turret", 3, new Translation3d(0, 0, 0)))
          .positions(List.of(TURRET_FRONT, TURRET_BACK))
          .build()
          .motor(Motor.KRAKEN_X60, gearing(50))
          .control(assumed(control(20, 0.5, profile(3, 10))));

  /** The intake out and rolling, and the elevator clear: three joints at once. */
  public static final Configuration INTAKING =
      Configuration.of(
          "intaking", PIVOT.at(PIVOT_OUT), ROLLER.at(ROLLER_INTAKING), ELEVATOR.at(ELEVATOR_CLEAR));

  /** The template's robot with these joints. */
  public static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
          .name("joints")
          .joints(List.of(PIVOT, ROLLER, ELEVATOR, TURRET))
          .configurations(List.of(INTAKING))
          .build();

  /** Each joint's motors, from its block: what the vendor helpers' tests drive it with. */
  public static final Drive PIVOT_MOTOR = ROBOT.math().drivesOf(PIVOT).get(0);

  public static final Drive ROLLER_MOTOR = ROBOT.math().drivesOf(ROLLER).get(0);
  public static final Drive ELEVATOR_MOTORS = ROBOT.math().drivesOf(ELEVATOR).get(0);
  public static final Drive TURRET_MOTOR = ROBOT.math().drivesOf(TURRET).get(0);

  /**
   * A joint as the fixtures have it, its motors taken off for a test to give it others: a block
   * says its motors once, and a joint re-wired keeps its motor.
   */
  public static Joint.Revolute unmotored(Joint.Revolute joint) {
    return joint.driven(DrivenBuilder.builder(joint.driven()).motors(Optional.empty()).build());
  }

  /** A sliding joint as the fixtures have it, its motors taken off for a test to give it others. */
  public static Joint.Prismatic unmotored(Joint.Prismatic joint) {
    return joint.driven(DrivenBuilder.builder(joint.driven()).motors(Optional.empty()).build());
  }

  /**
   * A limit switch in the robot's list of sensors, by a label of its own ("pivot down"), tripping
   * at a position: a test's, whose recordings name it so. A team's goes in its joint's block,
   * {@code .limitSwitch(End.LOWER, Degrees.of(0), ...)}, named for its joint and its end.
   */
  public static Sensor.Switch limit(
      String label, Joint joint, Sensor.Switch.Limit.End end, double position) {
    return limit(label, joint, end, Optional.of(position));
  }

  /** A limit switch in the robot's list of sensors, whose position isn't known. */
  public static Sensor.Switch limit(String label, Joint joint, Sensor.Switch.Limit.End end) {
    return limit(label, joint, end, Optional.empty());
  }

  private static Sensor.Switch limit(
      String label, Joint joint, Sensor.Switch.Limit.End end, Optional<Double> position) {
    return Sensor.Switch.builder()
        .label(label)
        .part(joint.parent())
        .kind(new Sensor.Switch.Limit(joint, position, end))
        .build();
  }

  /**
   * An absolute encoder in the robot's list of sensors, on a joint's axle, of a resolution: a
   * test's, for a joint without one in its block, a swerve module's steering.
   */
  public static Sensor.Encoder absoluteEncoder(String label, Joint joint, int countsPerTurn) {
    return Sensor.Encoder.builder()
        .label(label)
        .terms(List.of(Transmission.Term.builder().joint(joint).ratio(1).build()))
        .absolute(true)
        .countsPerTurn(countsPerTurn)
        .build();
  }

  /** A control whose joint is where its definition rests it at power on. */
  public static MechanismControl assumed(MechanismControl control) {
    return homed(control, MechanismControl.Homing.assumeAtRest());
  }

  /** A control with a homing recipe. */
  public static MechanismControl homed(MechanismControl control, MechanismControl.Homing homing) {
    return MechanismControlBuilder.builder(control).homing(homing).build();
  }

  /**
   * A position loop's feedback gains, the feedforward seeded from the definition, a profile, 40 A,
   * braking.
   */
  public static MechanismControl control(
      double kP, double kD, Optional<MechanismControl.Profile> profile) {
    return control(kP, kD, Optional.empty(), profile, 40);
  }

  public static MechanismControl control(
      double kP,
      double kD,
      Optional<MechanismControl.SoftLimits> softLimits,
      Optional<MechanismControl.Profile> profile,
      double statorAmps) {
    return MechanismControl.builder()
        .loop(MechanismControl.Loop.POSITION)
        .gains(MechanismControl.Gains.builder().kP(kP).kI(0).kD(kD).kS(0).build())
        .currentLimits(
            MechanismControl.CurrentLimits.builder()
                .stator(Amps.of(statorAmps))
                .supply(Amps.of(60))
                .supplyLower(Amps.of(40))
                .supplyLowerTime(Seconds.of(1))
                .build())
        .neutralMode(MechanismControl.NeutralMode.BRAKE)
        .deadChannel(MechanismControl.DeadChannel.DEFAULTS)
        .softLimits(softLimits)
        .profile(profile)
        .build();
  }

  public static Optional<MechanismControl.Profile> profile(double cruise, double acceleration) {
    return Optional.of(
        MechanismControl.Profile.builder()
            .cruiseVelocity(cruise)
            .acceleration(acceleration)
            .build());
  }

  public static Optional<MechanismControl.SoftLimits> soft(double lower, double upper) {
    return Optional.of(MechanismControl.SoftLimits.builder().lower(lower).upper(upper).build());
  }

  public static Gearing gearing(double ratio) {
    return Gearing.builder().ratio(ratio).efficiency(1).build();
  }

  /**
   * A link of a mass centered at a point in its joint's frame, a 10 cm cube's inertia, no shape.
   */
  static Link link(String label, double kilograms, Translation3d cg) {
    return Link.builder()
        .label(label)
        .mass(
            MassProperties.builder()
                .mass(Kilograms.of(kilograms))
                .cg(cg)
                .inertia(Inertia.box(Kilograms.of(kilograms), new Translation3d(0.1, 0.1, 0.1)))
                .build())
        .build();
  }

  /** A joint's motors, through a gearing, as a drive of the robot's own. */
  public static Drive motors(Joint.Revolute joint, Motor motor, int count, Gearing gearing) {
    return Drive.builder()
        .label(joint.label())
        .actuator(Actuator.Motors.builder().motor(motor).count(count).build())
        .transmission(Transmission.Geared.builder().joint(joint).gearing(gearing).build())
        .build();
  }

  /** A sliding joint's motors, through a gearing onto a drum, as a drive of the robot's own. */
  public static Drive motors(
      Joint.Prismatic joint, Motor motor, int count, Gearing gearing, Distance pitchRadius) {
    return Drive.builder()
        .label(joint.label())
        .actuator(Actuator.Motors.builder().motor(motor).count(count).build())
        .transmission(
            Transmission.Drum.builder()
                .joint(joint)
                .gearing(gearing)
                .pitchRadius(pitchRadius)
                .build())
        .build();
  }
}

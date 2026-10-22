package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.simulation.KnownGap.Check.stillOpen;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.simulation.KnownGap;
import com.michaelgrundvig.frc.simulation.WheelTruth;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.Terrain;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The truth suite's dynamics: the simulated robot against Newton, with nothing of the simulator's
 * own earlier output as the answer. Force against mass, torque against the yaw inertia, momentum
 * and energy kept or spent as they must be, and a robot that stands still standing still. What
 * today's simulator fails is written as a {@link KnownGap}, asserting today's number.
 *
 * <p>Each test commands the motor controllers directly ({@link TruthRig}), as the robot's code
 * would, and reads the truth the simulator keeps; none reads a sensor.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class DynamicsTruthSimTest {
  /**
   * A tread that can't slip at 60 A, even with the front wheels lightened by weight transfer (they
   * would at the template's 1.2): this checks force and mass, not grip.
   */
  private static final double NO_SLIP = 2.0;

  /**
   * From rest, every drive motor held at its stator limit pushes the robot with 4 Kt I G / r, and
   * physics says it speeds up at that over the robot's mass, each wheel's own moving parts counted
   * as mass at its tread: m + 4 J / r². Within 1% after the first 50 ms (the steering and the first
   * step settle), at 30 and at 60 A, while the motors stay at their limit (300 ms).
   *
   * <p>Today the robot speeds up about 2% short of it at either current: the motor controller's
   * limit is applied once a 4 ms step, as a voltage the drive's motor model then holds through the
   * step, and the current falls a little within each step as the wheel speeds up
   * (CurrentLimitedMotor's notes describe the same for the steering, which they fix there).
   */
  @ParameterizedTest
  @ValueSource(doubles = {30, 60})
  @KnownGap("the drive's current limit holds only at the start of each 4 ms step")
  void atTheStatorLimitTheRobotAcceleratesAsForceOverMass(double amps) {
    TruthRig rig = new TruthRig(NO_SLIP);
    rig.limitDrives(amps);
    rig.driveVolts(12);

    rig.run(0.06);
    double early = rig.velocity().getX();
    rig.run(0.24);
    double measured = (rig.velocity().getX() - early) / 0.24;

    double physics = 4 * rig.force(amps) / rig.effectiveMass();
    stillOpen(
        "the acceleration at " + amps + " A, m/s²",
        measured,
        physics,
        0.01,
        amps == 30 ? 4.3529 : 8.7058,
        0.002);
    assertThat(rig.velocity().getY()).as("straight ahead").isCloseTo(0, within(1e-9));
  }

  /**
   * Each module steered across its arm and pushed at a current well below slip, the robot spins up
   * as the four wheels' torque about its center over its yaw inertia, the wheels' own spin counted:
   * α = 4 F d / (I + 4 J d² / r²). Read back as the yaw inertia that acceleration implies, it's the
   * configured one ({@code Body.yawMomentOfInertia}) within 2%.
   *
   * <p>Today the robot spins as if all its mass sat at its wheels, m d² (each wheel carries a
   * quarter of the mass as inertia on its motor), and the configured inertia is read by nothing.
   */
  @Test
  @KnownGap("each wheel carries a quarter of the mass, so the robot spins as if it sat at them")
  void pushedAroundItsCenterTheRobotSpinsAsItsYawInertiaSays() {
    TruthRig rig = new TruthRig();
    rig.steerTangent();
    rig.run(0.5); // the modules turn to their angles
    rig.limitDrives(10);
    rig.driveVolts(12);

    rig.run(0.06);
    double early = rig.simulation.yawRateRadiansPerSecond();
    rig.run(0.24);
    double measured = (rig.simulation.yawRateRadiansPerSecond() - early) / 0.24;

    double d = rig.arm();
    double r = rig.wheelRadius();
    double wheels = 4 * rig.wheelInertia() * d * d / (r * r);
    double implied = 4 * rig.force(10) * d / measured - wheels;
    double configured = rig.body.yawMomentOfInertia(rig.robotMass).in(KilogramSquareMeters);
    stillOpen("the yaw inertia the robot spins as, kg m²", implied, configured, 0.02, 11.085, 0.01);
    assertThat(rig.simulation.truePose().getTranslation().getNorm())
        .as("spinning in place")
        .isLessThan(1e-9);
  }

  /** Spinning with its drive motors free (friction off), it keeps its rate within 0.1% for 2 s. */
  @Test
  void spinningFreeTheRobotKeepsItsRate() {
    TruthRig rig = new TruthRig();
    rig.steerTangent();
    rig.run(0.5);
    rig.limitDrives(30);
    rig.driveVolts(12);
    rig.run(0.5);
    rig.limitDrives(FREE);

    rig.run(0.04);
    double before = rig.simulation.yawRateRadiansPerSecond();
    rig.run(2);

    assertThat(before).as("spun up").isGreaterThan(2);
    assertThat(rig.simulation.yawRateRadiansPerSecond()).isCloseTo(before, within(before * 1e-3));
  }

  /**
   * A drive motor whose controller lets no current through: the wheel turns freely, as with
   * friction off. The configuration needs a positive limit; this one is a nanoampere.
   */
  private static final double FREE = 1e-9;

  /** Coasting with friction off, the robot keeps its momentum within 0.1% over 5 s. */
  @Test
  void coastingFreeTheRobotKeepsItsMomentum() {
    TruthRig rig = new TruthRig();
    rig.steer(Rotation2d.fromDegrees(30));
    rig.run(0.5);
    rig.driveVolts(12);
    rig.run(0.3);
    rig.limitDrives(FREE);

    rig.run(0.04);
    Translation2d before = rig.velocity().times(rig.mass());
    rig.run(5);
    Translation2d after = rig.velocity().times(rig.mass());

    assertThat(before.getNorm()).as("launched").isGreaterThan(50);
    assertThat(after.getDistance(before)).isLessThan(before.getNorm() * 1e-3);
  }

  /**
   * With its drive motors switched off (braking, as a motor at 0 V does), the robot's kinetic
   * energy never rises: by no more than 0.01 J in any loop, where a step's rounding is far below
   * that.
   */
  @Test
  void brakingTheRobotNeverGainsEnergy() {
    TruthRig rig = new TruthRig();
    rig.steer(Rotation2d.fromDegrees(-20));
    rig.run(0.5);
    rig.driveVolts(12);
    rig.run(0.6);
    rig.driveVolts(0);

    double before = kineticEnergy(rig);
    double mostGained = Double.NEGATIVE_INFINITY;
    for (int loop = 0; loop < 100; loop++) {
      rig.loops(1);
      double now = kineticEnergy(rig);
      mostGained = Math.max(mostGained, now - before);
      before = now;
    }

    assertThat(mostGained).isLessThan(0.01);
    assertThat(before).as("stopped").isLessThan(0.01);
  }

  /**
   * Coasting with friction off, its wheels steered a quarter turn: physics says the robot's energy
   * can only fall, as the wheels scrub sideways over the carpet until they roll its way. Today the
   * wheels keep their spin whatever the floor does, and the robot turns to follow them, its
   * velocity dipping and recovering: its energy falls, then rises back almost to what it was.
   */
  @Test
  @KnownGap("the floor never drags a wheel's spin: the robot follows its wheels within its grip")
  void steeringCoastingWheelsNeverGainsEnergy() {
    TruthRig rig = new TruthRig();
    rig.run(0.2);
    rig.driveVolts(12);
    rig.run(0.4);
    rig.limitDrives(FREE);
    rig.run(0.1);
    rig.steer(Rotation2d.CCW_90DEG);

    double lowest = kineticEnergy(rig);
    double rise = 0;
    for (int loop = 0; loop < 50; loop++) {
      rig.loops(1);
      double now = kineticEnergy(rig);
      lowest = Math.min(lowest, now);
      rise = Math.max(rise, now - lowest);
    }

    stillOpen(
        "energy never rises while coasting",
        rise <= 0.01,
        Math.abs(rise - 35.15) < 0.5,
        "it rose " + rise + " J after its lowest");
  }

  /**
   * A launch at the stator limit, wheels gripping: the motors' work (their torque, Kt I G, through
   * each wheel's turn) is the robot's kinetic energy, its wheels' spin included, within 2%. Today
   * it's 3.5% short: 2.2% as the acceleration is ({@link
   * #atTheStatorLimitTheRobotAcceleratesAsForceOverMass}), the motors giving less than their limit
   * between the starts of steps; and the rest because the robot moves each step at its wheels'
   * average speed over it, half a step behind their spin, so at 0.3 s it carries 1.3% less energy
   * than wheels and robot moving together would.
   */
  @Test
  @KnownGap("the drive's current limit holds only at the start of each 4 ms step")
  void aGrippingLaunchTurnsTheMotorsWorkIntoKineticEnergy() {
    TruthRig rig = new TruthRig();
    double[] start = wheelTurns(rig);
    rig.driveVolts(12);
    rig.run(0.3);

    double work = motorWork(rig, start, rig.force(30) * rig.wheelRadius());
    double energy = kineticEnergy(rig);
    stillOpen("kinetic energy over the motors' work", energy / work, 1, 0.02, 0.9648, 0.001);
  }

  /**
   * A launch past the tread's grip (120 A against μ 1.2): over 0.1 s, the motors' work is what the
   * robot's kinetic energy gained plus what the slipping wheels lost to friction, within 2%. Each
   * wheel's work is Kt I G through its turn, at the current the physics reports for it; each one's
   * friction is μ times its own load ({@code loadNewtons}) through how far its tread slid over the
   * floor; both summed loop by loop, each loop at the mean of its two ends' current and load.
   *
   * <p>Neither is the same at every wheel. Weight transfer at this push leaves each front wheel
   * about 65 N and loads each rear one past what its motor pushes (309 N), so physics says the rear
   * wheels grip, at their limit, and the front ones slip: their motors spin them up past the robot
   * within milliseconds (their own inertia against 78 N of friction), into their voltage limit,
   * where their current falls to what holds that friction, about 30 A. The window opens at 0.04 s,
   * once the front wheels have settled (in about 2 ms each time constant), so a loop's two ends
   * stand for all of it.
   *
   * <p>Today every wheel slips alike, carrying a quarter of the robot's weight, and its share of
   * the robot's inertia while it slips, so its motor's work goes into a robot the floor holds back,
   * and a third of it goes nowhere.
   */
  @Test
  @KnownGap("a slipping wheel still carries its share of the robot's inertia")
  void aSlippingLaunchSpendsTheMotorsWorkOnEnergyAndFriction() {
    TruthRig rig = new TruthRig();
    rig.limitDrives(120);
    rig.driveVolts(12);
    rig.run(0.04);

    double before = kineticEnergy(rig);
    double torquePerAmp = rig.driveMotor().Kt * rig.gearing();
    double work = 0;
    double friction = 0;
    double slid = 0;
    WheelState[] last = wheelStates(rig);
    for (int loop = 0; loop < 5; loop++) {
      rig.loops(1);
      WheelState[] now = wheelStates(rig);
      for (int i = 0; i < 4; i++) {
        double turned = now[i].turnRadians() - last[i].turnRadians();
        work += torquePerAmp * (last[i].amps() + now[i].amps()) / 2 * turned;
        // The tread's travel along the way the wheel points, less the ground's under it.
        Translation2d rolled = new Translation2d(turned * rig.wheelRadius(), now[i].heading());
        double slide = rolled.minus(now[i].contact().minus(last[i].contact())).getNorm();
        friction += MU * (last[i].loadNewtons() + now[i].loadNewtons()) / 2 * slide;
        slid += slide;
      }
      last = now;
    }
    double spent = kineticEnergy(rig) - before + friction;

    assertThat(slid).as("the wheels slipped").isGreaterThan(0.01);
    stillOpen(
        "kinetic energy gained and friction over the motors' work",
        spent / work,
        1,
        0.02,
        0.6888,
        0.002);
  }

  /** The template's tread: 1.2. */
  private static final double MU =
      ExampleRobot.SIM.definition().drivetrain().wheelCoefficientOfFriction();

  /**
   * One wheel as the physics has it now: its turn, its drive's current, its load, where its tread
   * meets the floor, and which way it points on the field.
   */
  private record WheelState(
      double turnRadians,
      double amps,
      double loadNewtons,
      Translation2d contact,
      Rotation2d heading) {}

  private static WheelState[] wheelStates(TruthRig rig) {
    WheelState[] states = new WheelState[4];
    Rotation2d robot = rig.simulation.truePose().getRotation();
    for (int i = 0; i < 4; i++) {
      WheelTruth wheel = rig.simulation.physics().wheel(i);
      states[i] =
          new WheelState(
              wheel.rotationRadians(),
              wheel.driveCurrentAmps(),
              wheel.loadNewtons(),
              wheel.contactPoint().toTranslation2d(),
              robot.plus(new Rotation2d(wheel.steerRadians())));
    }
    return states;
  }

  /** Standing for a minute with its motors off, the robot creeps less than 1 mm and 0.01°. */
  @Test
  void standingStillTheRobotStaysPut() {
    TruthRig rig = new TruthRig();
    Pose2d start = new Pose2d(3, 4, Rotation2d.fromDegrees(30));
    rig.simulation.place(start);
    rig.driveVolts(0);

    rig.run(60);

    assertCreptLessThanAMillimeter(rig, start);
  }

  /**
   * Its modules locked in an X (each wheel along its arm from the center) and its motors off, on a
   * 15° ramp, the robot creeps less than 1 mm and 0.01° in a minute. No motion of the frame rolls
   * all four wheels without sliding one across itself, so the slope's pull (sin 15°, 172 N) has to
   * be held across the wheels, by their grip alone: about 61 N across each, against μ N of 165 N
   * for the lightest (the uphill ones, weight transfer taking 23 N off each). Today's simulator
   * feels no pull down the slope at all, so it passes for want of a force; an engine that feels
   * gravity must pass it on its grip.
   */
  @Test
  void lockedInAnXOnARampTheRobotStaysPut() {
    TruthRig rig = new TruthRig();
    rig.simulation.setTerrain(new Terrain(List.of(RAMP)));
    for (int i = 0; i < 4; i++) {
      rig.modules[i].setSteerAngle(rig.wheels[i].getAngle().orElseThrow());
    }
    rig.run(0.5); // the modules turn into the X, on the flat floor
    Pose2d start = new Pose2d(3.3, 4, Rotation2d.ZERO);
    rig.simulation.place(start);
    rig.driveVolts(0);

    rig.run(60);

    assertThat(Math.toDegrees(-rig.simulation.pitchRadians()))
        .as("nose up the ramp")
        .isCloseTo(15, within(1e-6));
    assertCreptLessThanAMillimeter(rig, start);
  }

  /**
   * Facing up a 15° ramp, its wheels straight and its motors at 0 V, the robot creeps down it: a
   * motor at 0 V brakes only through its back EMF, I = ω / (Kv R), so holding a quarter of the
   * slope's pull (43 N a wheel, 16.7 A) takes its wheel turning at ω = I Kv R through its gearing.
   * Once that speed settles (its time constant is 0.09 s), the robot creeps at 0.217 m/s along the
   * slope, 0.210 m/s across the field: within 2%, over 2 s.
   *
   * <p>Today's simulator feels no pull down the slope, so it stands still.
   */
  @Test
  @KnownGap("nothing pulls the robot down a slope")
  void brakedOnARampWithItsWheelsStraightTheRobotCreepsDown() {
    TruthRig rig = new TruthRig();
    rig.simulation.setTerrain(new Terrain(List.of(RAMP)));
    rig.simulation.place(new Pose2d(3.3, 4, Rotation2d.ZERO));
    rig.driveVolts(0);
    rig.run(1);

    Translation2d from = rig.simulation.truePose().getTranslation();
    rig.run(2);
    double measured = (from.getX() - rig.simulation.truePose().getX()) / 2;

    double slope = Math.toRadians(15);
    double amps = rig.mass() * ImuReadings.STANDARD_GRAVITY * Math.sin(slope) / 4 / rig.force(1);
    double motorSpeed = amps * rig.driveMotor().Kv * rig.driveMotor().R;
    double physics = motorSpeed / rig.gearing() * rig.wheelRadius() * Math.cos(slope);
    stillOpen("the creep down the ramp across the field, m/s", measured, physics, 0.02, 0, 1e-9);
  }

  /**
   * A ramp rising 15° toward +x from x = 1.27 m to its crest at x = 5 m, across the field: the
   * robot placed at x = 3.3 m stands wholly on the slope.
   */
  private static final Bump RAMP =
      new Bump(
          "A 15° ramp",
          new Translation2d(5, -1),
          new Translation2d(5, 9),
          1.0,
          2 * 1.0 / Math.tan(Math.toRadians(15)));

  private static void assertCreptLessThanAMillimeter(TruthRig rig, Pose2d start) {
    Pose2d end = rig.simulation.truePose();
    assertThat(end.getTranslation().getDistance(start.getTranslation())).isLessThan(0.001);
    assertThat(Math.abs(end.getRotation().minus(start.getRotation()).getDegrees()))
        .isLessThan(0.01);
  }

  /** Every wheel's turn since the start, in radians: what it rolled, slipping or not. */
  private static double[] wheelTurns(TruthRig rig) {
    double[] turns = new double[4];
    for (int i = 0; i < 4; i++) {
      turns[i] = rig.simulation.physics().wheel(i).rotationRadians();
    }
    return turns;
  }

  /** The motors' work at a steady torque at the wheel: that torque through each wheel's turn. */
  private static double motorWork(TruthRig rig, double[] start, double torqueNewtonMeters) {
    double[] end = wheelTurns(rig);
    double work = 0;
    for (int i = 0; i < 4; i++) {
      work += torqueNewtonMeters * (end[i] - start[i]);
    }
    return work;
  }

  /**
   * The robot's kinetic energy, in joules: its mass moving, its body spinning about its center at
   * its configured yaw inertia, and each wheel's own spin.
   */
  private static double kineticEnergy(TruthRig rig) {
    double speed = rig.velocity().getNorm();
    double yawRate = rig.simulation.yawRateRadiansPerSecond();
    double energy =
        0.5 * rig.mass() * speed * speed
            + 0.5
                * rig.body.yawMomentOfInertia(rig.robotMass).in(KilogramSquareMeters)
                * yawRate
                * yawRate;
    ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
    for (ModuleIO module : rig.modules) {
      module.updateInputs(inputs);
      double spin = inputs.driveVelocityRadPerSec;
      energy += 0.5 * rig.wheelInertia() * spin * spin;
    }
    return energy;
  }
}

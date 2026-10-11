package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.Terrain;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Today's physics on its own, driven by the simplest motor controllers ({@link
 * TestRobot.Controllers}): it moves the robot as its wheels roll, at the grip it's given, on the
 * ground it's given, and does what each disturbance says. The truth suite holds it to physics;
 * these hold it to its own description.
 */
class KinematicPhysicsTest {
  private static final double STEP = 0.004;

  private final KinematicPhysics physics = TestRobot.physics();
  private final TestRobot.Controllers controllers = new TestRobot.Controllers();

  private void run(double seconds) {
    for (int i = 0; i < Math.round(seconds / STEP); i++) {
      physics.step(STEP, controllers);
    }
  }

  private double mass() {
    return TestRobot.BODY.mass().in(Kilograms);
  }

  /**
   * From rest, every motor at a 30 A stator limit, the robot speeds up straight ahead at the four
   * wheels' push over its mass and its wheels' own: within 3% (the physics holds each step's
   * voltage, so it runs about 2% short).
   */
  @Test
  void atItsCurrentLimitTheRobotSpeedsUpAsItsMotorsPush() {
    controllers.volts = 12;
    run(0.06);
    double early = physics.chassis().velocity().getX();
    run(0.24);
    double acceleration = (physics.chassis().velocity().getX() - early) / 0.24;

    double push = 4 * TestRobot.MOTOR.Kt * 30 * TestRobot.GEARING / TestRobot.RADIUS;
    double wheels = 4 * 0.001 / (TestRobot.RADIUS * TestRobot.RADIUS);
    assertThat(acceleration).isCloseTo(push / (mass() + wheels), within(0.03 * push / mass()));
    assertThat(physics.wheel(2).driveCurrentLimited()).as("held to its limit").isTrue();
    assertThat(physics.wheel(2).driveVolts()).as("cut back from 12 V").isBetween(0.0, 11.0);
    assertThat(physics.chassis().velocity().getY()).isCloseTo(0, within(1e-12));
    assertThat(physics.chassis().seconds()).isCloseTo(0.3, within(1e-9));
    assertThat(physics.wheels()).isEqualTo(4);
  }

  /**
   * Each wheel turns as its motor drives it, at its radius, pointing where it was steered; it
   * carries a quarter of the robot's weight, touches the ground, and nothing scrubs its steering.
   */
  @Test
  void eachWheelTurnsCarriesAQuarterAndTouchesTheGround() {
    controllers.volts = 6;
    controllers.steer(0.3);
    run(0.5);

    WheelTruth wheel = physics.wheel(1);
    assertThat(wheel.spinRadPerSec()).isPositive();
    assertThat(wheel.rotationRadians()).isPositive();
    assertThat(wheel.steerRadians()).isEqualTo(0.3);
    assertThat(wheel.radiusMeters()).isEqualTo(TestRobot.RADIUS);
    assertThat(wheel.driveCurrentAmps()).isPositive();
    assertThat(wheel.steerLoadNewtonMeters()).isZero();
    assertThat(wheel.loadNewtons()).isCloseTo(mass() * Physics.STANDARD_GRAVITY / 4, within(1e-9));
    assertThat(wheel.inContact()).isTrue();
    Pose2d pose = physics.chassis().pose();
    Translation2d under =
        pose.getTranslation().plus(TestRobot.wheels()[1].rotateBy(pose.getRotation()));
    assertThat(wheel.contactPoint()).isEqualTo(new Translation3d(under.getX(), under.getY(), 0));
    assertThat(physics.chassis().velocity().getAngle().orElseThrow().getRadians())
        .isCloseTo(0.3, within(1e-9));
  }

  /**
   * A motor turning faster than its battery's voltage would turn it, the battery sagged to 6 V:
   * holding its current to the limit would take more than 6 V across it, so its controller gives
   * the battery's 6 V, and the limit can't hold the current back.
   */
  @Test
  void aMotorOverrunGetsTheBatterysVoltageNotTheLimits() {
    controllers.volts = 12;
    controllers.limitAmps = Double.POSITIVE_INFINITY;
    run(2); // near its free speed at 12 V
    controllers.volts = 6;
    controllers.batteryVolts = 6;
    controllers.limitAmps = 1;
    run(0.004);

    assertThat(physics.wheel(0).driveVolts()).isCloseTo(6, within(1e-12));
    assertThat(physics.wheel(0).driveCurrentLimited()).isTrue();
  }

  /**
   * WPILib's motor model, as the physics keeps it: a motor's current is signed by its voltage, so a
   * motor at 0 V reads none, though it brakes a spinning wheel.
   */
  @Test
  void aMotorAtZeroVoltsReadsNoCurrentWhileItBrakes() {
    controllers.volts = 12;
    run(0.3);
    double before = physics.wheel(0).spinRadPerSec();
    controllers.volts = 0;
    controllers.limitAmps = Double.POSITIVE_INFINITY; // nothing holds it off 0 V
    run(0.02);

    assertThat(physics.wheel(0).spinRadPerSec()).isLessThan(before);
    assertThat(physics.wheel(0).driveCurrentAmps()).isZero();
  }

  /** Placed, it's there at once, its wheels unturned; a turn while carried counts as one made. */
  @Test
  void placedTheRobotIsThereAndItsTurnCounts() {
    physics.apply(new Disturbance.Placed(new Pose2d(3, 2, Rotation2d.fromDegrees(90))));

    assertThat(physics.chassis().pose()).isEqualTo(new Pose2d(3, 2, Rotation2d.fromDegrees(90)));
    assertThat(physics.chassis().yawTurnedRadians()).isCloseTo(Math.PI / 2, within(1e-12));
    assertThat(physics.wheel(0).rotationRadians()).isZero();
  }

  /**
   * Pushed, it slides toward the push's velocity, getting going at the push's pace, its wheels
   * never turning; stopped, it slows the same way.
   */
  @Test
  void pushedTheRobotSlidesAtThePushsPace() {
    physics.apply(new Disturbance.Pushed(new Translation2d(0, 1)));
    run(0.1);
    double getting = physics.chassis().velocity().getY();
    run(0.5);
    double going = physics.chassis().velocity().getY();
    physics.apply(new Disturbance.Pushed(Translation2d.ZERO));
    run(0.5);

    assertThat(getting)
        .isCloseTo(
            0.1 * KinematicPhysics.PUSH_ACCELERATION_METERS_PER_SECOND_SQUARED, within(1e-9));
    assertThat(going).isCloseTo(1, within(1e-9));
    assertThat(physics.chassis().velocity().getNorm()).isCloseTo(0, within(1e-9));
    assertThat(physics.wheel(0).rotationRadians()).isZero();
  }

  /**
   * Tilted by hand, it holds the tilt whatever the ground; set back down, it rides the ground: up a
   * ramp of a quarter rise, raised and nose up.
   */
  @Test
  void tiltedByHandItHoldsTheTiltUntilSetBackOnTheGround() {
    Terrain ramp =
        new Terrain(
            List.of(new Bump("Ramp", new Translation2d(5, 0), new Translation2d(5, 8), 0.5, 4)));
    physics.apply(new Disturbance.Ground(ramp));
    physics.apply(new Disturbance.Placed(new Pose2d(4, 4, Rotation2d.ZERO)));
    physics.apply(new Disturbance.TiltedByHand(0.1, -0.2));

    assertThat(physics.chassis().pitchRadians()).isEqualTo(0.1);
    assertThat(physics.chassis().rollRadians()).isEqualTo(-0.2);

    physics.apply(new Disturbance.Ground(ramp));
    assertThat(physics.chassis().pose3d().getZ()).isCloseTo(0.25, within(1e-9));
    assertThat(physics.chassis().pitchRadians()).isCloseTo(-Math.atan(0.25), within(1e-9));
    assertThat(physics.chassis().rollRadians()).isCloseTo(0, within(1e-9));
    assertThat(physics.wheel(0).contactPoint().getZ())
        .isCloseTo(ramp.heightAt(physics.wheel(0).contactPoint().toTranslation2d()), within(1e-9));
  }

  /** On a slick floor, a share of the wheels' rolling moves the robot. */
  @Test
  void onASlickFloorTheRobotMovesAShareOfItsWheelsRolling() {
    physics.apply(new Disturbance.Grip(0.5));
    controllers.volts = 3;
    run(1);

    double rolled = physics.wheel(0).rotationRadians() * TestRobot.RADIUS;
    assertThat(physics.chassis().pose().getX()).isCloseTo(rolled / 2, within(1e-9));
  }

  /** A worn tread rolls the robot less far per turn, from when it wore. */
  @Test
  void aWornTreadRollsTheRobotLessFarPerTurn() {
    for (int i = 0; i < 4; i++) {
      physics.apply(new Disturbance.TreadWorn(i, 0.9));
    }
    controllers.volts = 3;
    run(1);

    assertThat(physics.wheel(0).radiusMeters()).isCloseTo(0.9 * TestRobot.RADIUS, within(1e-12));
    assertThat(physics.chassis().pose().getX())
        .isCloseTo(physics.wheel(0).rotationRadians() * 0.9 * TestRobot.RADIUS, within(1e-9));
  }

  /** Spinning, a point off its center moves around it: the turn's rate times its arm, across it. */
  @Test
  void aPointOffTheCenterOfASpinningRobotSwingsAroundIt() {
    Translation2d[] wheels = TestRobot.wheels();
    for (int i = 0; i < 4; i++) {
      controllers.steer[i] = wheels[i].getAngle().orElseThrow().getRadians() + Math.PI / 2;
    }
    controllers.volts = 4;
    run(0.5);

    double rate = physics.chassis().yawRateRadiansPerSecond();
    Translation2d place = new Translation2d(0.5, 0);
    Translation2d expected =
        new Translation2d(0, rate * 0.5).rotateBy(physics.chassis().pose().getRotation());
    assertThat(rate).isPositive();
    assertThat(physics.chassis().pointVelocity(place).getDistance(expected)).isLessThan(1e-9);
  }

  @Test
  void stepsTakeTimeAndDisturbancesTakeSenseNumbers() {
    assertThatThrownBy(() -> physics.step(0, controllers))
        .hasMessageContaining("a step takes time");
    assertThatThrownBy(() -> new Disturbance.Grip(1.5))
        .hasMessage("a share of the wheels' rolling: 1.5");
    assertThatThrownBy(() -> new Disturbance.TreadWorn(0, 0))
        .hasMessage("a wheel's radius scale is positive: 0.0");
  }

  /**
   * What the controllers give each wheel, kept as given: the drive's voltage and limits, and the
   * steering.
   */
  @Test
  void aWheelsDriveKeepsWhatItsControllersGive() {
    WheelDrive drive = new WheelDrive();
    assertThat(drive.currentLimitAmps()).isInfinite();

    drive.drive(6, 40, 12);
    drive.steer(1.2);

    assertThat(drive.volts()).isEqualTo(6);
    assertThat(drive.currentLimitAmps()).isEqualTo(40);
    assertThat(drive.batteryVolts()).isEqualTo(12);
    assertThat(drive.steerRadians()).isEqualTo(1.2);
  }

  /** The drive's load: the wheel's share of the robot at its radius, and its own moving parts. */
  @Test
  void eachDriveTurnsItsShareOfTheRobotAtItsRadius() {
    assertThat(KinematicPhysics.driveInertiaKgM2(17, 0.05, 0.001))
        .isCloseTo(17 * 0.0025 + 0.001, within(1e-15));
  }
}

package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.assertj.core.api.Assertions.withinPercentage;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.EstimatorConfigBuilder;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.SensorErrorsBuilder;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;

/**
 * The simulator keeps where the robot really is, and its wheels and gyro report that with the
 * errors real ones have. Each test injects one error and checks that odometry drifts from the truth
 * the way a real robot's does: the gap localization has to close.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class DriveSimulationTest {
  private @Nullable DriveSimHarness harness;

  @AfterEach
  void tearDown() {
    if (harness != null) {
      harness.close();
    }
  }

  private DriveSimHarness robot(SensorErrors errors) {
    harness = new DriveSimHarness(ExampleRobot.SIM, errors);
    return harness;
  }

  /** Both IMUs reading with this scale and drift, and no noise. */
  private static SensorErrors gyro(double scale, double driftDegreesPerMinute) {
    return SensorErrors.NONE.withGyros(
        SensorErrors.GyroErrors.builder()
            .scale(scale)
            .driftDegreesPerMinute(driftDegreesPerMinute)
            .noiseDegrees(0)
            .build());
  }

  /** Without errors, and with the wheels holding their angle, odometry is the truth. */
  @Test
  void withoutErrorsOdometryIsTheTruth() {
    DriveSimHarness robot = robot(SensorErrors.NONE);

    robot.drive(2.0, 1.0, 0, 0);

    assertThat(robot.simulation.truePose().getTranslation().getNorm()).isGreaterThan(1);
    assertThat(robot.positionError()).isLessThan(1e-6);
    assertThat(robot.headingErrorDegrees()).isCloseTo(0.0, within(1e-6));
  }

  /**
   * Even without errors, turning wheels cost odometry a little: each wheel's angle comes from its
   * CANcoder, which sends it every 10 ms, so odometry sees a turning wheel up to 10 ms late. Over
   * two seconds of curving while spinning, about a centimeter. The gyro still has the heading
   * exactly.
   */
  @Test
  void turningWheelsReachOdometryAsLateAsTheirCanCodersSendThem() {
    DriveSimHarness robot = robot(SensorErrors.NONE);

    robot.drive(2.0, 0.6, 0.3, 0.4); // curving and spinning at once

    assertThat(robot.simulation.truePose().getTranslation().getNorm()).isGreaterThan(1);
    assertThat(robot.positionError()).isBetween(0.005, 0.02);
    assertThat(robot.headingErrorDegrees()).isCloseTo(0.0, within(1e-6));
  }

  /** Worn tread rolls less far per turn than the code thinks, so odometry overshoots. */
  @Test
  void wornTreadMakesOdometryOverestimateDistance() {
    DriveSimHarness robot = robot(SensorErrors.NONE.withWheelRadiusScale(0.98));

    robot.drive(2.0, 1.0, 0, 0);

    double traveled = robot.simulation.truePose().getX();
    assertThat(traveled).isGreaterThan(3);
    assertThat(robot.drive.getPose().getX() / traveled).isCloseTo(1 / 0.98, within(1e-3));
  }

  /**
   * A gyro drifting 10° a minute turns an estimate that only considers the gyro's errors by a
   * degree in a tenth of a minute.
   */
  @Test
  void aDriftingGyroTurnsTheEstimateWhileTheRobotSitsStill() {
    EstimatorConfig considering =
        EstimatorConfigBuilder.builder(EstimatorConfig.DEFAULTS)
            .gyroErrors(EstimatorConfig.GyroErrors.CONSIDERED)
            .build();
    harness = new DriveSimHarness(ExampleRobot.SIM, gyro(1.0, 10.0), considering);

    harness.idle(6); // a tenth of a minute

    assertThat(harness.headingErrorDegrees()).isCloseTo(1.0, within(0.01));
    assertThat(harness.positionError()).as("its wheels never turned").isLessThan(1e-9);
  }

  /**
   * The estimator as it ships reads that drift off the gyro while the robot stands, a second at a
   * time, and is left with a tenth of a degree: what the first seconds turned it.
   */
  @Test
  void theEstimatorReadsADriftingGyroOffWhileTheRobotSitsStill() {
    DriveSimHarness robot = robot(gyro(1.0, 10.0));

    robot.idle(6);

    assertThat(robot.headingErrorDegrees()).isLessThan(0.15);
  }

  @Test
  void aGyroReadingHighMisjudgesEveryTurn() {
    DriveSimHarness robot = robot(gyro(1.01, 0));

    robot.drive(1.0, 0, 0, 1.0); // spin in place

    double turnedDegrees = Math.toDegrees(robot.simulation.yawTurnedRadians());
    assertThat(turnedDegrees).isGreaterThan(200);
    assertThat(robot.headingErrorDegrees()).isCloseTo(turnedDegrees * 0.01, within(0.05));
  }

  /** An absolute encoder zeroed 5° off steers its module 5° off, and odometry can't tell. */
  @Test
  void anEncoderZeroedOffPointsItsModuleWrongAndOdometryDrifts() {
    SensorErrors oneOff =
        SensorErrorsBuilder.builder(SensorErrors.NONE)
            .steerOffsetDegrees(List.of(5.0, 0.0, 0.0, 0.0))
            .build();
    DriveSimHarness robot = robot(oneOff);

    robot.drive(2.0, 1.0, 0, 0);

    assertThat(robot.positionError()).isGreaterThan(0.05);
  }

  /** Another robot's push moves it without turning its wheels: only cameras can catch that. */
  @Test
  void aShoveMovesTheRobotWhereOdometryCannotSee() {
    DriveSimHarness robot = robot(SensorErrors.NONE);

    robot.simulation.shove(new Transform2d(0, 0.3, Rotation2d.ZERO));
    robot.idle(0.1);

    assertThat(robot.positionError()).isCloseTo(0.3, within(1e-9));
  }

  @Test
  void theGyroFeelsTheRobotTurnedByHand() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    robot.idle(0.1);

    robot.simulation.shove(new Transform2d(0, 0, Rotation2d.fromDegrees(30)));
    robot.idle(0.1);

    assertThat(robot.headingErrorDegrees()).as("the gyro followed").isCloseTo(0.0, within(1e-9));
    assertThat(robot.drive.getPose().getRotation().getDegrees()).isCloseTo(30.0, within(1e-9));
  }

  @Test
  void placedAndToldTheRobotKnowsWhereItIs() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    Pose2d start = new Pose2d(3, 4, Rotation2d.fromDegrees(90));

    robot.simulation.place(start);
    robot.loop();
    robot.localization.resetPose(start);

    assertThat(robot.positionError()).isLessThan(1e-9);
    assertThat(robot.headingErrorDegrees()).isCloseTo(0.0, within(1e-9));
  }

  /**
   * A long ramp up to a crest at x = 5, rising a quarter meter per meter: every wheel fits on it.
   */
  private static final Terrain RAMP =
      new Terrain(
          List.of(new Bump("Ramp", new Translation2d(5, 0), new Translation2d(5, 8), 0.5, 4)));

  @Test
  void theGroundUnderItsWheelsTiltsAndRaisesTheRobot() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    robot.simulation.setTerrain(RAMP);

    robot.simulation.place(new Pose2d(4, 4, Rotation2d.ZERO)); // facing up the ramp
    Pose3d facingUp = robot.simulation.truePose3d();
    robot.simulation.place(new Pose2d(4, 4, Rotation2d.CCW_PI_2)); // across it
    Pose3d across = robot.simulation.truePose3d();

    assertThat(facingUp.getZ()).isCloseTo(0.25, within(1e-9));
    assertThat(facingUp.getRotation().getY())
        .as("nose up: a negative pitch")
        .isCloseTo(-Math.atan(0.25), within(1e-9));
    assertThat(facingUp.getRotation().getX()).isCloseTo(0.0, within(1e-9));
    assertThat(across.getRotation().getX())
        .as("left side downhill: a negative roll")
        .isCloseTo(-Math.atan(0.25), within(1e-9));
    assertThat(across.getRotation().getY()).isCloseTo(0.0, within(1e-9));
  }

  /** Up a ramp the wheels roll farther than the robot moves across the field; odometry knows. */
  @Test
  void onAKnownRampOdometryCountsTheWheelsTravelAtTheSlope() {
    DriveSimHarness robot =
        new DriveSimHarness(ExampleRobot.SIM, SensorErrors.NONE, EstimatorConfig.DEFAULTS, RAMP);
    harness = robot;
    Pose2d start = new Pose2d(3.4, 4, Rotation2d.ZERO);
    robot.simulation.place(start);
    robot.loop();
    robot.localization.resetPose(start);
    double wheelBefore = robot.drive.wheelRotationsRadians()[0];

    robot.run(robot.drive.driveForward(1.0, 0.8), 1.0);

    double moved = robot.simulation.truePose().getX() - start.getX();
    double radius = ExampleRobot.SIM.definition().drivetrain().module(0).wheelRadius().in(Meters);
    double rolled = (robot.drive.wheelRotationsRadians()[0] - wheelBefore) * radius;
    assertThat(moved).isGreaterThan(0.5);
    assertThat(rolled).isCloseTo(moved * Math.hypot(1, 0.25), within(0.01));
    assertThat(robot.positionError()).isLessThan(0.005);
  }

  /**
   * The robot's configured mass is what its drive motors move. From rest, each drive motor at its
   * stator limit I pushes its wheel's tread with F = Kt I G / r, and the robot speeds up at the
   * four wheels' force over its mass: its configured mass, plus each wheel's own moving parts as
   * mass at its tread (2% of the template's robot). The same for a robot 40% lighter: the simulator
   * follows the configuration's mass. Within 4%: the simulator sets the motor controller's limit
   * once a 4 ms step, and the current falls a little within each step as the wheel speeds up, so
   * the robot speeds up slower than the limit allows, by 2% at the template's mass and 3.6% at the
   * lighter one.
   */
  @Test
  void fromRestAtItsStatorLimitTheRobotAcceleratesAsForceOverItsMass() {
    double template = RobotSim.massOf(ExampleRobot.SIM.definition()).in(Kilograms);
    for (double kilograms : new double[] {template, 0.6 * template}) {
      assertThat(accelerationFromRest(kilograms))
          .as("%.1f kg", kilograms)
          .isCloseTo(limitForceNewtons() / effectiveKilograms(kilograms), withinPercentage(4));
    }
  }

  /** The four drive motors' push at their stator limit, in newtons. */
  private static double limitForceNewtons() {
    DrivetrainGeometry.ModuleType modules = ExampleRobot.SIM.definition().drivetrain().module(0);
    DCMotor motor = modules.driveMotor().model();
    return 4
        * motor.Kt
        * ExampleRobot.SIM.drivetrain().limits().driveStatorCurrent().in(Amps)
        * modules.driveGearing().ratio()
        / modules.wheelRadius().in(Meters);
  }

  /** The robot's mass with its wheels' own inertia as mass at their treads, in kilograms. */
  private static double effectiveKilograms(double kilograms) {
    DrivetrainGeometry.ModuleType modules = ExampleRobot.SIM.definition().drivetrain().module(0);
    double radius = modules.wheelRadius().in(Meters);
    return kilograms
        + 4 * modules.driveInertiaOrDefault().in(KilogramSquareMeters) / (radius * radius);
  }

  /**
   * The template's robot at this mass, its drive motors asked for full voltage from rest: its
   * average acceleration over 0.3 s, in m/s², while every motor stays at its stator limit.
   */
  private static double accelerationFromRest(double kilograms) {
    DrivetrainControl drivetrain = ExampleRobot.SIM.drivetrain();
    DriveSimulation simulation =
        new DriveSimulation(
            ExampleRobot.SIM.definition().body(),
            Kilograms.of(kilograms),
            ExampleRobot.SIM.definition().drivetrain(),
            SensorErrors.NONE);
    for (int i = 0; i < 4; i++) {
      ModuleIOSim module = simulation.moduleSim(i);
      module.configure(drivetrain.gains(), drivetrain.limits());
      module.setDriveVoltage(12);
    }
    double seconds = 0.3;
    long nanos = 0;
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      nanos += Math.round(RobotLoop.PERIOD_SECONDS * 1e9);
      simulation.step(RobotLoop.PERIOD_SECONDS, nanos);
    }
    assertThat(simulation.truePose().getY()).isCloseTo(0, within(1e-9));
    return 2 * simulation.truePose().getX() / (seconds * seconds);
  }

  /** Noise comes from a seed, so a simulated run, like a test, gives the same answer every time. */
  @Test
  void theSameSeedRepeatsARunExactly() {
    Pose2d first = drivenWith(SensorErrors.TYPICAL);
    Pose2d second = drivenWith(SensorErrors.TYPICAL);

    assertThat(second).isEqualTo(first);
  }

  private static Pose2d drivenWith(SensorErrors errors) {
    RobotSim.setEnabled(false);
    try (DriveSimHarness robot = new DriveSimHarness(ExampleRobot.SIM, errors)) {
      robot.drive(1.0, 0.8, 0.2, 0.5);
      return robot.drive.getPose();
    }
  }

  @Test
  void aStepTakesTime() {
    assertThatThrownBy(() -> robot(SensorErrors.NONE).simulation.step(0, 0))
        .hasMessageContaining("a step takes time");
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.localization.DriveConditions;
import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.EstimatorConfigBuilder;
import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizationGate;
import com.michaelgrundvig.frc.localization.SlipCheck;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.localization.YawSource;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.runtime.sim.FakeCamera;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.SensorErrorsAccelErrorsBuilder;
import com.michaelgrundvig.frc.simulation.SensorErrorsBuilder;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.ImuMountBuilder;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.system.RobotController;

/**
 * The pose estimator on the simulated robot, with a season's wear on its wheels and gyro, and a
 * camera that sees the robot's true pose late and noisily. Does the estimate hold to the truth
 * where odometry alone drifts, and does it survive a shove, a slipping wheel, and a misread tag?
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class LocalizationSimTest {
  /** A match-like route: sprints, strafes, turns while driving, and a stop. */
  private static final double[][] ROUTE = {
    {3, 1, 0, 0},
    {2, 0, 1, 0.5},
    {3, -1, 0, 0},
    {2, 0.5, -0.5, -0.8},
    {3, -0.7, 0.4, 0.3},
    {2, 0, 0, 0}
  };

  private @Nullable DriveSimHarness harness;

  @AfterEach
  void tearDown() {
    if (harness != null) {
      harness.close();
    }
  }

  private DriveSimHarness robot(SensorErrors errors) {
    return robot(errors, EstimatorConfig.DEFAULTS);
  }

  private DriveSimHarness robot(SensorErrors errors, EstimatorConfig config) {
    if (harness != null) {
      harness.close();
    }
    harness = new DriveSimHarness(ExampleRobot.SIM, errors, config);
    return harness;
  }

  /** A good camera: 3 cm of noise, results 40 ms late, 30 frames a second. */
  private static FakeCamera camera(DriveSimHarness robot) {
    return new FakeCamera(robot.simulation, 0.03, 0.04, 30, 3);
  }

  /** Drives the route and returns the worst position error seen at the end of each leg. */
  private static double driveTheRoute(DriveSimHarness robot) {
    double worst = 0;
    for (double[] leg : ROUTE) {
      robot.drive(leg[0], leg[1], leg[2], leg[3]);
      worst = Math.max(worst, robot.positionError());
    }
    return worst;
  }

  /**
   * 15 s of driving: odometry alone drifts past 15 cm; with the camera, within the design's 8 cm
   * while moving. At the ends of the legs that turn while driving, the heavy robot is still
   * changing course, its wheels disagreeing with each other as they speed up and slow down to their
   * new speeds, so the estimate doubts them and leans on the camera's noisy fixes: 6.0 cm off at
   * worst, about the 6.4 cm it claims.
   */
  @Test
  void odometryAloneDriftsAndTheCameraHoldsTheEstimateToTheTruth() {
    double blind = driveTheRoute(robot(SensorErrors.TYPICAL));

    DriveSimHarness seeing = robot(SensorErrors.TYPICAL);
    seeing.watchWith(camera(seeing));
    double withCamera = driveTheRoute(seeing);

    assertThat(blind).as("odometry alone, worst").isGreaterThan(0.15);
    assertThat(withCamera).as("with the camera, worst").isLessThan(0.08);
    assertThat(seeing.positionError()).as("with the camera, at the end").isLessThan(0.02);
    LocalizationEstimate estimate = seeing.estimator.estimate();
    assertThat(estimate.mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(estimate.health().accepted()).isGreaterThan(300);
  }

  /**
   * The route without a camera, four seeds of the season-worn robot's noise, checked at the end of
   * every leg. After the first, three seconds at full stick, the estimate is 16 cm off claiming 10:
   * the worn tread puts its reckoning off by a share of every meter, where the estimator allows for
   * an error growing with the root of the distance (docs/testing.md, what it proves). After the
   * rest it claims far more than it's off: through the route's hard changes of direction the
   * simulated wheels disagree with each other for half a second, each reaching its new speed at its
   * own pace, and every step's disagreement counts as slip that keeps its direction, but the
   * simulated robot moves as the wheels' fit says, so their disagreement puts it nowhere. This
   * holds it to no worse.
   */
  @Test
  void blindTheEstimateIsNeverFarSurerThanItsRight() {
    for (long seed = 1; seed <= 4; seed++) {
      DriveSimHarness robot =
          robot(SensorErrorsBuilder.builder(SensorErrors.TYPICAL).seed(seed).build());
      for (double[] leg : ROUTE) {
        robot.drive(leg[0], leg[1], leg[2], leg[3]);
        double off = robot.positionError();
        assertThat(robot.estimator.estimate().covariance().largestPositionStdDev())
            .as("seed %d, %.2f m off", seed, off)
            .isGreaterThan(off / 1.6);
      }
    }
  }

  /**
   * Knocked 60 cm sideways, unfelt: the camera, reading two tags, disagrees, then keeps
   * disagreeing, and wins. Two tags agreeing are what tell a shove from a misread, which is of one
   * tag.
   */
  @Test
  void aShoveIsCaughtAndTheEstimateRecovers() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.watchWith(camera(robot).seeing(List.of(26, 27)));
    robot.drive(1.0, 0.3, 0, 0);

    robot.simulation.shove(new Transform2d(0, 0.6, Rotation2d.ZERO));
    List<LocalizationEstimate.Mode> modes = new ArrayList<>();
    for (int i = 0; i < 15; i++) {
      robot.drive(0.02, 0.3, 0, 0);
      modes.add(robot.estimator.estimate().mode());
    }

    assertThat(modes).as("it knew it was lost").contains(LocalizationEstimate.Mode.LOST);
    assertThat(robot.estimator.estimate().health().recoveries()).isEqualTo(1);
    assertThat(robot.positionError()).as("0.3 s after the shove").isLessThan(0.08);
    robot.drive(0.2, 0.3, 0, 0);
    assertThat(robot.positionError()).as("0.5 s after").isLessThan(0.04);
  }

  /** Knocked 60 cm sideways hard enough to feel: the camera is believed at once, no recovery. */
  @Test
  void aHitIsFeltSoTheCameraIsBelievedAtOnce() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.watchWith(camera(robot));
    robot.drive(1.0, 0.3, 0, 0);

    robot.simulation.hit(new Transform2d(0, 0.6, Rotation2d.ZERO));
    List<LocalizationEstimate.Mode> modes = new ArrayList<>();
    for (int i = 0; i < 8; i++) {
      robot.drive(0.02, 0.3, 0, 0);
      modes.add(robot.estimator.estimate().mode());
    }

    assertThat(modes).doesNotContain(LocalizationEstimate.Mode.LOST);
    assertThat(robot.estimator.estimate().health().recoveries()).isZero();
    assertThat(robot.positionError()).as("0.16 s after the hit").isLessThan(0.05);
  }

  /** Half a second on a bump, front wheels in the air: the tilt says their travel is suspect. */
  private static double crossABump(DriveSimHarness robot) {
    robot.drive(1.0, 0.5, 0, 0); // 1.1 m/s
    robot.simulation.tilt(Math.toRadians(-20), 0); // nose up
    robot.simulation.grip(0.4);
    robot.drive(0.5, 0.5, 0, 0);
    robot.simulation.tilt(0, 0);
    robot.simulation.grip(1);
    robot.drive(0.5, 0.5, 0, 0);
    return robot.positionError();
  }

  @Test
  void onABumpTheTiltDiscountsWheelsThatLostTheFloor() {
    EstimatorConfig ignoringTilt =
        EstimatorConfigBuilder.builder(EstimatorConfig.DEFAULTS)
            .tiltStartDegrees(80)
            .tiltZeroDegrees(89)
            .build();
    double trusting = crossABump(robot(SensorErrors.NONE, ignoringTilt));
    DriveSimHarness robot = robot(SensorErrors.NONE);
    double discounting = crossABump(robot);

    assertThat(trusting).as("trusting every wheel turn").isGreaterThan(0.2);
    assertThat(discounting).as("discounting by the tilt").isLessThan(trusting / 3);
    assertThat(robot.estimator.estimate().covariance().xStdDev())
        .as("and it knows it may be off")
        .isGreaterThan(discounting);
  }

  @Test
  void theDrivetrainReadsTiltAndJoltsFromItsSensors() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    robot.idle(0.1);
    boolean disabled = robot.localization.conditions().disabled();

    robot.drive(0.04, 1.0, 0, 0); // full stick from a standstill
    robot.simulation.tilt(0.3, 0.1);
    robot.simulation.hit(new Transform2d(0.1, 0, Rotation2d.ZERO));
    robot.drive(0.02, 1.0, 0, 0);
    DriveConditions hit = robot.localization.conditions();
    robot.drive(0.02, 1.0, 0, 0);

    assertThat(disabled).isTrue();
    assertThat(hit.tiltRadians()).isCloseTo(Math.acos(Math.cos(0.3) * Math.cos(0.1)), within(1e-9));
    assertThat(hit.collision()).isTrue();
    assertThat(hit.disabled()).isFalse();
    assertThat(robot.localization.conditions().collision()).as("a loop later").isFalse();
    assertThat(robot.localization.pitchRadians()).isEqualTo(0.3);
    assertThat(robot.localization.rollRadians()).isEqualTo(0.1);
  }

  /**
   * Tilted where it stands, and driven over the field's bump at 0.6 and full stick: what the IMU
   * feels of gravity as it tilts is the tilt, not a slide or a hit. Taken the wrong way, standing
   * tilted 15° would feel 5 m/s² the robot never moved by.
   */
  @Test
  void tiltedOrCrossingABumpIsNoSlideAndNoHit() {
    Pose2d beforeTheBump = new Pose2d(3.0, 5.5, Rotation2d.ZERO);
    for (double stick : new double[] {0.6, 1.0}) {
      DriveSimHarness robot = onTheField();
      robot.idle(0.02);
      robot.simulation.place(beforeTheBump);
      robot.localization.resetPose(beforeTheBump);
      double[] most = slidesAndHits(robot);
      robot.drive(3.0, stick, 0, 0);
      robot.drive(1.0, 0, 0, 0);

      assertThat(Math.toDegrees(most[2])).as("pitched over the bump").isGreaterThan(5);
      assertThat(most[0]).as("slip at " + stick).isZero();
      assertThat(most[1]).as("collision at " + stick).isZero();
    }

    DriveSimHarness standing = robot(SensorErrors.TYPICAL);
    double[] most = slidesAndHits(standing);
    standing.drive(0.5, 0, 0, 0);
    standing.simulation.tilt(Math.toRadians(15), Math.toRadians(8));
    standing.drive(1.0, 0, 0, 0);
    standing.simulation.tilt(0, 0);
    standing.drive(0.5, 0, 0, 0);

    assertThat(most[0]).as("slip, tilted").isZero();
    assertThat(most[1]).as("collision, tilted").isZero();
  }

  /** The simulated robot on the field's terrain, with a season's wear. */
  private DriveSimHarness onTheField() {
    if (harness != null) {
      harness.close();
    }
    harness =
        new DriveSimHarness(
            ExampleRobot.SIM,
            SensorErrors.TYPICAL,
            EstimatorConfig.DEFAULTS,
            RobotSim.FIELD.terrain());
    return harness;
  }

  /**
   * Watches every loop from now: the most slip, the most of a collision, and the most pitch the
   * drivetrain read, in that order.
   */
  private static double[] slidesAndHits(DriveSimHarness robot) {
    double[] most = new double[3];
    robot.eachLoop(
        () -> {
          DriveConditions conditions = robot.localization.conditions();
          most[0] = Math.max(most[0], conditions.slipMetersPerSecond());
          most[1] = Math.max(most[1], conditions.collisionShare());
          most[2] = Math.max(most[2], Math.abs(robot.localization.pitchRadians()));
        });
    return most;
  }

  /** Drives the route with the gyro unplugged a second in; returns the heading error at the end. */
  private static double loseTheGyro(DriveSimHarness robot, boolean backupToo) {
    robot.drive(1.0, 0.5, 0, 0.3);
    robot.simulation.disconnectGyro(true);
    robot.simulation.disconnectBackupGyro(backupToo);
    driveTheRoute(robot);
    return Math.abs(robot.headingErrorDegrees());
  }

  /** The gyro unplugged mid-match: the backup IMU keeps the heading, where the wheels drift. */
  @Test
  void whenTheGyroIsLostTheBackupImuKeepsTheHeading() {
    double wheels = loseTheGyro(robot(SensorErrors.TYPICAL), true);
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    double backup = loseTheGyro(robot, false);

    assertThat(backup).as("heading error on the backup IMU, degrees").isLessThan(1.5);
    assertThat(wheels).as("on the wheels alone").isGreaterThan(3 * backup);
    assertThat(robot.localization.yawSource()).contains(YawSource.BACKUP);
    assertThat(robot.localization.isUsingBackupAlertActive()).isTrue();
    assertThat(robot.localization.isGyroDisconnectedAlertActive()).isFalse();
    assertThat(robot.estimator.estimate().mode())
        .as("no camera has placed it, and the backup is no IMU fault")
        .isEqualTo(LocalizationEstimate.Mode.INITIALIZING);
  }

  /** Switching between IMUs never shows up as a turn, though their zeros differ. */
  @Test
  void switchingImusDoesNotTurnTheEstimate() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.drive(1.0, 0.3, 0, 0.5);
    double before = robot.headingErrorDegrees();

    robot.simulation.disconnectGyro(true);
    robot.drive(0.1, 0.3, 0, 0.5);
    double onBackup = robot.headingErrorDegrees();
    robot.simulation.disconnectGyro(false);
    robot.drive(0.1, 0.3, 0, 0.5);

    assertThat(onBackup).isCloseTo(before, within(0.2));
    assertThat(robot.headingErrorDegrees()).isCloseTo(before, within(0.2));
    assertThat(robot.localization.yawSource()).contains(YawSource.GYRO);
    assertThat(robot.localization.isUsingBackupAlertActive()).isFalse();
  }

  /** A gyro that freezes still reports: only the backup IMU's disagreement catches it. */
  @Test
  void aFrozenGyroIsCaughtByTheBackupImu() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.drive(1.0, 0, 0, 0.4);
    boolean beforeFreezing = robot.localization.isImuDisagreementAlertActive();

    robot.simulation.freezeGyro(true);
    robot.drive(0.3, 0, 0, 0.4);

    assertThat(beforeFreezing).isFalse();
    assertThat(robot.localization.isImuDisagreementAlertActive()).isTrue();
  }

  @Test
  void drivingHardIsNeverMistakenForAHit() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    boolean[] felt = {false};
    robot.eachLoop(() -> felt[0] |= robot.localization.conditions().collision());

    driveTheRoute(robot);

    assertThat(felt[0]).isFalse();
  }

  /**
   * A wheel spins out: the wheels disagree, so the estimate trusts them less, and the camera wins.
   */
  @Test
  void aSlippingWheelWidensTheUncertaintyAndTheCameraCorrectsIt() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    robot.watchWith(camera(robot));
    robot.idle(1.0);

    robot.simulation.spin(0, 3.0); // 15 cm of wheel travel the floor never saw
    robot.idle(0.02);
    double falseMove = robot.positionError();
    double uncertainty = robot.estimator.estimate().covariance().xStdDev();
    robot.idle(0.5);

    assertThat(falseMove).as("odometry was fooled").isGreaterThan(0.02);
    assertThat(uncertainty).as("but knew to doubt itself").isGreaterThan(falseMove);
    assertThat(robot.positionError()).as("half a second later").isLessThan(0.02);
  }

  /**
   * Full stick from a standstill: the drive motors sit at their current limit for most of a second
   * while the robot speeds up, its wheels gripping. That's no slip, so the estimate stays as sure
   * as odometry over that distance is.
   */
  @Test
  void aHardLaunchAtTheCurrentLimitIsNoSlip() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.idle(0.2);
    Translation2d start = robot.simulation.truePose().getTranslation();
    double[] most = new double[2]; // the most slip any loop saw, and the most suspect travel
    boolean[] hit = {false};
    robot.eachLoop(
        () -> {
          most[0] = Math.max(most[0], robot.localization.conditions().slipMetersPerSecond());
          most[1] = Math.max(most[1], robot.estimator.estimate().health().suspectTravelMeters());
          hit[0] |= robot.localization.conditions().collision();
        });

    robot.drive(0.5, 1.0, 0, 0);
    double speed = robot.drive.getRobotRelativeVelocities().vx;
    robot.drive(1.0, 1.0, 0, 0);

    double traveled = robot.simulation.truePose().getTranslation().getDistance(start);
    double odometry = EstimatorConfig.DEFAULTS.translationStdDevPerMeter() * Math.sqrt(traveled);
    assertThat(speed).as("speeding up at its current limit, about 4.4 m/s²").isBetween(1.9, 2.3);
    assertThat(most[0]).as("no slip").isZero();
    assertThat(most[1]).as("nothing suspect").isLessThan(0.005);
    assertThat(hit[0]).as("no collision").isFalse();
    assertThat(robot.estimator.estimate().covariance().largestPositionStdDev())
        .as("as sure as odometry over %.1f m", traveled)
        .isBetween(0.9 * odometry, 1.2 * odometry);
  }

  /** The uncertainty, the error, and the most slip seen, at the end of a run. */
  private record Outcome(double stdDev, double error, double mostSlip) {}

  /** Runs a script on a fresh robot, and says how it ended. */
  private Outcome outcome(SensorErrors errors, Consumer<DriveSimHarness> script) {
    DriveSimHarness robot = robot(errors);
    double[] most = {0};
    robot.eachLoop(
        () -> most[0] = Math.max(most[0], robot.localization.conditions().slipMetersPerSecond()));
    script.accept(robot);
    return new Outcome(
        robot.estimator.estimate().covariance().largestPositionStdDev(),
        robot.positionError(),
        most[0]);
  }

  /** A second at half stick, then a second with the front left wheel spinning past the floor. */
  private Outcome spinningAWheel(double metersPerSecond) {
    return outcome(
        SensorErrors.NONE,
        robot -> {
          robot.drive(1.0, 0.5, 0, 0);
          robot.simulation.spinAt(0, metersPerSecond);
          robot.drive(1.0, 0.5, 0, 0);
          robot.simulation.spinAt(0, 0);
        });
  }

  /**
   * One wheel spinning faster than the floor moves it: the wheels disagree with each other and with
   * the gyro, and the estimate doubts itself by about as far as the wheel put it off.
   */
  @Test
  void aWheelSpinningPastTheFloorIsDoubtedAsFarAsItPutTheEstimateOff() {
    Outcome gripping = spinningAWheel(0);
    Outcome slow = spinningAWheel(0.5);
    Outcome fast = spinningAWheel(1.0);

    assertThat(gripping.mostSlip()).isZero();
    assertThat(slow.mostSlip()).as("the gyro sees it").isPositive();
    assertThat(slow.stdDev()).isGreaterThan(3 * gripping.stdDev());
    assertThat(fast.stdDev()).as("the doubt grows with it").isGreaterThan(1.6 * slow.stdDev());
    for (Outcome spun : List.of(slow, fast)) {
      assertThat(spun.error()).as("off by").isBetween(0.5 * spun.stdDev(), 1.5 * spun.stdDev());
    }
  }

  /**
   * Two wheels spinning together past the floor, the front pair at 0.5 m/s: the fit takes half of
   * it into the travel, 25 cm in the second, while the wheels' disagreement says less than half of
   * that, and neither the gyro (a pair either side of the robot doesn't turn it) nor the
   * accelerometer (the fit's 0.25 m/s is under its tolerance) sees it. Every step's disagreement
   * counts as slip that keeps its direction, so the estimate doubts itself by about half as far as
   * it's off: a known limit (docs/testing.md), but no less than that.
   */
  @Test
  void aPairOfWheelsSpinningIsDoubtedAboutHalfAsFarAsItPutTheEstimateOff() {
    Outcome pair =
        outcome(
            SensorErrors.NONE,
            robot -> {
              robot.drive(1.0, 0.5, 0, 0);
              robot.simulation.spinAt(0, 0.5);
              robot.simulation.spinAt(1, 0.5);
              robot.drive(1.0, 0.5, 0, 0);
            });

    assertThat(pair.error()).as("off").isCloseTo(0.25, within(0.01));
    assertThat(pair.stdDev()).as("claimed").isGreaterThan(0.4 * pair.error());
  }

  /** A launch at full stick with only this share of the wheels' rolling moving the robot. */
  private Outcome launchingWithGrip(double grip) {
    return outcome(
        SensorErrors.TYPICAL,
        robot -> {
          robot.idle(0.2);
          robot.simulation.grip(grip);
          robot.drive(1.5, 1.0, 0, 0);
        });
  }

  /**
   * Every wheel spinning in a launch: they agree with each other, but the accelerometer feels the
   * robot speed up slower than they do. The doubt grows the less they grip. It covers only what the
   * accelerometer felt: the robot falling ever further behind wheels at a steady speed is the
   * cameras' to find.
   */
  @Test
  void wheelsSpinningInALaunchAreDoubtedTheMoreTheLessTheyGrip() {
    Outcome gripping = launchingWithGrip(1);
    Outcome threeQuarters = launchingWithGrip(0.75);
    Outcome half = launchingWithGrip(0.5);

    assertThat(gripping.mostSlip()).isZero();
    assertThat(threeQuarters.mostSlip()).isPositive();
    assertThat(threeQuarters.stdDev()).isGreaterThan(2 * gripping.stdDev());
    assertThat(half.stdDev()).isGreaterThan(1.5 * threeQuarters.stdDev());
    assertThat(half.error()).as("the slide").isGreaterThan(threeQuarters.error());
  }

  /** The simulator's robot as every robot starts: its IMUs' accelerometers not yet checked. */
  private static RobotConfig unchecked() {
    ImuMount gyro =
        ImuMountBuilder.builder(ExampleRobot.SIM.definition().imus().gyro())
            .accelerometerChecked(false)
            .build();
    ImuMount backup =
        ImuMountBuilder.builder(ExampleRobot.SIM.definition().imus().backup().orElseThrow())
            .accelerometerChecked(false)
            .build();
    return RobotConfigBuilder.builder(ExampleRobot.SIM)
        .definition(
            RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                .imus(Imus.builder().gyro(gyro).backup(backup).build())
                .build())
        .build();
  }

  /**
   * A robot whose accelerometers aren't checked yet, as every robot starts, and the gyro's turned a
   * quarter turn on its mount besides: wheels spinning in a launch at half grip are doubted about
   * as a checked robot doubts them, its accelerometer compared with the wheels by size, once it has
   * stood long enough to learn its bias, as it does on the field before a match. Launched 1.5 s,
   * the robot is 2.4 m behind its wheels.
   */
  @Test
  void anUncheckedRobotStillDoubtsWheelsSpinningInALaunch() {
    Outcome checked = launchingWithGrip(0.5);
    SensorErrors turned =
        SensorErrors.TYPICAL.withAccelerometer(
            SensorErrorsAccelErrorsBuilder.builder(SensorErrors.AccelErrors.PIGEON_2)
                .turnedDegrees(90)
                .build());
    if (harness != null) {
      harness.close();
    }
    harness = new DriveSimHarness(unchecked(), turned);
    DriveSimHarness robot = harness;
    robot.idle(SlipCheck.Settings.DEFAULTS.steadySeconds() + 0.5);
    robot.simulation.grip(0.5);
    robot.drive(1.5, 1.0, 0, 0);

    double claimed = robot.estimator.estimate().covariance().largestPositionStdDev();
    assertThat(robot.positionError()).as("off").isGreaterThan(2);
    assertThat(claimed)
        .as("claimed, against the checked robot's %.2f m", checked.stdDev())
        .isGreaterThan(0.8 * checked.stdDev());
  }

  /**
   * Stood 3 s and then drove 5 s, turning, unchecked: how sure the estimate was at the end of each,
   * the most slip each saw, the slip at the end, and whether the alert to check the IMU's
   * orientation was on, standing.
   */
  private record StoodThenDrove(
      double standingStdDev,
      double standingSlip,
      boolean alert,
      double stdDev,
      double mostSlip,
      double slip) {}

  /** An unchecked robot whose gyro's accelerometer reads this much more, along its own axes. */
  private StoodThenDrove standThenDrive(double leakForward, double leakLeft) {
    SensorErrors.AccelErrors part = SensorErrors.AccelErrors.PIGEON_2;
    SensorErrors leaking =
        SensorErrors.TYPICAL.withAccelerometer(
            SensorErrorsAccelErrorsBuilder.builder(part)
                .biasForward(part.biasForward() + leakForward)
                .biasLeft(part.biasLeft() + leakLeft)
                .build());
    if (harness != null) {
      harness.close();
    }
    harness = new DriveSimHarness(unchecked(), leaking);
    DriveSimHarness robot = harness;
    double[] most = {0};
    robot.eachLoop(
        () -> most[0] = Math.max(most[0], robot.localization.conditions().slipMetersPerSecond()));
    robot.drive(3.0, 0, 0, 0);
    double standingStdDev = robot.estimator.estimate().covariance().largestPositionStdDev();
    double standingSlip = most[0];
    boolean alert = robot.localization.isImuOrientationAlertActive();
    most[0] = 0;
    robot.drive(5.0, 0.6, 0.3, 0.5);
    return new StoodThenDrove(
        standingStdDev,
        standingSlip,
        alert,
        robot.estimator.estimate().covariance().largestPositionStdDev(),
        most[0],
        robot.localization.conditions().slipMetersPerSecond());
  }

  /**
   * A robot not yet checked whose gyro's case sits tilted on its mount, its definition left level
   * (the Pigeon's mount pose set in Tuner X, but not the orientation in the definition): gravity
   * leaks into the accelerometer's level axes, 1.7 m/s² at 10°, and a full g on a Pigeon mounted on
   * a vertical plate. Standing 3 s, the slip check learns it as the bias, claims no slide
   * meanwhile, and an alert says to check the orientation; driving after, it claims no more than
   * with a level IMU. Taken for a slide, the 10° case claimed 4 m standing and 10 m after driving.
   */
  @Test
  void anUncheckedImuTiltedOnItsMountIsLearnedAsItsBias() {
    StoodThenDrove level = standThenDrive(0, 0);
    StoodThenDrove tilted = standThenDrive(1.7, 0);
    StoodThenDrove vertical = standThenDrive(0, ImuReadings.STANDARD_GRAVITY);

    assertThat(level.alert()).as("level").isFalse();
    for (StoodThenDrove leaking : List.of(tilted, vertical)) {
      assertThat(leaking.standingSlip()).as("no slide standing").isZero();
      assertThat(leaking.standingStdDev()).as("standing").isLessThan(0.05);
      assertThat(leaking.slip()).as("none held after driving").isZero();
      assertThat(leaking.mostSlip()).as("driving").isLessThanOrEqualTo(level.mostSlip());
      assertThat(leaking.stdDev())
          .as("after driving, against the level IMU's %.3f m", level.stdDev())
          .isLessThan(1.1 * level.stdDev());
      assertThat(leaking.alert()).as("the alert").isTrue();
    }
  }

  /** Standing, then pushed sideways at this speed for a second, its wheels skidding. */
  private Outcome pushedSideways(double metersPerSecond) {
    return outcome(
        SensorErrors.TYPICAL,
        robot -> {
          robot.drive(0.5, 0, 0, 0);
          robot.simulation.push(new Translation2d(0, metersPerSecond));
          robot.drive(1.0, 0, 0, 0);
          robot.simulation.push(Translation2d.ZERO);
          robot.drive(0.5, 0, 0, 0);
        });
  }

  /**
   * Pushed sideways by another robot, its wheels never turning: only the accelerometer feels it, as
   * the push gets going. The slide stays seen while the push holds, let go no faster than the
   * accelerometer's bias could have added it, and when the push lets go the accelerometer feels the
   * robot slow, and it's over. The doubt grows with the push; how far the robot slid, the cameras
   * find.
   */
  @Test
  void aPushSidewaysIsDoubtedTheMoreTheFasterItSlides() {
    Outcome standing = pushedSideways(0);
    Outcome pushed = pushedSideways(1.0);
    Outcome shoved = pushedSideways(2.0);

    assertThat(standing.mostSlip()).isZero();
    assertThat(pushed.mostSlip()).isPositive();
    assertThat(pushed.stdDev()).isGreaterThan(5 * standing.stdDev());
    assertThat(shoved.stdDev()).isGreaterThan(2 * pushed.stdDev());
    assertThat(shoved.error()).as("slid").isGreaterThan(1.5);
  }

  /**
   * How the estimate came back once the camera saw again: how far off it had been, how long after
   * the camera's return it came within 10 cm of the truth for good, and the loops the scoring gate
   * let through while it was more than 15 cm off, with the worst of them.
   */
  private record Recovery(
      double worstError, double secondsBack, int gateOpenWhileOff, double worstWhileOpen) {}

  /**
   * A camera watches the robot standing, long enough for the slip check to learn its
   * accelerometer's bias (as it does before a match), is blind through what {@code blind} does, and
   * watches again through what {@code seen} does.
   */
  private Recovery seenAgain(Consumer<DriveSimHarness> blind, Consumer<DriveSimHarness> seen) {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    FakeCamera camera = camera(robot);
    // The loop, the last loop more than 10 cm off, and the loops the gate let through off.
    int[] loops = new int[3];
    double[] worst = new double[2]; // the worst error, and the worst the gate let through
    robot.eachLoop(
        () -> {
          loops[0]++;
          double error = robot.positionError();
          worst[0] = Math.max(worst[0], error);
          if (error > 0.10) {
            loops[1] = loops[0];
          }
          if (error > 0.15 && LocalizationGate.SCORING.allows(robot.estimator.estimate())) {
            loops[2]++;
            worst[1] = Math.max(worst[1], error);
          }
        });
    robot.watchWith(camera);
    robot.drive(SlipCheck.Settings.DEFAULTS.steadySeconds() + 0.5, 0, 0, 0);
    robot.stopWatching();
    blind.accept(robot);
    robot.watchWith(camera);
    int back = loops[0];
    seen.accept(robot);
    return new Recovery(
        worst[0], Math.max(0, loops[1] - back) * RobotLoop.PERIOD_SECONDS, loops[2], worst[1]);
  }

  /**
   * Pushed sideways at 1 m/s for a second while the camera is blind, and seen again half a second
   * after the push lets go: a meter off, and back within 10 cm in the camera's first fixes. The
   * scoring gate should let through no loop more than 15 cm off; it lets through 3, under 20 cm, as
   * the push gets going and the camera's last fix is still recent: the slip check sees the slide
   * only once it's past its tolerance, and the doubt it adds grows only as fast as the slide, from
   * then (docs/testing.md, known limits). The bounds are what it does today, so it does no worse;
   * where in a loop the slip check catches the slide hangs on the accelerometer's noise, and with
   * the IMUs' noise drawn as offsets from the seed it let through 2, under 18 cm.
   */
  @Test
  void pushedWhileTheCameraIsBlindTheEstimateComesBackWhenItSees() {
    Recovery pushed =
        seenAgain(
            robot -> {
              robot.simulation.push(new Translation2d(0, 1.0));
              robot.drive(1.0, 0, 0, 0);
              robot.simulation.push(Translation2d.ZERO);
              robot.drive(0.5, 0, 0, 0);
            },
            robot -> robot.drive(2.0, 0, 0, 0));

    assertThat(pushed.worstError()).as("a meter off").isGreaterThan(0.9);
    assertThat(pushed.secondsBack()).as("back within 10 cm, seconds").isLessThanOrEqualTo(0.1);
    assertThat(pushed.gateOpenWhileOff()).as("loops let through off").isLessThanOrEqualTo(3);
    assertThat(pushed.worstWhileOpen()).as("the worst let through").isLessThan(0.20);
  }

  /**
   * A launch at half grip while the camera is blind, seen again as the robot stops: 2.3 m off, and
   * back within 10 cm in half a second, through the camera's noise while the estimate still doubts
   * its wheels. The scoring gate should let through no loop more than 15 cm off; it lets through 2,
   * under 18 cm, as the launch starts, for the same reason as a push. The bounds are what it does
   * today, so it does no worse.
   */
  @Test
  void aLaunchAtHalfGripWhileTheCameraIsBlindComesBackWhenItSees() {
    Recovery launched =
        seenAgain(
            robot -> {
              robot.simulation.grip(0.5);
              robot.drive(1.5, 1.0, 0, 0);
            },
            robot -> {
              robot.drive(1.0, 0, 0, 0);
              robot.simulation.grip(1);
              robot.drive(2.0, 0, 0, 0);
            });

    assertThat(launched.worstError()).as("meters off").isGreaterThan(2);
    assertThat(launched.secondsBack()).as("back within 10 cm, seconds").isLessThanOrEqualTo(0.5);
    assertThat(launched.gateOpenWhileOff()).as("loops let through off").isLessThanOrEqualTo(2);
    assertThat(launched.worstWhileOpen()).as("the worst let through").isLessThan(0.18);
  }

  /**
   * One misread tag, 1.5 m off and claiming 3 cm of certainty. WPILib's estimator has no gate (its
   * documentation asks teams to filter measurements themselves), so it jumps; this one rejects it.
   */
  @Test
  void aMisreadTagIsGatedWhereWpilibsEstimatorJumps() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    FakeCamera camera = camera(robot);
    SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(ExampleRobot.SIM.definition().drivetrain().moduleTranslations());
    SwerveDrivePoseEstimator wpilib =
        new SwerveDrivePoseEstimator(
            kinematics, robot.drive.gyroYaw(), robot.drive.modulePositions(), Pose2d.ZERO);
    double[] worst = new double[2];
    robot.eachLoop(
        () -> {
          long now = RobotController.getTime();
          wpilib.updateWithTime(now / 1e9, robot.drive.gyroYaw(), robot.drive.modulePositions());
          for (VisionMeasurement seen : camera.poll(now)) {
            robot.service.camera(seen);
            wpilib.addVisionMeasurement(
                seen.pose(), seen.timestampNanos() / 1e9, VecBuilder.fill(0.03, 0.03, 1e6));
          }
          Pose2d truth = robot.simulation.truePose();
          worst[0] = Math.max(worst[0], robot.positionError());
          worst[1] =
              Math.max(
                  worst[1],
                  wpilib
                      .getEstimatedPosition()
                      .getTranslation()
                      .getDistance(truth.getTranslation()));
        });
    robot.idle(1.0);
    worst[0] = 0;
    worst[1] = 0;

    camera.lieOnce(new Transform2d(1.5, 0, Rotation2d.ZERO));
    robot.idle(0.3);

    assertThat(worst[0]).as("ours, worst after the misread").isLessThan(0.05);
    assertThat(worst[1]).as("WPILib's, worst after the misread").isGreaterThan(0.5);
    assertThat(robot.frames)
        .filteredOn(
            frame -> frame.measured().filter(weighed -> weighed.innovationMeters() > 1).isPresent())
        .extracting(FrameVerdict::accepted)
        .containsExactly(false);
  }
}

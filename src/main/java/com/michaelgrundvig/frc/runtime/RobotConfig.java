package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.Optional;

/**
 * Everything the code needs to know about one physical robot. Each robot the team builds is one
 * instance in {@code Robots}. Every value is checked when the record is built, so a bad value fails
 * at startup, and before that in the tests, which build every robot.
 *
 * <p>What the robot is, its definition, is read by everything that needs to know the robot:
 * localization, the simulator, the views. The rest is how this program drives it, wires it, and
 * judges what its sensors say.
 *
 * @param definition what the robot is: its body, drivetrain, cameras, IMUs and joints, where each
 *     is mounted, and, in each one's block, how it's driven and wired
 * @param vision how far the cameras' images are trusted, and which are left out: the thresholds and
 *     uncertainties, some tunable live
 * @param estimator how the pose estimator weighs odometry, the gyro, and the cameras
 */
@ConfigRecord
public record RobotConfig(
    RobotDefinition definition, VisionConfig vision, EstimatorConfig estimator) {
  public RobotConfig {
    Optional<SwerveWiring> drivetrainWiring = definition.drivetrain().wiring();
    Checks.on(RobotConfig.class, definition.name())
        .findings("definition", definition::findings)
        .check(
            "definition.drivetrain.wiring",
            () -> drivetrainWiring.map(Drive::whyNotDriven).orElse(null))
        .findings("joints", () -> Joints.problems(definition))
        .findings("definition.interlocks", () -> Joints.unenforceable(definition))
        .orThrow();
  }

  /** Starts building a robot's configuration, one named value at a time. */
  public static RobotConfigBuilder.DefinitionStage builder() {
    return RobotConfigBuilder.builder();
  }

  /** How the drivetrain is driven, from its block: its limits and gains, tunable live. */
  public DrivetrainControl drivetrain() {
    return definition.drivetrain().control().orElseThrow();
  }

  /**
   * Where the drivetrain's devices are on the CAN bus, and how its encoders read, from its block:
   * empty for a robot that only runs in simulation, whose drivetrain then stays still on real
   * hardware.
   */
  public Optional<SwerveWiring> drivetrainWiring() {
    return definition.drivetrain().wiring();
  }

  /** Which robot this is, as its logs name it: its definition's name. */
  public String name() {
    return definition.name();
  }

  /**
   * The same robot under another name: a practice robot of its design, which {@code Robots.all}
   * lists beside it, {@code COMPETITION.named("practice").with(...)}.
   */
  public RobotConfig named(String name) {
    return redefined(definition.named(name));
  }

  /**
   * The same robot with some of its joints wired or tuned otherwise, as its definition's {@link
   * RobotDefinition#with(Joint...)}: {@code COMPETITION.with(PIVOT.motors(motors ->
   * motors.on(TALON_FX.can(33))))}.
   */
  public RobotConfig with(Joint... changed) {
    return redefined(definition.with(changed));
  }

  /**
   * The same robot with some of its sensors wired otherwise, as its definition's {@link
   * RobotDefinition#with(Sensor...)}.
   */
  public RobotConfig with(Sensor... changed) {
    return redefined(definition.with(changed));
  }

  /**
   * The same robot with some of its cameras mounted otherwise, as its definition's {@link
   * RobotDefinition#with(CameraConfig...)}.
   */
  public RobotConfig with(CameraConfig... changed) {
    return redefined(definition.with(changed));
  }

  /**
   * The same robot, its drivetrain wired or driven otherwise, as its definition's {@link
   * RobotDefinition#with(DrivetrainGeometry)}.
   */
  public RobotConfig with(DrivetrainGeometry changed) {
    return redefined(definition.with(changed));
  }

  private RobotConfig redefined(RobotDefinition changed) {
    return RobotConfigBuilder.builder(this).definition(changed).build();
  }
}

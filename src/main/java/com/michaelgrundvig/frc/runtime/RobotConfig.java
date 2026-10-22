package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.localization.DefaultLocalizer;
import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.Localizer;
import com.michaelgrundvig.frc.localization.LocalizerMaker;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.Optional;

/**
 * Everything the code needs to know about one physical robot. Each robot a team builds is one of
 * these in {@code Robots}. Every value is checked as the record is built, so a bad value fails as
 * the program starts, and fails before that in the tests, which build every robot.
 *
 * <p>The definition says what the robot is. Everything that needs to know the robot reads it:
 * localization, the simulator, the views. The other values say how this program judges what the
 * robot's sensors tell it.
 *
 * @param definition what the robot is: its body, drivetrain, cameras, IMUs and joints, where each
 *     is mounted, and, in each one's block, how it's driven and wired
 * @param vision how far the cameras' images are trusted, and which are left out: the thresholds and
 *     uncertainties, some tunable live
 * @param estimator how the default localizer's pose estimator weighs odometry, the gyro, and the
 *     cameras. A robot that chooses another localizer keeps it, unused.
 * @param localizer what works out where the robot is, for a robot that chooses its own: {@code
 *     .localizer(WpilibLocalizer::new)} for WPILib's pose estimator, or a maker of the team's own
 *     (docs/localizers.md). Left out, the robot gets the default localizer.
 */
@ConfigRecord
public record RobotConfig(
    RobotDefinition definition,
    VisionConfig vision,
    EstimatorConfig estimator,
    Optional<LocalizerMaker> localizer) {
  public RobotConfig {
    Optional<SwerveWiring> drivetrainWiring = definition.drivetrain().wiring();
    // The definition's own findings come first, then what this program cannot drive: a drivetrain
    // wired in a way the template has no modules for, a joint it cannot run, and an interlock it
    // cannot enforce.
    Checks.on(RobotConfig.class, definition.name())
        .findings("definition", definition::refusals)
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

  /** How the drivetrain is driven, from its block: its limits and gains, which the pit can tune. */
  public DrivetrainControl drivetrain() {
    return definition.drivetrain().control().orElseThrow();
  }

  /**
   * Where the drivetrain's devices are on the CAN bus, and how its encoders read, from its block. A
   * robot that only runs in simulation has none, and on real hardware its drivetrain would then
   * stay still.
   */
  public Optional<SwerveWiring> drivetrainWiring() {
    return definition.drivetrain().wiring();
  }

  /**
   * Makes the robot's localizer for a field: the one the robot chose, or the default localizer
   * weighing its sources as {@link #estimator} says.
   */
  public Localizer localizerOn(FieldModel field) {
    return localizer.orElse(DefaultLocalizer.with(estimator)).make(definition, field);
  }

  /** Which robot this is, as its logs name it: its definition's name. */
  public String name() {
    return definition.name();
  }

  /**
   * The same robot under another name. This is how a practice robot of the same design is made, for
   * {@code Robots.all} to list beside it: {@code COMPETITION.named("practice").with(...)}.
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

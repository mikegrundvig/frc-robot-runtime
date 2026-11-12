package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.localization.DefaultLocalizer;
import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.LocalizationGate;
import com.michaelgrundvig.frc.localization.Localizer;
import com.michaelgrundvig.frc.localization.LocalizerMaker;
import com.michaelgrundvig.frc.localization.SlipCheck;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.pitwall.CameraIds;
import com.michaelgrundvig.frc.runtime.subsystems.drive.SwerveDrive;
import com.michaelgrundvig.frc.runtime.subsystems.localization.ScoringWhenNotJudged;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Everything the code needs to know about one physical robot. Each robot a team builds is one of
 * these in {@code Robots}. Every value is checked as the record is built, so a bad value fails as
 * the program starts, and fails before that in the tests, which build every robot.
 *
 * <p>The definition says what the robot is. Everything that needs to know the robot reads it:
 * localization, the simulator, the views. The other values say how this program judges what the
 * robot's sensors tell it, and each has the template's default: {@code RobotConfig.of(definition)}
 * is a whole robot. One robot is made from another by naming it and saying what differs: {@code
 * DRIVING.named("vision-bench").withCameras(CAMERA_1, CAMERA_2).withCoprocessors(VISION_1,
 * VISION_2)}, the {@code with...} verbs adding parts and the {@code with(...)} ones changing the
 * robot's own.
 *
 * @param definition what the robot is: its body, drivetrain, cameras, IMUs and joints, where each
 *     is mounted, and, in each one's block, how it's driven and wired
 * @param vision how far the cameras' images are trusted, and which are left out: the thresholds and
 *     uncertainties, some tunable live; {@code VisionConfig.defaults()} unless given
 * @param estimator how the default localizer's pose estimator weighs odometry, the gyro, and the
 *     cameras: {@code EstimatorConfig.defaults()} unless given. A robot that chooses another
 *     localizer keeps it, unused.
 * @param localizer what works out where the robot is, for a robot that chooses its own: {@code
 *     .withLocalizer(WpilibLocalizer::new)} for WPILib's pose estimator, or a maker of the team's
 *     own (docs/localizers.md). Left out, the robot gets the default localizer.
 * @param scoringWhenNotJudged what the scoring gate says while the robot's localizer reports too
 *     little for it to judge the estimate by, which the default localizer never does: {@code
 *     NOT_READY} unless given, so scoring waits; {@code .withScoringWhenNotJudged(READY)} lets it
 *     through, as a robot without a gate does. An alert says the gate isn't judging either way
 * @param coprocessors the robot's coprocessors, each by its Spotter agent's address ({@code
 *     "10.12.34.11"} for team 1234's first). The robot watches each, pushes each Visor's pack while
 *     any camera is on Visor, and installs and configures Visor on them (docs/visor.md). None
 *     unless given.
 * @param slipCheck how closely the default localizer wants the wheels to agree with the IMU before
 *     it doubts them for sliding over the floor together: {@code SlipCheck.Settings.defaults()}
 *     unless given. Raise its {@code speedToleranceMetersPerSecond} if clean driving shows a slide
 *     (docs/calibration.md, the accelerometer check)
 * @param scoringGate whether an estimate is good enough to score from: {@code
 *     LocalizationGate.SCORING} unless given. Set its position limit from the game: how far off a
 *     shot or a placement may land (docs/values.md)
 */
@ConfigRecord
public record RobotConfig(
    RobotDefinition definition,
    @RecordBuilder.Initializer(value = "defaults", source = VisionConfig.class) VisionConfig vision,
    @RecordBuilder.Initializer(value = "defaults", source = EstimatorConfig.class)
        EstimatorConfig estimator,
    Optional<LocalizerMaker> localizer,
    Optional<ScoringWhenNotJudged> scoringWhenNotJudged,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<String> coprocessors,
    @RecordBuilder.Initializer(value = "defaults", source = SlipCheck.Settings.class)
        SlipCheck.Settings slipCheck,
    @RecordBuilder.Initializer(value = "SCORING", source = LocalizationGate.class)
        LocalizationGate scoringGate) {
  public RobotConfig {
    // The definition's parts are checked against each other here, on the finished robot, so its
    // with... verbs could come in any order.
    definition.checked();
    coprocessors = List.copyOf(coprocessors);
    Optional<SwerveWiring> drivetrainWiring = definition.drivetrain().wiring();
    // The definition's own findings come first, then what this program cannot drive: a drivetrain
    // wired in a way the template has no modules for, a joint it cannot run, and an interlock it
    // cannot enforce; a board or a camera whose card on Pitwall would take another card's name; and
    // a camera whose name is too long for Pitwall's World view to draw it by.
    List<PitCards.Card> pitCards = PitCards.of(definition, coprocessors);
    Checks.on(RobotConfig.class, definition.name())
        .findings("definition", definition::refusals)
        .check(
            "definition.drivetrain.wiring",
            () -> drivetrainWiring.map(SwerveDrive::whyNotDriven).orElse(null))
        .findings("joints", () -> Joints.problems(definition))
        .findings("definition.interlocks", () -> Joints.unenforceable(definition))
        .check("coprocessors", () -> PitCards.whyNamesClash(pitCards, PitCards.Source.COPROCESSOR))
        .check("definition.cameras", () -> PitCards.whyNamesClash(pitCards, PitCards.Source.CAMERA))
        .check("definition.cameras", () -> CameraIds.whyTooLong(definition.cameras()))
        .orThrow();
  }

  /**
   * A robot as its definition says, with the template's settings for everything else: how far its
   * cameras are trusted, how its pose is estimated, and the rest. A robot that weighs its images or
   * its pose otherwise starts from {@link #builder} instead.
   */
  public static RobotConfig of(RobotDefinition definition) {
    return builder().definition(definition).build();
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
   * weighing its sources as {@link #estimator} and {@link #slipCheck} say.
   */
  public Localizer localizerOn(FieldModel field) {
    return localizer.orElse(DefaultLocalizer.with(estimator, slipCheck)).make(definition, field);
  }

  /**
   * What the scoring gate says while the robot's localizer reports too little for it to judge the
   * estimate by: as given, or {@link ScoringWhenNotJudged#NOT_READY}, so scoring waits.
   */
  public ScoringWhenNotJudged scoringWhenNotJudgedOrDefault() {
    return scoringWhenNotJudged.orElse(ScoringWhenNotJudged.NOT_READY);
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
   * The same robot with more cameras, as its definition's {@link
   * RobotDefinition#withCameras(CameraConfig...)}: {@code
   * DRIVING.named("vision-bench").withCameras(CAMERA_1, CAMERA_2)}.
   */
  public RobotConfig withCameras(CameraConfig... added) {
    return redefined(definition.withCameras(added));
  }

  /**
   * The same robot on a localizer of its own choosing: {@code .withLocalizer(WpilibLocalizer::new)}
   * for WPILib's pose estimator, or a maker of the team's own (docs/localizers.md). One it had is
   * replaced. Without it, the robot gets the default localizer.
   */
  public RobotConfig withLocalizer(LocalizerMaker localizer) {
    return RobotConfigBuilder.builder(this).localizer(localizer).build();
  }

  /**
   * The same robot with what its scoring gate says while its localizer reports too little to judge
   * the pose by: {@code .withScoringWhenNotJudged(ScoringWhenNotJudged.READY)} lets scoring through
   * without the gate. Without it, scoring waits.
   */
  public RobotConfig withScoringWhenNotJudged(ScoringWhenNotJudged whenNotJudged) {
    return RobotConfigBuilder.builder(this)
        .scoringWhenNotJudged(Optional.of(whenNotJudged))
        .build();
  }

  /**
   * The same robot with more coprocessors, each by its Spotter agent's address, after its own:
   * {@code .withCoprocessors("10.12.34.11", "10.12.34.12")}.
   */
  public RobotConfig withCoprocessors(String... added) {
    List<String> all = new ArrayList<>(coprocessors);
    all.addAll(List.of(added));
    return RobotConfigBuilder.builder(this).coprocessors(all).build();
  }

  /**
   * The same robot, its drivetrain wired or driven otherwise, as its definition's {@link
   * RobotDefinition#with(Drivetrain)}.
   */
  public RobotConfig with(Drivetrain changed) {
    return redefined(definition.with(changed));
  }

  private RobotConfig redefined(RobotDefinition changed) {
    return RobotConfigBuilder.builder(this).definition(changed).build();
  }
}

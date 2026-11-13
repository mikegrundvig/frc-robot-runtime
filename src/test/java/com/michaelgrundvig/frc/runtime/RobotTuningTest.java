package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.ParameterDocs;
import com.michaelgrundvig.frc.pitwall.RequestOutcome;
import com.michaelgrundvig.frc.pitwall.TunableTee;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfig.Software;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/** Every robot's configuration works with live tuning, within the limits its mechanisms declare. */
class RobotTuningTest {
  /** The guide's robots' committed values can be dialed in; a team's are checked in its tests. */
  @Test
  void everyRobotsCommittedValuesCanBeDialedIn() {
    for (RobotConfig robot : ExampleRobot.all()) {
      RobotChecks.committedValuesCanBeDialedIn(robot);
    }
    RobotChecks.committedValuesCanBeDialedIn(onVisor());
  }

  /**
   * The guide's robot with three cameras on Visor: one at the defaults (a fixed 1 ms, its own
   * gain), one with its own exposure and gain, and one on the camera's own automatic exposure and
   * gain.
   */
  private static RobotConfig onVisor() {
    List<CameraConfig> cameras = ExampleRobot.SIM.definition().cameras();
    return ExampleRobot.SIM.with(
        on(cameras.get(0), Software.visor()),
        on(cameras.get(1), Software.visor().exposure(Milliseconds.of(2.5)).gain(40)),
        on(cameras.get(2), Software.visor().camerasOwnExposure()));
  }

  private static CameraConfig on(CameraConfig camera, Software software) {
    return camera.withSoftware(software);
  }

  /**
   * A camera on Visor tunes what its definition fixes, each knob within the camera's own reach, and
   * gives the builder call to write the tuned value back; a camera on its own automatic exposure
   * and gain has nothing to tune.
   */
  @Test
  void aVisorCamerasExposureAndGainAreTunedWhereItsDefinitionFixesThem() {
    var cameras = RobotContainer.tunings(onVisor()).cameras();
    List<String> names =
        ExampleRobot.SIM.definition().cameras().stream().map(CameraConfig::name).toList();
    assertThat(cameras).containsOnlyKeys(names.get(0), names.get(1));

    record Case(String camera, String path, double committed, double min, double max) {}
    // At 30 frames a second, one frame is 33.3 ms: a longer exposure would slow the frame rate.
    List<Case> cases =
        List.of(
            new Case(names.get(0), "exposure.time", 1, 0.1, 1000.0 / 30),
            new Case(names.get(1), "exposure.time", 2.5, 0.1, 1000.0 / 30),
            new Case(names.get(1), "gain.value", 40, 0, 65_535));
    for (Case c : cases) {
      LiveConfig<Software.Visor> tuning = Objects.requireNonNull(cameras.get(c.camera()));
      assertThat(tuning.name()).isEqualTo(c.camera() + " camera");
      assertThat(tuning.committedValue(c.path())).as(c.path()).isEqualTo(c.committed());
      LiveConfig.Knob knob =
          tuning.knobs().stream().filter(k -> k.path().equals(c.path())).findFirst().orElseThrow();
      assertThat(knob.limits().min()).as(c.path()).isCloseTo(c.min(), within(1e-9));
      assertThat(knob.limits().max()).as(c.path()).isCloseTo(c.max(), within(1e-9));
    }
    assertThat(Objects.requireNonNull(cameras.get(names.get(0))).knobs())
        .as("its own gain: nothing to tune")
        .extracting(LiveConfig.Knob::path)
        .containsExactly("exposure.time");

    LiveConfig<Software.Visor> tuning = Objects.requireNonNull(cameras.get(names.get(1)));
    assertThat(tuning.set(Map.of("exposure.time", 1.5, "gain.value", 64.0))).isTrue();
    assertThat(tuning.get()).isEqualTo(Software.visor().exposure(Milliseconds.of(1.5)).gain(64));
    assertThat(tuning.changes())
        .extracting(LiveConfig.Change::builderCall)
        .containsExactly(".exposure(Milliseconds.of(1.5))", ".gain(64)");
    assertThat(tuning.whyNot(Map.of("gain.value", 40.5)))
        .isEqualTo(names.get(1) + " camera: gain.value takes a whole number, got 40.5");
    assertThat(tuning.whyNot(Map.of("exposure.time", 50.0)))
        .startsWith(names.get(1) + " camera: exposure.time: 50 ms is above its limit");
  }

  /** The container's list of tunings is the one every check reads: it misses none of them. */
  @Test
  void theListOfTuningsHasEveryOne() {
    var tunings = RobotContainer.tunings(ExampleRobot.SIM);

    // The drivetrain's and vision's, and one for each driven joint; the guide's cameras are on
    // PhotonVision, so none of theirs.
    assertThat(tunings.cameras()).isEmpty();
    assertThat(tunings.all())
        .hasSize(2 + tunings.joints().size())
        .doesNotHaveDuplicates()
        .extracting(LiveConfig::name)
        .doesNotHaveDuplicates();
  }

  @Test
  void aGainIsTunedByItsPathWithinItsLimits() {
    var drivetrain = RobotContainer.tunings(ExampleRobot.SIM).drivetrain();

    assertThat(drivetrain.set(Map.of("gains.steerKp", 18.5))).isTrue();
    assertThat(drivetrain.get().gains().steerKp()).isEqualTo(18.5);
    assertThat(drivetrain.whyNot(Map.of("gains.steerKp", 500.0)))
        .isEqualTo("drivetrain: gains.steerKp: 500 is above its limit of 100");
  }

  /**
   * The page says what each tunable value means, from its record's Javadoc, which the build writes
   * into the program as it compiles: every one needs a {@code @param} saying what it does.
   */
  @Test
  void everyTunableValueSaysWhatItMeans() {
    for (LiveConfig<?> live : RobotContainer.tunings(onVisor()).all()) {
      for (LiveConfig.Knob knob : live.knobs()) {
        String component = knob.path().substring(knob.path().lastIndexOf('.') + 1);
        assertThat(live.meaning(knob.path()))
            .as(
                "%s %s says nothing: add @param %s to %s's Javadoc, saying what it does and in"
                    + " what unit",
                live.name(),
                knob.path(),
                component,
                live.documentation().value(knob.path()).record())
            .isNotBlank();
      }
    }
  }

  /**
   * The build copies each configuration record's Javadoc into the program as it compiles, for the
   * page to show beside its values: what the record is, and what each value means.
   */
  @Test
  void aRecordsJavadocIsInTheProgram() {
    ParameterDocs gains = ParameterDocs.of(DrivetrainControl.SwerveGains.class);

    assertThat(gains.summary()).startsWith("Drive: velocity control on the motor controller");
    assertThat(gains.params().get("steerKp"))
        .isEqualTo("volts per radian of error: Tuner X's, per turn of the module, over 2π");
  }

  /** Within its limits isn't enough: the records' own checks have the last word, and say which. */
  @Test
  void aValueTheRecordsRejectIsRefusedNamingTheRecordsValue() {
    var vision = Vision.tuning(ExampleRobot.SIM.vision());

    assertThat(vision.whyNot(Map.of("maxAmbiguity", 0.0)))
        .startsWith("vision: VisionConfig.maxAmbiguity: 0 ");
  }

  /**
   * The page shows a refusal beside the value it concerns. A record's message names the value the
   * way {@code Checks} writes it, from the record that checked it ({@code
   * VisionConfig.maxAmbiguity}), and Pitwall reads that back to the value's tunable.
   */
  @Test
  void aRecordsRefusalIsTiedToTheTunableItNames() {
    var tee = new TunableTee(new MockTunableBackend(), () -> null, inputs -> {}, false);
    TunableRegistry.registerBackend("", tee);
    try {
      TunableRegistry.publish("Vision/config", Vision.tuning(ExampleRobot.SIM.vision()));
      tee.update();

      var outcome = tee.requestChanges(Map.of("/Vision/config/maxAmbiguity", 0.0));
      tee.update();

      assertThat(outcome.getNow(null).reasons())
          .singleElement()
          .satisfies(reason -> {
            assertThat(reason.layer()).isEqualTo(RequestOutcome.Layer.RECORD);
            assertThat(reason.by()).isEqualTo("VisionConfig");
            assertThat(reason.paths()).containsExactly("/Vision/config/maxAmbiguity");
          });
    } finally {
      TunableRegistry.reset();
    }
  }

  /**
   * The drivetrain's top speed is tuned live, but never past what its motors and gearing can reach,
   * which the robot's definition says and the pit can't change: its tuning limits end at the free
   * speed, and the page shows why beside the value. Every robot's committed top speed is within
   * them ({@link #everyRobotsCommittedValuesCanBeDialedIn}).
   */
  @Test
  void theTopSpeedIsHeldToTheDrivetrainsFreeSpeed() {
    var drivetrain = RobotContainer.tunings(ExampleRobot.SIM).drivetrain();

    assertThat(drivetrain.knobs())
        .filteredOn(knob -> knob.path().equals("limits.maxLinearSpeed"))
        .singleElement()
        .satisfies(knob ->
            assertThat(knob.limits().max())
                .isEqualTo(
                    ExampleRobot.SIM.definition().drivetrain().freeSpeed().in(MetersPerSecond)));
    assertThat(drivetrain.set(Map.of("limits.maxLinearSpeed", 4.5))).isTrue();
    assertThat(drivetrain.whyNot(Map.of("limits.maxLinearSpeed", 6.0)))
        .isEqualTo("drivetrain: limits.maxLinearSpeed: 6 m/s is above its limit of 4.77991 m/s");
    assertThat(drivetrain.set(Map.of("limits.maxLinearSpeed", 6.0))).isFalse();
    assertThat(drivetrain.get().limits().maxLinearSpeed().orElseThrow().in(MetersPerSecond))
        .isEqualTo(4.5);
  }

  /** Vision's thresholds are tuned like the drivetrain's gains: its configuration, live. */
  @Test
  void visionsThresholdsAreTunedThroughItsConfiguration() {
    var vision = Vision.tuning(ExampleRobot.SIM.vision());

    assertThat(vision.knobs())
        .extracting(LiveConfig.Knob::path)
        .containsExactly(
            "maxAmbiguity",
            "maxHeightErrorMeters",
            "linearStdDevBaseline",
            "angularStdDevBaseline",
            "distanceExponent",
            "outOfCalibrationMeters",
            "latencyMarginSeconds");
    assertThat(vision.set(Map.of("maxAmbiguity", 0.1))).isTrue();
    assertThat(vision.get().maxAmbiguity()).isEqualTo(0.1);
  }
}

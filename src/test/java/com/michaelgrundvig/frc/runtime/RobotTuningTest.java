package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.Outcome;
import com.michaelgrundvig.frc.pitwall.ParameterDocs;
import com.michaelgrundvig.frc.pitwall.TunableTee;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import java.util.Map;
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
  }

  /** The container's list of tunings is the one every check reads: it misses none of them. */
  @Test
  void theListOfTuningsHasEveryOne() {
    var tunings = RobotContainer.tunings(ExampleRobot.SIM);

    // The drivetrain's and vision's, and one for each driven joint.
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
    for (LiveConfig<?> live : RobotContainer.tunings(ExampleRobot.SIM).all()) {
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
    ParameterDocs gains = ParameterDocs.of(DrivetrainControl.Gains.class);

    assertThat(gains.summary()).startsWith("Drive: velocity control on the motor controller");
    assertThat(gains.params().get("steerKp")).isEqualTo("volts per radian of error");
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
          .satisfies(
              reason -> {
                assertThat(reason.layer()).isEqualTo(Outcome.Layer.RECORD);
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
        .satisfies(
            knob ->
                assertThat(knob.limits().max())
                    .isEqualTo(
                        ExampleRobot.SIM
                            .definition()
                            .drivetrain()
                            .freeSpeed()
                            .in(MetersPerSecond)));
    assertThat(drivetrain.set(Map.of("limits.maxLinearSpeed", 4.5))).isTrue();
    assertThat(drivetrain.whyNot(Map.of("limits.maxLinearSpeed", 6.0)))
        .isEqualTo("drivetrain: limits.maxLinearSpeed: 6 m/s is above its limit of 4.72868 m/s");
    assertThat(drivetrain.set(Map.of("limits.maxLinearSpeed", 6.0))).isFalse();
    assertThat(drivetrain.get().limits().maxLinearSpeed().in(MetersPerSecond)).isEqualTo(4.5);
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
            "mountToleranceMeters",
            "latencyMarginSeconds");
    assertThat(vision.set(Map.of("maxAmbiguity", 0.1))).isTrue();
    assertThat(vision.get().maxAmbiguity()).isEqualTo(0.1);
  }
}

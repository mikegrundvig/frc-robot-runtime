package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizerReport;
import com.michaelgrundvig.frc.localization.Verdict;
import com.michaelgrundvig.frc.runtime.examples.LocalizerExamples;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.math.geometry.Pose2d;

/**
 * A robot that chose its localizer: the whole robot in the simulator, on WPILib's pose estimator,
 * which reports none of a localizer's options. It drives and takes its cameras' measurements as any
 * robot does, and everything that would have shown an option says it is not reported.
 */
@SimulationTest
class ChosenLocalizerSimTest {
  /** The guide's robot, on WPILib's pose estimator. */
  private static final RobotConfig ON_WPILIB = LocalizerExamples.ON_WPILIB;

  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void aRobotOnWpilibsEstimatorDrivesAndSaysWhatIsNotReported() {
    sim.build(ON_WPILIB, SensorErrors.TYPICAL);
    sim.loops(1);
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1.0); // gamepad Y is negative when pushed forward
    sim.driver().notifyNewData();
    List<FrameVerdict> measured = new ArrayList<>();
    for (int loop = 0; loop < 75; loop++) {
      sim.loop();
      for (FrameVerdict frame : sim.robot().frames()) {
        if (frame.measured().isPresent()) {
          measured.add(frame);
        }
      }
    }

    RobotContainer robot = sim.robot();
    Pose2d truth = sim.simulation().truePose();
    assertThat(robot.drive().getPose().getTranslation().getDistance(truth.getTranslation()))
        .as("6 m driven on worn wheels, the cameras correcting WPILib's estimator")
        .isLessThan(0.3);

    // The pit's lines say what is not reported, and show no number in its place.
    assertThat(robot.localizationStatus())
        .isEqualTo("What the pose rests on and how sure it is: not reported by this localizer");
    assertThat(robot.scoring())
        .isEqualTo("Not judged: what the pose rests on is not reported by this localizer");
    assertThat(robot.readyToScore()).as("a gate that can't judge holds nothing back").isTrue();
    assertThat(robot.cameras())
        .contains("cameras connected")
        .contains("measurements given, how many were accepted is not reported")
        .doesNotContain("%");

    // Every measurement was given to the localizer, and nothing claims it was accepted.
    assertThat(measured).isNotEmpty();
    assertThat(measured).allSatisfy(frame -> assertThat(frame.verdict()).isEqualTo(Verdict.GIVEN));

    LocalizerReport report = robot.localization().report();
    assertThat(report.covariance()).isEmpty();
    assertThat(report.mode()).isEmpty();
    assertThat(report.health()).isEmpty();
    assertThat(robot.localization().yawSource()).isEmpty();

    // The default localizer's own estimate isn't there to ask for.
    assertThat(robot.localization().runsTheDefaultLocalizer()).isFalse();
    assertThatThrownBy(robot::estimate).isInstanceOf(IllegalStateException.class);
    assertThatThrownBy(() -> robot.localization().conditions())
        .isInstanceOf(IllegalStateException.class);
  }

  @Test
  void aRobotMayTellWpilibsEstimatorHowFarToTrustItsOdometry() {
    sim.build(LocalizerExamples.ON_WPILIB_TIGHTER, SensorErrors.NONE);
    sim.loops(25);

    assertThat(sim.robot().localization().runsTheDefaultLocalizer()).isFalse();
    assertThat(
            sim.robot()
                .drive()
                .getPose()
                .getTranslation()
                .getDistance(sim.simulation().truePose().getTranslation()))
        .isLessThan(0.1);
  }

  @Test
  void aMountIsNotCalibratedAgainstAPoseNothingIsKnownOf() {
    sim.build(ON_WPILIB, SensorErrors.NONE);
    sim.loops(5);
    String camera = ExampleRobot.SIM.definition().cameras().get(0).name();

    sim.scheduler().schedule(sim.robot().calibration().cameraMount(camera));
    sim.loops(5);

    assertThat(sim.robot().calibration().lastResult())
        .isEqualTo(
            camera
                + ": not calibrated: what the pose rests on, or how sure it is, is not reported by"
                + " this localizer");
  }

  @Test
  void theBackButtonsHeadingResetReachesAnyLocalizer() {
    sim.build(ON_WPILIB, SensorErrors.NONE);
    sim.loops(5);
    Pose2d before = sim.robot().drive().getPose();

    sim.robot().faceDownfield();

    Pose2d after = sim.robot().drive().getPose();
    assertThat(after.getTranslation().getDistance(before.getTranslation())).isLessThan(1e-6);
    assertThat(after.getRotation().getDegrees())
        .isCloseTo(sim.robot().matchField().driverForward().getDegrees(), within(1e-6));
  }
}

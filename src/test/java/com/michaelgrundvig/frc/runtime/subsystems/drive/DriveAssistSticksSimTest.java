package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.world.field.Heading;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * When an assist ({@link Drive#facing}) reads the driver's sticks: each time it starts, not when it
 * is built. So a robot program may build its assists before or after it hands the drive the sticks
 * ({@link Drive#takeDriver}). In the simulator, on a drive that starts without them.
 */
@ExtendWith(RobotSim.class)
class DriveAssistSticksSimTest {
  private static final Rotation2d NINETY = Rotation2d.fromDegrees(90);

  private static final String NO_STICKS =
      "the drivetrain has no driver's sticks: the robot hands them over as it builds the drive,"
          + " drive.takeDriver(...)";

  private final DriveSimHarness harness = DriveSimHarness.withoutDriver(ExampleRobot.SIM);
  private final Drive drive = harness.drive;

  @AfterEach
  void tearDown() {
    harness.close();
  }

  private void run(double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      harness.loop();
    }
  }

  private Pose2d truth() {
    return harness.simulation.truePose();
  }

  /**
   * An assist built as the robot starts, before the drive has the sticks, is built all the same.
   * Run before the sticks are handed over, it ends as it starts and its gate says why. Run again
   * once they are, the very same assist drives from them and holds its heading.
   */
  @Test
  void anAssistBuiltBeforeTheSticksAreHandedOverDrivesOnceTheyAre() {
    DriveAssist assist = drive.facing(Heading.field(NINETY));
    Gate aimed = assist.aimed();
    RobotSim.setEnabled(true);
    harness.sticks(1, 0, 0);

    harness.scheduler.schedule(assist);
    run(0.5);

    assertThat(harness.scheduler.isRunning(assist)).as("no sticks yet: ended").isFalse();
    assertThat(truth().getTranslation().getNorm()).as("no sticks yet: stayed put").isLessThan(0.01);
    assertThat(aimed.getAsBoolean()).isFalse();
    assertThat(aimed.why()).isEqualTo("aimed at field 90° is false: " + NO_STICKS);

    harness.takeDriver();
    harness.scheduler.schedule(assist);
    run(2);

    assertThat(harness.scheduler.isRunning(assist)).as("with the sticks: runs on").isTrue();
    assertThat(truth().getX()).as("with the sticks: drives along the field").isGreaterThan(3.0);
    assertThat(truth().getRotation().getDegrees())
        .as("with the sticks: holds its heading")
        .isCloseTo(90, within(2.0));
    harness.sticks(0, 0, 0);
    run(1);
    assertThat(aimed.getAsBoolean()).as("the gate made before the sticks came").isTrue();
  }

  /**
   * On a drive that is never handed the sticks, an assist is refused each time it runs, as it was
   * when it was refused as it was built: it ends as it starts, the robot stays put, and its gate
   * holds false saying what to do.
   */
  @Test
  void anAssistRunWithNoSticksEverHandedOverIsRefused() {
    DriveAssist assist = drive.facing(Heading.field(NINETY));
    Gate aimed = assist.aimed();
    RobotSim.setEnabled(true);
    harness.sticks(1, 0, 0);

    for (int run = 1; run <= 2; run++) {
      harness.scheduler.schedule(assist);
      harness.loop();
      assertThat(harness.scheduler.isRunning(assist)).as("run %d took its loop", run).isTrue();
      harness.loop();
      assertThat(harness.scheduler.isRunning(assist)).as("run %d ended", run).isFalse();
    }
    run(0.5);

    assertThat(truth().getTranslation().getNorm()).as("stayed put").isLessThan(0.01);
    assertThat(truth().getRotation().getDegrees()).isCloseTo(0, within(0.5));
    assertThat(aimed.getAsBoolean()).isFalse();
    assertThat(aimed.why()).isEqualTo("aimed at field 90° is false: " + NO_STICKS);
  }

  /**
   * Driving with the sticks is the same: the command built before the drive has the sticks ends as
   * it starts and says why, and the very same command drives once they are handed over.
   */
  @Test
  void drivingWithTheSticksBuiltBeforeTheyAreHandedOverDrivesOnceTheyAre() {
    Command driven = drive.driven();
    RobotSim.setEnabled(true);
    harness.sticks(1, 0, 0);

    harness.scheduler.schedule(driven);
    run(0.5);

    assertThat(harness.scheduler.isRunning(driven)).as("no sticks yet: ended").isFalse();
    assertThat(truth().getTranslation().getNorm()).as("no sticks yet: stayed put").isLessThan(0.01);

    harness.takeDriver();
    harness.scheduler.schedule(driven);
    run(2);

    assertThat(harness.scheduler.isRunning(driven)).as("with the sticks: runs on").isTrue();
    assertThat(truth().getX()).as("with the sticks: drives along the field").isGreaterThan(3.0);
  }
}

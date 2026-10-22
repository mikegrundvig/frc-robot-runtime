package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;

/** Values tuned while the robot runs reach every module's motor controllers. */
@ExtendWith(RobotSim.class)
class DriveTuningTest {
  /** A module that remembers what its motor controllers were last given. */
  private static final class RecordingModuleIO implements ModuleIO {
    DrivetrainControl.@Nullable Gains gains;
    DrivetrainControl.@Nullable Limits limits;

    @Override
    public void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
      this.gains = gains;
      this.limits = limits;
    }
  }

  @Test
  void tunedGainsAndLimitsReachEveryModule() {
    DrivetrainControl committed = ExampleRobot.SIM.drivetrain();
    DrivetrainControl tuned =
        DrivetrainControlBuilder.builder(committed)
            .limits(
                DrivetrainControlLimitsBuilder.builder(committed.limits())
                    .driveStatorCurrent(Amps.of(15))
                    .build())
            .gains(DrivetrainControlGainsBuilder.builder(committed.gains()).driveKp(3.0).build())
            .build();
    var config = new AtomicReference<>(committed);
    RecordingModuleIO[] modules = {
      new RecordingModuleIO(),
      new RecordingModuleIO(),
      new RecordingModuleIO(),
      new RecordingModuleIO()
    };

    try (Drive drive =
        new Drive(
            "TuningTest",
            DriveHardware.builder()
                .modules(List.of(modules))
                .gyro(new GyroIO() {})
                .backupImu(new GyroIO() {})
                .odometryLock(new ReentrantLock())
                .build(),
            ExampleRobot.SIM.definition(),
            RobotSim.massOf(ExampleRobot.SIM.definition()),
            config::get,
            Scheduler.createIndependentScheduler(),
            () -> Pose2d.ZERO)) {
      config.set(tuned);
      drive.periodic(false);
    }

    for (RecordingModuleIO module : modules) {
      assertThat(module.gains).isEqualTo(tuned.gains());
      assertThat(module.limits).isEqualTo(tuned.limits());
    }
  }
}

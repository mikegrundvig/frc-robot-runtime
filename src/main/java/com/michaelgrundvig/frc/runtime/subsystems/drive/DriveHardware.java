package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static com.michaelgrundvig.frc.world.config.Rules.size;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * The drivetrain's hardware, as its IO layer reads it: real devices on a robot, the simulator's in
 * the simulator, nothing in a replay (the log supplies the inputs). Built by name, so the gyro and
 * the backup IMU can't trade places: {@code DriveHardware.builder().modules(...).gyro(...)...}.
 *
 * @param modules the swerve modules: front left, front right, back left, back right
 * @param gyro the main gyro, which owns the robot's heading
 * @param backupImu a second IMU (Systemcore's own), checked against the gyro and standing in while
 *     it's lost; {@code new GyroIO() {}} for none
 * @param odometryLock held while the drivetrain reads its IO, when a background thread records the
 *     devices' high-rate readings, so every device's readings stay in step
 */
@ConfigRecord
public record DriveHardware(
    List<ModuleIO> modules, GyroIO gyro, GyroIO backupImu, Lock odometryLock) {
  public DriveHardware {
    modules = List.copyOf(modules);
    Checks.on(DriveHardware.class).field("modules", modules, size(4)).orThrow(); // swerve's four
  }

  public static DriveHardwareBuilder.ModulesStage builder() {
    return DriveHardwareBuilder.builder();
  }

  /**
   * No hardware: every module and IMU reads nothing. In a replay the log supplies the inputs; on a
   * robot whose configuration has no drivetrain wiring, the drivetrain stays still rather than
   * guess at motors.
   */
  public static DriveHardware none() {
    return builder()
        .modules(
            List.of(new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}))
        .gyro(new GyroIO() {})
        .backupImu(new GyroIO() {})
        .odometryLock(new ReentrantLock())
        .build();
  }
}

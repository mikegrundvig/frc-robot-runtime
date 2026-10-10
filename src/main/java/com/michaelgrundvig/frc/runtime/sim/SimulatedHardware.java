package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveHardware;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;

/**
 * The simulator's hardware: the simulated world's drivetrain, joints, sensors, PDH and cameras,
 * each part the robot's definition has, wired or not, built with the world from the definition. The
 * world stays its maker's to step and close.
 */
public final class SimulatedHardware implements Hardware {
  private final SimulatedWorld world;

  /**
   * @param world the world the robot drives in, simulated from the robot's definition
   */
  public SimulatedHardware(SimulatedWorld world) {
    this.world = world;
  }

  @Override
  public DriveHardware drive() {
    return world.drivetrainHardware();
  }

  @Override
  public JointIO joint(Joint joint) {
    return world.joints().io(joint);
  }

  @Override
  public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
    return world.joints().absoluteEncoder(joint);
  }

  @Override
  public SensorIO sensor(Sensor.Placed sensor) {
    return world.sensors().io(sensor);
  }

  @Override
  public PdhIO pdh() {
    return world.pdh();
  }

  @Override
  public List<CameraIO> cameras() {
    return List.of(world.cameras());
  }
}

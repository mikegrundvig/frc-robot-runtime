package com.michaelgrundvig.frc.runtime.hardware;

import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveHardware;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;

/**
 * Everything the robot's code reads and drives, made for the mode the program runs in: the
 * drivetrain's modules and IMUs, each joint's motor controllers and absolute encoder, each switch
 * and rangefinder, the PDH, and each camera. Behind it the robot's code is the same on the robot,
 * in the simulator and in a replay (AdvantageKit's IO pattern). The robot's container picks one as
 * it starts:
 *
 * <ul>
 *   <li>on a robot, {@link RobotHardware}: the devices its definition's wiring names, and nothing,
 *       said by an alert, for a part with no wiring, which then stays still rather than guess;
 *   <li>in the simulator, {@code com.michaelgrundvig.frc.runtime.sim.SimulatedHardware}: the
 *       simulated world's;
 *   <li>in a replay, {@link #replay}: nothing, since the log has every reading.
 * </ul>
 *
 * <p>Each part is asked for once, as the robot is built: the drivetrain's, then the joints' as
 * {@link Joints} asks for them, then the cameras'.
 */
public interface Hardware extends Joints.Hardware, AutoCloseable {
  /** The drivetrain's modules and IMUs. */
  DriveHardware drive();

  @Override
  JointIO joint(Joint joint);

  @Override
  AbsoluteEncoderIO absoluteEncoder(Joint joint);

  @Override
  SensorIO sensor(Sensor.Placed sensor);

  @Override
  PdhIO pdh();

  /** Each camera's, in the definition's order. */
  List<CameraIO> cameras();

  /** Releases the devices it opened, and its alerts. */
  @Override
  default void close() {}

  /**
   * A replay's: nothing, so every reading comes from the log.
   *
   * @param definition what the robot is: how many cameras it has
   */
  static Hardware replay(RobotDefinition definition) {
    List<CameraIO> cameras = new ArrayList<>();
    for (int i = 0; i < definition.cameras().size(); i++) {
      cameras.add(new CameraIO() {});
    }
    return new Hardware() {
      @Override
      public DriveHardware drive() {
        return DriveHardware.none();
      }

      @Override
      public JointIO joint(Joint joint) {
        return new JointIO() {};
      }

      @Override
      public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
        return new AbsoluteEncoderIO() {};
      }

      @Override
      public SensorIO sensor(Sensor.Placed sensor) {
        return new SensorIO() {};
      }

      @Override
      public PdhIO pdh() {
        return new PdhIO() {};
      }

      @Override
      public List<CameraIO> cameras() {
        return List.copyOf(cameras);
      }
    };
  }
}

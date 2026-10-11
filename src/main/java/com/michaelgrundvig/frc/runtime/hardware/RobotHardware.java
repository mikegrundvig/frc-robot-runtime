package com.michaelgrundvig.frc.runtime.hardware;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveHardware;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIOSystemcore;
import com.michaelgrundvig.frc.runtime.subsystems.drive.PhoenixDrivetrain;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIOPort;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIOSpark;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIOTalon;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIORev;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIOCanRange;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIOCandi;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIOPort;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIOPhotonVision;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.util.Alert;

/**
 * A robot's hardware: the devices its definition's wiring names, each on the controller, port or
 * CAN ID its block gives. A part with no wiring gets nothing, and stays still rather than guess at
 * motors, and an alert says so: the drivetrain, each joint, each sensor.
 */
public final class RobotHardware implements Hardware {
  private final RobotDefinition definition;
  private final RobotMath math;

  /** The drivetrain's CTRE hardware. Empty when its definition has no wiring for it. */
  private final Optional<PhoenixDrivetrain> phoenix;

  private final Alert noDrivetrain =
      new Alert(
          "Drive",
          "This robot's configuration has no drivetrain wiring (CAN IDs and encoder offsets), so"
              + " the drivetrain stays still. Add it to the robot's entry in Robots.",
          Alert.Level.HIGH);

  /** The joints and sensors with no wiring, as their alerts name them. */
  private final List<String> unwiredJoints = new ArrayList<>();

  private final List<String> unwiredSensors = new ArrayList<>();
  private final Alert jointsUnwired = new Alert("Joints", "unwired", "", Alert.Level.HIGH);
  private final Alert sensorsUnwired = new Alert("Sensors", "unwired", "", Alert.Level.HIGH);

  /**
   * Opens the drivetrain's devices, when its definition has its wiring; each joint's, sensor's and
   * camera's are opened as they're asked for.
   *
   * @param definition what the robot is, each part saying how it's wired
   */
  public RobotHardware(RobotDefinition definition) {
    this.definition = definition;
    math = definition.math();
    phoenix =
        definition
            .drivetrain()
            .wiring()
            .map(
                wiring ->
                    new PhoenixDrivetrain(
                        wiring, definition.drivetrain(), definition.imus().gyro().orientation()));
    noDrivetrain.set(phoenix.isEmpty());
  }

  @Override
  public DriveHardware drive() {
    if (phoenix.isEmpty()) {
      return DriveHardware.none();
    }
    PhoenixDrivetrain drivetrain = phoenix.get();
    return DriveHardware.builder()
        .modules(List.of(drivetrain.modules()))
        .gyro(drivetrain.gyro())
        .backupImu(
            definition
                .imus()
                .backup()
                .<GyroIO>map(backup -> new GyroIOSystemcore(backup.orientation()))
                .orElseGet(() -> new GyroIO() {}))
        .odometryLock(drivetrain.odometryLock())
        .build();
  }

  /** A joint's motor controllers, of the kind its block names: nothing for one with none. */
  @Override
  public JointIO joint(Joint joint) {
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    if (controllers.isEmpty()) {
      unwiredJoints.add(joint.label());
      jointsUnwired.setText(
          "No wiring for "
              + String.join(", ", unwiredJoints)
              + ", so they stay still. Add it to the robot's entry in Robots.");
      jointsUnwired.set(true);
      return new JointIO() {};
    }
    return switch (controllers.get(0).controller()) {
      case TALON_FX, TALON_FXS ->
          new JointIOTalon(math, joint, ControllerPins.of(definition, joint));
      case SPARK_MAX, SPARK_FLEX ->
          new JointIOSpark(math, joint, ControllerPins.of(definition, joint));
    };
  }

  /** A joint's absolute encoder on a Systemcore port; nothing for one with no wiring. */
  @Override
  public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
    return joint
        .driven()
        .absoluteEncoder()
        .flatMap(Driven.AbsoluteEncoder::port)
        .<AbsoluteEncoderIO>map(AbsoluteEncoderIOPort::new)
        .orElseGet(() -> new AbsoluteEncoderIO() {});
  }

  /**
   * A sensor's device, from the wiring it carries: nothing for one with none.
   *
   * @throws IllegalArgumentException for a switch on a joint's motor controller's limit input,
   *     which its joint reads from its controller, and nothing asks for
   */
  @Override
  public SensorIO sensor(Sensor.Placed sensor) {
    Optional<SensorIO> wired = wired(sensor);
    if (wired.isPresent()) {
      return wired.get();
    }
    unwiredSensors.add(sensor.label());
    sensorsUnwired.setText(
        "No wiring for "
            + String.join(", ", unwiredSensors)
            + ", so nothing waiting on them holds. Add it to the robot's entry in Robots.");
    sensorsUnwired.set(true);
    return new SensorIO() {};
  }

  /** A sensor's device, from the wiring it carries. Empty for a sensor with no wiring. */
  private static Optional<SensorIO> wired(Sensor.Placed sensor) {
    return switch (sensor) {
      case Sensor.Switch onSwitch -> onSwitch.wiring().map(wiring -> switchIO(onSwitch, wiring));
      case Sensor.Rangefinder rangefinder ->
          rangefinder.wiring().map(wiring -> new SensorIOCanRange(rangefinder, wiring));
    };
  }

  /**
   * A switch's device, by what its wire runs to: a Systemcore port, or a CANdi.
   *
   * @throws IllegalArgumentException for a switch on a joint's motor controller's limit input: its
   *     joint reads it from the controller, so it has no device of its own
   */
  private static SensorIO switchIO(Sensor.Switch onSwitch, SensorWiring.Connection wiring) {
    return switch (wiring.input()) {
      case SensorWiring.Port port -> new SensorIOPort(port.port(), wiring.polarity());
      case SensorWiring.Candi candi -> new SensorIOCandi(candi, wiring.polarity());
      case SensorWiring.ControllerLimit limit ->
          throw new IllegalArgumentException(
              onSwitch.label()
                  + " is on "
                  + possessive(limit.joint())
                  + " controller's limit input: "
                  + limit.joint()
                  + " reads it from its controller");
    };
  }

  /** The PDH its definition declares, or nothing for a robot that declares none. */
  @Override
  public PdhIO pdh() {
    return definition.pdh().<PdhIO>map(PdhIORev::new).orElseGet(() -> new PdhIO() {});
  }

  /** Each camera's PhotonVision, by its name. */
  @Override
  public List<CameraIO> cameras() {
    List<CameraIO> cameras = new ArrayList<>();
    for (CameraConfig camera : definition.cameras()) {
      cameras.add(new CameraIOPhotonVision(camera.name()));
    }
    return List.copyOf(cameras);
  }

  @Override
  public void close() {
    noDrivetrain.close();
    jointsUnwired.close();
    sensorsUnwired.close();
    phoenix.ifPresent(PhoenixDrivetrain::close);
  }
}

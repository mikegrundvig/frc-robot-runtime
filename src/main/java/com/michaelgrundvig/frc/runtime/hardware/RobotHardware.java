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
import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.util.Alert;

/**
 * A robot's hardware: the devices its definition's wiring names, each on the controller, port or
 * CAN ID its block gives. A part with no wiring gets nothing, and stays still rather than guess at
 * motors, and an alert says so: the drivetrain, each joint, each sensor.
 *
 * <p>On a laptop driving real devices (CTRE's hardware-attached simulation, {@link
 * RunsOn#DESKTOP}), only what the laptop can reach is opened: the CTRE devices on a CANivore
 * plugged into it. Everything else, Systemcore's own CAN ports, I/O ports and IMU, and the PDH,
 * gets nothing, as an unwired part does, and an alert names it: it isn't there to open.
 */
public final class RobotHardware implements Hardware {
  private final RobotDefinition definition;
  private final RobotMath math;

  /** The computer the program runs on, which decides what it can reach. */
  private final RunsOn runsOn;

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

  /** The parts a laptop can't reach, as their alert names them. */
  private final List<String> outOfReach = new ArrayList<>();

  private final Alert outOfReachAlert = new Alert("Robot", "outOfReach", "", Alert.Level.MEDIUM);

  /**
   * Opens the drivetrain's devices, when its definition has its wiring; each joint's, sensor's and
   * camera's are opened as they're asked for.
   *
   * @param definition what the robot is, each part saying how it's wired
   * @param runsOn the computer the program runs on: on a laptop, only the devices on a CANivore are
   *     opened
   */
  public RobotHardware(RobotDefinition definition, RunsOn runsOn) {
    this.definition = definition;
    this.runsOn = runsOn;
    math = definition.math();
    Optional<SwerveWiring> wiring = definition.drivetrain().wiring();
    if (wiring.isPresent() && !runsOn.reaches(wiring.get().canBus())) {
      noteOutOfReach("the drivetrain");
      wiring = Optional.empty();
    }
    phoenix =
        wiring.map(
            reached ->
                new PhoenixDrivetrain(
                    reached, definition.drivetrain(), definition.imus().gyro().orientation()));
    noDrivetrain.set(phoenix.isEmpty() && definition.drivetrain().wiring().isEmpty());
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
        .backupImu(backupImu())
        .odometryLock(drivetrain.odometryLock())
        .build();
  }

  /**
   * The backup IMU, Systemcore's own, where the definition has one: a laptop has none, so there it
   * is nothing.
   */
  private GyroIO backupImu() {
    if (definition.imus().backup().isEmpty()) {
      return new GyroIO() {};
    }
    if (!runsOn.hasSystemcoresOwnParts()) {
      noteOutOfReach("the backup IMU");
      return new GyroIO() {};
    }
    return new GyroIOSystemcore(definition.imus().backup().get().orientation());
  }

  /**
   * A joint's motor controllers, of the kind its block names: nothing for one with none, or, on a
   * laptop, for one with a device off the CANivore.
   */
  @Override
  public JointIO joint(Joint joint) {
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    if (!controllers.isEmpty() && !reachesAll(joint.devices())) {
      noteOutOfReach(joint.label());
      return new JointIO() {};
    }
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

  /**
   * A joint's absolute encoder on a Systemcore port; nothing for one with no wiring, or on a
   * laptop, which has no such ports.
   */
  @Override
  public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
    Optional<MechanismWiring.PortEncoder> port =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port);
    if (port.isEmpty()) {
      return new AbsoluteEncoderIO() {};
    }
    if (!runsOn.hasSystemcoresOwnParts()) {
      noteOutOfReach(joint.label() + "'s absolute encoder");
      return new AbsoluteEncoderIO() {};
    }
    return new AbsoluteEncoderIOPort(port.get());
  }

  /**
   * A sensor's device, from the wiring it carries: nothing for one with none.
   *
   * @throws IllegalArgumentException for a switch on a joint's motor controller's limit input,
   *     which its joint reads from its controller, and nothing asks for
   */
  @Override
  public SensorIO sensor(Sensor.Placed sensor) {
    if (!reaches(sensor)) {
      noteOutOfReach(sensor.label());
      return new SensorIO() {};
    }
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

  /**
   * Whether this computer reaches a sensor's device: on Systemcore, every one; on a laptop, a
   * CANdi's or CANrange's on a CANivore, and no switch on a Systemcore port. A switch on a joint's
   * controller's limit input has no device of its own, and is passed on to say so.
   */
  private boolean reaches(Sensor.Placed sensor) {
    return switch (sensor) {
      case Sensor.Switch onSwitch ->
          onSwitch.wiring().map(wiring -> reaches(wiring.input())).orElse(true);
      case Sensor.Rangefinder rangefinder ->
          rangefinder.wiring().map(wiring -> runsOn.reaches(wiring.bus())).orElse(true);
    };
  }

  /** Whether this computer reaches what a switch's wire runs to. */
  private boolean reaches(SensorWiring.SwitchInput input) {
    return switch (input) {
      case SensorWiring.Port port -> runsOn.hasSystemcoresOwnParts();
      case SensorWiring.Candi candi -> runsOn.reaches(candi.bus());
      case SensorWiring.ControllerLimit limit -> true;
    };
  }

  /** Whether this computer reaches every one of these devices' buses. */
  private boolean reachesAll(List<CanDevice> devices) {
    return devices.stream().allMatch(device -> runsOn.reaches(device.bus()));
  }

  /** Notes a part a laptop can't reach, for its alert. */
  private void noteOutOfReach(String part) {
    outOfReach.add(part);
    outOfReachAlert.setText(
        "Not on this laptop's CANivore, so not opened: "
            + String.join(", ", outOfReach)
            + ". Only a robot reaches them.");
    outOfReachAlert.set(true);
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

  /**
   * The PDH its definition declares, or nothing for a robot that declares none. A laptop has no
   * PDH: it's on Systemcore's CAN port.
   */
  @Override
  public PdhIO pdh() {
    if (definition.pdh().isEmpty()) {
      return new PdhIO() {};
    }
    if (!runsOn.hasSystemcoresOwnParts()) {
      noteOutOfReach("the PDH");
      return new PdhIO() {};
    }
    return new PdhIORev(definition.pdh().get());
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
    outOfReachAlert.close();
    phoenix.ifPresent(PhoenixDrivetrain::close);
  }
}

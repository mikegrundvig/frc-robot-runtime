package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveHardware;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.runtime.subsystems.vision.VisionSim;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The world the simulated robot drives in: the drivetrain's physics, which knows where the robot
 * really is, its joints' ({@link JointSimulation}), its switches and rangefinders ({@link
 * SensorSimulation}), its PDH, and PhotonVision's simulated cameras, rendering what they'd see from
 * there; each built from the robot's definition as the world is. The robot's code sees it only
 * through its IO interfaces ({@link SimulatedHardware}), as it would see real hardware.
 *
 * <p>The robot's wheels and sensors differ from its configuration as a real robot's do ({@link
 * SensorErrors}), so its estimate drifts from the truth, and localization is tested against that
 * gap, not against a perfect world.
 *
 * <p>Its parts can fail while it runs: {@link Faults} declares each one, and what can go wrong with
 * it, to Pitwall's simulator controls ({@link #addTo}), where the page and the scenarios apply
 * them.
 */
public final class SimulatedWorld implements AutoCloseable {
  private final FieldModel field;
  private final DriveSimulation drivetrain;

  /** Whether the robot's definition has a backup IMU, which the world then hands its code. */
  private final boolean hasBackupImu;

  private final VisionSim vision;
  private final CameraIOSim[] cameras;

  /** The cameras that ride on joints, which move with them. */
  private final List<CameraConfig> camerasOnJoints;

  /** The robot's driven joints, and the absolute encoders the robot reads itself. */
  private final JointSimulation joints;

  /** The robot's switches and rangefinders. */
  private final SensorSimulation sensors;

  private final PdhIOSim pdh;

  /**
   * @param field the field, with its tags and terrain
   * @param definition what the robot is: its drivetrain, frame and IMUs, its cameras, its joints,
   *     its sensors and its PDH, each simulated from it
   * @param errors how the robot's wheels, sensors, and camera mounts differ from its configuration
   */
  public SimulatedWorld(FieldModel field, RobotDefinition definition, SensorErrors errors) {
    this.field = field;
    Imus imus = definition.imus();
    List<CameraConfig> cameras = definition.cameras();
    this.drivetrain = new DriveSimulation(definition.body(), definition.drivetrain(), errors, imus);
    this.hasBackupImu = imus.backup().isPresent();
    this.drivetrain.setTerrain(field.terrain());
    // Each camera's noise from its own stream of the seed (Seeds).
    vision =
        new VisionSim(field, errors.tagErrors(), camera -> Seeds.camera(errors.seed(), camera));
    this.cameras = new CameraIOSim[cameras.size()];
    camerasOnJoints = cameras.stream().filter(camera -> camera.joint().isPresent()).toList();
    for (int i = 0; i < cameras.size(); i++) {
      CameraConfig camera = cameras.get(i);
      Transform3d mountError = errors.cameraMountError(camera.name());
      this.cameras[i] =
          new CameraIOSim(camera.name(), vision.camera(camera, mountError), vision, mountError);
    }
    joints = new JointSimulation(definition);
    sensors = new SensorSimulation(definition, joints);
    pdh = new PdhIOSim(definition, joints, this.drivetrain);
  }

  /**
   * Declares the world's parts that can fail, and what can go wrong with each, to Pitwall's
   * simulator controls: the cameras, the gyro and the backup IMU, each module, and the robot itself
   * ({@link Faults}).
   */
  public void addTo(Components components) {
    Faults.declare(this, components);
  }

  /**
   * The drivetrain's simulated modules and IMUs, for the drivetrain to read: no backup IMU for a
   * robot whose definition has none.
   */
  public DriveHardware drivetrainHardware() {
    // A backup IMU is always simulated, but only a robot whose definition has one gets to read it.
    GyroIO backupImu = new GyroIO() {};
    if (hasBackupImu) {
      backupImu = drivetrain.backupGyro();
    }
    return DriveHardware.builder()
        .modules(List.of(drivetrain.modules()))
        .gyro(drivetrain.gyro())
        .backupImu(backupImu)
        .odometryLock(new ReentrantLock())
        .build();
  }

  /** The simulated cameras, in the robot's configured order. */
  public CameraIO[] cameras() {
    return cameras.clone();
  }

  /** The robot's switches and rangefinders, as a test sets them. */
  public SensorSimulation sensors() {
    return sensors;
  }

  /**
   * The PDH, simulated: each channel what the simulated motor it feeds draws, the joints' and the
   * drivetrain's.
   */
  public PdhIOSim pdh() {
    return pdh;
  }

  /** The robot's driven joints' physics, and the absolute encoders the robot reads itself. */
  public JointSimulation joints() {
    return joints;
  }

  /** The cameras' names, in the robot's configured order. */
  public List<String> cameraNames() {
    return Arrays.stream(cameras).map(CameraIOSim::name).toList();
  }

  /** A simulated camera, for its faults. */
  CameraIOSim camera(int index) {
    return cameras[index];
  }

  /** The field the robot drives on. */
  FieldModel field() {
    return field;
  }

  /** The cameras' simulation, which places the field's tags: for their faults. */
  VisionSim vision() {
    return vision;
  }

  /** The drive team puts the robot here. */
  public void place(Pose2d pose) {
    drivetrain.place(pose);
  }

  /**
   * One loop: the world moves on, and the cameras see where the robot now is. Before the robot
   * reads its sensors, as on a robot between loops.
   */
  public void step(long nowNanos) {
    drivetrain.step(RobotLoop.PERIOD_SECONDS, nowNanos);
    joints.step(RobotLoop.PERIOD_SECONDS);
    for (CameraConfig camera : camerasOnJoints) {
      vision.carry(camera.name(), joints.cameraMount(camera));
    }
    vision.update(drivetrain.truePose3d());
  }

  /** Where the robot really is. */
  public Pose2d truePose() {
    return drivetrain.truePose();
  }

  /**
   * Logs where the robot really is, and how far the estimate is from it: what localization has to
   * close. In 3D too, raised and tilted as the robot really is, for AdvantageScope's ghost.
   */
  public void log(Pose2d estimated) {
    Pose2d truth = drivetrain.truePose();
    Logger.recordOutput("Sim/TruePose", truth);
    Logger.recordOutput("Sim/TruePose3d", drivetrain.truePose3d());
    Logger.recordOutput(
        "Sim/PoseErrorMeters", truth.getTranslation().getDistance(estimated.getTranslation()));
    Logger.recordOutput(
        "Sim/HeadingErrorDegrees", estimated.getRotation().minus(truth.getRotation()).getDegrees());
  }

  /** The drivetrain's physics: for tests to shove, tilt, or trip it. */
  public DriveSimulation drivetrain() {
    return drivetrain;
  }

  @Override
  public void close() {
    vision.close();
  }

  /** How many cameras it has. */
  int cameraCount() {
    return cameras.length;
  }
}

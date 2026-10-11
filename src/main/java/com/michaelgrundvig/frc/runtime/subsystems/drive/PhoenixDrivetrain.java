package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.lib.wiring.CanPorts;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.concurrent.locks.Lock;
import org.wpilib.math.geometry.Rotation3d;

/**
 * The drivetrain's hardware: four modules, each a TalonFX driving the wheel, a TalonFX turning it,
 * and a CANcoder reading its angle, or REV's SPARKs ({@link ModuleIOSpark}); a Pigeon 2; and the
 * thread that reads the CTRE devices at 250 Hz for odometry. Built on the robot from its wiring;
 * the simulator and replay never touch it.
 *
 * <p>Needs Phoenix 6 26.70 or later on the devices (firmware 26.70) for WPILib 2027 alpha 7.
 *
 * <p>Phoenix's own signal log (a {@code .hoot} file) is off: on Systemcore it would otherwise start
 * in every session, a second writer on the USB drive beside the AdvantageKit log, which is the
 * record and the one replay reads. A routine that needs a {@code .hoot} file, such as a
 * characterization in Phoenix Tuner X, calls {@code SignalLogger.start()} and {@code stop()} around
 * its run.
 */
public final class PhoenixDrivetrain implements AutoCloseable {
  /**
   * How often each module's CANcoder sends its position and velocity, in Hz: Phoenix 6's default,
   * set explicitly so nothing else slows it. Without a Phoenix Pro license, the steering motor
   * closes its loop on these, taken remotely, so however fast its loop runs, it sees a new angle
   * only this often, and that wait sets how high the steering's gains can go. Faster allows higher
   * gains, for more traffic on the CAN bus: measure the bus's load before raising it. The
   * simulator's steering hears its CANcoder at this rate too.
   */
  public static final double ENCODER_FREQUENCY_HZ = 100;

  private final PhoenixOdometryThread odometry = new PhoenixOdometryThread();
  private final ModuleIO[] modules = new ModuleIO[Module.COUNT];
  private final GyroIO gyro;

  /**
   * Configures every device (waiting for each, at startup) and starts reading them.
   *
   * @param wiring where the devices are, and how the encoders read
   * @param drivetrain each corner's module: its gearing and wheel
   * @param gyroOrientation how the Pigeon 2 is turned on the robot, from the robot's definition
   */
  public PhoenixDrivetrain(
      SwerveWiring wiring, DrivetrainGeometry drivetrain, Rotation3d gyroOrientation) {
    SignalLogger.enableAutoLogging(false);
    CANBus bus = bus(wiring.canBus());
    for (int i = 0; i < Module.COUNT; i++) {
      modules[i] = module(wiring, wiring.modules().get(i), drivetrain.module(i), bus);
    }
    gyro =
        new GyroIOPigeon2(wiring.gyroId(), bus, wiring.gyroYawScale(), gyroOrientation, odometry);
    odometry.start();
  }

  /**
   * One module's hardware, on the motor controllers the wiring names: TalonFXs, read at 250 Hz by
   * the odometry thread, or SPARKs, read once a loop.
   */
  private ModuleIO module(
      SwerveWiring wiring,
      SwerveWiring.ModuleWiring module,
      DrivetrainGeometry.ModuleType hardware,
      CANBus bus) {
    if (wiring.driveController() == MechanismWiring.Controller.TALON_FX) {
      return new ModuleIOTalonFX(module, bus, hardware, hardware.couplingRatio(), odometry);
    }
    // SPARKs are on one of Systemcore's own CAN ports: a drivetrain of SPARKs on a CANivore is
    // refused by the robot's definition before this is built.
    CanBus.Port port = (CanBus.Port) wiring.canBus();
    return new ModuleIOSpark(
        module,
        CanPorts.of(port.port()),
        wiring.driveController(),
        wiring.steerController(),
        hardware);
  }

  /** The modules: front left, front right, back left, back right. */
  public ModuleIO[] modules() {
    return modules.clone();
  }

  public GyroIO gyro() {
    return gyro;
  }

  /**
   * The lock the drivetrain holds while reading its IO, so the odometry thread's readings stay in
   * step across every device.
   */
  public Lock odometryLock() {
    return odometry.lock();
  }

  /** Phoenix's name for the CAN bus the wiring names. */
  static CANBus bus(CanBus bus) {
    return PhoenixMechanism.bus(bus);
  }

  /** Stops reading the devices. */
  @Override
  public void close() {
    odometry.interrupt();
  }
}

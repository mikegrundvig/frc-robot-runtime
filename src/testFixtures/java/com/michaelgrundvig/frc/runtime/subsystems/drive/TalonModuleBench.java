package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.Optional;

/**
 * The module conformance suite's bench for the CTRE module ({@link ModuleIOTalonFX}), against
 * Phoenix's own simulated devices: two TalonFXs and a CANcoder, with the odometry thread reading
 * them. As on the joints' CTRE bench, the devices run on the wall clock, a step waits for the
 * drive's next frame, and the sensors are set again every step, since Phoenix's simulated TalonFX
 * runs its loop again only when its sensor is set.
 */
public final class TalonModuleBench implements ModuleBench {
  // CAN IDs no other test in the runtime uses: Phoenix's simulated devices are all on one bus.
  private static final int DRIVE_ID = 17;
  private static final int STEER_ID = 18;
  private static final int ENCODER_ID = 19;

  private static final DrivetrainGeometry.ModuleType MODULE =
      ExampleRobot.SIM.definition().drivetrain().module(0);

  /** How long a step waits for the drive's next frame, in seconds, before going on without it. */
  private static final double FRAME_WAIT_SECONDS = 0.1;

  /** How long the odometry thread is given to stop, in milliseconds. */
  private static final long THREAD_STOP_MILLIS = 2000;

  public TalonModuleBench() {
    SignalLogger.enableAutoLogging(false);
  }

  @Override
  public String name() {
    return "a TalonFX module";
  }

  @Override
  public double wheelRadiusMeters() {
    return MODULE.wheelRadius().in(Meters);
  }

  @Override
  public Rig powerOn(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    return new TalonRig(gains, limits);
  }

  @Override
  public Optional<String> lacks(Ability ability) {
    return Optional.empty();
  }

  @Override
  public Optional<String> knownDifference(ModuleClause clause) {
    return switch (clause) {
      case M9 ->
          Optional.of(
              "the CTRE module reports its drive's stator current with its sign, negative while"
                  + " the motor sends current back, where the REV module and the simulator report"
                  + " its size");
      case M10 ->
          Optional.of(
              "the CTRE module reports its drive's supply current with its sign, negative while"
                  + " the motor sends current back, where the REV module and the simulator report"
                  + " its size");
      default -> Optional.empty();
    };
  }

  /** One CTRE module, powered on. */
  private static final class TalonRig implements Rig {
    private final CANBus bus = new CANBus("bench-module");
    private final TalonFX drive = new TalonFX(DRIVE_ID, bus);
    private final TalonFX steer = new TalonFX(STEER_ID, bus);
    private final CANcoder encoder = new CANcoder(ENCODER_ID, bus);
    private final PhoenixOdometryThread odometry = new PhoenixOdometryThread();
    private final ModuleIOTalonFX io;
    private final StatusSignal<?> driveFrame;
    private final double driveGearing = MODULE.driveGearing().ratio();
    private double wheelRotations;
    private double wheelRotationsPerSecond;
    private double moduleRotations;

    TalonRig(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
      RobotSim.setEnabled(true);
      drive.setControl(new NeutralOut());
      steer.setControl(new NeutralOut());
      turnTheSensors();
      SwerveWiring.ModuleWiring wiring =
          SwerveWiring.ModuleWiring.builder()
              .driveId(DRIVE_ID)
              .steerId(STEER_ID)
              .encoderOffset(Rotations.of(0))
              .driveInverted(false)
              .steerInverted(false)
              .encoderInverted(false)
              .encoderId(ENCODER_ID)
              .build();
      // No coupling: the wheel's reading is its drive's alone.
      io = new ModuleIOTalonFX(wiring, bus, MODULE, 0, odometry);
      io.configure(gains, limits);
      odometry.start();
      driveFrame = drive.getVelocity();
    }

    /** Turns the drive's rotor and the CANcoder's magnet to where the wheel and the module are. */
    private void turnTheSensors() {
      drive.getSimState().setRawRotorPosition(wheelRotations * driveGearing);
      drive.getSimState().setRotorVelocity(wheelRotationsPerSecond * driveGearing);
      encoder.getSimState().setRawPosition(moduleRotations);
      steer.getSimState().setRotorVelocity(0);
    }

    @Override
    public ModuleIO io() {
      return io;
    }

    @Override
    public void step() {
      RobotSim.stepClock();
      turnTheSensors();
      driveFrame.waitForUpdate(FRAME_WAIT_SECONDS, false);
    }

    @Override
    public void wheelIs(double radians, double radiansPerSecond) {
      wheelRotations = radians / (2 * Math.PI);
      wheelRotationsPerSecond = radiansPerSecond / (2 * Math.PI);
      turnTheSensors();
    }

    @Override
    public void moduleIs(double radians) {
      moduleRotations = radians / (2 * Math.PI);
      turnTheSensors();
    }

    @Override
    public double driveVolts() {
      return drive.getSimState().getMotorVoltage();
    }

    @Override
    public double steerVolts() {
      return steer.getSimState().getMotorVoltage();
    }

    /** Stops the odometry thread and leaves both motors neutral for the next power-on. */
    @Override
    public void close() {
      odometry.interrupt();
      try {
        odometry.join(THREAD_STOP_MILLIS);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
      drive.setControl(new NeutralOut());
      steer.setControl(new NeutralOut());
      wheelIs(0, 0);
      moduleIs(0);
    }
  }
}

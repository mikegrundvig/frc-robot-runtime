package com.michaelgrundvig.frc.runtime.subsystems.drive;

import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import java.util.Optional;

/**
 * A test bench for one kind of swerve module driver: it powers a driver on against simulated
 * devices, and lets the conformance suite ({@link ModuleIOConformance}) turn the wheel and the
 * module and see what the motors are given. The suite knows only {@link ModuleIO} and this, as the
 * joints' suite knows only its bench ({@code JointBench}).
 */
public interface ModuleBench {
  /** What the bench is called in a test's message, such as "a TalonFX module". */
  String name();

  /** The radius of the module's wheel, in meters: the drive's kP is in volts per meter a second. */
  double wheelRadiusMeters();

  /** Powers a module's driver on, configured with these gains and limits, pointing ahead. */
  Rig powerOn(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits);

  /** Why the bench cannot do something the suite would have it do, or empty if it can. */
  Optional<String> lacks(Ability ability);

  /**
   * Why the bench's driver is known not to keep a clause, or empty if it keeps it. The suite checks
   * that the driver still fails the clause, so a difference that is fixed fails until its entry is
   * taken away.
   */
  Optional<String> knownDifference(ModuleClause clause);

  /** Something the suite asks of a bench that not every simulated device can do. */
  enum Ability {
    /** Showing the voltage across the drive motor ({@link Rig#driveVolts}). */
    SHOW_THE_DRIVES_VOLTS,
    /** Showing the voltage across the steering motor ({@link Rig#steerVolts}). */
    SHOW_THE_STEERINGS_VOLTS,
    /** Running the drive motor itself, so the driver has its voltage and current to report. */
    RUN_THE_DRIVE_MOTOR
  }

  /** One driver, powered on against its simulated devices. */
  interface Rig extends AutoCloseable {
    /** The driver under test. */
    ModuleIO io();

    /** Lets one loop of the robot's time pass, 20 ms, the devices running through it. */
    void step();

    /** Turns the wheel to an angle, in radians, turning at a speed, in radians a second. */
    void wheelIs(double radians, double radiansPerSecond);

    /** Points the module at an angle, in radians, counterclockwise from straight ahead. */
    void moduleIs(double radians);

    /** The voltage across the drive motor now, positive when it rolls the wheel forward. */
    double driveVolts();

    /** The voltage across the steering motor now, positive when it turns the module forward. */
    double steerVolts();

    /** Leaves the devices as the next power-on needs them. */
    @Override
    void close();
  }
}

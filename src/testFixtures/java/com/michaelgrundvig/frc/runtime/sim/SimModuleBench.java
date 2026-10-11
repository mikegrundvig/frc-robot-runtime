package com.michaelgrundvig.frc.runtime.sim;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleBench;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleClause;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.simulation.KinematicPhysics;
import com.michaelgrundvig.frc.simulation.WheelDrive;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import java.util.List;
import java.util.Optional;
import org.wpilib.system.RobotController;

/**
 * The module conformance suite's bench for the simulator's module ({@link ModuleIOSim}), on a wheel
 * of a robot that stands still. The wheel's turning is its encoder's alone, as a wheel that slips
 * turns, and the module's angle is set through its absolute encoder's zero, so neither needs the
 * physics to move the robot.
 */
public final class SimModuleBench implements ModuleBench {
  private static final DrivetrainGeometry GEOMETRY = ExampleRobot.SIM.definition().drivetrain();

  /** The simulator's step: a fifth of a loop. */
  private static final int STEPS_A_LOOP = 5;

  @Override
  public String name() {
    return "the simulator's module";
  }

  @Override
  public double wheelRadiusMeters() {
    return GEOMETRY.module(0).wheelRadius().in(Meters);
  }

  @Override
  public Rig powerOn(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    return new SimRig(gains, limits);
  }

  @Override
  public Optional<String> lacks(Ability ability) {
    if (ability == Ability.SHOW_THE_STEERINGS_VOLTS) {
      return Optional.of(
          "the simulator's steering turns the module it is asked to turn, so its motor cannot be"
              + " held where the suite puts it");
    }
    if (ability == Ability.RUN_THE_DRIVE_MOTOR) {
      return Optional.of(
          "the simulator's drive motor is the physics', which the bench does not run: the robot"
              + " would drive off");
    }
    return Optional.empty();
  }

  @Override
  public Optional<String> knownDifference(ModuleClause clause) {
    return Optional.empty();
  }

  /** One simulated module, on a wheel that the floor does not move. */
  private final class SimRig implements Rig {
    private final ModuleIOSim sim;
    private final WheelDrive drive = new WheelDrive();
    private double wheelRadians;

    SimRig(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
      KinematicPhysics physics =
          new KinematicPhysics(
              ExampleRobot.SIM.definition().body(), GEOMETRY, List.of(1.0, 1.0, 1.0, 1.0));
      sim = new ModuleIOSim(GEOMETRY.module(0), physics.wheel(0), 0, 0);
      sim.configure(gains, limits);
    }

    @Override
    public ModuleIO io() {
      return sim;
    }

    /** Runs the module's controllers through a loop's steps, taking a reading at each. */
    @Override
    public void step() {
      for (int i = 0; i < STEPS_A_LOOP; i++) {
        sim.actuate(RobotLoop.PERIOD_SECONDS / STEPS_A_LOOP, drive);
      }
      RobotSim.stepClock();
      sim.sample(RobotController.getTime());
    }

    /** Turns the wheel as a slip does: its encoder counts it and the floor does not. */
    @Override
    public void wheelIs(double radians, double radiansPerSecond) {
      sim.spinAt(0);
      sim.spin(radians - wheelRadians);
      wheelRadians = radians;
      sim.spinAt(radiansPerSecond * wheelRadiusMeters());
    }

    /** Points the module by moving its absolute encoder's zero: it reads the angle from then on. */
    @Override
    public void moduleIs(double radians) {
      sim.setSteerOffset(sim.trueSteerAngle().getRadians() - radians);
    }

    @Override
    public double driveVolts() {
      return drive.volts();
    }

    @Override
    public double steerVolts() {
      throw new UnsupportedOperationException("the simulator's steering motor is not held");
    }

    @Override
    public void close() {
      // Nothing is held: the simulated module is this object's alone.
    }
  }
}

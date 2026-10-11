package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.experiment.Rig;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import java.util.List;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The steering's test rig for the pit: every wheel is pointed at one angle with the drive wheels
 * held still, and what the steering does about it is recorded, so that Pitwall's page can tune the
 * steering's gains by experiment.
 *
 * <p>Everything the rig records is a logged input, so a replay records the same.
 */
final class SteeringRig {
  /** The wheels by name, as a steering test records them, in the order the modules are numbered. */
  private static final List<String> WHEELS =
      List.of("Front left", "Front right", "Back left", "Back right");

  private final Module[] modules;
  private final DriveOutput output;

  /** The angle the wheels were last pointed at, in radians, counterclockwise from forward. */
  private double heldRadians;

  SteeringRig(Module[] modules, DriveOutput output) {
    this.modules = modules;
    this.output = output;
  }

  /**
   * Points every wheel at one angle for one loop, relative to the robot, with the drive wheels held
   * still. A test calls it every loop it runs.
   *
   * @param radians the angle, counterclockwise from the robot's forward
   */
  void hold(double radians) {
    heldRadians = radians;
    output.pointWheels(new Rotation2d(radians));
  }

  /**
   * The steering as a Pitwall rig. Its gains are {@code steerKp} and {@code steerKd}, tuned where
   * they always are. Its setpoint is the angle every wheel is pointed at. For each wheel it records
   * the wheel's angle at every high-rate reading, which is what an experiment is scored on, and
   * once a loop the steering motor's voltage and the most current it drew since the loop before.
   *
   * @param tuning the drivetrain's configuration as the pit tunes it
   * @param drivetrain the mechanism a test must own while it runs
   */
  Rig rig(LiveConfig<DrivetrainControl> tuning, Mechanism drivetrain) {
    Rig.Builder rig =
        Rig.builder()
            .name("Steering")
            .gains(tuning, "gains.steerKp", "gains.steerKd")
            .setpoint(Radians, this::hold)
            .limits(Degrees.of(-180), Degrees.of(180));
    for (int i = 0; i < modules.length; i++) {
      Module module = modules[i];
      rig.measurement(
          WHEELS.get(i),
          Degrees,
          (now, series) -> {
            for (int reading = 0; reading < module.odometryReadings(); reading++) {
              series.add(
                  module.odometryTimestampNanos(reading),
                  Math.toDegrees(nearestHeld(module.odometryAngleRadians(reading))));
            }
          });
    }
    for (int i = 0; i < modules.length; i++) {
      Module module = modules[i];
      rig.signal(WHEELS.get(i) + " voltage", Volts, module::steerAppliedVolts);
      rig.signal(WHEELS.get(i) + " current", Amps, module::steerCurrentAmps);
    }
    return rig.requires(drivetrain).build();
  }

  /**
   * A wheel's measured angle, as the turn of it nearest the angle held. A module reports its angle
   * between -180° and 180°, so a wheel held near either end would otherwise seem to jump a whole
   * turn as it crosses.
   */
  private double nearestHeld(double measuredRadians) {
    return heldRadians + MathUtil.angleModulus(measuredRadians - heldRadians);
  }
}

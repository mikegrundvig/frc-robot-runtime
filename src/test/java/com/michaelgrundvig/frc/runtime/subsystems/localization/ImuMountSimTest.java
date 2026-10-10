package com.michaelgrundvig.frc.runtime.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.SensorErrorsAccelErrorsBuilder;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.ImuMountBuilder;
import com.michaelgrundvig.frc.world.robot.ImusBuilder;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.Units;

/**
 * Where the IMU sits and how its accelerometer lies, against what localization believes: driving
 * that never slips must never look like slipping. The gyro's accelerometer reads as a Pigeon 2's
 * does on the simulator's season-worn robot ({@link SensorErrors.AccelErrors#PIGEON_2}), turned or
 * moved where a test says; the robot's definition says where it's mounted, how it's turned, and
 * whether it has been checked ({@link ImuMount}).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class ImuMountSimTest {
  /**
   * Driving the wheels grip through: launches from rest at full stick, stops, a spin in place at a
   * turn a second, and driving while spinning, each from standing.
   */
  private static final double[][] CLEAN = {
    {1.5, 1, 0, 0}, {1, 0, 0, 0},
    {1.5, 0, 1, 0}, {1, 0, 0, 0},
    {2, 0, 0, 1}, {1, 0, 0, 0},
    {2, 0.7, 0, 1}, {1, 0, 0, 0}
  };

  /** What the slip check said over a drive. */
  private record Verdict(int slipping, int collisions) {}

  private static Verdict drive(ImuMount believed, SensorErrors.AccelErrors accelerometer) {
    RobotConfig robot =
        RobotConfigBuilder.builder(ExampleRobot.SIM)
            .definition(
                RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
                    .imus(
                        ImusBuilder.builder(ExampleRobot.SIM.definition().imus())
                            .gyro(believed)
                            .build())
                    .build())
            .build();
    try (DriveSimHarness harness =
        new DriveSimHarness(robot, SensorErrors.TYPICAL.withAccelerometer(accelerometer))) {
      int[] counts = new int[2];
      harness.eachLoop(
          () -> {
            counts[0] += harness.localization.conditions().slipMetersPerSecond() > 0 ? 1 : 0;
            counts[1] += harness.localization.conditions().collision() ? 1 : 0;
          });
      for (double[] leg : CLEAN) {
        harness.drive(leg[0], leg[1], leg[2], leg[3]);
      }
      return new Verdict(counts[0], counts[1]);
    }
  }

  private static ImuMount at(Translation2d place, boolean checked) {
    return ImuMountBuilder.builder(ExampleRobot.SIM.definition().imus().gyro())
        .forward(Units.Meters.of(place.getX()))
        .left(Units.Meters.of(place.getY()))
        .accelerometerChecked(checked)
        .build();
  }

  private static SensorErrors.AccelErrors pigeon() {
    return SensorErrors.AccelErrors.PIGEON_2;
  }

  /** A Pigeon 2's accelerometer, with its noise, bias, range and sampling, sees no slip or hit. */
  @Test
  void aPigeonsAccelerometerSeesNoSlipOrHitInCleanDriving() {
    assertThat(drive(at(Translation2d.ZERO, true), pigeon())).isEqualTo(new Verdict(0, 0));
  }

  /**
   * Turned a quarter turn on its mount, or mounted logo down, an accelerometer reads speeding up as
   * a sideways slide. Until the robot passes the accelerometer check, its speeds aren't compared,
   * and a jolt counts by its size alone.
   */
  @Test
  void anUncheckedAccelerometerTurnedOrFlippedSeesNoSlip() {
    ImuMount unchecked = at(Translation2d.ZERO, false);
    SensorErrors.AccelErrors turned =
        SensorErrorsAccelErrorsBuilder.builder(pigeon()).turnedDegrees(90).build();
    SensorErrors.AccelErrors flipped =
        SensorErrorsAccelErrorsBuilder.builder(pigeon()).flipped(true).build();

    assertThat(drive(unchecked, turned)).isEqualTo(new Verdict(0, 0));
    assertThat(drive(unchecked, flipped)).isEqualTo(new Verdict(0, 0));
    assertThat(drive(at(Translation2d.ZERO, true), turned).slipping())
        .as("trusted as checked, the same accelerometer sees slip")
        .isPositive();
  }

  /**
   * An IMU 20 cm ahead of the center and 5 cm left feels the robot's turns swing it around. Where
   * the definition places it, that's no slip; believed at the center, it would be.
   */
  @Test
  void anImuOffTheCenterWhereTheDefinitionPlacesItSeesNoSlip() {
    Translation2d place = new Translation2d(0.2, 0.05);
    SensorErrors.AccelErrors offCenter =
        SensorErrorsAccelErrorsBuilder.builder(pigeon())
            .forwardMeters(place.getX())
            .leftMeters(place.getY())
            .build();

    Verdict placed = drive(at(place, true), pigeon());
    Verdict centered = drive(at(Translation2d.ZERO, true), offCenter);

    assertThat(placed).isEqualTo(new Verdict(0, 0));
    assertThat(centered.slipping()).as("believed at the center").isPositive();
  }

  /**
   * Mounted however the robot's definition says (here on its side, its x axis pointing left and its
   * z axis forward), the accelerometer reads gravity and the robot's motion along its own axes, and
   * the IO turns them into the robot's: no slip, no hit. Turned a quarter turn further on its mount
   * than the definition says, it reads slip.
   */
  @Test
  void anImuTurnedOnItsMountAsTheDefinitionSaysSeesNoSlip() {
    ImuMount onItsSide =
        ImuMountBuilder.builder(at(Translation2d.ZERO, true))
            .orientation(new Rotation3d(Math.PI / 2, 0, Math.PI / 2))
            .build();
    SensorErrors.AccelErrors turnedFurther =
        SensorErrorsAccelErrorsBuilder.builder(pigeon()).turnedDegrees(90).build();

    assertThat(drive(onItsSide, pigeon())).isEqualTo(new Verdict(0, 0));
    assertThat(drive(onItsSide, turnedFurther).slipping()).as("turned further").isPositive();
  }
}

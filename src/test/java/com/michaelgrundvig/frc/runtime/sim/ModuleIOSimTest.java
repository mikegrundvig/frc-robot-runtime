package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.simulation.KinematicPhysics;
import com.michaelgrundvig.frc.simulation.WheelDrive;
import com.michaelgrundvig.frc.simulation.WheelTruth;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation2d;

/** A simulated module's steering answers to its configuration, as a motor controller does. */
@ExtendWith(RobotSim.class)
class ModuleIOSimTest {
  private static final DrivetrainGeometry GEOMETRY = ExampleRobot.SIM.definition().drivetrain();
  private static final DrivetrainControl DRIVETRAIN = ExampleRobot.SIM.drivetrain();

  /** The simulator's step: a fifth of a 20 ms loop. */
  private static final double STEP_SECONDS = 0.004;

  /** A wheel of the template's robot, standing still: the modules here only steer. */
  private static WheelTruth wheel() {
    return new KinematicPhysics(
            ExampleRobot.SIM.definition().body(), GEOMETRY, List.of(1.0, 1.0, 1.0, 1.0))
        .wheel(0);
  }

  /** Runs a module's controllers through one of the simulator's steps. */
  private static void step(ModuleIOSim module) {
    module.actuate(STEP_SECONDS, new WheelDrive());
  }

  /** A module as configured, with the steering's current limit tuned to this. */
  private static ModuleIOSim module(double steerLimitAmps) {
    ModuleIOSim module = new ModuleIOSim(GEOMETRY.module(0), wheel(), 0, 0);
    module.configure(
        DRIVETRAIN.gains(),
        DrivetrainControlLimitsBuilder.builder(DRIVETRAIN.limits())
            .steerStatorCurrent(Amps.of(steerLimitAmps))
            .build());
    return module;
  }

  /**
   * The steering's current limit comes from the configuration, as tuned live. Held to its limit,
   * the motor gives a steady torque, so in the first 4 ms of a turn, while it's still at its limit,
   * a quarter of the limit turns the module a quarter as far.
   */
  @Test
  void aQuarterOfTheCurrentLimitTurnsTheModuleAQuarterAsFarAtFirst() {
    ModuleIOSim atForty = module(40);
    ModuleIOSim atTen = module(10);
    atForty.setSteerAngle(Rotation2d.CCW_90DEG);
    atTen.setSteerAngle(Rotation2d.CCW_90DEG);

    step(atForty);
    step(atTen);

    assertThat(atForty.trueSteerAngle().getDegrees()).isBetween(1.0, 3.0);
    assertThat(atTen.trueSteerAngle().getRadians())
        .isCloseTo(atForty.trueSteerAngle().getRadians() / 4, within(1e-12));
  }

  /**
   * The steering's loop runs on the angle the CANcoder last sent, every 10 ms. A stiff steerKp
   * corrects too far on each angle it hears, and past about 77 (at the committed steerKd, 0.08)
   * each correction overshoots by more than the error it corrected: a small error grows instead of
   * dying away. Each side of that edge, a tenth of a degree's error over 100 ms.
   */
  @Test
  void pastTheEdgeTheCanCodersWaitSetsASmallErrorGrowsInsteadOfDying() {
    assertThat(growth(70)).as("steerKp 70").isLessThan(0.5);
    assertThat(growth(85)).as("steerKp 85").isGreaterThan(2);
  }

  /**
   * The steering's loop runs every millisecond and its CANcoder sends every 10 ms by the time the
   * module keeps, however the physics cuts it into sub-steps: a module turning a quarter turn in
   * sub-steps of 0.5 or 1.5 ms is where it is in 4 ms ones, every 12 ms (where the three meet),
   * within the closed form's rounding.
   */
  @Test
  void theSteeringKeepsItsRatesWhateverTheSubSteps() {
    double[] subSteps = {0.004, 0.0005, 0.0015};
    double[][] angles = new double[subSteps.length][10];
    for (int k = 0; k < subSteps.length; k++) {
      ModuleIOSim module = module(40);
      module.setSteerAngle(Rotation2d.CCW_90DEG);
      for (int check = 0; check < 10; check++) {
        for (int step = 0; step < Math.round(0.012 / subSteps[k]); step++) {
          module.actuate(subSteps[k], new WheelDrive());
        }
        angles[k][check] = module.trueSteerAngle().getRadians();
      }
    }

    assertThat(angles[0][9]).as("turning").isGreaterThan(1);
    for (int check = 0; check < 10; check++) {
      assertThat(angles[1][check])
          .as("0.5 ms, %d", check)
          .isCloseTo(angles[0][check], within(1e-9));
      assertThat(angles[2][check])
          .as("1.5 ms, %d", check)
          .isCloseTo(angles[0][check], within(1e-9));
    }
  }

  /**
   * How a tenth of a degree's error grows over 100 ms at this steerKp: the largest error in the
   * last 20 ms over the largest in the first 20. Small enough to stay under the current limit.
   */
  private static double growth(double steerKp) {
    ModuleIOSim module = new ModuleIOSim(GEOMETRY.module(0), wheel(), 0, 0);
    module.configure(
        DrivetrainControlGainsBuilder.builder(DRIVETRAIN.gains()).steerKp(steerKp).build(),
        DRIVETRAIN.limits());
    module.setSteerAngle(Rotation2d.fromDegrees(0.1));
    double first = 0;
    double last = 0;
    for (int step = 1; step <= 25; step++) {
      step(module);
      double error = Math.abs(0.1 - module.trueSteerAngle().getDegrees());
      if (step <= 5) {
        first = Math.max(first, error);
      } else if (step > 20) {
        last = Math.max(last, error);
      }
    }
    return last / first;
  }

  /**
   * The steering's current, as the module reports it, is the most since the last loop: the limit in
   * the loop that starts a turn, though by that loop's end the motor is at speed and draws next to
   * nothing.
   */
  @Test
  void theSteeringReportsItsPeakCurrentWhichStaysWithinItsLimit() {
    ModuleIOSim module = module(20);
    ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
    module.setSteerAngle(Rotation2d.CCW_90DEG);

    for (int loop = 0; loop < 25; loop++) {
      for (int step = 0; step < 5; step++) {
        step(module);
      }
      module.updateInputs(inputs);
      assertThat(inputs.steerStatorCurrentAmps).isLessThanOrEqualTo(20);
      if (loop == 0) {
        assertThat(inputs.steerStatorCurrentAmps).as("the turn's first loop").isEqualTo(20);
      }
    }
    assertThat(inputs.steerStatorCurrentAmps).as("turned and holding").isLessThan(1);
  }
}

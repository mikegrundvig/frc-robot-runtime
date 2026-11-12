package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.TestWiring;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIOInputsAutoLogged;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiringModuleWiringBuilder;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The simulated PDH reads the drivetrain's channels as it does the joints': each what the simulated
 * motor it feeds draws, a module's drive or its steering.
 */
@ExtendWith(RobotSim.class)
class PdhIOSimTest {
  /** The template's robot, wired, each module's drive and steering fed from channels 0 to 7. */
  private static RobotDefinition fed() {
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      modules.add(
          SwerveWiringModuleWiringBuilder.builder()
              .drive(TALON_FX.can(10 + 2 * i).pdh(2 * i, Amps.of(40)))
              .steering(TALON_FX.can(11 + 2 * i).pdh(2 * i + 1, Amps.of(40)))
              .encoder(CANCODER.can(20 + i).offset(Rotations.of(0)))
              .build());
    }
    RobotDefinition template = ExampleRobot.SIM.definition();
    return RobotDefinitionBuilder.builder(template)
        .drivetrain(TestWiring.wired(template.drivetrain(), SwerveWiring.PIGEON_2.can(30), modules))
        .pdh(Pdh.can(1))
        .build();
  }

  @Test
  void eachModulesChannelsReadWhatItsDriveAndSteeringDraw() {
    RobotDefinition robot = fed();
    DriveSimulation drivetrain =
        new DriveSimulation(
            robot.body(), RobotSim.massOf(robot), robot.drivetrain(), SensorErrors.NONE);
    PdhIO pdh = new PdhIOSim(robot, new JointSimulation(robot), drivetrain);
    for (int i = 0; i < 4; i++) {
      ModuleIOSim module = drivetrain.moduleSim(i);
      module.configure(
          ExampleRobot.SIM.drivetrain().gains(), ExampleRobot.SIM.drivetrain().limits());
      module.setDriveVoltage(6);
      module.setSteerAngle(Rotation2d.CCW_90DEG);
    }
    PdhIOInputsAutoLogged inputs = new PdhIOInputsAutoLogged();

    drivetrain.step(0.02, 20_000_000L);
    pdh.updateInputs(inputs);

    ModuleIOSim frontLeft = drivetrain.moduleSim(0);
    assertThat(inputs.connected).isTrue();
    assertThat(inputs.volts).isEqualTo(12);
    assertThat(inputs.channelCurrentAmps).hasSize(24);
    assertThat(inputs.channelCurrentAmps[0])
        .as("front left's drive, speeding up")
        .isGreaterThan(1)
        .isCloseTo(frontLeft.driveSupplyAmps(), within(1e-9));
    assertThat(inputs.channelCurrentAmps[1])
        .as("its steering, turning")
        .isGreaterThan(0)
        .isCloseTo(frontLeft.steerSupplyAmps(), within(1e-9));
    assertThat(inputs.channelCurrentAmps[8]).as("a channel that feeds nothing").isZero();
    double total = 0;
    for (double amps : inputs.channelCurrentAmps) {
      total += amps;
    }
    assertThat(inputs.totalCurrentAmps).isCloseTo(total, within(1e-9));
  }
}

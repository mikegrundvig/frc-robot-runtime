package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;

/**
 * A wheel asked to push the robot gets the volts that push takes, from the model: the current its
 * torque needs, through the gearing, across the motor's resistance. Only the part along the wheel
 * counts.
 */
class ModuleFeedforwardTest {
  /** The module's motor controller, as the test reads it: what it was last asked for. */
  private static final class Controller implements ModuleIO {
    double feedforwardVolts;

    @Override
    public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
      this.feedforwardVolts = feedforwardVolts;
    }
  }

  @Test
  void aPushAlongTheWheelAddsItsVolts() {
    Controller controller = new Controller();
    Module module = new Module(controller, 0, ExampleRobot.SIM.definition().drivetrain().module(0));
    module.configure(ExampleRobot.SIM.drivetrain().gains(), ExampleRobot.SIM.drivetrain().limits());
    // A Kraken X60 through 6.75:1 on a 2 in wheel: 50 N at the carpet is 50 x 0.0508 / 6.75 N m at
    // the rotor, that over its torque constant in amps, that across its resistance in volts.
    DCMotor kraken = DCMotor.getKrakenX60(1);
    double voltsFor50 = 50 * 0.0508 / 6.75 / kraken.Kt * kraken.R;
    SwerveModuleVelocity ahead = new SwerveModuleVelocity(1.0, Rotation2d.ZERO);

    module.runSetpoint(ahead);
    double coasting = controller.feedforwardVolts;
    module.runSetpoint(ahead, 50, 0);
    double pushing = controller.feedforwardVolts;
    module.runSetpoint(ahead, 0, 50);
    double sideways = controller.feedforwardVolts;

    assertThat(pushing - coasting).isCloseTo(voltsFor50, within(1e-9));
    assertThat(voltsFor50)
        .as("about a volt for 77 N, a wheel's share at 30 A")
        .isCloseTo(0.64, within(0.02));
    assertThat(sideways).as("across the wheel: nothing").isCloseTo(coasting, within(1e-9));
  }
}

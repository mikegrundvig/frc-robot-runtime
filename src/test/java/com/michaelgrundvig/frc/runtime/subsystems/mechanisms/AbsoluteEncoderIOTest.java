package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.simulation.DutyCycleEncoderSim;

/**
 * A port encoder's IO against WPILib's simulated duty-cycle encoder: it reads the share raw, and
 * whether pulses arrive; the offset and direction stay the joint's mechanism's.
 */
@ExtendWith(RobotSim.class)
class AbsoluteEncoderIOTest {
  @Test
  void aDutyCycleEncoderReadsItsShareRawAndWhetherItAnswers() {
    AbsoluteEncoderIO.AbsoluteEncoderIOInputs inputs =
        new AbsoluteEncoderIO.AbsoluteEncoderIOInputs();
    try (AbsoluteEncoderIOPort port =
        new AbsoluteEncoderIOPort(
            MechanismWiring.PortEncoder.dutyCycle(3)
                .offset(org.wpilib.units.Units.Rotations.of(0.4))
                .inverted())) {
      DutyCycleEncoderSim sim = new DutyCycleEncoderSim(3);
      sim.set(0.25);
      sim.setConnected(true);
      port.updateInputs(inputs);
      assertThat(inputs.connected).isTrue();
      assertThat(inputs.share).isCloseTo(0.25, within(1e-9));

      sim.setConnected(false);
      port.updateInputs(inputs);
      assertThat(inputs.connected).isFalse();
    }
  }

  @Test
  void anAnalogEncoderAlwaysAnswers() {
    AbsoluteEncoderIO.AbsoluteEncoderIOInputs inputs =
        new AbsoluteEncoderIO.AbsoluteEncoderIOInputs();
    try (AbsoluteEncoderIOPort port =
        new AbsoluteEncoderIOPort(MechanismWiring.PortEncoder.analog(2, 10))) {
      port.updateInputs(inputs);
      assertThat(inputs.connected).isTrue();
      assertThat(inputs.share).isBetween(0.0, 1.0);
    }
  }
}

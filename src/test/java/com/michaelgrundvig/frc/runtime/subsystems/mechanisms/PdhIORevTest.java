package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.Pdh;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.simulation.PDPSim;

/**
 * A REV PDH, read through WPILib against its simulated one: each channel's current, the battery's
 * voltage, and, when it reads none, that it isn't answering.
 */
@ExtendWith(RobotSim.class)
class PdhIORevTest {
  @Test
  void itReadsEachChannelAndTheBattery() {
    PDPSim pdh = new PDPSim(1);
    pdh.resetData();
    PdhIORev io = new PdhIORev(Pdh.can(1));
    pdh.setVoltage(12.4);
    pdh.setCurrent(4, 7.5);
    pdh.setCurrent(21, 1.25);
    PdhIOInputsAutoLogged inputs = new PdhIOInputsAutoLogged();

    io.updateInputs(inputs);

    assertThat(inputs.connected).isTrue();
    assertThat(inputs.volts).isCloseTo(12.4, within(1e-9));
    assertThat(inputs.channelCurrentAmps).hasSize(Pdh.CHANNELS);
    assertThat(inputs.channelCurrentAmps[4]).isCloseTo(7.5, within(1e-9));
    assertThat(inputs.channelCurrentAmps[21]).isCloseTo(1.25, within(1e-9));
    assertThat(inputs.totalCurrentAmps).isCloseTo(8.75, within(1e-9));
  }

  /** A PDH that reads no battery isn't answering, once it's been so for half a second. */
  @Test
  void onePastHalfASecondWithNoBatteryIsntAnswering() {
    PDPSim pdh = new PDPSim(1);
    pdh.resetData();
    PdhIORev io = new PdhIORev(Pdh.can(1));
    pdh.setVoltage(0);
    PdhIOInputsAutoLogged inputs = new PdhIOInputsAutoLogged();

    io.updateInputs(inputs);
    assertThat(inputs.connected).as("not yet half a second").isTrue();
    for (int i = 0; i < 30; i++) {
      RobotSim.stepClock();
    }
    io.updateInputs(inputs);

    assertThat(inputs.connected).isFalse();
  }
}

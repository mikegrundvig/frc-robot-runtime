package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.simulation.DIOSim;

/**
 * Each sensor's IO against WPILib's and Phoenix's own simulated devices: a switch reads tripped or
 * clear whatever its polarity, which stays behind the IO; a rangefinder reads its distance, in a
 * mode and cone set from the definition.
 */
@ExtendWith(RobotSim.class)
class SensorIOTest {
  private static final String BUS = "sensors";

  @BeforeAll
  static void quietCtreLog() {
    SignalLogger.enableAutoLogging(false);
  }

  /** Reads until something is true, up to two seconds: Phoenix's simulated devices answer late. */
  private static boolean eventually(Runnable read, BooleanSupplier done)
      throws InterruptedException {
    for (int i = 0; i < 200; i++) {
      read.run();
      if (done.getAsBoolean()) {
        return true;
      }
      Thread.sleep(10);
    }
    return false;
  }

  /**
   * On a Systemcore port, which pulls its input down: a normally open switch reads tripped high, a
   * normally closed one tripped low.
   */
  @Test
  void aPortReadsTrippedByItsPolarity() {
    SensorIO.SensorIOInputs inputs = new SensorIO.SensorIOInputs();
    try (SensorIOPort open = new SensorIOPort(4, SensorWiring.Polarity.NORMALLY_OPEN);
        SensorIOPort closed = new SensorIOPort(5, SensorWiring.Polarity.NORMALLY_CLOSED)) {
      new DIOSim(4).setValue(true);
      new DIOSim(5).setValue(true);
      open.updateInputs(inputs);
      assertThat(inputs.connected).isTrue();
      assertThat(inputs.tripped).as("normally open, high").isTrue();
      closed.updateInputs(inputs);
      assertThat(inputs.tripped).as("normally closed, high").isFalse();

      new DIOSim(4).setValue(false);
      new DIOSim(5).setValue(false);
      open.updateInputs(inputs);
      assertThat(inputs.tripped).as("normally open, low").isFalse();
      closed.updateInputs(inputs);
      assertThat(inputs.tripped).as("normally closed, low").isTrue();
    }
  }

  /**
   * Two switches on one CANdi, each told its own polarity without undoing the other's: the normally
   * open one trips as its input is pulled low, the normally closed one as its input floats, as it
   * does with its wire cut.
   */
  @Test
  void aCandiIsToldWhichStateIsTrippedForEachInput() throws InterruptedException {
    SensorIOCandi open =
        new SensorIOCandi(
            CANDI.can(CanBus.canivore(BUS), 41).s1(), SensorWiring.Polarity.NORMALLY_OPEN);
    SensorIOCandi closed =
        new SensorIOCandi(
            CANDI.can(CanBus.canivore(BUS), 41).s2(), SensorWiring.Polarity.NORMALLY_CLOSED);
    CANdi device = new CANdi(41, new CANBus(BUS));
    SensorIO.SensorIOInputs a = new SensorIO.SensorIOInputs();
    SensorIO.SensorIOInputs b = new SensorIO.SensorIOInputs();

    device.getSimState().setS1State(S1StateValue.Low);
    device.getSimState().setS2State(S2StateValue.Floating);
    boolean tripped =
        eventually(
            () -> {
              open.updateInputs(a);
              closed.updateInputs(b);
            },
            () -> a.connected && b.connected && a.tripped && b.tripped);
    device.getSimState().setS1State(S1StateValue.Floating);
    device.getSimState().setS2State(S2StateValue.Low);
    boolean clear =
        eventually(
            () -> {
              open.updateInputs(a);
              closed.updateInputs(b);
            },
            () -> !a.tripped && !b.tripped);

    assertThat(tripped).as("S1 low, normally open; S2 floating, normally closed").isTrue();
    assertThat(clear).as("S1 floating; S2 low").isTrue();
  }

  /** A CANrange reads its distance, in meters. */
  @Test
  void aCanRangeReadsItsDistance() throws InterruptedException {
    Sensor.Rangefinder hopper = rangefinder(3, 6.75);
    SensorIOCanRange io = new SensorIOCanRange(hopper, CANRANGE.can(CanBus.canivore(BUS), 42));
    new CANrange(42, new CANBus(BUS)).getSimState().setDistance(0.3);
    SensorIO.SensorIOInputs inputs = new SensorIO.SensorIOInputs();

    boolean read =
        eventually(
            () -> io.updateInputs(inputs),
            () -> inputs.connected && Math.abs(inputs.meters - 0.3) < 0.001);

    assertThat(read).isTrue();
  }

  /**
   * Its mode follows the definition's range: past 1 m, long range; its cone the definition's,
   * within what a CANrange takes.
   */
  @Test
  void aCanRangesModeAndConeComeFromTheDefinition() {
    CANrangeConfiguration far = SensorIOCanRange.configuration(rangefinder(3, 6.75));
    CANrangeConfiguration near = SensorIOCanRange.configuration(rangefinder(1, 40));

    assertThat(far.ToFParams.UpdateMode).isEqualTo(UpdateModeValue.LongRangeUserFreq);
    assertThat(far.FovParams.FOVRangeX).isEqualTo(6.75);
    assertThat(near.ToFParams.UpdateMode).isEqualTo(UpdateModeValue.ShortRange100Hz);
    assertThat(near.FovParams.FOVRangeY).as("its widest").isEqualTo(27);
  }

  private static Sensor.Rangefinder rangefinder(double meters, double degrees) {
    return Sensor.Rangefinder.builder()
        .label("hopper range")
        .part(Part.CHASSIS)
        .range(Meters.of(meters))
        .fieldOfView(Degrees.of(degrees))
        .build();
  }
}

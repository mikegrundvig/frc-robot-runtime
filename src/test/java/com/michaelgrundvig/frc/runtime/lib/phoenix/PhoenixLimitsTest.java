package com.michaelgrundvig.frc.runtime.lib.phoenix;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * A Talon's limits from what's on them: it stops only on a limit switch, on its own pin with its
 * polarity or on a CANdi it reads over CAN, and never resets its position as one trips.
 */
class PhoenixLimitsTest {
  @Test
  void aLimitSwitchOnAPinStopsItAndABeamDoesnt() {
    HardwareLimitSwitchConfigs limits = new HardwareLimitSwitchConfigs();

    PhoenixMechanism.limits(
        limits,
        ControllerPins.NONE
            .with(
                SensorWiring.Direction.FORWARD,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false))
            .with(
                SensorWiring.Direction.REVERSE,
                new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true)));

    assertThat(limits.ForwardLimitEnable).as("a beam: read, not stopped on").isFalse();
    assertThat(limits.ReverseLimitEnable).isTrue();
    assertThat(limits.ReverseLimitSource).isEqualTo(ReverseLimitSourceValue.LimitSwitchPin);
    assertThat(limits.ForwardLimitType).isEqualTo(ForwardLimitTypeValue.NormallyOpen);
    assertThat(limits.ReverseLimitType).isEqualTo(ReverseLimitTypeValue.NormallyClosed);
    assertThat(limits.ReverseLimitAutosetPositionEnable).isFalse();
  }

  /** What's on a limit: a limit switch on its pin, either polarity, or on a CANdi's S1 or S2. */
  enum On {
    PIN_NORMALLY_OPEN(
        new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, true),
        "LimitSwitchPin",
        "NormallyOpen",
        0),
    PIN_NORMALLY_CLOSED(
        new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true),
        "LimitSwitchPin",
        "NormallyClosed",
        0),
    CANDI_S1(new ControllerPins.OnCandi(41, 1), "RemoteCANdiS1", "NormallyOpen", 41),
    CANDI_S2(new ControllerPins.OnCandi(41, 2), "RemoteCANdiS2", "NormallyOpen", 41);

    final ControllerPins.Limit limit;
    final String source;
    final String type;
    final int remote;

    On(ControllerPins.Limit limit, String source, String type, int remote) {
      this.limit = limit;
      this.source = source;
      this.type = type;
      this.remote = remote;
    }
  }

  /** Each direction with each limit switch on it, the other with nothing. */
  static Stream<Arguments> limitSwitches() {
    return Stream.of(SensorWiring.Direction.values())
        .flatMap(direction -> Stream.of(On.values()).map(on -> Arguments.of(direction, on)));
  }

  /**
   * The table, each direction by what's on it: a limit switch stops it, from its own pin, its
   * polarity told, or from a CANdi's input over CAN, whose closed already means tripped, by the
   * CANdi's ID; the other direction, with nothing on it, stops nothing. Neither resets its
   * position.
   */
  @ParameterizedTest
  @MethodSource("limitSwitches")
  void eachLimitSwitchIsTranslatedOnItsOwnSide(SensorWiring.Direction direction, On on) {
    HardwareLimitSwitchConfigs limits = new HardwareLimitSwitchConfigs();

    PhoenixMechanism.limits(limits, ControllerPins.NONE.with(direction, on.limit));

    boolean forward = direction == SensorWiring.Direction.FORWARD;
    assertThat(forward ? limits.ForwardLimitEnable : limits.ReverseLimitEnable).isTrue();
    assertThat(forward ? limits.ForwardLimitSource.name() : limits.ReverseLimitSource.name())
        .isEqualTo(on.source);
    assertThat(forward ? limits.ForwardLimitType.name() : limits.ReverseLimitType.name())
        .isEqualTo(on.type);
    assertThat(forward ? limits.ForwardLimitRemoteSensorID : limits.ReverseLimitRemoteSensorID)
        .isEqualTo(on.remote);
    assertThat(forward ? limits.ReverseLimitEnable : limits.ForwardLimitEnable)
        .as("nothing on the other")
        .isFalse();
    assertThat(forward ? limits.ReverseLimitSource : limits.ForwardLimitSource)
        .hasToString("LimitSwitchPin");
    assertThat(limits.ForwardLimitAutosetPositionEnable).isFalse();
    assertThat(limits.ReverseLimitAutosetPositionEnable).isFalse();
  }

  @Test
  void aCandisLimitSwitchIsReadOverCanAndNothingStopsNothing() {
    HardwareLimitSwitchConfigs limits = new HardwareLimitSwitchConfigs();

    PhoenixMechanism.limits(
        limits,
        ControllerPins.NONE.with(
            SensorWiring.Direction.FORWARD, new ControllerPins.OnCandi(41, 2)));

    assertThat(limits.ForwardLimitEnable).isTrue();
    assertThat(limits.ForwardLimitSource).isEqualTo(ForwardLimitSourceValue.RemoteCANdiS2);
    assertThat(limits.ForwardLimitRemoteSensorID).isEqualTo(41);
    assertThat(limits.ForwardLimitType)
        .as("the CANdi's closed already means tripped")
        .isEqualTo(ForwardLimitTypeValue.NormallyOpen);
    assertThat(limits.ReverseLimitEnable).as("nothing on it").isFalse();
  }
}

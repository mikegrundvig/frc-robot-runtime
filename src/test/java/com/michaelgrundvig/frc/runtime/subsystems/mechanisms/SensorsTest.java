package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfigBuilder;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Where a robot's switches and rangefinders are connected, each carrying its wiring, and the IDs
 * the definition holds them to.
 */
class SensorsTest {
  private static final CanBus BUS = CanBus.port(CanBus.Systemcore.CAN_S0);

  private static final Sensor.Rangefinder HOPPER_RANGE = range("hopper range");
  private static final Sensor.Rangefinder SHOOTER_RANGE = range("shooter range");

  /**
   * The definition holds a CANdi's and a CANrange's IDs to the rest of the bus's: a CANdi's two
   * inputs are one device, two CANranges on one ID aren't.
   */
  @Test
  void aCandisAndACanRangesIdsAreTheBussOwn() {
    Sensor.Switch down =
        JointFixtures.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER)
            .on(CANDI.can(BUS, 31).s1().normallyClosed());
    Sensor.Switch up =
        JointFixtures.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER)
            .on(CANDI.can(BUS, 31).s2().normallyClosed());
    RobotDefinition wired =
        RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
            .sensors(List.of(down, up, HOPPER_RANGE.on(CANRANGE.can(BUS, 30))))
            .build();

    assertThat(RobotConfigBuilder.builder(ExampleRobot.SIM).definition(wired).build().definition())
        .isEqualTo(wired);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(wired)
                    .sensors(
                        List.of(
                            down,
                            up,
                            HOPPER_RANGE.on(CANRANGE.can(BUS, 30)),
                            SHOOTER_RANGE.on(CANRANGE.can(BUS, 30))))
                    .build())
        .withMessage(
            "RobotDefinition[joints].devices: two CANranges on Systemcore's CAN_S0 port share CAN"
                + " ID 30, hopper range and shooter range: give each its own");
  }

  private static Sensor.Rangefinder range(String label) {
    return Sensor.Rangefinder.builder()
        .label(label)
        .part(Part.CHASSIS)
        .range(Meters.of(1.3))
        .fieldOfView(Degrees.of(27))
        .build();
  }
}

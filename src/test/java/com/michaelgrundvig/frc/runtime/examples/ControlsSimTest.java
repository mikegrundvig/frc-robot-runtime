package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Binding;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.simulation.GamepadSim;

/**
 * The guide's controls on the guide's robot, built as the robot builds a team's: the operator's
 * right trigger deploys the pivot, and the roller runs once it's out; each binding is in the
 * robot's record, with its button and its command.
 */
class ControlsSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void theOperatorsTriggerIntakesAndEachBindingIsRecorded() {
    sim.build(ExampleRobot.COMPETITION, SensorErrors.NONE);
    sim.controls(Controls::new);
    JointSimulation truth = Objects.requireNonNull(sim.world().joints());
    GamepadSim operator = sim.operator();
    RobotSim.setEnabled(true);
    sim.run(0.1);

    operator.setRightTriggerAxis(1);
    operator.notifyNewData();
    sim.run(0.2);
    double early = truth.velocity(ROLLER);
    sim.run(1.3);

    assertThat(early).as("held while the pivot swings out").isCloseTo(0, within(1.0));
    assertThat(truth.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
    assertThat(truth.velocity(ROLLER))
        .as("intaking, 2000 rpm")
        .isCloseTo(2000 * 2 * Math.PI / 60, within(20.0));
    assertThat(sim.bindings().all().stream().map(Binding::toString))
        .containsExactly(
            "driver back: on press: Reset heading",
            "operator right trigger: while held: Intake",
            "operator face right: on press: pivot → in");
  }
}

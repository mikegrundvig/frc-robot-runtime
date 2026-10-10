package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Sensor.Switch.Limit.End;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;

/**
 * A switch on a joint's motor controller's limit input is read by that joint, from its controller,
 * with the rest of the joint, and gated on like any sensor: its controller lost, so is it. Nothing
 * asks the hardware for one.
 */
@ExtendWith(RobotSim.class)
class PinnedSensorTest {
  /** The pivot on a SPARK MAX, its bottom switch on the SPARK's reverse limit input. */
  private static final Joint.Revolute ON_SPARK =
      unmotored(PIVOT)
          .motor(Motor.NEO, JointFixtures.gearing(25), SPARK_MAX.can(21))
          .limitSwitch(End.LOWER, Degrees.of(0), SensorWiring.Direction.REVERSE.normallyClosed());

  private static final RobotDefinition ROBOT = JointFixtures.ROBOT.with(ON_SPARK);

  private static final Sensor.Switch PIVOT_DOWN =
      Objects.requireNonNull(ROBOT.math().limitSwitch(ON_SPARK, End.LOWER));

  /** The pivot's controller, as a test leaves it: answering, its reverse input as set. */
  private static final class Controller implements JointIO {
    boolean connected = true;
    boolean reverse;

    @Override
    public void updateInputs(JointIOInputs inputs) {
      inputs.connected = connected;
      inputs.reverseLimit = reverse;
    }
  }

  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final Controller pivot = new Controller();
  private final List<String> askedFor = new ArrayList<>();
  private final Joints joints =
      new Joints(
          ROBOT,
          new Joints.Hardware() {
            @Override
            public JointIO joint(Joint joint) {
              return joint.equals(PIVOT) ? pivot : new JointIO() {};
            }

            @Override
            public SensorIO sensor(Sensor.Placed sensor) {
              askedFor.add(sensor.label());
              return new SensorIO() {};
            }
          },
          Joints.untuned(),
          scheduler);

  @Test
  void aSwitchOnAJointsPinIsReadThroughItsController() {
    Gate down = joints.gate(PIVOT_DOWN.tripped());

    pivot.reverse = true;
    joints.read(0, false);
    scheduler.run();
    boolean tripped = down.getAsBoolean();
    pivot.connected = false;
    joints.read(20_000_000, false);
    scheduler.run();

    assertThat(PIVOT_DOWN.label()).isEqualTo("pivot lower limit");
    assertThat(tripped).isTrue();
    assertThat(down.getAsBoolean()).as("its controller silent").isFalse();
    assertThat(down.blocking()).containsExactly("pivot lower limit lost");
    assertThat(askedFor).as("the hardware is never asked for it").isEmpty();
  }
}

package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Disabling;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIO;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.system.RobotController;

/**
 * Runs a robot's driven joints in simulation the way the robot runs them, without the rest of the
 * robot: the joints' physics, then each joint reads its sensors, then every command is canceled if
 * the joints were just disabled, then the command scheduler, then each joint applies what it was
 * asked. The class using it carries {@code @ExtendWith(RobotSim.class)}, for the stopped clock,
 * which closes every harness after its test, so no joint's alert outlives it.
 */
public final class JointsSimHarness implements AutoCloseable {
  /** The harnesses made since the last test ended: RobotSim closes them after each. */
  private static final List<JointsSimHarness> MADE = new ArrayList<>();

  public final Scheduler scheduler = Scheduler.createIndependentScheduler();
  public final JointSimulation simulation;
  public final SensorSimulation sensors;
  public final PdhIOSim pdh;
  public final Joints joints;
  private boolean enabled = true;

  /** Cancels every command in the loop the joints are disabled, as the robot's loop does. */
  private final Disabling disabling = new Disabling(scheduler);

  /**
   * The definition's joints, each driven as its block says, enabled, each where its definition says
   * it rests.
   */
  public JointsSimHarness(RobotDefinition definition) {
    this(definition, Joints.untuned());
  }

  /** The same, each joint's control as tuned live: read each loop, as on the robot. */
  public JointsSimHarness(
      RobotDefinition definition, Function<Joint, Supplier<MechanismControl>> tuned) {
    this(definition, tuned, (joint, io) -> io);
  }

  /**
   * The same, each joint's simulated IO handed through a test's own on its way to the joint's
   * mechanism: to record what it's told, or to fault it.
   */
  public JointsSimHarness(RobotDefinition definition, BiFunction<Joint, JointIO, JointIO> wrapped) {
    this(definition, Joints.untuned(), wrapped);
  }

  private JointsSimHarness(
      RobotDefinition definition,
      Function<Joint, Supplier<MechanismControl>> tuned,
      BiFunction<Joint, JointIO, JointIO> wrapped) {
    simulation = new JointSimulation(definition);
    sensors = new SensorSimulation(definition, simulation);
    pdh = new PdhIOSim(definition, simulation, null);
    joints =
        new Joints(
            definition,
            new Joints.Hardware() {
              @Override
              public JointIO joint(Joint joint) {
                return wrapped.apply(joint, simulation.io(joint));
              }

              @Override
              public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
                return simulation.absoluteEncoder(joint);
              }

              @Override
              public SensorIO sensor(Sensor.Placed sensor) {
                return sensors.io(sensor);
              }

              @Override
              public PdhIO pdh() {
                return pdh;
              }
            },
            tuned,
            scheduler);
    MADE.add(this); // once its joints are built: a definition they refuse leaves nothing
  }

  /** Enables the joints, or disables them, from the next loop. */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  /** Releases its joints' alerts. */
  @Override
  public void close() {
    joints.close();
  }

  /** Closes every harness made since the last call: RobotSim's, after each test. */
  public static void closeAll() {
    MADE.forEach(JointsSimHarness::close);
    MADE.clear();
  }

  /** One robot loop. */
  public void loop() {
    RobotSim.stepClock();
    simulation.step(RobotLoop.PERIOD_SECONDS);
    joints.read(RobotController.getTime(), enabled);
    disabling.cancelCommandsIfDisabled(enabled);
    scheduler.run();
    joints.write();
  }

  /** Robot loops for {@code seconds} of the robot's time. */
  public void run(double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      loop();
    }
  }

  /** Schedules a command, then runs loops for {@code seconds}. */
  public void run(Command command, double seconds) {
    scheduler.schedule(command);
    run(seconds);
  }

  /**
   * Robot loops until something holds, at most {@code seconds} of them: how long it took, in
   * seconds, or NaN if it never held.
   */
  public double runUntil(BooleanSupplier done, double seconds) {
    for (int i = 1; i <= RobotLoop.loops(seconds); i++) {
      loop();
      if (done.getAsBoolean()) {
        return i * RobotLoop.PERIOD_SECONDS;
      }
    }
    return Double.NaN;
  }
}

package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.Nullable;

/**
 * The PDH in the simulator: each channel reads what the simulated motor its definition says it
 * feeds draws from the battery, a joint's ({@link JointSimulation}) or a swerve module's drive or
 * steering ({@link DriveSimulation}); the battery holds 12 V, as the simulator's motors have it. A
 * channel that feeds nothing, or a device the simulator doesn't run, reads nothing.
 *
 * <p>A test can zero a channel's reading ({@link #zero}), as a tripped breaker or an unplugged
 * motor would leave it. The motor itself still runs: the simulated joint keeps every motor its
 * definition gives it, so this tests what the robot does with the reading, not the joint without
 * its motor.
 */
public final class PdhIOSim implements PdhIO {
  /** One channel, and what it reads. */
  private record Channel(int number, DoubleSupplier amps) {}

  private final List<Channel> channels = new ArrayList<>();
  private final boolean[] zeroed = new boolean[Pdh.CHANNELS];

  /**
   * @param definition what the robot is: which channel feeds which device
   * @param joints the robot's simulated joints
   * @param drivetrain the simulated drivetrain, or none, where a test runs only the joints
   */
  public PdhIOSim(
      RobotDefinition definition, JointSimulation joints, @Nullable DriveSimulation drivetrain) {
    for (Joint joint : definition.joints()) {
      int controller = 0;
      for (CanDevice device : joint.devices()) {
        if (!device.kind().motorController()) {
          continue;
        }
        int which = controller++;
        device
            .feed()
            .ifPresent(
                feed ->
                    channels.add(new Channel(feed.channel(), () -> amps(joints, joint, which))));
      }
    }
    SwerveWiring wiring = definition.drivetrain().wiring().orElse(null);
    if (wiring != null && drivetrain != null) {
      for (int i = 0; i < wiring.modules().size(); i++) {
        ModuleIOSim module = drivetrain.moduleSim(i);
        SwerveWiring.ModuleWiring each = wiring.modules().get(i);
        each.driveFeed()
            .ifPresent(feed -> channels.add(new Channel(feed.channel(), module::driveSupplyAmps)));
        each.steerFeed()
            .ifPresent(feed -> channels.add(new Channel(feed.channel(), module::steerSupplyAmps)));
      }
    }
  }

  /**
   * What one of a joint's motor controllers draws, or nothing for a joint the simulator doesn't
   * run.
   */
  private static double amps(JointSimulation joints, Joint joint, int controller) {
    return joints.simulates(joint) ? joints.supplyAmps(joint, controller) : 0;
  }

  /**
   * Zeroes a channel's reading, or restores it: the PDH reads no current there, as it would with
   * the breaker tripped, while the simulated motor on it runs on.
   */
  public void zero(int channel, boolean zeroed) {
    this.zeroed[channel] = zeroed;
  }

  @Override
  public void updateInputs(PdhIOInputs inputs) {
    double[] amps = new double[Pdh.CHANNELS];
    for (Channel channel : channels) {
      if (!zeroed[channel.number()]) {
        amps[channel.number()] += channel.amps().getAsDouble();
      }
    }
    double total = 0;
    for (double each : amps) {
      total += each;
    }
    inputs.connected = true;
    inputs.channelCurrentAmps = amps;
    inputs.totalCurrentAmps = total;
    inputs.volts = Motor.NOMINAL_VOLTS; // the simulator's battery, which its motors draw from
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The robot's power distribution hub (PDH), as the robot program reads it.
 *
 * <p>Once a loop it is read through its IO ({@link PdhIO}): the current on each channel, their
 * total, and the battery's voltage, all logged under {@code PDH}. The current of each device the
 * PDH feeds is also logged by the device's name ({@code PDH/Devices/<name>}), since the robot's
 * definition says which channel feeds which device.
 *
 * <p>A robot whose definition declares no PDH reads none, and logs nothing of it.
 */
public final class Power {
  /** One device the PDH feeds, and where its current is logged. */
  private record Fed(String key, int channel) {}

  private final Optional<Pdh> pdh;
  private final PdhIO io;
  private final PdhIOInputsAutoLogged inputs = new PdhIOInputsAutoLogged();
  private final List<Fed> fed;

  /**
   * @param definition what the robot is: its PDH, if it declares one, and the devices it feeds
   * @param io its hardware: the PDH, the simulator's, or none, in replay or with no PDH declared
   */
  public Power(RobotDefinition definition, PdhIO io) {
    pdh = definition.pdh();
    this.io = io;
    fed =
        definition.devices().stream()
            .filter(device -> device.feed().isPresent())
            .map(device -> new Fed("PDH/Devices/" + device.name(), channel(device)))
            .toList();
  }

  /** Reads the PDH, if the robot declares one: once a loop, after the joints. */
  void read() {
    if (pdh.isEmpty()) {
      return;
    }
    io.updateInputs(inputs);
    Logger.processInputs("PDH", inputs);
    for (Fed each : fed) {
      Logger.recordOutput(each.key(), amps(each.channel()));
    }
  }

  /**
   * Whether the channels' currents are read this loop: the robot declares a PDH, and the PDH
   * answers.
   */
  public boolean reads() {
    return pdh.isPresent() && inputs.connected;
  }

  /**
   * The current a channel draws this loop, in amps. It is NaN while the PDH is not read, and for a
   * channel the PDH does not report.
   */
  public double amps(int channel) {
    boolean isReported = reads() && channel < inputs.channelCurrentAmps.length;
    if (!isReported) {
      return Double.NaN;
    }
    return inputs.channelCurrentAmps[channel];
  }

  /**
   * The current all the channels draw together this loop, in amps, or NaN while the PDH is not
   * read.
   */
  public double totalAmps() {
    return reads() ? inputs.totalCurrentAmps : Double.NaN;
  }

  private static int channel(CanDevice device) {
    return device.feed().orElseThrow().channel();
  }
}

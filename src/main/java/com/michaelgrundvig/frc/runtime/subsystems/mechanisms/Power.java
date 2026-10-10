package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The robot's PDH, as its program reads it: once a loop, through its IO ({@link PdhIO}), each
 * channel's current, their total and the battery's voltage, logged under {@code PDH}; and each
 * device it feeds, by name, as the definition says which channel feeds it ({@code
 * PDH/Devices/<name>}). A robot that declares no PDH reads none, and logs nothing of it.
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
   * Whether the robot declares a PDH, and it answers this loop: its channels' currents are read.
   */
  public boolean reads() {
    return pdh.isPresent() && inputs.connected;
  }

  /**
   * The current a channel draws this loop, in amps: NaN while the PDH isn't read, or for a channel
   * it doesn't report.
   */
  public double amps(int channel) {
    return reads() && channel < inputs.channelCurrentAmps.length
        ? inputs.channelCurrentAmps[channel]
        : Double.NaN;
  }

  /** The current its channels draw together this loop, in amps: NaN while it isn't read. */
  public double totalAmps() {
    return reads() ? inputs.totalCurrentAmps : Double.NaN;
  }

  private static int channel(CanDevice device) {
    return device.feed().orElseThrow().channel();
  }
}

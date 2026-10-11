package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.Optional;

/**
 * A joint's absolute encoder in the simulator, where the robot reads the encoder itself. It reads
 * where the joint really is and reports it as its port would: as a share of a turn, with the port's
 * offset and direction in it, so the joint's mechanism undoes them and seeds from it exactly as it
 * does from a real one. A test can unplug it, freeze it, or slip it on its shaft ({@link
 * JointSimulation#unplug}, {@link JointSimulation#freeze}, {@link JointSimulation#slip}).
 */
final class AbsoluteEncoderIOSim implements AbsoluteEncoderIO {
  private final JointIOSim joint;

  /** The encoder's turns for each of the joint's units: for each radian, or for each meter. */
  private final double turnsPerUnit;

  /**
   * The Systemcore port it is wired to, which turns a position into the raw signal the port reads.
   * Empty for an encoder with no wiring, which reports the share of a turn as it is.
   */
  private final Optional<MechanismWiring.PortEncoder> port;

  private final EncoderReading reading = new EncoderReading();
  private boolean connected = true;

  AbsoluteEncoderIOSim(
      JointIOSim joint, Sensor.Encoder model, Optional<MechanismWiring.PortEncoder> port) {
    this.joint = joint;
    double unitsPerTurn = joint.joint() instanceof Joint.Revolute ? 2 * Math.PI : 1;
    turnsPerUnit = Math.abs(model.ratio(joint.joint())) / unitsPerTurn;
    this.port = port;
  }

  /** Unplugs it, or plugs it back in: it reads nothing while unplugged. */
  void plugIn(boolean connected) {
    this.connected = connected;
  }

  /** Its reading, for a test to freeze or slip. */
  EncoderReading reading() {
    return reading;
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    inputs.connected = connected;
    double turns = reading.of(joint.position()) * turnsPerUnit;
    if (port.isPresent()) {
      inputs.share = port.get().raw(turns);
      return;
    }
    // An absolute encoder tells one turn from the next only by the share of the turn it is in.
    inputs.share = turns - Math.floor(turns);
  }
}

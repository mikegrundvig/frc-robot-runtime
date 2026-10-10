package com.michaelgrundvig.frc.runtime.sim;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Sensor;
import org.jspecify.annotations.Nullable;

/**
 * A joint's absolute encoder in the simulator: it reads where the joint really is, as its port
 * would, its offset and direction undone, so the joint's mechanism seeds from it as from a real
 * one. Unplugged, frozen or slipped on a test's word ({@link JointSimulation#unplug}, {@link
 * JointSimulation#freeze}, {@link JointSimulation#slip}).
 */
final class AbsoluteEncoderIOSim implements AbsoluteEncoderIO {
  private final JointIOSim joint;

  /** Its signal's turns for each of the joint's units: radians, or meters. */
  private final double perUnit;

  private final MechanismWiring.@Nullable PortEncoder port;

  boolean connected = true;

  /** Whether its reading is stuck where it was, as a frozen encoder's is. */
  boolean frozen;

  /** How far off it reads the joint, in the joint's units: its magnet turned on its shaft. */
  double slip;

  /** Its reading as last read: what it reads while frozen. */
  private double share = Double.NaN;

  AbsoluteEncoderIOSim(
      JointIOSim joint, Sensor.Encoder model, MechanismWiring.@Nullable PortEncoder port) {
    this.joint = joint;
    double perTurn = joint.joint() instanceof Joint.Revolute ? 2 * Math.PI : 1;
    perUnit = Math.abs(model.ratio(joint.joint())) / perTurn;
    this.port = port;
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    inputs.connected = connected;
    if (!frozen || Double.isNaN(share)) {
      double reading = (joint.position() + slip) * perUnit;
      MechanismWiring.PortEncoder wired = port;
      share = wired != null ? wired.raw(reading) : reading - Math.floor(reading);
    }
    inputs.share = share;
  }
}

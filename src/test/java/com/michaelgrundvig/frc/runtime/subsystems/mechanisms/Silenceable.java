package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismControl;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A simulated joint's IO that a test can silence, so it reads as motor controllers that aren't
 * there, or say they didn't take their settings, or whose frames don't come for a while, so its
 * readings aren't fresh: what the simulator's joints don't do of themselves, but go silent ({@code
 * JointSimulation.silence}), or whose position reads as something that isn't a number. It notes
 * anything it's told that isn't a number, which no motor controller should be.
 */
final class Silenceable implements JointIO {
  private final JointIO joint;
  boolean silent;
  boolean stale;
  String unconfigured = "";

  /** Whether its position reads as something that isn't a number, as a bad ratio's would. */
  boolean readsNoNumber;

  /** Each request it was told with an argument that isn't a number. */
  final List<String> notNumbers = new ArrayList<>();

  private void check(String request, double... values) {
    for (double value : values) {
      if (!Double.isFinite(value)) {
        notNumbers.add(request + " " + Arrays.toString(values));
        return;
      }
    }
  }

  Silenceable(JointIO joint) {
    this.joint = joint;
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    joint.updateInputs(inputs);
    if (silent) {
      inputs.connected = false;
    }
    inputs.fresh = !stale;
    inputs.unconfigured = unconfigured;
    if (readsNoNumber) {
      inputs.position = Double.NaN;
    }
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    check("position", position, velocity, feedforwardVolts);
    joint.setPosition(position, velocity, feedforwardVolts);
  }

  @Override
  public void setVelocity(double velocity) {
    check("velocity", velocity);
    joint.setVelocity(velocity);
  }

  @Override
  public void setVoltage(double volts) {
    check("volts", volts);
    joint.setVoltage(volts);
  }

  @Override
  public void setDutyCycle(double share) {
    check("share", share);
    joint.setDutyCycle(share);
  }

  @Override
  public void stop() {
    joint.stop();
  }

  @Override
  public void home(double volts) {
    check("home", volts);
    joint.home(volts);
  }

  @Override
  public void resetPosition(double position) {
    check("reset", position);
    joint.resetPosition(position);
  }

  @Override
  public void retune(MechanismControl control) {
    joint.retune(control);
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismControl;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * A joint's IO that notes what it's told and what it last read, on its way to the joint's own: for
 * checks on what the robot asks of its joints ({@link JointInvariants}). It changes nothing.
 */
public final class WatchedJoint implements JointIO {
  /**
   * One request the joint's IO was told.
   *
   * @param kind which: "position", "velocity", "voltage", "duty cycle", "stop", "home", "reset"
   * @param values its numbers, in the joint's units: a position's point, speed and added volts
   */
  public record Told(String kind, double... values) {
    /** Whether every number it was told is one: not NaN, nor infinite. */
    boolean numbers() {
      return Arrays.stream(values).allMatch(Double::isFinite);
    }

    @Override
    public String toString() {
      StringBuilder text = new StringBuilder(kind);
      for (double value : values) {
        text.append(String.format(Locale.ROOT, " %.4g", value));
      }
      return text.toString();
    }
  }

  private final JointIO joint;

  /** What it was told since it was last {@link #cleared}, in order. */
  private final List<Told> told = new ArrayList<>();

  /** Its last reading: where its motor controller says the joint is, and how fast it moves. */
  private double position = Double.NaN;

  private double motorPosition = Double.NaN;
  private double velocity;
  private double supplyAmps;
  private boolean connected;
  private boolean followersConnected;
  private boolean restarted;

  /**
   * @param joint the joint's own IO, the simulator's
   */
  public WatchedJoint(JointIO joint) {
    this.joint = joint;
  }

  /** What it was told since it was last cleared, in order. */
  List<Told> told() {
    return List.copyOf(told);
  }

  /** Forgets what it was told: a loop's checks are done. */
  void cleared() {
    told.clear();
  }

  /**
   * Where its motor's own count says the joint is, as last read: its motor position, where the
   * position it reads is an absolute encoder's.
   */
  double onMotor() {
    return Double.isNaN(motorPosition) ? position : motorPosition;
  }

  /** How fast the joint moved, as last read. */
  double velocity() {
    return velocity;
  }

  /** What its leading motor controller draws from the battery, by its own reading, as last read. */
  double supplyAmps() {
    return supplyAmps;
  }

  /** Whether its motor controllers answered, as last read. */
  boolean connected() {
    return connected;
  }

  /** Whether each of its followers answered, as last read: true for a joint with none. */
  boolean followersConnected() {
    return followersConnected;
  }

  /** Whether its leading motor controller said it had restarted, as last read. */
  boolean restarted() {
    return restarted;
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    joint.updateInputs(inputs);
    position = inputs.position;
    motorPosition = inputs.motorPosition;
    velocity = inputs.velocity;
    supplyAmps = inputs.supplyCurrentAmps;
    connected = inputs.connected;
    followersConnected = true;
    for (boolean follower : inputs.followersConnected) {
      followersConnected &= follower;
    }
    restarted = inputs.restarted;
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    told.add(new Told("position", position, velocity, feedforwardVolts));
    joint.setPosition(position, velocity, feedforwardVolts);
  }

  @Override
  public void setVelocity(double velocity) {
    told.add(new Told("velocity", velocity));
    joint.setVelocity(velocity);
  }

  @Override
  public void setVoltage(double volts) {
    told.add(new Told("voltage", volts));
    joint.setVoltage(volts);
  }

  @Override
  public void setDutyCycle(double share) {
    told.add(new Told("duty cycle", share));
    joint.setDutyCycle(share);
  }

  @Override
  public void stop() {
    told.add(new Told("stop"));
    joint.stop();
  }

  @Override
  public void home(double volts) {
    told.add(new Told("home", volts));
    joint.home(volts);
  }

  @Override
  public void resetPosition(double position) {
    told.add(new Told("reset", position));
    joint.resetPosition(position);
  }

  @Override
  public void retune(MechanismControl control) {
    joint.retune(control);
  }
}

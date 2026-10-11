package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import org.littletonrobotics.junction.Logger;

/**
 * Where one joint logs what it does, under {@code Joints/<label>}. The keys are worked out once, so
 * that logging builds no text each loop.
 *
 * <p>The keys a person reads when a joint will not move are, in the order to read them: {@code
 * State}, {@code Command}, {@code Goal}, {@code At}, {@code HeldBy} and {@code Outcome}.
 */
final class JointLog {
  private final String stateKey;
  private final String commandKey;
  private final String goalKey;
  private final String goalNameKey;
  private final String atKey;
  private final String heldByKey;
  private final String setpointKey;
  private final String setpointVelocityKey;
  private final String outcomeKey;
  private final String rejectedKey;
  private final String absoluteAtKey;
  private final String switchDoubtsKey;
  private final String pdhCurrentKey;
  private final String deadChannelsKey;

  /**
   * @param key where the joint logs, {@code Joints/<label>}
   */
  JointLog(String key) {
    stateKey = key + "/State";
    commandKey = key + "/Command";
    goalKey = key + "/Goal";
    goalNameKey = key + "/GoalName";
    atKey = key + "/At";
    heldByKey = key + "/HeldBy";
    setpointKey = key + "/Setpoint";
    setpointVelocityKey = key + "/SetpointVelocity";
    outcomeKey = key + "/Outcome";
    rejectedKey = key + "/Rejected";
    absoluteAtKey = key + "/AbsoluteAt";
    switchDoubtsKey = key + "/SwitchDoubts";
    pdhCurrentKey = key + "/PdhCurrentAmps";
    deadChannelsKey = key + "/DeadChannels";
  }

  /**
   * What the joint was asked this loop.
   *
   * @param command who asked: the command's name, "outside any command", or "idle" when nothing
   *     asked
   * @param goalName the name of what was asked for ("out"), or empty when it has none
   * @param rejected a request that was refused this loop for not being a number, or empty
   */
  void asked(String command, String goalName, String rejected) {
    Logger.recordOutput(commandKey, command);
    Logger.recordOutput(goalNameKey, goalName);
    Logger.recordOutput(rejectedKey, rejected);
  }

  /**
   * What the joint did this loop.
   *
   * @param state where the joint stands, with why when it is faulted
   * @param goal where it is headed or held, or the number it was asked for while it runs at an
   *     output
   * @param at where it is, in its names for its places
   * @param heldBy what holds it short of what it was asked, or empty
   * @param outcome how the last move on it ended
   */
  void did(String state, double goal, String at, String heldBy, String outcome) {
    Logger.recordOutput(stateKey, state);
    Logger.recordOutput(goalKey, goal);
    Logger.recordOutput(atKey, at);
    Logger.recordOutput(heldByKey, heldBy);
    Logger.recordOutput(outcomeKey, outcome);
  }

  /** Where the joint's profile has it this loop, and how fast. */
  void setpoint(double position, double velocity) {
    Logger.recordOutput(setpointKey, position);
    Logger.recordOutput(setpointVelocityKey, velocity);
  }

  /** Where the joint's absolute encoder says the joint is. */
  void encoderAt(double position) {
    Logger.recordOutput(absoluteAtKey, position);
  }

  /** What each of the joint's limit switches is doubted for, in their order. */
  void switchDoubts(String[] doubts) {
    Logger.recordOutput(switchDoubtsKey, doubts);
  }

  /**
   * What the PDH reads of the joint's motors.
   *
   * @param togetherAmps the current they draw together on their channels, or NaN while the PDH is
   *     not read
   * @param deadChannels what each motor controller that draws nothing is flagged for, in their
   *     order
   */
  void power(double togetherAmps, String[] deadChannels) {
    Logger.recordOutput(pdhCurrentKey, togetherAmps);
    Logger.recordOutput(deadChannelsKey, deadChannels);
  }
}

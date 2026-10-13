package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.record.Holder;
import com.michaelgrundvig.frc.world.record.JointOwner;
import com.michaelgrundvig.frc.world.record.JointPlace;
import com.michaelgrundvig.frc.world.record.JointState;
import com.michaelgrundvig.frc.world.record.MoveOutcome;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Where one joint logs what it does, under {@code Joints/<label>}. The keys are worked out once, so
 * that logging builds no text each loop.
 *
 * <p>What the joint did is logged as facts: records and enums from the robot's model ({@code
 * com.michaelgrundvig.frc.world.record}), which name each thing by where it is in the robot's
 * definition. Text is logged only where it is a team's own, or a message with numbers in it: a
 * command's name, a move's name, a fault. A sentence is made from the facts wherever a person reads
 * one, by the model's functions for it.
 *
 * <p>The keys a person reads when a joint will not move are, in the order to read them: {@code
 * State} (with {@code Fault}), {@code Owner} (with {@code Command}), {@code Goal}, {@code At},
 * {@code HeldBy} and {@code Outcome} (with {@code OutcomeName}, {@code OutcomeWhy} and {@code
 * OutcomeHeldBy}).
 *
 * <p>A record is logged in the loop it changes and not again until it changes: a logged value stays
 * in the log until another replaces it, and logging a record costs an allocation each time. A
 * steady joint therefore logs none.
 */
final class JointLog {
  private static final Holder[] NOTHING_HOLDS = new Holder[0];

  private final String stateKey;
  private final String faultKey;
  private final String ownerKey;
  private final String commandKey;
  private final String goalKey;
  private final String goalNameKey;
  private final String atKey;
  private final String heldByKey;
  private final String setpointKey;
  private final String setpointVelocityKey;
  private final String outcomeKey;
  private final String outcomeNameKey;
  private final String outcomeWhyKey;
  private final String outcomeHeldByKey;
  private final String rejectedKey;
  private final String absoluteAtKey;
  private final String switchDoubtsKey;
  private final String pdhCurrentKey;
  private final String deadChannelsKey;

  // What was last logged of each record, which is logged again only when it is another.

  private Optional<JointState> state = Optional.empty();
  private Optional<String> fault = Optional.empty();
  private Optional<JointOwner> owner = Optional.empty();
  private Optional<JointPlace> at = Optional.empty();
  private Optional<List<Holder>> heldBy = Optional.empty();
  private Optional<MoveEnding> outcome = Optional.empty();

  /**
   * @param key where the joint logs, {@code Joints/<label>}
   */
  JointLog(String key) {
    stateKey = key + "/State";
    faultKey = key + "/Fault";
    ownerKey = key + "/Owner";
    commandKey = key + "/Command";
    goalKey = key + "/Goal";
    goalNameKey = key + "/GoalName";
    atKey = key + "/At";
    heldByKey = key + "/HeldBy";
    setpointKey = key + "/Setpoint";
    setpointVelocityKey = key + "/SetpointVelocity";
    outcomeKey = key + "/Outcome";
    outcomeNameKey = key + "/OutcomeName";
    outcomeWhyKey = key + "/OutcomeWhy";
    outcomeHeldByKey = key + "/OutcomeHeldBy";
    rejectedKey = key + "/Rejected";
    absoluteAtKey = key + "/AbsoluteAt";
    switchDoubtsKey = key + "/SwitchDoubts";
    pdhCurrentKey = key + "/PdhCurrentAmps";
    deadChannelsKey = key + "/DeadChannels";
  }

  /**
   * What the joint was asked this loop.
   *
   * @param owner who asked: a command, code outside any command, or nothing
   * @param command the name of the command that asked, or empty when none did
   * @param goalName the name of what was asked for ("out"), or empty when it has none
   * @param rejected a request that was refused this loop for not being a number, or empty
   */
  void asked(JointOwner owner, String command, String goalName, String rejected) {
    boolean ownerChanged = this.owner.isEmpty() || this.owner.get() != owner;
    if (ownerChanged) {
      this.owner = Optional.of(owner);
      Logger.recordOutput(ownerKey, owner);
    }
    Logger.recordOutput(commandKey, command);
    Logger.recordOutput(goalNameKey, goalName);
    Logger.recordOutput(rejectedKey, rejected);
  }

  /**
   * What the joint did this loop.
   *
   * @param state where the joint stands
   * @param fault the text of its fault, or empty while it is not faulted
   * @param goal where it is headed or held, or the number it was asked for while it runs at an
   *     output
   * @param at where it is among its places, or what it runs at
   * @param heldBy what holds it short of what it was asked, or empty
   */
  void did(JointState state, String fault, double goal, JointPlace at, List<Holder> heldBy) {
    stood(state, fault);
    Logger.recordOutput(goalKey, goal);
    boolean movedToAnotherPlace = this.at.isEmpty() || this.at.get() != at;
    if (movedToAnotherPlace) {
      this.at = Optional.of(at);
      Logger.recordOutput(atKey, at);
    }
    boolean heldByChanged = this.heldBy.isEmpty() || this.heldBy.get() != heldBy;
    if (heldByChanged) {
      this.heldBy = Optional.of(heldBy);
      Logger.recordOutput(heldByKey, heldBy.toArray(NOTHING_HOLDS));
    }
  }

  /** Logs where the joint stands, and its fault's text, when either is not what was last logged. */
  private void stood(JointState state, String fault) {
    boolean stateChanged = this.state.isEmpty() || this.state.get() != state;
    if (stateChanged) {
      this.state = Optional.of(state);
      Logger.recordOutput(stateKey, state);
    }
    boolean faultChanged = this.fault.isEmpty() || !this.fault.get().equals(fault);
    if (faultChanged) {
      this.fault = Optional.of(fault);
      Logger.recordOutput(faultKey, fault);
    }
  }

  /**
   * How the last move on the joint ended, logged once, when it ends: every joint's part of the
   * move, and beside them the move's name, the text of its reason, and what held its joints if it
   * timed out.
   */
  void outcome(MoveEnding ending) {
    boolean isLogged = outcome.isPresent() && outcome.get() == ending;
    if (isLogged) {
      return;
    }
    outcome = Optional.of(ending);
    Logger.recordOutput(outcomeKey, ending.parts().toArray(MoveOutcome[]::new));
    Logger.recordOutput(outcomeNameKey, ending.name());
    Logger.recordOutput(outcomeWhyKey, ending.why());
    Logger.recordOutput(outcomeHeldByKey, ending.heldAsItEnded().toArray(NOTHING_HOLDS));
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

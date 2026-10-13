package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.record.Holder;
import com.michaelgrundvig.frc.world.record.MoveOutcome;
import com.michaelgrundvig.frc.world.record.MoveReason;
import com.michaelgrundvig.frc.world.record.MoveResult;
import com.michaelgrundvig.frc.world.record.MoveSentences;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.List;
import java.util.Optional;

/**
 * How one run of a {@link Move} ended: the facts each of its joints logs, with the text that goes
 * beside them. Code and tests read the facts ({@link #result}, {@link #reason}, {@link #seconds},
 * {@link #parts}). A person reads the sentence ({@link #sentence}), which is made from the facts
 * the first time it is asked for, by the model's one function for it.
 */
public final class MoveEnding {
  private final RobotDefinition definition;
  private final String name;
  private final List<MoveOutcome> parts;
  private final List<Holder> heldAsItEnded;
  private final String why;

  /** The sentence, once it has been asked for. */
  private Optional<String> sentence = Optional.empty();

  /**
   * @param definition the robot, whose names the sentence is said in
   * @param name the move's name
   * @param parts every joint's part of the move, in the move's order: at least one
   * @param heldAsItEnded what held each of the move's joints as it timed out; empty otherwise
   * @param why the text of the move's reason, or empty for a reason the facts say whole
   */
  MoveEnding(
      RobotDefinition definition,
      String name,
      List<MoveOutcome> parts,
      List<Holder> heldAsItEnded,
      String why) {
    this.definition = definition;
    this.name = name;
    this.parts = List.copyOf(parts);
    this.heldAsItEnded = List.copyOf(heldAsItEnded);
    this.why = why;
  }

  /** The robot the move's joints are part of, which its facts point into. */
  public RobotDefinition definition() {
    return definition;
  }

  /** The move's name, which is its command's: "pivot → out". */
  public String name() {
    return name;
  }

  /** Every joint's part of the move, in the move's order. */
  public List<MoveOutcome> parts() {
    return parts;
  }

  /** What held each of the move's joints as it timed out. Empty for any other ending. */
  public List<Holder> heldAsItEnded() {
    return heldAsItEnded;
  }

  /** The text of the move's reason, or empty for a reason the facts say whole. */
  public String why() {
    return why;
  }

  /** How the move ended. */
  public MoveResult result() {
    return parts.getFirst().result();
  }

  /** Why it ended so, where the result alone does not say. */
  public MoveReason reason() {
    return parts.getFirst().reason();
  }

  /** How long the run took, in seconds. */
  public double seconds() {
    return parts.getFirst().seconds();
  }

  /** The move's ending as a person reads it: "pivot → out: ARRIVED in 0.41 s". */
  public String sentence() {
    if (sentence.isEmpty()) {
      sentence = Optional.of(MoveSentences.outcome(definition, name, parts, heldAsItEnded, why));
    }
    return sentence.get();
  }

  @Override
  public String toString() {
    return sentence();
  }
}

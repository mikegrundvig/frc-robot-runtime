package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.world.record.ControlSentences;
import com.michaelgrundvig.frc.world.record.GateTerm;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.List;

/**
 * Why a gate on where a joint is holds false while that isn't known ({@link JointView#knowsWhere}):
 * the joint's motor controllers didn't answer with this loop's readings, or they did and the joint
 * doesn't know where it is (unhomed, homing, faulted). Each gate on where a joint is makes one as
 * it is built, so that a loop that asks makes nothing.
 */
final class WhereUnknown {
  private final GateTerm silent;
  private final String silentShown;
  private final GateTerm unknown;
  private final String unknownShown;

  /**
   * @param gate the gate's name, which its terms are said in
   * @param joint the joint's index in the definition's joints
   */
  WhereUnknown(RobotDefinition definition, String gate, int joint) {
    silent = new GateTerm(GateTerm.Kind.JOINT_NOT_ANSWERING, joint);
    silentShown = ControlSentences.gateTerm(definition, gate, silent, List.of());
    unknown = new GateTerm(GateTerm.Kind.JOINT_POSITION_UNKNOWN, joint);
    unknownShown = ControlSentences.gateTerm(definition, gate, unknown, List.of());
  }

  /** Adds to a gate's terms why where the joint is isn't known this loop. */
  void addTo(Gate.Terms why, JointView view) {
    if (!view.answers()) {
      why.add(silent, silentShown);
      return;
    }
    why.add(unknown, unknownShown);
  }
}

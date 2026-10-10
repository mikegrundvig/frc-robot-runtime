package com.michaelgrundvig.frc.runtime.dashboard;

import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * The robot's state, as the drive team's dashboard shows it: what it needs to know before and
 * during a match, in words and yes-or-no. Built by name once a loop, after the robot has updated
 * everything it reads.
 *
 * @param readyToScore whether localization is good enough to score from
 * @param scoring "Ready to score", or why not
 * @param station the alliance and station, such as "Red 2"
 * @param onStart whether the robot stands on its chosen start
 * @param placement the pre-match placement check, in words
 * @param localization what the pose rests on, and how sure it is
 * @param cameras which cameras are connected, and how their measurements fare
 * @param loop how long loops take, and whether they keep up
 */
@ConfigRecord
public record RobotStatus(
    boolean readyToScore,
    String scoring,
    String station,
    boolean onStart,
    String placement,
    String localization,
    String cameras,
    String loop) {
  public static RobotStatusBuilder.ReadyToScoreStage builder() {
    return RobotStatusBuilder.builder();
  }
}

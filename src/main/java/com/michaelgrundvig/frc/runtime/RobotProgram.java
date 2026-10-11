package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.world.game.Game;
import com.michaelgrundvig.frc.world.robot.Body;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.framework.RobotBase;

/**
 * What a team's {@code Main} hands the robot program, and the one place the program starts. The
 * runtime never names a team's class. It is given what it needs here instead.
 *
 * <p>{@code Main} is one call:
 *
 * <pre>{@code
 * RobotProgram.start(
 *     Robots::current,   // which robot this is
 *     Rebuilt::game,     // this season's field and venues
 *     Controls::new);    // what it does
 * }</pre>
 *
 * @param robot which robot this is
 * @param season this season's field and venues, for a robot of this body: the model's class for the
 *     season, or a {@code Game} the program defines
 * @param controls what the robot does, and the drive team's buttons for it
 * @param build which code this is, for the log
 */
public record RobotProgram(
    Supplier<RobotConfig> robot,
    Function<Body, Game> season,
    ControlsMaker<?> controls,
    BuildInfo build) {
  /** Starts the robot program, with no build information. */
  public static void start(
      Supplier<RobotConfig> robot, Function<Body, Game> season, ControlsMaker<?> controls) {
    start(robot, season, controls, BuildInfo.UNKNOWN);
  }

  /**
   * Starts the robot program. This hands WPILib a way to make the robot, and WPILib makes it once
   * its own startup is done.
   */
  public static void start(
      Supplier<RobotConfig> robot,
      Function<Body, Game> season,
      ControlsMaker<?> controls,
      BuildInfo build) {
    RobotProgram program = new RobotProgram(robot, season, controls, build);
    RobotBase.startRobot(() -> new Robot(program));
  }
}

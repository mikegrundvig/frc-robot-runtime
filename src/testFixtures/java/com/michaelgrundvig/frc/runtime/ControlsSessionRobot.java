package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.simulator.JoystickInput;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;

/**
 * The guide's robot with its guide's controls ({@link ExampleControls}), and a scripted session on
 * top, for the check that a real log holds every key a move, a gate and a binding write ({@link
 * SessionRecording#recordWithControls}). In the simulator it does what a drive team does: it picks
 * the red alliance's station 1, enables teleop, and presses buttons from the simulator's controls.
 * The operator's face-down button deploys the pivot and the face-right button brings it in, so two
 * moves run and end. The driver's right trigger, held, starts the aimed shot: its gate to feed is
 * false while the shooter spins up and the robot turns, and true once both are done. Each press
 * fires its binding. Then something shoves the robot most of a meter sideways, so its cameras'
 * measurements are refused, and it stops.
 *
 * <p>This is a second session beside {@link ReplaySessionRobot}'s, which the replay test and a
 * team's layout checks share and which has no controls, so that neither depends on this script.
 * Like that one it runs on a clock of its own ({@link SessionClock}), a loop at a time, so it is
 * the same session on any computer, however busy.
 */
final class ControlsSessionRobot extends Robot {
  /** How many loops the recorded session runs, about ten seconds. */
  static final int LOOPS = 560;

  /** The loop in which the robot is shoved. */
  private static final int SHOVED_AT = 480;

  private static final int DRIVER = 0;
  private static final int OPERATOR = 1;

  /** The clock the recorded session runs on. A replay has none: its log says the time. */
  private final Optional<SessionClock> clock;

  private int loop;

  /**
   * @param clock the session's clock, stopped before this program was made; none in a replay
   */
  ControlsSessionRobot(Optional<SessionClock> clock) {
    super(
        new RobotProgram(
            () -> ExampleRobot.COMPETITION, Rebuilt.GAME, ExampleControls::new, BuildInfo.UNKNOWN));
    this.clock = clock;
    if (clock.isPresent()) {
      // A loop follows the last at once: the session's clock says the time, not the computer's.
      setUseTiming(false);
    }
  }

  /** Runs the session robot: {@code java com.michaelgrundvig.frc.runtime.ControlsSessionRobot}. */
  public static void main(String... args) {
    SessionRecording.start(ControlsSessionRobot::new);
  }

  @Override
  public void robotPeriodic() {
    // The script runs before the robot's code, the way Driver Station data arrives between loops.
    if (mode() == RunMode.SIM) {
      script(++loop);
    }
    super.robotPeriodic();
  }

  /**
   * How many loops the session has run, which AdvantageKit logs after every loop, and with it the
   * moment the session's clock moves ({@link SessionClock#loopIsOver}). AdvantageKit reads an
   * output like this one when a loop is over and before it gives the next loop its time, and no
   * other code of a robot program's runs between two loops. Inside a loop the clock must stand
   * still: moved there, WPILib's watchdog would take every loop for a late one, and say so.
   */
  @AutoLogOutput(key = "Session/Loops")
  int loopsRunAndTheClockMoves() {
    clock.ifPresent(SessionClock::loopIsOver);
    return loop;
  }

  private Simulator simulator() {
    return pitwall().simulator().orElseThrow();
  }

  /** One gamepad with one button pressed, or with its right trigger pulled. */
  private static JoystickInput pressing(Gamepad.Button button) {
    List<Boolean> buttons = new ArrayList<>();
    while (buttons.size() <= button.value) {
      buttons.add(false);
    }
    buttons.set(button.value, true);
    return new JoystickInput(List.of(), buttons, List.of());
  }

  /** The driver's gamepad with its right trigger pulled, and its left stick pushed up or not. */
  private static JoystickInput pullingTheRightTrigger(double leftStickUp) {
    List<Double> axes = new ArrayList<>();
    while (axes.size() <= Gamepad.Axis.RIGHT_TRIGGER.value) {
      axes.add(0.0);
    }
    axes.set(Gamepad.Axis.RIGHT_TRIGGER.value, 1.0);
    axes.set(Gamepad.Axis.LEFT_Y.value, -leftStickUp);
    return new JoystickInput(axes, List.of(), List.of());
  }

  /** The session, by loop. */
  private void script(int loop) {
    switch (loop) {
      case 5 ->
          simulator()
              .requestDriverStation(
                  station ->
                      station
                          .withAllianceStation(AllianceStationID.RED_1)
                          .withMode(RobotMode.TELEOPERATED)
                          .withEnabled(true));
      // Deploy: the pivot swings out and the move ends there.
      case 20 -> simulator().joystick(OPERATOR, pressing(Gamepad.Button.FACE_DOWN), null);
      case 25 -> simulator().joystick(OPERATOR, null, null);
      // Bring it in: a second move on the pivot.
      case 90 -> simulator().joystick(OPERATOR, pressing(Gamepad.Button.FACE_RIGHT), null);
      case 95 -> simulator().joystick(OPERATOR, null, null);
      // The aimed shot, held: its gate is false until the shooter is up and the robot is aimed.
      case 140 -> simulator().joystick(DRIVER, pullingTheRightTrigger(0), null);
      case 260 -> simulator().joystick(DRIVER, pullingTheRightTrigger(0.5), null);
      case 300 -> simulator().joystick(DRIVER, pullingTheRightTrigger(0), null);
      case 460 -> simulator().joystick(DRIVER, null, null);
      // Something shoves the robot sideways, which its wheels never feel: its cameras' next
      // measurements disagree with the estimate, and the estimator refuses them.
      case SHOVED_AT ->
          simulator()
              .requestFault(
                  Setting.of(
                      FaultCatalog.ROBOT, FaultCatalog.SHOVED, Map.of(FaultCatalog.LEFT, -0.8)));
      case LOOPS -> {
        Logger.end(); // closes the log, once it has written every loop it was handed
        System.exit(0);
      }
      default -> {
        // nothing this loop
      }
    }
  }
}

package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.simulator.JoystickInput;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.RobotBase;
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
 * fires its binding. Then it stops.
 *
 * <p>This is a second session beside {@link ReplaySessionRobot}'s, which the replay test and a
 * team's layout checks share and which has no controls, so that neither depends on this script.
 */
final class ControlsSessionRobot extends Robot {
  /** How many loops the recorded session runs, about ten seconds. */
  static final int LOOPS = 520;

  private static final int DRIVER = 0;
  private static final int OPERATOR = 1;

  private int loop;

  ControlsSessionRobot() {
    super(
        new RobotProgram(
            () -> ExampleRobot.COMPETITION,
            Rebuilt::game,
            ExampleControls::new,
            BuildInfo.UNKNOWN));
  }

  /** Runs the session robot: {@code java com.michaelgrundvig.frc.runtime.ControlsSessionRobot}. */
  public static void main(String... args) {
    RobotBase.startRobot(ControlsSessionRobot::new);
  }

  @Override
  public void robotPeriodic() {
    // The script runs before the robot's code, the way Driver Station data arrives between loops.
    if (mode() == RunMode.SIM) {
      script(++loop);
    }
    super.robotPeriodic();
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
      case LOOPS -> {
        Logger.end(); // closes the log
        System.exit(0);
      }
      default -> {
        // nothing this loop
      }
    }
  }
}

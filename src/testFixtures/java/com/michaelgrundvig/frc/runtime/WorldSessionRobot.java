package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.pitwall.simulator.JoystickInput;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import com.michaelgrundvig.frc.runtime.sim.SimulatedWorld;
import com.michaelgrundvig.frc.runtime.views.world.LiveWorldWitness;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;

/**
 * The guide's robot driven about the field in the simulator, then shoved, for the check that the
 * World view drawn from the record is the view of the robot's own objects ({@link
 * SessionRecording#recordTheWorld}). It drives so its cameras see tags and the estimate follows
 * them. Then two of its cameras go blind and something shoves it most of a meter sideways, into the
 * field, which its wheels never feel: the estimate and the truth part, the cameras' measurements
 * are refused, and the estimate recovers.
 *
 * <p>Every loop a witness logs what the live view's layers say of the robot's own objects ({@link
 * LiveWorldWitness}), beside the loop's record. Like the other session robots it runs on a clock of
 * its own ({@link SessionClock}), so it is the same session on any computer.
 */
final class WorldSessionRobot extends Robot {
  /** How many loops the recorded session runs, about twelve seconds. */
  static final int LOOPS = 600;

  /** The loop in which the robot is shoved. */
  static final int SHOVED_AT = 215;

  private static final int DRIVER = 0;

  /** The clock the recorded session runs on. */
  private final Optional<SessionClock> clock;

  /** What the live view says, logged every loop; made in the first loop, once the robot is. */
  private @Nullable LiveWorldWitness witness;

  private int loop;

  WorldSessionRobot(Optional<SessionClock> clock) {
    super(
        new RobotProgram(
            () -> ExampleRobot.COMPETITION,
            Rebuilt::game,
            ExampleControls::new,
            BuildInfo.UNKNOWN));
    this.clock = clock;
    if (clock.isPresent()) {
      // A loop follows the last at once: the session's clock says the time, not the computer's.
      setUseTiming(false);
    }
  }

  /** Runs the session robot: {@code java com.michaelgrundvig.frc.runtime.WorldSessionRobot}. */
  public static void main(String... args) {
    SessionRecording.start(WorldSessionRobot::new);
  }

  @Override
  public void robotPeriodic() {
    script(++loop);
    super.robotPeriodic();
    witness().log();
  }

  private LiveWorldWitness witness() {
    LiveWorldWitness made = witness;
    if (made == null) {
      SimulatedWorld world = container().world().orElseThrow();
      made =
          new LiveWorldWitness(
              container().matchField()::field,
              ExampleRobot.COMPETITION.definition(),
              container().worldReadings(),
              () -> world.drivetrain().truePose3d());
      witness = made;
    }
    return made;
  }

  /** The loops run, logged after every loop, and with it the moment the session's clock moves. */
  @AutoLogOutput(key = "Session/Loops")
  int loopsRunAndTheClockMoves() {
    clock.ifPresent(SessionClock::loopIsOver);
    return loop;
  }

  private Simulator simulator() {
    return pitwall().simulator().orElseThrow();
  }

  /** The driver's gamepad with its sticks pushed: forward, to the left, and turning left. */
  private static JoystickInput driving(double forward, double left, double turningLeft) {
    List<Double> axes = new ArrayList<>();
    while (axes.size() <= Gamepad.Axis.RIGHT_X.value) {
      axes.add(0.0);
    }
    axes.set(Gamepad.Axis.LEFT_Y.value, -forward);
    axes.set(Gamepad.Axis.LEFT_X.value, -left);
    axes.set(Gamepad.Axis.RIGHT_X.value, -turningLeft);
    return new JoystickInput(axes, List.of(), List.of());
  }

  private void unplug(String camera) {
    simulator().requestFault(Setting.of(FaultCatalog.camera(camera), FaultCatalog.UNPLUGGED));
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
      case 30 -> simulator().joystick(DRIVER, driving(0.5, 0, 0), null);
      case 110 -> simulator().joystick(DRIVER, driving(0, 0.4, 0.3), null);
      case 190 -> simulator().joystick(DRIVER, null, null);
      // The two cameras facing backward go blind, so the ones that still see agree against the
      // estimate once the robot is shoved, and none of them is believed at once.
      case 200 -> {
        unplug("back-left");
        unplug("back-right");
      }
      case SHOVED_AT ->
          simulator()
              .requestFault(
                  Setting.of(
                      FaultCatalog.ROBOT, FaultCatalog.SHOVED, Map.of(FaultCatalog.LEFT, -0.8)));
      case 480 -> simulator().joystick(DRIVER, driving(-0.4, 0, 0.2), null);
      case 560 -> simulator().joystick(DRIVER, null, null);
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

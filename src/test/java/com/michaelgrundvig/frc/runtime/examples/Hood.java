package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.examples.ServoHoodRobot.HOOD;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.discrete.PWM;

/**
 * A hood on two linear servos, driven by a subsystem of the team's own: the template doesn't drive
 * servos yet, so the hood's block ({@link ServoHoodRobot#HOOD}) leaves it undriven, and this tells
 * its servos where to go with WPILib's PWM. It reads the robot's own hood from its definition, its
 * range, its places by name and where each servo pushes, rather than restating them.
 * docs/mechanisms.md, "When the template can't drive it", is this code (DocExamplesTest).
 */
final class Hood implements Mechanism {
  private final Scheduler scheduler;
  private final Joint hood;
  private final RobotMath math;
  private final List<Drive> servos;
  private final List<PWM> pulses;

  /** The angle its servos were last told, in radians: NaN until they're first told. */
  private double told = Double.NaN;

  /**
   * @param robot the robot's definition: its own hood is the one driven, a practice robot's places
   * @param left the left servo's port, and the right's: the hood's block has no PWM wiring yet
   */
  Hood(Scheduler scheduler, RobotDefinition robot, PWM left, PWM right) {
    this.scheduler = scheduler;
    hood = robot.own(HOOD); // this robot's own hood
    math = robot.math();
    servos = hood.ownDrives(); // its servos as its block names them: left, then right
    pulses = List.of(left, right);
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  /** Goes to one of its places, and ends once its servos have had time to get there. */
  Command goTo(Position place) {
    return run(co -> co.wait(Seconds.of(tell(hood.own(place).value()))))
        .named("Hood to " + place.name());
  }

  /**
   * Tells each servo the share of its travel that puts the hood at an angle, held to its range, and
   * says how long the slower takes to get there. Nothing reads back where it is: it's assumed where
   * it was told before, or the first time, at the far end of its range.
   */
  private double tell(double angle) {
    Range range = hood.range().orElseThrow();
    double to = Math.max(range.lower(), Math.min(range.upper(), angle));
    double far = to - range.lower() > range.upper() - to ? range.lower() : range.upper();
    double from = Double.isNaN(told) ? far : told;
    double seconds = 0;
    for (int i = 0; i < servos.size(); i++) {
      Drive drive = servos.get(i);
      Actuator.Servo servo = (Actuator.Servo) drive.actuator();
      double length = math.coordinate(drive, Map.of(hood, to)); // eye to eye, there
      // A WCP-0408 takes a pulse of 1000 µs fully retracted, to 2000 µs fully extended.
      pulses.get(i).setPulseTimeMicroseconds((int) Math.round(1000 + 1000 * servo.command(length)));
      double travel = Math.abs(length - math.coordinate(drive, Map.of(hood, from)));
      seconds = Math.max(seconds, travel / servo.speed());
    }
    told = to;
    Logger.recordOutput("Hood/Told", Math.toDegrees(to));
    return seconds;
  }
}

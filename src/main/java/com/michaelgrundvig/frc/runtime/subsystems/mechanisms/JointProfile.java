package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.Travel;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * The profile a joint's position follows toward its goal, a loop at a time: WPILib's {@link
 * TrapezoidProfile}, at a share of the joint's cruise and its acceleration ({@link Travel}). Each
 * loop it moves its point on from where it had it the loop before, or, restarted, from where the
 * joint is measured: as a move begins, and whenever what drives the joint changes under it. Its
 * point and speed are what the joint's motor controller is told, with the acceleration it took to
 * get there, which kA turns into volts.
 */
final class JointProfile {
  private final Travel travel;

  /** The profile, at a share of the cruise. */
  private TrapezoidProfile profile;

  private double fraction = 1;

  /** Where it has the joint now, and where it's headed. */
  private final TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private final TrapezoidProfile.State goal = new TrapezoidProfile.State();

  /** Whether its next step starts from where the joint is measured. */
  private boolean restart = true;

  JointProfile(Travel travel) {
    this.travel = travel;
    profile = profile(1);
  }

  /** Its next step starts from where the joint is measured, not from where it had the joint. */
  void restart() {
    restart = true;
  }

  /**
   * One loop on toward a position, at a share of the cruise, from its point the loop before, or
   * from where the joint is, if it restarts or its point isn't a number: the position as a goal
   * from where the joint is ({@link Travel#goalNear}), the turn its interlocks judge it at. A point
   * that isn't a number, from a reading that wasn't one, would stay one from then on.
   *
   * @param position where it's headed, in the joint's units
   * @param fraction the share of the cruise, above 0 and at most 1
   * @param at where the joint is measured
   * @param velocity how fast the joint is measured moving
   * @return the acceleration its point took this loop, a second squared
   */
  double step(double position, double fraction, double at, double velocity) {
    if (fraction != this.fraction) {
      profile = profile(fraction);
      this.fraction = fraction;
    }
    if (restart || !Double.isFinite(setpoint.position) || !Double.isFinite(setpoint.velocity)) {
      setpoint.position = at;
      setpoint.velocity = velocity;
      restart = false;
    }
    goal.position = travel.goalNear(position, at);
    goal.velocity = 0;
    double before = setpoint.velocity;
    if (setpoint.position != goal.position || setpoint.velocity != 0) {
      TrapezoidProfile.State next = profile.calculate(RobotLoop.PERIOD_SECONDS, setpoint, goal);
      setpoint.position = next.position;
      setpoint.velocity = next.velocity;
    }
    return (setpoint.velocity - before) / RobotLoop.PERIOD_SECONDS;
  }

  /** Where it has the joint this loop, in its units. */
  double position() {
    return setpoint.position;
  }

  /** How fast it has the joint moving this loop, in its units a second. */
  double velocity() {
    return setpoint.velocity;
  }

  /** Where it's headed, as of its last step. */
  double goal() {
    return goal.position;
  }

  /** Whether it has got there: its point at its goal, still. */
  boolean arrived() {
    return setpoint.position == goal.position && setpoint.velocity == 0;
  }

  /**
   * How long it takes from where the joint is to a position, at a share of the cruise, in seconds.
   */
  double secondsTo(double position, double fraction, double at, double velocity) {
    return profile(fraction)
        .timeLeftUntil(
            new TrapezoidProfile.State(at, velocity),
            new TrapezoidProfile.State(travel.goalNear(position, at), 0));
  }

  private TrapezoidProfile profile(double fraction) {
    return new TrapezoidProfile(
        new TrapezoidProfile.Constraints(travel.cruise() * fraction, travel.acceleration()));
  }
}

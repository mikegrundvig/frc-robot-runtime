package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.record.Holder;
import com.michaelgrundvig.frc.world.robot.EncoderLoop;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.Optional;

/**
 * What a joint tells its motor controller each loop.
 *
 * <p>Most of what a joint is asked goes to the controller as it is: a speed, a voltage, a share of
 * the battery, or neutral. A position does not. The joint is moved toward it along a motion profile
 * ({@link JointProfile}), computed here on the robot, and each loop the controller is told only the
 * profile's next point: where the joint should be, how fast it should be moving there, and the
 * volts its acceleration takes (kA). The controller adds its own feedforward for speed, friction
 * and gravity.
 *
 * <p>A joint whose loop closes on an absolute encoder the robot reads closes that loop here too
 * ({@link EncoderLoop}): each loop it works out the volts that take the joint, from where the
 * encoder says it is, to the profile's point, and sends those volts.
 *
 * <p>Nothing that is not a number is ever sent. A joint whose position reads as something that is
 * not a number can turn its profile's point, or the volts worked out from it, into something that
 * is not a number either. The joint is then put in neutral for that loop, and what it was told says
 * why ({@link #drive}).
 */
final class JointOutput {
  private final RobotMath math;
  private final Joint joint;
  private final JointIO io;
  private final JointReadings readings;
  private final JointProfile profile;

  /**
   * The volts the joint's motors take for each unit of acceleration, from its controller's plan.
   */
  private final double kA;

  /**
   * The loop the joint closes itself on its absolute encoder, where its port's wiring says so
   * ({@code PortEncoder#closesLoop}). Empty where the motor controller runs the loop on the motor's
   * own encoder.
   */
  private Optional<EncoderLoop> encoderLoop;

  /**
   * What the joint is told in a loop in which following its profile would have sent something that
   * is not a number: neutral, saying that its position is not a number. It is built once, so that a
   * joint whose position stays that way allocates nothing.
   */
  private final Demand positionLost;

  /** How one loop of following the profile went. */
  private enum Followed {
    /**
     * The motor controller was told the profile's point, or the volts that take the joint there.
     */
    TOLD,
    /** What would have been told was not a number, so the motors were put in neutral instead. */
    NOT_A_NUMBER
  }

  /**
   * @param jointIndex where the joint is in the definition's joints, which is how the log names it
   *     in what holds it
   */
  JointOutput(
      RobotMath math,
      Joint joint,
      int jointIndex,
      JointIO io,
      JointReadings readings,
      JointProfile profile,
      double kA,
      Optional<EncoderLoop> encoderLoop) {
    this.math = math;
    this.joint = joint;
    this.io = io;
    this.readings = readings;
    this.profile = profile;
    this.kA = kA;
    this.encoderLoop = encoderLoop;
    positionLost = Demand.NEUTRAL.heldTo(JointAsk.NEUTRAL, Holder.noPosition(jointIndex));
  }

  /** Whether the joint closes its loop here, on its absolute encoder. */
  boolean closesLoopOnEncoder() {
    return encoderLoop.isPresent();
  }

  /**
   * Takes the joint's control as the pit has just tuned it. The motor controller takes the new
   * gains and current limits, and so does the loop the joint closes itself, if it closes one.
   */
  void retune(MechanismControl control) {
    io.retune(control);
    if (encoderLoop.isPresent()) {
      encoderLoop = Optional.of(ControllerSettings.encoderLoop(math, joint, control));
    }
  }

  /** Tells the motor controller where the joint is, once its homing has found it. */
  void tellWhereItIs(double position) {
    io.resetPosition(position);
  }

  /**
   * Tells the motor controller what the joint is to do this loop.
   *
   * @return what the motor controller was told. That is the demand itself, except in a loop in
   *     which following the profile would have sent something that is not a number: the joint is
   *     then neutral, and what is handed back says that its position is not a number.
   */
  Demand drive(Demand demand) {
    if (demand.restartProfile()) {
      profile.restart();
    }
    Demand told = demand;
    switch (demand.ask()) {
      case JointAsk.ToPosition toPosition -> {
        if (followProfile(toPosition) == Followed.NOT_A_NUMBER) {
          told = positionLost;
        }
      }
      case JointAsk.AtVelocity atVelocity -> io.setVelocity(atVelocity.speed());
      case JointAsk.AtVoltage atVoltage -> io.setVoltage(atVoltage.volts());
      case JointAsk.AtDutyCycle atDutyCycle -> io.setDutyCycle(atDutyCycle.share());
      case JointAsk.Neutral neutral -> io.stop();
    }
    boolean isOnItsProfile = told.ask() instanceof JointAsk.ToPosition;
    if (!isOnItsProfile) {
      // The profile has lost track of a joint that is driven some other way. When the joint is
      // next sent to a position, the profile starts from where the joint is then.
      profile.restart();
    }
    return told;
  }

  /**
   * Moves the joint one loop on along its profile toward a position, and tells the motor controller
   * the profile's new point, or, for a joint that closes its loop here, the volts that take it
   * there.
   *
   * <p>The profile and the loop closed here both work from where the joint reads. A reading that is
   * not a number can make the point, or the volts, not a number: a turret's goal is the turn
   * nearest where it reads, and the loop closed here corrects by how far the reading is from the
   * point. Whatever would be sent is checked first, and if any of it is not a number the motors are
   * put in neutral instead.
   */
  private Followed followProfile(JointAsk.ToPosition toPosition) {
    double acceleration =
        profile.step(
            toPosition.position(),
            toPosition.cruiseShare(),
            readings.position(),
            readings.velocity());
    double accelerationVolts = kA * acceleration;
    boolean pointIsANumber =
        Double.isFinite(profile.position())
            && Double.isFinite(profile.velocity())
            && Double.isFinite(accelerationVolts);
    if (!pointIsANumber) {
      io.stop();
      return Followed.NOT_A_NUMBER;
    }
    if (encoderLoop.isEmpty()) {
      io.setPosition(profile.position(), profile.velocity(), accelerationVolts);
      return Followed.TOLD;
    }
    double volts =
        encoderLoop
            .get()
            .volts(
                profile.position(),
                profile.velocity(),
                accelerationVolts,
                readings.position(),
                readings.velocity());
    if (!Double.isFinite(volts)) {
      io.stop();
      return Followed.NOT_A_NUMBER;
    }
    io.setVoltage(volts);
    return Followed.TOLD;
  }

  /**
   * Pushes the joint at a voltage for its homing. The push ignores the joint's soft limits, which
   * are in the wrong place until the joint is homed.
   */
  void pushHome(double volts) {
    io.home(volts);
    profile.restart();
  }

  /**
   * Holds the joint at a position by its motor's own count, on the motor controller's own loop.
   * This is for a joint that has lost the absolute encoder its loop closes on, so that it does not
   * go limp.
   */
  void holdOnMotor(double position) {
    io.setPosition(position, 0, 0);
    profile.restart();
  }

  /** Puts the joint's motors in neutral: they brake or coast, as the joint's control says. */
  void stop() {
    io.stop();
    profile.restart();
  }
}

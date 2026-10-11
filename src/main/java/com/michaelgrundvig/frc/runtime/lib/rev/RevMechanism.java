package com.michaelgrundvig.frc.runtime.lib.rev;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A mechanism's REV SPARK settings, translated from how the program drives its joint ({@link
 * ControllerPlan}): its ratios, gains, soft limits, current limit and followers, said the way a
 * SPARK MAX or a SPARK Flex takes them. The plan owns every value. This class only says them in a
 * SPARK's units, and refuses what a SPARK can't take. Every function here is pure: nothing touches
 * a device.
 *
 * <p><b>Units.</b> A SPARK reads its motor's own encoder in the motor's rotations, and its speed in
 * rotations a minute (RPM). REVLib 2027 has no conversion factors, so every number sent to a SPARK
 * is converted here.
 *
 * <p><b>What the SPARK's loop runs, and what the robot adds.</b> The SPARK is told to compensate
 * for battery voltage to {@link Motor#NOMINAL_VOLTS}, so its loop's output is a share of that
 * voltage. The loop runs the plan's kP: per rotation of error in slot 0, for positions, and per RPM
 * of error in slot 1, for speeds. Everything else in the plan's control law, its feedforward (kS,
 * kV, kA and gravity) and its kD, the robot works out each loop in volts ({@link
 * ControllerPlan#feedforward}) and sends with the setpoint as the loop's arbitrary feedforward.
 * Volts are the one unit a SPARK takes exactly, whatever the units of its own feedforward and
 * derivative terms. Measured in REV's simulation (2026-10-05): a kP of 0.01 on 10 rotations of
 * error gave a share of 0.1, and 1.2 V of feedforward gave a share of 0.1.
 *
 * <p><b>A loop closed on the absolute encoder.</b> A joint's wiring can have the SPARK run its loop
 * on the absolute encoder on its data port instead of the motor ({@code
 * DataPortEncoder#closesLoop}). Then its setpoints, soft limits and kP are per turn of that
 * encoder, and its speeds per RPM of that encoder. The SPARK is set to read the encoder as zero at
 * the low edge of the joint's window, so the reading never wraps inside the joint's range, and
 * position wrapping is switched off: wrapping takes the short way round to a setpoint, which here
 * could be through a hard stop. The feedforward is still the robot's, and the speed its kD damps
 * toward is still the motor's. Two things here are as REV documents them and unconfirmed on a real
 * SPARK, because REV's simulation runs its loop on the motor's encoder whatever it is told: that
 * the soft limits follow the sensor the loop runs on, and that the absolute encoder's speed is in
 * RPM.
 *
 * <p><b>Current.</b> A SPARK limits the current through its motor (its "smart current limit", set
 * from the control's stator limit). It has no limit on what it draws from the battery, so the
 * control's supply limits are not applied.
 *
 * <p><b>kI.</b> The loop's kI for positions is sent in REV's units: a share of {@link
 * Motor#NOMINAL_VOLTS} per turn of error, per millisecond, since the loop runs every millisecond.
 * So it is the plan's kI per second, divided by a thousand (REV's "SPARK closed-loop units",
 * docs.revrobotics.com). REV's simulation integrates per second instead; the template's own
 * simulator integrates as REV documents the device. This is unconfirmed on a real SPARK until a
 * bench check: the integral of a known error, read back after a second.
 */
public final class RevMechanism {
  private RevMechanism() {}

  /**
   * How often a SPARK sends each signal the robot reads every loop, in milliseconds: 20 ms, the
   * robot loop's period. Sending faster would load the CAN bus for readings nothing uses.
   */
  private static final int SIGNAL_MS = 20;

  /** The SPARK's loop runs every millisecond, and its integral accumulates once each time. */
  private static final double LOOPS_PER_SECOND = 1000;

  /** A SPARK's speeds are in rotations a minute. */
  private static final double SECONDS_PER_MINUTE = 60;

  /**
   * One mechanism's SPARK settings.
   *
   * @param plan how the program drives its joint: its rotations, which positions are converted
   *     with, and the feedforward the robot works out each loop
   * @param leader the leading SPARK's settings, applied whole at startup
   * @param followers each following SPARK's settings, in the wiring's order
   * @param rotorRatio how many turns the motor makes for one rotation of the mechanism
   * @param loopRatio how many turns the sensor the loop runs on makes for one rotation of the
   *     mechanism. The sensor is the motor's encoder, or the absolute encoder when the loop is
   *     closed on it.
   * @param loopZero what that sensor reads with the mechanism at zero: 0 for the motor's encoder.
   *     The absolute encoder is read from the window's edge, so for it this is minus the edge.
   * @param positionKp the loop's kP for positions: a share of {@link Motor#NOMINAL_VOLTS} per turn
   *     of error of the sensor the loop runs on
   * @param velocityKp the loop's kP for speeds: a share per RPM of error of that sensor
   * @param positionKi the loop's kI for positions: a share of {@link Motor#NOMINAL_VOLTS} per turn
   *     of error of that sensor, per millisecond
   * @param softLimits the soft limits in that sensor's turns, lower then upper, if the control has
   *     them
   * @param absolute the definition's absolute encoder that the SPARK's data port reads, if the
   *     wiring names one. It tells the motor where the joint is at startup.
   * @param windowEdge where the SPARK's reading of that encoder is zero. An absolute encoder can
   *     tell apart one turn's worth of positions, a window ({@code RobotMath.absoluteWindow}); this
   *     is the encoder's reading at the window's low edge, in its turns from the joint's zero. With
   *     zero there, the SPARK's reading, from 0 to 1, wraps outside the joint's range. 0 without an
   *     encoder.
   * @param closedLoop whether the loop runs on that encoder, as the wiring says, and not on the
   *     motor's
   * @param notes what is doubtful about the settings, each a sentence for a log or a test
   */
  public record Settings(
      ControllerPlan plan,
      SparkBaseConfig leader,
      List<Follower> followers,
      double rotorRatio,
      double loopRatio,
      double loopZero,
      double positionKp,
      double velocityKp,
      double positionKi,
      Optional<double[]> softLimits,
      Optional<Sensor.Encoder> absolute,
      double windowEdge,
      boolean closedLoop,
      List<String> notes) {
    public Settings {
      followers = List.copyOf(followers);
      notes = List.copyOf(notes);
    }

    /** The joint's rotations, which the SPARK's motor turns are counted through. */
    public MechanismRotations rotations() {
      return plan.rotations();
    }

    /**
     * Only what a retune changes that the SPARK itself holds: the loop's kP for positions and for
     * speeds, its kI for positions, and the current limit. A configuration with nothing else set
     * changes nothing else on the SPARK.
     *
     * <p>The kI is always sent, a kI of zero too. At startup a zero is left out, because the SPARK
     * was just reset to REV's defaults; here the SPARK may hold a kI from an earlier retune, which
     * only a zero takes away.
     *
     * <p>The rest of a retune needs no sending. The plan's kD and its feedforward (kS, kV, kA and
     * gravity) are worked out on the robot each loop from {@link #plan}, and a SPARK has no supply
     * current limit to set.
     */
    public SparkBaseConfig tuning(MechanismWiring.Controller controller, MechanismControl control) {
      SparkBaseConfig tuning = blank(controller);
      tuning.smartCurrentLimit(statorAmps(control));
      tuning
          .closedLoop
          .p(positionKp, ClosedLoopSlot.kSlot0)
          .i(positionKi, ClosedLoopSlot.kSlot0)
          .p(velocityKp, ClosedLoopSlot.kSlot1);
      return tuning;
    }

    /**
     * Only the soft limits' on-off switches, both set the same way. They are switched off while the
     * joint homes past them.
     */
    public SparkBaseConfig softLimitsEnabled(
        MechanismWiring.Controller controller, boolean enabled) {
      SparkBaseConfig limits = blank(controller);
      limits.softLimit.forwardSoftLimitEnabled(enabled).reverseSoftLimitEnabled(enabled);
      return limits;
    }
  }

  /**
   * A following SPARK.
   *
   * @param id its CAN ID
   * @param config its settings: following the leader, turning the same way or the opposite way as
   *     the wiring's directions say, with the leader's current limit and idle mode
   */
  public record Follower(int id, SparkBaseConfig config) {}

  /**
   * Sets a SPARK's two limit inputs from what is wired to them ({@link ControllerPins}). Each input
   * with a switch on it is told the switch's polarity. The SPARK stops its motor on an input only
   * if the switch there is a limit switch; an input with nothing on it, or with a beam break, stops
   * nothing.
   *
   * <p>Neither input resets the SPARK's position when it trips. Working out where the joint is
   * belongs to the robot's homing, where it can be checked.
   *
   * @throws IllegalArgumentException for a limit on a CANdi, which a SPARK can't read
   */
  public static void limits(LimitSwitchConfig limits, ControllerPins pins) {
    limits
        .forwardLimitSwitchType(type(pins.forward()))
        .forwardLimitSwitchTriggerBehavior(behavior(pins.forward()))
        .reverseLimitSwitchType(type(pins.reverse()))
        .reverseLimitSwitchTriggerBehavior(behavior(pins.reverse()));
  }

  /** The polarity a SPARK is told for one limit input: normally open for one with nothing on it. */
  private static LimitSwitchConfig.Type type(Optional<ControllerPins.Limit> limit) {
    if (limit.isEmpty()) {
      return LimitSwitchConfig.Type.kNormallyOpen;
    }
    return switch (limit.get()) {
      case ControllerPins.OnPin pin ->
          pin.polarity() == SensorWiring.Polarity.NORMALLY_CLOSED
              ? LimitSwitchConfig.Type.kNormallyClosed
              : LimitSwitchConfig.Type.kNormallyOpen;
      case ControllerPins.OnCandi candi ->
          throw new IllegalArgumentException("a SPARK can't read a CANdi as its limit");
    };
  }

  /** Whether a SPARK stops its motor when one limit input trips. */
  private static LimitSwitchConfig.Behavior behavior(Optional<ControllerPins.Limit> limit) {
    boolean stopsOnIt = limit.map(ControllerPins.Limit::stops).orElse(false);
    if (stopsOnIt) {
      return LimitSwitchConfig.Behavior.kStopMovingMotor;
    }
    return LimitSwitchConfig.Behavior.kKeepMovingMotor;
  }

  /**
   * The absolute encoder on the leader's data port, with what is needed to work out its settings.
   *
   * @param math what the robot's definition derives: the window of positions the encoder tells
   *     apart
   * @param encoder the definition's absolute encoder that the data port reads
   * @param port how it is read there: its offset, its direction, and whether the loop closes on it
   */
  private record OnDataPort(
      RobotMath math, Sensor.Encoder encoder, MechanismWiring.DataPortEncoder port) {}

  /**
   * How the SPARK's reading of the absolute encoder relates to the mechanism.
   *
   * @param ratio how many turns the encoder makes for one rotation of the mechanism
   * @param windowEdge the encoder's reading at the low edge of the joint's window, in its turns
   *     from the joint's zero: where the SPARK is set to read zero
   */
  private record EncoderScale(double ratio, double windowEdge) {
    /** A mechanism with no absolute encoder: nothing is scaled by it. */
    static final EncoderScale NONE = new EncoderScale(0, 0);
  }

  /**
   * The sensor the SPARK's loop runs on, and how its turns relate to the mechanism.
   *
   * @param sensor the motor's own encoder, or the absolute encoder on the data port
   * @param ratio how many turns that sensor makes for one rotation of the mechanism
   * @param zero what that sensor reads with the mechanism at zero
   */
  private record LoopSensor(FeedbackSensor sensor, double ratio, double zero) {
    boolean isTheAbsoluteEncoder() {
      return sensor == FeedbackSensor.kAbsoluteEncoder;
    }

    /** A position in the mechanism's rotations, as this sensor reads it. */
    double turnsAt(double mechanismRotations) {
      return mechanismRotations * ratio + zero;
    }
  }

  /**
   * The settings for a mechanism on SPARKs, read by its motor's own encoder.
   *
   * @param plan how the program drives its joint, naming its motors' SPARKs, the leader first
   * @throws IllegalArgumentException for controllers that aren't SPARKs
   */
  public static Settings configure(ControllerPlan plan) {
    return build(plan, Optional.empty());
  }

  /**
   * The settings for a mechanism on SPARKs with an absolute encoder on the leader's data port. The
   * encoder tells the motor where the joint is at startup, so the joint needs no homing, and the
   * SPARK's loop runs on it where the wiring says so.
   *
   * @param math what the robot's definition derives: the window the encoder tells apart
   * @param absolute the definition's absolute encoder that the data port reads
   * @param port how it is read there: its offset, its direction, and whether the loop closes on it
   * @throws IllegalArgumentException as {@link #configure(ControllerPlan)} does, and for an encoder
   *     that isn't an absolute encoder on this joint alone, or whose joint's range covers more than
   *     one of its turns
   */
  public static Settings configure(
      RobotMath math,
      ControllerPlan plan,
      Sensor.Encoder absolute,
      MechanismWiring.DataPortEncoder port) {
    Joint joint = plan.joint();
    boolean onThisJointAlone = absolute.joints().equals(List.of(joint));
    if (!absolute.absolute() || !onThisJointAlone) {
      throw new IllegalArgumentException(
          absolute.label() + " must be an absolute encoder on " + joint.label() + " alone");
    }
    return build(plan, Optional.of(new OnDataPort(math, absolute, port)));
  }

  /** Works out every SPARK's settings for one mechanism, with or without an absolute encoder. */
  private static Settings build(ControllerPlan plan, Optional<OnDataPort> onDataPort) {
    MechanismWiring.Device leading = plan.controllers().get(0);
    MechanismWiring.Controller controller = sparkKind(plan, leading);
    List<String> notes = new ArrayList<>(plan.notes());
    MechanismRotations rotations = plan.rotations();

    EncoderScale encoder =
        onDataPort.map(each -> encoderScale(plan, each, notes)).orElse(EncoderScale.NONE);
    double rotorRatio = rotations.ratioPerRotation(plan.rotorRatio());
    boolean closedOnEncoder = onDataPort.map(each -> each.port().closedLoop()).orElse(false);
    LoopSensor loopSensor =
        closedOnEncoder
            ? new LoopSensor(
                FeedbackSensor.kAbsoluteEncoder, encoder.ratio(), -encoder.windowEdge())
            : new LoopSensor(FeedbackSensor.kPrimaryEncoder, rotorRatio, 0);
    int statorAmps = statorAmps(plan.control());

    SparkBaseConfig leader = blank(controller);
    leader.inverted(leading.turnsClockwise());
    setOutput(leader, plan, statorAmps);

    // The loop's gains, each a share of the nominal voltage per unit of error of its sensor.
    double kPPerRotation = rotations.gainPerRotation(plan.kP());
    double positionKp = kPPerRotation / (Motor.NOMINAL_VOLTS * loopSensor.ratio());
    double velocityKp =
        kPPerRotation / (Motor.NOMINAL_VOLTS * loopSensor.ratio() * SECONDS_PER_MINUTE);
    double positionKi =
        rotations.gainPerRotation(plan.kI())
            / (Motor.NOMINAL_VOLTS * loopSensor.ratio() * LOOPS_PER_SECOND);
    setLoop(leader, loopSensor, positionKp, velocityKp, positionKi);

    Optional<double[]> softLimits =
        plan.softLimits().map(limits -> setSoftLimits(leader, rotations, loopSensor, limits));
    onDataPort.ifPresent(each -> setDataPortEncoder(leader, controller, each, encoder));
    leader
        .signals
        .primaryEncoderPositionPeriodMs(SIGNAL_MS)
        .primaryEncoderVelocityPeriodMs(SIGNAL_MS)
        .appliedOutputPeriodMs(SIGNAL_MS)
        .outputCurrentPeriodMs(SIGNAL_MS)
        .busVoltagePeriodMs(SIGNAL_MS);

    return new Settings(
        plan,
        leader,
        followers(plan, controller, leading, statorAmps),
        rotorRatio,
        loopSensor.ratio(),
        loopSensor.zero(),
        positionKp,
        velocityKp,
        positionKi,
        softLimits,
        onDataPort.map(OnDataPort::encoder),
        encoder.windowEdge(),
        closedOnEncoder,
        notes);
  }

  /**
   * The kind of SPARK the mechanism's motors are on.
   *
   * @throws IllegalArgumentException for a leading controller that isn't a SPARK
   */
  private static MechanismWiring.Controller sparkKind(
      ControllerPlan plan, MechanismWiring.Device leading) {
    MechanismWiring.Controller controller = leading.controller();
    boolean isASpark =
        controller == MechanismWiring.Controller.SPARK_MAX
            || controller == MechanismWiring.Controller.SPARK_FLEX;
    if (!isASpark) {
      throw new IllegalArgumentException(
          possessive(plan.joint().label())
              + " wiring is "
              + possessive(controller.shown())
              + ", not a SPARK's");
    }
    return controller;
  }

  /**
   * How the absolute encoder's turns relate to the mechanism, and where its window's low edge is.
   * One turn of the encoder must tell the joint's whole range apart, and its reading must wrap
   * outside that range. A reading that wraps too near the range is noted.
   *
   * @throws IllegalArgumentException for a joint whose range covers more than one turn of the
   *     encoder
   */
  private static EncoderScale encoderScale(
      ControllerPlan plan, OnDataPort onDataPort, List<String> notes) {
    RobotMath math = onDataPort.math();
    Sensor.Encoder absolute = onDataPort.encoder();
    MechanismRotations rotations = plan.rotations();
    Range window = math.absoluteWindow(List.of(absolute));
    double wrapPoint = ControllerPlan.wrapPoint(math, absolute);
    Optional.ofNullable(ControllerPlan.wrapProblem(absolute, wrapPoint)).ifPresent(notes::add);
    double ratio = rotations.ratioPerRotation(Math.abs(absolute.ratio(plan.joint())));
    double windowEdge = ratio * rotations.toRotations(window.lower());
    return new EncoderScale(ratio, windowEdge);
  }

  /**
   * Sets what a leader and a follower share: whether the motor brakes or coasts when idle, the
   * limit on the current through it, and voltage compensation, which makes the SPARK's output a
   * share of {@link Motor#NOMINAL_VOLTS} whatever the battery reads.
   *
   * @return the same configuration, to set more on
   */
  private static SparkBaseConfig setOutput(
      SparkBaseConfig config, ControllerPlan plan, int statorAmps) {
    return config
        .idleMode(plan.brakes() ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(statorAmps)
        .voltageCompensation(Motor.NOMINAL_VOLTS);
  }

  /**
   * Sets the leader's loop: the sensor it runs on, its kP for positions (slot 0) and for speeds
   * (slot 1), and its kI for positions when the plan has one.
   */
  private static void setLoop(
      SparkBaseConfig leader,
      LoopSensor loopSensor,
      double positionKp,
      double velocityKp,
      double positionKi) {
    leader
        .closedLoop
        .feedbackSensor(loopSensor.sensor())
        .p(positionKp, ClosedLoopSlot.kSlot0)
        .p(velocityKp, ClosedLoopSlot.kSlot1);
    if (positionKi != 0) {
      leader.closedLoop.i(positionKi, ClosedLoopSlot.kSlot0);
    }
    if (loopSensor.isTheAbsoluteEncoder()) {
      // The encoder's reading wraps outside the joint's range, so the loop must not wrap: the
      // short way round to a setpoint could be through a hard stop.
      leader.closedLoop.positionWrappingEnabled(false);
    }
  }

  /**
   * Switches the leader's soft limits on, at the plan's limits in the turns of the sensor its loop
   * runs on.
   *
   * @return those limits, lower then upper
   */
  private static double[] setSoftLimits(
      SparkBaseConfig leader,
      MechanismRotations rotations,
      LoopSensor loopSensor,
      MechanismControl.SoftLimits limits) {
    double lower = loopSensor.turnsAt(rotations.toRotations(limits.lower()));
    double upper = loopSensor.turnsAt(rotations.toRotations(limits.upper()));
    leader
        .softLimit
        .reverseSoftLimit(lower)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(upper)
        .forwardSoftLimitEnabled(true);
    return new double[] {lower, upper};
  }

  /**
   * Sets how the leader reads the absolute encoder on its data port. The SPARK applies the
   * encoder's direction and offset itself, as a CANcoder does. The offset is chosen so the reading
   * grows with the joint and is zero at the window's low edge, which puts the point where the
   * reading wraps where the joint never is. The reading is sent once a robot loop.
   */
  private static void setDataPortEncoder(
      SparkBaseConfig leader,
      MechanismWiring.Controller controller,
      OnDataPort onDataPort,
      EncoderScale encoder) {
    if (controller == MechanismWiring.Controller.SPARK_MAX) {
      // A SPARK MAX's data port serves several kinds of sensor, and has to be told this one.
      leader.absoluteEncoder.setSparkMaxDataPortConfig();
    }
    MechanismWiring.DataPortEncoder port = onDataPort.port();
    leader.absoluteEncoder.inverted(port.reversed()).zeroOffset(port.raw(encoder.windowEdge()));
    leader.signals.absoluteEncoderPositionPeriodMs(SIGNAL_MS);
  }

  /**
   * Each follower's settings, in the wiring's order: the leader's idle mode, current limit and
   * voltage compensation, and following the leader, turning the opposite way where the wiring's
   * directions differ.
   */
  private static List<Follower> followers(
      ControllerPlan plan,
      MechanismWiring.Controller controller,
      MechanismWiring.Device leading,
      int statorAmps) {
    List<MechanismWiring.Device> controllers = plan.controllers();
    List<Follower> followers = new ArrayList<>();
    for (MechanismWiring.Device motor : controllers.subList(1, controllers.size())) {
      SparkBaseConfig follower = blank(controller);
      setOutput(follower, plan, statorAmps).follow(leading.id(), plan.opposes(motor));
      followers.add(new Follower(motor.id(), follower));
    }
    return followers;
  }

  /**
   * An empty configuration for the kind of SPARK. Only what is set on a configuration is sent, so
   * one built from this changes nothing it does not name.
   */
  private static SparkBaseConfig blank(MechanismWiring.Controller controller) {
    if (controller == MechanismWiring.Controller.SPARK_FLEX) {
      return new SparkFlexConfig();
    }
    return new SparkMaxConfig();
  }

  /** The control's stator limit as a SPARK's smart current limit takes it: whole amps. */
  private static int statorAmps(MechanismControl control) {
    return (int) Math.round(control.currentLimits().stator().in(Amps));
  }
}

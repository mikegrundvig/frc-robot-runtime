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
import org.jspecify.annotations.Nullable;

/**
 * A mechanism's REV SPARK settings, translated from how the program drives its joint ({@link
 * ControllerPlan}): its ratios, gains seeded from the definition, soft limits, current limits and
 * followers, said as a SPARK MAX or a SPARK Flex takes them. The plan owns every value; this says
 * them in a SPARK's units and refuses what a SPARK can't take. Pure functions: nothing here touches
 * a device.
 *
 * <p>A SPARK reads its motor's own encoder in rotations and RPM: REVLib 2027 has no conversion
 * factors. With its voltage compensated to {@link Motor#NOMINAL_VOLTS}, its loop's output is a
 * share of that. Its loop runs the plan's kP, per rotor rotation of error in slot 0, for positions,
 * and per rotor RPM in slot 1, for speeds. The rest, the plan's feedforward law (kS, kV, kA and
 * gravity) and its kD, the robot works out each loop in volts ({@link ControllerPlan#feedforward})
 * and adds as the loop's arbitrary feedforward: volts are the one unit a SPARK takes exactly,
 * whatever the units of its own feedforward and derivative. Measured in REV's simulation
 * (2026-10-05): kP 0.01 on 10 rotations of error gave a share of 0.1, and 1.2 V of feedforward a
 * share of 0.1.
 *
 * <p>A SPARK whose loop closes on the absolute encoder on its data port ({@code
 * DataPortEncoder#closesLoop}) runs it on that encoder's turns instead, with no conversion factor
 * either: its setpoints, its soft limits and its kP are per turn of the encoder, read from the low
 * edge of the joint's window, where the SPARK reads zero, so it never wraps inside the joint's
 * range and needs no position wrapping, which would take the short way round, through a hard stop.
 * Its speeds, on the same sensor, are per RPM of the encoder. The feedforward is the robot's, as
 * ever, and the speed it damps toward is the motor's. That REV's soft limits follow the sensor the
 * loop runs on, and that the absolute encoder's speed is in RPM, are as REV documents them, and
 * unconfirmed: REV's simulation runs its loop on the motor's own encoder whatever it's told.
 *
 * <p>A SPARK limits its motor's current (its smart limit, from the control's stator limit), not the
 * battery's, so the supply limits aren't applied.
 *
 * <p>Its kI goes to the loop for positions, in REV's units: a share of {@link Motor#NOMINAL_VOLTS}
 * per turn of error of the sensor the loop runs on, per millisecond, the loop's period, so the
 * plan's kI per second over a thousand (REV's "SPARK closed-loop units", docs.revrobotics.com).
 * REV's simulation integrates per second instead; the template's own simulator integrates as REV
 * documents the device. Unconfirmed on a SPARK until a bench check: an integral of a known error
 * read back after a second.
 */
public final class RevMechanism {
  private RevMechanism() {}

  /** A SPARK's position and speed signals, every loop: 20 ms. */
  private static final int SIGNAL_MS = 20;

  /**
   * One mechanism's SPARK settings.
   *
   * @param plan how the program drives its joint: its rotations, which its positions are converted
   *     with, and the feedforward the robot works out each loop
   * @param leader the leading SPARK's, applied whole at startup
   * @param followers each following SPARK's, in the wiring's order
   * @param rotorRatio rotor rotations for each of the mechanism's
   * @param loopRatio turns of the sensor its loop runs on for each of the mechanism's rotations:
   *     the rotor's, or, closed on its absolute encoder, the encoder's
   * @param loopZero what that sensor reads with the mechanism at zero: 0 for its rotor, and for its
   *     absolute encoder, read from the window's edge, minus the edge
   * @param positionKp the loop's kP for positions: a share of {@link Motor#NOMINAL_VOLTS} per turn
   *     of error of the sensor it runs on
   * @param velocityKp the loop's kP for speeds: a share per RPM of error of that sensor
   * @param positionKi the loop's kI for positions: a share of {@link Motor#NOMINAL_VOLTS} per turn
   *     of error of that sensor per millisecond
   * @param softLimits the soft limits, in that sensor's turns, lower then upper, if the control has
   *     them
   * @param absolute the definition's absolute encoder the SPARK's data port reads, if its wiring
   *     names one: read once at startup, to tell the motor where the joint is
   * @param windowEdge where the SPARK's reading of that encoder is zero: the encoder's reading, in
   *     its turns from the joint's zero, at the low edge of the window it tells apart ({@code
   *     RobotMath.absoluteWindow}), so the SPARK's reading, from 0 to 1, wraps outside the joint's
   *     range; 0 without one
   * @param closedLoop whether its loop runs on that encoder, as its wiring says, rather than on its
   *     rotor
   * @param notes what's doubtful about them, each a sentence for a log or a test
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

    /** The joint's rotations, which the SPARK's rotor turns are counted through. */
    public MechanismRotations rotations() {
      return plan.rotations();
    }

    /** The settings the pit tunes live, alone: the loop's gains and the current limits. */
    public SparkBaseConfig tuning(MechanismWiring.Controller controller, MechanismControl control) {
      SparkBaseConfig tuning = blank(controller);
      tuning.smartCurrentLimit(statorAmps(control));
      tuning.closedLoop.p(positionKp, ClosedLoopSlot.kSlot0).p(velocityKp, ClosedLoopSlot.kSlot1);
      return tuning;
    }

    /** The soft limits switched on or off, alone: off while the joint homes past them. */
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
   * @param config its settings: following the leader, aligned or opposed as the wiring's directions
   *     say, with the leader's current limit and idle mode
   */
  public record Follower(int id, SparkBaseConfig config) {}

  /**
   * A SPARK's limit inputs, from what's on them ({@link ControllerPins}): a switch's polarity, and
   * whether the SPARK stops on it, only for a limit switch; an input with nothing on it stops
   * nothing. Neither resets its position as it trips: homing is the robot's, where it can be
   * checked.
   *
   * @throws IllegalArgumentException for a CANdi's limit, which a SPARK can't read
   */
  public static void limits(LimitSwitchConfig limits, ControllerPins pins) {
    limits
        .forwardLimitSwitchType(type(pins.forward()))
        .forwardLimitSwitchTriggerBehavior(behavior(pins.forward()))
        .reverseLimitSwitchType(type(pins.reverse()))
        .reverseLimitSwitchTriggerBehavior(behavior(pins.reverse()));
  }

  private static LimitSwitchConfig.Type type(Optional<ControllerPins.Limit> limit) {
    return switch (limit.orElse(null)) {
      case ControllerPins.OnPin pin ->
          pin.polarity() == SensorWiring.Polarity.NORMALLY_CLOSED
              ? LimitSwitchConfig.Type.kNormallyClosed
              : LimitSwitchConfig.Type.kNormallyOpen;
      case ControllerPins.OnCandi candi ->
          throw new IllegalArgumentException("a SPARK can't read a CANdi as its limit");
      case null -> LimitSwitchConfig.Type.kNormallyOpen;
    };
  }

  private static LimitSwitchConfig.Behavior behavior(Optional<ControllerPins.Limit> limit) {
    return limit.map(ControllerPins.Limit::stops).orElse(false)
        ? LimitSwitchConfig.Behavior.kStopMovingMotor
        : LimitSwitchConfig.Behavior.kKeepMovingMotor;
  }

  /**
   * A mechanism on SPARKs, read by its motor's own encoder.
   *
   * @param plan how the program drives its joint, its motors' SPARKs named, the leader first
   * @throws IllegalArgumentException for controllers that aren't SPARKs
   */
  public static Settings configure(ControllerPlan plan) {
    return build(plan, null, null, null);
  }

  /**
   * A mechanism on SPARKs with an absolute encoder on the leader's data port: the definition's
   * absolute encoder on the joint, read once at startup, so the joint needs no homing, and the
   * SPARK's loop running on it where the wiring says so.
   *
   * @param math what the robot's definition derives: the window the encoder tells apart
   * @param absolute the definition's absolute encoder the data port reads
   * @param port how it's read there: its offset, its direction, and whether the loop closes on it
   * @throws IllegalArgumentException as the other does, for an encoder that isn't absolute on the
   *     joint alone, or whose joint's range is more than one of its turns tells apart
   */
  public static Settings configure(
      RobotMath math,
      ControllerPlan plan,
      Sensor.Encoder absolute,
      MechanismWiring.DataPortEncoder port) {
    Joint joint = plan.joint();
    if (!absolute.absolute() || !absolute.joints().equals(List.of(joint))) {
      throw new IllegalArgumentException(
          absolute.label() + " must be an absolute encoder on " + joint.label() + " alone");
    }
    return build(plan, math, absolute, port);
  }

  private static Settings build(
      ControllerPlan plan,
      @Nullable RobotMath math,
      Sensor.@Nullable Encoder absolute,
      MechanismWiring.@Nullable DataPortEncoder port) {
    Joint joint = plan.joint();
    List<MechanismWiring.Device> controllers = plan.controllers();
    MechanismWiring.Device leading = controllers.get(0);
    MechanismWiring.Controller controller = leading.controller();
    if (controller != MechanismWiring.Controller.SPARK_MAX
        && controller != MechanismWiring.Controller.SPARK_FLEX) {
      throw new IllegalArgumentException(
          possessive(joint.label())
              + " wiring is "
              + possessive(controller.shown())
              + ", not a SPARK's");
    }
    List<String> notes = new ArrayList<>(plan.notes());
    MechanismRotations rotations = plan.rotations();
    double windowEdge = 0;
    double encoderRatio = 0;
    if (math != null && absolute != null) {
      // One turn of the encoder must tell the joint's range apart, and wrap outside it, as a
      // CANcoder's discontinuity point puts it.
      Range window = math.absoluteWindow(List.of(absolute));
      String wraps = ControllerPlan.wrapProblem(absolute, ControllerPlan.wrapPoint(math, absolute));
      if (wraps != null) {
        notes.add(wraps);
      }
      encoderRatio = rotations.ratioPerRotation(Math.abs(absolute.ratio(joint)));
      windowEdge = encoderRatio * rotations.toRotations(window.lower());
    }
    boolean closed = absolute != null && port != null && port.closedLoop();
    double ratio = rotations.ratioPerRotation(plan.rotorRatio());
    int amps = statorAmps(plan.control());

    SparkBaseConfig leader = blank(controller);
    leader
        .inverted(leading.turnsClockwise())
        .idleMode(plan.brakes() ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(amps)
        .voltageCompensation(Motor.NOMINAL_VOLTS);
    // The sensor its loop runs on, and its turns: the rotor's, or the absolute encoder's from the
    // window's edge.
    double loopRatio = closed ? encoderRatio : ratio;
    double loopZero = closed ? -windowEdge : 0;
    double kP = rotations.gainPerRotation(plan.kP());
    double positionKp = kP / (Motor.NOMINAL_VOLTS * loopRatio);
    double velocityKp = kP / (Motor.NOMINAL_VOLTS * loopRatio * 60);
    // Its integral accumulates each millisecond, its loop's period: per second over a thousand.
    double positionKi =
        rotations.gainPerRotation(plan.kI()) / (Motor.NOMINAL_VOLTS * loopRatio * 1000);
    leader
        .closedLoop
        .feedbackSensor(closed ? FeedbackSensor.kAbsoluteEncoder : FeedbackSensor.kPrimaryEncoder)
        .p(positionKp, ClosedLoopSlot.kSlot0)
        .p(velocityKp, ClosedLoopSlot.kSlot1);
    if (positionKi != 0) {
      leader.closedLoop.i(positionKi, ClosedLoopSlot.kSlot0);
    }
    if (closed) {
      // Its reading wraps outside the joint's range: the short way round could be through a stop.
      leader.closedLoop.positionWrappingEnabled(false);
    }
    Optional<double[]> softLimits = Optional.empty();
    if (plan.softLimits().isPresent()) {
      MechanismControl.SoftLimits limits = plan.softLimits().get();
      double lower = rotations.toRotations(limits.lower()) * loopRatio + loopZero;
      double upper = rotations.toRotations(limits.upper()) * loopRatio + loopZero;
      leader
          .softLimit
          .reverseSoftLimit(lower)
          .reverseSoftLimitEnabled(true)
          .forwardSoftLimit(upper)
          .forwardSoftLimitEnabled(true);
      softLimits = Optional.of(new double[] {lower, upper});
    }
    if (absolute != null && port != null) {
      if (controller == MechanismWiring.Controller.SPARK_MAX) {
        leader.absoluteEncoder.setSparkMaxDataPortConfig();
      }
      // The SPARK applies the encoder's direction and offset itself, as a CANcoder does: its
      // reading grows with the joint, and reads zero at the window's low edge, so it wraps where
      // the joint never is.
      leader.absoluteEncoder.inverted(port.reversed()).zeroOffset(port.raw(windowEdge));
      leader.signals.absoluteEncoderPositionPeriodMs(SIGNAL_MS);
    }
    leader
        .signals
        .primaryEncoderPositionPeriodMs(SIGNAL_MS)
        .primaryEncoderVelocityPeriodMs(SIGNAL_MS)
        .appliedOutputPeriodMs(SIGNAL_MS)
        .outputCurrentPeriodMs(SIGNAL_MS)
        .busVoltagePeriodMs(SIGNAL_MS);

    List<Follower> followers = new ArrayList<>();
    for (MechanismWiring.Device motor : controllers.subList(1, controllers.size())) {
      SparkBaseConfig follower = blank(controller);
      follower
          .idleMode(
              plan.brakes() ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
          .smartCurrentLimit(amps)
          .voltageCompensation(Motor.NOMINAL_VOLTS)
          .follow(leading.id(), plan.opposes(motor));
      followers.add(new Follower(motor.id(), follower));
    }

    return new Settings(
        plan,
        leader,
        followers,
        ratio,
        loopRatio,
        loopZero,
        positionKp,
        velocityKp,
        positionKi,
        softLimits,
        Optional.ofNullable(absolute),
        windowEdge,
        closed,
        notes);
  }

  /** An empty configuration for the controller: only what's set on it is applied. */
  private static SparkBaseConfig blank(MechanismWiring.Controller controller) {
    return controller == MechanismWiring.Controller.SPARK_FLEX
        ? new SparkFlexConfig()
        : new SparkMaxConfig();
  }

  /** The control's stator limit, as a SPARK's smart current limit takes it: whole amps. */
  private static int statorAmps(MechanismControl control) {
    return (int) Math.round(control.currentLimits().stator().in(Amps));
  }
}

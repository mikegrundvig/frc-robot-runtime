package com.michaelgrundvig.frc.runtime.lib.phoenix;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.runtime.lib.wiring.CanPorts;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * A mechanism's Phoenix 6 settings, translated from how the program drives its joint ({@link
 * ControllerPlan}): its TalonFX's ratios, gains, gravity, limits, direction and followers, in
 * Phoenix's units and terms, and its CANcoder's. The plan owns every value; this says them as
 * Phoenix holds them. Pure functions: the configurations are new objects for the caller to apply,
 * and no device is touched here.
 *
 * <p>Positions are in the joint's rotations ({@link MechanismRotations}): a revolute joint's
 * rotations, a prismatic joint's drum's rotations. Every ratio Phoenix is given is a magnitude, and
 * every direction is the wiring's, as the joint moves forward ({@link MechanismWiring}): each motor
 * controller's, the leader first, as the joint's block names them. The gains go in slot 0, which
 * runs the plan's feedforward law itself, but for its kA term, which the program adds with each
 * request.
 *
 * <p>What Phoenix can't hold is refused, by an {@link IllegalArgumentException} saying what and
 * why: a ratio, a gain or a gravity offset past its limits. What no motor controller could do the
 * plan refuses, and what the definition refuses of the joint's own block, soft limits past its
 * stops among them, never gets here. What's doubtful is noted, beside the settings ({@link
 * Settings#notes}): the plan's doubts, then Phoenix's own, an encoder wrapping close to the joint's
 * range and a gravity offset at Phoenix's limit.
 */
public final class PhoenixMechanism {
  private PhoenixMechanism() {}

  // Phoenix's own limits, in 26.70.0-alpha-2's configuration classes.

  /**
   * The largest ratio Phoenix holds: {@code FeedbackConfigs.SensorToMechanismRatio} and {@code
   * RotorToSensorRatio}, and {@code DifferentialSensorsConfigs.SensorToDifferentialRatio}, each
   * from -1000 to 1000.
   */
  static final double MAX_RATIO = 1000;

  /**
   * The farthest {@code Slot0Configs.GravityArmPositionOffset} goes, in rotations, either way: its
   * documented minimum and maximum, -0.25 and 0.25.
   */
  static final double MAX_GRAVITY_OFFSET = 0.25;

  /** kS and kG, in volts: from -128 to 127. */
  static final double MIN_VOLTS = -128;

  static final double MAX_VOLTS = 127;

  // The helper's own judgment: chosen, each with its reason.

  /**
   * How near Phoenix's ±0.25-rotation limit a gravity offset may come before it's noted: 5°. A
   * center of mass that close to straight above or below the pivot at the joint's zero crosses it
   * with a CAD change (wiring, a heavier roller), and Phoenix can't take the offset then.
   */
  static final double GRAVITY_OFFSET_MARGIN = 5.0 / 360;

  /**
   * One mechanism's settings.
   *
   * @param plan how the program drives its joint, which they say in Phoenix's terms: its units are
   *     what its code converts the joint's positions with, to and from Phoenix's
   * @param leader the leading TalonFX's: everything Phoenix needs to drive the joint
   * @param followers each following TalonFX's, in the wiring's order
   * @param cancoder the CANcoder's, if the mechanism has one
   * @param notes what's doubtful about them, each a sentence for a log or a test
   */
  public record Settings(
      ControllerPlan plan,
      TalonFXConfiguration leader,
      List<FollowerSettings> followers,
      Optional<CANcoderConfiguration> cancoder,
      List<String> notes) {
    public Settings {
      followers = List.copyOf(followers);
      notes = List.copyOf(notes);
    }

    /** The joint's rotations, which Phoenix's positions, speeds and gains are in. */
    public MechanismRotations rotations() {
      return plan.rotations();
    }
  }

  /**
   * A following TalonFX: its request and its own configuration, which keeps its current limits,
   * neutral mode and position reading in step with the leader's.
   *
   * @param id its CAN ID
   * @param request what to send it: follow the leader, aligned or opposed as the wiring's
   *     directions say
   * @param configuration its own settings
   */
  public record FollowerSettings(int id, Follower request, TalonFXConfiguration configuration) {}

  /**
   * A mechanism read by its motors' rotors alone.
   *
   * @param plan how the program drives its joint, its motors' controllers named, the leader first
   * @throws IllegalArgumentException for what Phoenix can't hold
   */
  public static Settings configure(ControllerPlan plan) {
    return build(plan, null, null, null);
  }

  /**
   * A mechanism with a CANcoder: the definition's absolute encoder on the joint, wired as its
   * block's CANcoder says, and read as its feedback says.
   *
   * @param math what the robot's definition derives: where the encoder's reading wraps
   * @param encoder the definition's absolute encoder the CANcoder is
   * @param cancoder where it's wired, and how the TalonFX reads it
   * @throws IllegalArgumentException as the other does, and for an encoder that isn't absolute on
   *     the joint alone
   */
  public static Settings configure(
      RobotMath math,
      ControllerPlan plan,
      Sensor.Encoder encoder,
      MechanismWiring.Cancoder cancoder) {
    return build(plan, math, encoder, cancoder);
  }

  /** The TalonFXS's motor arrangement for each catalog motor it drives, by its JST connector. */
  private static final Map<Motor, MotorArrangementValue> ARRANGEMENTS =
      Map.of(
          Motor.MINION, MotorArrangementValue.Minion_JST,
          Motor.NEO, MotorArrangementValue.NEO_JST,
          Motor.NEO_550, MotorArrangementValue.NEO550_JST,
          Motor.NEO_VORTEX, MotorArrangementValue.VORTEX_JST);

  /**
   * A TalonFXS's settings, from the ones worked out for a TalonFX on the same mechanism: the same
   * slot, limits, profile, direction and neutral mode; its feedback in a TalonFXS's own terms (its
   * motor's hall sensor for a TalonFX's rotor); and the motor plugged into it, which a TalonFXS
   * needs to be told.
   *
   * @throws IllegalArgumentException for a motor a TalonFXS doesn't drive
   */
  public static TalonFXSConfiguration talonFxs(TalonFXConfiguration fx, Motor motor) {
    MotorArrangementValue arrangement = ARRANGEMENTS.get(motor);
    if (arrangement == null) {
      throw new IllegalArgumentException("a TalonFXS doesn't drive a " + motor.name());
    }
    TalonFXSConfiguration fxs = new TalonFXSConfiguration();
    fxs.MotorOutput = fx.MotorOutput;
    fxs.CurrentLimits = fx.CurrentLimits;
    fxs.SoftwareLimitSwitch = fx.SoftwareLimitSwitch;
    fxs.HardwareLimitSwitch = fx.HardwareLimitSwitch;
    fxs.ClosedLoopGeneral = fx.ClosedLoopGeneral;
    fxs.Slot0 = fx.Slot0;
    fxs.Commutation.MotorArrangement = arrangement;
    fxs.ExternalFeedback.SensorToMechanismRatio = fx.Feedback.SensorToMechanismRatio;
    fxs.ExternalFeedback.RotorToSensorRatio = fx.Feedback.RotorToSensorRatio;
    fxs.ExternalFeedback.FeedbackRemoteSensorID = fx.Feedback.FeedbackRemoteSensorID;
    fxs.ExternalFeedback.ExternalFeedbackSensorSource =
        switch (fx.Feedback.FeedbackSensorSource) {
          case RemoteCANcoder -> ExternalFeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> ExternalFeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> ExternalFeedbackSensorSourceValue.SyncCANcoder;
          default -> ExternalFeedbackSensorSourceValue.Commutation;
        };
    return fxs;
  }

  /**
   * A CANcoder's settings: its offset and direction from the wiring, and its discontinuity point
   * from the definition ({@link ControllerPlan#wrapPoint}), where its reading wraps as far from the
   * joint's range as a turn allows.
   *
   * @throws IllegalArgumentException for an encoder that isn't absolute, or whose joint's range is
   *     more of its turn than one turn holds: a pair decoded together is read in code ({@code
   *     RobotMath.absoluteWindow})
   */
  public static CANcoderConfiguration cancoder(
      RobotMath math, Sensor.Encoder encoder, MechanismWiring.Cancoder wiring) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = wiring.offset().in(Rotations);
    config.MagnetSensor.SensorDirection =
        wiring.turnsClockwise()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ControllerPlan.wrapPoint(math, encoder);
    return config;
  }

  /**
   * A TalonFX's or TalonFXS's limits, from what's on them ({@link ControllerPins}): a switch on its
   * pin, its polarity told; a limit switch on a CANdi on its bus, read over CAN, the CANdi's closed
   * already meaning tripped; and nothing, which stops nothing. It stops only on a limit switch.
   * Neither resets its position as it trips: homing is the robot's, where it can be checked.
   */
  public static void limits(HardwareLimitSwitchConfigs limits, ControllerPins pins) {
    limits.ForwardLimitAutosetPositionEnable = false;
    limits.ReverseLimitAutosetPositionEnable = false;
    limits.ForwardLimitEnable = pins.forward().map(ControllerPins.Limit::stops).orElse(false);
    limits.ReverseLimitEnable = pins.reverse().map(ControllerPins.Limit::stops).orElse(false);
    limits.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    limits.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    limits.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    limits.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    switch (pins.forward().orElse(null)) {
      case ControllerPins.OnPin pin ->
          limits.ForwardLimitType =
              pin.polarity() == SensorWiring.Polarity.NORMALLY_OPEN
                  ? ForwardLimitTypeValue.NormallyOpen
                  : ForwardLimitTypeValue.NormallyClosed;
      case ControllerPins.OnCandi candi -> {
        limits.ForwardLimitSource =
            candi.input() == 1
                ? ForwardLimitSourceValue.RemoteCANdiS1
                : ForwardLimitSourceValue.RemoteCANdiS2;
        limits.ForwardLimitRemoteSensorID = candi.id();
      }
      case null -> {}
    }
    switch (pins.reverse().orElse(null)) {
      case ControllerPins.OnPin pin ->
          limits.ReverseLimitType =
              pin.polarity() == SensorWiring.Polarity.NORMALLY_OPEN
                  ? ReverseLimitTypeValue.NormallyOpen
                  : ReverseLimitTypeValue.NormallyClosed;
      case ControllerPins.OnCandi candi -> {
        limits.ReverseLimitSource =
            candi.input() == 1
                ? ReverseLimitSourceValue.RemoteCANdiS1
                : ReverseLimitSourceValue.RemoteCANdiS2;
        limits.ReverseLimitRemoteSensorID = candi.id();
      }
      case null -> {}
    }
  }

  /** Phoenix's bus for the wiring's. */
  public static CANBus bus(CanBus bus) {
    return switch (bus) {
      case CanBus.Port port -> new CANBus(CanPorts.of(port.port()));
      case CanBus.Canivore canivore -> new CANBus(canivore.name());
    };
  }

  private static Settings build(
      ControllerPlan plan,
      @Nullable RobotMath math,
      Sensor.@Nullable Encoder encoder,
      MechanismWiring.@Nullable Cancoder wired) {
    Joint joint = plan.joint();
    List<MechanismWiring.Device> controllers = plan.controllers();
    List<String> notes = new ArrayList<>(plan.notes());
    MechanismRotations rotations = plan.rotations();
    double rotorRatio = plan.rotorRatio();
    TalonFXConfiguration leader = new TalonFXConfiguration();
    MechanismWiring.Device leading = controllers.get(0);
    output(leader, leading, plan);
    Optional<CANcoderConfiguration> cancoder = Optional.empty();
    String ratioName = possessive(joint.label()) + " SensorToMechanismRatio";
    if (math == null || encoder == null || wired == null) {
      leader.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      leader.Feedback.SensorToMechanismRatio =
          ratio(ratioName, rotations.ratioPerRotation(rotorRatio));
    } else {
      if (!encoder.absolute() || !encoder.joints().equals(List.of(joint))) {
        throw new IllegalArgumentException(
            encoder.label()
                + " must be an absolute encoder on "
                + joint.label()
                + " alone, as a CANcoder is");
      }
      CANcoderConfiguration config = cancoder(math, encoder, wired);
      String wraps =
          ControllerPlan.wrapProblem(encoder, config.MagnetSensor.AbsoluteSensorDiscontinuityPoint);
      if (wraps != null) {
        notes.add(wraps);
      }
      cancoder = Optional.of(config);
      double sensor = Math.abs(encoder.ratio(joint));
      switch (wired.feedback()) {
        case ROTOR -> {
          leader.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
          leader.Feedback.SensorToMechanismRatio =
              ratio(ratioName, rotations.ratioPerRotation(rotorRatio));
        }
        case REMOTE, FUSED, SYNC -> {
          leader.Feedback.FeedbackSensorSource = source(wired.feedback());
          leader.Feedback.FeedbackRemoteSensorID = wired.id();
          leader.Feedback.SensorToMechanismRatio =
              ratio(ratioName, rotations.ratioPerRotation(sensor));
          leader.Feedback.RotorToSensorRatio =
              ratio(possessive(joint.label()) + " RotorToSensorRatio", rotorRatio / sensor);
        }
      }
    }
    leader.Slot0 = Slot0Configs.from(slot(plan, notes));
    // A joint that turns freely takes the short way round to a position, a turn being a turn.
    leader.ClosedLoopGeneral.ContinuousWrap = plan.turnsFreely();
    plan.softLimits()
        .ifPresent(
            limits -> {
              leader.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
              leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                  rotations.toRotations(limits.upper());
              leader.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
              leader.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                  rotations.toRotations(limits.lower());
            });

    List<FollowerSettings> followers = new ArrayList<>();
    for (MechanismWiring.Device motor : controllers.subList(1, controllers.size())) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      output(config, motor, plan);
      config.Feedback.SensorToMechanismRatio =
          ratio(ratioName, rotations.ratioPerRotation(rotorRatio));
      followers.add(
          new FollowerSettings(
              motor.id(),
              new Follower(
                  leading.id(),
                  plan.opposes(motor) ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned),
              config));
    }
    return new Settings(plan, leader, followers, cancoder, notes);
  }

  /** Phoenix's feedback source for how the wiring reads the CANcoder. */
  private static FeedbackSensorSourceValue source(MechanismWiring.Feedback feedback) {
    return switch (feedback) {
      case ROTOR -> FeedbackSensorSourceValue.RotorSensor;
      case REMOTE -> FeedbackSensorSourceValue.RemoteCANcoder;
      case FUSED -> FeedbackSensorSourceValue.FusedCANcoder;
      case SYNC -> FeedbackSensorSourceValue.SyncCANcoder;
    };
  }

  /** A motor's direction, neutral mode and current limits. */
  private static void output(
      TalonFXConfiguration config, MechanismWiring.Device motor, ControllerPlan plan) {
    config.MotorOutput.Inverted =
        motor.turnsClockwise()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode =
        plan.brakes() ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    MechanismControl.CurrentLimits limits = plan.currentLimits();
    CurrentLimitsConfigs current = config.CurrentLimits;
    current.StatorCurrentLimitEnable = true;
    current.StatorCurrentLimit = limits.stator().in(Amps);
    current.SupplyCurrentLimitEnable = true;
    current.SupplyCurrentLimit = limits.supply().in(Amps);
    current.SupplyCurrentLowerLimit = limits.supplyLower().in(Amps);
    current.SupplyCurrentLowerTime = limits.supplyLowerTime().in(Seconds);
  }

  /**
   * A slot's gains in Phoenix's units, per rotation: the plan's, tuned where given and seeded where
   * not, and gravity's type, offset and kG.
   */
  private static SlotConfigs slot(ControllerPlan plan, List<String> notes) {
    Joint joint = plan.joint();
    MechanismRotations rotations = plan.rotations();
    SlotConfigs slot = new SlotConfigs();
    slot.kP = rotations.gainPerRotation(plan.kP());
    slot.kI = rotations.gainPerRotation(plan.kI());
    slot.kD = rotations.gainPerRotation(plan.kD());
    slot.kS = volts(joint, "kS", plan.kS());
    slot.kV = rotations.gainPerRotation(plan.kV());
    slot.kA = rotations.gainPerRotation(plan.kA());
    ControllerPlan.Gravity gravity = plan.gravity();
    switch (gravity.kind()) {
      case NONE -> {}
      case STEADY -> {
        slot.GravityType = GravityTypeValue.Elevator_Static;
        slot.kG = volts(joint, "kG", gravity.kG());
      }
      case COSINE -> {
        // Phoenix's onboard gravity takes an offset within ±0.25 rotations.
        slot.GravityType = GravityTypeValue.Arm_Cosine;
        slot.GravityArmPositionOffset =
            plan.control().gains().gravityAngle().isPresent()
                ? givenOffset(joint, -gravity.offset())
                : gravityOffset(joint, gravity.offset() / (2 * Math.PI), notes);
        slot.kG = volts(joint, "kG", gravity.kG());
      }
    }
    return slot;
  }

  /**
   * Phoenix's {@code GravityArmPositionOffset} for an arm's gravity angle as its control gives it,
   * within half a turn of its zero, in radians: refused more than a quarter turn from its zero,
   * where Phoenix can't take it.
   */
  private static double givenOffset(Joint joint, double angle) {
    double offset = -angle / (2 * Math.PI);
    if (Math.abs(offset) > MAX_GRAVITY_OFFSET + 1e-9) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s's gravity angle, %s, is more than a quarter turn from its zero, past Phoenix's"
                  + " gravity offset of ±0.25 rotations: its positive direction lowers it at its"
                  + " zero. Turn its axis around in the definition (and its motors' and encoder's"
                  + " directions in the wiring), so that positive raises it; its gravity angle is"
                  + " then %s",
              joint.label(),
              joint.shown(angle),
              joint.shown(Math.IEEEremainder(Math.PI - angle, 2 * Math.PI))));
    }
    return Math.max(-MAX_GRAVITY_OFFSET, Math.min(MAX_GRAVITY_OFFSET, offset));
  }

  /**
   * Phoenix's {@code GravityArmPositionOffset} for an offset in rotations: refused past ±0.25,
   * noted near it.
   */
  private static double gravityOffset(Joint joint, double offset, List<String> notes) {
    if (Math.abs(offset) > MAX_GRAVITY_OFFSET + 1e-9) {
      double turned = offset > 0 ? 0.5 - offset : -0.5 - offset;
      turned = Math.abs(turned) < 1e-9 ? 0 : turned;
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s's gravity offset is %s rotations, past Phoenix's ±0.25: its positive direction"
                  + " lowers it. Turn its axis around in the definition (and its motors' and"
                  + " encoder's directions in the wiring), so that positive raises it; the offset"
                  + " is then %s",
              joint.label(),
              shown(offset),
              shown(turned)));
    }
    if (Math.abs(offset) > MAX_GRAVITY_OFFSET - GRAVITY_OFFSET_MARGIN) {
      notes.add(
          String.format(
              Locale.ROOT,
              "%s's gravity offset is %s rotations, at Phoenix's limit of ±0.25: its center of mass"
                  + " is %s from straight above or below the pivot at its zero, and a CAD change"
                  + " that moves it past makes the offset one Phoenix can't take. Zero %s nearer"
                  + " horizontal in CAD",
              joint.label(),
              shown(offset),
              joint.shown((MAX_GRAVITY_OFFSET - Math.abs(offset)) * 2 * Math.PI),
              joint.label()));
    }
    return Math.max(-MAX_GRAVITY_OFFSET, Math.min(MAX_GRAVITY_OFFSET, offset));
  }

  /** A ratio Phoenix holds, or a refusal naming it. */
  private static double ratio(String name, double ratio) {
    if (!(Math.abs(ratio) <= MAX_RATIO) || ratio == 0) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s would be %s, and Phoenix holds from -1000 to 1000, never 0",
              name,
              shown(ratio)));
    }
    return ratio;
  }

  /** Volts Phoenix holds as kS or kG, or a refusal. */
  private static double volts(Joint joint, String gain, double volts) {
    if (volts < MIN_VOLTS || volts > MAX_VOLTS) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s's %s, %s V, is past what Phoenix holds, -128 to 127 V",
              joint.label(),
              gain,
              shown(volts)));
    }
    return volts;
  }
}

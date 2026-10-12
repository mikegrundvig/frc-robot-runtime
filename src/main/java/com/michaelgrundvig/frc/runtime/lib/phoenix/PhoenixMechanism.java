package com.michaelgrundvig.frc.runtime.lib.phoenix;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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

/**
 * A mechanism's Phoenix 6 settings, translated from how the program drives its joint ({@link
 * ControllerPlan}): the leading TalonFX's ratios, gains, gravity, limits and direction, each
 * follower's settings, and the CANcoder's. The plan owns every value. This class only says them the
 * way Phoenix holds them, in Phoenix's units and names. Every function here is pure: it returns new
 * configuration objects for the caller to apply, and touches no device.
 *
 * <p><b>Units.</b> Phoenix's positions are in the joint's rotations ({@link MechanismRotations}): a
 * turning joint's own rotations, or for a sliding joint, its drum's. Every ratio given to Phoenix
 * is a magnitude. Every direction comes from the wiring ({@link MechanismWiring}), which says which
 * way each motor turns as the joint moves forward.
 *
 * <p><b>Gains.</b> The gains go in two slots: slot 0 for positions ({@link #POSITION_SLOT}), and
 * slot 1 for speeds ({@link #SPEED_SLOT}), which is the same slot without its kI. A Phoenix slot's
 * kI adds up whatever error its request works on, so a speed asked of slot 0 would add up the
 * speed's error, and kI is the position loop's alone ({@code JointIO}'s J7). The TalonFX runs the
 * plan's feedforward law itself (kS, kV and gravity) from whichever slot a request names. The one
 * exception is the kA term, which the program works out from its motion profile and adds to each
 * request.
 *
 * <p><b>What is refused.</b> A value Phoenix cannot hold is refused with an {@link
 * IllegalArgumentException} that says what and why: a ratio, a gain or a gravity offset past
 * Phoenix's limits. What no motor controller could do, the plan has already refused, and what the
 * robot's definition refuses about the joint's own block (soft limits past its stops, for one)
 * never gets here.
 *
 * <p><b>What is doubted.</b> A value that is allowed but risky is noted beside the settings ({@link
 * Settings#notes}): first the plan's own doubts, then Phoenix's, which are an encoder whose reading
 * wraps close to the joint's range and a gravity offset close to Phoenix's limit.
 */
public final class PhoenixMechanism {
  private PhoenixMechanism() {}

  /** The slot a position request runs on: the plan's gains, its kI among them. */
  public static final int POSITION_SLOT = 0;

  /** The slot a speed request runs on: the plan's gains without its kI. */
  public static final int SPEED_SLOT = 1;

  // Phoenix's own limits, as its configuration classes document them in 26.70.0-alpha-2.

  /**
   * The largest ratio Phoenix holds, either way. It applies to {@code
   * FeedbackConfigs.SensorToMechanismRatio} and {@code RotorToSensorRatio}, and to {@code
   * DifferentialSensorsConfigs.SensorToDifferentialRatio}: each runs from -1000 to 1000.
   */
  static final double MAX_RATIO = 1000;

  /**
   * The farthest {@code Slot0Configs.GravityArmPositionOffset} goes, in rotations, either way. Its
   * documented range is -0.25 to 0.25: a quarter turn.
   */
  static final double MAX_GRAVITY_OFFSET = 0.25;

  /** The least and most Phoenix holds as kS or kG, in volts: -128 to 127. */
  static final double MIN_VOLTS = -128;

  static final double MAX_VOLTS = 127;

  // This helper's own judgment, each value with its reason.

  /**
   * How near Phoenix's quarter-turn limit a gravity offset may come before it is noted, in
   * rotations: 5°. An arm whose center of mass is that close to straight above or below the pivot,
   * with the joint at zero, can cross the limit after an ordinary CAD change (wiring, a heavier
   * roller), and then Phoenix cannot take the offset at all.
   */
  static final double GRAVITY_OFFSET_MARGIN = 5.0 / 360;

  /** How far past a limit a value may be by rounding alone before it is refused. */
  private static final double ROUNDING = 1e-9;

  /**
   * One mechanism's settings.
   *
   * @param plan how the program drives its joint, which these settings say in Phoenix's terms. Its
   *     rotations are what the joint's IO converts positions with, to and from Phoenix's.
   * @param leader the leading TalonFX's settings: everything Phoenix needs to drive the joint
   * @param followers each following TalonFX's settings, in the wiring's order
   * @param cancoder the CANcoder's settings, if the mechanism has one
   * @param notes what is doubtful about the settings, each a sentence for a log or a test
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
   * A following TalonFX. Following copies the leader's output and nothing else, so a follower still
   * needs its own configuration to have the leader's current limits and neutral mode.
   *
   * @param id its CAN ID
   * @param request what to send it: follow the leader, turning the same way or the opposite way as
   *     the wiring's directions say
   * @param configuration its own settings
   */
  public record FollowerSettings(int id, Follower request, TalonFXConfiguration configuration) {}

  /**
   * The CANcoder a mechanism has, with what is needed to work out its settings.
   *
   * @param math what the robot's definition derives: where the encoder's reading should wrap
   * @param encoder the definition's absolute encoder that the CANcoder is
   * @param wiring where the CANcoder is wired, and how the TalonFX reads it
   */
  private record WiredCancoder(
      RobotMath math, Sensor.Encoder encoder, MechanismWiring.Cancoder wiring) {}

  /**
   * The settings for a mechanism read by its motors' rotors alone.
   *
   * @param plan how the program drives its joint, naming its motors' controllers, the leader first
   * @throws IllegalArgumentException for what Phoenix can't hold
   */
  public static Settings configure(ControllerPlan plan) {
    return build(plan, Optional.empty());
  }

  /**
   * The settings for a mechanism with a CANcoder: the definition's absolute encoder on the joint,
   * wired as the joint's block says and read the way its feedback says.
   *
   * @param math what the robot's definition derives: where the encoder's reading wraps
   * @param encoder the definition's absolute encoder that the CANcoder is
   * @param cancoder where it is wired, and how the TalonFX reads it
   * @throws IllegalArgumentException as {@link #configure(ControllerPlan)} does, and for an encoder
   *     that isn't an absolute encoder on this joint alone
   */
  public static Settings configure(
      RobotMath math,
      ControllerPlan plan,
      Sensor.Encoder encoder,
      MechanismWiring.Cancoder cancoder) {
    return build(plan, Optional.of(new WiredCancoder(math, encoder, cancoder)));
  }

  /**
   * Which motor a TalonFXS is told it drives, for each catalog motor it can drive. Each is plugged
   * into the TalonFXS's JST connector.
   */
  private static final Map<Motor, MotorArrangementValue> ARRANGEMENTS =
      Map.of(
          Motor.MINION, MotorArrangementValue.Minion_JST,
          Motor.NEO, MotorArrangementValue.NEO_JST,
          Motor.NEO_550, MotorArrangementValue.NEO550_JST,
          Motor.NEO_VORTEX, MotorArrangementValue.VORTEX_JST);

  /**
   * A TalonFXS's settings, from the ones worked out for a TalonFX on the same mechanism. The gains,
   * limits, direction and neutral mode carry over as they are. Two things differ. A TalonFXS has to
   * be told which motor is plugged into it. And its feedback settings have other names: what a
   * TalonFX calls its rotor, a TalonFXS reads from its motor's own commutation sensor.
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
    fxs.Slot1 = fx.Slot1;
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
   * A CANcoder's settings: its offset and direction from the wiring, and where its reading wraps
   * from the definition. An absolute encoder reads a share of one turn, so somewhere in the turn
   * its reading jumps from its highest value back to its lowest. That point is put as far from the
   * joint's range as one turn allows ({@link ControllerPlan#wrapPoint}).
   *
   * @throws IllegalArgumentException for an encoder that isn't absolute, or whose joint's range
   *     covers more than one turn of it. (A pair of encoders decoded together is read in code:
   *     {@code RobotMath.absoluteWindow}.)
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
   * Sets a TalonFX's or TalonFXS's two limit inputs from what is wired to them ({@link
   * ControllerPins}). Each input is one of three things:
   *
   * <ul>
   *   <li>Nothing. The controller never stops on it.
   *   <li>A switch on the controller's own limit pin. The controller is told its polarity, and
   *       stops on it only if it is a limit switch (a beam break wired there is read, not stopped
   *       on).
   *   <li>A limit switch on a CANdi on the same bus, which the controller reads over CAN and always
   *       stops on. The CANdi is told which state means tripped, so its "closed" already does.
   * </ul>
   *
   * <p>Neither input resets the controller's position when it trips. Working out where the joint is
   * belongs to the robot's homing, where it can be checked.
   */
  public static void limits(HardwareLimitSwitchConfigs limits, ControllerPins pins) {
    limits.ForwardLimitAutosetPositionEnable = false;
    limits.ReverseLimitAutosetPositionEnable = false;
    limits.ForwardLimitEnable = pins.forward().map(ControllerPins.Limit::stops).orElse(false);
    limits.ReverseLimitEnable = pins.reverse().map(ControllerPins.Limit::stops).orElse(false);
    // What an input with nothing on it is left as: its own pin, normally open.
    limits.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    limits.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    limits.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    limits.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    pins.forward().ifPresent(limit -> setForwardLimit(limits, limit));
    pins.reverse().ifPresent(limit -> setReverseLimit(limits, limit));
  }

  private static void setForwardLimit(HardwareLimitSwitchConfigs limits, ControllerPins.Limit on) {
    switch (on) {
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
    }
  }

  /** The same as {@link #setForwardLimit}, for the reverse input: Phoenix names each separately. */
  private static void setReverseLimit(HardwareLimitSwitchConfigs limits, ControllerPins.Limit on) {
    switch (on) {
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
    }
  }

  /** Phoenix's name for a CAN bus the wiring names: one of Systemcore's ports, or a CANivore. */
  public static CANBus bus(CanBus bus) {
    return switch (bus) {
      case CanBus.Port port -> new CANBus(CanPorts.of(port.port()));
      case CanBus.Canivore canivore -> new CANBus(canivore.name());
    };
  }

  /** Works out every device's settings for one mechanism, with or without a CANcoder. */
  private static Settings build(ControllerPlan plan, Optional<WiredCancoder> wired) {
    List<String> notes = new ArrayList<>(plan.notes());
    MechanismWiring.Device leading = plan.controllers().get(0);

    TalonFXConfiguration leader = new TalonFXConfiguration();
    setOutput(leader, leading, plan);
    Optional<CANcoderConfiguration> cancoder =
        wired.map(each -> cancoderFor(plan.joint(), each, notes));
    setFeedback(leader.Feedback, plan, wired);
    SlotConfigs gains = slot(plan, notes);
    leader.Slot0 = Slot0Configs.from(gains);
    leader.Slot1 = speedSlot(gains);
    // The loop never wraps, on a joint that turns freely either: a position is taken as its
    // number says. Which way round such a joint goes is its profile's choice (Travel.goalNear),
    // made before a position gets here, so that every controller turns the same way.
    leader.ClosedLoopGeneral.ContinuousWrap = false;
    plan.softLimits().ifPresent(limits -> setSoftLimits(leader.SoftwareLimitSwitch, plan, limits));

    return new Settings(plan, leader, followers(plan, leading), cancoder, notes);
  }

  /**
   * The CANcoder's settings for a mechanism, after checking that the definition's encoder is one a
   * CANcoder can be. A reading that wraps too near the joint's range is noted.
   *
   * @throws IllegalArgumentException for an encoder that isn't absolute, or isn't on this joint
   *     alone
   */
  private static CANcoderConfiguration cancoderFor(
      Joint joint, WiredCancoder wired, List<String> notes) {
    Sensor.Encoder encoder = wired.encoder();
    boolean onThisJointAlone = encoder.joints().equals(List.of(joint));
    if (!encoder.absolute() || !onThisJointAlone) {
      throw new IllegalArgumentException(
          encoder.label()
              + " must be an absolute encoder on "
              + joint.label()
              + " alone, as a CANcoder is");
    }
    CANcoderConfiguration config = cancoder(wired.math(), encoder, wired.wiring());
    double wrapPoint = config.MagnetSensor.AbsoluteSensorDiscontinuityPoint;
    Optional.ofNullable(ControllerPlan.wrapProblem(encoder, wrapPoint)).ifPresent(notes::add);
    return config;
  }

  /**
   * Sets which sensor the leader's position comes from, and the ratios that turn that sensor's
   * rotations into the joint's.
   *
   * <p>A leader that reads its own rotor has one ratio: rotor turns for each rotation of the joint
   * ({@code SensorToMechanismRatio}). A leader that reads the CANcoder has two: CANcoder turns for
   * each rotation of the joint ({@code SensorToMechanismRatio}), and rotor turns for each turn of
   * the CANcoder ({@code RotorToSensorRatio}), which Phoenix uses to relate the two sensors.
   *
   * @throws IllegalArgumentException for a ratio Phoenix can't hold
   */
  private static void setFeedback(
      FeedbackConfigs feedback, ControllerPlan plan, Optional<WiredCancoder> wired) {
    Joint joint = plan.joint();
    MechanismRotations rotations = plan.rotations();
    double rotorRatio = plan.rotorRatio();
    String sensorToMechanism = possessive(joint.label()) + " SensorToMechanismRatio";
    Optional<WiredCancoder> readByTheLeader =
        wired.filter(each -> each.wiring().feedback() != MechanismWiring.Feedback.ROTOR);
    if (readByTheLeader.isEmpty()) {
      feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      feedback.SensorToMechanismRatio =
          ratio(sensorToMechanism, rotations.ratioPerRotation(rotorRatio));
      return;
    }
    MechanismWiring.Cancoder cancoder = readByTheLeader.get().wiring();
    double cancoderRatio = Math.abs(readByTheLeader.get().encoder().ratio(joint));
    feedback.FeedbackSensorSource = source(cancoder.feedback());
    feedback.FeedbackRemoteSensorID = cancoder.id();
    feedback.SensorToMechanismRatio =
        ratio(sensorToMechanism, rotations.ratioPerRotation(cancoderRatio));
    feedback.RotorToSensorRatio =
        ratio(possessive(joint.label()) + " RotorToSensorRatio", rotorRatio / cancoderRatio);
  }

  /** Switches the leader's soft limits on, at the plan's limits in Phoenix's rotations. */
  private static void setSoftLimits(
      SoftwareLimitSwitchConfigs softLimits,
      ControllerPlan plan,
      MechanismControl.SoftLimits limits) {
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = plan.rotations().toRotations(limits.upper());
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = plan.rotations().toRotations(limits.lower());
  }

  /**
   * Each follower's settings, in the wiring's order: its own direction, neutral mode and current
   * limits, the rotor's ratio so its position reads in step with the leader's, and a request to
   * follow the leader, turning the opposite way where the wiring's directions differ.
   */
  private static List<FollowerSettings> followers(
      ControllerPlan plan, MechanismWiring.Device leading) {
    List<MechanismWiring.Device> controllers = plan.controllers();
    String sensorToMechanism = possessive(plan.joint().label()) + " SensorToMechanismRatio";
    List<FollowerSettings> followers = new ArrayList<>();
    for (MechanismWiring.Device motor : controllers.subList(1, controllers.size())) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      setOutput(config, motor, plan);
      config.Feedback.SensorToMechanismRatio =
          ratio(sensorToMechanism, plan.rotations().ratioPerRotation(plan.rotorRatio()));
      MotorAlignmentValue alignment =
          plan.opposes(motor) ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned;
      followers.add(
          new FollowerSettings(motor.id(), new Follower(leading.id(), alignment), config));
    }
    return followers;
  }

  /** Phoenix's feedback source for how the wiring has the TalonFX read the CANcoder. */
  private static FeedbackSensorSourceValue source(MechanismWiring.Feedback feedback) {
    return switch (feedback) {
      case ROTOR -> FeedbackSensorSourceValue.RotorSensor;
      case REMOTE -> FeedbackSensorSourceValue.RemoteCANcoder;
      case FUSED -> FeedbackSensorSourceValue.FusedCANcoder;
      case SYNC -> FeedbackSensorSourceValue.SyncCANcoder;
    };
  }

  /**
   * Sets one motor controller's direction, neutral mode and current limits.
   *
   * <p>Both current limits are switched on and given the plan's values. The stator limit caps the
   * current through the motor, and so its torque. The supply limit caps what the controller draws
   * from the battery, and steps down to a lower limit once it has been held for a time, which is
   * what keeps a breaker from tripping. Phoenix has defaults for all of these, so all are set here
   * rather than left to change unseen with a Phoenix update.
   */
  private static void setOutput(
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
   * The plan's gains as one Phoenix slot holds them: per rotation where the plan's are per radian
   * or per meter, with gravity's kind, offset and kG.
   *
   * @throws IllegalArgumentException for a kS, a kG or a gravity offset Phoenix can't hold
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
        // The same pull wherever the joint is, as on an elevator.
        slot.GravityType = GravityTypeValue.Elevator_Static;
        slot.kG = volts(joint, "kG", gravity.kG());
      }
      case COSINE -> {
        // A pull that varies as the cosine of an arm's angle from level. Phoenix needs to know
        // where level is, as an offset from the joint's zero, and holds only a quarter turn.
        slot.GravityType = GravityTypeValue.Arm_Cosine;
        slot.GravityArmPositionOffset = armOffset(plan, gravity, notes);
        slot.kG = volts(joint, "kG", gravity.kG());
      }
    }
    return slot;
  }

  /**
   * The slot speed requests run on: the position slot's gains and feedforward with no kI. Phoenix
   * adds up the error of whatever a slot's request asks for, and a speed's error is not something
   * the plan's kI was tuned for.
   */
  private static Slot1Configs speedSlot(SlotConfigs positions) {
    Slot1Configs speeds = Slot1Configs.from(positions);
    speeds.kI = 0;
    return speeds;
  }

  /**
   * Phoenix's {@code GravityArmPositionOffset} for an arm, in rotations. The plan's offset is in
   * radians. It comes either from an angle the joint's control gives outright, or from where the
   * definition puts the arm's center of mass; each is checked with its own message.
   *
   * @throws IllegalArgumentException for an offset past Phoenix's quarter turn
   */
  private static double armOffset(
      ControllerPlan plan, ControllerPlan.Gravity gravity, List<String> notes) {
    boolean angleIsGiven = plan.control().gains().gravityAngle().isPresent();
    if (angleIsGiven) {
      return givenOffset(plan.joint(), -gravity.offset());
    }
    return gravityOffset(plan.joint(), gravity.offset() / (2 * Math.PI), notes);
  }

  /**
   * Phoenix's {@code GravityArmPositionOffset} for an arm whose control gives its gravity angle, in
   * radians within half a turn of the joint's zero.
   *
   * @throws IllegalArgumentException for an angle more than a quarter turn from the joint's zero,
   *     which Phoenix can't take
   */
  private static double givenOffset(Joint joint, double angle) {
    double offset = -angle / (2 * Math.PI);
    if (Math.abs(offset) > MAX_GRAVITY_OFFSET + ROUNDING) {
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
   * Phoenix's {@code GravityArmPositionOffset} for an offset in rotations worked out from the
   * definition. An offset close to Phoenix's limit is noted.
   *
   * @throws IllegalArgumentException for an offset past ±0.25 rotations
   */
  private static double gravityOffset(Joint joint, double offset, List<String> notes) {
    if (Math.abs(offset) > MAX_GRAVITY_OFFSET + ROUNDING) {
      // What the offset would be with the joint's axis turned around, for the message.
      double turned = offset > 0 ? 0.5 - offset : -0.5 - offset;
      if (Math.abs(turned) < ROUNDING) {
        turned = 0;
      }
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
    boolean nearTheLimit = Math.abs(offset) > MAX_GRAVITY_OFFSET - GRAVITY_OFFSET_MARGIN;
    if (nearTheLimit) {
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

  /**
   * A ratio, checked against what Phoenix holds.
   *
   * @param name what the ratio is, for the refusal
   * @throws IllegalArgumentException for a ratio that is zero, not a number, or past ±1000
   */
  private static double ratio(String name, double ratio) {
    boolean withinPhoenixsRange = Math.abs(ratio) <= MAX_RATIO;
    if (!withinPhoenixsRange || ratio == 0) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s would be %s, and Phoenix holds from -1000 to 1000, never 0",
              name,
              shown(ratio)));
    }
    return ratio;
  }

  /**
   * A kS or kG in volts, checked against what Phoenix holds.
   *
   * @throws IllegalArgumentException for volts outside -128 to 127
   */
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

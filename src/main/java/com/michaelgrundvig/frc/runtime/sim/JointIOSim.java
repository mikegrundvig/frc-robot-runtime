package com.michaelgrundvig.frc.runtime.sim;

import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.simulation.JointPlant;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;

/**
 * A simulated joint: its motor controller, as the joint's control and wiring describe it, driving
 * the joint's plant, its motors moving its link, built from the robot's definition alone ({@link
 * JointPlant}, in the simulation library: the motors through the transmission, the inertia,
 * gravity, the hard stops).
 *
 * <p>Gravity is the plant's: what the kG tuned in the joint's block holds, through its motors, or,
 * where its block tunes none, its links' weight. It's the robot's, from its block in the definition
 * the simulator was built from, not its motor controller's: a test can run the controller with
 * another kG, re-tuning the joint ({@code PIVOT.retuned(control -> control.withKg(0))}) in a robot
 * whose definition keeps its own, and live tuning doesn't move it.
 *
 * <p>The motor controller runs as the controller its block names runs what its vendor's helper
 * configures, from the joint's plan ({@link ControllerPlan}), its loop every millisecond on the
 * joint's own position and speed: a TalonFX's slot, the plan's whole law on the device ({@link
 * TalonFxEmulator}), for a TalonFX or a TalonFXS; a SPARK's kP on the device, with the plan's
 * feedforward the robot works out as it asks ({@link SparkEmulator}). A joint wired to nothing runs
 * on the controller its motor runs on, as its settings are checked: a NEO's a SPARK MAX, a Kraken's
 * a TalonFX ({@link ControllerPlan#controllers}). Either way: its stator current limit; its soft
 * limits, past which it drives no further; and neutral as its control says, braking or coasting.
 * Like a real one, it knows only where its own count says the joint is: from where the joint was at
 * power on, taken to be where it rests, until homing tells it otherwise ({@link #powerOnAt}, {@link
 * #resetPosition}). The battery holds 12 V, and the supply current limit isn't modeled, as the
 * drivetrain's isn't ({@code ModuleIOSim}).
 *
 * <p>Its limits are what the robot's IO for the joint is given ({@link ControllerPins#of}): a
 * switch on one of its controller's own pins, which it reads to the robot with the joint ({@link
 * JointIO.JointIOInputs#forwardLimit}), and a limit switch on a CANdi a TalonFX reads over CAN,
 * which the robot reads as a sensor of its own. On a limit switch it stops by itself, in its
 * millisecond loop, homing's push included: it drives no further toward the switch's end while the
 * switch reads tripped, as the simulated world has it ({@link SensorSimulation}), whatever the
 * robot asks. Its forward limit stops it moving the joint forward, as the template takes a
 * controller's forward to be (open question M22).
 *
 * <p>An absolute encoder it reads, a CANcoder or one on a SPARK's data port, tells it where the
 * joint is, as a real one's IO seeds it from the encoder, and is read each loop beside its count
 * where the robot's IO reads it so ({@link JointIO.JointIOInputs#absolutePosition}): a CANcoder its
 * TalonFX reads its rotor beside, or a data port. A test can freeze that encoder, or slip it on its
 * shaft ({@link JointSimulation#freeze}, {@link JointSimulation#slip}).
 *
 * <p>A test can fail it as a robot's fails ({@link JointSimulation}): silent, as one off the CAN
 * bus is, it answers nothing and hears nothing, so it goes neutral; restarted, as one whose breaker
 * reset is, it says so once, goes neutral until it's asked again, and counts from 0, or reads the
 * absolute encoder its block has it read, as a real one is seeded from it again; jammed, it stays
 * where it is whatever its motors do, which stall against what holds it. One of its followers can
 * go off the bus alone: neutral, drawing nothing, the joint driven by the rest. It keeps its tuning
 * through a restart: a TalonFX's is saved on it, and a SPARK's IO tells it again.
 *
 * <p>What the robot would refuse of the joint's settings, its configuration refused before the
 * simulator started; what it would doubt, the robot says at startup, in the simulator as on the
 * robot ({@link ControllerSettings#findings}).
 */
final class JointIOSim implements JointIO {
  /**
   * A motor controller runs its closed loop every millisecond: a TalonFX's (CTRE's Phoenix 6
   * documentation: 1 kHz), and a SPARK's.
   */
  static final double DEVICE_LOOP_SECONDS = 0.001;

  private enum Mode {
    NEUTRAL,
    POSITION,
    VELOCITY,
    VOLTAGE,
    DUTY_CYCLE,
    HOMING
  }

  private final Joint joint;
  private final RobotMath math;
  private final JointPlant plant;
  private final ControllerEmulator controller;

  /** What its motor controller has on its limits, as the robot's IO is told. */
  private final ControllerPins pins;

  /**
   * Whether the switch on each of its limits reads tripped, as the simulated world has it: never,
   * until the world's sensors are simulated ({@link #readsLimits}).
   */
  private BooleanSupplier forwardTripped = () -> false;

  private BooleanSupplier reverseTripped = () -> false;

  /** How its motor controller drives it: its block's control, or as the pit tunes it. */
  private ControllerPlan plan;

  /** Where the motor controller reads the joint, less where it really is: 0 until misplaced. */
  private double offset;

  /** Where, and how fast, the motor controller last told the robot the joint is. */
  private double read;

  private double readVelocity;

  private Mode mode = Mode.NEUTRAL;
  private double setVolts;

  /** Seconds of the robot's loop not yet run, under a millisecond: the next step's. */
  private double owed;

  /** Whether it's silent, off the bus: it answers nothing and hears nothing. */
  private boolean silent;

  /** Whether it restarted since the robot last read it. */
  private boolean restarted;

  /**
   * The absolute encoder it reads beside its count, where the robot's IO reads it so; null for
   * none.
   */
  private final Sensor.@Nullable Encoder absolute;

  /**
   * The absolute encoder its motor controller reads and is seeded from, as a real one is at power
   * on and after it restarts: inside the window its turn tells apart. Null for none.
   */
  private final Sensor.@Nullable Encoder seedsFrom;

  /**
   * Whether its motor controller's own loop runs on the absolute encoder it reads, not on its
   * motor's count: a TalonFX's on its CANcoder (remote, fused or sync), a SPARK's on its data port.
   * Its position is then the encoder's.
   */
  private final boolean loopOnEncoder;

  /** Whether the absolute encoder it reads is stuck, as a frozen encoder's is. */
  boolean encoderFrozen;

  /** How far off the absolute encoder it reads says the joint is, in its units: a loose magnet. */
  double encoderSlip;

  /** Where that encoder last said the joint is, unwrapped: what it says while frozen. */
  private double encoderSaid = Double.NaN;

  /**
   * Its devices, as its block wires them: its motor controllers, the leader first, then a CANcoder
   * if it has one; and its followers. None faults, and its followers answer with it.
   */
  private final int devices;

  private final int followers;

  /** Which of its followers are off the CAN bus, neutral, the joint driven without them. */
  private final boolean[] silentFollowers;

  /**
   * @param math what the robot's definition derives: its joint's block sets the gravity on it
   * @param joint the joint as its motor controller runs it: its block's control and wiring
   */
  JointIOSim(RobotMath math, Joint joint) {
    this(math, joint, ControllerPins.NONE);
  }

  /**
   * The same, with what's on its motor controller's limits.
   *
   * @param pins its limits, from the robot's sensor wiring ({@link ControllerPins#of})
   */
  JointIOSim(RobotMath math, Joint joint, ControllerPins pins) {
    this.joint = joint;
    this.math = math;
    this.pins = pins;
    plan = ControllerPlan.of(math, joint, ControllerSettings.control(joint));
    plant = JointPlant.of(math, joint);
    controller = ControllerEmulator.of(plan);
    read = plant.position();
    absolute = readsBeside(joint) ? math.absoluteEncoder(joint) : null;
    int controllers = joint.driven().motors().map(motors -> motors.controllers().size()).orElse(0);
    followers = Math.max(0, controllers - 1);
    silentFollowers = new boolean[followers];
    devices =
        controllers
            + (joint
                    .driven()
                    .absoluteEncoder()
                    .flatMap(Driven.AbsoluteEncoder::cancoder)
                    .isPresent()
                ? 1
                : 0);
    loopOnEncoder =
        joint
            .driven()
            .absoluteEncoder()
            .map(Driven.AbsoluteEncoder::runsControllerLoop)
            .orElse(false);
    seedsFrom = readsEncoder() ? math.absoluteEncoder(joint) : null;
  }

  /** The joint. */
  Joint joint() {
    return joint;
  }

  /**
   * Reads the switches on its limits from now on, as the simulated world has them: whether each is
   * tripped.
   */
  void readsLimits(BooleanSupplier forward, BooleanSupplier reverse) {
    forwardTripped = forward;
    reverseTripped = reverse;
  }

  /**
   * Gravity's torque on the joint, positive turning it forward, with the joints where they stand:
   * newton meters, or newtons for a slide ({@link JointPlant#gravityTorque}).
   */
  double gravityTorque(Map<Joint, Double> positions) {
    return plant.gravityTorque(positions);
  }

  /**
   * The current each of its motors draws from the battery, in amps: what its controller reads, and
   * what its PDH channel does.
   */
  double supplyAmps() {
    return plant.supplyAmps();
  }

  /** Where the joint really is, in its units. */
  double position() {
    return plant.position();
  }

  /** How fast it really moves. */
  double velocity() {
    return plant.velocity();
  }

  /** Silences it, or brings it back: off the CAN bus, it answers nothing and goes neutral. */
  void silence(boolean silent) {
    this.silent = silent;
    if (silent) {
      mode = Mode.NEUTRAL;
    }
  }

  /**
   * Takes one of its followers off the CAN bus, or brings it back: it answers nothing and hears
   * nothing, so it goes neutral, drawing nothing from the battery, and its leader drives the joint
   * without it.
   *
   * @param follower which, from 0 for the first after the leader
   * @throws IllegalArgumentException for a follower it hasn't
   */
  void silenceFollower(int follower, boolean silent) {
    if (follower < 0 || follower >= followers) {
      throw new IllegalArgumentException(
          joint.label() + " has " + followers + " followers, not one numbered " + follower);
    }
    silentFollowers[follower] = silent;
    int driving = 1;
    for (boolean each : silentFollowers) {
      driving += each ? 0 : 1;
    }
    plant.drivenBy(driving);
  }

  /**
   * The current one of its motor controllers draws from the battery, in amps: what its PDH channel
   * reads. Each that drives the joint draws alike; a follower off the bus draws nothing.
   *
   * @param controller which, from 0 for the leader
   */
  double supplyAmps(int controller) {
    return controller > 0 && controller <= followers && silentFollowers[controller - 1]
        ? 0
        : plant.supplyAmps();
  }

  /**
   * Jams it where it is, or frees it: stuck still, as a game piece wedged in it or a bent part
   * leaves it, whatever its motors do, which stall against it as against a hard stop.
   */
  void jam(boolean jammed) {
    plant.jam(jammed);
  }

  /**
   * Restarts it: neutral until it's asked again, it counts from 0, or reads where the joint is from
   * the absolute encoder its controller reads, inside the window the encoder's turn tells apart, as
   * a real one is seeded, a joint that turns past it included; and it says it restarted the next
   * time it's read.
   */
  void restart() {
    mode = Mode.NEUTRAL;
    restarted = true;
    Sensor.Encoder seeds = seedsFrom;
    offset = (seeds != null ? inWindow(seeds) : 0) - plant.position();
  }

  /** Whether its motor controller reads an absolute encoder, and is seeded from it. */
  private boolean readsEncoder() {
    return joint
        .driven()
        .absoluteEncoder()
        .map(Driven.AbsoluteEncoder::readByController)
        .orElse(false);
  }

  /**
   * Whether the robot's IO reads the absolute encoder its motor controller reads each loop, beside
   * its count: a CANcoder its TalonFX reads its rotor beside, or a SPARK's data port.
   */
  private static boolean readsBeside(Joint joint) {
    return joint
        .driven()
        .absoluteEncoder()
        .map(
            encoder ->
                encoder.dataPort().isPresent()
                    || encoder
                        .cancoder()
                        .filter(on -> on.feedback() == MechanismWiring.Feedback.ROTOR)
                        .isPresent())
        .orElse(false);
  }

  /** Puts the joint somewhere, still, where its motor controller reads it too: a test's start. */
  void place(double position) {
    plant.place(position);
    offset = 0;
  }

  /**
   * Powers it on somewhere other than where it rests, still: its motor controller reads it where it
   * rests anyway, as a joint read by its motor alone does, until it's homed; or, seeded from the
   * absolute encoder it reads, inside the window the encoder's turn tells apart.
   */
  void powerOnAt(double position) {
    plant.place(position);
    Sensor.Encoder seeds = seedsFrom;
    offset = (seeds != null ? inWindow(seeds) : joint.restPosition()) - position;
  }

  /**
   * Runs the motor controller and the joint for a time, a millisecond at a time as the motor
   * controller does.
   *
   * @param gravityTorque gravity's torque on the joint through the step, positive turning it
   *     forward: newton meters, or newtons for a slide
   */
  void step(double seconds, double gravityTorque) {
    owed += seconds;
    while (owed >= DEVICE_LOOP_SECONDS - 1e-12) {
      owed -= DEVICE_LOOP_SECONDS;
      deviceLoop(gravityTorque);
    }
  }

  /** One millisecond: the motor controller's loop, then the joint, held at its stops. */
  private void deviceLoop(double gravityTorque) {
    double position = controllerReads(); // where the motor controller reads it
    double velocity = plant.velocity();
    double limit = plan.currentLimits().stator().in(Amps);
    if (mode == Mode.NEUTRAL) {
      if (plan.brakes()) {
        plant.run(0, limit, gravityTorque, DEVICE_LOOP_SECONDS);
      } else {
        plant.coast(gravityTorque, DEVICE_LOOP_SECONDS);
      }
      return;
    }
    double volts = Math.clamp(volts(position, velocity), -Motor.NOMINAL_VOLTS, Motor.NOMINAL_VOLTS);
    if (mode != Mode.HOMING && pastSoftLimit(position, volts)) {
      volts = 0; // homing ignores the soft limits
    }
    if (atLimitSwitch(volts)) {
      volts = 0; // and stops on its limit switches, as everything else does
    }
    plant.run(volts, limit, gravityTorque, DEVICE_LOOP_SECONDS);
  }

  /** Whether it's at or past a soft limit, driven on past it. */
  private boolean pastSoftLimit(double position, double volts) {
    return plan.softLimits()
        .map(
            limits ->
                (position >= limits.upper() && volts > 0)
                    || (position <= limits.lower() && volts < 0))
        .orElse(false);
  }

  /** Whether a limit switch it stops on reads tripped, driven on toward it. */
  private boolean atLimitSwitch(double volts) {
    return (volts > 0 && stops(pins.forward()) && forwardTripped.getAsBoolean())
        || (volts < 0 && stops(pins.reverse()) && reverseTripped.getAsBoolean());
  }

  private static boolean stops(Optional<ControllerPins.Limit> limit) {
    return limit.map(ControllerPins.Limit::stops).orElse(false);
  }

  /** Whether a limit is a switch on its own pin, which it reads to the robot with the joint. */
  private static boolean onPin(Optional<ControllerPins.Limit> limit) {
    return limit.orElse(null) instanceof ControllerPins.OnPin;
  }

  /** What the motor controller asks of the motors this millisecond. */
  private double volts(double position, double velocity) {
    return switch (mode) {
      case POSITION -> controller.holding(position, velocity, DEVICE_LOOP_SECONDS);
      case VELOCITY -> controller.running(position, velocity);
      case VOLTAGE, HOMING -> setVolts;
      case DUTY_CYCLE -> setVolts * Motor.NOMINAL_VOLTS;
      case NEUTRAL -> 0;
    };
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    if (silent) {
      inputs.connected = false; // its last readings stand, as a robot's do
      Arrays.fill(inputs.followersConnected, false);
      return;
    }
    inputs.restarted = restarted;
    restarted = false;
    inputs.connected = true;
    if (inputs.faults.length != devices) {
      inputs.faults = new int[devices];
      inputs.followersConnected = new boolean[followers];
      inputs.followersRestarted = new boolean[followers];
    }
    for (int i = 0; i < followers; i++) {
      inputs.followersConnected[i] = !silentFollowers[i];
    }
    inputs.position = controllerReads();
    // A SPARK's loop closed on its data port: its motor's count beside, as its IO reads it.
    inputs.motorPosition =
        loopOnEncoder && absolute != null ? plant.position() + offset : Double.NaN;
    inputs.velocity = plant.velocity();
    inputs.appliedVolts = plant.appliedVolts();
    inputs.statorCurrentAmps = plant.statorAmps();
    inputs.supplyCurrentAmps = plant.supplyAmps();
    inputs.absolutePosition = absolutePosition();
    inputs.forwardLimit = onPin(pins.forward()) && forwardTripped.getAsBoolean();
    inputs.reverseLimit = onPin(pins.reverse()) && reverseTripped.getAsBoolean();
    inputs.unconfigured = "";
    read = inputs.position;
    readVelocity = inputs.velocity;
  }

  /**
   * Where its motor controller reads the joint: its motor's count, or the absolute encoder its loop
   * runs on.
   */
  private double controllerReads() {
    return loopOnEncoder ? encoderReads() : plant.position() + offset;
  }

  /**
   * Where the absolute encoder its motor controller reads says the joint is, unwrapped: where it
   * really is, as far off as the encoder slipped, or where it was while it's frozen.
   */
  private double encoderReads() {
    if (!encoderFrozen || Double.isNaN(encoderSaid)) {
      encoderSaid = plant.position() + encoderSlip;
    }
    return encoderSaid;
  }

  /**
   * Where the absolute encoder it reads beside its count says the joint is, wrapped into the window
   * its turn tells apart, as a real one's IO reads it: NaN for none.
   */
  private double absolutePosition() {
    Sensor.Encoder model = absolute;
    return model == null ? Double.NaN : inWindow(model);
  }

  /**
   * Where an absolute encoder the joint's motor controller reads says the joint is, wrapped into
   * the window its turn tells apart, as a real one reads it.
   */
  private double inWindow(Sensor.Encoder model) {
    double reading =
        encoderReads()
            * Math.abs(model.ratio(joint))
            / (joint instanceof Joint.Revolute ? 2 * Math.PI : 1);
    return math.fromAbsolute(model, reading - Math.floor(reading));
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    if (silent) {
      return;
    }
    controller.hold(
        position, velocity, feedforwardVolts, read, readVelocity, mode != Mode.POSITION);
    mode = Mode.POSITION;
  }

  @Override
  public void setVelocity(double velocity) {
    if (silent) {
      return;
    }
    controller.run(velocity, read);
    mode = Mode.VELOCITY;
  }

  @Override
  public void setVoltage(double volts) {
    if (silent) {
      return;
    }
    mode = Mode.VOLTAGE;
    setVolts = volts;
  }

  @Override
  public void setDutyCycle(double share) {
    if (silent) {
      return;
    }
    mode = Mode.DUTY_CYCLE;
    setVolts = share;
  }

  @Override
  public void stop() {
    mode = Mode.NEUTRAL;
  }

  @Override
  public void home(double volts) {
    if (silent) {
      return;
    }
    mode = Mode.HOMING;
    setVolts = volts;
  }

  @Override
  public void resetPosition(double position) {
    if (silent) {
      return;
    }
    offset = position - plant.position();
  }

  /** Takes the pit's tuning: its gains and current limits, as its motor controller would. */
  @Override
  public void retune(MechanismControl control) {
    plan = ControllerPlan.of(math, joint, control);
    controller.retune(plan);
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * The conformance suite's bench for the CTRE joint ({@link JointIOTalon}), against Phoenix's own
 * simulated devices: a TalonFX turning a Kraken, or a TalonFXS turning a NEO.
 *
 * <p>Phoenix's simulated devices run on the wall clock, on a thread of their own, and send a frame
 * about every 20 ms. So a step here waits for the leader's next frame, and the suite waits for what
 * it expects, never for a number of loops.
 *
 * <p>They cannot be taken off the bus or restarted. The bench does both where the driver hears its
 * devices ({@link JointIOTalon.Frames}): a silent device's frames do not come, and a restarted one
 * says it started and counts from zero.
 *
 * <p>The devices are the same ones for every test, at the same CAN IDs: Phoenix keeps a simulated
 * device for the life of the JVM. Powering on puts each back as a device out of its box would be,
 * and the driver's own startup configures it afresh. The IDs are ones no other test in the runtime
 * uses, whatever its bus is called: Phoenix's simulated devices are all on one bus, where a TalonFX
 * and a TalonFXS at the same ID garble each other's settings.
 */
public final class TalonJointBench implements JointBench {
  /** The first of the two TalonFXs' CAN IDs, and of the two TalonFXSs'. */
  private static final int FIRST_FX_ID = 4;

  private static final int FIRST_FXS_ID = 6;
  private static final int CANCODER_ID = 8;
  private static final int CANDI_ID = 10;

  /** The battery, in volts. */
  private static final double BATTERY_VOLTS = 12;

  /** How long a step waits for the leader's next frame, in seconds, before going on without it. */
  private static final double FRAME_WAIT_SECONDS = 0.1;

  /**
   * How long a read of a device's settings waits for them, in seconds. A simulated TalonFXS that
   * has just been sent settings without waiting can take most of a second to answer.
   */
  private static final double SETTINGS_WAIT_SECONDS = 1;

  /** How many times a read of a device's settings asks for them before it gives up. */
  private static final int SETTINGS_TRIES = 5;

  private final MechanismWiring.Controller kind;
  private final String busName;

  /**
   * @param kind the controller on the bench: a TalonFX or a TalonFXS
   */
  public TalonJointBench(MechanismWiring.Controller kind) {
    this.kind = kind;
    busName = "bench-" + kind.name().toLowerCase(java.util.Locale.ROOT);
    SignalLogger.enableAutoLogging(false);
  }

  private CanBus bus() {
    return CanBus.canivore(busName);
  }

  @Override
  public String name() {
    return kind.shown();
  }

  @Override
  public Motor motor() {
    return kind == MechanismWiring.Controller.TALON_FXS ? Motor.NEO : Motor.KRAKEN_X60;
  }

  @Override
  public MechanismWiring.Device controller(int index) {
    int first = kind == MechanismWiring.Controller.TALON_FXS ? FIRST_FXS_ID : FIRST_FX_ID;
    return kind.can(bus(), first + index);
  }

  @Override
  public Optional<MechanismWiring.EncoderWiring> encoderBesideTheCount() {
    return Optional.of(CANCODER.can(bus(), CANCODER_ID).offset(Rotations.of(0)));
  }

  @Override
  public Optional<MechanismWiring.EncoderWiring> encoderTheLoopRunsOn() {
    return Optional.of(CANCODER.can(bus(), CANCODER_ID).offset(Rotations.of(0)).remote());
  }

  @Override
  public Optional<ControllerPins.Limit> limitReadElsewhere() {
    return Optional.of(new ControllerPins.OnCandi(CANDI_ID, 1));
  }

  @Override
  public Rig powerOn(RobotDefinition robot, Joint joint, ControllerPins pins, double poweredOnAt) {
    return new TalonRig(robot, joint, pins, poweredOnAt, new CANBus(busName));
  }

  @Override
  public Optional<String> lacks(Ability ability) {
    boolean followerNotShown =
        ability == Ability.SHOW_A_FOLLOWERS_OUTPUT && kind == MechanismWiring.Controller.TALON_FXS;
    if (followerNotShown) {
      return Optional.of(
          "Phoenix's simulated TalonFXS in follower mode reports its output off most of the time,"
              + " whatever its leader does and at whatever rate the leader's frames come (probed"
              + " with Phoenix 26.70.0-alpha-2): whether a TalonFXS follows is a check for the"
              + " bench");
    }
    return Optional.empty();
  }

  @Override
  public Optional<String> knownDifference(JointClause clause) {
    return switch (clause) {
      case J29 ->
          Optional.of(
              "Phoenix keeps a TalonFX's last reading and the driver does not say when a frame"
                  + " did not come: it always reads fresh");
      default -> Optional.empty();
    };
  }

  /**
   * One simulated motor controller, as the bench drives it: Phoenix has a class for each of the two
   * kinds, and a simulation class for each, with the same methods and no common type.
   */
  private sealed interface Controller {
    CommonTalon talon();

    void powerOn(double batteryVolts);

    void rotor(double turns, double turnsPerSecond);

    void limitPin(SensorWiring.Direction direction, boolean closedToGround);

    double motorVolts();

    StatusCode read(MotorOutputConfigs into);

    StatusCode read(CurrentLimitsConfigs into);

    StatusCode read(Slot1Configs into);

    static Controller open(MechanismWiring.Controller kind, int id, CANBus bus) {
      if (kind == MechanismWiring.Controller.TALON_FXS) {
        return new Fxs(new TalonFXS(id, bus));
      }
      return new Fx(new TalonFX(id, bus));
    }
  }

  private record Fx(TalonFX device) implements Controller {
    @Override
    public CommonTalon talon() {
      return device;
    }

    @Override
    public void powerOn(double batteryVolts) {
      device.getSimState().setSupplyVoltage(batteryVolts);
    }

    @Override
    public void rotor(double turns, double turnsPerSecond) {
      device.getSimState().setRawRotorPosition(turns);
      device.getSimState().setRotorVelocity(turnsPerSecond);
    }

    @Override
    public void limitPin(SensorWiring.Direction direction, boolean closedToGround) {
      if (direction == SensorWiring.Direction.FORWARD) {
        device.getSimState().setForwardLimit(closedToGround);
      } else {
        device.getSimState().setReverseLimit(closedToGround);
      }
    }

    @Override
    public double motorVolts() {
      return device.getSimState().getMotorVoltage();
    }

    @Override
    public StatusCode read(MotorOutputConfigs into) {
      return device.getConfigurator().refresh(into, SETTINGS_WAIT_SECONDS);
    }

    @Override
    public StatusCode read(CurrentLimitsConfigs into) {
      return device.getConfigurator().refresh(into, SETTINGS_WAIT_SECONDS);
    }

    @Override
    public StatusCode read(Slot1Configs into) {
      return device.getConfigurator().refresh(into, SETTINGS_WAIT_SECONDS);
    }
  }

  private record Fxs(TalonFXS device) implements Controller {
    @Override
    public CommonTalon talon() {
      return device;
    }

    @Override
    public void powerOn(double batteryVolts) {
      device.getSimState().setSupplyVoltage(batteryVolts);
    }

    @Override
    public void rotor(double turns, double turnsPerSecond) {
      device.getSimState().setRawRotorPosition(turns);
      device.getSimState().setRotorVelocity(turnsPerSecond);
    }

    @Override
    public void limitPin(SensorWiring.Direction direction, boolean closedToGround) {
      if (direction == SensorWiring.Direction.FORWARD) {
        device.getSimState().setForwardLimit(closedToGround);
      } else {
        device.getSimState().setReverseLimit(closedToGround);
      }
    }

    @Override
    public double motorVolts() {
      return device.getSimState().getMotorVoltage();
    }

    @Override
    public StatusCode read(MotorOutputConfigs into) {
      return device.getConfigurator().refresh(into, SETTINGS_WAIT_SECONDS);
    }

    @Override
    public StatusCode read(CurrentLimitsConfigs into) {
      return device.getConfigurator().refresh(into, SETTINGS_WAIT_SECONDS);
    }

    @Override
    public StatusCode read(Slot1Configs into) {
      return device.getConfigurator().refresh(into, SETTINGS_WAIT_SECONDS);
    }
  }

  /**
   * Where the driver hears its devices, with the bench's hand on it: a silent device's frames do
   * not come, one frame of the leader's can be lost, and a device can be said to have started
   * again. Everything else is Phoenix's.
   */
  private static final class BenchFrames extends JointIOTalon.Frames {
    /** A joint has a leader, its followers and a CANcoder: never this many devices. */
    private static final int MOST_DEVICES = 8;

    private final boolean[] silent = new boolean[MOST_DEVICES];
    private final boolean[] startedAgain = new boolean[MOST_DEVICES];
    private boolean losesTheLeadersNextFrame;

    @Override
    StatusCode refresh(int device, BaseStatusSignal[] signals) {
      if (silent[device]) {
        return StatusCode.RxTimeout;
      }
      if (device == Rig.LEADER && losesTheLeadersNextFrame) {
        losesTheLeadersNextFrame = false;
        return StatusCode.RxTimeout;
      }
      return super.refresh(device, signals);
    }

    @Override
    StatusCode refresh(int device, StatusSignal<?> signal) {
      if (silent[device]) {
        return StatusCode.RxTimeout;
      }
      return super.refresh(device, signal);
    }

    @Override
    BooleanSupplier startFrames(int device, ParentDevice parent) {
      BooleanSupplier phoenix = super.startFrames(device, parent);
      return () -> {
        boolean benchRestartedIt = startedAgain[device];
        startedAgain[device] = false;
        return phoenix.getAsBoolean() || benchRestartedIt;
      };
    }
  }

  /** One CTRE joint, powered on. */
  private static final class TalonRig implements Rig {
    private final JointIOTalon io;
    private final BenchFrames frames = new BenchFrames();
    private final List<MechanismWiring.Device> wired;
    private final List<Controller> controllers = new ArrayList<>();
    private final Optional<CANcoder> cancoder;
    private final Optional<CANdi> candi;
    private final ControllerPins pins;
    private final MechanismRotations rotations;

    /** How many turns a rotor makes for one rotation of the mechanism. */
    private final double rotorRatio;

    /** Whether the leader reads the joint from its CANcoder, and not by its rotor's count. */
    private final boolean leaderReadsTheCancoder;

    /** The leader's position signal, on the bench's own handle: a step waits for its next frame. */
    private final StatusSignal<?> leaderFrame;

    private final long poweredOnNanos = System.nanoTime();
    private final double poweredOnAt;
    private double encoderSlip;
    private double position;
    private double velocity;

    TalonRig(
        RobotDefinition robot, Joint joint, ControllerPins pins, double poweredOnAt, CANBus bus) {
      this.pins = pins;
      this.poweredOnAt = poweredOnAt;
      position = poweredOnAt;
      ControllerPlan plan =
          ControllerPlan.of(robot.math(), joint, ControllerSettings.control(joint));
      rotations = plan.rotations();
      rotorRatio = rotations.ratioPerRotation(plan.rotorRatio());
      wired = joint.driven().motors().orElseThrow().controllers();
      Optional<MechanismWiring.Cancoder> wiredCancoder =
          joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder);
      leaderReadsTheCancoder =
          wiredCancoder.isPresent()
              && wiredCancoder.get().feedback() != MechanismWiring.Feedback.ROTOR;

      // Each device as it comes out of its box, the joint where it is powered on.
      RobotSim.setEnabled(true);
      for (MechanismWiring.Device each : wired) {
        Controller controller = Controller.open(each.controller(), each.id(), bus);
        controller.powerOn(BATTERY_VOLTS);
        controller.rotor(0, 0);
        controller.talon().setControl(new NeutralOut());
        controllers.add(controller);
      }
      cancoder = wiredCancoder.map(wiring -> new CANcoder(wiring.id(), bus));
      cancoder.ifPresent(
          device -> device.getSimState().setMagnetHealth(MagnetHealthValue.Magnet_Green));
      candi = candiId(pins).map(id -> new CANdi(id, bus));
      limit(SensorWiring.Direction.FORWARD, false);
      limit(SensorWiring.Direction.REVERSE, false);
      turnTheSensors(0);

      io = new JointIOTalon(robot.math(), joint, pins, frames);
      leaderFrame = controllers.get(LEADER).talon().getPosition();
    }

    /** The CANdi a limit is read from, if either is. */
    private static Optional<Integer> candiId(ControllerPins pins) {
      return pins.forward()
          .or(pins::reverse)
          .filter(ControllerPins.OnCandi.class::isInstance)
          .map(limit -> ((ControllerPins.OnCandi) limit).id());
    }

    /**
     * Turns each controller's rotor, and the CANcoder's magnet, to where the joint is. A rotor
     * counts from where it was powered on, and counts backward when its motor is mounted clockwise.
     * The CANcoder is on the joint's own axle and reads zero with the joint at zero.
     */
    private void turnTheSensors(double velocity) {
      double turnsSincePowerOn = (rotations.toRotations(position - poweredOnAt)) * rotorRatio;
      double turnsPerSecond = rotations.toRotations(velocity) * rotorRatio;
      for (int i = 0; i < controllers.size(); i++) {
        double way = forward(i);
        controllers.get(i).rotor(way * turnsSincePowerOn, way * turnsPerSecond);
      }
      cancoder.ifPresent(
          device ->
              device.getSimState().setRawPosition(rotations.toRotations(position + encoderSlip)));
    }

    /** Which way a controller's rotor turns as the joint moves forward: 1, or -1 for clockwise. */
    private double forward(int device) {
      return wired.get(device).turnsClockwise() ? -1 : 1;
    }

    @Override
    public JointIO io() {
      return io;
    }

    @Override
    public void step() {
      RobotSim.stepClock();
      turnTheSensors(velocity);
      leaderFrame.waitForUpdate(FRAME_WAIT_SECONDS, false);
    }

    @Override
    public double deviceSeconds() {
      return (System.nanoTime() - poweredOnNanos) / 1e9;
    }

    @Override
    public void jointIs(double position) {
      jointMoves(position, 0);
    }

    @Override
    public void jointMoves(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
      turnTheSensors(velocity);
    }

    @Override
    public void encoderSlips(double off) {
      encoderSlip = off;
      turnTheSensors(velocity);
    }

    /**
     * Sets the level of the pin, or of the CANdi's input, the switch is on. A normally open switch
     * closes its pin to ground as it trips, and a normally closed one opens it.
     */
    @Override
    public void limit(SensorWiring.Direction direction, boolean tripped) {
      Optional<ControllerPins.Limit> on =
          direction == SensorWiring.Direction.FORWARD ? pins.forward() : pins.reverse();
      if (on.isPresent() && on.get() instanceof ControllerPins.OnCandi) {
        S1StateValue level = tripped ? S1StateValue.Low : S1StateValue.Floating;
        candi.orElseThrow().getSimState().setS1State(level);
        return;
      }
      boolean normallyClosed =
          on.isPresent()
              && on.get() instanceof ControllerPins.OnPin pin
              && pin.polarity() == SensorWiring.Polarity.NORMALLY_CLOSED;
      controllers.get(LEADER).limitPin(direction, normallyClosed != tripped);
    }

    /**
     * Says the device started again. A leader that counts its rotor counts from zero again; one
     * that reads its CANcoder reads on. Phoenix keeps a device's settings through a restart.
     */
    @Override
    public void restart(int device) {
      frames.startedAgain[device] = true;
      if (device == LEADER && !leaderReadsTheCancoder) {
        controllers.get(LEADER).talon().setPosition(0, SETTINGS_WAIT_SECONDS);
      }
    }

    @Override
    public void loseAFrame() {
      frames.losesTheLeadersNextFrame = true;
    }

    @Override
    public void silence(int device, boolean silent) {
      frames.silent[device] = silent;
    }

    @Override
    public double motorVolts(int device) {
      return forward(device) * controllers.get(device).motorVolts();
    }

    @Override
    public MechanismControl.NeutralMode neutralMode() {
      MotorOutputConfigs output = new MotorOutputConfigs();
      read(() -> controllers.get(LEADER).read(output));
      return output.NeutralMode == NeutralModeValue.Brake
          ? MechanismControl.NeutralMode.BRAKE
          : MechanismControl.NeutralMode.COAST;
    }

    @Override
    public OptionalDouble supplyLimitAmps() {
      CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
      read(() -> controllers.get(LEADER).read(limits));
      if (!limits.SupplyCurrentLimitEnable) {
        return OptionalDouble.empty();
      }
      return OptionalDouble.of(limits.SupplyCurrentLimit);
    }

    @Override
    public double positionKi() {
      throw new UnsupportedOperationException(
          "Phoenix's simulated controller adds its integral up in time");
    }

    /** A Talon runs its speeds on a slot of their own, which the bench reads back from it. */
    @Override
    public OptionalDouble speedKi() {
      Slot1Configs speeds = new Slot1Configs();
      read(() -> controllers.get(LEADER).read(speeds));
      return OptionalDouble.of(speeds.kI);
    }

    /**
     * Reads some of the leader's settings back from it, asking again a few times if it does not
     * answer: a simulated controller that is busy taking settings can miss a request for them.
     */
    private static void read(Supplier<StatusCode> settings) {
      StatusCode read = StatusCode.RxTimeout;
      for (int i = 0; i < SETTINGS_TRIES && !read.isOK(); i++) {
        read = settings.get();
      }
      if (!read.isOK()) {
        throw new IllegalStateException("the leader did not send its settings: " + read);
      }
    }

    /** Leaves every controller neutral: Phoenix's simulated devices outlive the test. */
    @Override
    public void close() {
      for (Controller controller : controllers) {
        controller.talon().setControl(new NeutralOut());
      }
    }
  }
}

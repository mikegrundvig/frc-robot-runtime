package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.rev.RevMechanism;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.util.Signal;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * The conformance suite's bench for the REV joint ({@link JointIOSpark}), against REV's own
 * simulated SPARK: a SPARK MAX turning a NEO, or a SPARK Flex turning a NEO Vortex.
 *
 * <p>REV's simulated SPARK runs when it is told to ({@code SparkSim.iterate}), with the speed its
 * motor turns at, so a step here is exact and the same every time. It keeps a position of its own,
 * which it writes over the SPARK's encoder at every step. So when the driver tells the SPARK where
 * the joint is, the bench carries that into the simulation's position, as the SPARK's firmware
 * would.
 *
 * <p>With a SPARK's loop on the encoder on its data port, the simulation has one position for both
 * sensors: it writes it over the absolute encoder's too. The bench then keeps that position at what
 * the encoder reads, and the motor's own count is not shown apart from it.
 *
 * <p>It adds its loop's integral up once a step, where a SPARK adds it up every millisecond. So the
 * suite reads a SPARK's kI back from what the SPARK took ({@link Rig#positionKi}).
 *
 * <p>It always answers and never restarts. A lost frame goes in through the driver's own door for
 * one ({@link JointIOSpark#updateInputs(JointIO.JointIOInputs, Signal, Signal)}). A restart is the
 * "has reset" warning REV's simulation can raise, with what a restart does to a SPARK done by the
 * bench: its motor's count back at zero, and its settings back as they were saved at startup.
 *
 * <p>REV allows one object for a CAN ID, and a SPARK opened again at the ID of one that was closed
 * reads nothing from REV's simulation. So every power-on here is on SPARKs at IDs that have not
 * been used, on a CAN port no other test's SPARKs are on, and nothing is closed.
 */
public final class SparkJointBench implements JointBench {
  /** The battery, in volts. */
  private static final double BATTERY_VOLTS = 12;

  private static final double MILLISECONDS_PER_SECOND = 1000;

  /** A SPARK reports its motor's speed in rotations a minute. */
  private static final double SECONDS_PER_MINUTE = 60;

  /**
   * The first CAN ID not yet used on each kind's port, for the life of the JVM: REV's simulated
   * SPARKs outlive the test that opened them.
   */
  private static final AtomicInteger NEXT_MAX_ID = new AtomicInteger(1);

  private static final AtomicInteger NEXT_FLEX_ID = new AtomicInteger(1);

  private final MechanismWiring.Controller kind;

  /**
   * @param kind the controller on the bench: a SPARK MAX or a SPARK Flex
   */
  public SparkJointBench(MechanismWiring.Controller kind) {
    this.kind = kind;
  }

  private boolean isFlex() {
    return kind == MechanismWiring.Controller.SPARK_FLEX;
  }

  @Override
  public String name() {
    return kind.shown();
  }

  @Override
  public Motor motor() {
    return isFlex() ? Motor.NEO_VORTEX : Motor.NEO;
  }

  private AtomicInteger nextId() {
    return isFlex() ? NEXT_FLEX_ID : NEXT_MAX_ID;
  }

  /** The port the bench's SPARKs are on: one for each kind, and neither the tests' usual port. */
  private CanBus port() {
    return CanBus.port(isFlex() ? CanBus.Systemcore.CAN_S3 : CanBus.Systemcore.CAN_S2);
  }

  /** The next SPARKs not yet used: the ones the next power-on opens. */
  @Override
  public MechanismWiring.Device controller(int index) {
    return kind.can(port(), nextId().get() + index);
  }

  @Override
  public Optional<MechanismWiring.EncoderWiring> encoderBesideTheCount() {
    return Optional.of(DATA_PORT.offset(Rotations.of(0)));
  }

  @Override
  public Optional<MechanismWiring.EncoderWiring> encoderTheLoopRunsOn() {
    return Optional.of(DATA_PORT.offset(Rotations.of(0)).closesLoop());
  }

  @Override
  public Optional<ControllerPins.Limit> limitReadElsewhere() {
    return Optional.empty();
  }

  @Override
  public Rig powerOn(RobotDefinition robot, Joint joint, ControllerPins pins, double poweredOnAt) {
    // The joint was wired to the next unused SPARKs: the power-on after this one gets the next.
    nextId().addAndGet(joint.driven().motors().orElseThrow().controllers().size());
    return new SparkRig(robot, joint, pins, poweredOnAt, kind, motor());
  }

  @Override
  public Optional<String> lacks(Ability ability) {
    return switch (ability) {
      case SILENCE_A_FOLLOWER -> Optional.of("REV's simulated SPARK always sends its faults");
      case SHOW_A_FOLLOWERS_OUTPUT -> Optional.of("REV's simulated SPARK does not follow another");
      case SHOW_THE_SUPPLY_LIMIT -> Optional.of("a SPARK has no limit on its supply current");
      case INTEGRATE_IN_TIME ->
          Optional.of("REV's simulated SPARK adds its integral up once a step, not every 1 ms");
      case TURN_THE_MOTOR_APART_FROM_ITS_LOOPS_ENCODER ->
          Optional.of(
              "REV's simulated SPARK has one position for its motor and the encoder its loop runs"
                  + " on");
      case RESTART_THE_LEADER, RESTART_A_FOLLOWER, LOSE_A_FRAME, SILENCE_THE_LEADER ->
          Optional.empty();
    };
  }

  @Override
  public Optional<String> knownDifference(JointClause clause) {
    return Optional.empty();
  }

  /** One REV joint, powered on. */
  private static final class SparkRig implements Rig {
    /** A frame that did not come, as REV hands one over: an error, with REV's default of zero. */
    private static final Signal<Double> LOST = Signal.ofError(0.0, REVLibError.kTimeout, 0);

    private final JointIOSpark spark;
    private final SparkSim leader;
    private final List<SparkSim> followers = new ArrayList<>();
    private final List<SparkBase> opened = new ArrayList<>();
    private final MechanismWiring.Controller kind;

    /** The leader's settings at startup, which a restart falls back to. */
    private final RevMechanism.Settings startup;

    private final MechanismControl startupControl;
    private final MechanismRotations rotations;

    /** How many turns the motor makes for one rotation of the mechanism. */
    private final double rotorRatio;

    private final boolean hasAbsoluteEncoder;

    /** Whether the SPARK's loop runs on the encoder on its data port. */
    private final boolean loopRunsOnEncoder;

    /**
     * What the SPARK's count reads with the joint at zero, in the motor's turns. A count starts
     * where the definition rests the joint, wherever the joint is, and moves when the driver tells
     * the SPARK where the joint is.
     */
    private double countAtZero;

    /** What the simulation last wrote to the SPARK's encoder, to see the driver write over it. */
    private double lastWritten;

    private double position;
    private double velocity;
    private double encoderSlip;
    private int steps;
    private boolean silent;
    private boolean losesTheNextFrame;
    private final JointIO driver = new Driver();

    SparkRig(
        RobotDefinition robot,
        Joint joint,
        ControllerPins pins,
        double poweredOnAt,
        MechanismWiring.Controller kind,
        Motor motor) {
      this.kind = kind;
      position = poweredOnAt;
      startupControl = ControllerSettings.control(joint);
      startup = ControllerSettings.spark(robot.math(), joint, startupControl);
      rotations = startup.rotations();
      rotorRatio = startup.rotorRatio();
      hasAbsoluteEncoder = joint.driven().absoluteEncoder().isPresent();
      loopRunsOnEncoder = startup.closedLoop();

      RobotSim.setEnabled(true);
      spark = new JointIOSpark(robot.math(), joint, pins);
      opened.add(spark.leader());
      leader = new SparkSim(spark.leader(), motor.model());
      leader.setBusVoltage(BATTERY_VOLTS);
      for (int i = 0; i < startup.followers().size(); i++) {
        opened.add(spark.follower(i));
        followers.add(new SparkSim(spark.follower(i), motor.model()));
      }
      countAtZero = turns(joint.restPosition() - poweredOnAt);
      lastWritten = encoderCount();
      turnTheSensors();
    }

    /** The SPARK's own count of its motor's turns, as its encoder has it now. */
    private double encoderCount() {
      return spark.leader().getEncoder().getPosition().get(lastWritten);
    }

    /** A distance the joint moves, in the motor's turns. */
    private double turns(double distance) {
      return rotations.toRotations(distance) * rotorRatio;
    }

    /**
     * Puts the simulation's motor, and the encoder on the data port, where the joint is. REV's
     * simulated absolute encoder reports what it is given, as a share of a turn from the low edge
     * of the window the SPARK was told.
     */
    private void turnTheSensors() {
      leader.setPosition(simulatedPosition());
      if (hasAbsoluteEncoder) {
        leader.getAbsoluteEncoderSim().setPosition(encoderReads());
      }
    }

    /** What the encoder on the data port reads: a share of a turn from its window's low edge. */
    private double encoderReads() {
      double encoderTurns = rotations.toRotations(position + encoderSlip) - startup.windowEdge();
      return encoderTurns - Math.floor(encoderTurns);
    }

    /**
     * The simulation's one position: the motor's count, or, with the SPARK's loop on the data
     * port's encoder, what that encoder reads.
     */
    private double simulatedPosition() {
      if (loopRunsOnEncoder) {
        return encoderReads();
      }
      return countAtZero + turns(position);
    }

    /**
     * Takes what the driver told the SPARK its motor's count is, if it told it anything since the
     * last step: the simulation would write its own position over it otherwise.
     */
    private void takeWhereTheDriverSaysItIs() {
      if (loopRunsOnEncoder) {
        return;
      }
      double count = encoderCount();
      if (count != lastWritten) {
        countAtZero = count - turns(position);
        lastWritten = count;
      }
    }

    @Override
    public JointIO io() {
      return driver;
    }

    /** The driver, with the bench's hand on its frames: a lost or silent frame goes in as lost. */
    private final class Driver implements JointIO {
      @Override
      public void updateInputs(JointIOInputs inputs) {
        if (silent || losesTheNextFrame) {
          losesTheNextFrame = false;
          spark.updateInputs(inputs, LOST, LOST);
          return;
        }
        spark.updateInputs(inputs);
      }

      @Override
      public void setPosition(double position, double velocity, double feedforwardVolts) {
        spark.setPosition(position, velocity, feedforwardVolts);
      }

      @Override
      public void setVelocity(double velocity) {
        spark.setVelocity(velocity);
      }

      @Override
      public void setVoltage(double volts) {
        spark.setVoltage(volts);
      }

      @Override
      public void setDutyCycle(double share) {
        spark.setDutyCycle(share);
      }

      @Override
      public void stop() {
        spark.stop();
      }

      @Override
      public void home(double volts) {
        spark.home(volts);
      }

      @Override
      public void resetPosition(double position) {
        spark.resetPosition(position);
      }

      @Override
      public void retune(MechanismControl control) {
        spark.retune(control);
      }
    }

    /**
     * Runs the simulated SPARK for a loop with its motor at the joint's speed, the joint ending the
     * loop where the suite has it.
     */
    @Override
    public void step() {
      takeWhereTheDriverSaysItIs();
      double rpm = turns(velocity) * SECONDS_PER_MINUTE;
      double turnsThisLoop = rpm / SECONDS_PER_MINUTE * RobotLoop.PERIOD_SECONDS;
      leader.setPosition(simulatedPosition() - turnsThisLoop);
      leader.iterate(rpm, BATTERY_VOLTS, RobotLoop.PERIOD_SECONDS);
      lastWritten = encoderCount();
      RobotSim.stepClock();
      steps++;
    }

    @Override
    public double deviceSeconds() {
      return steps * RobotLoop.PERIOD_SECONDS;
    }

    @Override
    public void jointIs(double position) {
      jointMoves(position, 0);
    }

    @Override
    public void jointMoves(double position, double velocity) {
      takeWhereTheDriverSaysItIs();
      this.position = position;
      this.velocity = velocity;
      turnTheSensors();
    }

    @Override
    public void encoderSlips(double off) {
      takeWhereTheDriverSaysItIs();
      encoderSlip = off;
      turnTheSensors();
    }

    /** REV's simulated limit input is "pressed" after the SPARK's polarity: pressed is tripped. */
    @Override
    public void limit(SensorWiring.Direction direction, boolean tripped) {
      if (direction == SensorWiring.Direction.FORWARD) {
        leader.getForwardLimitSwitchSim().setPressed(tripped);
      } else {
        leader.getReverseLimitSwitchSim().setPressed(tripped);
      }
    }

    /**
     * Raises a SPARK's "has reset" warning, and does to it what a restart does: its settings are
     * the ones saved on it at startup again, and the leader's motor counts from zero.
     */
    @Override
    public void restart(int device) {
      SparkBase restarted = opened.get(device);
      SparkSim sim = device == LEADER ? leader : followers.get(device - 1);
      restarted.configure(
          savedTuning(), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      if (device == LEADER) {
        restarted.configure(
            startup.softLimitsEnabled(kind, true),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
        countAtZero = -turns(position);
        turnTheSensors();
        spark.leader().getEncoder().setPosition(0);
        lastWritten = encoderCount();
      }
      sim.getFaultManager()
          .setStickyWarnings(
              new SparkBase.Warnings(false, false, false, false, false, false, true, false));
    }

    /** The tuning a SPARK was saved with at startup: its block's gains and current limit. */
    private SparkBaseConfig savedTuning() {
      return startup.tuning(kind, startupControl);
    }

    @Override
    public void loseAFrame() {
      losesTheNextFrame = true;
    }

    @Override
    public void silence(int device, boolean silent) {
      if (device != LEADER) {
        throw new UnsupportedOperationException("REV's simulated follower cannot be silenced");
      }
      this.silent = silent;
    }

    @Override
    public double motorVolts(int device) {
      if (device != LEADER) {
        throw new UnsupportedOperationException("REV's simulated SPARK does not follow another");
      }
      return leader.getAppliedOutput() * leader.getBusVoltage();
    }

    @Override
    public MechanismControl.NeutralMode neutralMode() {
      SparkBaseConfig.IdleMode mode =
          switch (spark.leader()) {
            case SparkFlex flex -> flex.configAccessor.getIdleMode();
            case SparkMax max -> max.configAccessor.getIdleMode();
            default -> throw new IllegalStateException("neither a SPARK MAX nor a SPARK Flex");
          };
      return mode == SparkBaseConfig.IdleMode.kBrake
          ? MechanismControl.NeutralMode.BRAKE
          : MechanismControl.NeutralMode.COAST;
    }

    @Override
    public OptionalDouble supplyLimitAmps() {
      return OptionalDouble.empty();
    }

    /**
     * A SPARK holds its kI as a share of the battery's voltage for each turn of its motor and each
     * millisecond. The plan's is in volts for each of the joint's units and each second.
     */
    @Override
    public double positionKi() {
      double sparks =
          switch (spark.leader()) {
            case SparkFlex flex -> flex.configAccessor.closedLoop.getI(ClosedLoopSlot.kSlot0);
            case SparkMax max -> max.configAccessor.closedLoop.getI(ClosedLoopSlot.kSlot0);
            default -> throw new IllegalStateException("neither a SPARK MAX nor a SPARK Flex");
          };
      double motorTurnsPerUnit = rotorRatio / rotations.fromRotations(1);
      return sparks * BATTERY_VOLTS * motorTurnsPerUnit * MILLISECONDS_PER_SECOND;
    }

    /** A SPARK runs its speeds on slot 1, which the driver gives a kP alone. */
    @Override
    public OptionalDouble speedKi() {
      return OptionalDouble.of(
          switch (spark.leader()) {
            case SparkFlex flex -> flex.configAccessor.closedLoop.getI(ClosedLoopSlot.kSlot1);
            case SparkMax max -> max.configAccessor.closedLoop.getI(ClosedLoopSlot.kSlot1);
            default -> throw new IllegalStateException("neither a SPARK MAX nor a SPARK Flex");
          });
    }

    /** Leaves the SPARKs neutral. They stay open: see the bench. */
    @Override
    public void close() {
      spark.stop();
    }
  }
}

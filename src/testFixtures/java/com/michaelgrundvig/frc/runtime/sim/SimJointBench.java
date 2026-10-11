package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointBench;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointClause;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * The conformance suite's bench for the simulator's joint ({@link JointIOSim}), which emulates the
 * motor controller the joint's block names: a TalonFX turning a Kraken, or a SPARK MAX turning a
 * NEO. The simulator is its own device, so the bench moves its joint and reads its motor directly.
 */
public final class SimJointBench implements JointBench {
  /** The controller's CAN ID from which the bench numbers its controllers. Nothing is on a bus. */
  private static final int FIRST_ID = 10;

  private static final int CANCODER_ID = 20;
  private static final int CANDI_ID = 30;

  private final MechanismWiring.Controller emulated;

  /**
   * @param emulated the motor controller the simulator emulates: a TalonFX or a SPARK MAX
   */
  public SimJointBench(MechanismWiring.Controller emulated) {
    this.emulated = emulated;
  }

  private boolean emulatesATalon() {
    return emulated == MechanismWiring.Controller.TALON_FX;
  }

  @Override
  public String name() {
    return "the simulator emulating " + emulated.shown();
  }

  @Override
  public Motor motor() {
    return emulatesATalon() ? Motor.KRAKEN_X60 : Motor.NEO;
  }

  @Override
  public MechanismWiring.Device controller(int index) {
    return emulated.can(FIRST_ID + index);
  }

  @Override
  public Optional<MechanismWiring.EncoderWiring> encoderBesideTheCount() {
    if (emulatesATalon()) {
      return Optional.of(CANCODER.can(CANCODER_ID).offset(Rotations.of(0)));
    }
    return Optional.of(DATA_PORT.offset(Rotations.of(0)));
  }

  @Override
  public Optional<MechanismWiring.EncoderWiring> encoderTheLoopRunsOn() {
    if (emulatesATalon()) {
      return Optional.of(CANCODER.can(CANCODER_ID).offset(Rotations.of(0)).remote());
    }
    return Optional.of(DATA_PORT.offset(Rotations.of(0)).closesLoop());
  }

  @Override
  public Optional<ControllerPins.Limit> limitReadElsewhere() {
    if (emulatesATalon()) {
      return Optional.of(new ControllerPins.OnCandi(CANDI_ID, 1));
    }
    return Optional.empty();
  }

  @Override
  public Rig powerOn(RobotDefinition robot, Joint joint, ControllerPins pins, double poweredOnAt) {
    return new SimRig(new JointIOSim(robot.math(), joint, pins), poweredOnAt);
  }

  @Override
  public Optional<String> lacks(Ability ability) {
    return switch (ability) {
      case RESTART_A_FOLLOWER -> Optional.of("the simulator restarts only a joint's leader");
      case LOSE_A_FRAME -> Optional.of("the simulator's controller never loses a single frame");
      case SHOW_A_FOLLOWERS_OUTPUT ->
          Optional.of("the simulator models a joint's motors as one motor");
      case SHOW_THE_SUPPLY_LIMIT -> Optional.of("the simulator does not model the supply limit");
      case RESTART_THE_LEADER,
          SILENCE_THE_LEADER,
          SILENCE_A_FOLLOWER,
          INTEGRATE_IN_TIME,
          TURN_THE_MOTOR_APART_FROM_ITS_LOOPS_ENCODER ->
          Optional.empty();
    };
  }

  @Override
  public Optional<String> knownDifference(JointClause clause) {
    return switch (clause) {
      case J19 ->
          Optional.of(
              "the simulator takes a retune's whole control, its soft limits and neutral mode too,"
                  + " where a TalonFX and a SPARK are sent only gains and current limits");
      case J27 ->
          Optional.of(
              "the simulator reads as not connected in the first loop of a silence, where a real"
                  + " driver waits half a second");
      default -> Optional.empty();
    };
  }

  /** One simulated joint, held where the suite puts it. */
  private static final class SimRig implements Rig {
    private final JointIOSim sim;
    private boolean forwardTripped;
    private boolean reverseTripped;

    /** Where the suite last put the joint, and how fast it has it moving. */
    private double position;

    private double velocity;
    private int steps;

    SimRig(JointIOSim sim, double poweredOnAt) {
      this.sim = sim;
      sim.powerOnAt(poweredOnAt);
      sim.readsLimits(() -> forwardTripped, () -> reverseTripped);
      position = poweredOnAt;
      sim.jam(true);
    }

    @Override
    public JointIO io() {
      return sim;
    }

    /**
     * Runs the simulated controller and joint for a loop. A joint the suite has moving is put back
     * where the suite has it afterwards, at its speed: the suite decides where the joint is, not
     * the motor.
     */
    @Override
    public void step() {
      sim.step(RobotLoop.PERIOD_SECONDS, 0);
      steps++;
      if (velocity != 0) {
        sim.carry(position, velocity);
      }
    }

    @Override
    public double deviceSeconds() {
      return steps * RobotLoop.PERIOD_SECONDS;
    }

    /** Still, the joint is jammed where it is, so its motor cannot move it. */
    @Override
    public void jointIs(double position) {
      this.position = position;
      velocity = 0;
      sim.jam(false);
      sim.carry(position, 0);
      sim.jam(true);
    }

    @Override
    public void jointMoves(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
      sim.jam(false);
      sim.carry(position, velocity);
    }

    @Override
    public void encoderSlips(double off) {
      sim.encoder().slip(off);
    }

    @Override
    public void limit(SensorWiring.Direction direction, boolean tripped) {
      if (direction == SensorWiring.Direction.FORWARD) {
        forwardTripped = tripped;
      } else {
        reverseTripped = tripped;
      }
    }

    @Override
    public void restart(int device) {
      requireTheLeader(device);
      sim.restart();
    }

    @Override
    public void loseAFrame() {
      throw new UnsupportedOperationException("the simulator's controller never loses a frame");
    }

    @Override
    public void silence(int device, boolean silent) {
      if (device == LEADER) {
        sim.silence(silent);
      } else {
        sim.silenceFollower(device - 1, silent);
      }
    }

    @Override
    public double motorVolts(int device) {
      requireTheLeader(device);
      return sim.appliedVolts();
    }

    @Override
    public MechanismControl.NeutralMode neutralMode() {
      return sim.plan().brakes()
          ? MechanismControl.NeutralMode.BRAKE
          : MechanismControl.NeutralMode.COAST;
    }

    @Override
    public OptionalDouble supplyLimitAmps() {
      return OptionalDouble.empty();
    }

    @Override
    public double positionKi() {
      throw new UnsupportedOperationException(
          "the simulator's controller adds its integral up in time");
    }

    @Override
    public OptionalDouble speedKi() {
      return OptionalDouble.empty();
    }

    @Override
    public void close() {
      // Nothing is held: the simulated joint is this object's alone.
    }

    private static void requireTheLeader(int device) {
      if (device != LEADER) {
        throw new UnsupportedOperationException("the simulator models only a joint's leader");
      }
    }
  }
}

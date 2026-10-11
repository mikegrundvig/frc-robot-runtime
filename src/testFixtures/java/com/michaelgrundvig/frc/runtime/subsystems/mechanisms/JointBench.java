package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

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
 * A test bench for one kind of joint driver: it powers a driver on against a simulated device, and
 * lets the conformance suite ({@link JointIOConformance}) move the joint, trip its switches and
 * fail its devices, and see what the device does. The suite knows only {@link JointIO} and this, so
 * everything that belongs to one vendor is in that vendor's bench.
 *
 * <p>To check a new driver, write its bench and a test class of a few lines:
 *
 * <pre>{@code
 * class MyJointConformanceTest extends JointIOConformance {
 *   MyJointConformanceTest() {
 *     super(new MyJointBench());
 *   }
 * }
 * }</pre>
 *
 * <p><b>The joint on the bench</b> is a real place, which the suite moves ({@link Rig#jointIs}).
 * The device reads it as its own sensors would. A driver read by its motor's count alone cannot
 * know where the joint was powered on, so it starts by reading where the definition rests the
 * joint, and from then on its reading moves as far as the joint does.
 */
public interface JointBench {
  /** What the bench is called in a test's message, such as "TalonFX". */
  String name();

  /** The motor the bench's controllers drive. The suite gears it to each of its joints. */
  Motor motor();

  /**
   * One of the bench's motor controllers, as a joint's block wires it. The suite lists them on a
   * joint in order, so the first it asks for leads and the rest follow.
   *
   * <p>The suite wires a joint just before it powers it on. A bench may hand out other controllers
   * after each power-on, where its simulated devices cannot be used twice.
   *
   * @param index which, from 0. A bench has at least two.
   */
  MechanismWiring.Device controller(int index);

  /**
   * An absolute encoder on the joint's own axle that the leading controller reads beside its
   * motor's count, reading zero with the joint at zero. Empty where the device reads none so.
   */
  Optional<MechanismWiring.EncoderWiring> encoderBesideTheCount();

  /**
   * The same encoder, with the leading controller's loop running on it. Empty where the device
   * cannot run its loop on one.
   */
  Optional<MechanismWiring.EncoderWiring> encoderTheLoopRunsOn();

  /**
   * A limit that the leading controller reads from another device, not from its own pin: the input
   * the suite trips with {@link Rig#limit}. Empty where the device can read none so.
   */
  Optional<ControllerPins.Limit> limitReadElsewhere();

  /**
   * Powers a driver on for a joint, with the joint somewhere.
   *
   * @param robot the robot the joint is on
   * @param joint the joint, wired to this bench's controllers and encoders
   * @param pins what is on the leading controller's limit inputs
   * @param poweredOnAt where the joint really is as its devices power on, in its units
   */
  Rig powerOn(RobotDefinition robot, Joint joint, ControllerPins pins, double poweredOnAt);

  /**
   * Why the bench cannot do something the suite would have it do, or empty if it can. The suite
   * skips what needs it, and the skipped test says why.
   */
  Optional<String> lacks(Ability ability);

  /**
   * Why the bench's driver is known not to keep a clause of the contract, or empty if it keeps it.
   * The suite then checks that the driver still fails the clause, so a difference that is fixed
   * fails here until its entry is taken away.
   *
   * <p>This is for a clause on which the drivers differ and nobody has yet decided which is right.
   * A driver that breaks a clause the contract has always stated is fixed, not listed.
   */
  Optional<String> knownDifference(JointClause clause);

  /** Something the suite asks of a bench that not every simulated device can do. */
  enum Ability {
    /** Restarting the leading controller ({@link Rig#restart}). */
    RESTART_THE_LEADER,
    /** Restarting a following controller. */
    RESTART_A_FOLLOWER,
    /** Losing one frame of the leading controller's ({@link Rig#loseAFrame}). */
    LOSE_A_FRAME,
    /** Silencing the leading controller ({@link Rig#silence}). */
    SILENCE_THE_LEADER,
    /** Silencing a following controller. */
    SILENCE_A_FOLLOWER,
    /** Showing the voltage across a following controller's motor ({@link Rig#motorVolts}). */
    SHOW_A_FOLLOWERS_OUTPUT,
    /** Showing the limit on the current drawn from the battery ({@link Rig#supplyLimitAmps}). */
    SHOW_THE_SUPPLY_LIMIT,
    /**
     * Adding a loop's integral up in time, as a motor controller does. A bench that cannot shows
     * the kI its controller took instead ({@link Rig#positionKi}).
     */
    INTEGRATE_IN_TIME,
    /**
     * Turning a motor apart from the absolute encoder its controller's loop runs on, so that the
     * motor's count beside the encoder can be checked.
     */
    TURN_THE_MOTOR_APART_FROM_ITS_LOOPS_ENCODER
  }

  /** One driver, powered on against its simulated devices. */
  interface Rig extends AutoCloseable {
    /** The leading controller's number among the joint's devices. Followers come after it. */
    int LEADER = 0;

    /** The driver under test. */
    JointIO io();

    /**
     * Lets one loop of the robot's time pass, 20 ms, the devices running through it. The suite
     * reads the driver's inputs after each.
     */
    void step();

    /**
     * How long the devices have run since power on, in seconds: what a device's integral has
     * counted. It is the robot's time where the simulated device runs on that, and the wall clock's
     * where it runs on its own.
     */
    double deviceSeconds();

    /** Puts the joint somewhere, still, its motor and its encoder with it. */
    void jointIs(double position);

    /**
     * Puts the joint somewhere, moving at a speed, in its units a second. It reads there, at that
     * speed, after every {@link #step}, until it is put somewhere else.
     */
    void jointMoves(double position, double velocity);

    /**
     * Slips the joint's absolute encoder on its shaft: from now on it says the joint is this far
     * from where it is, in the joint's units. Zero puts it back.
     */
    void encoderSlips(double off);

    /** Trips, or releases, the switch on one of the leading controller's limit inputs. */
    void limit(SensorWiring.Direction direction, boolean tripped);

    /**
     * Restarts a device, as a breaker that reset does: it counts its motor from zero, and has
     * whatever a restart leaves it of its settings.
     *
     * @param device which: {@link #LEADER}, or a follower, from 1 in block order
     */
    void restart(int device);

    /**
     * Loses the leading controller's next frame: the next read of the driver's inputs gets none.
     */
    void loseAFrame();

    /**
     * Takes a device off the CAN bus, or brings it back.
     *
     * @param device which: {@link #LEADER}, or a follower, from 1 in block order
     */
    void silence(int device, boolean silent);

    /**
     * The voltage across a device's motor now, as the device itself has it, positive when it drives
     * the joint forward.
     *
     * @param device which: {@link #LEADER}, or a follower, from 1 in block order
     */
    double motorVolts(int device);

    /** What the leading controller does in neutral, as the device itself has it. */
    MechanismControl.NeutralMode neutralMode();

    /**
     * The leading controller's limit on the current it draws from the battery, in amps, as the
     * device itself has it.
     */
    OptionalDouble supplyLimitAmps();

    /**
     * The kI of the leading controller's position loop, in volts for each of the joint's units and
     * each second, as the device itself has it. Asked only of a bench that cannot {@link
     * Ability#INTEGRATE_IN_TIME}.
     */
    double positionKi();

    /**
     * The kI the leading controller holds for the loop its speed requests run on, as the device
     * itself has it, in the device's own units. Empty where the bench's device holds no gains of
     * its own for speeds that can be read back.
     */
    OptionalDouble speedKi();

    /**
     * Leaves the devices as the next power-on needs them: neutral, and let go where they can be.
     */
    @Override
    void close();
  }
}

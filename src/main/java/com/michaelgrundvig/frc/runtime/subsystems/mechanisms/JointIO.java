package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismControl;
import org.littletonrobotics.junction.AutoLog;

/**
 * One driven joint's door to hardware: its motor controllers on a robot ({@link JointIOTalon},
 * {@link JointIOSpark}), the simulator's model of it ({@code
 * com.michaelgrundvig.frc.runtime.sim.JointIOSim}), or nothing, in replay, where its inputs come
 * from the log. Everything is in the joint's own units, radians or meters, as the robot's
 * definition has them; the hardware's units stay behind the door.
 *
 * <p>Each request holds until the next: the joint's mechanism ({@link JointMechanism}) makes one
 * every loop, so a loop that makes none is a bug, not a pause.
 *
 * <h2>The contract</h2>
 *
 * <p>What every driver keeps, clause by clause. The conformance suite ({@code JointIOConformance},
 * in the test fixtures) checks each clause against every driver here, on its vendor's simulated
 * device, and a driver for another motor controller is checked by giving the suite a bench for it
 * ({@code JointBench}). A clause that says "where the device has one" does not bind a device that
 * has none. Where a driver here is known to differ from a clause, the clause says so. Those are
 * open questions, which the suite keeps count of, and not leave to differ.
 *
 * <p><b>Units and direction</b>
 *
 * <ul>
 *   <li><b>J1.</b> Every position is in the joint's own units, radians for a revolute joint and
 *       meters for a prismatic one, and every velocity is in those units a second. The gearing, the
 *       drum and the hardware's units stay behind the driver.
 *   <li><b>J2.</b> Forward is the joint's forward, as the definition has it, however the motor is
 *       mounted. A positive voltage or duty cycle drives the joint toward greater positions, and a
 *       joint moving that way reads a positive velocity.
 * </ul>
 *
 * <p><b>Requests</b>
 *
 * <ul>
 *   <li><b>J3.</b> A request holds until the next one is made. The driver does not let one lapse.
 *   <li><b>J4.</b> Until it is first asked anything, the motors are neutral.
 *   <li><b>J5.</b> {@link #setPosition} holds the joint at a point of a profile on the motor
 *       controller's loop, which works on where the controller reads the joint. The volts across
 *       the motor are kP times how far the joint reads from the position, plus kI times that error
 *       added up over time, plus kD times how far its speed reads from the velocity, plus the
 *       plan's feedforward for that position and velocity ({@code ControllerPlan#feedforward}: kS,
 *       kV and gravity), plus the feedforward volts given.
 *   <li><b>J6.</b> {@link #setVelocity} runs the joint at a speed on the controller's loop: kP
 *       times how far its speed reads from the velocity, plus the plan's feedforward for that
 *       velocity where the joint is.
 *   <li><b>J7.</b> kI is the position loop's alone, on every driver: a velocity request adds
 *       nothing up, however long its error lasts. A controller whose gains sit in slots runs its
 *       speeds on a slot with no kI, and a retune keeps that slot's other gains the same as the
 *       position slot's.
 *   <li><b>J8.</b> {@link #setVoltage} puts that voltage across the motors, with its sign, up to
 *       the battery's and to what the current limits allow.
 *   <li><b>J9.</b> {@link #setDutyCycle} puts that share of the battery's voltage across the
 *       motors, with its sign.
 *   <li><b>J10.</b> {@link #stop} puts no voltage across the motors, whatever was asked before, and
 *       they brake or coast as the control's neutral mode says.
 *   <li><b>J11.</b> {@link #home} puts a voltage across the motors as {@link #setVoltage} does,
 *       except that the soft limits do not stop it.
 *   <li><b>J12.</b> {@link #resetPosition} tells the controller where the joint is. The joint reads
 *       there within a loop or two, and from then on its reading moves as far as the joint does.
 *       Nothing that was asked changes.
 * </ul>
 *
 * <p><b>Limits</b>
 *
 * <ul>
 *   <li><b>J13.</b> Where the control has soft limits, they are in force from startup. With the
 *       joint at or past one, no request but {@link #home} drives it further past: the controller
 *       puts nothing across the motors, or drives the joint back. A request back toward the range
 *       goes through.
 *   <li><b>J14.</b> After a homing push the soft limits are in force again with the first request
 *       that is not one, whether or not {@link #resetPosition} came between.
 *   <li><b>J15.</b> While a switch on one of the leading controller's limit inputs is tripped, and
 *       the wiring says the controller stops on it, no request drives the joint toward it, {@link
 *       #home} included, and a request away from it goes through. The forward input's switch stops
 *       forward, and the reverse input's backward. A switch the wiring says not to stop on, such as
 *       a beam, never stops the motor.
 *   <li><b>J16.</b> {@link JointIOInputs#forwardLimit} and {@link JointIOInputs#reverseLimit} are
 *       true while the switch on that input is tripped, by the switch's polarity: a normally open
 *       switch as it closes, a normally closed one as it opens. Each is false with nothing on its
 *       input.
 *   <li><b>J17.</b> Where the device can read a limit from another device, as a TalonFX reads a
 *       CANdi's, that limit stops the motor as J15 says, and the inputs' limits stay false: the
 *       robot reads that switch as a sensor of its own.
 * </ul>
 *
 * <p><b>Retune</b>
 *
 * <ul>
 *   <li><b>J18.</b> {@link #retune} puts the control's gains and current limits in force from the
 *       next request: kP, kI, kD, the feedforward's kS, the limit on the current through the motor
 *       and, where the device has one, the limit on the current from the battery. A value tuned to
 *       zero is in force as zero. It does not wait for the device.
 *   <li><b>J19.</b> A retune changes nothing else: the soft limits and the neutral mode stay as
 *       they were at startup. Known difference: the simulator takes the whole control.
 * </ul>
 *
 * <p><b>Readings</b>
 *
 * <ul>
 *   <li><b>J20.</b> A joint with no absolute encoder reads where its definition rests it at
 *       startup, wherever it is, and from then on its position moves as far as the joint does.
 *   <li><b>J21.</b> {@link JointIOInputs#appliedVolts} is the voltage across the leading motor with
 *       its sign: positive drives the joint forward.
 *   <li><b>J22.</b> {@link JointIOInputs#statorCurrentAmps} is a size: never negative, whichever
 *       way the current flows.
 *   <li><b>J23.</b> {@link JointIOInputs#supplyCurrentAmps} is a size, on every driver: never
 *       negative, even while the motor sends current back to the battery. Which way the current
 *       flows is read from the applied voltage and the velocity, not from a current's sign.
 *   <li><b>J24.</b> {@link JointIOInputs#unconfigured} is empty when every device took its settings
 *       at startup.
 *   <li><b>J25.</b> {@link JointIOInputs#faults} has one number for each device, in order: the
 *       leader, each follower in block order, then the CANcoder. 0 is no fault. {@link
 *       JointIOInputs#followersConnected} and {@link JointIOInputs#followersRestarted} have one
 *       entry for each follower, in block order.
 * </ul>
 *
 * <p><b>Lost frames and silence</b>
 *
 * <ul>
 *   <li><b>J26.</b> {@link JointIOInputs#connected} is false once the leading controller has been
 *       silent for half a second, and true again with the first frame that comes.
 *   <li><b>J27.</b> It stays true through a shorter silence: one lost frame is a glitch. Known
 *       difference: the simulator reads as not connected in the first loop of a silence.
 *   <li><b>J28.</b> While frames do not come, every reading stands as it was last read. A lost
 *       frame never reads as zero, or as something that is not a number.
 *   <li><b>J29.</b> {@link JointIOInputs#fresh} is false in a loop the leading controller's
 *       position and velocity did not come, and true in a loop they did. Known difference: a
 *       TalonFX's driver always says fresh.
 *   <li><b>J30.</b> The first request made after a silence is in force.
 * </ul>
 *
 * <p><b>Restarts</b>
 *
 * <ul>
 *   <li><b>J31.</b> {@link JointIOInputs#restarted} is true in exactly one loop after the leading
 *       controller restarts. The controller counts its motor from zero again, so the joint reads
 *       from zero until it is told where it is, or as J37 says.
 *   <li><b>J32.</b> Whatever a restart does to the device, by the next request everything a retune
 *       put in force is in force again, and so are the soft limits.
 * </ul>
 *
 * <p><b>Followers</b>
 *
 * <ul>
 *   <li><b>J33.</b> Each following controller drives the joint the way its leader does, whichever
 *       way round its motor is mounted.
 *   <li><b>J34.</b> A follower's entry in {@link JointIOInputs#followersRestarted} is true in
 *       exactly one loop after it restarts, the leader's {@code restarted} is not, and the follower
 *       follows again.
 *   <li><b>J35.</b> A follower's entry in {@link JointIOInputs#followersConnected} is false once it
 *       has been silent for half a second, as read every {@link #HEALTH_LOOPS} loops, while {@code
 *       connected} stays true. It is true again when the follower answers.
 * </ul>
 *
 * <p><b>Absolute encoders</b>
 *
 * <ul>
 *   <li><b>J36.</b> {@link JointIOInputs#absolutePosition} is where an absolute encoder that the
 *       leading controller reads beside its motor's count says the joint is, each loop, in the
 *       joint's units. It is not a number without such an encoder, or in a loop without a good
 *       reading of it.
 *   <li><b>J37.</b> Such an encoder's first good reading tells the motor's count where the joint is
 *       at startup, and again after the leading controller restarts. Between those the position is
 *       the motor's count, so an encoder that slips shows as the two drifting apart.
 *   <li><b>J38.</b> Where the controller's loop runs on an absolute encoder, the position is the
 *       encoder's reading every loop. {@link JointIOInputs#motorPosition} is the motor's own count
 *       where the driver reads it beside the encoder, and not a number otherwise.
 * </ul>
 *
 * <p><b>A joint that turns freely</b>
 *
 * <ul>
 *   <li><b>J39.</b> A joint with no range reads on past a whole turn either way, without wrapping.
 *   <li><b>J40.</b> A position is taken as its number says, on every driver: the controller drives
 *       toward it the way the number lies from where the joint reads, even when the other way round
 *       is shorter, and no controller's loop is set to wrap. The joint's profile chooses the turn
 *       ({@code Travel#goalNear}), and the driver is told points along the way it chose.
 * </ul>
 *
 * <p>Two inputs are a CANcoder's alone, and are checked with the CTRE driver ({@code
 * JointIOTalonTest}), not by the suite: {@link JointIOInputs#magnetBad} and {@link
 * JointIOInputs#encoderRestarted}.
 */
public interface JointIO {
  @AutoLog
  class JointIOInputs {
    /** Whether the motor controllers answered lately: false after half a second of silence. */
    public boolean connected = false;

    /**
     * Whether its position and speed are this loop's: false the loop a status frame of the leading
     * motor controller's doesn't come, when they're the last it read. The interlocks reading the
     * joint hold at once on a reading that isn't fresh, where {@link #connected} waits half a
     * second of them to call the controller silent. A SPARK says so; a TalonFX, whose last reading
     * Phoenix keeps, and the simulator, read fresh.
     */
    public boolean fresh = true;

    /** Where the joint is, in radians for a revolute joint or meters for a prismatic one. */
    public double position = 0.0;

    /** How fast it moves, in radians or meters a second. */
    public double velocity = 0.0;

    /**
     * Where the absolute encoder its leading motor controller reads says the joint is, each loop,
     * in its units: a SPARK's data port's, or a CANcoder's that a TalonFX reads its rotor beside.
     * NaN without one, or this loop without a good reading; an encoder only the robot reads has
     * inputs of its own ({@link AbsoluteEncoderIO}).
     */
    public double absolutePosition = Double.NaN;

    /**
     * Where its motor's own count says the joint is, where {@link #position} is an absolute
     * encoder's word instead (a SPARK's loop closed on its data port): NaN otherwise, its position
     * being its motor's count.
     */
    public double motorPosition = Double.NaN;

    /**
     * Whether the CANcoder its leading motor controller reads says its magnet is too close or too
     * far (red): what it reads then isn't trusted, and the joint isn't seeded from it.
     */
    public boolean magnetBad = false;

    /** The voltage across the leading motor. */
    public double appliedVolts = 0.0;

    /** The current through the leading motor, either way: what its stator limit holds. */
    public double statorCurrentAmps = 0.0;

    /**
     * The current between the leading motor's controller and the battery, either way: what it draws
     * while it drives, and what it sends back while it brakes. Never negative.
     */
    public double supplyCurrentAmps = 0.0;

    /**
     * Whether its leading motor controller's forward limit input reads tripped, the polarity of the
     * switch on it told to the controller: false with nothing on it.
     */
    public boolean forwardLimit = false;

    /** The same, its reverse limit input. */
    public boolean reverseLimit = false;

    /**
     * The joint's devices that didn't take their configuration at startup, such as "motor 32", or
     * empty: they run on their defaults, without the configured ratios, gains or current limits.
     */
    public String unconfigured = "";

    /**
     * Whether its leading motor controller restarted since the last loop: a breaker that reset, a
     * power connector that came loose. A restarted controller counts its motor from 0, or from the
     * absolute encoder it reads, so its joint finds where it is again ({@link JointMechanism}). A
     * TalonFX says so in the frame it sends as it starts; a SPARK, in its sticky warnings.
     */
    public boolean restarted = false;

    /** Whether each following motor controller restarted since the last loop, in block order. */
    public boolean[] followersRestarted = new boolean[0];

    /** Whether the CANcoder its leading motor controller reads restarted since the last loop. */
    public boolean encoderRestarted = false;

    /**
     * Whether each following motor controller answered lately, in block order: false after half a
     * second of silence. Read a few times a second ({@link JointIO#HEALTH_LOOPS}).
     */
    public boolean[] followersConnected = new boolean[0];

    /**
     * Each device's active faults, as its vendor numbers them: the leader, each follower in block
     * order, then its CANcoder. A TalonFX's or a CANcoder's fault field; a SPARK's fault bits, its
     * warnings' sixteen bits above them. 0 for none. Read a few times a second ({@link
     * JointIO#HEALTH_LOOPS}), the last reading standing between.
     */
    public int[] faults = new int[0];
  }

  /**
   * Its devices' health, their followers' answering and their faults, is read once in this many
   * loops, ten times a second: what a log needs to show a follower off the bus, and a few frames a
   * second's worth of reading.
   */
  int HEALTH_LOOPS = 5;

  default void updateInputs(JointIOInputs inputs) {}

  /**
   * Holds the joint at a point of a profile: where it should be, how fast it should be moving
   * there, and the volts the program adds to the motor controller's own feedforward (its kV, kS and
   * gravity), such as the profile's acceleration times kA.
   */
  default void setPosition(double position, double velocity, double feedforwardVolts) {}

  /** Runs the joint at a speed, in its units a second, on the motor controller's velocity loop. */
  default void setVelocity(double velocity) {}

  /** A voltage across the motors. */
  default void setVoltage(double volts) {}

  /** A share of the battery's voltage, from -1 to 1. */
  default void setDutyCycle(double share) {}

  /** Neutral: the motors brake or coast, as the joint's control says. */
  default void stop() {}

  /**
   * A voltage across the motors that ignores the soft limits: homing pushes the joint into a hard
   * stop, past where its soft limits would stop it, since until it's homed they're in the wrong
   * place.
   */
  default void home(double volts) {}

  /** Tells the motor controller where the joint is now: homing found it. */
  default void resetPosition(double position) {}

  /**
   * Takes the pit's tuned gains and current limits, as the motor controller's own loop runs them:
   * its kP, kI, kD and kS, and its current limits. Without waiting: the loop goes on.
   *
   * <p>Every one of them is in force from the next request on, wherever it runs. A TalonFX holds
   * them all in its two slots, one for positions and one for speeds. A SPARK holds its kP, its kI
   * and the limit on the current through its motor; its kD and kS the robot works out each loop,
   * and it has no limit on what it draws from the battery.
   */
  default void retune(MechanismControl control) {}
}

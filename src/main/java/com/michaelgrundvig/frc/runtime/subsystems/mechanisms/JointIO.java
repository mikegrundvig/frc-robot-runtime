package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismControl;
import org.littletonrobotics.junction.AutoLog;

/**
 * One driven joint's door to hardware: its motor controllers on a robot ({@link JointIOTalon}), the
 * simulator's model of it ({@code com.michaelgrundvig.frc.runtime.sim.JointIOSim}), or nothing, in
 * replay, where its inputs come from the log. Everything is in the joint's own units, radians or
 * meters, as the robot's definition has them; the hardware's units stay behind the door.
 *
 * <p>Each request holds until the next: the joint's mechanism ({@link JointMechanism}) makes one
 * every loop, so a loop that makes none is a bug, not a pause.
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

    /** The current the leading motor draws from the battery. */
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
   * them all in its slot. A SPARK holds its kP, its kI and the limit on the current through its
   * motor; its kD and kS the robot works out each loop, and it has no limit on what it draws from
   * the battery.
   */
  default void retune(MechanismControl control) {}
}

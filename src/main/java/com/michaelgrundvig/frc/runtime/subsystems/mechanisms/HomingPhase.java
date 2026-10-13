package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism.State;
import com.michaelgrundvig.frc.world.record.JointState;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalLong;

/**
 * Where a joint stands in finding out where it is: one of the phases below, each with what it needs
 * to carry on. {@link JointHoming} moves a joint from one phase to the next.
 *
 * <p>There are more phases than the four {@link State}s a joint logs, because several of them look
 * the same from outside. A joint that is {@code FAULTED}, for instance, may be faulted for good, or
 * only until its absolute encoder answers or is trusted again. Each phase says which state it shows
 * as, and answers the few questions the joint's mechanism asks of it.
 */
sealed interface HomingPhase {
  /** The state the joint's mechanism logs for this phase. */
  State state();

  /**
   * The phase as {@code State} logs it: where the joint stands, with what a homing joint is pushed
   * toward and for how long at most, or which of the reasons a faulted one is faulted for. A phase
   * that has more to say than its standing is made with what it logs, so that asking makes nothing.
   */
  JointState recorded();

  /** Why moves on the joint are refused, if it is faulted. */
  default Optional<String> fault() {
    return Optional.empty();
  }

  /** Whether the joint is waiting for its absolute encoder's next reading to say where it is. */
  default boolean awaitsEncoder() {
    return false;
  }

  /**
   * Where the joint holds by its motor's own count while its absolute encoder cannot be used, if it
   * holds at all. A joint in any other phase is not held this way.
   */
  default OptionalDouble heldOnMotor() {
    return OptionalDouble.empty();
  }

  /** The joint knows where it is, or only runs at outputs and has nothing to find. */
  record Ready() implements HomingPhase {
    @Override
    public State state() {
      return State.READY;
    }

    @Override
    public JointState recorded() {
      return JointState.READY;
    }
  }

  /**
   * The joint found where it is in this very loop. It is ready, but its motor controller reports
   * its new count only from the next loop, so nothing drives the joint until then.
   */
  record JustFound() implements HomingPhase {
    @Override
    public State state() {
      return State.READY;
    }

    @Override
    public JointState recorded() {
      return JointState.READY;
    }
  }

  /**
   * The joint does not know where it is, and homes by its recipe when the robot is next enabled.
   */
  record Unhomed() implements HomingPhase {
    @Override
    public State state() {
      return State.UNHOMED;
    }

    @Override
    public JointState recorded() {
      return JointState.UNHOMED;
    }
  }

  /**
   * The joint is waiting for its absolute encoder's reading to say where it is.
   *
   * @param since when it began to wait, in nanoseconds
   * @param heldOnMotor where it holds by its motor's count meanwhile. Present only for a joint that
   *     was running when the encoder its loop closes on was lost, which holds on its motor
   *     controller's own loop so that it does not go limp.
   */
  record AwaitingEncoder(long since, OptionalDouble heldOnMotor) implements HomingPhase {
    @Override
    public State state() {
      return State.UNHOMED;
    }

    @Override
    public JointState recorded() {
      return JointState.UNHOMED;
    }

    @Override
    public boolean awaitsEncoder() {
      return true;
    }
  }

  /**
   * The joint is being pushed into its hard stop.
   *
   * @param recorded what it logs: which stop it is pushed toward, and for how long at most
   * @param since when the push began, in nanoseconds
   * @param stillSince since when the joint has been still, in nanoseconds, or empty while it moves
   */
  record PushingToStop(JointState recorded, long since, OptionalLong stillSince)
      implements HomingPhase {
    @Override
    public State state() {
      return State.HOMING;
    }
  }

  /** How far a joint that homes to a limit switch has got. */
  sealed interface Approach {
    /** The switch has not been read yet. Its first reading decides which way the joint sets off. */
    record SwitchNotYetRead() implements Approach {}

    /**
     * The joint started on its switch, somewhere past the point where the switch trips, and is
     * backing off until the switch clears.
     */
    record BackingOff() implements Approach {}

    /**
     * The joint is driven toward its switch and is home when the switch trips.
     *
     * @param lastRead where the joint read in the loop before. The switch trips somewhere between
     *     there and where the joint reads in the loop it trips.
     */
    record HeadingIn(double lastRead) implements Approach {}
  }

  /**
   * The joint is being driven onto its limit switch.
   *
   * @param recorded what it logs: which switch it is driven toward, and for how long at most
   * @param since when it set off, in nanoseconds
   * @param approach how far it has got
   * @param onto the switch it is driven onto. A joint is in this phase only with a switch to seek,
   *     which is what lets every loop of the push be timed by reading it.
   */
  record SeekingSwitch(JointState recorded, long since, Approach approach, Sensor.Switch onto)
      implements HomingPhase {
    @Override
    public State state() {
      return State.HOMING;
    }
  }

  /**
   * The joint's moves are refused until it homes again.
   *
   * @param recorded what it logs: which of the reasons it is faulted for
   * @param why what is wrong and what to do about it
   * @param awaitsEncoder whether the joint's absolute encoder is still to say where the joint is.
   *     Its reading then tells the joint's motor controller where the joint is, though the joint
   *     stays faulted for whatever else is wrong.
   */
  record Faulted(JointState recorded, String why, boolean awaitsEncoder) implements HomingPhase {
    @Override
    public State state() {
      return State.FAULTED;
    }

    @Override
    public Optional<String> fault() {
      return Optional.of(why);
    }
  }

  /**
   * The joint's moves are refused because the absolute encoder that says where it is gives no
   * reading. The encoder's next reading makes the joint ready again.
   *
   * @param recorded what it logs: which of the reasons it is faulted for
   * @param why what is wrong
   * @param since when the joint began to wait for the reading, in nanoseconds
   * @param heldOnMotor where it holds by its motor's count meanwhile, for a joint whose loop closes
   *     on the encoder
   */
  record EncoderSilent(JointState recorded, String why, long since, OptionalDouble heldOnMotor)
      implements HomingPhase {
    @Override
    public State state() {
      return State.FAULTED;
    }

    @Override
    public Optional<String> fault() {
      return Optional.of(why);
    }

    @Override
    public boolean awaitsEncoder() {
      return true;
    }
  }

  /**
   * The joint's moves are refused because its absolute encoder is not trusted: a CANcoder says its
   * magnet is bad, or the encoder and the motor's count have come apart, so one of them slipped.
   * The joint is ready again once the encoder is trusted again.
   *
   * @param recorded what it logs: which of the reasons it is faulted for
   * @param why what is wrong and what to do about it
   * @param heldOnMotor where it holds by its motor's count meanwhile, for a joint whose motor
   *     controller's loop runs on the motor's count and so can still hold
   */
  record EncoderDistrusted(JointState recorded, String why, OptionalDouble heldOnMotor)
      implements HomingPhase {
    @Override
    public State state() {
      return State.FAULTED;
    }

    @Override
    public Optional<String> fault() {
      return Optional.of(why);
    }
  }
}

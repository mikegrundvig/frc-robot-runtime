package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.record.JointState;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * Decides each loop whether a joint's absolute encoder can be trusted, and tells the joint's homing
 * when it cannot.
 *
 * <p>An encoder is not trusted for either of two reasons:
 *
 * <ul>
 *   <li>It is a CANcoder that says its magnet is too close or too far.
 *   <li>It has been apart from the motor's count for a quarter of a second ({@link EncoderSlip}),
 *       so one of the two slipped.
 * </ul>
 *
 * <p>Either way, where the joint is, is no longer known. The joint is faulted, and holds where its
 * motor's count says, if its motor controller's loop runs on that count, or is stopped if it does
 * not. It stays so until the encoder is trusted again: its magnet good, and the encoder and the
 * motor's count agreeing, where the two can be compared. Then the joint is ready, where it is.
 */
final class EncoderTrust {
  /** What a loop's check found. */
  enum Change {
    /** Nothing the joint's mechanism has to act on. */
    NONE,
    /**
     * The encoder is trusted again as of this loop. The joint holds where it is now, and not where
     * it was held while the encoder was distrusted.
     */
    TRUSTED_AGAIN
  }

  /**
   * What is fixed about how a joint's absolute encoder is wired.
   *
   * @param loopRunsOnMotor whether the motor controller's loop runs on the motor's own count, so
   *     that the joint can still hold a position while its encoder is not trusted. That is not so
   *     for a TalonFX whose loop runs on its CANcoder, or a SPARK whose loop runs on its data port.
   * @param readBesideMotor whether the encoder is read beside the motor's count, so that the two
   *     can be compared: an encoder the robot reads, one on a SPARK's data port, or a CANcoder that
   *     a TalonFX is only seeded from. A CANcoder that a TalonFX's loop runs on is not, because
   *     then the controller's count is the encoder's.
   * @param cancoder the joint's CANcoder as a fault names it ("CANcoder 34"), or empty for none
   */
  record Wiring(boolean loopRunsOnMotor, boolean readBesideMotor, String cancoder) {
    /** How a joint's absolute encoder is wired, from the joint's block. */
    static Wiring of(Driven.AbsoluteEncoder encoder) {
      boolean seedsATalonOnly =
          encoder
              .cancoder()
              .filter(on -> on.feedback() == MechanismWiring.Feedback.ROTOR)
              .isPresent();
      boolean readBesideMotor =
          encoder.port().isPresent() || encoder.dataPort().isPresent() || seedsATalonOnly;
      String cancoder = encoder.cancoder().map(on -> "CANcoder " + on.id()).orElse("");
      return new Wiring(!encoder.runsControllerLoop(), readBesideMotor, cancoder);
    }
  }

  private final String label;
  private final Travel travel;
  private final Wiring wiring;
  private final EncoderSlip slip;
  private final JointReadings readings;
  private final JointHoming homing;

  EncoderTrust(
      String label,
      Travel travel,
      Wiring wiring,
      EncoderSlip slip,
      JointReadings readings,
      JointHoming homing) {
    this.label = label;
    this.travel = travel;
    this.wiring = wiring;
    this.slip = slip;
    this.readings = readings;
    this.homing = homing;
  }

  /** Checks the encoder for this loop, and tells the joint's homing what it finds. */
  Change check(long now, boolean enabled) {
    double encoder = readings.positionByAnyEncoderOrNaN();
    double motor = readings.positionByMotor();
    // The two are compared only while the joint knows where it is, and not in the loop it found
    // out, when the motor's count has not yet caught up with what the joint was told.
    boolean comparable = homing.state() == JointMechanism.State.READY && !homing.justFound();
    Optional<String> slipped =
        slip.slipped(now, comparable ? encoder : Double.NaN, motor, readings.velocity());

    if (readings.magnetIsBad()) {
      homing.distrust(
          JointState.Fault.ENCODER_MAGNET_BAD, magnetIsBad(), heldOnMotor(motor, enabled));
      return Change.NONE;
    }
    if (slipped.isPresent()) {
      homing.distrust(
          JointState.Fault.ENCODER_SLIPPED,
          oneOfThemSlipped(slipped.get()),
          heldOnMotor(motor, enabled));
      return Change.NONE;
    }
    if (!homing.distrusted()) {
      return Change.NONE;
    }
    boolean agreeAgain = !wiring.readBesideMotor() || slip.agree(encoder, motor);
    if (agreeAgain) {
      homing.trust();
      return Change.TRUSTED_AGAIN;
    }
    if (!enabled) {
      // Disabled, the joint may be moved by hand. It holds wherever it is when it is enabled.
      homing.stillDistrusted(heldOnMotor(motor, enabled));
    }
    return Change.NONE;
  }

  /**
   * Where the joint holds by its motor's count while its encoder is not trusted: where it comes to
   * a stop from its speed, or just where it is while the robot is disabled. Empty for a joint whose
   * motor controller's loop does not run on the motor's count, which cannot hold and is stopped.
   */
  private OptionalDouble heldOnMotor(double motor, boolean enabled) {
    if (!wiring.loopRunsOnMotor()) {
      return OptionalDouble.empty();
    }
    double velocity = enabled ? readings.velocity() : 0;
    return OptionalDouble.of(travel.stoppingPoint(motor, velocity));
  }

  /** What the joint does until its encoder is trusted again, as a fault begins to say it. */
  private String untilTrusted() {
    if (wiring.loopRunsOnMotor()) {
      return label + " holds where its motor says until ";
    }
    return label + " is stopped until ";
  }

  private String magnetIsBad() {
    return possessive(label)
        + " "
        + wiring.cancoder()
        + " says its magnet is too close or too far, so what it reads isn't trusted: "
        + untilTrusted()
        + "it's good. Check the magnet's gap and that it's centered on the CANcoder";
  }

  private String oneOfThemSlipped(String howFarApart) {
    String thenWhat =
        homing.homesByEncoder()
            ? "home it to tell its motor where it is from the encoder again"
            : "restart the robot code to tell its motor where it is again";
    return possessive(label)
        + " absolute encoder and its motor's count are "
        + howFarApart
        + ": one of them slipped, a magnet loose on its shaft or a belt that skipped, so"
        + " where "
        + label
        + " is isn't known, and "
        + untilTrusted()
        + "they agree. Check both, then "
        + thenWhat;
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.function.BooleanSupplier;

/**
 * Whether one of a module's devices has taken its settings, and sending them again until it does.
 *
 * <p>Each device is sent its whole configuration as the robot starts. One that doesn't confirm (off
 * the bus, at a wrong CAN ID, or on a CANivore that wasn't plugged in yet) is noted, and the module
 * is held still: it runs on its defaults, without its directions, gearing or current limits. Once
 * the device answers, its whole configuration is sent to it again, and again after each time it
 * doesn't confirm, until it does. Then the module drives again.
 *
 * <p>A send waits for the device's answer, up to a quarter second, so while the robot runs it goes
 * to the sender, a thread of its own, and the loop asks each time round whether the answer has
 * come. Phoenix and REVLib are called from that thread and from the loop at once.
 */
final class DeviceSettings {
  /** Where a device's settings stand. */
  enum State {
    /** Not sent yet: a motor's settings wait for its gains, which come with the first configure. */
    NOT_YET_SENT,
    /** The device confirmed its whole configuration. */
    TAKEN,
    /** The device didn't confirm, and is sent it again once it answers. */
    NOT_TAKEN,
    /** Being sent again, on the sender. */
    BEING_SENT
  }

  /** The device, as an alert names it: "drive motor 3", "CANcoder 9". */
  private final String name;

  private final Executor sender;
  private State state = State.NOT_YET_SENT;

  /** The send under way: present exactly while the state is BEING_SENT. */
  private Optional<CompletableFuture<Boolean>> sending = Optional.empty();

  /**
   * @param name the device, as an alert names it
   * @param sender where a send while the robot runs waits for its answer: a thread of its own on a
   *     robot, or the caller itself in a test
   */
  DeviceSettings(String name, Executor sender) {
    this.name = name;
    this.sender = sender;
  }

  String name() {
    return name;
  }

  State state() {
    return state;
  }

  /** Whether the device runs without its whole configuration: not taken yet, or being sent. */
  boolean missing() {
    return state == State.NOT_TAKEN || state == State.BEING_SENT;
  }

  /** Notes how the send at startup went. */
  void sentAtStartup(boolean taken) {
    state = taken ? State.TAKEN : State.NOT_TAKEN;
  }

  /**
   * Whether the device should be sent its settings again now: it didn't take them, nothing is being
   * sent, and it answered this loop.
   */
  boolean wantsSendingAgain(boolean answered) {
    return state == State.NOT_TAKEN && answered;
  }

  /**
   * Sends the settings again, on the sender.
   *
   * @param send one whole send, which waits for the device's answer: true when it confirmed
   */
  void sendAgain(BooleanSupplier send) {
    state = State.BEING_SENT;
    sending =
        Optional.of(
            CompletableFuture.supplyAsync(send::getAsBoolean, sender)
                .exceptionally(failure -> false));
  }

  /**
   * Takes the answer to a send under way, if it has come.
   *
   * @return whether the device took its settings with this answer
   */
  boolean answerCame() {
    if (sending.isEmpty()) {
      return false;
    }
    if (!sending.get().isDone()) {
      return false;
    }
    boolean taken = sending.get().join();
    sending = Optional.empty();
    state = taken ? State.TAKEN : State.NOT_TAKEN;
    return taken;
  }
}

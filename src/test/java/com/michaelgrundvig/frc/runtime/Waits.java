package com.michaelgrundvig.frc.runtime;

import java.time.Duration;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Waits for what happens on its own time: a simulated CTRE device answering, a datagram arriving.
 * Each wait allows the 30 s the end-to-end waits allow, and ends as soon as what it waits for is
 * so, so only a failure takes that long. A failure names what was waited for.
 */
public final class Waits {
  /** The longest a wait takes before the test fails. */
  public static final Duration LIMIT = Duration.ofSeconds(30);

  /** How often a wait reads again. */
  private static final long READ_EVERY_MILLIS = 10;

  private Waits() {}

  /**
   * Reads every few milliseconds until something is so.
   *
   * @param what what is waited for, as the failure says it
   * @param read reads again: a device's inputs, say
   * @param done whether it's so, from what was read
   */
  public static void until(String what, Runnable read, BooleanSupplier done)
      throws InterruptedException {
    until(() -> what, read, done);
  }

  /**
   * The same, the failure's words made as it fails, so they can say what was last read.
   *
   * @param what what is waited for, and what was read
   */
  public static void until(Supplier<String> what, Runnable read, BooleanSupplier done)
      throws InterruptedException {
    long until = System.nanoTime() + LIMIT.toNanos();
    while (System.nanoTime() < until) {
      read.run();
      if (done.getAsBoolean()) {
        return;
      }
      Thread.sleep(READ_EVERY_MILLIS);
    }
    throw new AssertionError("waited " + LIMIT.toSeconds() + " s for " + what.get());
  }
}

package com.michaelgrundvig.frc.runtime.lib.health;

import java.util.OptionalLong;

/**
 * A condition over the session: whether it holds now, how many times it began, how long it held in
 * all, and when it last ended. A frame drop over in a second is still there to see after the match.
 */
public final class Episodes {
  private boolean holds;
  private long sinceNanos;
  private int count;
  private long endedNanos;
  private long heldNanos;
  private boolean everEnded;

  /** Whether it holds as of this time. Once a loop, the time never going back. */
  public void update(boolean holdsNow, long nowNanos) {
    if (holdsNow && !holds) {
      count++;
      sinceNanos = nowNanos;
    }
    if (!holdsNow && holds) {
      heldNanos += nowNanos - sinceNanos;
      endedNanos = nowNanos;
      everEnded = true;
    }
    holds = holdsNow;
  }

  /** Whether it holds now. */
  public boolean holds() {
    return holds;
  }

  /** How many times it began this session. */
  public int count() {
    return count;
  }

  /** How long it held this session, in seconds, up to this time: the current episode included. */
  public double seconds(long nowNanos) {
    long held = heldNanos;
    if (holds) {
      held += nowNanos - sinceNanos;
    }
    return held / 1e9;
  }

  /**
   * What happened this session, in words for the pit: "for 4 s so far" while it holds the first
   * time; "once this session, for 3 s, 2 min ago"; "4 times this session, 12 s in all, the last 40
   * s ago"; "never" if it never held.
   */
  public String describe(long nowNanos) {
    if (count == 0) {
      return "never";
    }
    String total = Durations.said(seconds(nowNanos));
    if (holds && count == 1) {
      return "for " + total + " so far";
    }
    String times = count + " times this session, " + total + " in all";
    if (count == 1) {
      times = "once this session, for " + total;
    }
    if (holds) {
      return times + ", now too";
    }
    String ago = Durations.said((nowNanos - endedNanos) / 1e9);
    if (count == 1) {
      return times + ", " + ago + " ago";
    }
    return times + ", the last " + ago + " ago";
  }

  /** When it last stopped holding; empty if it never has. */
  public OptionalLong lastEndedNanos() {
    if (!everEnded) {
      return OptionalLong.empty();
    }
    return OptionalLong.of(endedNanos);
  }
}

package com.michaelgrundvig.frc.runtime.lib.health;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth.Warning;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import java.util.function.IntConsumer;
import org.junit.jupiter.api.Test;

/**
 * A camera's health, from its frames as a robot loop reads them: a 60 fps camera on Visor read by a
 * 50 Hz loop, with something going wrong in each scenario.
 */
class CameraHealthTest {
  private static final long LOOP_NANOS = 20_000_000L;
  private static final double FRAME_PERIOD_SECONDS = 1.0 / 60;

  /**
   * A camera on Visor as the robot hears it: its frames' numbers, dropped count and detect time.
   */
  private static final class Visor {
    double framesPerLoop = 60 * 0.020;
    double detectMs = 4;
    long sequence = -1;
    long dropped;
    boolean numbered = true;
    // What the newest frame to arrive said: the robot hears nothing of a frame the network lost.
    long heardSequence = -1;
    long heardDropped;
    double owed;
    int droppedNext;
    int lostNext;
    // One frame held back on the network, to arrive in the next loop after newer ones.
    boolean holdOneBack;
    int heldBack;

    /** One loop's frames: those made since the last, less any dropped or lost, as read. */
    void loop(CameraHealth health, long nowNanos) {
      owed += framesPerLoop;
      int made = (int) owed;
      owed -= made;
      int dropping = Math.min(droppedNext, made);
      dropped = (dropped + dropping) & 0xffff_ffffL;
      int sent = made - dropping;
      // Visor's number is 32 bits, and wraps.
      sequence = (sequence + sent) & 0xffff_ffffL;
      // A lost frame is the oldest of the loop's, so the newest still arrives when any does.
      int arrived = Math.max(0, sent - lostNext);
      droppedNext -= dropping;
      lostNext = 0;
      if (arrived > 0) {
        heardSequence = sequence;
        heardDropped = dropped;
      }
      // A frame held back last loop arrives now; one held back now is one before the newest.
      arrived += heldBack;
      heldBack = 0;
      if (holdOneBack && arrived >= 2) {
        holdOneBack = false;
        heldBack = 1;
        arrived--;
      }
      double[] detect = new double[arrived];
      java.util.Arrays.fill(detect, detectMs);
      if (!numbered) {
        health.update(nowNanos, arrived, new double[0]);
        return;
      }
      if (heardSequence < 0) {
        health.update(nowNanos, arrived, detect);
        return;
      }
      health.update(nowNanos, arrived, heardSequence, heardDropped, detect);
    }
  }

  /**
   * A scenario: what goes wrong at which loop, and what the camera's health says after twelve
   * seconds of it (600 loops).
   */
  private record Scenario(
      String name,
      java.util.function.BiConsumer<Visor, Integer> atLoop,
      double fps,
      CameraHealth.Numbers numbers,
      long droppedLately,
      long lostLately,
      double detectMeanMs,
      int restarts,
      Set<Warning> warnings) {}

  /** A camera on Visor, whose frames are numbered. */
  private static Scenario scenario(
      String name,
      java.util.function.BiConsumer<Visor, Integer> atLoop,
      double fps,
      long dropped,
      long lost,
      double detectMs,
      int restarts,
      Set<Warning> warnings) {
    return new Scenario(
        name,
        atLoop,
        fps,
        CameraHealth.Numbers.REPORTED,
        dropped,
        lost,
        detectMs,
        restarts,
        warnings);
  }

  /** A camera whose frames aren't numbered: no dropped or lost count. */
  private static Scenario unnumbered(
      String name, java.util.function.BiConsumer<Visor, Integer> atLoop, double fps) {
    return new Scenario(
        name, atLoop, fps, CameraHealth.Numbers.NOT_REPORTED, 0, 0, Double.NaN, 0, NONE);
  }

  private static final Set<Warning> NONE = EnumSet.noneOf(Warning.class);

  @Test
  void eachScenarioReadsAsItHappened() {
    List<Scenario> scenarios =
        List.of(
            scenario("healthy", (visor, loop) -> {}, 60, 0, 0, 4, 0, NONE),
            scenario(
                "dropping frames: three, 4 s ago",
                (visor, loop) -> {
                  if (loop == 400) {
                    visor.droppedNext = 3;
                  }
                },
                59.7,
                3,
                0,
                4,
                0,
                EnumSet.of(Warning.DROPPED_FRAMES)),
            scenario(
                "dropped frames, but 11 s ago: out of the window",
                (visor, loop) -> {
                  if (loop == 10) {
                    visor.droppedNext = 3;
                  }
                },
                60,
                0,
                0,
                4,
                0,
                NONE),
            scenario(
                "losing frames on the network",
                (visor, loop) -> {
                  if (loop % 50 == 0) {
                    visor.lostNext = 1;
                  }
                },
                59,
                0,
                10,
                4,
                0,
                EnumSet.of(Warning.LOST_FRAMES)),
            scenario(
                "detection slower than the frame",
                (visor, loop) -> visor.detectMs = 20,
                60,
                0,
                0,
                20,
                0,
                EnumSet.of(Warning.SLOW_DETECTION)),
            scenario(
                "Visor restarted: its counts start again, nothing lost or dropped",
                (visor, loop) -> {
                  if (loop == 300) {
                    visor.sequence = -1;
                    visor.dropped = 0;
                  }
                },
                60,
                0,
                0,
                4,
                1,
                NONE),
            scenario(
                "its dropped count wraps past 32 bits",
                (visor, loop) -> {
                  if (loop == 0) {
                    visor.dropped = 0xffff_ffffL - 1;
                  }
                  if (loop == 550) {
                    visor.droppedNext = 4;
                  }
                },
                59.6,
                4,
                0,
                4,
                0,
                EnumSet.of(Warning.DROPPED_FRAMES)),
            scenario(
                "a frame arrives after a newer one: late, not lost, and no restart",
                (visor, loop) -> {
                  if (loop == 400) {
                    visor.holdOneBack = true;
                  }
                },
                60,
                0,
                0,
                4,
                0,
                NONE),
            scenario(
                "Visor restarted, the first frame heard numbered above the last: frames lost",
                (visor, loop) -> {
                  if (loop == 400) {
                    // It counted from 0 again, and every frame up to 29 past the old last was
                    // lost on the way. By the numbers alone, that is 29 frames lost, and no
                    // restart.
                    visor.sequence += 29;
                  }
                },
                60,
                0,
                29,
                4,
                0,
                EnumSet.of(Warning.LOST_FRAMES)),
            scenario(
                "its number wraps past 32 bits: one restart, nothing lost",
                (visor, loop) -> {
                  if (loop == 0) {
                    visor.sequence = 0xffff_ffffL - 300;
                  }
                },
                60,
                0,
                0,
                4,
                1,
                NONE),
            unnumbered(
                "PhotonVision: frames counted, the rest unknown",
                (visor, loop) -> visor.numbered = false,
                60));
    for (Scenario scenario : scenarios) {
      CameraHealth health = new CameraHealth(FRAME_PERIOD_SECONDS);
      Visor visor = new Visor();
      for (int loop = 0; loop < 600; loop++) {
        scenario.atLoop().accept(visor, loop);
        visor.loop(health, loop * LOOP_NANOS);
      }
      String name = scenario.name();
      assertThat(health.framesPerSecond()).as(name).isCloseTo(scenario.fps(), within(0.5));
      assertThat(health.numbers()).as(name).isEqualTo(scenario.numbers());
      if (scenario.numbers() == CameraHealth.Numbers.REPORTED) {
        assertThat(health.droppedLately()).as(name).isEqualTo(scenario.droppedLately());
        assertThat(health.lostLately()).as(name).isEqualTo(scenario.lostLately());
      } else {
        assertThatThrownBy(health::droppedLately)
            .as(name)
            .isInstanceOf(IllegalStateException.class);
        assertThatThrownBy(health::lostLately).as(name).isInstanceOf(IllegalStateException.class);
      }
      if (Double.isNaN(scenario.detectMeanMs())) {
        assertThat(health.detectMeanMs()).as(name).isNaN();
        assertThat(health.detectWorstMs()).as(name).isNaN();
      } else {
        assertThat(health.detectMeanMs()).as(name).isCloseTo(scenario.detectMeanMs(), within(1e-9));
        assertThat(health.detectWorstMs())
            .as(name)
            .isCloseTo(scenario.detectMeanMs(), within(1e-9));
      }
      assertThat(health.restarts()).as(name).isEqualTo(scenario.restarts());
      for (Warning warning : Warning.values()) {
        assertThat(health.warns(warning))
            .as(name + ": " + warning)
            .isEqualTo(scenario.warnings().contains(warning));
      }
    }
  }

  /**
   * A drop is a warning for 10 s and then gone, and the session remembers it: how many times, and
   * when it ended.
   */
  @Test
  void aWarningPassesAndTheSessionRemembersIt() {
    CameraHealth health = new CameraHealth(FRAME_PERIOD_SECONDS);
    Visor visor = new Visor();
    IntConsumer loop = index -> visor.loop(health, index * LOOP_NANOS);
    for (int i = 0; i < 100; i++) {
      loop.accept(i);
    }
    visor.droppedNext = 1;
    loop.accept(100); // 2 s in
    assertThat(health.warns(Warning.DROPPED_FRAMES)).isTrue();
    Episodes drops = health.episodes(Warning.DROPPED_FRAMES);
    assertThat(drops.holds()).isTrue();
    assertThat(drops.count()).isEqualTo(1);
    for (int i = 101; i < 700; i++) {
      loop.accept(i);
    }
    assertThat(health.warns(Warning.DROPPED_FRAMES)).as("14 s in").isFalse();
    assertThat(drops.holds()).isFalse();
    assertThat(drops.count()).isEqualTo(1);
    // It held from 2 s until its second left the window, 9 to 10 s later.
    assertThat(drops.seconds(health.nowNanos())).isBetween(9.0, 10.0);
    assertThat(drops.lastEndedNanos()).isPresent();
  }

  @Test
  void theFrameRateWaitsForASecond() {
    CameraHealth health = new CameraHealth(FRAME_PERIOD_SECONDS);
    Visor visor = new Visor();
    for (int i = 0; i < 49; i++) {
      visor.loop(health, i * LOOP_NANOS);
    }
    assertThat(health.framesPerSecond()).as("0.96 s in").isNaN();
    visor.loop(health, 50 * LOOP_NANOS);
    assertThat(health.framesPerSecond()).isCloseTo(60, within(2.0));
  }

  @Test
  void aCameraThatStopsSendingHasNoFramesAfterTenSeconds() {
    CameraHealth health = new CameraHealth(FRAME_PERIOD_SECONDS);
    Visor visor = new Visor();
    for (int i = 0; i < 300; i++) {
      visor.loop(health, i * LOOP_NANOS);
    }
    for (int i = 300; i < 900; i++) {
      health.update(i * LOOP_NANOS, 0, visor.heardSequence, visor.heardDropped, new double[0]);
    }
    assertThat(health.framesPerSecond()).isZero();
    assertThat(health.detectMeanMs()).isNaN();
    assertThat(health.lostLately()).as("nothing sent, nothing lost").isZero();
  }
}

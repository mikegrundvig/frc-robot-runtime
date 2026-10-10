package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * A joint's limit switches checked against where it is: doubted after a quarter second of
 * disagreeing by more than a tenth of its range. No simulator, each switch read as a test says.
 */
class SwitchDoubtsTest {
  /**
   * The pivot's switch at its lower end, tripping at 30°: its range is 135°, so 13.5° either way.
   */
  private static final Sensor.Switch PIVOT_DOWN =
      JointFixtures.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Math.toRadians(30));

  private static final long LOOP = 20_000_000L;

  private final SwitchReadings switches = new SwitchReadings(PIVOT_DOWN);
  private final SwitchDoubts pivot = new SwitchDoubts(PIVOT, List.of(PIVOT_DOWN));
  private long now;

  /** Checks it each loop for a number of loops, the pivot where it is. */
  private void loops(int count, boolean knowsWhere, double degrees) {
    for (int i = 0; i < count; i++) {
      pivot.check(now, knowsWhere, Math.toRadians(degrees), switches.sensors());
      now += LOOP;
    }
  }

  /** Tripped with the pivot out at 130°: doubted, but not for a moment's disagreement. */
  @Test
  void trippedNowhereNearWhereItTripsIsDoubtedAfterAQuarterSecond() {
    switches.trip(PIVOT_DOWN, true);

    loops(13, true, 130);
    List<String> early = pivot.doubts();
    loops(1, true, 130);

    assertThat(early).isEmpty();
    assertThat(pivot.doubts())
        .containsExactly(
            "pivot down reads tripped with pivot at 130°, nowhere near where it trips, 30°: a"
                + " cut wire, if it's normally closed, or a switch out of place");
    assertThat(pivot.logged()).hasSize(1);
  }

  /** Clear with the pivot at its rest, well past where it trips. */
  @Test
  void clearWellPastWhereItTripsIsDoubted() {
    loops(14, true, 1.5);

    assertThat(pivot.doubts())
        .containsExactly(
            "pivot down reads clear with pivot at 1.5°, past where it trips, 30°: a cut wire, if"
                + " it's normally open, or a switch out of place");
  }

  /**
   * Nothing's doubted where it agrees, within the margin, while the joint doesn't know where it is,
   * or with the switch lost; and a doubt clears as the switch agrees again.
   */
  @Test
  void nothingIsDoubtedWhereItCantBeKnown() {
    switches.trip(PIVOT_DOWN, true);
    loops(20, true, 20);
    List<String> within = pivot.doubts();
    loops(20, false, 130);
    List<String> unhomed = pivot.doubts();
    switches.lose(PIVOT_DOWN, true);
    loops(20, true, 130);
    List<String> lost = pivot.doubts();
    switches.lose(PIVOT_DOWN, false);
    loops(20, true, 130);
    loops(1, true, 25);

    assertThat(within).isEmpty();
    assertThat(unhomed).isEmpty();
    assertThat(lost).isEmpty();
    assertThat(pivot.doubts()).as("agreeing again").isEmpty();
  }

  /** A switch whose position isn't known can't be doubted; a free joint's margin is 15°. */
  @Test
  void aSwitchWithNoPositionIsNeverDoubted() {
    Sensor.Switch somewhere =
        JointFixtures.limit("turret left", TURRET, Sensor.Switch.Limit.End.LOWER);
    SwitchReadings turretSwitch = new SwitchReadings(somewhere);
    SwitchDoubts turret = new SwitchDoubts(TURRET, List.of(somewhere));
    turretSwitch.trip(somewhere, true);

    for (int i = 0; i < 20; i++) {
      turret.check(i * LOOP, true, 3, turretSwitch.sensors());
    }

    assertThat(turret.doubts()).isEmpty();
  }
}

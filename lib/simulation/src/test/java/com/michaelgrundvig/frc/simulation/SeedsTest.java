package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * Each of a world's random streams starts where {@link Seeds} says: its seed mixed with its kind,
 * then its index, by SplitMix64's mixing. The truth suite holds them to being apart and
 * uncorrelated (the robot program's {@code SeedsTruthSimTest}, beside its cameras').
 */
class SeedsTest {
  /** SplitMix64's first number from a state of zero, as its reference implementation gives it. */
  @Test
  void mixingIsSplitMix64() {
    assertThat(Seeds.mix(0, 0L)).isEqualTo(0xE220A8397B1DCDAFL);
  }

  @Test
  void eachStreamIsTheSeedMixedWithItsKindThenItsIndex() {
    assertThat(Seeds.gyro(7)).isEqualTo(Seeds.stream(7, 0x6779726f73L, 0));
    assertThat(Seeds.backupGyro(7)).isEqualTo(Seeds.stream(7, 0x6779726f73L, 1));
    assertThat(Seeds.stream(7, 5, 2)).isEqualTo(Seeds.mix(Seeds.mix(7, 5L), 2L));
    assertThat(Seeds.upAxis(8)).isEqualTo(Seeds.mix(8, 0x75702d61786973L));
  }

  @Test
  void seedsDependOnEveryNumberMixedInAndItsOrder() {
    assertThat(Seeds.mix(1, 0.5, 2.0))
        .isEqualTo(Seeds.mix(1, 0.5, 2.0))
        .isNotEqualTo(Seeds.mix(1, 2.0, 0.5))
        .isNotEqualTo(Seeds.mix(2, 0.5, 2.0));
  }

  /** A hundred worlds' IMU and camera streams, every one its own. */
  @Test
  void noTwoStreamsOfAHundredWorldsShareASeed() {
    Set<Long> seen = new HashSet<>();
    int streams = 0;
    for (long seed = 1; seed <= 100; seed++) {
      long[] world = {
        Seeds.gyro(seed),
        Seeds.gyroAccelerometer(seed),
        Seeds.upAxis(Seeds.gyroAccelerometer(seed)),
        Seeds.backupGyro(seed),
        Seeds.backupAccelerometer(seed),
        Seeds.upAxis(Seeds.backupAccelerometer(seed)),
        Seeds.camera(seed, 0),
        Seeds.camera(seed, 1),
        Seeds.camera(seed, 2),
        Seeds.camera(seed, 3)
      };
      for (long stream : world) {
        seen.add(stream);
        streams++;
      }
    }
    assertThat(seen).hasSize(streams);
  }
}

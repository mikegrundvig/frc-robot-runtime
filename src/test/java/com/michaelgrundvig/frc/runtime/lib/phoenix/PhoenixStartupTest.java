package com.michaelgrundvig.frc.runtime.lib.phoenix;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.StatusCode;
import com.michaelgrundvig.frc.world.robot.CanBus;
import java.util.List;
import org.junit.jupiter.api.Test;

/** How many times a device is sent its settings as the robot starts, and when it stops trying. */
class PhoenixStartupTest {
  /** A bus that is seen, or not; whether the device answers; how many sends; what it says. */
  private record Row(boolean busSeen, boolean answers, int sends, boolean confirmed) {}

  @Test
  void aDeviceIsTriedAFewTimesOnABusThatIsSeenAndOnceOnOneThatIsnt() {
    List<Row> rows =
        List.of(
            new Row(true, true, 1, true),
            new Row(true, false, 5, false),
            new Row(false, true, 1, true),
            new Row(false, false, 1, false));
    for (Row row : rows) {
      int[] sends = {0};

      boolean confirmed =
          PhoenixStartup.confirmed(
              PhoenixStartup.tries(row.busSeen()),
              () -> {
                sends[0]++;
                return row.answers() ? StatusCode.OK : StatusCode.EcuIsNotPresent;
              });

      assertThat(sends[0]).as("%s: sends", row).isEqualTo(row.sends());
      assertThat(confirmed).as("%s", row).isEqualTo(row.confirmed());
    }
  }

  /** One of Systemcore's own ports isn't asked about: it is part of the controller. */
  @Test
  void systemcoresOwnPortIsTakenAsSeen() {
    assertThat(PhoenixStartup.busSeen(CanBus.port(CanBus.Systemcore.CAN_S0))).isTrue();
  }
}

package com.michaelgrundvig.frc.runtime.dashboard;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/** What reaches the dashboards: every loop in the pit, fewer on the field. */
class DashboardStreamTest {
  /** A dashboard that remembers which loops reached it. */
  private static final class Dashboard implements LogDataReceiver {
    final List<Long> received = new ArrayList<>();
    boolean started;
    boolean ended;

    @Override
    public void start() {
      started = true;
    }

    @Override
    public void end() {
      ended = true;
    }

    @Override
    public void putTable(LogTable table) {
      received.add(table.getTimestamp());
    }
  }

  private static LogTable loop(long timestamp, boolean fieldAttached) {
    LogTable table = new LogTable(timestamp);
    table.put("DriverStation/FMSAttached", fieldAttached);
    return table;
  }

  @Test
  void inThePitEveryLoopStreamsAndOnTheFieldOneInFive() throws InterruptedException {
    Dashboard dashboard = new Dashboard();
    DashboardStream stream = new DashboardStream(dashboard);

    stream.start();
    for (long t = 0; t < 3; t++) {
      stream.putTable(loop(t, false));
    }
    for (long t = 3; t < 13; t++) {
      stream.putTable(loop(t, true));
    }
    stream.end();

    assertThat(dashboard.received).containsExactly(0L, 1L, 2L, 5L, 10L);
    assertThat(dashboard.started).isTrue();
    assertThat(dashboard.ended).isTrue();
  }
}

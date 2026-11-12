package com.michaelgrundvig.frc.runtime.dashboard;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/**
 * What reaches the dashboards: every loop in the pit, fewer on the field. And among the log's other
 * receivers, as AdvantageKit runs them, a stream that throws never costs the log's file a loop.
 */
class DashboardStreamTest {
  private static final int LOOPS = 10;

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

  /** A receiver that counts the tables it was given: the log's file. */
  private static final class Counting implements LogDataReceiver {
    int tables;

    @Override
    public void putTable(LogTable table) {
      tables++;
    }
  }

  /** Dashboards that take loops until one, then throw: NetworkTables' publisher, gone wrong. */
  private static final class Failing implements LogDataReceiver {
    final List<Long> received = new ArrayList<>();
    private final Failure failure;

    Failing(Failure failure) {
      this.failure = failure;
    }

    @Override
    public void putTable(LogTable table) {
      received.add(table.getTimestamp());
      if (received.size() > failure.failsAt()) {
        throw failure.thrown();
      }
    }
  }

  /**
   * One way streaming a loop goes wrong.
   *
   * @param failsAt the loop it first throws in, counting from 0
   * @param thrown what it throws
   * @param alertThrows whether saying so throws too
   * @param said what the alert says of it
   */
  private record Failure(
      String name, int failsAt, RuntimeException thrown, boolean alertThrows, String said) {}

  private static final List<Failure> FAILURES =
      List.of(
          new Failure(
              "on the first loop",
              0,
              new IllegalStateException("publisher closed"),
              false,
              "publisher closed"),
          new Failure(
              "partway through",
              4,
              new ArrayIndexOutOfBoundsException("Index 6 out of bounds for length 6"),
              false,
              "Index 6 out of bounds for length 6"),
          new Failure(
              "with no message",
              2,
              new NullPointerException(),
              false,
              "java.lang.NullPointerException"),
          new Failure(
              "and the alert throws too",
              1,
              new IllegalStateException("publisher closed"),
              true,
              "publisher closed"));

  private static LogTable loop(long timestamp, boolean fieldAttached) {
    LogTable table = new LogTable(timestamp);
    table.put("DriverStation/FMSAttached", fieldAttached);
    return table;
  }

  /**
   * A robot's stream: every loop in the pit and one in five on the field. The simulator's: every
   * loop, wherever, as it has no radio to spare.
   */
  @Test
  void inThePitEveryLoopStreamsAndOnARobotsFieldOneInFive() throws InterruptedException {
    record Row(DashboardStream.OnTheField onTheField, List<Long> streamed) {}
    List<Row> rows =
        List.of(
            new Row(DashboardStream.OnTheField.THINNED, List.of(0L, 1L, 2L, 5L, 10L)),
            new Row(
                DashboardStream.OnTheField.EVERY_LOOP,
                List.of(0L, 1L, 2L, 3L, 4L, 5L, 6L, 7L, 8L, 9L, 10L, 11L, 12L)));
    for (Row row : rows) {
      Dashboard dashboard = new Dashboard();
      DashboardStream stream = new DashboardStream(dashboard, row.onTheField());

      stream.start();
      for (long t = 0; t < 3; t++) {
        stream.putTable(loop(t, false));
      }
      for (long t = 3; t < 13; t++) {
        stream.putTable(loop(t, true));
      }
      stream.end();

      assertThat(dashboard.received).as(row.onTheField().name()).isEqualTo(row.streamed());
      assertThat(dashboard.started).isTrue();
      assertThat(dashboard.ended).isTrue();
    }
  }

  @Test
  void aStreamThatThrowsNeverStopsTheLog() throws InterruptedException {
    for (Failure failure : FAILURES) {
      Counting file = new Counting();
      Failing dashboards = new Failing(failure);
      List<String> alerts = new ArrayList<>();
      DashboardStream stream =
          new DashboardStream(
              dashboards,
              DashboardStream.OnTheField.THINNED,
              text -> {
                alerts.add(text);
                if (failure.alertThrows()) {
                  throw new IllegalStateException("no alerts in this program");
                }
              });
      List<LogDataReceiver> receivers = List.of(stream, file);

      for (long t = 0; t < LOOPS; t++) {
        LogTable table = loop(t, false);
        // As AdvantageKit's receiver thread does it: each in turn, nothing caught.
        for (LogDataReceiver receiver : receivers) {
          receiver.putTable(table);
        }
      }

      String name = failure.name();
      assertThat(file.tables).as(name + ": the log's file").isEqualTo(LOOPS);
      assertThat(dashboards.received)
          .as(name + ": the stream stops at the loop that threw")
          .hasSize(failure.failsAt() + 1);
      assertThat(alerts).as(name + ": said once").hasSize(1);
      assertThat(alerts.get(0))
          .as(name)
          .startsWith("The dashboards' stream stopped")
          .endsWith(failure.said());
    }
  }
}

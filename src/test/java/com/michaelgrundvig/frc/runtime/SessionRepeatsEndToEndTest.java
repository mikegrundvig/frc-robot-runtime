package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.CleanupMode;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;

/**
 * A recorded session is the same session every time ({@link SessionRecording}): recorded twice, its
 * two logs hold the same keys, of the same types, with the same values at the same times, to the
 * bit. That is what the tests that share a session rest on. A session whose loops hung on the
 * computer's clock differed from one recording to the next even on a quiet computer, and failed on
 * a busy one about once in ten runs.
 *
 * <p>Two recordings that agree are the check. A session that waited for a length of time, or let
 * another thread's work land in whichever loop it happened to, would differ between them.
 *
 * <p>Asked for by name, the second recording is made while every processor of the computer is kept
 * busy, so its loops take several times as long and its threads get their turns late ({@link
 * #UNDER_LOAD}). That is a harder version of the same check. It takes over the whole computer while
 * it runs, so it is never part of a routine build: run it alone, when the session's own code
 * changes.
 *
 * <p>A few keys are about the computer and not the robot, and they alone are left out ({@link
 * #ABOUT_THE_COMPUTER}).
 */
@EndToEndTest
class SessionRepeatsEndToEndTest {
  /**
   * The keys that say something about the computer the program ran on, each by how its name starts.
   * Everything else in a log is the robot's, and must repeat.
   *
   * <ul>
   *   <li>{@code Loop/}: how long each loop took on the computer's own clock, section by section,
   *       and the alert raised from that.
   *   <li>{@code SystemStats/EpochTime}: the computer's date and time.
   *   <li>{@code LoggedRobot/GC}: how long, and how often, Java collected garbage in a loop.
   *   <li>{@code Logger/QueuedCycles}: how many loops' records waited for the log's own thread to
   *       write them.
   *   <li>{@code Console}: what the program printed, which names the port Pitwall was given and the
   *       log's own file, named at random and then for the date.
   * </ul>
   */
  private static final List<String> ABOUT_THE_COMPUTER =
      List.of(
          "/RealOutputs/Loop/",
          "/SystemStats/EpochTime",
          "/RealOutputs/LoggedRobot/GC",
          "/RealOutputs/Logger/QueuedCycles",
          "/RealOutputs/Console");

  /**
   * Whether the second recording is made on a computer kept busy: only when the environment asks
   * for it, {@code SESSIONS_UNDER_LOAD=1}. Left out, both recordings are made as any test runs.
   */
  private static final boolean UNDER_LOAD = System.getenv("SESSIONS_UNDER_LOAD") != null;

  @TempDir(cleanup = CleanupMode.ON_SUCCESS) // a failed run keeps its logs to look at
  static Path dir;

  /** A session, by the folder it is recorded in. */
  @FunctionalInterface
  private interface Session {
    Path record(Path dir) throws IOException, InterruptedException;
  }

  /**
   * One key of a log as it was written: its type, and each value with its time.
   *
   * @param type the type and the metadata the key was started with
   * @param samples each value written, as its bytes in hex, after its time in nanoseconds
   */
  private record Series(String type, List<String> samples) {}

  @Test
  @Timeout(value = 4, unit = TimeUnit.MINUTES) // two runs of the program, each allowed 90 s
  void theSessionWithoutControlsRecordsTheSameLogEveryTime() throws Exception {
    recordsTheSameLogTwice("plain", SessionRecording::record);
  }

  @Test
  @Timeout(value = 4, unit = TimeUnit.MINUTES) // two runs of the program, each allowed 90 s
  void theSessionWithControlsRecordsTheSameLogEveryTime() throws Exception {
    recordsTheSameLogTwice("controls", SessionRecording::recordWithControls);
  }

  private static void recordsTheSameLogTwice(String name, Session session) throws Exception {
    Path quietDir = Files.createDirectories(dir.resolve(name + "-first"));
    Path busyDir = Files.createDirectories(dir.resolve(name + "-second"));
    Map<String, Series> quiet = read(session.record(quietDir));
    Map<String, Series> busy = recordedAgain(session, busyDir);

    assertThat(quiet.size()).as("keys the session logs").isGreaterThan(500);
    assertThat(busy.keySet()).as("the keys of the two recordings").isEqualTo(quiet.keySet());
    List<String> differing = new ArrayList<>();
    for (Map.Entry<String, Series> key : quiet.entrySet()) {
      Series again = Objects.requireNonNull(busy.get(key.getKey()));
      if (!key.getValue().equals(again)) {
        differing.add(key.getKey() + ": " + firstDifference(key.getValue(), again));
      }
    }
    assertThat(differing).as("keys that differ between the two recordings").isEmpty();
  }

  /**
   * Every key of a log but those about the computer, by name.
   *
   * <p>The log is read with the reader's own {@code forEach}. Its iterator, which a for-each loop
   * uses, stops 16 bytes short of the end of the file, so the last value in a log went unread
   * whenever it was a number or a flag, and which key that is differs from one log to the next.
   */
  private static Map<String, Series> read(Path log) throws IOException {
    DataLogReader reader = new DataLogReader(ByteBuffer.wrap(Files.readAllBytes(log)));
    assertThat(reader.isValid()).as("a valid log: " + log).isTrue();
    Map<Integer, String> names = new LinkedHashMap<>();
    Map<String, Series> keys = new TreeMap<>();
    reader.forEach(record -> take(record, names, keys));
    return keys;
  }

  /** Takes one record of a log: a new key, or a value of one already found. */
  private static void take(
      DataLogRecord record, Map<Integer, String> names, Map<String, Series> keys) {
    if (record.isStart()) {
      var start = record.getStartData();
      if (ABOUT_THE_COMPUTER.stream().noneMatch(start.name::startsWith)) {
        names.put(start.entry, start.name);
        keys.put(start.name, new Series(start.type + " " + start.metadata, new ArrayList<>()));
      }
      return;
    }
    String name = names.get(record.getEntry());
    if (record.isControl() || name == null) {
      return;
    }
    Objects.requireNonNull(keys.get(name))
        .samples()
        .add(record.getTimestamp() + " " + HexFormat.of().formatHex(record.getRaw()));
  }

  /** The session recorded a second time: on a busy computer when that was asked for. */
  private static Map<String, Series> recordedAgain(Session session, Path into) throws Exception {
    if (!UNDER_LOAD) {
      return read(session.record(into));
    }
    try (Busy keptBusy = new Busy()) {
      return read(session.record(into));
    }
  }

  private static String firstDifference(Series quiet, Series busy) {
    if (!quiet.type().equals(busy.type())) {
      return quiet.type() + " first, " + busy.type() + " second";
    }
    int both = Math.min(quiet.samples().size(), busy.samples().size());
    for (int i = 0; i < both; i++) {
      if (!quiet.samples().get(i).equals(busy.samples().get(i))) {
        return "value "
            + i
            + ": "
            + quiet.samples().get(i)
            + " first, "
            + busy.samples().get(i)
            + " second";
      }
    }
    return quiet.samples().size() + " values first, " + busy.samples().size() + " second";
  }

  /** Keeps every processor of the computer busy until it is closed. */
  private static final class Busy implements AutoCloseable {
    private final List<Thread> threads = new ArrayList<>();
    private volatile boolean stopped;

    Busy() {
      for (int i = 0; i < Runtime.getRuntime().availableProcessors(); i++) {
        Thread thread = new Thread(this::spin, "Busy " + i);
        thread.setDaemon(true);
        thread.start();
        threads.add(thread);
      }
    }

    private void spin() {
      while (!stopped) {
        Thread.onSpinWait();
      }
    }

    @Override
    public void close() throws InterruptedException {
      stopped = true;
      for (Thread thread : threads) {
        thread.join();
      }
    }
  }
}

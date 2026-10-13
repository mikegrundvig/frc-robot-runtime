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
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.CleanupMode;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;

/**
 * AdvantageKit replay, end to end, with Pitwall in the loop. The real robot program records a
 * session in the simulator ({@link ReplaySessionRobot}): the Driver Station set from the
 * simulator's controls and a joystick driven from the page, a tuning change from Pitwall's page, a
 * WPILib tunable changed through Pitwall's tee, an action started from the page, which drives the
 * robot, and an experiment on the steering, scored from its trace. Then it replays its own log.
 * Every output the session recorded must come out of the replay the same, at the same moments,
 * because every one of those changes was logged as an input.
 *
 * <p>Each run is its own process: AdvantageKit's logger starts once per program, and a replay ends
 * the program when the log runs out ({@link SessionRecording}).
 */
@EndToEndTest
class ReplayEndToEndTest {
  private static final List<String> OUTPUTS =
      List.of(
          "Session/Gain",
          "Session/Operator",
          "Session/UtilityInits",
          "Session/SteerKp",
          "Session/Action",
          "Session/Experiment",
          "Session/ExperimentRise",
          "Drive/Pose",
          "Field/Alliance",
          "Field/Placement",
          "Field/Zones/Trench");

  @TempDir(cleanup = CleanupMode.ON_SUCCESS) // a failed run keeps its logs to look at
  static Path dir;

  /** The session's log, as the robot recorded it. */
  private static Path recorded = Path.of("");

  @BeforeAll
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  static void recordTheSession() throws Exception {
    recorded = SessionRecording.record(dir);
  }

  /** An output in a log: its name without the table, and its type. */
  record Entry(String name, String type) {}

  /** One logged value and when. Values are numbers, text, or raw bytes as hex. */
  record Sample(long timestamp, Object value) {}

  @Test
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  void aReplayReproducesTheSessionIncludingEveryChangeMadeThroughPitwall() throws Exception {
    SessionRecording.run(
        dir,
        "replay",
        Map.of("frc.simMode", "REPLAY"),
        Map.of("AKIT_LOG_PATH", recorded.toString()));
    Path replayed =
        recorded.resolveSibling(
            recorded.getFileName().toString().replace(".wpilog", "_sim.wpilog"));

    Map<String, List<Sample>> original = outputs(recorded, "RealOutputs/");
    Map<String, List<Sample>> replay = outputs(replayed, "ReplayOutputs/");

    // The session happened: each change took effect while recording.
    assertThat(values(original, "Session/Gain")).contains(1.0, 2.5);
    assertThat(values(original, "Session/Operator")).as("the page's stick").contains(0.0, 0.5);
    assertThat(values(original, "Session/UtilityInits"))
        .as("enabled in utility once")
        .contains(1.0);
    assertThat(values(original, "Session/SteerKp")).contains(16.0, 18.5);
    assertThat(values(original, "Session/Action")).contains("READY", "RUNNING", "ENDED");
    assertThat(values(original, "Session/Experiment")).contains("NONE", "RUNNING", "ENDED");
    assertThat(values(original, "Session/ExperimentRise"))
        .as("the steering's rise, scored from its trace")
        .anySatisfy(rise -> assertThat((Double) rise).isBetween(0.02, 0.3));
    assertThat(values(original, "Field/Alliance")).contains("Red 1");
    assertThat(values(original, "Field/Placement")).contains("Ready on Left: 0 cm, 0° off");
    assertThat(values(original, "Field/Zones/Trench")).as("drove under it").contains(true);

    // AdvantageKit's reader reads each loop up to the next loop's timestamp, so a replay runs every
    // loop of the log except the last.
    long lastReplayed =
        replay.values().stream().flatMap(List::stream).mapToLong(Sample::timestamp).max().orElse(0);
    long notReplayed =
        original.values().stream()
            .flatMap(List::stream)
            .mapToLong(Sample::timestamp)
            .filter(time -> time > lastReplayed)
            .distinct()
            .count();
    assertThat(notReplayed).as("recorded loops the replay didn't run").isLessThanOrEqualTo(1);

    // AdvantageKit writes an output when it changes, so each series is the output's changes: the
    // replay must change it at the same moments, to the same values.
    for (String output : OUTPUTS) {
      List<Sample> expected =
          original.getOrDefault(output, List.of()).stream()
              .filter(sample -> sample.timestamp() <= lastReplayed)
              .toList();
      assertThat(expected).as(output + " recorded").isNotEmpty();
      List<Sample> actual = replay.getOrDefault(output, List.of());
      assertThat(actual)
          .as(output + " replayed: " + firstDifference(expected, actual))
          .isEqualTo(expected);
    }
  }

  /**
   * Every output under a log's output table, by name without the table.
   *
   * <p>The log is read with the reader's own {@code forEach}. Its iterator, which a for-each loop
   * uses, stops 16 bytes short of the end of the file, and a number or a flag written last in the
   * log is shorter than that: the last value of whichever output was written last went unread.
   */
  private static Map<String, List<Sample>> outputs(Path log, String table) throws IOException {
    DataLogReader reader = new DataLogReader(ByteBuffer.wrap(Files.readAllBytes(log)));
    assertThat(reader.isValid()).as("a valid log: " + log).isTrue();
    Map<Integer, Entry> entries = new LinkedHashMap<>();
    Map<String, List<Sample>> samples = new LinkedHashMap<>();
    reader.forEach(record -> take(record, table, entries, samples));
    return samples;
  }

  /** Takes one record of a log: a new output under the table, or a value of one already found. */
  private static void take(
      DataLogRecord record,
      String table,
      Map<Integer, Entry> entries,
      Map<String, List<Sample>> samples) {
    if (record.isStart()) {
      var start = record.getStartData();
      String name = start.name.startsWith("/") ? start.name.substring(1) : start.name;
      if (name.startsWith(table)) {
        entries.put(start.entry, new Entry(name.substring(table.length()), start.type));
      }
      return;
    }
    if (record.isControl()) {
      return;
    }
    Entry entry = entries.get(record.getEntry());
    if (entry == null) {
      return;
    }
    Object value =
        switch (entry.type()) {
          case "double" -> record.getDouble();
          case "string" -> record.getString();
          case "boolean" -> record.getBoolean();
          default -> HexFormat.of().formatHex(record.getRaw());
        };
    samples
        .computeIfAbsent(entry.name(), n -> new ArrayList<>())
        .add(new Sample(record.getTimestamp(), value));
  }

  private static String firstDifference(List<Sample> expected, List<Sample> actual) {
    for (int i = 0; i < Math.min(expected.size(), actual.size()); i++) {
      if (!expected.get(i).equals(actual.get(i))) {
        return "sample "
            + i
            + " of "
            + expected.size()
            + " recorded, "
            + actual.size()
            + " replayed: "
            + expected.get(i)
            + " vs "
            + actual.get(i);
      }
    }
    return expected.size() + " recorded, " + actual.size() + " replayed";
  }

  private static List<Object> values(Map<String, List<Sample>> outputs, String name) {
    return outputs.getOrDefault(name, List.of()).stream().map(Sample::value).distinct().toList();
  }
}

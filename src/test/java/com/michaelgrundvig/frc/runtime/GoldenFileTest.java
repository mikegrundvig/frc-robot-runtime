package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** Recording a session again says what it changed, so a moved recording is read, not waved by. */
class GoldenFileTest {
  private static final String HEADER = "t,x,state";

  private static final List<String> RECORDED =
      List.of(HEADER, "0.0,1.0,still", "0.02,1.5,moving", "0.04,2.0,moving");

  @Test
  void recordingAgainSaysWhatChanged() {
    record Row(String name, Optional<List<String>> before, List<String> now, String says) {}
    List<Row> rows =
        List.of(
            new Row(
                "none before", Optional.empty(), RECORDED, "session.csv: a new recording, 4 rows"),
            new Row(
                "the same, within the check's tolerance",
                Optional.of(RECORDED),
                List.of(HEADER, "0.0,1.0000001,still", "0.02,1.5,moving", "0.04,2.0,moving"),
                "session.csv: 0 of 4 rows changed, 0 added, 0 removed"),
            new Row(
                "two rows moved, the first in two columns",
                Optional.of(RECORDED),
                List.of(HEADER, "0.0,1.0,still", "0.02,1.6,still", "0.04,2.1,moving"),
                "session.csv: 2 of 4 rows changed, 0 added, 0 removed; the first at row 2:"
                    + " x 1.5 -> 1.6, state moving -> still"),
            new Row(
                "a row added",
                Optional.of(RECORDED),
                List.of(HEADER, "0.0,1.0,still", "0.02,1.5,moving", "0.04,2.0,moving", "0.06,2.5,"),
                "session.csv: 0 of 4 rows changed, 1 added, 0 removed"),
            new Row(
                "a row removed",
                Optional.of(RECORDED),
                List.of(HEADER, "0.0,1.0,still", "0.02,1.5,moving"),
                "session.csv: 0 of 3 rows changed, 0 added, 1 removed"));
    for (Row row : rows) {
      assertThat(GoldenFile.changes("session.csv", HEADER, row.before(), row.now()))
          .as(row.name())
          .isEqualTo(row.says());
    }
  }
}

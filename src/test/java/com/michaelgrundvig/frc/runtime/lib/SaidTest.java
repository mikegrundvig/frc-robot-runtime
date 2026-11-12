package com.michaelgrundvig.frc.runtime.lib;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.Test;

/** What an exception says, for the reader of an alert or a log. */
class SaidTest {
  /** An exception, and what each form says of it: its message, and its kind with it. */
  private record Row(String name, Throwable thrown, String said, String withKind) {}

  @Test
  void anExceptionSaysItsMessageOrElseWhatItIs() {
    List<Row> rows =
        List.of(
            new Row(
                "a message",
                new IllegalStateException("no reading yet"),
                "no reading yet",
                "IllegalStateException: no reading yet"),
            new Row(
                "a message with a line break, stripped",
                new UnsupportedOperationException("Photonlib version mismatch\n"),
                "Photonlib version mismatch",
                "UnsupportedOperationException: Photonlib version mismatch"),
            new Row(
                "no message",
                new NullPointerException(),
                "java.lang.NullPointerException",
                "NullPointerException"),
            new Row(
                "a blank message",
                new IllegalArgumentException("  "),
                "java.lang.IllegalArgumentException:   ",
                "IllegalArgumentException"));
    for (Row row : rows) {
      assertThat(Said.of(row.thrown())).as(row.name()).isEqualTo(row.said());
      assertThat(Said.withKind(row.thrown())).as(row.name()).isEqualTo(row.withKind());
    }
  }
}

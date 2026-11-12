package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.spotter.manager.Connection;
import com.michaelgrundvig.frc.spotter.manager.Level;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

/**
 * What the robot logs of a board is all a replay needs: a minute of a board that throttles, runs
 * hot, drops out, reboots and loses a USB device, logged loop by loop as AdvantageKit logs the
 * inputs, then read back from those tables alone, judges and shows the same.
 */
class CoprocessorHealthReplayTest {
  private static final long SECOND = 1_000_000_000L;

  /** A board whose reports change at set times. */
  private static final class Eventful implements BoardIO {
    long loop;

    @Override
    public void updateInputs(BoardIOInputs inputs) {
      double seconds = loop++ / 50.0;
      inputs.connection = Connection.CONNECTED;
      inputs.why = "";
      inputs.hostname = "vision-2";
      inputs.cpuBusiestPercent = 70 + seconds % 7;
      inputs.cpuCapped = seconds >= 10 && seconds < 15;
      inputs.hottestCelsius = 60 + seconds / 3;
      inputs.thermalMarginCelsius = 30 - seconds / 2;
      inputs.thermalMarginLevel = inputs.thermalMarginCelsius < 10 ? Level.WARNING : Level.OK;
      inputs.usbDevices = "7-1 ThriftyCam";
      inputs.usbDisconnects = seconds >= 25 ? 1 : 0;
      inputs.uptimeSeconds = 900 + seconds;
      inputs.agentDescribes = 1;
      if (seconds >= 30 && seconds < 40) {
        inputs.connection = Connection.MISSING;
        inputs.why = "nothing heard from it for over 1000 ms";
      }
      if (seconds >= 40) {
        // Back, having rebooted.
        inputs.uptimeSeconds = seconds - 25;
        inputs.usbDisconnects = 0;
        inputs.agentDescribes = 2;
      }
      inputs.unavailable = new String[] {};
    }
  }

  @Test
  void aReplayOfTheLogJudgesTheBoardTheSame() {
    List<String> boards = List.of("10.12.34.12");
    Eventful board = new Eventful();
    CoprocessorHealth robot = CoprocessorHealth.of(boards, address -> board);
    List<LogTable> logged = new ArrayList<>();
    for (int loop = 0; loop <= 3000; loop++) {
      robot.periodic(loop * SECOND / 50);
      LogTable table = new LogTable(loop * 20_000L);
      robot.boards().get(0).inputs.toLog(table);
      logged.add(table);
    }

    // In a replay AdvantageKit fills each loop's inputs from the log: here, from the tables.
    int[] next = {0};
    BoardIO fromTheLog =
        new BoardIO() {
          @Override
          public void updateInputs(BoardIOInputs inputs) {
            ((BoardIOInputsAutoLogged) inputs).fromLog(logged.get(next[0]++));
          }
        };
    CoprocessorHealth replay = CoprocessorHealth.of(boards, address -> fromTheLog);
    for (int loop = 0; loop <= 3000; loop++) {
      replay.periodic(loop * SECOND / 50);
    }
    CoprocessorHealth.Watched replayed = replay.boards().get(0);

    CoprocessorHealth.Watched original = robot.boards().get(0);
    assertThat(replayed.health.reboots()).isEqualTo(original.health.reboots()).isEqualTo(1);
    assertThat(replayed.health.dropouts().count()).isEqualTo(1);
    assertThat(replayed.health.throttled().count()).isEqualTo(1);
    assertThat(replayed.health.lowThermalMargin().count()).isEqualTo(1);
    assertThat(replayed.health.usbDisconnects()).isEqualTo(1);
    assertThat(new BoardPit(replayed).card()).isEqualTo(new BoardPit(original).card());
  }

  /** Each of Spotter's words for a board is the runtime's own of the same name, one for one. */
  @Test
  void spottersWordsAreTheHealthsOwn() {
    for (Connection spotters : Connection.values()) {
      assertThat(CoprocessorHealth.connection(spotters).name()).isEqualTo(spotters.name());
    }
    for (Level spotters : Level.values()) {
      assertThat(CoprocessorHealth.level(spotters).name()).isEqualTo(spotters.name());
    }
  }
}

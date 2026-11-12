package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.runtime.lib.health.BoardHealth;
import com.michaelgrundvig.frc.spotter.manager.Connection;
import com.michaelgrundvig.frc.spotter.manager.Level;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;

/**
 * A coprocessor's card on Pitwall, after a minute of what its agent reported: fine, throttling,
 * running hot, missing and back, rebooted, a USB device dropping off, missing still, and a board
 * without the debian pack.
 */
class BoardPitTest {
  private static final long SECOND = 1_000_000_000L;

  /** A board whose reports the test writes: from a time on, the inputs as the step leaves them. */
  private static final class Scripted implements BoardIO {
    final BoardIOInputs now = fine();

    @Override
    public void updateInputs(BoardIOInputs inputs) {
      inputs.connection = now.connection;
      inputs.why = now.why;
      inputs.hostname = now.hostname;
      inputs.cpuBusiestPercent = now.cpuBusiestPercent;
      inputs.cpuCapped = now.cpuCapped;
      inputs.hottestCelsius = now.hottestCelsius;
      inputs.thermalMarginCelsius = now.thermalMarginCelsius;
      inputs.thermalMarginLevel = now.thermalMarginLevel;
      inputs.usbDisconnects = now.usbDisconnects;
      inputs.usbDevices = now.usbDevices;
      inputs.uptimeSeconds = now.uptimeSeconds;
      inputs.unavailable = now.unavailable;
      inputs.agentDescribes = now.agentDescribes;
      inputs.packPushes = now.packPushes;
      inputs.packPushesSinceDescribed = now.packPushesSinceDescribed;
    }
  }

  private static BoardIO.BoardIOInputs fine() {
    BoardIO.BoardIOInputs inputs = new BoardIO.BoardIOInputs();
    inputs.connection = Connection.CONNECTED;
    inputs.hostname = "vision-1";
    inputs.cpuBusiestPercent = 63;
    inputs.hottestCelsius = 58;
    inputs.thermalMarginCelsius = 27;
    inputs.thermalMarginLevel = Level.OK;
    inputs.usbDisconnects = 0;
    inputs.usbDevices = "7-1 ThriftyCam";
    inputs.uptimeSeconds = 600;
    inputs.agentDescribes = 1;
    return inputs;
  }

  /**
   * At 20 s into the minute, something changes; at 30 s, it may change back.
   *
   * @param at20 what changes at 20 s
   * @param at30 what changes at 30 s
   * @param state the card's state at the end
   * @param level the card's level at the end
   * @param lines each of its lines, by label, with its level, in the card's order
   * @param counted what its health counted this session, which the lines' values report
   * @param says what some of its lines say, label and the start of its value: the wording
   */
  private record Scenario(
      String name,
      Consumer<BoardIO.BoardIOInputs> at20,
      Consumer<BoardIO.BoardIOInputs> at30,
      String state,
      Devices.Level level,
      List<Line> lines,
      Counted counted,
      List<String> says) {}

  /** A line of the card: its label, and its level. */
  private record Line(String label, Devices.Level level) {}

  /**
   * What the board's health counted this session.
   *
   * @param dropouts times it went missing after it was reached
   * @param throttled times it slowed itself for heat or power
   * @param nearSlowing times it ran near slowing for heat
   * @param reboots times it rebooted
   * @param usbDisconnects times a USB device dropped off it
   * @param packRestarts times it was away while its agent restarted with the robot's packs
   */
  private record Counted(
      int dropouts,
      int throttled,
      int nearSlowing,
      int reboots,
      long usbDisconnects,
      int packRestarts) {
    static final Counted NOTHING = new Counted(0, 0, 0, 0, 0, 0);

    static Counted of(BoardHealth health) {
      return new Counted(
          health.dropouts().count(),
          health.throttled().count(),
          health.lowThermalMargin().count(),
          health.reboots(),
          health.usbDisconnects(),
          health.packRestarts());
    }
  }

  /** A reached board's lines, each fine, but those named here at their own levels. */
  private static List<Line> fineBut(Map<String, Devices.Level> levels, Line... after) {
    List<Line> lines = new ArrayList<>();
    lines.add(new Line("Host", Devices.Level.NOTE));
    for (String label : List.of("Reached", "Up for", "CPU", "Heat", "USB")) {
      lines.add(new Line(label, levels.getOrDefault(label, Devices.Level.OK)));
    }
    lines.addAll(List.of(after));
    return List.copyOf(lines);
  }

  private static List<Scenario> scenarios() {
    Consumer<BoardIO.BoardIOInputs> nothing = inputs -> {};
    Consumer<BoardIO.BoardIOInputs> backFine =
        inputs -> {
          BoardIO.BoardIOInputs fine = fine();
          inputs.connection = fine.connection;
          inputs.why = "";
          inputs.cpuCapped = false;
          inputs.thermalMarginCelsius = fine.thermalMarginCelsius;
          inputs.thermalMarginLevel = Level.OK;
        };
    Devices.Level ok = Devices.Level.OK;
    Devices.Level warning = Devices.Level.WARNING;
    Devices.Level problem = Devices.Level.PROBLEM;
    Devices.Level note = Devices.Level.NOTE;
    return List.of(
        new Scenario(
            "fine",
            nothing,
            nothing,
            "Connected",
            ok,
            fineBut(Map.of()),
            Counted.NOTHING,
            List.of(
                "Host: vision-1",
                "Reached: yes, all session",
                "Up for: 10 min",
                "CPU: 63% busiest core, full speed",
                "Heat: 58 °C hottest, 27 °C from slowing",
                "USB: 7-1 ThriftyCam")),
        new Scenario(
            "throttled for 10 s",
            inputs -> inputs.cpuCapped = true,
            backFine,
            "Connected",
            warning,
            fineBut(Map.of("CPU", warning)),
            new Counted(0, 1, 0, 0, 0, 0),
            List.of(
                "CPU: 63% busiest core; slowed for heat or power once this session, for 10 s,"
                    + " 30 s ago")),
        new Scenario(
            "near slowing for heat, failing, now",
            inputs -> {
              inputs.thermalMarginCelsius = 4;
              inputs.thermalMarginLevel = Level.FAILING;
            },
            nothing,
            "Connected",
            problem,
            fineBut(Map.of("Heat", problem)),
            new Counted(0, 0, 1, 0, 0, 0),
            List.of("Heat: 58 °C hottest, 4 °C from slowing; near slowing for 40 s so far")),
        new Scenario(
            "missing for 10 s, back",
            inputs -> {
              inputs.connection = Connection.MISSING;
              inputs.why = "nothing heard from it for over 5000 ms";
            },
            backFine,
            "Connected",
            warning,
            fineBut(Map.of("Reached", warning)),
            new Counted(1, 0, 0, 0, 0, 0),
            List.of("Reached: yes; missing once this session, for 10 s, 30 s ago")),
        new Scenario(
            "away while its agent restarted with the robot's packs",
            inputs -> {
              inputs.packPushes = 1;
              inputs.packPushesSinceDescribed = 1;
              inputs.connection = Connection.MISSING;
            },
            inputs -> {
              backFine.accept(inputs);
              inputs.agentDescribes = 2;
              inputs.packPushesSinceDescribed = 0;
            },
            "Connected",
            ok,
            fineBut(Map.of(), new Line("Packs", note)),
            new Counted(0, 0, 0, 0, 0, 1),
            List.of(
                "Reached: yes, all session",
                "Packs: away once this session while its agent restarted with the robot's")),
        new Scenario(
            "missing still",
            inputs -> {
              inputs.connection = Connection.MISSING;
              inputs.why = "Connection refused";
            },
            nothing,
            "Missing",
            problem,
            fineBut(Map.of("Reached", problem)),
            new Counted(1, 0, 0, 0, 0, 0),
            List.of("Reached: missing for 40 s so far: Connection refused")),
        new Scenario(
            "rebooted: missing, then back up for 30 s less",
            inputs -> inputs.connection = Connection.MISSING,
            inputs -> {
              backFine.accept(inputs);
              inputs.uptimeSeconds = 25;
            },
            "Connected",
            warning,
            fineBut(Map.of("Reached", warning, "Up for", warning)),
            new Counted(1, 0, 0, 1, 0, 0),
            List.of("Up for: 25 s; rebooted once this session, last seen 30 s ago")),
        new Scenario(
            "a USB device dropped off",
            inputs -> inputs.usbDisconnects = 1,
            nothing,
            "Connected",
            warning,
            fineBut(Map.of("USB", warning)),
            new Counted(0, 0, 0, 0, 1, 0),
            List.of("USB: 7-1 ThriftyCam; a device dropped off once this session, last 40 s ago")),
        new Scenario(
            "no debian pack on its image",
            inputs -> {
              inputs.cpuBusiestPercent = Double.NaN;
              inputs.hottestCelsius = Double.NaN;
              inputs.uptimeSeconds = Double.NaN;
              inputs.usbDevices = "";
              inputs.unavailable = new String[] {"debian.uptime.seconds: vision-1 has no value"};
            },
            nothing,
            "Connected",
            ok,
            fineBut(Map.of("Heat", note), new Line("Not reported", note)),
            Counted.NOTHING,
            List.of(
                "CPU: unknown busiest core",
                "Heat: unknown",
                "Not reported: debian.uptime.seconds: vision-1 has no value")),
        new Scenario(
            "another Spotter",
            inputs -> {
              inputs.connection = Connection.OTHER_PROTOCOL;
              inputs.why = "it speaks Spotter 3.0";
            },
            nothing,
            "Another Spotter",
            problem,
            List.of(new Line("Host", note), new Line("Reached", problem)),
            Counted.NOTHING,
            List.of("Reached: it speaks Spotter 3.0")));
  }

  /** A minute of the scenario's reports, the board's health read each loop as the robot does. */
  private static CoprocessorHealth aMinuteOf(Scenario scenario) {
    Scripted board = new Scripted();
    CoprocessorHealth health = CoprocessorHealth.of(List.of("10.12.34.11"), address -> board);
    for (int loop = 0; loop <= 3000; loop++) {
      if (loop == 1000) {
        scenario.at20().accept(board.now);
      }
      if (loop == 1500) {
        scenario.at30().accept(board.now);
      }
      health.periodic(loop * SECOND / 50);
    }
    return health;
  }

  /** Each scenario's card: its state and level, each line's level, and what the health counted. */
  @Test
  void eachScenarioShowsOnTheCard() {
    for (Scenario scenario : scenarios()) {
      CoprocessorHealth health = aMinuteOf(scenario);
      Devices.Device device = BoardPit.devices(health).get(0);
      CoprocessorHealth.Watched board = health.boards().get(0);
      Devices.Card card = new BoardPit(board).card();
      String name = scenario.name();
      assertThat(device.name()).isEqualTo("10-12-34-11");
      assertThat(device.kind()).isEqualTo("Coprocessor");
      assertThat(card.state()).as(name).isEqualTo(scenario.state());
      assertThat(card.level()).as(name).isEqualTo(scenario.level());
      assertThat(card.lines())
          .as(name)
          .map(line -> new Line(line.label(), line.level()))
          .isEqualTo(scenario.lines());
      assertThat(Counted.of(board.health)).as(name).isEqualTo(scenario.counted());
    }
  }

  /** What the card's lines say, in words: the one test of the wording. */
  @Test
  void eachLineSaysItInWords() {
    for (Scenario scenario : scenarios()) {
      CoprocessorHealth health = aMinuteOf(scenario);
      Devices.Card card = new BoardPit(health.boards().get(0)).card();
      List<String> lines =
          card.lines().stream().map(line -> line.label() + ": " + line.value()).toList();
      for (String said : scenario.says()) {
        assertThat(lines).as(scenario.name()).anySatisfy(line -> assertThat(line).startsWith(said));
      }
    }
  }

  @Test
  void aBoardsNameOnThePageIsItsAddressAsADevicesName() {
    assertThat(BoardPit.deviceName("10.12.34.11:5808")).isEqualTo("10-12-34-11-5808");
    assertThat(BoardPit.deviceName("Vision_Front")).isEqualTo("vision-front");
    assertThat(BoardPit.deviceName("...")).isEqualTo("board");
  }
}

package com.michaelgrundvig.frc.runtime.autos;

import com.michaelgrundvig.frc.pitwall.Devices;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * The autos' card on Pitwall's Pit view: the auto the robot will run, where it starts, and the
 * choice of any other, as the venue's card offers its venues ({@code VenuePit}).
 *
 * <p>A choice is made on Pitwall's server thread, never in the robot loop; the robot takes it at
 * the start of a loop while disabled ({@link Autos#update}). The card is read in the loop, so it
 * reads only what the loop already holds.
 */
public final class AutoPit {
  private final Autos autos;

  AutoPit(Autos autos) {
    this.autos = autos;
  }

  /** The autos' card, for {@code pitwall.devices().add(...)}. */
  public static Devices.Device device(Autos autos) {
    AutoPit pit = new AutoPit(autos);
    return Devices.Device.named("auto", "Autonomous", pit::card).choose("auto", pit::choose);
  }

  Devices.Card card() {
    Auto chosen = autos.chosen();
    List<Devices.Line> lines = new ArrayList<>();
    lines.add(new Devices.Line("Runs", chosen.name(), Devices.Level.OK));
    lines.add(
        new Devices.Line(
            "Starts on",
            chosen.start() != null ? chosen.start().name() : "wherever the Driver Station says",
            Devices.Level.NOTE));
    Devices.Level level = Devices.Level.OK;
    String failure = autos.failure();
    if (failure != null) {
      lines.add(new Devices.Line("Last run", failure, Devices.Level.PROBLEM));
      level = Devices.Level.PROBLEM;
    }
    String refusal = autos.refusal();
    if (refusal != null) {
      lines.add(new Devices.Line("Left out", refusal, Devices.Level.PROBLEM));
      level = Devices.Level.PROBLEM;
    }
    if (autos.pending()) {
      lines.add(
          new Devices.Line(
              "Chosen", "takes effect when the robot is disabled", Devices.Level.NOTE));
    }
    List<Devices.Option> options =
        autos.names().stream().map(name -> new Devices.Option(name, name)).toList();
    return new Devices.Card(
        chosen.name(),
        level,
        lines,
        List.of(new Devices.Choice("auto", "Auto", options, autos.requested())));
  }

  /** Chooses an auto, for the robot to take while disabled. */
  String choose(String value) throws IOException {
    try {
      autos.choose(value);
    } catch (IllegalArgumentException e) {
      throw new Devices.Refused(String.valueOf(e.getMessage()));
    }
    return "Chose " + value + ": the robot takes it at the start of a loop while disabled";
  }
}

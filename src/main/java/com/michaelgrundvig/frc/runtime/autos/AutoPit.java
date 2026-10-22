package com.michaelgrundvig.frc.runtime.autos;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.world.field.Start;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * The autos' card on Pitwall's Pit view. It shows the auto the robot will run and where that auto
 * starts, and offers the choice of any other, in the way the venue's card offers its venues ({@code
 * VenuePit}). It says it is the card of what autonomous runs, so in the simulator the page shows it
 * beside its Driver Station too, over the press that sets autonomous and enables.
 *
 * <p>A choice is made on one of Pitwall's server threads, never in the robot loop. The robot takes
 * it at the start of a loop while it is disabled ({@link Autos#update}). The card itself is read in
 * the robot loop, so it reads only what the loop already holds.
 */
public final class AutoPit {
  private final Autos autos;

  AutoPit(Autos autos) {
    this.autos = autos;
  }

  /** The autos' card, for {@code pitwall.devices().add(...)}. */
  public static Devices.Device device(Autos autos) {
    AutoPit pit = new AutoPit(autos);
    return Devices.Device.named("auto", "Autonomous", pit::card)
        .choose("auto", pit::choose)
        .autonomous();
  }

  /** The card as of now: what runs, where it starts, anything wrong, and the choice. */
  Devices.Card card() {
    Auto chosen = autos.chosen();
    List<Devices.Line> lines = new ArrayList<>();
    lines.add(new Devices.Line("Runs", chosen.name(), Devices.Level.OK));
    String startsOn = chosen.start().map(Start::name).orElse("wherever the Driver Station says");
    lines.add(new Devices.Line("Starts on", startsOn, Devices.Level.NOTE));

    Optional<String> failure = autos.failure();
    Optional<String> refusal = autos.refusal();
    if (failure.isPresent()) {
      lines.add(new Devices.Line("Last run", failure.get(), Devices.Level.PROBLEM));
    }
    if (refusal.isPresent()) {
      lines.add(new Devices.Line("Left out", refusal.get(), Devices.Level.PROBLEM));
    }
    if (autos.pending()) {
      lines.add(
          new Devices.Line(
              "Chosen", "takes effect when the robot is disabled", Devices.Level.NOTE));
    }

    boolean somethingIsWrong = failure.isPresent() || refusal.isPresent();
    Devices.Level level = somethingIsWrong ? Devices.Level.PROBLEM : Devices.Level.OK;
    List<Devices.Option> options = new ArrayList<>();
    for (String name : autos.names()) {
      options.add(new Devices.Option(name, name));
    }
    Devices.Choice choice = new Devices.Choice("auto", "Auto", options, autos.requested());
    return new Devices.Card(chosen.name(), level, lines, List.of(choice));
  }

  /**
   * Chooses an auto, for the robot to take while it is disabled.
   *
   * @return what the page says it did
   * @throws Devices.Refused saying which autos there are, if none has this name
   */
  String choose(String value) throws IOException {
    try {
      autos.choose(value);
    } catch (IllegalArgumentException e) {
      throw new Devices.Refused(String.valueOf(e.getMessage()));
    }
    return "Chose " + value + ": the robot takes it at the start of a loop while disabled";
  }
}

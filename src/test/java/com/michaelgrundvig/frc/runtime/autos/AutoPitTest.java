package com.michaelgrundvig.frc.runtime.autos;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.field.Place;
import java.io.IOException;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The autos' card on the Pit view, as the venue's is: the choice the page makes is the pit's, taken
 * by the robot while disabled, and refused, saying why, for an auto the robot doesn't have.
 */
@ExtendWith(RobotSim.class)
class AutoPitTest {
  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final Autos autos = new Autos(scheduler);
  private final Devices.Device device = AutoPit.device(autos);
  private final AutoPit pit = new AutoPit(autos);

  AutoPitTest() {
    autos.add(
        "Left",
        Place.at("Left start", new Pose2d(3.6, 7.4, Rotation2d.ZERO)),
        () -> Command.noRequirements(coroutine -> {}).named("Left"));
  }

  @AfterEach
  void close() {
    autos.close();
  }

  private Devices.Card card() {
    return pit.card();
  }

  @Test
  void thePagesChoiceIsTakenWhileDisabled() throws IOException {
    String said = device.chooser("auto").orElseThrow().choose("Left");
    Devices.Card asked = card();
    autos.update(false);
    Devices.Card enabled = card();
    autos.update(true);
    Devices.Card taken = card();

    assertThat(said).startsWith("Chose Left");
    assertThat(asked.choices().get(0).current()).isEqualTo("Left");
    assertThat(asked.choices().get(0).options())
        .extracting(Devices.Option::value)
        .containsExactly("Do nothing", "Left");
    assertThat(enabled.state()).as("enabled: still the one it had").isEqualTo("Do nothing");
    assertThat(enabled.lines()).extracting(Devices.Line::label).contains("Chosen");
    assertThat(taken.state()).isEqualTo("Left");
    assertThat(taken.lines())
        .contains(new Devices.Line("Starts on", "Left start", Devices.Level.NOTE));
    assertThat(taken.lines()).extracting(Devices.Line::label).doesNotContain("Chosen");
  }

  @Test
  void anAutoTheRobotDoesntHaveIsRefusedSayingWhich() {
    assertThatThrownBy(() -> device.chooser("auto").orElseThrow().choose("Right"))
        .isInstanceOf(Devices.Refused.class)
        .hasMessage("there's no auto Right on the robot: it has Do nothing, Left");
    assertThat(autos.requested()).isEqualTo("Do nothing");
  }
}

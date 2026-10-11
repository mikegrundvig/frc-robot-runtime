package com.michaelgrundvig.frc.runtime.examples;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.AutoCheck;
import com.michaelgrundvig.frc.runtime.ExampleControls;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/**
 * The guide's robot has two autos of its own and the one the template adds: all run on each
 * alliance and pass the template's own check, as docs/autos.md says to test an auto.
 */
class ExampleAutosSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void theGuidesAutosAndDoNothingPassTheTemplatesCheck() {
    var results = AutoCheck.check(sim, ExampleRobot.COMPETITION, ExampleControls::new);

    assertThat(results)
        .extracting(AutoCheck.Result::auto)
        .containsOnly("Do nothing", "Grab one", "Grab and shoot");
    assertThat(results).hasSize(6);
    for (AutoCheck.Result result : results) {
      if (result.auto().equals("Do nothing")) {
        assertThat(result.moves()).as("moves nothing").isEmpty();
      } else {
        assertThat(result.moves())
            .anyMatch(line -> line.startsWith("drive → Pickup: ARRIVED in "))
            .anyMatch(line -> line.startsWith("pivot → in: ARRIVED in "));
      }
      if (result.auto().equals("Grab and shoot")) {
        assertThat(result.moves())
            .as("the second leg, and a feed that ran")
            .anyMatch(line -> line.startsWith("drive → Shot: ARRIVED in "))
            .anyMatch(line -> line.startsWith("at feeding: CANCELED after "));
      }
    }
  }
}

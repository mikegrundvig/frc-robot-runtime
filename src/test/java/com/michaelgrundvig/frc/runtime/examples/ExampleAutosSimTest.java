package com.michaelgrundvig.frc.runtime.examples;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.AutoCheck;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/**
 * The guide's robot has one auto of its own and the one the template adds: both run on each
 * alliance and pass the template's own check, as docs/autos.md says to test an auto.
 */
class ExampleAutosSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void theGuidesAutoAndDoNothingPassTheTemplatesCheck() {
    var results = AutoCheck.check(sim, ExampleRobot.COMPETITION, Controls::new);

    assertThat(results).extracting(AutoCheck.Result::auto).containsOnly("Do nothing", "Grab one");
    assertThat(results).hasSize(4);
    for (AutoCheck.Result result : results) {
      if (result.auto().equals("Do nothing")) {
        assertThat(result.moves()).as("moves nothing").isEmpty();
      } else {
        assertThat(result.moves())
            .anyMatch(line -> line.startsWith("drive → Pickup: ARRIVED in "))
            .anyMatch(line -> line.startsWith("pivot → in: ARRIVED in "));
      }
    }
  }
}

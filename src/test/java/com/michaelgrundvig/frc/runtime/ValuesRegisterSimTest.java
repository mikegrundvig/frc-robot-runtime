package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.TunableTee;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/**
 * The drive team's switches against the values register (docs/values.md): every WPILib tunable the
 * running robot publishes, apart from its configuration, has a row saying where it came from and
 * why. Switches are made by the mechanisms as the robot is built, so this builds the robot.
 */
@SimulationTest
class ValuesRegisterSimTest {
  private static final Path REGISTER =
      Path.of(System.getProperty("frc.projectDir", "."), "docs", "values.md");

  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void everySwitchTheRobotPublishesHasARow() throws IOException {
    sim.robot();
    sim.loop(); // the tunables update, as on a robot
    List<String> cameras =
        ExampleRobot.SIM.definition().cameras().stream().map(CameraConfig::name).toList();
    Set<String> switches = new TreeSet<>();
    for (TunableTee.Listed listed : sim.pitwall().tunables().view()) {
      switches.add(generic(listed.path(), cameras));
    }

    assertThat(switches).contains("Vision/autoStartHoldSeconds", "Vision/<camera>/enabled");
    assertThat(ValuesRegister.parse(Files.readString(REGISTER)).switchProblems(switches)).isEmpty();
  }

  /**
   * A switch's path as the register names it: no leading slash, and a camera as {@code <camera>}.
   */
  private static String generic(String path, List<String> cameras) {
    List<String> parts = new ArrayList<>();
    for (String part : path.replaceFirst("^/", "").split("/", -1)) {
      parts.add(cameras.contains(part) ? "<camera>" : part);
    }
    return String.join("/", parts);
  }
}

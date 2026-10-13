package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.vision.camera.MjpegServer;
import org.wpilib.vision.stream.CameraServer;

/**
 * A simulated camera serves no video. PhotonVision's camera simulation starts two video servers on
 * the next free ports from 1181, on every address of the computer, whether or not anything views
 * the streams: a test run took three hundred ports that way, got in the way of any other run beside
 * it, and offered its cameras' pictures to the network. {@link VisionSim} closes them.
 *
 * <p>The ports are not looked at: another process may hold them, and a server that could not bind
 * would then look closed. The servers are looked at.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class VisionSimVideoServersSimTest {
  @Test
  void aCameraAddedToTheSimulationHasNoVideoServers() {
    CameraConfig config = ExampleRobot.SIM.definition().cameras().get(0);
    try (VisionSim vision =
        new VisionSim(RobotSim.FIELD, Map.of(), camera -> Seeds.camera(1, camera))) {
      ((CameraIOPhotonVision) vision.camera(config)).camera().close();

      assertThat(CameraServer.getServer("serve_" + config.name() + "-raw")).isNull();
      assertThat(CameraServer.getServer("serve_" + config.name() + "-processed")).isNull();
    }
  }

  @Test
  void closeVideoServersClosesAndForgetsBoth() {
    try (MjpegServer raw = CameraServer.addServer("serve_probe-raw");
        MjpegServer processed = CameraServer.addServer("serve_probe-processed")) {
      VisionSim.closeVideoServers("probe");

      assertThat(raw.isValid()).as("the raw stream's server").isFalse();
      assertThat(processed.isValid()).as("the processed stream's server").isFalse();
      assertThat(CameraServer.getServer("serve_probe-raw")).isNull();
      assertThat(CameraServer.getServer("serve_probe-processed")).isNull();
    }
  }
}

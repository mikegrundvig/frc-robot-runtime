package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.CleanupMode;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * A recorded session does not open NetworkTables' port ({@link SessionRecording}). WPILib's robot
 * constructor starts a server on port 5810 and announces it as "robot", and left to, every test
 * that recorded a session would have held that port for as long as it ran: a dashboard on the
 * computer could have connected to the test's robot and changed its tunables, a simulator a person
 * had open would have been beaten to the port or beaten the test to it, and two sessions at once
 * would have collided.
 *
 * <p>The test looks at the sockets of the process the session runs in, from the moment it starts to
 * the moment it ends, and not at whether the port is free, so it passes whether or not something
 * else on the computer holds 5810. It also requires to have seen the session's own page listen, so
 * that a watcher that saw nothing could not pass for one that saw no server.
 */
@EndToEndTest
class SessionStaysOffTheNetworkEndToEndTest {
  /** Pitwall's address, which the session prints as it starts. */
  private static final Pattern PITWALL_PORT = Pattern.compile("Pitwall: http://localhost:(\\d+)");

  @TempDir(cleanup = CleanupMode.ON_SUCCESS) // a failed run keeps its logs to look at
  Path dir;

  /** A session, by the folder it is recorded in and the watcher it tells of its process. */
  @FunctionalInterface
  private interface Session {
    Path record(Path dir, Consumer<ProcessHandle> watcher) throws IOException, InterruptedException;
  }

  @Test
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  void theSessionWithoutControlsOpensNoNetworkTablesServer() throws Exception {
    opensNoNetworkTablesServer(SessionRecording::record);
  }

  @Test
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  void theSessionWithControlsOpensNoNetworkTablesServer() throws Exception {
    opensNoNetworkTablesServer(SessionRecording::recordWithControls);
  }

  private void opensNoNetworkTablesServer(Session session) throws Exception {
    assumeTrue(ListeningSockets.available(), "this computer cannot say what a process listens on");

    ListeningSockets.Watch watch = new ListeningSockets.Watch();
    session.record(dir, watch::start);
    Set<Integer> listened = watch.portsSeen();

    String output = Files.readString(dir.resolve("record.out"));
    Matcher pitwall = PITWALL_PORT.matcher(output);
    assertThat(pitwall.find()).as("Pitwall's address in the session's output:\n" + output).isTrue();
    assertThat(listened)
        .as("the ports the session listened on, which include Pitwall's page")
        .contains(Integer.parseInt(pitwall.group(1)));
    assertThat(listened)
        .as("the ports the session listened on")
        .doesNotContain(NetworkTableInstance.DEFAULT_PORT);
    // NetworkTables says so when it starts a server, and when it announces one on the network.
    assertThat(output).doesNotContain("NT: Listening").doesNotContain("mDNS announcing");
  }
}

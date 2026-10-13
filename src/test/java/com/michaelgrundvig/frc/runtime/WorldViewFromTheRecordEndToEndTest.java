package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.AdvantageKitLog;
import com.michaelgrundvig.frc.pitwall.push.Feed;
import com.michaelgrundvig.frc.pitwall.robot.RobotSession;
import com.michaelgrundvig.frc.pitwall.scene.Scenes;
import com.michaelgrundvig.frc.pitwall.scene.StillState;
import com.michaelgrundvig.frc.pitwall.world.WorldScene;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.CleanupMode;
import org.junit.jupiter.api.io.TempDir;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

/**
 * Pitwall's World view, drawn from the robot's record, shows what the robot's own objects said: for
 * every loop of a real session, the robot where localization had it, the ellipse as its covariance
 * has it, the ghost where the simulator had the robot, the wheels as the drivetrain measured them,
 * and a sight line for each tag the estimator weighed, drawn as it fared.
 *
 * <p>The session is the guide's robot driven about the field, two of its cameras blinded, and then
 * shoved ({@link SessionRecording#recordTheWorld}), in a process of its own. Every loop it logs
 * what its own objects say, as a witness, beside the loop's record. Here each loop's record is
 * handed to Pitwall's own receiver, the view is drawn from the moment that makes, and the drawing
 * is held to the witness. The view is drawn from the log's keys alone, so a key the runtime stopped
 * writing, or wrote another way, fails here.
 *
 * <p>It is the check that stood between the view's two implementations while there were two: the
 * runtime's layers, which read those objects live, said the same in every loop of this session,
 * value for value, before they were removed.
 */
@EndToEndTest
class WorldViewFromTheRecordEndToEndTest {
  /**
   * How far apart two numbers may be: the session's memory keeps a pose and a spread as 32-bit
   * numbers, good to a few micrometers across a field.
   */
  private static final double CLOSE = 1e-4;

  /**
   * How far ahead a wheel's arrow reaches, in seconds: the arrow's length is its speed times it.
   */
  private static final double ARROW_SECONDS = 0.2;

  /** How many loops a sight line stays after its measurement: 300 ms of 20 ms loops. */
  private static final int LOOPS_HELD = 15;

  /** The farthest the ellipse is drawn, in meters. */
  private static final double MAX_RADIUS = 20;

  private static final String WITNESS = "/RealOutputs/Witness/World/";
  private static final List<String> MODULES =
      List.of("front-left", "front-right", "back-left", "back-right");
  private static final RobotDefinition DEFINITION = ExampleRobot.COMPETITION.definition();
  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  @TempDir(cleanup = CleanupMode.ON_SUCCESS) // a failed run keeps its log to look at
  static Path dir;

  private static final RobotSession session =
      RobotSession.of(
          DEFINITION,
          RobotLoop.PERIOD_SECONDS,
          RobotSession.REMEMBERED_SECONDS,
          Vision.sightingWords());
  private static final WorldScene world =
      WorldScene.of(
          DEFINITION,
          () -> RobotSim.FIELD,
          WorldScene.Truth.IN_THE_RECORD,
          Vision.sightingWords(),
          camera -> Optional.empty());

  /** The same view on a page, sent live from the session's newest loop. */
  private static final AtomicLong clock = new AtomicLong();

  private static final Scenes scenes = new Scenes(clock::get, System.out::println);

  /** Every difference found, each with its loop: none, if the drawing is the witness's. */
  private static final List<String> differences = new ArrayList<>();

  /** Each camera's latest measurement of each tag, as the witness said it: its loop and how. */
  private static final Map<String, int[]> lastMeasured = new HashMap<>();

  private static final Map<String, String> lastFared = new HashMap<>();

  private static int loopsCompared;
  private static final Set<String> lineEndings = new TreeSet<>();
  private static final Set<String> rests = new TreeSet<>();
  private static double farthestApartMeters;
  private static int recoveries;

  /** The size of each patch the page was sent, by the loop it was sent in. */
  private static final Map<Integer, Integer> patchBytes = new HashMap<>();

  private static final List<String> patches = new ArrayList<>();
  private static int descriptionBytes;

  @BeforeAll
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  static void recordTheSessionAndDrawEveryLoopOfItFromTheRecord() throws Exception {
    Path recorded = SessionRecording.recordTheWorld(dir);
    WPILOGReader reader = new WPILOGReader(recorded.toString());
    reader.start();
    LogDataReceiver receiver = AdvantageKitLog.receiver(session);
    scenes.add(world.scene(session));
    Feed live = scenes.feeds().getFirst();
    descriptionBytes = JSON.toJson(scenes.descriptions()).length();
    LogTable table = new LogTable(0);
    int loop = 0;
    while (reader.updateTable(table)) {
      // AdvantageKit hands a receiver a copy of the table each loop, as here.
      receiver.putTable(LogTable.clone(table));
      loop++;
      clock.set(table.getTimestamp());
      Map<String, ?> patch = live.patch();
      if (!patch.isEmpty()) {
        String sent = JSON.toJson(patch);
        patches.add(sent);
        patchBytes.put(loop, sent.getBytes(StandardCharsets.UTF_8).length);
      }
      Map<String, LogTable.LogValue> logged = table.getAll(false);
      if (logged.containsKey(WITNESS + "Estimate")) {
        compare(loop, logged, world.draw(session.newest().orElseThrow()));
        loopsCompared++;
      }
    }
  }

  /** What the witness logged under a name, in a loop that has a witness. */
  private static LogTable.LogValue witness(Map<String, LogTable.LogValue> logged, String name) {
    return Objects.requireNonNull(logged.get(WITNESS + name), name);
  }

  private static void differ(int loop, String what, double witness, double drawn) {
    differences.add(
        String.format(
            Locale.ROOT,
            "loop %d: %s: the robot said %.6f, drawn %.6f",
            loop,
            what,
            witness,
            drawn));
  }

  /** Holds a frame's drawn pose to what the witness said: x, y, z, roll, pitch and yaw. */
  private static void comparePose(int loop, String frame, double[] witness, StillState drawn) {
    double[] pose = drawn.poses().get(frame);
    if (pose == null) {
      differences.add("loop " + loop + ": frame " + frame + " is not drawn");
      return;
    }
    for (int part = 0; part < 6; part++) {
      double apart = Math.abs(witness[part] - pose[part]);
      if (part >= 3) {
        apart = Math.abs(Math.IEEEremainder(witness[part] - pose[part], 2 * Math.PI));
      }
      if (apart > CLOSE) {
        differ(loop, "frame " + frame + " part " + part, witness[part], pose[part]);
      }
    }
  }

  private static void compareValue(int loop, String name, double witness, StillState drawn) {
    Double value = drawn.values().get(name);
    boolean same = value != null && Math.abs(witness - value) <= CLOSE * Math.max(1, witness);
    if (!same) {
      differ(loop, "value " + name, witness, value == null ? Double.NaN : value);
    }
  }

  /** Holds the view drawn from a loop's record to what the robot's own objects said in it. */
  private static void compare(int loop, Map<String, LogTable.LogValue> logged, StillState drawn) {
    double[] estimate = witness(logged, "Estimate").getDoubleArray();
    double[] truth = witness(logged, "Truth").getDoubleArray();
    comparePose(loop, "robot", estimate, drawn);
    comparePose(loop, "truth", truth, drawn);
    farthestApartMeters =
        Math.max(farthestApartMeters, Math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]));

    double[] modules = witness(logged, "Modules").getDoubleArray();
    for (int module = 0; module < MODULES.size(); module++) {
      String name = MODULES.get(module);
      double[] turned = drawn.poses().getOrDefault("module." + name, new double[6]);
      double apart = Math.abs(Math.IEEEremainder(modules[2 * module + 1] - turned[5], 2 * Math.PI));
      if (apart > CLOSE) {
        differ(loop, "wheel " + name + " angle", modules[2 * module + 1], turned[5]);
      }
      compareValue(loop, "speed." + name, modules[2 * module] * ARROW_SECONDS, drawn);
    }

    // The ellipse reaches two standard deviations along the covariance's own axes. The longer is
    // its largest standard deviation; the two variances add up to the covariance's trace.
    double[] spread = witness(logged, "Spread").getDoubleArray();
    double[] ellipse = drawn.poses().getOrDefault("uncertainty", new double[6]);
    if (Math.hypot(ellipse[0] - estimate[0], ellipse[1] - estimate[1]) > CLOSE) {
      differ(loop, "the ellipse's place along x", estimate[0], ellipse[0]);
    }
    double major = Math.min(MAX_RADIUS, 2 * spread[0]);
    double minorVariance = Math.max(0, spread[1] - spread[0] * spread[0]);
    compareValue(loop, "uncertainty.major", major, drawn);
    compareValue(
        loop, "uncertainty.minor", Math.min(MAX_RADIUS, 2 * Math.sqrt(minorVariance)), drawn);

    String restsOn = witness(logged, "Rests").getString();
    rests.add(restsOn);
    boolean wrong = restsOn.equals("LOST") || restsOn.equals("IMU_FAULT");
    boolean drawnWrong = "bad dashed".equals(drawn.styles().get("uncertainty.ellipse"));
    if (wrong != drawnWrong) {
      differences.add("loop " + loop + ": resting on " + restsOn + ", styles " + drawn.styles());
    }
    recoveries = (int) witness(logged, "Recoveries").getInteger();
    compareLines(loop, witness(logged, "Measured").getStringArray(), drawn);
  }

  /**
   * A sight line is drawn for each tag a camera measured in this loop or in the {@link #LOOPS_HELD}
   * before it, as its latest measurement fared, and for nothing else.
   */
  private static void compareLines(int loop, String[] measured, StillState drawn) {
    for (String each : measured) {
      String sight = each.substring(0, each.lastIndexOf('.'));
      lastMeasured.put(sight, new int[] {loop});
      lastFared.put(sight, each.substring(each.lastIndexOf('.') + 1));
    }
    Set<String> expected = new TreeSet<>();
    lastMeasured.forEach(
        (sight, when) -> {
          if (loop - when[0] <= LOOPS_HELD) {
            expected.add("ray." + sight + "." + lastFared.get(sight));
          }
        });
    Set<String> lines = new TreeSet<>();
    for (Map<String, Object> line :
        drawn.shapesOfTheMoment().getOrDefault("sight-lines", List.of())) {
      lines.add(String.valueOf(line.get("id")));
      lineEndings.add(String.valueOf(line.get("style")));
    }
    if (!lines.equals(expected)) {
      differences.add("loop " + loop + ": sight lines: measured " + expected + ", drawn " + lines);
    }
  }

  @Test
  void everyLoopDrawnFromTheRecordIsWhatTheRobotsOwnObjectsSaid() {
    System.out.printf(
        Locale.ROOT,
        "World view from the record against the robot's own objects: %d loops compared; sight"
            + " lines drawn as %s; the estimate rested on %s; the estimate and the truth at most"
            + " %.2f m apart; %d recoveries; %d differences%n",
        loopsCompared,
        lineEndings,
        rests,
        farthestApartMeters,
        recoveries,
        differences.size());
    assertThat(loopsCompared).as("loops compared").isGreaterThan(500);
    assertThat(differences.stream().limit(20).toList())
        .as("differences between the drawing and the robot's objects, of " + differences.size())
        .isEmpty();
  }

  /** The comparison means something only if the session made the view move in every way. */
  @Test
  void theSessionMovedEverythingTheViewDraws() {
    assertThat(lineEndings).as("accepted and refused sight lines").contains("ok", "bad dashed");
    assertThat(rests).contains("TRACKING", "DEAD_RECKONING", "LOST");
    assertThat(farthestApartMeters)
        .as("the shove parted the estimate and the truth")
        .isGreaterThan(0.5);
    assertThat(recoveries).as("the estimate recovered").isPositive();
  }

  /**
   * What the view costs to send, live, on this robot: its description once, nothing while the robot
   * stands still, and while it drives at most 20 patches a second, far under the 250 kbps the
   * design allows a scene.
   */
  @Test
  void liveTheViewCostsLittleToSendAndNothingWhenStill() {
    int[] driving = {30, 190};
    int[] still = {440, 479};
    int patchesDriving = 0;
    int patchesStill = 0;
    long bytesDriving = 0;
    for (Map.Entry<Integer, Integer> sent : patchBytes.entrySet()) {
      int loop = sent.getKey();
      if (loop >= driving[0] && loop < driving[1]) {
        patchesDriving++;
        bytesDriving += sent.getValue();
      }
      if (loop >= still[0] && loop < still[1]) {
        patchesStill++;
      }
    }
    double seconds = (driving[1] - driving[0]) * RobotLoop.PERIOD_SECONDS;
    double kbps = bytesDriving * 8 / seconds / 1000;
    System.out.printf(
        Locale.ROOT,
        "World view, live: description %d B; driving: %d patches in %.1f s, %.1f kbps; still: %d"
            + " patches in %d loops%n",
        descriptionBytes,
        patchesDriving,
        seconds,
        kbps,
        patchesStill,
        still[1] - still[0]);
    assertThat(descriptionBytes).as("the scene's description, once").isLessThan(60_000);
    assertThat(patchesStill).as("patches while the robot stood still").isZero();
    assertThat(patchesDriving).as("patches while driving").isGreaterThan(20);
    assertThat(patchesDriving / seconds).as("patches a second").isLessThanOrEqualTo(20);
    assertThat(kbps).as("kbps while driving").isLessThan(250);
    assertThat(patches).anyMatch(patch -> patch.contains("\"sight-lines\":"));
  }
}

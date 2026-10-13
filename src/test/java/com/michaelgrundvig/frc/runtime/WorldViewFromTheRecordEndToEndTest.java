package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.AdvantageKitLog;
import com.michaelgrundvig.frc.pitwall.robot.Moment;
import com.michaelgrundvig.frc.pitwall.robot.RobotSession;
import com.michaelgrundvig.frc.pitwall.scene.StillState;
import com.michaelgrundvig.frc.pitwall.world.WorldScene;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.vision.Vision;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.CleanupMode;
import org.junit.jupiter.api.io.TempDir;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

/**
 * Pitwall's World view, drawn from the robot's record, is the view the runtime's live layers draw
 * from the robot's own objects: for every loop of a real session, frame for frame, value for value,
 * style for style and sight line for sight line.
 *
 * <p>The session is the guide's robot driven about the field and then shoved ({@link
 * SessionRecording#recordTheWorld}), in a process of its own. Every loop, a witness has the live
 * layers say what moves and logs it beside the loop's record. Here each loop's record is handed to
 * Pitwall's own receiver, the view is drawn from the moment that makes, and the two are compared.
 */
@EndToEndTest
class WorldViewFromTheRecordEndToEndTest {
  /**
   * How far apart two numbers may be: the session's memory keeps a pose and a spread as 32-bit
   * numbers, good to a few micrometers across a field.
   */
  private static final double CLOSE = 1e-4;

  private static final String WITNESS = "/RealOutputs/Witness/World/";
  private static final RobotDefinition DEFINITION = ExampleRobot.COMPETITION.definition();

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

  /** Every difference found, each with its loop: none, if the two views are one. */
  private static final List<String> differences = new ArrayList<>();

  private static int loopsCompared;
  private static final Set<String> framesCompared = new TreeSet<>();
  private static final Set<String> valuesCompared = new TreeSet<>();
  private static final Set<String> lineEndings = new TreeSet<>();
  private static final Set<String> stylesSeen = new TreeSet<>();
  private static double farthestApartMeters;
  private static final List<String> restedOn = new ArrayList<>();

  @BeforeAll
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  static void recordTheSessionAndDrawEveryLoopOfItFromTheRecord() throws Exception {
    Path recorded = SessionRecording.recordTheWorld(dir);
    WPILOGReader reader = new WPILOGReader(recorded.toString());
    reader.start();
    LogDataReceiver receiver = AdvantageKitLog.receiver(session);
    LogTable table = new LogTable(0);
    int loop = 0;
    while (reader.updateTable(table)) {
      // AdvantageKit hands a receiver a copy of the table each loop, as here.
      receiver.putTable(LogTable.clone(table));
      loop++;
      Map<String, LogTable.LogValue> logged = table.getAll(false);
      if (logged.containsKey(WITNESS + "Frames")) {
        compare(loop, logged, world.draw(session.newest().orElseThrow()));
        loopsCompared++;
      }
    }
  }

  /** Compares what the live layers said in a loop with the view drawn from the loop's record. */
  private static void compare(int loop, Map<String, LogTable.LogValue> logged, StillState drawn) {
    String[] frames = logged.get(WITNESS + "Frames").getStringArray();
    double[] poses = logged.get(WITNESS + "Poses").getDoubleArray();
    for (int each = 0; each < frames.length; each++) {
      double[] fromTheRecord = drawn.poses().get(frames[each]);
      framesCompared.add(frames[each]);
      if (fromTheRecord == null) {
        differences.add("loop " + loop + ": frame " + frames[each] + " is not drawn");
        continue;
      }
      for (int part = 0; part < 6; part++) {
        double live = poses[each * 6 + part];
        boolean angle = part >= 3;
        double apart = Math.abs(live - fromTheRecord[part]);
        if (angle) {
          apart = Math.abs(Math.IEEEremainder(live - fromTheRecord[part], 2 * Math.PI));
        }
        if (angle && frames[each].equals("uncertainty")) {
          apart = outlineMovedBy(apart, drawn);
        }
        if (apart > CLOSE) {
          differences.add(
              String.format(
                  Locale.ROOT,
                  "loop %d: frame %s part %d: live %.6f, from the record %.6f",
                  loop,
                  frames[each],
                  part,
                  live,
                  fromTheRecord[part]));
        }
      }
    }
    // What rides on a joint is drawn where the joint is, which the live layers never did.
    for (String frame : drawn.poses().keySet()) {
      boolean carried = frame.startsWith("camera.") || frame.startsWith("sensor.");
      if (!carried && !List.of(frames).contains(frame)) {
        differences.add("loop " + loop + ": frame " + frame + " is drawn, and live it is not");
      }
    }
    String[] names = logged.get(WITNESS + "ValueNames").getStringArray();
    double[] values = logged.get(WITNESS + "Values").getDoubleArray();
    for (int each = 0; each < names.length; each++) {
      Double fromTheRecord = drawn.values().get(names[each]);
      valuesCompared.add(names[each]);
      boolean same =
          fromTheRecord != null
              && Math.abs(values[each] - fromTheRecord) <= CLOSE * Math.max(1, values[each]);
      if (!same) {
        differences.add(
            "loop "
                + loop
                + ": value "
                + names[each]
                + ": live "
                + values[each]
                + ", from the record "
                + fromTheRecord);
      }
    }
    List<String> styles = new ArrayList<>();
    drawn.styles().forEach((shape, style) -> styles.add(shape + "=" + style));
    List<String> liveStyles = List.of(logged.get(WITNESS + "Styles").getStringArray());
    stylesSeen.addAll(liveStyles);
    if (!new TreeSet<>(styles).equals(new TreeSet<>(liveStyles))) {
      differences.add("loop " + loop + ": styles: live " + liveStyles + ", drawn " + styles);
    }
    Set<String> lines = new TreeSet<>();
    for (Map<String, Object> line :
        drawn.shapesOfTheMoment().getOrDefault("sight-lines", List.of())) {
      lines.add(line.get("id") + " " + line.get("style") + " " + line.get("between"));
      lineEndings.add(String.valueOf(line.get("style")));
    }
    Set<String> liveLines = new TreeSet<>(List.of(logged.get(WITNESS + "Lines").getStringArray()));
    if (!lines.equals(liveLines)) {
      differences.add("loop " + loop + ": sight lines: live " + liveLines + ", drawn " + lines);
    }
    Moment.OfWorld where = session.newest().orElseThrow().world();
    where.estimate().ifPresent(estimate -> restedOn.add(estimate.rests() + "@" + loop));
    if (where.estimate().isPresent() && where.truth().isPresent()) {
      double apart =
          where
              .estimate()
              .get()
              .pose()
              .getTranslation()
              .getDistance(where.truth().get().getTranslation());
      farthestApartMeters = Math.max(farthestApartMeters, apart);
    }
  }

  /**
   * How far an ellipse's outline moves when the ellipse is turned by an angle: the angle times how
   * much longer than wide it is. The way a nearly round ellipse lies is decided by differences too
   * small for a 32-bit number to hold, and turning a round ellipse moves nothing. An ellipse is an
   * ellipse again after half a turn.
   */
  private static double outlineMovedBy(double radians, StillState drawn) {
    double turned = Math.abs(Math.IEEEremainder(radians, Math.PI));
    double major = drawn.values().getOrDefault("uncertainty.major", 0.0);
    double minor = drawn.values().getOrDefault("uncertainty.minor", 0.0);
    return turned * (major - minor);
  }

  @Test
  void everyLoopDrawnFromTheRecordIsWhatTheLiveLayersSaid() {
    System.out.printf(
        Locale.ROOT,
        "World view from the record against the live layers: %d loops compared; frames %s; values"
            + " %s; sight lines drawn as %s; styles %s; the estimate and the truth at most %.2f m"
            + " apart; %d differences%n",
        loopsCompared,
        framesCompared,
        valuesCompared,
        lineEndings,
        stylesSeen,
        farthestApartMeters,
        differences.size());
    String last = "";
    for (String each : restedOn) {
      String rests = each.substring(0, each.indexOf('@'));
      if (!rests.equals(last)) {
        System.out.println("  rests on " + each);
        last = rests;
      }
    }
    assertThat(loopsCompared).as("loops compared").isGreaterThan(500);
    assertThat(differences.stream().limit(20).toList())
        .as("differences between the two views, of " + differences.size())
        .isEmpty();
  }

  /** The comparison means something only if the session made the view move in every way. */
  @Test
  void theSessionMovedEverythingTheViewDraws() {
    assertThat(framesCompared)
        .contains("robot", "uncertainty", "truth", "module.front-left", "module.back-right");
    assertThat(valuesCompared)
        .contains("uncertainty.major", "uncertainty.minor", "speed.front-left");
    assertThat(lineEndings).as("accepted and refused sight lines").contains("ok", "bad dashed");
    assertThat(farthestApartMeters)
        .as("the shove parted the estimate and the truth")
        .isGreaterThan(0.5);
    assertThat(session.newest().orElseThrow().world().estimate().orElseThrow().recoveries())
        .as("the estimate recovered")
        .isPositive();
  }
}

package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.SoftAssertions.assertSoftly;

import com.michaelgrundvig.frc.pitwall.AdvantageKitLog;
import com.michaelgrundvig.frc.pitwall.robot.LogKeys;
import com.michaelgrundvig.frc.pitwall.robot.Moment;
import com.michaelgrundvig.frc.pitwall.robot.RobotSession;
import com.michaelgrundvig.frc.world.record.JointState;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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
 * The keys Pitwall's Robot view reads are the keys the robot really logs, each of the type Pitwall
 * reads it as. Pitwall lists them in one place ({@link LogKeys}), and nothing else ties that list
 * to the runtime's logging: a key renamed here, or a fact given another shape, would leave the page
 * drawing nothing, with every test of either side still passing.
 *
 * <p>So a real session's log is read back: the guide's robot, with the guide's controls, runs in
 * the simulator in a process of its own and records its log ({@link
 * SessionRecording#recordWithControls}). In that session the drive team deploys and stows the
 * pivot, so two moves end, and holds the aimed shot, so a gate goes false and then true, and each
 * press fires its binding. Every loop of that log is handed to Pitwall's own receiver, and every
 * key in the list is looked for in it, for every joint, sensor and gamepad, and for what a move, a
 * gate and a binding write once they have happened. Each of those must be there, and not only be of
 * the right type if it is.
 */
@EndToEndTest
class RobotViewKeysEndToEndTest {
  /** What the log holds under a key: one of AdvantageKit's types, and for a fact its struct. */
  private record Logged(LogTable.LoggableType type, String struct) {
    static Logged plain(LogTable.LoggableType type) {
      return new Logged(type, "");
    }

    static Logged fact(String struct) {
      return new Logged(LogTable.LoggableType.Raw, "struct:" + struct);
    }

    static Logged of(LogTable.LogValue value) {
      String custom = value.customTypeStr == null ? "" : value.customTypeStr;
      return new Logged(value.type, custom);
    }
  }

  private static final Logged FLAG = Logged.plain(LogTable.LoggableType.Boolean);
  private static final Logged NUMBER = Logged.plain(LogTable.LoggableType.Double);
  private static final Logged TEXT = Logged.plain(LogTable.LoggableType.String);
  private static final Logged TEXTS = Logged.plain(LogTable.LoggableType.StringArray);

  /** Every joint's inputs, as Pitwall reads them: logged from the first loop. */
  private static final Map<String, Logged> JOINT_INPUTS =
      Map.of(
          LogKeys.CONNECTED, FLAG,
          LogKeys.FRESH, FLAG,
          LogKeys.POSITION, NUMBER,
          LogKeys.VELOCITY, NUMBER,
          LogKeys.APPLIED_VOLTS, NUMBER,
          LogKeys.MAGNET_BAD, FLAG);

  /** Every joint's outputs, as Pitwall reads them: logged from the first loop. */
  private static final Map<String, Logged> JOINT_OUTPUTS =
      Map.of(
          LogKeys.STATE, Logged.fact("JointState"),
          LogKeys.FAULT, TEXT,
          LogKeys.OWNER, TEXT,
          LogKeys.COMMAND, TEXT,
          LogKeys.GOAL, NUMBER,
          LogKeys.GOAL_NAME, TEXT,
          LogKeys.AT, Logged.fact("JointPlace"),
          LogKeys.HELD_BY, Logged.fact("Holder[]"),
          LogKeys.SETPOINT, NUMBER,
          LogKeys.SETPOINT_VELOCITY, NUMBER);

  /**
   * What a joint logs once a move on it has ended. The session ends two moves on the pivot, so it
   * must have these, and no other joint is held to them.
   */
  private static final Map<String, Logged> JOINT_OUTPUTS_AFTER_A_MOVE_ENDS =
      Map.of(
          LogKeys.OUTCOME,
          Logged.fact("MoveOutcome[]"),
          LogKeys.OUTCOME_NAME,
          TEXT,
          LogKeys.OUTCOME_WHY,
          TEXT,
          LogKeys.OUTCOME_HELD_BY,
          Logged.fact("Holder[]"));

  /** The joints a move ends on in the session. */
  private static final Set<String> JOINTS_A_MOVE_ENDS_ON = Set.of("pivot");

  /** What a joint with an absolute encoder logs: where the encoder says the joint is. */
  private static final Map<String, Logged> JOINT_WITH_AN_ABSOLUTE_ENCODER =
      Map.of(LogKeys.ABSOLUTE_AT, NUMBER);

  private static final Map<String, Logged> SENSOR =
      Map.of(LogKeys.SENSOR_CONNECTED, FLAG, LogKeys.TRIPPED, FLAG, LogKeys.METERS, NUMBER);

  private static final Map<String, Logged> GAMEPAD =
      Map.of(
          LogKeys.BUTTONS, Logged.plain(LogTable.LoggableType.Integer),
          LogKeys.AXES, Logged.plain(LogTable.LoggableType.FloatArray),
          LogKeys.POVS, Logged.plain(LogTable.LoggableType.IntegerArray));

  private static final Map<String, Logged> ROBOT =
      Map.of(
          LogKeys.ENABLED, FLAG,
          LogKeys.ROBOT_MODE, TEXT,
          LogKeys.DRIVER_STATION_ATTACHED, FLAG,
          LogKeys.EMERGENCY_STOP, FLAG,
          LogKeys.FIRED, Logged.fact("BindingFired[]"),
          LogKeys.BINDINGS, TEXTS);

  /** The gates the guide's controls make, which the session must log. */
  private static final List<String> GATES_THE_CONTROLS_MAKE = List.of("piece in", "ready to feed");

  /** A gate's own keys beside whether it holds. */
  private static final Map<String, Logged> GATE =
      Map.of(LogKeys.BLOCKING, Logged.fact("GateTerm[]"), LogKeys.BLOCKING_TEXT, TEXTS);

  @TempDir(cleanup = CleanupMode.ON_SUCCESS) // a failed run keeps its log to look at
  static Path dir;

  /** What the last loop of the session's log holds, by key as the log names it. */
  private static final Map<String, Logged> logged = new LinkedHashMap<>();

  /**
   * What some keys held over the session, each change in the order it came: a boolean as "true" or
   * "false", a text as itself, and an array of texts or facts as "none" while empty and "some" once
   * it holds any. A key that is written again with the same value adds nothing.
   */
  private static final Map<String, List<String>> changes = new LinkedHashMap<>();

  private static final RobotDefinition DEFINITION = ExampleRobot.COMPETITION.definition();
  private static final RobotSession session = RobotSession.of(DEFINITION);
  private static int loops;

  @BeforeAll
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  static void recordTheSessionAndHandItsLogToPitwall() throws Exception {
    Path recorded = SessionRecording.recordWithControls(dir);
    WPILOGReader reader = new WPILOGReader(recorded.toString());
    reader.start();
    LogDataReceiver receiver = AdvantageKitLog.receiver(session);
    LogTable table = new LogTable(0);
    while (reader.updateTable(table)) {
      // AdvantageKit hands a receiver a copy of the table each loop, as here.
      receiver.putTable(LogTable.clone(table));
      loops++;
      table.getAll(false).forEach(RobotViewKeysEndToEndTest::noticeIfItChanged);
    }
    table.getAll(false).forEach((key, value) -> logged.put(key, Logged.of(value)));
  }

  /** Remembers what a key holds, if that is one of the kinds the session is asked about. */
  private static void noticeIfItChanged(String key, LogTable.LogValue value) {
    String now =
        switch (value.type) {
          case Boolean -> String.valueOf(value.getBoolean());
          case String -> value.getString();
          case StringArray -> emptyOrSome(value.getStringArray().length);
          case Raw -> emptyOrSome(value.getRaw().length);
          default -> null;
        };
    if (now == null) {
      return;
    }
    List<String> seen = changes.computeIfAbsent(key, each -> new ArrayList<>());
    if (seen.isEmpty() || !seen.getLast().equals(now)) {
      seen.add(now);
    }
  }

  private static String emptyOrSome(int length) {
    return length == 0 ? "none" : "some";
  }

  /** Every key Pitwall reads is in the log, of the type Pitwall reads it as. */
  @Test
  void everyKeyPitwallReadsIsInARealSessionsLogWithItsType() {
    Map<String, Logged> required = new LinkedHashMap<>(ROBOT);
    for (Joint joint : DEFINITION.joints()) {
      JOINT_INPUTS.forEach((key, type) -> required.put(LogKeys.joint(joint.label()) + key, type));
      JOINT_OUTPUTS.forEach(
          (key, type) -> required.put(LogKeys.jointOutputs(joint.label()) + key, type));
      if (JOINTS_A_MOVE_ENDS_ON.contains(joint.label())) {
        JOINT_OUTPUTS_AFTER_A_MOVE_ENDS.forEach(
            (key, type) -> required.put(LogKeys.jointOutputs(joint.label()) + key, type));
      }
      if (joint.driven().absoluteEncoder().isPresent()) {
        JOINT_WITH_AN_ABSOLUTE_ENCODER.forEach(
            (key, type) -> required.put(LogKeys.jointOutputs(joint.label()) + key, type));
      }
    }
    for (String gate : GATES_THE_CONTROLS_MAKE) {
      required.put(LogKeys.GATES + gate, FLAG);
      GATE.forEach((under, type) -> required.put(LogKeys.GATES + gate + under, type));
    }
    for (String sensor : List.of("claw beam", "hopper")) {
      SENSOR.forEach((key, type) -> required.put(LogKeys.sensor(sensor) + key, type));
    }
    for (int port = 0; port < 2; port++) {
      String gamepad = LogKeys.gamepad(port);
      GAMEPAD.forEach((key, type) -> required.put(gamepad + key, type));
    }
    for (String key : logged.keySet()) {
      boolean isAGatesOwn =
          key.startsWith(LogKeys.GATES)
              && !key.endsWith(LogKeys.BLOCKING)
              && !key.endsWith(LogKeys.BLOCKING_TEXT);
      if (isAGatesOwn) {
        required.put(key, FLAG);
        GATE.forEach((under, type) -> required.put(key + under, type));
      }
    }

    assertThat(loops).as("loops in the session's log").isGreaterThan(50);
    List<String> wrong = new ArrayList<>();
    required.forEach(
        (key, type) -> {
          Logged found = logged.get(key);
          if (found == null) {
            wrong.add(key + " is not in the log: Pitwall reads it as " + type);
          } else if (!found.equals(type)) {
            wrong.add(key + " is logged as " + found + ", and Pitwall reads it as " + type);
          }
        });
    assertThat(wrong).as("keys Pitwall's Robot view reads (LogKeys) against the log").isEmpty();
  }

  /**
   * The list checked above is all of Pitwall's list: a key added to {@link LogKeys} and not to this
   * test fails here, so a new key cannot go unchecked.
   */
  @Test
  void everyKeyInPitwallsListIsCheckedHere() throws IllegalAccessException {
    Set<String> listed = new TreeSet<>();
    for (Field field : LogKeys.class.getFields()) {
      if (Modifier.isStatic(field.getModifiers()) && field.getType() == String.class) {
        listed.add((String) field.get(null));
      }
    }
    Set<String> checked = new TreeSet<>();
    for (Map<String, Logged> keys :
        List.of(
            JOINT_INPUTS,
            JOINT_OUTPUTS,
            JOINT_OUTPUTS_AFTER_A_MOVE_ENDS,
            JOINT_WITH_AN_ABSOLUTE_ENCODER,
            SENSOR,
            GAMEPAD,
            ROBOT,
            GATE)) {
      checked.addAll(keys.keySet());
    }
    // Prefixes, which the keys above are built on, and the one input only some joints log: an
    // absolute encoder the robot reads itself, which the guide's robot has none of.
    checked.addAll(List.of(LogKeys.OUTPUTS, LogKeys.GATES, LogKeys.ABSOLUTE_CONNECTED));

    assertThat(checked).containsExactlyInAnyOrderElementsOf(listed);
  }

  /**
   * What a move, a gate and a binding write is there because the session made them happen, not only
   * because the keys exist: each key below changed through the values on its row, in that order.
   * The pivot's two moves ended by name, the gate to feed was false and then true with its blocking
   * terms present while it was false and gone once it held, and bindings fired.
   */
  @Test
  void aMoveAGateAndABindingAreLoggedAsTheyHappen() {
    String pivot = LogKeys.jointOutputs("pivot");
    String gate = LogKeys.GATES + "ready to feed";
    Map<String, List<String>> expected = new LinkedHashMap<>();
    expected.put(pivot + LogKeys.OUTCOME, List.of("some"));
    expected.put(pivot + LogKeys.OUTCOME_NAME, List.of("pivot → out", "pivot → in"));
    expected.put(gate, List.of("false", "true"));
    expected.put(gate + LogKeys.BLOCKING, List.of("some", "none"));
    expected.put(gate + LogKeys.BLOCKING_TEXT, List.of("some", "none"));
    expected.put(LogKeys.FIRED, List.of("some"));
    expected.put(LogKeys.BINDINGS, List.of("some"));

    assertSoftly(
        softly ->
            expected.forEach(
                (key, order) ->
                    softly
                        .assertThat(changes.getOrDefault(key, List.of()))
                        .as(key + " should have held " + order + " in turn")
                        .containsSubsequence(order.toArray(String[]::new))));
  }

  /** Read through Pitwall's own receiver, the log gives every joint, as the facts it logged. */
  @Test
  void pitwallsSessionHoldsEveryJointOfTheLogAsFacts() {
    Moment newest = session.newest().orElseThrow();

    assertThat(session.taken()).isEqualTo(loops);
    assertThat(newest.joints())
        .extracting(Moment.OfJoint::label)
        .containsExactlyElementsOf(DEFINITION.joints().stream().map(Joint::label).toList());
    assertThat(newest.joints())
        .allSatisfy(
            joint -> {
              assertThat(joint.state()).as(joint.label()).isEqualTo(JointState.READY);
              assertThat(joint.link()).as(joint.label()).isEqualTo(Moment.Link.ANSWERS);
              assertThat(joint.position()).as(joint.label()).isFinite();
            });
    assertThat(newest.sensors()).extracting(Moment.OfSensor::label).contains("claw beam", "hopper");
    assertThat(session.view(session.taken() - 1)).isPresent();
  }
}

package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

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
 * <p>So a real session's log is read back: the session robot runs in the simulator in a process of
 * its own and records its log, as it does for the replay test. Every loop of that log is handed to
 * Pitwall's own receiver, and every key in the list is looked for in it, for every joint, sensor
 * and gamepad.
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
   * What a joint logs only once something has happened to it: how a move on it ended, and where an
   * absolute encoder says it is. The session robot has no controls, so no move ends in it. These
   * are held to their type wherever the log has them, and are not required to be there.
   */
  private static final Map<String, Logged> JOINT_OUTPUTS_ONCE_THEY_HAPPEN =
      Map.of(
          LogKeys.OUTCOME, Logged.fact("MoveOutcome[]"),
          LogKeys.OUTCOME_NAME, TEXT,
          LogKeys.OUTCOME_WHY, TEXT,
          LogKeys.OUTCOME_HELD_BY, Logged.fact("Holder[]"),
          LogKeys.ABSOLUTE_AT, NUMBER);

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

  /** A gate's own keys beside whether it holds, where the log has a gate. */
  private static final Map<String, Logged> GATE =
      Map.of(LogKeys.BLOCKING, Logged.fact("GateTerm[]"), LogKeys.BLOCKING_TEXT, TEXTS);

  @TempDir(cleanup = CleanupMode.ON_SUCCESS) // a failed run keeps its log to look at
  static Path dir;

  /** What the last loop of the session's log holds, by key as the log names it. */
  private static final Map<String, Logged> logged = new LinkedHashMap<>();

  private static final RobotDefinition DEFINITION = ExampleRobot.COMPETITION.definition();
  private static final RobotSession session = RobotSession.of(DEFINITION);
  private static int loops;

  @BeforeAll
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  static void recordTheSessionAndHandItsLogToPitwall() throws Exception {
    Path recorded = SessionRecording.record(dir);
    WPILOGReader reader = new WPILOGReader(recorded.toString());
    reader.start();
    LogDataReceiver receiver = AdvantageKitLog.receiver(session);
    LogTable table = new LogTable(0);
    while (reader.updateTable(table)) {
      // AdvantageKit hands a receiver a copy of the table each loop, as here.
      receiver.putTable(LogTable.clone(table));
      loops++;
    }
    table.getAll(false).forEach((key, value) -> logged.put(key, Logged.of(value)));
  }

  /** Every key Pitwall reads is in the log, of the type Pitwall reads it as. */
  @Test
  void everyKeyPitwallReadsIsInARealSessionsLogWithItsType() {
    Map<String, Logged> required = new LinkedHashMap<>(ROBOT);
    Map<String, Logged> whereLogged = new LinkedHashMap<>();
    for (Joint joint : DEFINITION.joints()) {
      JOINT_INPUTS.forEach((key, type) -> required.put(LogKeys.joint(joint.label()) + key, type));
      JOINT_OUTPUTS.forEach(
          (key, type) -> required.put(LogKeys.jointOutputs(joint.label()) + key, type));
      JOINT_OUTPUTS_ONCE_THEY_HAPPEN.forEach(
          (key, type) -> whereLogged.put(LogKeys.jointOutputs(joint.label()) + key, type));
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
    whereLogged.forEach(
        (key, type) -> {
          Logged found = logged.get(key);
          if (found != null && !found.equals(type)) {
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
            JOINT_OUTPUTS_ONCE_THEY_HAPPEN,
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

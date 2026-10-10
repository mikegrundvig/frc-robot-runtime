package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;

/**
 * What a robot's driven joints are told, and what they do in the simulator, as the rows a recording
 * compares ({@code com.michaelgrundvig.frc.runtime.GoldenFile}): each joint's findings and its
 * motor controllers' settings as the robot works them out at startup, and, loop by loop, where each
 * joint really is. The recordings pin today's outputs, so a change meant to change nothing (how the
 * settings are worked out, where the simulator's joints are modeled) is shown to change nothing.
 */
public final class JointRecordings {
  private JointRecordings() {}

  /** What a settings row's columns are. */
  public static final String SETTINGS = "joint,device,group,setting,value";

  /**
   * Every driven joint's findings, in the robot's words, then, unless one is refused, its motor
   * controllers' settings: for a TalonFX (or a joint wired to nothing, which the simulator runs as
   * one), each setting Phoenix is told that isn't its default, the leader's, each follower's and
   * its CANcoder's; for a SPARK, every parameter it's told, by REV's number, and what the robot
   * works out beside them.
   */
  public static List<String> settings(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<String> rows = new ArrayList<>();
    rows.add(SETTINGS);
    for (Joint joint : Joints.driven(definition)) {
      String label = joint.label();
      List<Finding> findings = ControllerSettings.findings(math, joint);
      for (Finding finding : findings) {
        rows.add(String.join(",", label, "findings", finding.level().name(), finding.message()));
      }
      if (findings.stream().anyMatch(Finding::refused)) {
        continue;
      }
      MechanismWiring.Controller controller =
          joint
              .driven()
              .motors()
              .map(Driven.Motors::controllers)
              .filter(wired -> !wired.isEmpty())
              .map(wired -> wired.get(0).controller())
              .orElse(MechanismWiring.Controller.TALON_FX);
      switch (controller) {
        case TALON_FX -> talon(rows, math, joint, null);
        case TALON_FXS ->
            talon(
                rows,
                math,
                joint,
                ((Actuator.Motors) math.drivesOf(joint).get(0).actuator()).motor());
        case SPARK_MAX, SPARK_FLEX -> spark(rows, math, joint);
      }
    }
    return rows;
  }

  /**
   * A TalonFX's settings, or, for a motor on a TalonFXS, a TalonFXS's, as the joint's IO converts
   * them.
   */
  private static void talon(List<String> rows, RobotMath math, Joint joint, @Nullable Motor onFxs) {
    String label = joint.label();
    var settings = ControllerSettings.talon(math, joint, ControllerSettings.control(joint));
    talon(rows, label, "leader", settings.leader(), onFxs);
    for (var follower : settings.followers()) {
      String device = "follower " + follower.id();
      rows.add(
          String.join(",", label, device, "request", "LeaderID", "" + follower.request().LeaderID));
      rows.add(
          String.join(
              ",",
              label,
              device,
              "request",
              "MotorAlignment",
              "" + follower.request().MotorAlignment));
      talon(rows, label, device, follower.configuration(), onFxs);
    }
    settings
        .cancoder()
        .ifPresent(
            cancoder ->
                changed(
                    rows,
                    label,
                    "cancoder",
                    cancoder.toString(),
                    new CANcoderConfiguration().toString()));
  }

  private static void talon(
      List<String> rows,
      String label,
      String device,
      TalonFXConfiguration config,
      @Nullable Motor onFxs) {
    if (onFxs == null) {
      changed(rows, label, device, config.toString(), new TalonFXConfiguration().toString());
    } else {
      changed(
          rows,
          label,
          device,
          PhoenixMechanism.talonFxs(config, onFxs).toString(),
          new TalonFXSConfiguration().toString());
    }
  }

  private static void spark(List<String> rows, RobotMath math, Joint joint) {
    String label = joint.label();
    var settings = ControllerSettings.spark(math, joint, ControllerSettings.control(joint));
    parameters(rows, label, "leader", settings.leader().flatten());
    for (var follower : settings.followers()) {
      parameters(rows, label, "follower " + follower.id(), follower.config().flatten());
    }
    rows.add(robots(label, "rotorRatio", settings.rotorRatio()));
    rows.add(robots(label, "loopRatio", settings.loopRatio()));
    rows.add(robots(label, "loopZero", settings.loopZero()));
    rows.add(robots(label, "positionKp", settings.positionKp()));
    rows.add(robots(label, "velocityKp", settings.velocityKp()));
    rows.add(robots(label, "windowEdge", settings.windowEdge()));
    rows.add(String.join(",", label, "robot", "", "closedLoop", "" + settings.closedLoop()));
    rows.add(
        String.join(
            ",",
            label,
            "robot",
            "",
            "absolute",
            settings.absolute().map(encoder -> encoder.label()).orElse("none")));
    settings
        .softLimits()
        .ifPresentOrElse(
            limits -> {
              rows.add(robots(label, "softLimits.lower", limits[0]));
              rows.add(robots(label, "softLimits.upper", limits[1]));
            },
            () -> rows.add(String.join(",", label, "robot", "", "softLimits", "none")));
  }

  private static String robots(String label, String setting, double value) {
    return String.join(",", label, "robot", "", setting, number(value));
  }

  /**
   * Each line of a Phoenix configuration's own listing that differs from its default's, as a row:
   * the two list the same settings in the same order, so the defaults are left out and the rest is
   * everything Phoenix is told. A value's unit, where Phoenix gives one, is a column of its own.
   */
  private static void changed(
      List<String> rows, String label, String device, String listed, String defaults) {
    List<String> lines = listed.lines().toList();
    List<String> standard = defaults.lines().toList();
    if (lines.size() != standard.size()) {
      throw new IllegalStateException(device + "'s listing isn't its default's shape");
    }
    String group = "";
    for (int i = 0; i < lines.size(); i++) {
      String line = lines.get(i).strip();
      if (line.startsWith("Config Group: ")) {
        group = line.substring("Config Group: ".length());
        continue;
      }
      int colon = line.indexOf(": ");
      if (colon < 0 || lines.get(i).equals(standard.get(i))) {
        continue;
      }
      String setting = line.substring(0, colon);
      String[] value = line.substring(colon + 2).split(" ", 2);
      rows.add(
          String.join(",", label, device, group, setting, value[0])
              + (value.length > 1 ? "," + value[1] : ""));
    }
  }

  /**
   * A SPARK's parameters, as REV flattens them (a parameter's number and its value's bits, in hex),
   * in the order of their numbers: what the SPARK is told, exactly.
   */
  private static void parameters(List<String> rows, String label, String device, String flat) {
    flat.lines()
        .filter(line -> !line.isBlank())
        .map(line -> line.split(",", 2))
        .sorted(Comparator.comparingInt(pair -> Integer.parseInt(pair[0])))
        .forEach(pair -> rows.add(String.join(",", label, device, "parameter", pair[0], pair[1])));
  }

  /**
   * A session in the simulator, recorded: every second loop, for each joint, where it really is and
   * how fast it moves, the current each of its motors draws from the battery, where its mechanism
   * reads it, its goal, and its state.
   */
  public static final class Session {
    private final JointsSimHarness sim;
    private final List<Joint> joints;
    private final List<String> rows = new ArrayList<>();
    private int loop;

    /** A session of the joints a harness drives, recorded in this order. */
    public Session(JointsSimHarness sim, List<Joint> joints) {
      this.sim = sim;
      this.joints = List.copyOf(joints);
      StringBuilder header = new StringBuilder("loop");
      for (Joint joint : joints) {
        header.append(
            String.format(
                Locale.ROOT,
                ",%1$s position,%1$s velocity,%1$s supply amps,%1$s read at,%1$s goal,%1$s state",
                joint.label()));
      }
      rows.add(header.toString());
    }

    /** Runs the harness's loops for a time, recording every second one. */
    public void run(double seconds) {
      for (int i = 0; i < Math.round(seconds / 0.02); i++) {
        sim.loop();
        if (++loop % 2 == 0) {
          rows.add(row());
        }
      }
    }

    /** The session's rows so far, its header first. */
    public List<String> rows() {
      return rows;
    }

    private String row() {
      StringBuilder row = new StringBuilder().append(loop);
      for (Joint joint : joints) {
        JointMechanism mechanism = sim.joints.of(joint);
        row.append(',')
            .append(number(sim.simulation.position(joint)))
            .append(',')
            .append(number(sim.simulation.velocity(joint)))
            .append(',')
            .append(number(sim.simulation.supplyAmps(joint)))
            .append(',')
            .append(number(mechanism.position()))
            .append(',')
            .append(number(mechanism.goal()))
            .append(',')
            .append(mechanism.state());
      }
      return row.toString();
    }
  }

  /** A number to a ten-millionth: finer than a recording's tolerance, a millionth. */
  private static String number(double value) {
    return String.format(Locale.ROOT, "%.7f", value);
  }
}

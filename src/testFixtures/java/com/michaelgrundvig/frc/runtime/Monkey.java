package com.michaelgrundvig.frc.runtime;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.simulator.Component;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Parameter;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.runtime.behavior.Binding;
import com.michaelgrundvig.frc.runtime.hardware.Hardware;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.runtime.sim.PdhIOSim;
import com.michaelgrundvig.frc.runtime.sim.SensorSimulation;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveHardware;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveInvariants;
import com.michaelgrundvig.frc.runtime.subsystems.localization.LocalizationInvariants;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.AbsoluteEncoderIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointInvariants;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.PdhIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.SensorIO;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.WatchedJoint;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.simulation.FaultCatalog;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.path.Keyframe;
import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Random;
import java.util.Set;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.Measure;

/**
 * A monkey at the controls: a whole robot, built as the robot builds it with a team's controls, run
 * through a match or two of random input from one seed, every loop checked against what must always
 * hold (of its joints, {@link JointInvariants}; of its drive moves, {@link DriveInvariants}; of
 * where it thinks it is, {@link LocalizationInvariants}), and no exception escaping the robot's
 * loop. Its report counts the loops each joint and drive move check had something to judge. Its
 * drive team presses, holds, taps and releases every button the robot's bindings record, both
 * gamepads' sticks wander, commands a team's code could schedule (each joint's moves, its outputs,
 * its configurations, a law of its own, now and then a target that isn't a number; the drivetrain's
 * moves to a pose, a heading, along a path, or at a speed until something holds) start and are
 * canceled, and the robot is disabled and enabled as a match does, and now and then as a brownout
 * or the drive team does, at any moment, mid-homing or mid-move, and its mode switched between
 * autonomous and teleop. With faults on, at random times something fails and later recovers: a
 * sensor tripped, blinded, silenced or its wire cut; a joint's motor controller silenced or
 * restarted, its encoder frozen, slipped or unplugged, the joint jammed, one of its motors' PDH
 * breakers tripped (its controller off the CAN bus without power, a leader's taking the joint with
 * it), or one of its followers off the CAN bus alone; and anything the simulator's controls can
 * fail (the drivetrain's modules and IMUs, the cameras, every camera at once, the robot shoved).
 *
 * <p>Everything random comes from one {@link Random}, seeded, and the simulator's clock moves only
 * as it's stepped, so a seed runs the same every time: a failure names its seed and its loop, and
 * {@code -PmonkeySeed=<seed>} runs it again, exactly (docs/testing.md).
 *
 * <pre>{@code
 * new Monkey(sim, Monkey.robot(ExampleRobot.COMPETITION, Controls::new), seed, 150, true).run();
 * }</pre>
 */
public final class Monkey {
  /**
   * A robot for the monkey to drive: its configuration, and the team's controls bound on it.
   *
   * @param config the robot, as Robots.java holds it
   * @param controls its {@code Controls}' constructor, as {@link RobotSim#controls} makes it
   */
  public record Robot(RobotConfig config, ControlsMaker<?> controls) {
    @Override
    public String toString() {
      return config.name();
    }
  }

  /** A robot and its controls. */
  public static Robot robot(RobotConfig config, ControlsMaker<?> controls) {
    return new Robot(config, controls);
  }

  /**
   * How a run went: how much the monkey did, and where everything really went.
   *
   * @param loops the loops it ran
   * @param actions what it did at the controls, as it did it
   * @param faults how many faults it applied
   * @param fingerprint where every joint and the robot really were, loop by loop, as one number:
   *     the same for the same seed, every time
   * @param judged how many loops each of its joints' and drive moves' checks had something to
   *     judge, by its name: one that never fires may never have had the chance
   */
  public record Report(
      int loops, int actions, int faults, long fingerprint, Map<String, Integer> judged) {}

  /** A match: disabled before it, autonomous, disabled between, teleop. In seconds. */
  private static final double[] MATCH = {1, 15, 2, 135};

  /** How often each kind of thing happens, a second. */
  private static final double PRESSES = 3;

  private static final double STICKS = 1;
  private static final double COMMANDS = 0.6;
  private static final double CANCELS = 0.3;
  private static final double BROWNOUTS = 0.02;
  private static final double TOGGLES = 0.05;
  private static final double SWITCHES = 0.03;
  private static final double FAULTS = 0.25;

  /** How far inside the field's walls a drive move's poses lie, in meters. */
  private static final double FIELD_MARGIN = 0.5;

  /** How many of its last actions a failure shows. */
  private static final int SHOWN = 12;

  private final RobotSim sim;
  private final Robot robot;
  private final long seed;
  private final double seconds;
  private final boolean faults;
  private final Random random;

  /** The test class that built it, which a failure names to run it again. */
  private final String test;

  private final Map<Joint, WatchedJoint> watched = new LinkedHashMap<>();
  private final Deque<String> recent = new ArrayDeque<>();
  private final List<Input> inputs = new ArrayList<>();
  private final List<Fault> catalog = new ArrayList<>();

  /** Each input held, and the loop it's let go at. */
  private final Map<Input, Integer> releases = new LinkedHashMap<>();

  /** Each fault on, and the loop it's undone at. */
  private final Map<Fault, Integer> clears = new LinkedHashMap<>();

  /**
   * The joints jammed, the PDH channels whose breakers are tripped, and the followers off the CAN
   * bus, as the faults have them.
   */
  private final Set<Joint> jammed = new HashSet<>();

  private final Set<Integer> tripped = new HashSet<>();
  private final Set<CanDevice> offTheBus = new HashSet<>();

  /** The joints whose motor controllers are silenced, off the CAN bus together. */
  private final Set<Joint> silenced = new HashSet<>();

  private int loop;
  private RobotMode mode = RobotMode.UNKNOWN;
  private boolean enabled;
  private int actions;
  private int applied;
  private int disabledUntil = -1;
  private int switchedUntil = -1;
  private @Nullable FieldModel field;
  private long fingerprint = 17;

  /**
   * @param sim the simulator, the test's
   * @param robot the robot, and the controls bound on it
   * @param seed where its random choices start
   * @param seconds how long it runs, in the robot's seconds: a match is 153
   * @param faults whether things fail as it runs
   */
  public Monkey(RobotSim sim, Robot robot, long seed, double seconds, boolean faults) {
    this.sim = sim;
    this.robot = robot;
    this.seed = seed;
    this.seconds = seconds;
    this.faults = faults;
    random = new Random(seed);
    // The class that called this constructor, or the one it's nested in: Gradle's --tests names a
    // top-level class.
    test =
        StackWalker.getInstance(StackWalker.Option.RETAIN_CLASS_REFERENCE)
            .getCallerClass()
            .getNestHost()
            .getName();
  }

  /** How to run it again, alone: the test that built it, at its seed. */
  public String rerun() {
    return "./gradlew test --tests " + test + " -PmonkeySeed=" + seed;
  }

  /**
   * Runs it: the robot built, its controls bound, and the match played.
   *
   * @throws AssertionError naming the seed, the loop and what broke, the first loop anything does
   */
  public Report run() {
    // Hands off, and disabled, whatever ran before.
    for (GamepadSim pad : List.of(sim.driver(), sim.operator())) {
      for (Gamepad.Button button : Gamepad.Button.values()) {
        pad.setRawButton(button.value, false);
      }
      for (Gamepad.Axis axis : Gamepad.Axis.values()) {
        pad.setRawAxis(axis.value, 0);
      }
      pad.notifyNewData();
    }
    RobotSim.setEnabled(false);
    sim.build(robot.config(), SensorErrors.NONE, this::watch);
    sim.controls(robot.controls());
    RobotDefinition definition = sim.robot().definition();
    Joints joints = sim.joints();
    JointInvariants invariants =
        new JointInvariants(
            definition, joints, watched, sim.scheduler(), jammed::contains, this::drawsNothing);
    DriveInvariants drive = new DriveInvariants(sim.scheduler());
    LocalizationInvariants localization =
        new LocalizationInvariants(
            sim.robot().localization(),
            sim.drive(),
            sim.robot()::frames,
            this::blind,
            !definition.cameras().isEmpty(),
            robot.config().estimator());
    field = Rebuilt.field(definition.body());
    for (Binding binding : sim.bindings().all()) {
      bound(binding);
    }
    catalog(definition, joints);
    int total = RobotLoop.loops(seconds);
    for (loop = 0; loop < total; loop++) {
      act(joints, invariants);
      try {
        sim.loop();
      } catch (RuntimeException e) {
        throw new AssertionError(failure("the robot's loop threw " + e), e);
      }
      long now = RobotController.getTime();
      String broken = invariants.broken(now, enabled);
      if (broken == null) {
        broken = drive.broken(now);
      }
      if (broken == null) {
        broken = localization.broken(enabled);
      }
      if (broken != null) {
        throw new AssertionError(failure(broken));
      }
      remember(joints);
    }
    Map<String, Integer> judged = new LinkedHashMap<>(invariants.judged());
    judged.putAll(drive.judged());
    System.out.printf(Locale.ROOT, "Monkey on %s, seed %d, judged: %s%n", robot, seed, judged);
    return new Report(total, actions, applied, fingerprint, judged);
  }

  /** Whether every camera is unplugged, as the simulator's controls have it. */
  private boolean blind() {
    boolean any = false;
    for (Component component : sim.components().components()) {
      if (component.kind().equals(FaultCatalog.Camera.KIND)) {
        for (var fault : component.faults()) {
          if (fault.name().equals(FaultCatalog.Camera.UNPLUGGED.name())) {
            if (!fault.isOn()) {
              return false;
            }
            any = true;
          }
        }
      }
    }
    return any;
  }

  /** Each part of the robot's hardware as it is, each joint's watched on its way. */
  private Hardware watch(Hardware hardware) {
    return new Hardware() {
      @Override
      public DriveHardware drive() {
        return hardware.drive();
      }

      @Override
      public JointIO joint(Joint joint) {
        WatchedJoint io = new WatchedJoint(hardware.joint(joint));
        watched.put(joint, io);
        return io;
      }

      @Override
      public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
        return hardware.absoluteEncoder(joint);
      }

      @Override
      public SensorIO sensor(Sensor.Placed sensor) {
        return hardware.sensor(sensor);
      }

      @Override
      public PdhIO pdh() {
        return hardware.pdh();
      }

      @Override
      public List<CameraIO> cameras() {
        return hardware.cameras();
      }

      @Override
      public void close() {
        hardware.close();
      }
    };
  }

  // ---- What it does, a loop at a time ----

  /** This loop's doings, before the robot runs it. */
  private void act(Joints joints, JointInvariants invariants) {
    mode();
    for (Map.Entry<Input, Integer> held : List.copyOf(releases.entrySet())) {
      if (held.getValue() <= loop) {
        held.getKey().set(0);
        releases.remove(held.getKey());
        did("let go of " + held.getKey());
      }
    }
    for (Map.Entry<Fault, Integer> on : List.copyOf(clears.entrySet())) {
      if (on.getValue() <= loop) {
        Fault fault = on.getKey();
        fault.clear().run();
        clears.remove(fault);
        touch(invariants, fault);
        did("cleared " + fault);
      }
    }
    if (!inputs.isEmpty() && happens(PRESSES)) {
      press(inputs.get(random.nextInt(inputs.size())));
    }
    if (happens(STICKS)) {
      sticks();
    }
    if (loop > 0 && happens(COMMANDS)) {
      command(joints);
    }
    if (happens(CANCELS)) {
      cancel();
    }
    if (faults && !catalog.isEmpty() && happens(FAULTS)) {
      fail(invariants, catalog.get(random.nextInt(catalog.size())));
    }
    sim.driver().notifyNewData();
    sim.operator().notifyNewData();
  }

  /** Whether something that happens this often a second happens this loop. */
  private boolean happens(double perSecond) {
    return random.nextDouble() < perSecond * RobotLoop.PERIOD_SECONDS;
  }

  /** How long something lasts, in loops: up to so many seconds, most of them short. */
  private int lasting(double atMostSeconds) {
    double share = random.nextDouble();
    return Math.max(1, RobotLoop.loops(atMostSeconds * share * share));
  }

  /**
   * The match's mode this loop; now and then a brownout that disables it for a moment, the drive
   * team disabling it a while at any moment (mid-homing, mid-move), or its mode switched between
   * autonomous and teleop while it's enabled.
   */
  private void mode() {
    double t = (loop * RobotLoop.PERIOD_SECONDS) % (MATCH[0] + MATCH[1] + MATCH[2] + MATCH[3]);
    RobotMode now = RobotMode.TELEOPERATED;
    boolean on = true;
    if (t < MATCH[0]) {
      on = false;
    } else if (t < MATCH[0] + MATCH[1]) {
      now = RobotMode.AUTONOMOUS;
    } else if (t < MATCH[0] + MATCH[1] + MATCH[2]) {
      on = false;
    }
    if (on && disabledUntil < loop && happens(BROWNOUTS)) {
      disabledUntil = loop + lasting(1.5);
      did("brownout");
    }
    if (on && disabledUntil < loop && happens(TOGGLES)) {
      disabledUntil = loop + lasting(4);
      did("disabled by the drive team");
    }
    if (on && switchedUntil < loop && happens(SWITCHES)) {
      switchedUntil = loop + lasting(10);
      did("switched its mode");
    }
    on &= loop >= disabledUntil;
    if (loop < switchedUntil) {
      now = now == RobotMode.AUTONOMOUS ? RobotMode.TELEOPERATED : RobotMode.AUTONOMOUS;
    }
    if (now != mode || on != enabled) {
      mode = now;
      enabled = on;
      RobotSim.enable(mode, enabled);
      did((enabled ? "enabled in " : "disabled in ") + mode);
    }
  }

  /** A button or trigger: pressed, tapped, held a while, or let go if held. */
  private void press(Input input) {
    if (releases.containsKey(input)) {
      input.set(0);
      releases.remove(input);
      did("let go of " + input);
      return;
    }
    double value = input.axis() && random.nextInt(4) == 0 ? random.nextDouble() : 1;
    input.set(value);
    int held = random.nextInt(3) == 0 ? 1 + random.nextInt(3) : lasting(6);
    releases.put(input, loop + held);
    did(String.format(Locale.ROOT, "pressed %s (%.2f) for %d loops", input, value, held));
  }

  /** One of the four sticks pushed somewhere, or let go. */
  private void sticks() {
    GamepadSim pad = random.nextBoolean() ? sim.driver() : sim.operator();
    boolean left = random.nextBoolean();
    double x = stick();
    double y = stick();
    pad.setRawAxis(left ? Gamepad.Axis.LEFT_X.value : Gamepad.Axis.RIGHT_X.value, x);
    pad.setRawAxis(left ? Gamepad.Axis.LEFT_Y.value : Gamepad.Axis.RIGHT_Y.value, y);
    did(
        String.format(
            Locale.ROOT,
            "%s %s stick to %.2f, %.2f",
            pad == sim.driver() ? "driver" : "operator",
            left ? "left" : "right",
            x,
            y));
  }

  /** A stick's axis: let go, at an end, or anywhere between. */
  private double stick() {
    return switch (random.nextInt(5)) {
      case 0 -> 0;
      case 1 -> random.nextBoolean() ? 1 : -1;
      default -> random.nextDouble() * 2 - 1;
    };
  }

  /** A command a team's code could schedule, on a joint or a configuration. */
  private void command(Joints joints) {
    List<JointMechanism> all = joints.all();
    List<Configuration> configurations = sim.robot().definition().configurations();
    Command command;
    if (random.nextInt(4) == 0) {
      command = driveCommand();
    } else if (!configurations.isEmpty() && random.nextInt(8) == 0) {
      Configuration configuration = configurations.get(random.nextInt(configurations.size()));
      command =
          random.nextBoolean()
              ? joints.goTo(configuration)
              : joints
                  .run(configuration)
                  .withTimeout(Seconds.of(lasting(4) * RobotLoop.PERIOD_SECONDS));
    } else if (!all.isEmpty()) {
      command = jointCommand(all.get(random.nextInt(all.size())));
    } else {
      return;
    }
    sim.scheduler().schedule(command);
    did("scheduled " + command.name());
  }

  /**
   * One of the drivetrain's moves, at random: to a pose, a heading, along a path through keyframes
   * (now and then none, or two of a name, which it refuses), or at a speed until something holds.
   */
  private Command driveCommand() {
    Drive drive = sim.drive();
    double timeout = lasting(6) * RobotLoop.PERIOD_SECONDS;
    return switch (random.nextInt(6)) {
      case 0 -> drive.goTo(onTheField());
      case 1 -> drive.goTo(onTheField()).within(Seconds.of(timeout));
      case 2 -> drive.turnTo(Rotation2d.fromDegrees(random.nextDouble() * 360 - 180));
      case 3 ->
          drive.runUntil(
              new ChassisVelocities(stick() * 2, stick() * 2, stick() * 3),
              () -> random.nextInt(100) == 0);
      default -> drive.follow(keyframes());
    };
  }

  /** A pose on the field, inside its walls, facing anywhere. */
  private Pose2d onTheField() {
    FieldModel on = Objects.requireNonNull(field);
    return new Pose2d(
        FIELD_MARGIN + random.nextDouble() * (on.length() - 2 * FIELD_MARGIN),
        FIELD_MARGIN + random.nextDouble() * (on.width() - 2 * FIELD_MARGIN),
        Rotation2d.fromDegrees(random.nextDouble() * 360 - 180));
  }

  /** A path's keyframes: one to four, now and then named, now and then none or two of a name. */
  private Keyframe[] keyframes() {
    int count = random.nextInt(20) == 0 ? 0 : 1 + random.nextInt(4);
    Keyframe[] path = new Keyframe[count];
    for (int i = 0; i < count; i++) {
      Keyframe keyframe = Keyframe.at(onTheField());
      path[i] = random.nextInt(4) == 0 ? keyframe.named("k" + random.nextInt(3)) : keyframe;
    }
    return path;
  }

  /** One of a joint's commands, at random, with options at random. */
  private Command jointCommand(JointMechanism mechanism) {
    Joint joint = mechanism.joint();
    double timeout = lasting(4) * RobotLoop.PERIOD_SECONDS;
    List<Output> outputs = new ArrayList<>(joint.outputs());
    outputs.add(Output.STOPPED);
    Output output = outputs.get(random.nextInt(outputs.size()));
    int choice = random.nextInt(joint.holdsPositions() ? 10 : 5);
    return switch (choice) {
      case 0 -> mechanism.run(output).withTimeout(Seconds.of(timeout));
      case 1 -> mechanism.runUntil(output, mechanism.isAtGoal());
      case 2 -> follow(mechanism, false).withTimeout(Seconds.of(timeout));
      case 3 -> law(mechanism).withTimeout(Seconds.of(timeout));
      case 4 -> mechanism.home();
      case 5 -> mechanism.hold().withTimeout(Seconds.of(timeout));
      case 6 -> follow(mechanism, true).withTimeout(Seconds.of(timeout));
      case 7 -> goTo(mechanism).atSpeed(Percent.of(5 + random.nextInt(96)));
      case 8 -> goTo(mechanism).within(Seconds.of(timeout));
      default -> {
        List<Position> positions = joint.positions();
        yield positions.isEmpty()
            ? goTo(mechanism)
            : mechanism.goTo(positions.get(random.nextInt(positions.size())));
      }
    };
  }

  /** A joint to a computed position: somewhere in its range, or a little past it. */
  private Move goTo(JointMechanism mechanism) {
    double to = somewhere(mechanism.joint());
    return mechanism.joint() instanceof Joint.Revolute
        ? mechanism.goTo(Radians.of(to))
        : mechanism.goTo(Meters.of(to));
  }

  /**
   * A position for a joint, in its units: one of its named, or one inside its range or a little
   * past it.
   */
  private double somewhere(Joint joint) {
    List<Position> positions = joint.positions();
    if (!positions.isEmpty() && random.nextBoolean()) {
      return positions.get(random.nextInt(positions.size())).value();
    }
    Range range = joint.range().orElse(null);
    double lower = range == null ? -Math.PI : range.lower();
    double upper = range == null ? Math.PI : range.upper();
    double margin = 0.1 * (upper - lower);
    return lower - margin + random.nextDouble() * (upper - lower + 2 * margin);
  }

  /** A position or a speed in a joint's units, as a measure: radians or meters, a second or not. */
  private static Measure<?> measure(Joint joint, double value, boolean speed) {
    boolean turns = joint instanceof Joint.Revolute;
    if (speed) {
      return turns ? RadiansPerSecond.of(value) : MetersPerSecond.of(value);
    }
    return turns ? Radians.of(value) : Meters.of(value);
  }

  /**
   * A joint chasing a target that wanders, a position or a speed, which now and then isn't a
   * number, as a division by zero gives.
   */
  private Command follow(JointMechanism mechanism, boolean position) {
    Joint joint = mechanism.joint();
    double[] target = {Double.NaN};
    double speed = sim.robot().definition().math().freeSpeed(joint);
    Supplier<Measure<?>> chased =
        () -> {
          if (Double.isNaN(target[0]) || random.nextInt(50) == 0) {
            target[0] = position ? somewhere(joint) : (random.nextDouble() * 2 - 1) * speed;
          }
          double now = random.nextInt(200) == 0 ? notANumber() : target[0];
          return measure(joint, now, !position);
        };
    return mechanism.follow(chased);
  }

  /** A law of a team's own: volts every loop, now and then volts that aren't a number. */
  private Command law(JointMechanism mechanism) {
    double volts = (random.nextDouble() * 2 - 1) * 6;
    return Command.requiring(mechanism)
        .executing(
            co -> {
              while (true) {
                mechanism.setVoltage(random.nextInt(100) == 0 ? notANumber() : volts);
                co.yield();
              }
            })
        .named(String.format(Locale.ROOT, "%s at %.1f V", mechanism.getName(), volts));
  }

  private double notANumber() {
    return switch (random.nextInt(3)) {
      case 0 -> Double.NaN;
      case 1 -> Double.POSITIVE_INFINITY;
      default -> Double.NEGATIVE_INFINITY;
    };
  }

  /** One of the commands running canceled, as something else taking its joints would. */
  private void cancel() {
    List<Command> running = new ArrayList<>(sim.scheduler().getRunningCommands());
    if (running.isEmpty()) {
      return;
    }
    Command command = running.get(random.nextInt(running.size()));
    sim.scheduler().cancel(command);
    did("canceled " + command.name());
  }

  /** A fault applied, and, if it lasts, undone later. */
  private void fail(JointInvariants invariants, Fault fault) {
    if (clears.containsKey(fault)) {
      return;
    }
    fault.apply().run();
    applied++;
    touch(invariants, fault);
    did("failed " + fault);
    if (fault.clear() != NOTHING) {
      clears.put(fault, loop + lasting(8));
    }
  }

  private static void touch(JointInvariants invariants, Fault fault) {
    Joint joint = fault.joint();
    if (joint != null) {
      invariants.touched(joint);
    }
  }

  private void did(String what) {
    actions++;
    recent.addLast(
        String.format(Locale.ROOT, "%d (%.2f s): %s", loop, loop * RobotLoop.PERIOD_SECONDS, what));
    if (recent.size() > SHOWN) {
      recent.removeFirst();
    }
  }

  /** Where every joint and the robot really are this loop, folded into the run's fingerprint. */
  private void remember(Joints joints) {
    JointSimulation truth = Objects.requireNonNull(sim.world().joints());
    for (JointMechanism joint : joints.all()) {
      fingerprint = 31 * fingerprint + Double.hashCode(truth.position(joint.joint()));
    }
    Pose2d pose = sim.simulation().truePose();
    fingerprint = 31 * fingerprint + Double.hashCode(pose.getX());
    fingerprint = 31 * fingerprint + Double.hashCode(pose.getY());
  }

  private String failure(String what) {
    return String.format(
            Locale.ROOT,
            "Monkey on %s, seed %d, faults %s: loop %d (%.2f s), %s: %s%n",
            robot,
            seed,
            faults ? "on" : "off",
            loop,
            loop * RobotLoop.PERIOD_SECONDS,
            enabled ? "enabled in " + mode : "disabled",
            what)
        + "Its last doings:\n  "
        + String.join("\n  ", recent)
        + "\nRun it again: "
        + rerun();
  }

  // ---- The drive team's inputs ----

  /** A button or a trigger on one of the gamepads, as the robot's bindings name it. */
  private record Input(String gamepad, GamepadSim pad, String name, int index, boolean axis) {
    void set(double value) {
      if (axis) {
        pad.setRawAxis(index, value);
      } else {
        pad.setRawButton(index, value > 0.5);
      }
    }

    @Override
    public String toString() {
      return gamepad + " " + name;
    }
  }

  /** Every button and trigger a binding reads, each once: "operator right bumper and ...". */
  private void bound(Binding binding) {
    Map<String, Integer> buttons = new LinkedHashMap<>();
    for (Gamepad.Button button : Gamepad.Button.values()) {
      buttons.put(button.name().toLowerCase(Locale.ROOT).replace('_', ' '), button.value);
    }
    String text = binding.button();
    for (Map.Entry<String, Integer> button : buttons.entrySet()) {
      found(binding, text, button.getKey(), button.getValue(), false);
    }
    found(binding, text, "left trigger", Gamepad.Axis.LEFT_TRIGGER.value, true);
    found(binding, text, "right trigger", Gamepad.Axis.RIGHT_TRIGGER.value, true);
  }

  private void found(Binding binding, String text, String name, int index, boolean axis) {
    int at = text.indexOf(name);
    if (at < 0) {
      return;
    }
    String before = text.substring(0, at);
    String gamepad =
        before.endsWith("driver ")
            ? "driver"
            : before.endsWith("operator ") ? "operator" : binding.gamepad();
    GamepadSim pad = gamepad.equals("driver") ? sim.driver() : sim.operator();
    Input input = new Input(gamepad, pad, name, index, axis);
    if (!inputs.contains(input)) {
      inputs.add(input);
    }
  }

  // ---- What can fail ----

  private static final Runnable NOTHING = () -> {};

  /**
   * Something that can fail.
   *
   * @param name what fails, and how
   * @param joint the joint it fails, if it's one
   * @param apply fails it
   * @param clear puts it back; {@link #NOTHING} for something that happens once
   */
  private record Fault(String name, @Nullable Joint joint, Runnable apply, Runnable clear) {
    @Override
    public String toString() {
      return name;
    }
  }

  /** What can fail on this robot: its joints, its sensors, and the simulator's controls' parts. */
  private void catalog(RobotDefinition definition, Joints joints) {
    JointSimulation truth = Objects.requireNonNull(sim.world().joints());
    for (JointMechanism mechanism : joints.all()) {
      Joint joint = mechanism.joint();
      String label = joint.label();
      catalog.add(
          new Fault(
              label + "'s controllers silenced",
              joint,
              () -> {
                silenced.add(joint);
                power(joint);
              },
              () -> {
                silenced.remove(joint);
                power(joint);
              }));
      catalog.add(
          new Fault(label + "'s controller restarted", joint, () -> truth.restart(joint), NOTHING));
      boolean encoder =
          Joints.readsAbsolute(joint) != null
              || joint
                  .driven()
                  .absoluteEncoder()
                  .map(Driven.AbsoluteEncoder::readByController)
                  .orElse(false);
      if (encoder) {
        catalog.add(
            new Fault(
                label + "'s encoder frozen",
                joint,
                () -> truth.freeze(joint, true),
                () -> truth.freeze(joint, false)));
        double off = (random.nextDouble() * 2 - 1) * (joint instanceof Joint.Revolute ? 0.5 : 0.1);
        catalog.add(
            new Fault(
                String.format(Locale.ROOT, "%s's encoder slipped %s", label, joint.shown(off)),
                joint,
                () -> truth.slip(joint, off),
                () -> truth.slip(joint, 0)));
      }
      if (Joints.readsAbsolute(joint) != null) {
        catalog.add(
            new Fault(
                label + "'s encoder unplugged",
                joint,
                () -> truth.unplug(joint, true),
                () -> truth.unplug(joint, false)));
      }
      catalog.add(
          new Fault(
              label + " jammed",
              joint,
              () -> {
                truth.jam(joint, true);
                jammed.add(joint);
              },
              () -> {
                truth.jam(joint, false);
                jammed.remove(joint);
              }));
      breakers(joint);
      followers(joint);
    }
    SensorSimulation sensors = Objects.requireNonNull(sim.world().sensors());
    for (Sensor sensor : definition.math().sensors()) {
      sensorFaults(sensors, sensor);
    }
    Components components = sim.components();
    List<String> cameras = new ArrayList<>();
    for (Component component : components.components()) {
      if (component.kind().equals(FaultCatalog.Camera.KIND)) {
        cameras.add(component.name());
      }
    }
    if (!cameras.isEmpty()) {
      // Every camera at once, as the coprocessor they share losing power leaves them.
      String unplugged = FaultCatalog.Camera.UNPLUGGED.name();
      catalog.add(
          new Fault(
              "every camera unplugged",
              null,
              () -> cameras.forEach(each -> components.apply(Setting.of(each, unplugged))),
              () -> cameras.forEach(each -> components.undo(each, unplugged))));
    }
    for (Component component : components.components()) {
      for (var fault : component.faults()) {
        Map<String, Double> values = new LinkedHashMap<>();
        for (Parameter parameter : fault.parameters()) {
          double min = parameter.limits().min();
          double max = parameter.limits().max();
          values.put(parameter.name(), min + random.nextDouble() * (max - min));
        }
        String name =
            component.name() + ": " + fault.name() + (values.isEmpty() ? "" : " " + values);
        Runnable apply = () -> components.apply(Setting.of(component.name(), fault.name(), values));
        catalog.add(
            new Fault(
                name,
                null,
                apply,
                fault.kind() == com.michaelgrundvig.frc.pitwall.simulator.Fault.Kind.LASTING
                    ? () -> components.undo(component.name(), fault.name())
                    : NOTHING));
      }
    }
  }

  /**
   * Each of a joint's motors' PDH breakers tripping: its channel reads nothing, and its motor
   * controller, without power, stops answering on CAN: a follower goes neutral, and the leader's
   * joint goes silent, its followers neutral with it.
   */
  private void breakers(Joint joint) {
    PdhIOSim pdh = sim.world().pdh();
    for (CanDevice device : motors(joint)) {
      if (device.feed().isEmpty()) {
        continue;
      }
      int channel = device.feed().get().channel();
      catalog.add(
          new Fault(
              String.format(
                  Locale.ROOT, "%s's breaker tripped, channel %d", device.name(), channel),
              joint,
              () -> {
                pdh.zero(channel, true);
                tripped.add(channel);
                power(joint);
              },
              () -> {
                pdh.zero(channel, false);
                tripped.remove(channel);
                power(joint);
              }));
    }
  }

  /** A joint's motor controllers, the leader first. */
  private static List<CanDevice> motors(Joint joint) {
    return joint.devices().stream().filter(device -> device.kind().motorController()).toList();
  }

  /**
   * Whether a joint's motor controllers answer, as the faults on them leave them: the joint silent
   * while its controllers are silenced or its leader has no power, and each follower neutral while
   * it's off the bus or has no power.
   */
  private void power(Joint joint) {
    JointSimulation truth = Objects.requireNonNull(sim.world().joints());
    List<CanDevice> controllers = motors(joint);
    truth.silence(
        joint,
        silenced.contains(joint) || (!controllers.isEmpty() && unpowered(controllers.get(0))));
    for (int i = 1; i < controllers.size(); i++) {
      truth.silenceFollower(joint, i - 1, drawsNothing(controllers.get(i)));
    }
  }

  /** Whether a motor controller's breaker is tripped. */
  private boolean unpowered(CanDevice motor) {
    return motor.feed().map(feed -> tripped.contains(feed.channel())).orElse(false);
  }

  /**
   * Whether a motor draws nothing from the battery: its breaker tripped, or a follower off the bus.
   */
  private boolean drawsNothing(CanDevice motor) {
    return offTheBus.contains(motor) || unpowered(motor);
  }

  /**
   * Each of a joint's followers off the CAN bus alone, as a loose connector leaves one: neutral,
   * drawing nothing, its leader driving the joint without it.
   */
  private void followers(Joint joint) {
    List<CanDevice> controllers = motors(joint);
    for (CanDevice follower :
        controllers.subList(Math.min(1, controllers.size()), controllers.size())) {
      catalog.add(
          new Fault(
              follower.name() + " off the CAN bus",
              joint,
              () -> {
                offTheBus.add(follower);
                power(joint);
              },
              () -> {
                offTheBus.remove(follower);
                power(joint);
              }));
    }
  }

  /** What can go wrong with a sensor: tripped or blinded, silenced, its wire cut. */
  private void sensorFaults(SensorSimulation sensors, Sensor sensor) {
    if (!(sensor instanceof Sensor.Placed placed)) {
      return;
    }
    String label = placed.label();
    if (placed instanceof Sensor.Switch onSwitch) {
      boolean followsJoint =
          onSwitch.kind() instanceof Sensor.Switch.Limit limit
              && Objects.requireNonNull(sim.world().joints()).simulates(limit.joint());
      if (!followsJoint) {
        catalog.add(
            new Fault(
                label + " tripped",
                null,
                () -> sensors.trip(onSwitch, true),
                () -> sensors.trip(onSwitch, false)));
      }
      SensorWiring.Polarity polarity =
          onSwitch
              .wiring()
              .map(SensorWiring.Connection::polarity)
              .orElse(SensorWiring.Polarity.NORMALLY_OPEN);
      catalog.add(
          new Fault(
              label + "'s wire cut",
              null,
              () -> sensors.cut(onSwitch, polarity),
              () -> sensors.cut(onSwitch, null)));
    }
    if (placed instanceof Sensor.Rangefinder rangefinder) {
      double meters = random.nextDouble() * rangefinder.range().in(Meters);
      catalog.add(
          new Fault(
              String.format(Locale.ROOT, "%s sees something at %.2f m", label, meters),
              null,
              () -> sensors.see(rangefinder, Meters.of(meters)),
              () -> sensors.see(rangefinder, rangefinder.range())));
    }
    if (com.michaelgrundvig.frc.world.robot.ControllerPins.onPin(placed) == null) {
      catalog.add(
          new Fault(
              label + " silenced",
              null,
              () -> sensors.silence(placed, true),
              () -> sensors.silence(placed, false)));
    }
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiPredicate;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.util.Alert;

/**
 * Every driven joint of the robot, each a v3 {@link Mechanism} the template builds from the robot's
 * definition, the joint's block saying how it's driven and wired ({@link JointMechanism}). The
 * composition root builds it once; behavior code looks its joints up by the joints themselves,
 * never by a label:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=look-up}
 *
 * <p>A configuration's joints move together, and hold it, outputs included, with {@code
 * joints.run(FEEDING)}:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=configurations}
 *
 * <p>A condition the model says, where a joint is, how fast it runs or what a sensor reads, is a
 * gate too: {@code joints.gate("coral in claw", CLAW_BEAM.tripped())} ({@link #gate(String,
 * Condition)}).
 *
 * <p>It enforces the definition's interlocks beneath every command, each on the joint it restricts
 * ({@link JointMechanism}); one about a joint the template doesn't drive can't be, and is refused.
 *
 * <p>Each loop it reads every joint before the command scheduler runs ({@link #read}), and applies
 * what the commands asked after it ({@link #write}), so a goal set this loop is applied this loop.
 * What it reads that's wrong, it says in alerts of its own: the sensors lost, the limit switches
 * doubted, the motor controllers drawing nothing on their channels, and a PDH that doesn't answer.
 */
@CheckReturnValue
public final class Joints implements AutoCloseable {
  /**
   * What the joints read and drive, made for the mode the program runs in: on a robot, the devices
   * each block's wiring names; in the simulator, its own, built from the robot's definition; in a
   * replay, none, since the log has every reading. The joints ask for each once as they're built:
   * the sensors, then each driven joint's motor controllers and the absolute encoder the robot
   * reads itself, then the PDH. A switch on a joint's motor controller's limit input isn't asked
   * for: the joint reads it from its controller.
   *
   * <p>A test that needs only its joints' hardware gives it as a lambda, {@code joint -> io}, the
   * rest none. The robot's hardware ({@code com.michaelgrundvig.frc.runtime.hardware.Hardware})
   * gives every one.
   */
  @FunctionalInterface
  public interface Hardware {
    /** A driven joint's motor controllers: its TalonFX's or SPARK's, the simulator's, or none. */
    JointIO joint(Joint joint);

    /**
     * A joint's absolute encoder, where the robot reads it itself ({@link #readsAbsolute}): on a
     * Systemcore port, the simulator's, or none.
     */
    default AbsoluteEncoderIO absoluteEncoder(Joint joint) {
      return new AbsoluteEncoderIO() {};
    }

    /**
     * A switch or a rangefinder: on a port or a CAN device, the simulator's, or none. Never a
     * switch on a joint's motor controller's limit input, which the joint reads from its
     * controller.
     */
    default SensorIO sensor(Sensor.Placed sensor) {
      return new SensorIO() {};
    }

    /** The PDH, read with the joints ({@link #power}): the robot's, the simulator's, or none. */
    default PdhIO pdh() {
      return new PdhIO() {};
    }
  }

  private final JointMechanism[] joints;
  private final Map<Joint, JointMechanism> byJoint = new HashMap<>();

  /** Stand-ins for joints with no mechanism, asked for once the robot runs ({@link #of}). */
  private final Map<Joint, JointMechanism> standIns = new HashMap<>();

  private final RobotDefinition definition;
  private final Sensors sensors;
  private final Power power;

  /** The command scheduler its mechanisms and gates are registered with. */
  private final Scheduler scheduler;

  /**
   * The sensors with no wiring in the definition, by label: on a robot an alert of their own says
   * so, so they aren't said again as lost.
   */
  private final List<String> unwired = new ArrayList<>();

  private final ListAlert lost =
      new ListAlert(
          "Sensors",
          "lost",
          Alert.Level.HIGH,
          labels ->
              String.join(", ", labels)
                  + (labels.size() == 1 ? " is" : " are")
                  + " lost: nothing waiting on "
                  + (labels.size() == 1 ? "it" : "them")
                  + " holds. Check the wiring and the CAN bus.");

  private final ListAlert doubted =
      new ListAlert("Sensors", "doubted", Alert.Level.HIGH, doubts -> String.join("; ", doubts));

  private final ListAlert dead =
      new ListAlert("Power", "dead", Alert.Level.HIGH, dead -> String.join("; ", dead));

  /** The PDH the robot declares, by its CAN ID, while it doesn't answer. */
  private final ListAlert pdhSilent =
      new ListAlert(
          "Power",
          "silent",
          Alert.Level.MEDIUM,
          ids ->
              "The PDH isn't answering at CAN ID "
                  + ids.get(0)
                  + ", so no channel's current is read. Check its CAN wiring, and its ID in REV's"
                  + " Hardware Client.");

  /**
   * Every driven joint's mechanism, with the robot's switches and rangefinders ({@link #sensors})
   * and its PDH ({@link #power}). A joint whose absolute encoder the robot reads itself ({@link
   * #readsAbsolute}) waits for its first reading, and tells its motor controller where it is from
   * it.
   *
   * @param definition what the robot is, each joint's block saying how it's driven and wired
   * @param hardware what each joint, sensor and the PDH read and drive, for the mode the program
   *     runs in
   * @param tunings each joint's control, read each loop, its gains and current limits handed to its
   *     motor controller as they change: as the pit tunes it ({@link JointMechanism#tuning}), or as
   *     its block commits it ({@link #untuned})
   * @param scheduler the command scheduler its mechanisms are registered with
   * @throws IllegalArgumentException if the template can't drive the definition's joints ({@link
   *     #problems}), or an interlock is about a joint the template doesn't drive
   */
  public Joints(
      RobotDefinition definition,
      Hardware hardware,
      Function<Joint, Supplier<MechanismControl>> tunings,
      Scheduler scheduler) {
    List<Finding> refused = new ArrayList<>(problems(definition));
    refused.addAll(unenforceable(definition));
    if (!refused.isEmpty()) {
      throw new IllegalArgumentException(
          String.join("; ", refused.stream().map(Finding::message).toList()));
    }
    this.definition = definition;
    this.scheduler = scheduler;
    RobotMath math = definition.math();
    sensors = new Sensors(definition, hardware::sensor, scheduler);
    List<JointMechanism> built = new ArrayList<>();
    for (Joint joint : driven(definition)) {
      JointMechanism mechanism =
          new JointMechanism(
              math,
              joint,
              tunings.apply(joint),
              hardware.joint(joint),
              readsAbsolute(joint) != null ? hardware.absoluteEncoder(joint) : null,
              sensors,
              scheduler);
      built.add(mechanism);
      byJoint.put(joint, mechanism);
    }
    joints = built.toArray(JointMechanism[]::new);
    power = new Power(definition, hardware.pdh());
    for (Sensor sensor : definition.math().sensors()) {
      if (unwired(sensor)) {
        unwired.add(sensor.label());
      }
    }
    // An interlock reads the joint it names through what that joint shows, so each joint's are
    // handed it once every joint is built: two joints may restrict each other.
    for (Interlock interlock : definition.everyInterlock()) {
      enforced(interlock.joint())
          .restrict(
              interlock,
              interlock.other().map(this::enforced).map(JointMechanism::view).orElse(null));
    }
  }

  /** Whether a sensor's definition gives it no wiring: it runs in the simulator alone. */
  private static boolean unwired(Sensor sensor) {
    return switch (sensor) {
      case Sensor.Switch onSwitch -> onSwitch.wiring().isEmpty();
      case Sensor.Rangefinder rangefinder -> rangefinder.wiring().isEmpty();
      case Sensor.Encoder encoder -> false;
    };
  }

  /** Each joint's control as its block commits it, never tuned: for a robot nobody tunes. */
  public static Function<Joint, Supplier<MechanismControl>> untuned() {
    return joint -> () -> ControllerSettings.control(joint);
  }

  /**
   * Each joint the template drives, in the definition's order: one with a control in its block,
   * turned by one drive of motors ({@link #whyNotDriven}). Each is a mechanism of its own.
   */
  public static List<Joint> driven(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<Joint> driven = new ArrayList<>();
    for (Joint joint : definition.joints()) {
      if (joint.driven().control().isPresent() && whyNotDriven(math, joint) == null) {
        driven.add(joint);
      }
    }
    return driven;
  }

  /**
   * Why the template can't drive a robot's joints as their blocks say, each about its joint;
   * nothing when it can: a joint with a control it doesn't drive yet ({@link #whyNotDriven}), and
   * settings its motor controllers would refuse, as the robot would say it at startup ({@link
   * ControllerSettings#findings}), wired or not. A robot's configuration asks this as it's built,
   * and the definition has already refused what its blocks say of themselves.
   */
  public static List<Finding> problems(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<Finding> problems = new ArrayList<>();
    for (Joint joint : definition.joints()) {
      if (joint.driven().control().isEmpty()) {
        continue;
      }
      String why = whyNotDriven(math, joint);
      if (why != null) {
        problems.add(
            Finding.refused(
                joint.label(),
                joint.label() + " has a control, and the template doesn't drive it: " + why));
      } else {
        ControllerSettings.findings(math, joint).stream()
            .filter(Finding::refused)
            .forEach(problems::add);
      }
    }
    return problems;
  }

  /**
   * Why the template doesn't drive a joint, or null when it does: one drive of motors turning a
   * shaft. Any motor of the catalog: in the simulator as it is, on a robot through the controller
   * its block names ({@link com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller}). A
   * joint that follows another is carried by its leader's moves; a joint driven by a servo, or by
   * two drives (a differential's pair, a hood's two servos), isn't driven by the template yet.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's
   */
  public static @Nullable String whyNotDriven(RobotMath math, Joint joint) {
    if (!math.motorsAlone(joint)) {
      return whyNotMotorsAlone(math, joint);
    }
    try {
      MechanismRotations.of(joint, math.drivesOf(joint).get(0));
    } catch (IllegalArgumentException e) {
      return String.valueOf(e.getMessage());
    }
    return null;
  }

  /** What turns a joint that isn't turned by motors alone ({@link RobotMath#motorsAlone}). */
  private static String whyNotMotorsAlone(RobotMath math, Joint joint) {
    Joint leader = math.leaderOf(joint);
    if (leader != null) {
      return "it follows " + leader.label() + ", whose moves carry it";
    }
    List<Drive> drives = math.drivesOf(joint);
    if (drives.isEmpty()) {
      List<String> movers = new ArrayList<>();
      for (Drive drive : math.drives()) {
        if (math.leverage(drive, joint, math.rests()) != 0) {
          movers.add(drive.label());
        }
      }
      return movers.isEmpty()
          ? "nothing drives it"
          : "it's moved by "
              + String.join(" and ", movers)
              + ", and the template drives only motors turning a shaft so far";
    }
    if (drives.size() > 1) {
      return "it's driven by "
          + String.join(" and ", drives.stream().map(Drive::label).toList())
          + " together, which the template doesn't drive yet";
    }
    return drives.get(0).label() + " isn't motors turning a shaft";
  }

  /**
   * A joint's absolute encoder, when the robot reads it itself and tells the motor where the joint
   * is from it: on a Systemcore port, or unwired, read only by the simulator. Null for none, or one
   * its motor controller reads, a CANcoder or a SPARK's data port, or for a joint whose control
   * names a homing recipe, which it homes by instead.
   */
  public static Driven.@Nullable AbsoluteEncoder readsAbsolute(Joint joint) {
    Driven.AbsoluteEncoder encoder = joint.driven().absoluteEncoder().orElse(null);
    return encoder == null
            || encoder.readByController()
            || joint.driven().control().flatMap(MechanismControl::homing).isPresent()
            || !joint.holdsPositions()
        ? null
        : encoder;
  }

  /**
   * Why the template can't enforce a robot's interlocks, each about the joint restricted; nothing
   * when it can enforce every one: an interlock about a joint it doesn't drive. A robot's
   * configuration asks this as it's built, before anything of the robot is made.
   */
  public static List<Finding> unenforceable(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<Finding> problems = new ArrayList<>();
    for (Interlock interlock : definition.everyInterlock()) {
      List<Joint> joints = new ArrayList<>(List.of(interlock.joint()));
      interlock.other().ifPresent(joints::add);
      for (Joint joint : joints) {
        String why = whyNotDriven(math, joint);
        if (why != null) {
          problems.add(
              Finding.refused(
                  interlock.joint().label(),
                  "\""
                      + interlock
                      + "\" can't be enforced: "
                      + joint.label()
                      + " has no mechanism, as "
                      + why));
          break;
        }
      }
    }
    return problems;
  }

  /** The mechanism of a joint an interlock speaks of, which has one ({@link #unenforceable}). */
  private JointMechanism enforced(Joint joint) {
    return Objects.requireNonNull(byJoint.get(joint), joint.label());
  }

  /**
   * A joint's mechanism. A joint that has none, one the template doesn't drive or one that isn't
   * the robot's, is refused as a move to a place not a joint's own is ({@link Refusals}): asked for
   * as the robot starts, it throws, saying why; once it runs, its mechanism is a stand-in, never
   * driven, whose every move ends {@link com.michaelgrundvig.frc.runtime.behavior.Outcome#REFUSED}
   * and every gate holds false, saying why.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's, or one the template
   *     doesn't drive, saying why, as the robot starts
   */
  public JointMechanism of(Joint joint) {
    JointMechanism mechanism = byJoint.get(joint);
    if (mechanism != null) {
      return mechanism;
    }
    String why = noMechanism(joint);
    Refusals.asBuilt(scheduler, why);
    return standIns.computeIfAbsent(
        joint,
        standIn -> JointMechanism.standIn(definition.math(), standIn, sensors, scheduler, why));
  }

  /** Why a joint has no mechanism: the template doesn't drive it, or it isn't the robot's. */
  private String noMechanism(Joint joint) {
    return definition.joints().contains(joint)
        ? joint.label() + " has no mechanism: " + whyNotDriven(definition.math(), joint)
        : joint.label() + " isn't one of " + possessive(definition.name()) + " joints";
  }

  /**
   * Several joints' mechanisms, as one requirement: {@code Command.requiring(joints.all(PIVOT,
   * ROLLER))}: the joints a command drives together.
   *
   * @throws IllegalArgumentException as {@link #of(Joint)} does, for any of them
   */
  public List<Mechanism> all(Joint... joints) {
    List<Mechanism> mechanisms = new ArrayList<>();
    for (Joint joint : joints) {
      mechanisms.add(of(joint));
    }
    return List.copyOf(mechanisms);
  }

  /**
   * Moves a configuration's joints to their named places together, each on its own profile, and
   * ends when every one is there; it requires exactly the joints it names. A joint it runs at an
   * output is there at once for a voltage or a duty cycle, and within its band for a speed. One
   * that names a joint the template doesn't drive is refused whenever it would start, saying why:
   * the robot still runs, and the startup's doubts name it.
   */
  public Move goTo(Configuration configuration) {
    return Move.configuration(
        configuration, mechanisms(configuration), true, refused(configuration), scheduler);
  }

  /**
   * Holds a configuration's joints at their named places and outputs until canceled: {@code
   * driver.rightTrigger().whileTrue(joints.run(FEEDING))}. Refused, as {@link #goTo(Configuration)}
   * is, if it names a joint the template doesn't drive.
   */
  public Move run(Configuration configuration) {
    return Move.configuration(
        configuration, mechanisms(configuration), false, refused(configuration), scheduler);
  }

  /**
   * Why a configuration's move can't run, or null when it can: a place not its joint's own, refused
   * as a move there is ({@link Refusals}, thrown as the robot starts), or the joints it names that
   * the template doesn't drive.
   */
  private @Nullable String refused(Configuration configuration) {
    for (Condition.At target : configuration.targets()) {
      String notOwn = Refusals.asBuilt(scheduler, definition.whyNotOwn(target));
      if (notOwn != null) {
        return notOwn;
      }
    }
    return undriven(definition, configuration);
  }

  /**
   * A gate on a configuration: every joint it names at its named place, or running at its named
   * output, "at stowed". Never true for one that names a joint the template doesn't drive.
   */
  public Gate isAt(Configuration configuration) {
    List<Gate> terms = new ArrayList<>();
    for (Condition.At target : configuration.targets()) {
      JointMechanism mechanism = byJoint.get(target.joint());
      terms.add(
          mechanism != null
              ? target.isOutput()
                  ? mechanism.isAt(target.output().orElseThrow())
                  : mechanism.isAt(target.position().orElseThrow())
              : Gate.of(scheduler, target.joint().label() + " driven", () -> false));
    }
    return Gate.all("at " + configuration.name(), terms.toArray(Gate[]::new));
  }

  /**
   * A gate on a condition the robot's model says, named as the condition reads: {@code
   * joints.gate(PIVOT.atLeast(Degrees.of(110)))} is "pivot at least 110°" ({@link #gate(String,
   * Condition)}).
   *
   * @throws IllegalArgumentException as {@link #gate(String, Condition)} does, as the robot starts
   */
  public Gate gate(Condition condition) {
    return gate(condition.toString(), condition);
  }

  /**
   * A gate on a condition the robot's model says, named for what it means to the robot, logged as
   * {@code Gates/<name>}: where a joint is, {@code joints.gate("intake out of the frame",
   * PIVOT.atLeast(Degrees.of(110)))}, {@code ELEVATOR.between(...)}; how fast one runs, {@code
   * SHOOTER.upToSpeed(RPM.of(3000))}; or what a sensor reads, {@code joints.gate("coral in claw",
   * CLAW_BEAM.tripped())}. Combine and debounce it as any gate ({@link Gate#all}, {@link
   * Gate#debounce}). A place it names is where this robot puts it, so a practice robot's gate on
   * {@code PIVOT.atLeast(PIVOT_OUT)} opens from its own out; a joint at one of its places, {@code
   * PIVOT.at(PIVOT_OUT)}, is there within its band, as {@link JointMechanism#isAt(Position)} is.
   *
   * <p>A condition on what isn't known holds false, saying why: a joint that doesn't know where it
   * is (unhomed, faulted, silent), one whose motor controllers don't answer, a sensor that's lost
   * ("claw beam lost"). One on a joint with no mechanism, a sensor the robot hasn't, or a place not
   * the joint's own ({@code PIVOT.atLeast(ELEVATOR_CLEAR)}), is refused as {@link #of} refuses the
   * joint ({@link Refusals#gate}): thrown as the robot starts, and once it runs, a gate that holds
   * false saying why.
   *
   * @throws IllegalArgumentException for a joint with no mechanism, a sensor the robot hasn't, or a
   *     place not the joint's own, as the robot starts
   */
  public Gate gate(String name, Condition condition) {
    String notOwn = definition.whyNotOwn(condition);
    if (notOwn != null) {
      return Refusals.gate(scheduler, name, notOwn);
    }
    return switch (definition.own(condition)) {
      case Condition.OnSensor onSensor -> sensors.gate(name, onSensor);
      case Condition.At at -> {
        JointMechanism mechanism = byJoint.get(at.joint());
        if (mechanism == null) {
          yield Refusals.gate(scheduler, name, noMechanism(at.joint()));
        }
        yield at.isOutput()
            ? mechanism.isAt(name, mechanism.joint().output(at.name()))
            : mechanism.isAt(name, mechanism.joint().position(at.name()));
      }
      case Condition.OnJoint where ->
          reading(
              name,
              where.joint(),
              (view, why) -> {
                if (!view.knowsWhere()) {
                  why.add(where.joint().label() + " doesn't know where it is");
                  return false;
                }
                return where.holdsAt(view.position());
              });
      case Condition.UpToSpeed speed ->
          reading(
              name,
              speed.joint(),
              (view, why) -> {
                if (!view.answers()) {
                  why.add(speed.joint().label() + " isn't answering");
                  return false;
                }
                return speed.holdsAt(view.velocity());
              });
    };
  }

  /**
   * A gate on what a joint's mechanism reads, where it is or how fast it runs; refused, saying why,
   * for a joint with no mechanism.
   *
   * @param holds whether it holds, given what the joint shows, adding why not
   */
  private Gate reading(String name, Joint joint, BiPredicate<JointView, List<String>> holds) {
    JointMechanism mechanism = byJoint.get(joint);
    if (mechanism == null) {
      return Refusals.gate(scheduler, name, noMechanism(joint));
    }
    JointView view = mechanism.view();
    return new Gate(scheduler, name, why -> holds.test(view, why));
  }

  /**
   * Why a configuration can't run, or null when it can: the joints it names that the template
   * doesn't drive, each with why ({@link #whyNotDriven}). The startup's doubts list these.
   */
  public static @Nullable String cantRun(RobotDefinition definition, Configuration configuration) {
    String undriven = undriven(definition, configuration);
    return undriven == null ? null : configuration.name() + " can't run: " + undriven;
  }

  /** The joints a configuration names that the template doesn't drive, each with why; or null. */
  private static @Nullable String undriven(
      RobotDefinition definition, Configuration configuration) {
    RobotMath math = definition.math();
    List<String> undriven = new ArrayList<>();
    for (Condition.At target : configuration.targets()) {
      String why = whyNotDriven(math, target.joint());
      if (why != null) {
        undriven.add(target.joint().label() + " has no mechanism: " + why);
      }
    }
    return undriven.isEmpty() ? null : String.join("; ", undriven);
  }

  /** Every joint's mechanism, in the definition's order. */
  public List<JointMechanism> all() {
    return List.of(joints);
  }

  /**
   * Where every joint is as of this loop, by joint: what localization needs of a camera riding one.
   * A joint that doesn't know where it is yet isn't in it.
   */
  public Map<Joint, Double> positions() {
    Map<Joint, Double> positions = new LinkedHashMap<>();
    for (JointMechanism joint : joints) {
      if (Double.isFinite(joint.position())) {
        positions.put(joint.joint(), joint.position());
      }
    }
    return positions;
  }

  /**
   * What the robot's limit switches are doubted for this loop: each one reading tripped nowhere
   * near where it trips, or clear well past it, with its joint homed. Empty while they agree.
   */
  public List<String> switchDoubts() {
    List<String> doubts = new ArrayList<>();
    for (JointMechanism joint : joints) {
      doubts.addAll(joint.switchDoubts());
    }
    return doubts;
  }

  /**
   * The motor controllers that read as drawing nothing on their PDH channels while their joints
   * work, as of this loop, each said with what to check: a tripped breaker, unplugged power, or a
   * channel its block names wrong. Empty while each draws as it should.
   */
  public List<String> deadChannels() {
    List<String> dead = new ArrayList<>();
    for (JointMechanism joint : joints) {
      dead.addAll(joint.deadChannels());
    }
    return dead;
  }

  /** The robot's switches and rangefinders, read with the joints. */
  public Sensors sensors() {
    return sensors;
  }

  /** The robot's PDH, read with the joints: each channel's current, and each joint's. */
  public Power power() {
    return power;
  }

  /**
   * Reads every joint, then every sensor, then the PDH: once a loop, before the command scheduler
   * runs. Then says what's wrong in its alerts.
   */
  public void read(long nowNanos, boolean enabled) {
    for (JointMechanism joint : joints) {
      joint.read(nowNanos, enabled);
    }
    sensors.read();
    power.read();
    for (JointMechanism joint : joints) {
      joint.sensed();
      joint.powered(power);
    }
    List<String> lostNow = new ArrayList<>(sensors.lost());
    lostNow.removeAll(unwired);
    lost.set(lostNow);
    doubted.set(switchDoubts());
    dead.set(deadChannels());
    pdhSilent.set(
        definition
            .pdh()
            .filter(pdh -> !power.reads())
            .map(pdh -> List.of(String.valueOf(pdh.id())))
            .orElse(List.of()));
  }

  /** Releases its alerts, and each joint's, so a test can build another robot. */
  @Override
  public void close() {
    for (JointMechanism joint : joints) {
      joint.close();
    }
    standIns.values().forEach(JointMechanism::close);
    lost.close();
    doubted.close();
    dead.close();
    pdhSilent.close();
  }

  /** Applies what the commands asked of each joint, or its idle: once a loop, after them. */
  public void write() {
    for (JointMechanism joint : joints) {
      joint.write();
    }
  }

  /** Each target's joint's mechanism, in order: null for one the template doesn't drive. */
  private List<@Nullable JointMechanism> mechanisms(Configuration configuration) {
    List<@Nullable JointMechanism> mechanisms = new ArrayList<>();
    for (Condition.At target : configuration.targets()) {
      mechanisms.add(byJoint.get(target.joint()));
    }
    return mechanisms;
  }
}

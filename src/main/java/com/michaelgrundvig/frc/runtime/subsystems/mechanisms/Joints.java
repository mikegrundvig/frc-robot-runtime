package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Refusals;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.record.ControlSentences;
import com.michaelgrundvig.frc.world.record.GateTerm;
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
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiPredicate;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent;
import org.wpilib.util.Alert;

/**
 * Every driven joint of the robot. Each is a Commands v3 {@link Mechanism} that the template builds
 * from the robot's definition, where the joint's block says how it is driven and wired ({@link
 * JointMechanism}). The composition root builds this once. Behavior code looks a joint's mechanism
 * up by the joint itself, never by a label:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=look-up}
 *
 * <p><b>Configurations.</b> A configuration names a place or an output for each of several joints.
 * Its joints move there together, and {@code joints.run(FEEDING)} holds them there, outputs
 * included:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=configurations}
 *
 * <p><b>Gates.</b> Any condition the model can say is a gate: where a joint is, how fast it runs,
 * or what a sensor reads, such as {@code joints.gate("coral in claw", CLAW_BEAM.tripped())} ({@link
 * #gate(String, Condition)}).
 *
 * <p><b>Interlocks.</b> The definition's interlocks are enforced beneath every command, each on the
 * joint it restricts ({@link JointMechanism}). An interlock about a joint the template doesn't
 * drive cannot be enforced, so the robot is refused ({@link #unenforceable}).
 *
 * <p><b>Each loop</b> every joint is read before the command scheduler runs ({@link #read}), and
 * what the commands asked is applied after it ({@link #write}), so a goal set this loop is applied
 * this loop. Whatever reads wrong is said in alerts of this class's own: sensors lost, limit
 * switches doubted, motor controllers drawing nothing on their channels, and a PDH that doesn't
 * answer.
 */
@CheckReturnValue
public final class Joints implements AutoCloseable {
  /**
   * What the joints read and drive, made for the mode the program runs in:
   *
   * <ul>
   *   <li>On a robot, the devices each block's wiring names.
   *   <li>In the simulator, the simulator's own, built from the robot's definition.
   *   <li>In a replay, nothing, since the log has every reading.
   * </ul>
   *
   * <p>The joints ask for each piece once, as they are built, in this order: every sensor, then
   * each driven joint's motor controllers with the absolute encoder the robot reads itself, then
   * the PDH. A switch on a motor controller's limit input is never asked for: its joint reads it
   * from the controller.
   *
   * <p>A test that needs only its joints' hardware gives it as a lambda, {@code joint -> io}, and
   * everything else is none. The robot's own hardware ({@code
   * com.michaelgrundvig.frc.runtime.hardware.Hardware}) gives every piece.
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

  /** The names behind the numbers in what the joints log, logged once. */
  private final RecordedNames names;

  private final RobotDefinition definition;
  private final Sensors sensors;
  private final Power power;

  /** The command scheduler its mechanisms and gates are registered with. */
  private final Scheduler scheduler;

  /**
   * The commands that have had a turn and have not ended, each one itself and not another equal to
   * it. A command is looked at once, on its first turn, for the joints it takes ({@link
   * #tellItsJoints}); on every later turn finding it here is all that is done.
   */
  private final Set<Command> begun = Collections.newSetFromMap(new IdentityHashMap<>());

  /**
   * The sensors with no wiring in the definition, by label: on a robot an alert of their own says
   * so, so they aren't said again as lost.
   */
  private final List<String> unwired = new ArrayList<>();

  /** The sensors that have stopped answering, by label. */
  private final ListAlert lost =
      new ListAlert("Sensors", "lost", Alert.Level.HIGH, Joints::saysTheSensorsAreLost);

  /** The limit switches whose readings disagree with where their joints are, each a sentence. */
  private final ListAlert doubted =
      new ListAlert("Sensors", "doubted", Alert.Level.HIGH, Joints::joinedAsSentences);

  /** The motor controllers drawing nothing on their PDH channels, each a sentence. */
  private final ListAlert dead =
      new ListAlert("Power", "dead", Alert.Level.HIGH, Joints::joinedAsSentences);

  /** The PDH the robot declares, by its CAN ID, while it doesn't answer. */
  private final ListAlert pdhSilent =
      new ListAlert("Power", "silent", Alert.Level.MEDIUM, Joints::saysThePdhIsSilent);

  /** The alert's text for sensors that are lost: "claw beam is lost: ...". */
  private static String saysTheSensorsAreLost(List<String> labels) {
    boolean one = labels.size() == 1;
    return String.join(", ", labels)
        + (one ? " is" : " are")
        + " lost: nothing waiting on "
        + (one ? "it" : "them")
        + " holds. Check the wiring and the CAN bus.";
  }

  /** The alert's text for a PDH that doesn't answer, from a list holding its CAN ID. */
  private static String saysThePdhIsSilent(List<String> ids) {
    return "The PDH isn't answering at CAN ID "
        + ids.get(0)
        + ", so no channel's current is read. Check its CAN wiring, and its ID in REV's"
        + " Hardware Client.";
  }

  /** An alert's text for a list of whole sentences: the sentences, one after another. */
  private static String joinedAsSentences(List<String> sentences) {
    return String.join("; ", sentences);
  }

  /**
   * Builds every driven joint's mechanism, with the robot's switches and rangefinders ({@link
   * #sensors}) and its PDH ({@link #power}). A joint whose absolute encoder the robot reads itself
   * ({@link #readsAbsolute}) waits for the encoder's first reading, and then tells its motor
   * controller where the joint is.
   *
   * @param definition what the robot is. Each joint's block says how it is driven and wired.
   * @param hardware what each joint, each sensor and the PDH read and drive, for the mode the
   *     program runs in
   * @param tunings each joint's control, which is read every loop. When its gains or current limits
   *     change, they are handed to the joint's motor controller. It is either the control as the
   *     pit tunes it ({@link JointMechanism#tuning}), or the control its block commits ({@link
   *     #untuned}).
   * @param scheduler the command scheduler its mechanisms are registered with
   * @throws IllegalArgumentException if the template can't drive the definition's joints ({@link
   *     #problems}), or an interlock is about a joint the template doesn't drive
   */
  public Joints(
      RobotDefinition definition,
      Hardware hardware,
      Function<Joint, Supplier<MechanismControl>> tunings,
      Scheduler scheduler) {
    refuseWhatCannotBeBuilt(definition);
    this.definition = definition;
    this.scheduler = scheduler;
    names = new RecordedNames(definition);
    // The hardware is asked for in this order: the sensors, each joint's, then the PDH.
    sensors = new Sensors(definition, hardware::sensor, scheduler);
    List<JointMechanism> built = new ArrayList<>();
    for (Joint joint : driven(definition)) {
      JointMechanism mechanism =
          new JointMechanism(
              definition,
              joint,
              tunings.apply(joint),
              hardware.joint(joint),
              absoluteEncoderTheRobotReads(joint).map(encoder -> hardware.absoluteEncoder(joint)),
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
    // An interlock reads the other joint it names through what that joint's mechanism shows. So
    // each mechanism is handed its interlocks only now, once every mechanism is built: two joints
    // may restrict each other.
    List<Interlock> everyInterlock = definition.everyInterlock();
    for (int index = 0; index < everyInterlock.size(); index++) {
      Interlock interlock = everyInterlock.get(index);
      JointMechanism restricted = enforced(interlock.joint());
      Optional<JointView> other = interlock.other().map(this::enforced).map(JointMechanism::view);
      restricted.restrict(interlock, other, index);
    }
    scheduler.addEventListener(this::heard);
  }

  /**
   * Hears everything the scheduler does, to tell each joint when a command takes it and when a
   * command that may have had it ends. A joint's gates answer for what the command that has it
   * asked ({@link Tenure}), and only the scheduler knows which command that is before the command
   * has asked anything.
   */
  private void heard(SchedulerEvent event) {
    switch (event) {
      case SchedulerEvent.Mounted turn -> tellItsJoints(turn.command());
      case SchedulerEvent.Completed done -> tellEveryJointItEnded(done.command());
      case SchedulerEvent.CompletedWithError failed -> tellEveryJointItEnded(failed.command());
      case SchedulerEvent.Canceled canceled -> tellEveryJointItEnded(canceled.command());
      default -> {}
    }
  }

  /**
   * Tells each joint a command requires that the command has it, on the command's first turn. A
   * joint is its command's from that turn, and not from when the command was scheduled: a command
   * that is scheduled may still be put aside for another before it runs.
   */
  private void tellItsJoints(Command command) {
    boolean hadATurnBefore = !begun.add(command);
    if (hadATurnBefore) {
      return;
    }
    for (JointMechanism joint : joints) {
      if (command.requires(joint)) {
        joint.takenBy(holderOf(joint, command));
      }
    }
  }

  /**
   * The command that has a joint, given one that requires it: that command, or the one furthest out
   * of those that started it and require the joint too. A shot that requires the shooter and forks
   * the command that spins it up has the shooter, before the fork and after the spin-up ends.
   */
  private Command holderOf(JointMechanism joint, Command command) {
    Command holder = command;
    for (Command up = scheduler.getParentOf(command); up != null; up = scheduler.getParentOf(up)) {
      if (up.requires(joint)) {
        holder = up;
      }
    }
    return holder;
  }

  /** Tells every joint that a command has ended, which lets go of each joint it had. */
  private void tellEveryJointItEnded(Command command) {
    begun.remove(command);
    for (JointMechanism joint : joints) {
      joint.endedFor(command);
    }
  }

  /**
   * Refuses a robot whose joints the template can't drive as their blocks say, or whose interlocks
   * it can't enforce, with every reason in one message.
   *
   * @throws IllegalArgumentException if there is any reason
   */
  private static void refuseWhatCannotBeBuilt(RobotDefinition definition) {
    List<Finding> refused = new ArrayList<>(problems(definition));
    refused.addAll(unenforceable(definition));
    if (!refused.isEmpty()) {
      throw new IllegalArgumentException(
          String.join("; ", refused.stream().map(Finding::message).toList()));
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
   * Why the template can't drive a robot's joints as their blocks say. Each finding is about one
   * joint, and the list is empty when every joint can be driven. There are two kinds:
   *
   * <ul>
   *   <li>A joint has a control, and the template doesn't drive that kind of joint yet ({@link
   *       #whyNotDriven}).
   *   <li>A joint's settings are ones its motor controllers would refuse, said as the robot would
   *       say it at startup ({@link ControllerSettings#findings}). This is checked whether or not
   *       the joint is wired.
   * </ul>
   *
   * <p>A robot's configuration asks this as it is built. By then the definition has already refused
   * whatever is wrong with a block by itself.
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
        continue;
      }
      // The template drives it. What remains is what its motor controllers would refuse.
      for (Finding finding : ControllerSettings.findings(math, joint)) {
        if (finding.refused()) {
          problems.add(finding);
        }
      }
    }
    return problems;
  }

  /**
   * Why the template doesn't drive a joint, or null when it does.
   *
   * <p>The template drives a joint moved by one drive of motors turning a shaft. The motors can be
   * any in the catalog: the simulator runs them as they are, and a robot runs them through the
   * controller the joint's block names ({@link
   * com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller}). It does not drive a joint
   * that follows another, which its leader's moves carry. And it does not yet drive a joint moved
   * by a servo, or by two drives together (a differential's pair, a hood's two servos).
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
    if (drives.size() == 1) {
      return drives.get(0).label() + " isn't motors turning a shaft";
    }
    if (drives.size() > 1) {
      return "it's driven by "
          + String.join(" and ", drives.stream().map(Drive::label).toList())
          + " together, which the template doesn't drive yet";
    }
    // No drive is the joint's own. Another joint's drive may still move it, through a coupling.
    List<String> movers = driveLabelsThatMove(math, joint);
    if (movers.isEmpty()) {
      return "nothing drives it";
    }
    return "it's moved by "
        + String.join(" and ", movers)
        + ", and the template drives only motors turning a shaft so far";
  }

  /** The labels of the robot's drives that move a joint at all, with the robot at rest. */
  private static List<String> driveLabelsThatMove(RobotMath math, Joint joint) {
    List<String> movers = new ArrayList<>();
    for (Drive drive : math.drives()) {
      if (math.leverage(drive, joint, math.rests()) != 0) {
        movers.add(drive.label());
      }
    }
    return movers;
  }

  /**
   * A joint's absolute encoder, when the robot reads it itself and tells the motor controller where
   * the joint is from it. Such an encoder is on a Systemcore port, or is unwired and read only by
   * the simulator. Null when the joint has no absolute encoder, when its motor controller reads the
   * encoder (a CANcoder, or one on a SPARK's data port), and when the joint's control names a
   * homing recipe, which the joint homes by instead.
   */
  public static Driven.@Nullable AbsoluteEncoder readsAbsolute(Joint joint) {
    return absoluteEncoderTheRobotReads(joint).orElse(null);
  }

  /**
   * A joint's absolute encoder, when the robot reads it itself ({@link #readsAbsolute}). Empty for
   * a joint with no absolute encoder, and for each case where something else finds where the joint
   * is.
   */
  private static Optional<Driven.AbsoluteEncoder> absoluteEncoderTheRobotReads(Joint joint) {
    Optional<Driven.AbsoluteEncoder> encoder = joint.driven().absoluteEncoder();
    if (encoder.isEmpty()) {
      return Optional.empty();
    }
    // A CANcoder or a SPARK's data port is read by the joint's motor controller.
    if (encoder.get().readByController()) {
      return Optional.empty();
    }
    // A joint whose control names a homing recipe homes by that instead.
    boolean homesByARecipe = joint.driven().control().flatMap(MechanismControl::homing).isPresent();
    if (homesByARecipe) {
      return Optional.empty();
    }
    // A joint that holds no positions, such as a roller, has no use for knowing where it is.
    if (!joint.holdsPositions()) {
      return Optional.empty();
    }
    return encoder;
  }

  /**
   * Why the template can't enforce a robot's interlocks. An interlock can't be enforced when it is
   * about a joint the template doesn't drive. Each finding is about the joint the interlock
   * restricts, and the list is empty when every interlock can be enforced. A robot's configuration
   * asks this as it is built, before anything of the robot is made.
   */
  public static List<Finding> unenforceable(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<Finding> problems = new ArrayList<>();
    for (Interlock interlock : definition.everyInterlock()) {
      whyNotEnforceable(math, interlock)
          .ifPresent(why -> problems.add(Finding.refused(interlock.joint().label(), why)));
    }
    return problems;
  }

  /**
   * Why one interlock can't be enforced, if it can't: the first of its joints that has no
   * mechanism, the joint it restricts and then the other joint it reads, with why that joint has
   * none.
   */
  private static Optional<String> whyNotEnforceable(RobotMath math, Interlock interlock) {
    List<Joint> itsJoints = new ArrayList<>(List.of(interlock.joint()));
    interlock.other().ifPresent(itsJoints::add);
    for (Joint joint : itsJoints) {
      String why = whyNotDriven(math, joint);
      if (why != null) {
        return Optional.of(
            "\""
                + interlock
                + "\" can't be enforced: "
                + joint.label()
                + " has no mechanism, as "
                + why);
      }
    }
    return Optional.empty();
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
    Refusals.asBuilt(scheduler, Optional.of(why));
    return standIns.computeIfAbsent(
        joint, standIn -> JointMechanism.standIn(definition, standIn, sensors, scheduler, why));
  }

  /** Why a joint has no mechanism: the template doesn't drive it, or it isn't the robot's. */
  private String noMechanism(Joint joint) {
    boolean isTheRobots = definition.joints().contains(joint);
    if (!isTheRobots) {
      return joint.label() + " isn't one of " + possessive(definition.name()) + " joints";
    }
    return joint.label() + " has no mechanism: " + whyNotDriven(definition.math(), joint);
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
        configuration,
        mechanisms(configuration),
        new MovePlan.GoTo(),
        refused(configuration),
        scheduler,
        definition);
  }

  /**
   * Holds a configuration's joints at their named places and outputs until canceled: {@code
   * driver.rightTrigger().whileTrue(joints.run(FEEDING))}. Refused, as {@link #goTo(Configuration)}
   * is, if it names a joint the template doesn't drive.
   */
  public Move run(Configuration configuration) {
    return Move.configuration(
        configuration,
        mechanisms(configuration),
        new MovePlan.Run(Optional.empty()),
        refused(configuration),
        scheduler,
        definition);
  }

  /**
   * Why a configuration's move cannot run, if it cannot. Either one of its places is not its
   * joint's own, which is refused as a move to such a place is ({@link Refusals}: thrown as the
   * robot starts), or it names joints the template does not drive.
   */
  private Optional<String> refused(Configuration configuration) {
    for (Condition.At target : configuration.targets()) {
      Optional<String> notOwn =
          Refusals.asBuilt(scheduler, Optional.ofNullable(definition.whyNotOwn(target)));
      if (notOwn.isPresent()) {
        return notOwn;
      }
    }
    return Optional.ofNullable(undriven(definition, configuration));
  }

  /**
   * A gate on a configuration: every joint it names at its named place, or running at its named
   * output, "at stowed". Never true for one that names a joint the template doesn't drive.
   */
  public Gate isAt(Configuration configuration) {
    List<Gate> terms = new ArrayList<>();
    for (Condition.At target : configuration.targets()) {
      terms.add(isAt(target));
    }
    return Gate.all("at " + configuration.name(), terms.toArray(Gate[]::new));
  }

  /**
   * A gate on one of a configuration's targets: its joint at its named place, or running at its
   * named output. For a joint the template does not drive, it is a gate that never holds.
   */
  private Gate isAt(Condition.At target) {
    JointMechanism mechanism = byJoint.get(target.joint());
    if (mechanism == null) {
      return Gate.of(scheduler, target.joint().label() + " driven", () -> false);
    }
    if (target.isOutput()) {
      return mechanism.isAt(target.output().orElseThrow());
    }
    return mechanism.isAt(target.position().orElseThrow());
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
      case Condition.At at -> atGate(name, at);
      case Condition.OnJoint where -> reading(name, where.joint(), whetherItIs(name, where));
      case Condition.UpToSpeed speed -> reading(name, speed.joint(), whetherItRunsAt(name, speed));
    };
  }

  /**
   * A gate on a joint being at one of its named places, or running at one of its named outputs.
   * Refused, saying why, for a joint with no mechanism.
   */
  private Gate atGate(String name, Condition.At at) {
    JointMechanism mechanism = byJoint.get(at.joint());
    if (mechanism == null) {
      return Refusals.gate(scheduler, name, noMechanism(at.joint()));
    }
    if (at.isOutput()) {
      return mechanism.isAt(name, mechanism.joint().output(at.name()));
    }
    return mechanism.isAt(name, mechanism.joint().position(at.name()));
  }

  /**
   * Whether a condition on where a joint is holds, given what the joint shows. It holds false,
   * saying why, while the joint doesn't know where it is.
   */
  private BiPredicate<JointView, Gate.Terms> whetherItIs(String name, Condition.OnJoint where) {
    GateTerm unknown =
        new GateTerm(
            GateTerm.Kind.JOINT_POSITION_UNKNOWN, definition.joints().indexOf(where.joint()));
    String unknownShown = ControlSentences.gateTerm(definition, name, unknown, List.of());
    return (view, why) -> {
      if (!view.knowsWhere()) {
        why.add(unknown, unknownShown);
        return false;
      }
      return where.holdsAt(view.position());
    };
  }

  /**
   * Whether a condition on how fast a joint runs holds, given what the joint shows. It holds false,
   * saying why, while the joint's motor controllers don't answer.
   */
  private BiPredicate<JointView, Gate.Terms> whetherItRunsAt(
      String name, Condition.UpToSpeed speed) {
    GateTerm silent =
        new GateTerm(GateTerm.Kind.JOINT_NOT_ANSWERING, definition.joints().indexOf(speed.joint()));
    String silentShown = ControlSentences.gateTerm(definition, name, silent, List.of());
    return (view, why) -> {
      if (!view.answers()) {
        why.add(silent, silentShown);
        return false;
      }
      return speed.holdsAt(view.velocity());
    };
  }

  /**
   * A gate on what a joint's mechanism reads, where it is or how fast it runs; refused, saying why,
   * for a joint with no mechanism.
   *
   * @param holds whether it holds, given what the joint shows, adding why not
   */
  private Gate reading(String name, Joint joint, BiPredicate<JointView, Gate.Terms> holds) {
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
    names.logOnce();
    for (JointMechanism joint : joints) {
      joint.read(nowNanos, enabled);
    }
    sensors.read();
    power.read();
    // Each joint then takes what the sensors and the PDH read this loop: its limit switches, and
    // the current on its motor controllers' channels.
    for (JointMechanism joint : joints) {
      joint.sensed();
      joint.powered(power);
    }
    sayWhatIsWrong();
  }

  /** Raises, changes or drops each alert to match what was read this loop. */
  private void sayWhatIsWrong() {
    // A sensor with no wiring never answers. Its own alert says so on a robot, so it isn't said
    // again here as lost.
    List<String> lostNow = new ArrayList<>(sensors.lost());
    lostNow.removeAll(unwired);
    lost.set(lostNow);
    doubted.set(switchDoubts());
    dead.set(deadChannels());
    pdhSilent.set(silentPdhIds());
  }

  /** The CAN ID of the PDH the robot declares, while it doesn't answer. Empty otherwise. */
  private List<String> silentPdhIds() {
    if (definition.pdh().isEmpty() || power.reads()) {
      return List.of();
    }
    return List.of(String.valueOf(definition.pdh().get().id()));
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

  /**
   * The mechanism of each of a configuration's targets' joints, in the targets' order: empty for a
   * joint the template does not drive.
   */
  private List<Optional<JointMechanism>> mechanisms(Configuration configuration) {
    List<Optional<JointMechanism>> mechanisms = new ArrayList<>();
    for (Condition.At target : configuration.targets()) {
      mechanisms.add(Optional.ofNullable(byJoint.get(target.joint())));
    }
    return mechanisms;
  }
}

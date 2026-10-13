package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RPM;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.math.filter.Debouncer;

/**
 * What "at goal" means, scenario by scenario: a joint is at its goal only with respect to what the
 * command that has it has asked. Each scenario runs for a joint held at positions and for one run
 * at speeds, and for a gate made as the robot starts and one made once it runs, which are kept up
 * to date in different ways and must answer the same.
 *
 * <p>The gate is read where a team's code reads it: inside a command, in the same turn as whatever
 * changed what the joint is asked. That is the read that used to pass on the answer for the idle
 * joint a command had just taken.
 */
@ExtendWith(RobotSim.class)
class AtGoalSimTest {
  private static final double RPM_IN_RADIANS_PER_SECOND = 2 * Math.PI / 60;

  /** Which joint a scenario is about, and which of its gates. */
  enum Subject {
    /** The pivot, held at positions, its gate made as the robot starts. */
    PIVOT_GATE_MADE_AT_THE_START(true, true),
    /** The pivot, its gate made once the robot runs. */
    PIVOT_GATE_MADE_RUNNING(true, false),
    /** The roller, run at speeds, its gate made as the robot starts. */
    ROLLER_GATE_MADE_AT_THE_START(false, true),
    /** The roller, its gate made once the robot runs. */
    ROLLER_GATE_MADE_RUNNING(false, false);

    final boolean isThePivot;
    final boolean madeAtTheStart;

    Subject(boolean isThePivot, boolean madeAtTheStart) {
      this.isThePivot = isThePivot;
      this.madeAtTheStart = madeAtTheStart;
    }
  }

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);

  /** What a command read of the gate, in the order it read it. */
  private final List<Boolean> read = new ArrayList<>();

  /** The loops in which a command read the gate as true with the joint not where it was asked. */
  private final List<Integer> trueWhileNotThere = new ArrayList<>();

  private int loopsWatched;

  /** The gate under test: the subject's joint's "at goal", made when the subject says. */
  private Gate atGoal(Subject subject) {
    return made(subject, JointMechanism::isAtGoal);
  }

  /**
   * A gate on the subject's joint, made when the subject says: before the scheduler first runs, or
   * a few loops after. Either way the robot has run a few loops by the time this hands it back.
   */
  private Gate made(Subject subject, Function<JointMechanism, Gate> gate) {
    JointMechanism joint = joint(subject);
    if (subject.madeAtTheStart) {
      Gate made = gate.apply(joint);
      loops(5);
      return made;
    }
    loops(5);
    return gate.apply(joint);
  }

  private void loops(int count) {
    for (int loop = 0; loop < count; loop++) {
      sim.loop();
    }
  }

  private JointMechanism joint(Subject subject) {
    return subject.isThePivot ? pivot : roller;
  }

  /**
   * The first thing a scenario asks of the joint: the pivot out at 130°, the roller at 1500 rpm.
   */
  private Move first(Subject subject) {
    return subject.isThePivot ? pivot.goTo(PIVOT_OUT) : roller.run(ROLLER_INTAKING);
  }

  /**
   * A second request, for what the joint is already doing, asked another way: the same angle as a
   * computed one, and the same speed as a target to follow.
   */
  private Move sameAgainAskedAnew(Subject subject) {
    return subject.isThePivot ? pivot.goTo(Degrees.of(130)) : roller.follow(() -> RPM.of(1500));
  }

  /** Whether the joint is really where the first request asks, within the band it may stray in. */
  private boolean isThere(Subject subject) {
    if (subject.isThePivot) {
      return Math.abs(sim.simulation.position(PIVOT) - Math.toRadians(130)) <= Math.toRadians(4);
    }
    double off = sim.simulation.velocity(ROLLER) - 1500 * RPM_IN_RADIANS_PER_SECOND;
    return Math.abs(off) <= 0.06 * 1500 * RPM_IN_RADIANS_PER_SECOND;
  }

  /** A command that has the subject's joint and does as a scenario says. */
  private Command holding(Subject subject, Consumer<Coroutine> does) {
    return Command.requiring(joint(subject)).executing(does).named("Holds the joint");
  }

  /**
   * Reads the gate every loop from now on, from inside the command, and notes each loop it says the
   * joint is at its goal while the joint is not where the first request asked.
   */
  private void watch(Coroutine co, Subject subject, Gate gate) {
    while (true) {
      boolean saysAtGoal = gate.getAsBoolean();
      read.add(saysAtGoal);
      if (saysAtGoal && !isThere(subject)) {
        trueWhileNotThere.add(loopsWatched);
      }
      loopsWatched++;
      co.yield();
    }
  }

  /** Idle and stopped, with nobody's command on it, a joint is where it is asked to be. */
  @ParameterizedTest
  @EnumSource(Subject.class)
  void idleAndStoppedItIsAtItsGoal(Subject subject) {
    Gate gate = atGoal(subject);

    boolean readByTheTest = gate.getAsBoolean();
    sim.run(Command.noRequirements(co -> read.add(gate.getAsBoolean())).named("Reads"), 0.04);

    assertThat(readByTheTest).isTrue();
    assertThat(read).as("read by a command that leaves the joint alone").containsExactly(true);
  }

  /**
   * A command takes the joint and asks nothing of it. From its first turn the joint is not at its
   * goal, for as long as that lasts: the command has not said what its goal is. When the command
   * ends, the joint is nobody's again, and idle and stopped it is at its goal.
   */
  @ParameterizedTest
  @EnumSource(Subject.class)
  void takenByACommandThatHasNotAskedItIsNotAtItsGoal(Subject subject) {
    Gate gate = atGoal(subject);
    Command takes =
        holding(
            subject,
            co -> {
              for (int loop = 0; loop < 5; loop++) {
                read.add(gate.getAsBoolean());
                co.yield();
              }
            });

    sim.scheduler.schedule(takes);
    loops(6);
    boolean rightAfterItEnded = gate.getAsBoolean();
    sim.loop();

    assertThat(read)
        .as("each loop the command had it")
        .containsExactly(false, false, false, false, false);
    assertThat(rightAfterItEnded).as("idle again, and judged as idle").isTrue();
    assertThat(gate.getAsBoolean()).isTrue();
  }

  /**
   * The shape of the guide's shot: a command takes the joint, and a command it forks does the
   * asking, here three loops later. The joint is not at its goal through the gap, nor in the turn
   * it is asked, nor while it is on its way. It is once it is there.
   */
  @ParameterizedTest
  @EnumSource(Subject.class)
  void takenByAParentWhoseChildAsksLaterItIsNotAtItsGoalUntilItIsThere(Subject subject) {
    Gate gate = atGoal(subject);
    List<Boolean> inTheGap = new ArrayList<>();
    List<Boolean> asItIsAsked = new ArrayList<>();
    Command shot =
        holding(
            subject,
            co -> {
              for (int loop = 0; loop < 3; loop++) {
                inTheGap.add(gate.getAsBoolean());
                co.yield();
              }
              co.fork(first(subject));
              asItIsAsked.add(gate.getAsBoolean());
              co.yield();
              watch(co, subject, gate);
            });

    sim.run(shot, 2);

    assertThat(inTheGap).containsExactly(false, false, false);
    assertThat(asItIsAsked).as("in the turn its child asks").containsExactly(false);
    assertThat(trueWhileNotThere).as("loops it said at goal while on its way").isEmpty();
    assertThat(read).as("on its way, it is not at its goal").startsWith(false);
    assertThat(read.getLast()).as("there, it is").isTrue();
    assertThat(isThere(subject)).isTrue();
  }

  /**
   * Asked for a second thing in place of the first, the joint is not at its goal in the turn that
   * changes, even though the second happens to be what it is already doing. The answer the gate had
   * was to the first request. The joint is judged against the second when its own step at the end
   * of the loop takes it, so from the next loop the gate answers for the second, and here says yes.
   */
  @ParameterizedTest
  @EnumSource(Subject.class)
  void askedForASecondThingItIsNotAtItsGoalUntilJudgedAgainstThat(Subject subject) {
    Gate gate = atGoal(subject);
    Command twice =
        holding(
            subject,
            co -> {
              co.fork(first(subject));
              co.waitUntil(gate);
              read.add(gate.getAsBoolean());
              co.fork(sameAgainAskedAnew(subject));
              read.add(gate.getAsBoolean());
              co.yield();
              read.add(gate.getAsBoolean());
              co.yield();
              read.add(gate.getAsBoolean());
            });

    sim.run(twice, 2);

    assertThat(read)
        .as("there for the first; asked the second; and the two loops after")
        .containsExactly(true, false, true, true);
  }

  /**
   * A target a command follows moves a little every loop, and that is one request: a roller that is
   * at its speed stays at its goal while the speed asked creeps up 1 rpm a loop. A target that
   * jumps is a new request, and the joint is not at its goal in the turn it jumps.
   */
  @ParameterizedTest
  @EnumSource(
      value = Subject.class,
      names = {"ROLLER_GATE_MADE_AT_THE_START", "ROLLER_GATE_MADE_RUNNING"})
  void aTargetThatCreepsIsOneRequestAndOneThatJumpsIsANewOne(Subject subject) {
    Gate gate = atGoal(subject);
    double[] rpm = {1500};
    List<Boolean> asItCreeps = new ArrayList<>();
    Command follows =
        holding(
            subject,
            co -> {
              co.fork(roller.follow(() -> RPM.of(rpm[0])));
              co.waitUntil(gate);
              for (int loop = 0; loop < 25; loop++) {
                rpm[0] += 1;
                co.yield();
                asItCreeps.add(gate.getAsBoolean());
              }
              rpm[0] += 500;
              co.yield();
              read.add(gate.getAsBoolean());
            });

    sim.run(follows, 2);

    assertThat(asItCreeps)
        .as("each loop, read after the follower asked")
        .hasSize(25)
        .containsOnly(true);
    assertThat(read).as("in the turn the target jumped 500 rpm").containsExactly(false);
  }

  /**
   * The command ends and the joint goes idle. Its gate then answers for the idle joint, which
   * nobody has asked anything of: the pivot holds where it arrived, and is at its goal the next
   * loop; the roller is asked to stop, and is at its goal once it has.
   */
  @ParameterizedTest
  @EnumSource(Subject.class)
  void whenItsCommandEndsItIsAtItsGoalAsAnIdleJointIs(Subject subject) {
    Gate gate = atGoal(subject);
    Command runs =
        holding(
            subject,
            co -> {
              co.fork(first(subject));
              co.park();
            });
    sim.scheduler.schedule(runs);
    double secondsToGetThere = sim.runUntil(gate, 2);

    sim.scheduler.cancel(runs);
    boolean asItEnds = gate.getAsBoolean();
    sim.loop();
    boolean aLoopLater = gate.getAsBoolean();
    BooleanSupplier stopped = () -> Math.abs(sim.simulation.velocity(joint(subject).joint())) < 1;
    sim.runUntil(stopped, 5);
    sim.run(0.1);

    assertThat(secondsToGetThere).as("it got there").isFinite();
    assertThat(asItEnds)
        .as("as its command ends, it has not been judged as an idle joint")
        .isFalse();
    assertThat(aLoopLater)
        .as("the pivot holds; the roller is still spinning")
        .isEqualTo(subject.isThePivot);
    assertThat(gate.getAsBoolean()).as("idle and stopped").isTrue();
  }

  /**
   * The gate as a shot uses it: one term of "ready", debounced to ride out a dip. The debounce
   * rides out a dip in the answer about one request. It does not carry the answer for the idle
   * joint over to the command that takes it.
   */
  @ParameterizedTest
  @EnumSource(Subject.class)
  void aDebouncedGateOverItDoesNotCarryTheIdleAnswerToTheCommandThatTakesIt(Subject subject) {
    Gate ready =
        made(
            subject,
            joint ->
                Gate.all("ready", joint.isAtGoal(), Gate.of(sim.scheduler, "aimed", () -> true))
                    .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING));
    boolean idle = ready.getAsBoolean();
    Command shot =
        holding(
            subject,
            co -> {
              co.fork(first(subject));
              watch(co, subject, ready);
            });

    sim.run(shot, 2);

    assertThat(idle).as("idle, ready holds").isTrue();
    assertThat(trueWhileNotThere).as("loops ready held while the joint was on its way").isEmpty();
    assertThat(read).startsWith(false);
    assertThat(read.getLast()).isTrue();
  }
}

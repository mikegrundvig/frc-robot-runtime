package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.system.RobotController;

/**
 * What one joint's part of a robot loop costs: the bytes it allocates and the time it takes, for
 * everything a {@link JointMechanism} does between reading its hardware and writing to it. This is
 * the hottest code in the robot, since every joint runs it every 20 ms, so a change to it is
 * measured here before and after.
 *
 * <p>Two joints are measured, each with its interlock and without it: the fixture robot's pivot,
 * sent to a position past a keep-out, and its roller, run at a speed under a run-only-when. The
 * difference between a joint with its interlock and the same joint without is what enforcing
 * interlocks costs. The numbers are printed, and written to {@code build/joint-loop-cost.txt} under
 * the directory the tests run in.
 *
 * <p>A third robot gives the pivot a limit switch short of "out", so that the pivot sent there is
 * held on its switch. A fourth gives it one it rests on, so that the pivot pushed down at a voltage
 * is held neutral on that switch.
 *
 * <p>Only the allocation is asserted: it is exact and repeatable, where the time depends on what
 * else the computer is doing. Every joint is held to a loose bound. A joint under an interlock is
 * also held to a tight one, because each of these scenarios is a steady state, in which the same
 * thing is asked and the interlock holds or does not hold as it did the loop before, and enforcing
 * an interlock through such a loop must make no new objects. A joint held on a limit switch is held
 * as tightly.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class JointLoopCostSimTest {
  /** Loops run before anything is measured, so the JIT has compiled the loop. */
  private static final int WARM_UP_LOOPS = 4000;

  private static final int MEASURED_LOOPS = 4000;

  private static final long LOOP_NANOS = Math.round(RobotLoop.PERIOD_SECONDS * 1e9);

  /** A joint's loop allocated nothing like this much when this test was written. */
  private static final double MOST_BYTES_A_LOOP = 1000;

  /**
   * The most a joint under an interlock may allocate a loop, on average, in a steady state. It
   * measured 0.0 in all four scenarios when this was written. The smallest object is 16 bytes, so
   * this fails as soon as anything at all is made every loop, and lets pass something made only now
   * and then.
   */
  private static final double MOST_BYTES_A_LOOP_UNDER_AN_INTERLOCK = 8;

  /**
   * The most a joint held on its limit switch may allocate a loop, on average: as little as under
   * an interlock. Neither holding the joint on the switch nor the model's check of the switch
   * against where the joint is ({@code SwitchDoubt}) makes anything.
   */
  private static final double MOST_BYTES_A_LOOP_HELD_ON_A_SWITCH =
      MOST_BYTES_A_LOOP_UNDER_AN_INTERLOCK;

  private static final com.sun.management.ThreadMXBean THREADS =
      (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

  private static final RobotDefinition GUARDED =
      RobotDefinitionBuilder.builder(ROBOT)
          .name("guarded")
          .interlocks(
              List.of(
                  Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN)),
                  Interlock.that(PIVOT)
                      .keepsOut(Range.degrees(40, 80))
                      .when(ELEVATOR.atMost(ELEVATOR_CLEAR))))
          .build();

  /** The pivot's switch at 120°, short of "out" at 130°. */
  private static final Sensor.Switch PIVOT_UP =
      JointFixtures.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER, Math.toRadians(120));

  private static final RobotDefinition SWITCHED =
      RobotDefinitionBuilder.builder(ROBOT).name("switched").sensors(List.of(PIVOT_UP)).build();

  /** The pivot's switch at 5°, above where it rests at 1.5°: tripped while the pivot is in. */
  private static final Sensor.Switch PIVOT_DOWN =
      JointFixtures.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Math.toRadians(5));

  private static final RobotDefinition SWITCHED_BELOW =
      RobotDefinitionBuilder.builder(ROBOT)
          .name("switched below")
          .sensors(List.of(PIVOT_DOWN))
          .build();

  /** What a joint is asked each loop while it is measured. */
  private enum Asking {
    /**
     * The pivot is sent to "out". That is past the keep-out, which holds it at the region's edge,
     * and past the limit switch, which holds it where it tripped.
     */
    PIVOT_OUT,
    /** The roller is run at "intaking", which its run-only-when holds while the pivot is in. */
    ROLLER_INTAKING_WITH_PIVOT_IN,
    /**
     * The pivot is pushed down at a voltage, into the switch it rests on, which holds it neutral.
     */
    PIVOT_DOWN_AT_A_VOLTAGE,
    /** Nothing is asked: the joint does what it does idle. */
    NOTHING
  }

  /** What one joint's loop cost, averaged over the measured loops. */
  private record Cost(String what, double mostBytesALoop, double bytesALoop, double nanosALoop) {
    String shown() {
      return String.format(
          Locale.ROOT, "%-58s %8.1f bytes/loop %8.0f ns/loop", what, bytesALoop, nanosALoop);
    }
  }

  @Test
  void aJointsLoopAllocatesLittle() throws IOException {
    List<Cost> costs = new ArrayList<>();
    costs.add(measure("pivot to a position, no interlocks", ROBOT, PIVOT, Asking.PIVOT_OUT));
    costs.add(
        measure("pivot to a position, held by its keep-out", GUARDED, PIVOT, Asking.PIVOT_OUT));
    costs.add(measure("pivot idle, no interlocks", ROBOT, PIVOT, Asking.NOTHING));
    costs.add(measure("pivot idle, under its keep-out", GUARDED, PIVOT, Asking.NOTHING));
    costs.add(
        measure(
            "roller at a speed, no interlocks",
            ROBOT,
            ROLLER,
            Asking.ROLLER_INTAKING_WITH_PIVOT_IN));
    costs.add(
        measure(
            "roller at a speed, held by its run-only-when",
            GUARDED,
            ROLLER,
            Asking.ROLLER_INTAKING_WITH_PIVOT_IN));
    costs.add(measure("roller idle, no interlocks", ROBOT, ROLLER, Asking.NOTHING));
    costs.add(measure("roller idle, under its run-only-when", GUARDED, ROLLER, Asking.NOTHING));
    costs.add(
        measure(
            "pivot to a position, held on its limit switch", SWITCHED, PIVOT, Asking.PIVOT_OUT));
    costs.add(
        measure(
            "pivot at a voltage, held neutral on its limit switch",
            SWITCHED_BELOW,
            PIVOT,
            Asking.PIVOT_DOWN_AT_A_VOLTAGE));

    StringBuilder report = new StringBuilder();
    for (Cost cost : costs) {
      report.append(cost.shown()).append('\n');
    }
    Files.createDirectories(Path.of("build"));
    Files.writeString(Path.of("build", "joint-loop-cost.txt"), report);
    System.out.print(report);

    for (Cost cost : costs) {
      assertThat(cost.bytesALoop()).as(cost.what()).isLessThan(cost.mostBytesALoop());
    }
  }

  /**
   * Runs a robot's joints as the robot does, and measures what one of them costs a loop: its read,
   * what it does with the sensors and the PDH, what it is asked, and its write. The other joints,
   * the simulated physics and the sensors' own reading are run but not measured.
   */
  private static Cost measure(String what, RobotDefinition robot, Joint joint, Asking asking) {
    JointsSimHarness sim = new JointsSimHarness(robot);
    JointMechanism measured = sim.joints.of(joint);
    Sensors sensors = sim.joints.sensors();
    Power power = sim.joints.power();
    long thread = Thread.currentThread().threadId();
    long start = RobotController.getTime();
    long bytes = 0;
    long nanos = 0;
    for (int loop = 0; loop < WARM_UP_LOOPS + MEASURED_LOOPS; loop++) {
      sim.simulation.step(RobotLoop.PERIOD_SECONDS);
      // The joints are handed the time. The simulated clock itself is left alone, because moving it
      // wakes every timer in the test's process, which would take longer than what is measured.
      long now = start + loop * LOOP_NANOS;
      long bytesBefore = THREADS.getThreadAllocatedBytes(thread);
      long nanosBefore = System.nanoTime();
      measured.read(now, true);
      long nanosAfter = System.nanoTime();
      long bytesAfter = THREADS.getThreadAllocatedBytes(thread);
      long loopBytes = bytesAfter - bytesBefore;
      long loopNanos = nanosAfter - nanosBefore;
      for (JointMechanism other : sim.joints.all()) {
        if (other != measured) {
          other.read(now, true);
        }
      }
      sensors.read();
      power.read();
      for (JointMechanism other : sim.joints.all()) {
        if (other != measured) {
          other.sensed();
          other.powered(power);
        }
      }

      bytesBefore = THREADS.getThreadAllocatedBytes(thread);
      nanosBefore = System.nanoTime();
      measured.sensed();
      measured.powered(power);
      ask(measured, asking);
      measured.write();
      nanosAfter = System.nanoTime();
      bytesAfter = THREADS.getThreadAllocatedBytes(thread);
      loopBytes += bytesAfter - bytesBefore;
      loopNanos += nanosAfter - nanosBefore;
      for (JointMechanism other : sim.joints.all()) {
        if (other != measured) {
          other.write();
        }
      }

      if (loop >= WARM_UP_LOOPS) {
        bytes += loopBytes;
        nanos += loopNanos;
      }
    }
    return new Cost(
        what,
        mostBytesALoop(robot),
        (double) bytes / MEASURED_LOOPS,
        (double) nanos / MEASURED_LOOPS);
  }

  /** The most a joint of a robot may allocate a loop, on average. */
  private static double mostBytesALoop(RobotDefinition robot) {
    if (robot == GUARDED) {
      return MOST_BYTES_A_LOOP_UNDER_AN_INTERLOCK;
    }
    if (robot == SWITCHED || robot == SWITCHED_BELOW) {
      return MOST_BYTES_A_LOOP_HELD_ON_A_SWITCH;
    }
    return MOST_BYTES_A_LOOP;
  }

  /** What a move would ask each loop: built once, as a move builds it as it begins. */
  private static final JointAsk TO_PIVOT_OUT =
      new JointAsk.ToPosition(PIVOT_OUT.value(), JointAsk.FULL_CRUISE);

  private static final Optional<String> PIVOT_OUT_NAME = Optional.of(PIVOT_OUT.name());
  private static final JointAsk ROLLER_AT_INTAKING = Asks.running(ROLLER_INTAKING);
  private static final Optional<String> ROLLER_INTAKING_NAME = Optional.of(ROLLER_INTAKING.name());

  private static final JointAsk PIVOT_PUSHED_DOWN = new JointAsk.AtVoltage(-1);

  private static void ask(JointMechanism joint, Asking asking) {
    switch (asking) {
      case PIVOT_OUT -> joint.ask(TO_PIVOT_OUT, PIVOT_OUT_NAME);
      case PIVOT_DOWN_AT_A_VOLTAGE -> joint.ask(PIVOT_PUSHED_DOWN, Optional.empty());
      case ROLLER_INTAKING_WITH_PIVOT_IN -> joint.ask(ROLLER_AT_INTAKING, ROLLER_INTAKING_NAME);
      case NOTHING -> {}
    }
  }
}

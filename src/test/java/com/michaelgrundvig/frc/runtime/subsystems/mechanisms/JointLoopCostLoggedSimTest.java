package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.InItsOwnJvm;
import com.michaelgrundvig.frc.runtime.LoggerOn;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * What one joint's part of a robot loop costs with AdvantageKit's logger running: the scenarios of
 * {@link JointLoopCostSimTest}, which runs with the logger off. Off, {@code Logger.recordOutput}
 * returns at once and the bytes the log costs are not seen there. Running, every call makes a log
 * value and builds the key's full name, about 200 bytes a call, whether or not the value changed.
 * The joint logs a fact (its state, its place, what holds it) only in the loop it changes, because
 * of this cost, and this is the test that shows what is left.
 *
 * <p>Each scenario is a steady state: the same thing is asked loop after loop, and the joint idles,
 * is held by an interlock, or has arrived or runs at its speed. The bound on each is a little above
 * what it measured when this was written, so one more key written every loop fails it. The measure
 * is the joint's read and its write, with the other joints, the sensors and the simulated physics
 * run but not measured.
 *
 * <p>When it was written, a joint allocated about 4,650 bytes a loop in every scenario, of which
 * about 3,400 were its IO's inputs (eighteen values AdvantageKit's {@code processInputs} writes
 * each loop) and about 1,220 the six keys the joint writes each loop whether they changed or not:
 * {@code Command}, {@code GoalName} and {@code Rejected} (192 bytes each), {@code Goal} (200),
 * {@code Setpoint} (216) and {@code SetpointVelocity} (232).
 *
 * <p>It runs in a JVM of its own ({@link InItsOwnJvm}), which also keeps the logger, that starts
 * once and is never stopped, out of every other test's JVM. The numbers are printed and written to
 * {@code build/joint-loop-cost-logged.txt} under the directory the tests run in.
 */
@SimulationTest
@InItsOwnJvm
@ExtendWith(RobotSim.class)
class JointLoopCostLoggedSimTest {
  /** The most the pivot may allocate a loop, on average, in any of its scenarios. */
  private static final double MOST_BYTES_A_LOOP_PIVOT = 4_800;

  /** The most the roller may allocate a loop: it has a longer input than the pivot's. */
  private static final double MOST_BYTES_A_LOOP_ROLLER = 4_850;

  /** One scenario, with the most a joint may allocate a loop in it. */
  private record Row(
      String what,
      RobotDefinition robot,
      Joint joint,
      JointLoopCostSimTest.Asking asking,
      double most) {}

  private static final List<Row> ROWS =
      List.of(
          new Row(
              "pivot idle",
              ROBOT,
              PIVOT,
              JointLoopCostSimTest.Asking.NOTHING,
              MOST_BYTES_A_LOOP_PIVOT),
          new Row(
              "roller idle",
              ROBOT,
              ROLLER,
              JointLoopCostSimTest.Asking.NOTHING,
              MOST_BYTES_A_LOOP_ROLLER),
          new Row(
              "pivot arrived at a position",
              ROBOT,
              PIVOT,
              JointLoopCostSimTest.Asking.PIVOT_OUT,
              MOST_BYTES_A_LOOP_PIVOT),
          new Row(
              "roller at a speed",
              ROBOT,
              ROLLER,
              JointLoopCostSimTest.Asking.ROLLER_INTAKING_WITH_PIVOT_IN,
              MOST_BYTES_A_LOOP_ROLLER),
          new Row(
              "pivot held by its keep-out",
              JointLoopCostSimTest.GUARDED,
              PIVOT,
              JointLoopCostSimTest.Asking.PIVOT_OUT,
              MOST_BYTES_A_LOOP_PIVOT),
          new Row(
              "roller held by its run-only-when",
              JointLoopCostSimTest.GUARDED,
              ROLLER,
              JointLoopCostSimTest.Asking.ROLLER_INTAKING_WITH_PIVOT_IN,
              MOST_BYTES_A_LOOP_ROLLER));

  @Test
  void aJointsLoopWithTheLoggerRunningAllocatesNoMoreThanItDid() throws IOException {
    LoggerOn.start();
    List<JointLoopCostSimTest.Cost> costs = new ArrayList<>();
    for (Row row : ROWS) {
      costs.add(JointLoopCostSimTest.measure(row.what(), row.robot(), row.joint(), row.asking()));
    }

    StringBuilder report = new StringBuilder();
    for (JointLoopCostSimTest.Cost cost : costs) {
      report.append(cost.shown()).append('\n');
    }
    Files.createDirectories(Path.of("build"));
    Files.writeString(Path.of("build", "joint-loop-cost-logged.txt"), report);
    System.out.print(report);

    for (int i = 0; i < ROWS.size(); i++) {
      assertThat(costs.get(i).bytesALoop()).as(ROWS.get(i).what()).isLessThan(ROWS.get(i).most());
    }
  }
}

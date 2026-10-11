package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.DriveSimHarness;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.RobotController;

/**
 * What the drivetrain's part of a robot loop costs: the bytes it allocates and the time it takes,
 * for its read before the commands run ({@link Drive#periodic}) and for the command that drives it.
 * The drive runs every 20 ms, so a change to it is measured here before and after.
 *
 * <p>Three steady situations are measured: driven by the sticks, facing a place while the sticks
 * translate, and following a move's path. The simulated physics and localization are run but not
 * measured. The numbers are printed, and written to {@code build/drive-loop-cost.txt} under the
 * directory the tests run in.
 *
 * <p>Each situation is measured in several batches of loops, one after another, and the cheapest
 * batch is the one reported. What a loop allocates settles as the JIT compiles more of it, and how
 * soon that happens differs from run to run by a few hundred bytes a loop, so one batch alone is
 * not repeatable.
 *
 * <p>Only the allocation is asserted. Each situation is held to a bound a little above what it
 * measured when the bound was set, so that a change that makes the drive allocate more each loop
 * fails here and is looked at. The time is not asserted, because it depends on what else the
 * computer is doing.
 *
 * <p>Most of what is still allocated is not the drive's own to save. In the order of size when the
 * bounds were set: the simulator's module and gyro IO, which copy their high-rate readings each
 * loop (about 1,000 bytes); WPILib's kinematics, which turn a velocity into four wheel velocities
 * and back (about 1,200); the command scheduler resuming the command (about 700); a sample of the
 * wheels for each high-rate reading, which localization keeps (640); and the wheels' poses for the
 * log while the wheels turn (512).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class DriveLoopCostSimTest {
  /** Loops run before anything is measured, so the JIT has compiled the loop. */
  private static final int WARM_UP_LOOPS = 8000;

  /** How many batches of loops are measured, of which the cheapest is reported. */
  private static final int BATCHES = 5;

  private static final int LOOPS_A_BATCH = 2000;

  private static final com.sun.management.ThreadMXBean THREADS =
      (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

  /** The two ends a followed move drives between, far enough apart to cruise between them. */
  private static final Pose2d NEAR_END = new Pose2d(2, 2, Rotation2d.ZERO);

  private static final Pose2d FAR_END = new Pose2d(5, 3, Rotation2d.CCW_90DEG);

  /** What drives the drivetrain while it is measured. */
  private enum Driving {
    /** The driver's sticks, translating and turning. It measured 4,540 bytes a loop. */
    STICKS(5_200),
    /** The sticks translating, the drivetrain facing the hub. It measured 5,380 bytes a loop. */
    FACING_A_PLACE(6_200),
    /**
     * One move after another between two poses. Only loops in which a move follows count. It
     * measured 4,980 bytes a loop.
     */
    FOLLOWING_A_MOVE(5_700);

    /** The most the drive's loop may allocate in this situation, in bytes, on average. */
    final double mostBytesALoop;

    Driving(double mostBytesALoop) {
      this.mostBytesALoop = mostBytesALoop;
    }
  }

  /** What the drive's loop cost, averaged over the loops of the cheapest batch. */
  private record Cost(String what, double mostBytesALoop, double bytesALoop, double nanosALoop) {
    String shown() {
      return String.format(
          Locale.ROOT, "%-44s %8.1f bytes/loop %8.0f ns/loop", what, bytesALoop, nanosALoop);
    }
  }

  @Test
  void theDrivesLoopAllocatesNoMoreThanItDid() throws IOException {
    List<Cost> costs = new ArrayList<>();
    costs.add(measure("driven by the sticks", Driving.STICKS));
    costs.add(measure("facing a place while translating", Driving.FACING_A_PLACE));
    costs.add(measure("following a move", Driving.FOLLOWING_A_MOVE));

    StringBuilder report = new StringBuilder();
    for (Cost cost : costs) {
      report.append(cost.shown()).append('\n');
    }
    Files.createDirectories(Path.of("build"));
    Files.writeString(Path.of("build", "drive-loop-cost.txt"), report);
    System.out.print(report);

    for (Cost cost : costs) {
      assertThat(cost.bytesALoop()).as(cost.what()).isLessThan(cost.mostBytesALoop());
    }
  }

  /**
   * Runs the drivetrain as the robot does, and measures what its own part of each loop costs: its
   * read, and the scheduler's run of the one command that drives it.
   */
  private static Cost measure(String what, Driving driving) {
    try (DriveSimHarness harness = new DriveSimHarness(ExampleRobot.SIM)) {
      Drive drive = harness.drive;
      harness.simulation.place(NEAR_END);
      harness.loop();
      harness.localization.resetPose(NEAR_END);
      harness.loop();
      RobotSim.setEnabled(true);
      switch (driving) {
        case STICKS -> {
          harness.sticks(0.5, 0.3, 0.4);
          harness.scheduler.schedule(drive.driven());
        }
        case FACING_A_PLACE -> {
          harness.sticks(0.3, -0.2, 0);
          harness.scheduler.schedule(drive.facing(Heading.toward(Rebuilt.HUB)));
        }
        case FOLLOWING_A_MOVE -> {}
      }

      long thread = Thread.currentThread().threadId();
      DriveMove move = drive.goTo(FAR_END);
      boolean towardFarEnd = true;
      long bytes = 0;
      long nanos = 0;
      int measured = 0;
      int loop = 0;
      double leastBytesALoop = Double.POSITIVE_INFINITY;
      double leastNanosALoop = Double.POSITIVE_INFINITY;
      int batches = 0;
      while (batches < BATCHES) {
        boolean starting = false;
        if (driving == Driving.FOLLOWING_A_MOVE && !harness.scheduler.isRunning(move)) {
          move = drive.goTo(towardFarEnd ? FAR_END : NEAR_END);
          towardFarEnd = !towardFarEnd;
          harness.scheduler.schedule(move);
          starting = true;
        }
        RobotSim.stepClock();
        harness.simulation.step(RobotLoop.PERIOD_SECONDS, RobotController.getTime());
        boolean enabled = RobotState.isEnabled();

        long bytesBefore = THREADS.getThreadAllocatedBytes(thread);
        long nanosBefore = System.nanoTime();
        drive.periodic(enabled);
        long nanosAfter = System.nanoTime();
        long bytesAfter = THREADS.getThreadAllocatedBytes(thread);
        long loopBytes = bytesAfter - bytesBefore;
        long loopNanos = nanosAfter - nanosBefore;

        harness.localization.periodic(drive.readings(), enabled);
        harness.localization.update();

        bytesBefore = THREADS.getThreadAllocatedBytes(thread);
        nanosBefore = System.nanoTime();
        harness.scheduler.run();
        nanosAfter = System.nanoTime();
        bytesAfter = THREADS.getThreadAllocatedBytes(thread);
        loopBytes += bytesAfter - bytesBefore;
        loopNanos += nanosAfter - nanosBefore;

        // A move plans its path in its first loop and says how it ended in its last: neither is the
        // steady following measured here.
        boolean ending = driving == Driving.FOLLOWING_A_MOVE && move.outcome().isPresent();
        boolean steady = !starting && !ending;
        if (loop >= WARM_UP_LOOPS && steady) {
          bytes += loopBytes;
          nanos += loopNanos;
          measured++;
        }
        loop++;
        if (measured == LOOPS_A_BATCH) {
          leastBytesALoop = Math.min(leastBytesALoop, (double) bytes / LOOPS_A_BATCH);
          leastNanosALoop = Math.min(leastNanosALoop, (double) nanos / LOOPS_A_BATCH);
          bytes = 0;
          nanos = 0;
          measured = 0;
          batches++;
        }
      }
      return new Cost(what, driving.mostBytesALoop, leastBytesALoop, leastNanosALoop);
    }
  }
}

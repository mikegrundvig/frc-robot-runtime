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
 * <p>Only the allocation is asserted, and loosely: it is exact and repeatable, where the time
 * depends on what else the computer is doing.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class DriveLoopCostSimTest {
  /** Loops run before anything is measured, so the JIT has compiled the loop. */
  private static final int WARM_UP_LOOPS = 4000;

  private static final int MEASURED_LOOPS = 4000;

  /** A drive's loop allocated nothing like this much when this test was written. */
  private static final double MOST_BYTES_A_LOOP = 12_000;

  private static final com.sun.management.ThreadMXBean THREADS =
      (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

  /** The two ends a followed move drives between, far enough apart to cruise between them. */
  private static final Pose2d NEAR_END = new Pose2d(2, 2, Rotation2d.ZERO);

  private static final Pose2d FAR_END = new Pose2d(5, 3, Rotation2d.CCW_90DEG);

  /** What drives the drivetrain while it is measured. */
  private enum Driving {
    /** The driver's sticks, translating and turning. */
    STICKS,
    /** The sticks translating, the drivetrain facing the hub. */
    FACING_A_PLACE,
    /** One move after another between two poses. Only loops in which a move follows count. */
    FOLLOWING_A_MOVE
  }

  /** What the drive's loop cost, averaged over the measured loops. */
  private record Cost(String what, double bytesALoop, double nanosALoop) {
    String shown() {
      return String.format(
          Locale.ROOT, "%-44s %8.1f bytes/loop %8.0f ns/loop", what, bytesALoop, nanosALoop);
    }
  }

  @Test
  void theDrivesLoopAllocatesLittle() throws IOException {
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
      assertThat(cost.bytesALoop()).as(cost.what()).isLessThan(MOST_BYTES_A_LOOP);
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
      while (measured < MEASURED_LOOPS) {
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
      }
      return new Cost(what, (double) bytes / MEASURED_LOOPS, (double) nanos / MEASURED_LOOPS);
    }
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.InItsOwnJvm;
import com.michaelgrundvig.frc.runtime.LoggerOn;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * What the drivetrain's part of a robot loop costs with AdvantageKit's logger running, driven by
 * the sticks: the situation of {@link DriveLoopCostSimTest}, which runs with the logger off. Off,
 * {@code Logger.recordOutput} returns at once and the bytes the log costs are not seen there.
 * Running, every call makes a log value and builds the key's full name, about 200 to 400 bytes a
 * call, and the drive writes the most keys of anything in the robot.
 *
 * <p>When it was written, the drive allocated about 33,500 bytes a loop, against about 4,500 with
 * the logger off. Nearly all of the 29,000 in its read are its IO's inputs, which AdvantageKit's
 * {@code processInputs} writes whole each loop: seventeen values for each of four modules, nine for
 * each of two gyros. The 3,500 in the command's run are the keys the drive writes each loop, its
 * pose and the two arrays of wheel setpoints.
 *
 * <p>The bound is a little above what it measured, so that one more key written every loop fails
 * it. It runs in a JVM of its own ({@link InItsOwnJvm}), which also keeps the logger, that starts
 * once and is never stopped, out of every other test's JVM. The number is printed and written to
 * {@code build/drive-loop-cost-logged.txt} under the directory the tests run in.
 */
@SimulationTest
@InItsOwnJvm
@ExtendWith(RobotSim.class)
class DriveLoopCostLoggedSimTest {
  /** The most the drive's loop may allocate driven by the sticks with the logger running. */
  private static final double MOST_BYTES_A_LOOP_ON_THE_STICKS = 34_000;

  @Test
  void theDrivesLoopWithTheLoggerRunningAllocatesNoMoreThanItDid() throws IOException {
    LoggerOn.start();

    DriveLoopCostSimTest.Cost cost =
        DriveLoopCostSimTest.measure("driven by the sticks", DriveLoopCostSimTest.Driving.STICKS);

    Files.createDirectories(Path.of("build"));
    Files.writeString(Path.of("build", "drive-loop-cost-logged.txt"), cost.shown() + "\n");
    System.out.println(cost.shown());
    assertThat(cost.bytesALoop()).isLessThan(MOST_BYTES_A_LOOP_ON_THE_STICKS);
  }
}

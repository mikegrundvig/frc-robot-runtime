package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.Imus;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The truth suite's determinism: a simulated world is a function of its configuration, its seed,
 * and what's done to it, so the same run comes out the same to the bit alone, repeated, beside
 * seven others on threads, and in another JVM. Its physics and its IMUs, with a season-worn robot's
 * errors, through launches, turns, a push, and a tilt.
 */
class DeterminismTruthTest {
  private static final double STEP = 0.004;

  @Test
  void aRunIsTheSameToTheBitAloneRepeatedBesideOthersAndInAnotherJvm() throws Exception {
    String alone = fingerprint(3);

    assertThat(fingerprint(3)).as("repeated").isEqualTo(alone);
    assertThat(fingerprint(4)).as("another seed").isNotEqualTo(alone);
    ExecutorService threads = Executors.newFixedThreadPool(8);
    try {
      List<Future<String>> runs = new ArrayList<>();
      for (int i = 0; i < 8; i++) {
        runs.add(threads.submit(() -> fingerprint(3)));
      }
      for (Future<String> run : runs) {
        assertThat(run.get()).as("beside seven others").isEqualTo(alone);
      }
    } finally {
      threads.shutdownNow();
    }
    assertThat(inAnotherJvm(3)).as("in another JVM").isEqualTo(alone);
  }

  /** The run in a JVM of its own, started as this one was: its fingerprint, as it prints it. */
  private static String inAnotherJvm(long seed) throws IOException, InterruptedException {
    Process child =
        new ProcessBuilder(
                ProcessHandle.current().info().command().orElseThrow(),
                "-Djava.library.path=" + System.getProperty("java.library.path"),
                "-cp",
                System.getProperty("java.class.path"),
                DeterminismTruthTest.class.getName(),
                Long.toString(seed))
            .redirectErrorStream(true)
            .start();
    String out = new String(child.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
    assertThat(child.waitFor()).as(out).isZero();
    return out.lines()
        .filter(line -> line.startsWith("fingerprint "))
        .findFirst()
        .orElseThrow()
        .substring("fingerprint ".length());
  }

  /** Prints the run's fingerprint, for {@link #inAnotherJvm}. */
  public static void main(String[] args) throws NoSuchAlgorithmException {
    System.out.println("fingerprint " + fingerprint(Long.parseLong(args[0])));
  }

  /** A season-worn robot's run, every bit of its truth and its IMUs' readings, digested. */
  static String fingerprint(long seed) throws NoSuchAlgorithmException {
    SensorErrors errors = SensorErrorsBuilder.builder(SensorErrors.TYPICAL).seed(seed).build();
    KinematicPhysics physics = TestRobot.physics();
    TestRobot.Controllers controllers = new TestRobot.Controllers();
    ChassisTruth chassis = physics.chassis();
    GyroModel gyro =
        new GyroModel(
            chassis,
            errors.gyro(),
            new AccelerometerModel(
                chassis,
                errors.accelerometer(),
                Imus.UNCHECKED.gyro(),
                Seeds.gyroAccelerometer(seed)),
            Seeds.gyro(seed));
    MessageDigest digest = MessageDigest.getInstance("SHA-256");
    for (int step = 0; step < 1500; step++) {
      if (step % 250 == 0) {
        controllers.steer(step / 250 * 0.7);
        controllers.volts = step % 500 == 0 ? 12 : -7;
      }
      switch (step) {
        case 400 -> physics.apply(new Disturbance.Pushed(new Translation2d(0.5, -1)));
        case 600 -> physics.apply(new Disturbance.Pushed(Translation2d.ZERO));
        case 800 -> physics.apply(new Disturbance.TiltedByHand(0.05, -0.1));
        case 1000 -> {
          gyro.jolt(new Translation2d(20, 3));
          physics.apply(
              new Disturbance.Placed(
                  chassis.pose().plus(new Transform2d(0.1, 0.2, Rotation2d.fromDegrees(15)))));
        }
        default -> {}
      }
      physics.step(STEP, controllers);
      gyro.feel(STEP);
      gyro.sample();
      Pose3d pose = chassis.pose3d();
      digest(digest, pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().getZ());
      digest(digest, gyro.yawRadians(), physics.wheel(step % 4).spinRadPerSec());
      if (step % 5 == 4) {
        gyro.endLoop(5 * STEP);
        Translation2d felt = gyro.acceleration();
        if (felt != null) {
          digest(digest, felt.getX(), felt.getY());
        }
      }
    }
    return HexFormat.of().formatHex(digest.digest());
  }

  private static void digest(MessageDigest digest, double... values) {
    for (double value : values) {
      long bits = Double.doubleToRawLongBits(value);
      for (int i = 0; i < 8; i++) {
        digest.update((byte) (bits >>> (8 * i)));
      }
    }
  }
}

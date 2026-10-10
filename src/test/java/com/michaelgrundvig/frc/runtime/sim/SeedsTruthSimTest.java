package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.SensorErrorsBuilder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The truth suite's seeds: a simulated run is a function of its seed, so its random streams must be
 * its own, apart from one another and from every other seed's, and the run must come out the same
 * to the bit however it's run.
 */
@ExtendWith(RobotSim.class)
class SeedsTruthSimTest {
  /** The seeds a study runs: the consistency study's twelve, and a scenario lab's hundred. */
  private static final int SEEDS = 100;

  /** The template's robot's cameras. */
  private static final int CAMERAS = ExampleRobot.SIM.definition().cameras().size();

  /**
   * Every random stream, over a study's seeds, starts from a seed of its own: no two draw the same
   * numbers. Each is the world's seed mixed with its kind, then its index ({@link Seeds#stream});
   * as offsets from the world's seed, 202 overlapped, a seed's gyro drawing what the seed before it
   * gave its gyro's accelerometer, and seed 1's gyro what its first camera drew.
   */
  @Test
  void everyStreamOfEverySeedIsItsOwn() {
    Map<Long, String> streams = new LinkedHashMap<>();
    List<String> shared = new ArrayList<>();
    for (long seed = 1; seed <= SEEDS; seed++) {
      Map<String, Long> world = streams(seed);
      for (Map.Entry<String, Long> stream : world.entrySet()) {
        String name = "seed " + seed + "'s " + stream.getKey();
        String before = streams.putIfAbsent(stream.getValue(), name);
        if (before != null) {
          shared.add(name + " = " + before);
        }
      }
    }

    assertThat(shared).isEmpty();
  }

  /**
   * Streams from seeds of their own draw uncorrelated numbers: the first 10,000 normal draws of
   * every pair of a world's streams (neighbors included, a seed and the next), correlated within
   * three standard errors, 3 / √n. Every stream counts by its name, so two that share a seed are
   * compared too, and caught: they draw the same numbers, as seed 1's first two cameras once drew
   * what its gyro and its gyro's accelerometer did.
   */
  @Test
  void streamsFromTheirOwnSeedsAreUncorrelated() {
    int n = 10_000;
    List<String> names = new ArrayList<>();
    List<double[]> draws = new ArrayList<>();
    for (Map.Entry<String, Long> stream : streams(1).entrySet()) {
      Random random = new Random(stream.getValue());
      double[] values = new double[n];
      for (int i = 0; i < n; i++) {
        values[i] = random.nextGaussian();
      }
      names.add(stream.getKey());
      draws.add(values);
    }
    List<String> correlated = new ArrayList<>();
    for (int a = 0; a < draws.size(); a++) {
      for (int b = a + 1; b < draws.size(); b++) {
        double r = correlation(draws.get(a), draws.get(b));
        if (!(Math.abs(r) < 3 / Math.sqrt(n))) {
          correlated.add(
              names.get(a) + " and " + names.get(b) + String.format(Locale.ROOT, ", %.3f", r));
        }
      }
    }

    assertThat(correlated).isEmpty();
  }

  /**
   * Streams start apart too: over a hundred worlds, each stream's first draw against every other
   * stream's of the same world, and against its own of the next world, correlated within three
   * standard errors, 3 / √n. Java's {@code Random} seeded with numbers a step apart starts from
   * nearly the same state, so its first draws move together: seeded as offsets from the world's
   * seed, a world's four cameras drew the same first number (correlated 1.0 over a hundred worlds),
   * its IMUs' streams nearly so (0.96 and more), and each IMU's stream the next world's.
   */
  @Test
  void streamsStartApartWithinAWorldAndFromTheNext() {
    Map<String, double[]> first = new LinkedHashMap<>();
    for (long seed = 1; seed <= SEEDS + 1; seed++) {
      for (Map.Entry<String, Long> stream : streams(seed).entrySet()) {
        first.computeIfAbsent(stream.getKey(), name -> new double[SEEDS + 1])[(int) seed - 1] =
            new Random(stream.getValue()).nextDouble();
      }
    }
    List<String> names = List.copyOf(first.keySet());
    List<double[]> draws = List.copyOf(first.values());
    List<String> together = new ArrayList<>();
    for (int a = 0; a < names.size(); a++) {
      for (int b = a + 1; b < names.size(); b++) {
        check(together, names.get(a) + " and " + names.get(b), draws.get(a), draws.get(b), 0);
      }
      check(together, names.get(a) + " and the next world's", draws.get(a), draws.get(a), 1);
    }

    assertThat(together).isEmpty();
  }

  /** Notes these first draws if they move together, a hundred worlds of one against the other's. */
  private static void check(
      List<String> together, String what, double[] one, double[] other, int worldsOn) {
    double r =
        correlation(
            Arrays.copyOfRange(one, 0, SEEDS),
            Arrays.copyOfRange(other, worldsOn, SEEDS + worldsOn));
    if (!(Math.abs(r) < 3 / Math.sqrt(SEEDS))) {
      together.add(what + String.format(Locale.ROOT, ", %.2f", r));
    }
  }

  /**
   * The same seed gives the same run, to the bit: repeated, and run on eight threads at once, each
   * its own world.
   */
  @Test
  void aRunIsTheSameToTheBitAloneRepeatedAndBesideOthers() throws Exception {
    List<Long> alone = fingerprint(3);
    assertThat(fingerprint(3)).as("repeated").isEqualTo(alone);
    assertThat(fingerprint(4)).as("another seed").isNotEqualTo(alone);

    ExecutorService threads = Executors.newFixedThreadPool(8);
    try {
      List<Future<List<Long>>> runs = new ArrayList<>();
      for (int i = 0; i < 8; i++) {
        runs.add(threads.submit(() -> fingerprint(3)));
      }
      for (Future<List<Long>> run : runs) {
        assertThat(run.get()).as("beside seven others").isEqualTo(alone);
      }
    } finally {
      threads.shutdownNow();
    }
  }

  /** A world's random streams by name, from its seed. */
  private static Map<String, Long> streams(long seed) {
    Map<String, Long> streams = new LinkedHashMap<>();
    streams.put("gyro", Seeds.gyro(seed));
    streams.put("gyro's accelerometer", Seeds.gyroAccelerometer(seed));
    streams.put("gyro's accelerometer's up axis", Seeds.upAxis(Seeds.gyroAccelerometer(seed)));
    streams.put("backup IMU", Seeds.backupGyro(seed));
    streams.put("backup IMU's accelerometer", Seeds.backupAccelerometer(seed));
    streams.put(
        "backup IMU's accelerometer's up axis", Seeds.upAxis(Seeds.backupAccelerometer(seed)));
    for (int camera = 0; camera < CAMERAS; camera++) {
      String[] order = {"first", "second", "third", "fourth", "fifth", "sixth"};
      streams.put(
          (camera < order.length ? order[camera] : "#" + (camera + 1)) + " camera",
          Seeds.camera(seed, camera));
    }
    return streams;
  }

  /**
   * A season-worn robot driven through launches, turns, and a hit for 3 s, its truth and every
   * sensor reading taken down to the bit.
   */
  private static List<Long> fingerprint(long seed) {
    SensorErrors errors = SensorErrorsBuilder.builder(SensorErrors.TYPICAL).seed(seed).build();
    TruthRig rig =
        new TruthRig(
            ExampleRobot.SIM.definition().body(),
            ExampleRobot.SIM.definition().drivetrain().wheelCoefficientOfFriction(),
            errors);
    List<Long> bits = new ArrayList<>();
    for (int loop = 0; loop < 150; loop++) {
      if (loop % 50 == 0) {
        rig.steer(Rotation2d.fromDegrees(40 * loop / 50.0));
        rig.driveVolts(loop == 100 ? -8 : 10);
      }
      if (loop == 75) {
        rig.simulation.hit(new org.wpilib.math.geometry.Transform2d(0.1, 0.2, Rotation2d.ZERO));
      }
      rig.loops(1);
      Pose3d truth = rig.simulation.truePose3d();
      add(bits, truth.getX(), truth.getY(), truth.getZ(), truth.getRotation().getZ());
      GyroIO.GyroIOInputs gyro = new GyroIO.GyroIOInputs();
      rig.simulation.gyro().updateInputs(gyro);
      add(
          bits,
          gyro.yaw.getRadians(),
          gyro.accelerationXMetersPerSecSq,
          gyro.accelerationYMetersPerSecSq);
      add(bits, gyro.odometryYawsRad);
      for (ModuleIO module : rig.modules) {
        ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
        module.updateInputs(inputs);
        add(
            bits,
            inputs.drivePositionRad,
            inputs.driveStatorCurrentAmps,
            inputs.steerAngle.getRadians());
        add(bits, inputs.odometryDrivePositionsRad);
      }
    }
    return bits;
  }

  private static void add(List<Long> bits, double... values) {
    for (double value : values) {
      bits.add(Double.doubleToRawLongBits(value));
    }
  }

  private static double correlation(double[] a, double[] b) {
    double meanA = 0;
    double meanB = 0;
    for (int i = 0; i < a.length; i++) {
      meanA += a[i] / a.length;
      meanB += b[i] / b.length;
    }
    double ab = 0;
    double aa = 0;
    double bb = 0;
    for (int i = 0; i < a.length; i++) {
      ab += (a[i] - meanA) * (b[i] - meanB);
      aa += (a[i] - meanA) * (a[i] - meanA);
      bb += (b[i] - meanB) * (b[i] - meanB);
    }
    return ab / Math.sqrt(aa * bb);
  }
}

package com.michaelgrundvig.frc.runtime.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.localization.BodyMotion;
import com.michaelgrundvig.frc.localization.OdometrySample;
import com.michaelgrundvig.frc.localization.SwerveKinematicsModel;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.subsystems.drive.GyroIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.simulation.Consistency;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * The truth suite's odometry and sensor rates. With exact sensors on a flat floor, well inside the
 * tread's grip, the template's kinematics run on the simulated encoders' readings land where the
 * robot truly went; and the sensors report at the rates their configuration gives.
 *
 * <p>Today the simulator moves the robot by the same least-squares fit of the wheels that odometry
 * uses, so odometry matching the truth is no surprise here; it is the check that a physics engine
 * whose wheels roll on a compliant tread (a rolling radius a little under the tread's) must pass.
 */
@ExtendWith(RobotSim.class)
class OdometryTruthSimTest {
  /**
   * Straight, curving, and spinning in place, each held steady after its wheels point their way
   * (under 30% of the grip): odometry from the encoders' 250 Hz readings is within 0.1% of the
   * distance the robot truly went, and its heading within 0.05° a turn. Consistency today, truth
   * for an engine whose tread rolls on a compliant contact.
   */
  @Test
  @Consistency("today's physics moves the robot by the same fit of its wheels that odometry makes")
  void withExactSensorsOdometryFollowsTheTruth() {
    // Robot-relative: forward, left (m/s), counterclockwise (rad/s).
    double[][] maneuvers = {{1.0, 0.6, 0}, {1.2, 0, 0.8}, {0, 0, 1.5}, {0.5, -0.8, -0.6}};
    for (double[] maneuver : maneuvers) {
      TruthRig rig = new TruthRig();
      command(rig, maneuver[0], maneuver[1], maneuver[2], false);
      rig.run(0.4); // the modules turn to their angles
      command(rig, maneuver[0], maneuver[1], maneuver[2], true);
      rig.run(0.4); // and come up to speed
      OdometrySample[] before = samples(rig);
      OdometrySample last = before[before.length - 1];
      Pose2d odometry = rig.simulation.truePose(); // where that last reading was taken
      double traveled = 0;
      double turned = 0;
      SwerveKinematicsModel kinematics = new SwerveKinematicsModel(rig.wheels);
      for (int loop = 0; loop < 100; loop++) {
        Pose2d from = rig.simulation.truePose();
        rig.loops(1);
        Pose2d to = rig.simulation.truePose();
        traveled += to.getTranslation().getDistance(from.getTranslation());
        turned += Math.abs(to.getRotation().minus(from.getRotation()).getRadians());
        for (OdometrySample sample : samples(rig)) {
          BodyMotion motion = kinematics.motion(last, sample);
          odometry =
              odometry.plus(
                  new Twist2d(motion.dxMeters(), motion.dyMeters(), motion.dthetaRadians()).exp());
          last = sample;
        }
      }

      Pose2d truth = rig.simulation.truePose();
      String what = java.util.Arrays.toString(maneuver);
      assertThat(odometry.getTranslation().getDistance(truth.getTranslation()))
          .as("position, %s", what)
          .isLessThanOrEqualTo(Math.max(1e-3 * traveled, 1e-9));
      assertThat(Math.abs(odometry.getRotation().minus(truth.getRotation()).getDegrees()))
          .as("heading, %s", what)
          .isLessThanOrEqualTo(Math.max(0.05 * turned / (2 * Math.PI), 1e-9));
    }
  }

  /**
   * Each module's CANcoder sends its angle every 10 ms (its configured 100 Hz), so a module turning
   * steadily reads a new angle 100 times a second, within 1%, and never in two 4 ms readings
   * running; the gyro, and each drive encoder, read 250 times a second.
   */
  @Test
  void theSensorsReadAtTheirConfiguredRates() {
    TruthRig rig = new TruthRig();
    turn(rig, 0, 10); // the modules get turning
    drain(rig);
    rig.simulation.gyro().updateInputs(new GyroIO.GyroIOInputs());
    int[] changes = new int[4];
    boolean[] lastChanged = new boolean[4];
    double[] lastAngle = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
    int gyroReadings = 0;
    int driveReadings = 0;
    for (int loop = 10; loop < 60; loop++) {
      turn(rig, loop, loop + 1);
      for (int m = 0; m < 4; m++) {
        ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
        rig.modules[m].updateInputs(inputs);
        driveReadings += m == 0 ? inputs.odometryDrivePositionsRad.length : 0;
        for (double angle : inputs.odometrySteerAnglesRad) {
          boolean changed = !Double.isNaN(lastAngle[m]) && angle != lastAngle[m];
          if (changed) {
            assertThat(lastChanged[m]).as("module %d changed in two readings running", m).isFalse();
            changes[m]++;
          }
          lastChanged[m] = changed;
          lastAngle[m] = angle;
        }
      }
      GyroIO.GyroIOInputs gyro = new GyroIO.GyroIOInputs();
      rig.simulation.gyro().updateInputs(gyro);
      gyroReadings += gyro.odometryYawsRad.length;
    }

    for (int count : changes) {
      assertThat(count).as("new angles in a second").isCloseTo(100, within(1));
    }
    assertThat(gyroReadings).as("gyro readings in a second").isEqualTo(250);
    assertThat(driveReadings).as("drive encoder readings in a second").isEqualTo(250);
  }

  /** Every module turning steadily, 10° a loop (500°/s), over these loops. */
  private static void turn(TruthRig rig, int from, int to) {
    for (int loop = from; loop < to; loop++) {
      for (ModuleIO module : rig.modules) {
        module.setSteerAngle(Rotation2d.fromDegrees(10.0 * loop));
      }
      rig.loops(1);
    }
  }

  /**
   * Each module steered and, if {@code driving}, driven for this robot-relative motion: its
   * velocity is the robot's plus the turn swinging it around the center.
   */
  private static void command(
      TruthRig rig, double forward, double left, double turn, boolean driving) {
    double kv = ExampleRobot.SIM.drivetrain().gains().driveKv();
    for (int i = 0; i < 4; i++) {
      Translation2d wheel = rig.wheels[i];
      Translation2d velocity =
          new Translation2d(forward - turn * wheel.getY(), left + turn * wheel.getX());
      rig.modules[i].setSteerAngle(velocity.getAngle().orElseThrow());
      double speed = driving ? velocity.getNorm() : 0;
      rig.modules[i].setDriveVelocity(speed / rig.wheelRadius(), kv * speed);
    }
  }

  /** Reads and forgets every module's readings so far. */
  private static void drain(TruthRig rig) {
    samples(rig);
  }

  /**
   * The modules' readings since the last read, each instant's four together, at the configured
   * radius.
   */
  private static OdometrySample[] samples(TruthRig rig) {
    ModuleIO.ModuleIOInputs[] inputs = new ModuleIO.ModuleIOInputs[4];
    for (int m = 0; m < 4; m++) {
      inputs[m] = new ModuleIO.ModuleIOInputs();
      rig.modules[m].updateInputs(inputs[m]);
    }
    int count = inputs[0].odometryTimestampsNanos.length;
    OdometrySample[] samples = new OdometrySample[count];
    for (int k = 0; k < count; k++) {
      double[] distances = new double[4];
      double[] angles = new double[4];
      for (int m = 0; m < 4; m++) {
        distances[m] = inputs[m].odometryDrivePositionsRad[k] * rig.wheelRadius();
        angles[m] = inputs[m].odometrySteerAnglesRad[k];
      }
      samples[k] = new OdometrySample(inputs[0].odometryTimestampsNanos[k], distances, angles);
    }
    return samples;
  }
}

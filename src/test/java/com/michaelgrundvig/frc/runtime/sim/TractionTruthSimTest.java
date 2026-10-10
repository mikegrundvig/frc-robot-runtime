package com.michaelgrundvig.frc.runtime.sim;

import static com.michaelgrundvig.frc.simulation.KnownGap.Check.stillOpen;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.simulation.KnownGap;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.WheelTruth;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The truth suite's traction: where the tread runs out of grip, against Coulomb friction (μ times
 * the load on each wheel) and the loads that quasi-static weight transfer gives. Today's simulator
 * caps the robot as a whole at μ g, every wheel alike, so the limit is right in every direction but
 * no wheel slips before the others.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class TractionTruthSimTest {
  /** The template's tread: 1.2. */
  private static final double MU =
      ExampleRobot.SIM.definition().drivetrain().wheelCoefficientOfFriction();

  /**
   * A slick tread, μ 0.5, on which every wheel slips even as loaded as weight transfer makes any of
   * them here: the most loaded, the trailing one at 45° (m g / 4 + √2 m a h / L, 291 N at a = μ g),
   * holds 146 N, where its motor pushes 309 N at 120 A.
   */
  private static final double SLICK = 0.5;

  /**
   * Pushed past its grip (120 A on a slick tread, μ 0.5) in each of 16 directions, the robot speeds
   * up at μ g, whatever the direction: within 3% of μ g, and every direction within 5% of the
   * others. With every wheel slipping, the floor pushes back with μ times the whole weight, however
   * weight transfer shares it out, along the way the wheels point and without turning the robot:
   * the loads shift in proportion to where each wheel sits, so their pushes' moments about the
   * center cancel. A friction pyramid (Coulomb friction along and across each wheel, apart) would
   * give up to 41% more at 45°.
   *
   * <p>Not on the template's tread, μ 1.2: there weight transfer loads the rear wheels (at an
   * angle, the outside ones) past what their motors push at 120 A, so those grip, and a correct
   * engine speeds the robot up straight ahead at 11.27 m/s², (2 F + μ m g / 2) / (m + 2 J / r² + μ
   * m h / L) with F a motor's push, below μ g's 11.77.
   */
  @Test
  void pastItsGripTheRobotSpeedsUpAtMuGInEveryDirection() {
    List<Double> accelerations = new ArrayList<>();
    for (int k = 0; k < 16; k++) {
      Rotation2d direction = Rotation2d.fromDegrees(22.5 * k);
      TruthRig rig = new TruthRig(SLICK);
      rig.steer(direction);
      rig.run(0.5);
      rig.limitDrives(120);
      rig.driveVolts(12);
      rig.run(0.06);
      Translation2d early = rig.velocity();
      rig.run(0.1);
      Translation2d change = rig.velocity().minus(early);

      accelerations.add(change.getNorm() / 0.1);
      assertThat(change.getAngle().orElseThrow().minus(direction).getDegrees())
          .as("the way the wheels point, at %s", direction)
          .isCloseTo(0, within(0.5));
    }

    double gripLimit = SLICK * ImuReadings.STANDARD_GRAVITY;
    double least = accelerations.stream().mapToDouble(Double::doubleValue).min().orElseThrow();
    double most = accelerations.stream().mapToDouble(Double::doubleValue).max().orElseThrow();
    assertThat(accelerations)
        .allSatisfy(a -> assertThat(a).isCloseTo(gripLimit, within(0.03 * gripLimit)));
    assertThat(most / least).isLessThan(1.05);
  }

  /**
   * From rest, the drive motors pushing straight ahead at a steady current: the least current at
   * which each wheel slips, found by halving between runs, puts each wheel's push at μ times its
   * own load, plus the share of it that spins the wheel itself up. Speeding up shifts load to the
   * rear wheels (m a h / (2 L) off each front wheel, h the center of gravity's height and L the
   * wheelbase), so the front wheels slip first. A front wheel's tread holds m a / 4 until that
   * reaches μ (m g / 4 − m a h / (2 L)), at a = μ g / (1 + 2 μ h / L), and its motor pushes (m + 4
   * J / r²) a / 4 then: 124.7 N (122.9 at the tread, the rest the wheel's own spin), about 48 A on
   * the template's robot, within 3%.
   *
   * <p>Today every wheel slips at once, at the push that would speed the whole robot up at μ g,
   * since the simulator caps the robot as a whole and loads every wheel alike.
   */
  @Test
  @KnownGap("the robot as a whole is held to μ g; no wheel carries a load of its own")
  void eachWheelSlipsWhereItsPushPassesMuTimesItsLoad() {
    TruthRig template = new TruthRig();
    double[] onsets = new double[4];
    for (int wheel = 0; wheel < 4; wheel++) {
      double gripping = 10;
      double slipping = 150;
      while (slipping - gripping > 0.05) {
        double amps = (gripping + slipping) / 2;
        if (slips(amps, wheel)) {
          slipping = amps;
        } else {
          gripping = amps;
        }
      }
      onsets[wheel] = template.force(slipping);
    }

    double g = ImuReadings.STANDARD_GRAVITY;
    double h = template.body.cgHeight().in(Meters);
    double wheelbase = template.wheels[0].getX() - template.wheels[2].getX();
    double front = template.effectiveMass() * MU * g / 4 / (1 + 2 * MU * h / wheelbase);
    boolean allAtOnce = Math.abs(onsets[0] - onsets[3]) < 0.01 * onsets[0];
    stillOpen("the front left wheel's push at its slip, N", onsets[0], front, 0.03, 208.9, 0.5);
    stillOpen(
        "the front wheels slip first",
        onsets[0] < onsets[2] && onsets[1] < onsets[3],
        allAtOnce,
        "the wheels' pushes at their slips are " + java.util.Arrays.toString(onsets) + " N");
  }

  /**
   * Speeding up straight ahead at the template's 30 A, gripping, the wheels carry the robot's
   * weight as quasi-static weight transfer shares it: each front wheel m g / 4 − m a h / (2 L),
   * each rear one as much more, within 3%, with a the robot's acceleration, h the center of
   * gravity's height, and L the wheelbase. Today every wheel carries a quarter of the weight
   * however hard the robot speeds up.
   */
  @Test
  @KnownGap("every wheel carries a quarter of the robot's weight")
  void speedingUpShiftsLoadFromTheFrontWheelsToTheRear() {
    TruthRig rig = new TruthRig();
    rig.driveVolts(12);
    rig.run(0.1);
    double before = rig.velocity().getX();
    rig.loops(1);
    double acceleration = (rig.velocity().getX() - before) / 0.02;

    double m = rig.mass();
    double h = rig.body.cgHeight().in(Meters);
    double wheelbase = rig.wheels[0].getX() - rig.wheels[2].getX();
    double front = m * ImuReadings.STANDARD_GRAVITY / 4 - m * acceleration * h / (2 * wheelbase);
    WheelTruth frontLeft = rig.simulation.physics().wheel(0);
    WheelTruth backLeft = rig.simulation.physics().wheel(2);
    assertThat(acceleration).as("speeding up").isGreaterThan(4);
    stillOpen("the front left wheel's load, N", frontLeft.loadNewtons(), front, 0.03, 166.25, 0.01);
    assertThat(backLeft.loadNewtons())
        .as("today, the back's too")
        .isEqualTo(frontLeft.loadNewtons());
  }

  /**
   * One module, the front left, pointing 10° off the others (its absolute encoder zeroed off, so
   * its steering holds it there), all four driven ahead at 1 m/s by their velocity loops: the robot
   * turns toward the odd wheel, counterclockwise, by more than 0.05° in a second, but no faster
   * than the odd wheel rolls along with the two rear ones, v sin 10° / (b (sin 10° + 2 cos 10°)),
   * 16.4° a second at 1 m/s (v the robot's speed, b half its wheelbase).
   *
   * <p>Statics says it can't go straight. Going straight, the odd wheel slides across itself, and
   * its friction (μ N, 199.5 N) pushes its corner left and turns the robot; the velocity loops make
   * no couple against that while the wheels agree on their speed, so the other three would have to
   * hold the push and its moment by their grip across them alone, which takes 214 N across the
   * other front wheel, past its 199.5 N. That wheel slides, and the robot turns. Turning faster
   * than the odd wheel rolls with the rear two, the odd wheel's slide reverses, and every wheel's
   * friction turns the robot back. Where between the two it settles hangs on how stiffly the
   * velocity loops resist the turn, which statics doesn't say.
   *
   * <p>Today the robot moves as the best fit of all four wheels, turning about 5° a second: within
   * the bound by geometry, not by friction. Odometry fits the wheels the same way, so it never errs
   * for it.
   */
  @Test
  void aWheelPointedOffTurnsTheRobotNoFasterThanItRolls() {
    TruthRig rig =
        new TruthRig(
            ExampleRobot.SIM.definition().body(), MU, SensorErrors.NONE.withSteerOffset(0, 10));
    double kv = ExampleRobot.SIM.drivetrain().gains().driveKv();
    rig.steer(Rotation2d.ZERO);
    rig.run(0.5);
    for (ModuleIO module : rig.modules) {
      module.setDriveVelocity(1.0 / rig.wheelRadius(), kv);
    }
    rig.run(0.5);

    Pose2d before = rig.simulation.truePose();
    rig.run(1);
    Pose2d after = rig.simulation.truePose();
    double turned = after.getRotation().minus(before.getRotation()).getDegrees();

    double speed = after.getTranslation().getDistance(before.getTranslation());
    double b = rig.wheels[0].getX();
    double off = Math.toRadians(10);
    double rolling = speed * Math.sin(off) / (b * (Math.sin(off) + 2 * Math.cos(off)));
    assertThat(speed).as("driven at 1 m/s").isCloseTo(1, within(0.05));
    assertThat(turned)
        .as("degrees turned toward the odd wheel in a second")
        .isGreaterThan(0.05)
        .isLessThan(Math.toDegrees(rolling));
  }

  /**
   * Whether this wheel slips within 0.2 s of a launch from rest at this current: its tread moving
   * over the floor faster than a millimeter a second, against the floor under it.
   */
  private static boolean slips(double amps, int wheel) {
    TruthRig rig = new TruthRig();
    rig.limitDrives(amps);
    rig.driveVolts(12);
    for (int loop = 0; loop < 10; loop++) {
      double before = rig.simulation.physics().wheel(wheel).rotationRadians();
      Translation2d start = rig.simulation.truePose().getTranslation();
      rig.loops(1);
      double rolled =
          (rig.simulation.physics().wheel(wheel).rotationRadians() - before) * rig.wheelRadius();
      double moved = rig.simulation.truePose().getTranslation().getDistance(start);
      if ((rolled - moved) / 0.02 > 0.001) {
        return true;
      }
    }
    return false;
  }
}

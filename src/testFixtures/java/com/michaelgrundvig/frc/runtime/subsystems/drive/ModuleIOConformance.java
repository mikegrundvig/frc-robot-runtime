package com.michaelgrundvig.frc.runtime.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;
import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleBench.Ability;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import java.util.Arrays;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.RobotController;

/**
 * The contract a swerve module's driver keeps ({@link ModuleIO}), as tests: the clauses about
 * units, signs and odometry readings, for every driver. It works as the joints' suite does ({@code
 * JointIOConformance}): a driver's test class hands this its bench ({@link ModuleBench}), a
 * scenario asks again every loop as the robot does, and waits for what it expects rather than for a
 * number of loops. A restarted drive's count is here (M16). The clauses about lost frames, a
 * restart's tuning and retuning are not here yet.
 */
@ExtendWith(RobotSim.class)
public abstract class ModuleIOConformance {
  private static final DrivetrainControl DRIVETRAIN = ExampleRobot.SIM.drivetrain();

  /** How many loops a scenario waits for something to become true before it fails. */
  private static final int PATIENCE_LOOPS = 100;

  /** How many loops something must stay true to be called steady. */
  private static final int STEADY_LOOPS = 5;

  /** How close an angle or a wheel's turn must read, in radians. */
  private static final double NEAR = 0.02;

  /** How close a wheel's turning must read, in radians: far inside a turn. */
  private static final double WHEEL_NEAR = 0.3;

  /** How close a voltage must be to the one expected, in volts. */
  private static final double VOLTS = 0.15;

  private final ModuleBench bench;
  private final ModuleIO.ModuleIOInputs inputs = new ModuleIO.ModuleIOInputs();
  private ModuleBench.@Nullable Rig poweredOn;

  /** What the scenario last asked of the driver, asked again every loop. Nothing at first. */
  private Runnable lastRequest = () -> {};

  protected ModuleIOConformance(ModuleBench bench) {
    this.bench = bench;
  }

  @AfterEach
  void powerOff() {
    if (poweredOn != null) {
      poweredOn.close();
      poweredOn = null;
    }
  }

  /**
   * Powers a module on with the example robot's gains, and room under its drive's current limit:
   * the volts the scenarios ask for stall a motor without reaching it.
   */
  private ModuleBench.Rig powerOn() {
    DrivetrainControl.Limits roomy =
        DrivetrainControlLimitsBuilder.builder(DRIVETRAIN.limits())
            .driveStatorCurrent(Amps.of(200))
            .build();
    ModuleBench.Rig rig = bench.powerOn(DRIVETRAIN.gains(), roomy);
    poweredOn = rig;
    eventually("both motors answer", () -> inputs.driveConnected && inputs.steerConnected);
    return rig;
  }

  private ModuleBench.Rig rig() {
    return Objects.requireNonNull(poweredOn, "nothing is powered on");
  }

  /** Asks something of the driver, now and again every loop until something else is asked. */
  private void ask(Runnable request) {
    lastRequest = request;
    request.run();
  }

  /** One loop, as the robot runs one: time passes, the inputs are read, the request is made. */
  private void loop() {
    rig().step();
    rig().io().updateInputs(inputs);
    lastRequest.run();
  }

  /** Waits for something to become true, a loop at a time, and fails if it never does. */
  private void eventually(String what, BooleanSupplier done) {
    for (int i = 0; i < PATIENCE_LOOPS; i++) {
      loop();
      if (done.getAsBoolean()) {
        return;
      }
    }
    fail("%s: %s never came true in %d loops. %s", bench.name(), what, PATIENCE_LOOPS, state());
  }

  /** Waits for something to become true and stay true for a few loops running. */
  private void settles(String what, BooleanSupplier holds) {
    int held = 0;
    for (int i = 0; i < PATIENCE_LOOPS; i++) {
      loop();
      held = holds.getAsBoolean() ? held + 1 : 0;
      if (held == STEADY_LOOPS) {
        return;
      }
    }
    fail(
        "%s: %s never held for %d loops running in %d loops. %s",
        bench.name(), what, STEADY_LOOPS, PATIENCE_LOOPS, state());
  }

  private String state() {
    return String.format(
        Locale.ROOT,
        "Now: drive restarted=%b position=%.4f velocity=%.4f volts=%.3f stator=%.2f A; steering"
            + " angle=%.4f velocity=%.4f volts=%.3f; odometry readings of the wheel %s",
        inputs.driveRestarted,
        inputs.drivePositionRad,
        inputs.driveVelocityRadPerSec,
        inputs.driveAppliedVolts,
        inputs.driveStatorCurrentAmps,
        inputs.steerAngle.getRadians(),
        inputs.steerVelocityRadPerSec,
        inputs.steerAppliedVolts,
        Arrays.toString(inputs.odometryDrivePositionsRad));
  }

  /** Checks a clause a driver may be known to differ on: such a driver must still fail it. */
  private void clause(ModuleClause clause, Runnable check) {
    Optional<String> difference = bench.knownDifference(clause);
    if (difference.isEmpty()) {
      check.run();
      return;
    }
    try {
      check.run();
    } catch (AssertionError stillDiffers) {
      return;
    }
    fail(
        "%s is listed as differing from %s (%s), but it keeps the clause now: take the entry out of"
            + " its bench's known differences",
        bench.name(), clause, difference.get());
  }

  private void needs(Ability ability) {
    Optional<String> why = bench.lacks(ability);
    Assumptions.assumeTrue(
        why.isEmpty(), () -> bench.name() + " cannot " + ability + ": " + why.orElse(""));
  }

  /**
   * The wheel reads in radians at the wheel, through the drive's gearing, with its sign, and its
   * speed in radians a second. Every device took its settings.
   */
  @Test
  void m1m17_theWheelReadsInRadiansAtTheWheelForwardPositive() {
    powerOn();
    double[][] rows = {{4 * Math.PI, 10}, {-2 * Math.PI, -10}};

    for (double[] row : rows) {
      rig().wheelIs(row[0], row[1]);

      eventually(
          "M1: the wheel at " + row[0] + " rad turning at " + row[1] + " rad/s",
          () ->
              Math.abs(inputs.drivePositionRad - row[0]) <= 0.3
                  && Math.abs(inputs.driveVelocityRadPerSec - row[1]) <= 0.3);
    }
    assertThat(inputs.unconfigured).as("M17").isEmpty();
  }

  /**
   * The module's angle reads counterclockwise from straight ahead, within half a turn either way:
   * an angle past half a turn reads as the same direction the other way round.
   */
  @Test
  void m3_theModulesAngleIsWithinHalfATurnEitherWay() {
    powerOn();
    double[][] rows = {{0.5, 0.5}, {-2.0, -2.0}, {3.5, 3.5 - 2 * Math.PI}};

    for (double[] row : rows) {
      rig().moduleIs(row[0]);

      eventually(
          "M3: the module at " + row[0] + " rad reads " + row[1],
          () -> Math.abs(inputs.steerAngle.getRadians() - row[1]) <= NEAR);
    }
  }

  /**
   * Odometry's readings come as three lists of one length, oldest first, each timed on the robot's
   * clock no later than now, and the newest says where the wheel and the module are.
   */
  @Test
  void m12_odometryReadingsComeInStepTimedOnTheRobotsClock() {
    powerOn();
    rig().wheelIs(2 * Math.PI, 0);
    rig().moduleIs(0.5);
    eventually(
        "M12: a reading of the wheel a turn on and the module at half a radian",
        () -> {
          int newest = inputs.odometryTimestampsNanos.length - 1;
          return newest >= 0
              && Math.abs(inputs.odometryDrivePositionsRad[newest] - 2 * Math.PI) <= 0.3
              && Math.abs(inputs.odometrySteerAnglesRad[newest] - 0.5) <= NEAR;
        });

    long[] times = inputs.odometryTimestampsNanos;
    assertThat(inputs.odometryDrivePositionsRad).as("M12").hasSameSizeAs(times);
    assertThat(inputs.odometrySteerAnglesRad).as("M12").hasSameSizeAs(times);
    assertThat(times).as("M12: oldest first").isSorted();
    assertThat(times[times.length - 1])
        .as("M12: on the robot's clock, not after now")
        .isLessThanOrEqualTo(RobotController.getTime());
  }

  /**
   * A drive controller that restarts counts its wheel from zero, and the driver says so in exactly
   * one loop. Every reading before that loop is of the old count and every reading after it is of
   * the new one, the odometry readings too, so a drivetrain that leaves the wheel out for that one
   * loop never takes the count starting again for travel. From there the count moves as far as the
   * wheel does.
   */
  @Test
  void m16_aRestartedDriveSaysSoInTheLoopItsCountStartsAgain() {
    ModuleBench.Rig rig = powerOn();
    double rolled = 4 * Math.PI;
    rig.wheelIs(rolled, 0);
    settles("the wheel reads two turns on", () -> everyReadingOfTheWheelIs(rolled));
    assertThat(inputs.driveRestarted).as("M16: nothing restarted yet").isFalse();

    rig.restartTheDrive();
    int saidSo = 0;
    for (int i = 0; i < 4 * STEADY_LOOPS; i++) {
      loop();
      if (inputs.driveRestarted) {
        // This loop's own readings may be of either count.
        saidSo++;
        continue;
      }
      double count = saidSo == 0 ? rolled : 0;
      assertThat(everyReadingOfTheWheelIs(count))
          .as(
              "M16: %d loops on, %s the loop that said it restarted, every reading of the wheel is"
                  + " %.2f rad. %s",
              i + 1, saidSo == 0 ? "before" : "after", count, state())
          .isTrue();
    }
    assertThat(saidSo).as("M16: said in exactly one loop").isEqualTo(1);

    rig.wheelIs(rolled + 2 * Math.PI, 0);
    eventually("M16: it counts on from zero, a turn", () -> everyReadingOfTheWheelIs(2 * Math.PI));
  }

  /**
   * Whether the wheel reads near an angle, in radians, in the loop's reading and in each of its
   * odometry readings. An odometry reading that is not a number is one that did not come, which the
   * drivetrain passes over.
   */
  private boolean everyReadingOfTheWheelIs(double radians) {
    if (Math.abs(inputs.drivePositionRad - radians) > WHEEL_NEAR) {
      return false;
    }
    for (double reading : inputs.odometryDrivePositionsRad) {
      if (Double.isFinite(reading) && Math.abs(reading - radians) > WHEEL_NEAR) {
        return false;
      }
    }
    return true;
  }

  /**
   * A wheel speed runs the drive's loop: kP on the error in the wheel's speed over the floor, plus
   * the volts the request adds, either way, and the driver reports the same voltage with its sign.
   * Stop drives it no more.
   */
  @Test
  void m5m7m8_aWheelSpeedRunsTheDrivesLoopSigned() {
    needs(Ability.SHOW_THE_DRIVES_VOLTS);
    ModuleBench.Rig rig = powerOn();
    rig.wheelIs(0, 0);
    double kP = DRIVETRAIN.gains().driveKp() * bench.wheelRadiusMeters();

    for (double speed : new double[] {8, -8}) {
      double volts = kP * speed + Math.signum(speed) * 1.5;
      ask(() -> rig.io().setDriveVelocity(speed, Math.signum(speed) * 1.5));

      settles(
          String.format(Locale.ROOT, "M5: %.1f V for %.0f rad/s, standing", volts, speed),
          () -> Math.abs(rig.driveVolts() - volts) <= VOLTS + 0.05 * Math.abs(volts));
      if (bench.lacks(Ability.RUN_THE_DRIVE_MOTOR).isEmpty()) {
        assertThat(Math.signum(inputs.driveAppliedVolts)).as("M8").isEqualTo(Math.signum(speed));
        assertThat(inputs.driveStatorCurrentAmps).as("M9: stalled").isGreaterThan(1);
      }
    }
    ask(() -> rig.io().stop());
    settles("M7: stopped", () -> Math.abs(rig.driveVolts()) <= VOLTS);
  }

  /**
   * A wheel that rolls faster than its drive's volts would turn it makes the motor a generator,
   * which sends current back to the battery. The currents the driver reports are sizes all the
   * same: neither is negative.
   */
  @Test
  void m9m10_theDrivesCurrentsAreNeverNegativeWhileItSendsCurrentBack() {
    needs(Ability.RUN_THE_DRIVE_MOTOR);
    ModuleBench.Rig rig = powerOn();
    rig.wheelIs(0, 60);
    ask(() -> rig.io().setDriveVelocity(60, 3));
    eventually("current flows", () -> inputs.driveStatorCurrentAmps > 5);

    double leastStator = Double.POSITIVE_INFINITY;
    double leastSupply = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 2 * STEADY_LOOPS; i++) {
      loop();
      leastStator = Math.min(leastStator, inputs.driveStatorCurrentAmps);
      leastSupply = Math.min(leastSupply, inputs.driveSupplyCurrentAmps);
    }

    double stator = leastStator;
    double supply = leastSupply;
    clause(ModuleClause.M9, () -> assertThat(stator).as("M9").isNotNegative());
    clause(ModuleClause.M10, () -> assertThat(supply).as("M10").isNotNegative());
  }

  /**
   * An angle runs the steering's loop toward it, either way, and across the back of the module the
   * short way round. Stop drives it no more.
   */
  @Test
  void m6m7_anAngleRunsTheSteeringsLoopTheShortWayRound() {
    needs(Ability.SHOW_THE_STEERINGS_VOLTS);
    ModuleBench.Rig rig = powerOn();
    // Where the module is, the angle asked, and the way the steering should push: 1 forward.
    double[][] rows = {{0, 0.3, 1}, {0, -0.3, -1}, {3.0, -3.0, 1}, {-3.0, 3.0, -1}};

    for (double[] row : rows) {
      rig.moduleIs(row[0]);
      eventually(
          "the module reads " + row[0],
          () -> Math.abs(inputs.steerAngle.getRadians() - row[0]) <= NEAR);
      ask(() -> rig.io().setSteerAngle(new Rotation2d(row[1])));

      settles(
          "M6: from " + row[0] + " rad to " + row[1] + " rad",
          () -> row[2] * rig.steerVolts() > VOLTS);
    }
    ask(() -> rig.io().stop());
    settles("M7: stopped", () -> Math.abs(rig.steerVolts()) <= VOLTS);
  }
}

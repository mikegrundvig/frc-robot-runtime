package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;

/** The steering's motor: exactly WPILib's model below its current limit, a steady torque at it. */
class CurrentLimitedMotorTest {
  /** The template's steering: a Kraken X60 turning an MK4i module through 150/7. */
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

  private static final double INERTIA_KG_M2 = 0.004;
  private static final double GEARING = 150.0 / 7.0;

  /** At 40 A, how fast the module speeds up: the motor's torque through the gearing. */
  private static final double ACCELERATION_AT_40_AMPS = 40 * MOTOR.Kt * GEARING / INERTIA_KG_M2;

  private final CurrentLimitedMotor motor = new CurrentLimitedMotor(MOTOR, INERTIA_KG_M2, GEARING);

  @Test
  void belowItsLimitItMovesExactlyAsWpilibsModel() {
    LinearSystem<N2, N1, N2> plant =
        Models.singleJointedArmFromPhysicalConstants(MOTOR, INERTIA_KG_M2, GEARING);
    Matrix<N2, N1> state = VecBuilder.fill(0, 0);

    for (double volts : new double[] {12, -6, 3, 0}) {
      motor.run(volts, Double.POSITIVE_INFINITY, 0.004);
      state = plant.calculateX(state, VecBuilder.fill(volts), 0.004);

      assertThat(motor.positionRadians()).isCloseTo(state.get(0, 0), within(1e-9));
      assertThat(motor.velocityRadPerSec()).isCloseTo(state.get(1, 0), within(1e-9));
    }
  }

  @Test
  void atItsLimitTheMotorGivesASteadyTorque() {
    // From rest, 12 V would draw the motor's stall current, 366 A: held to 40, for now.
    double peak = motor.run(12, 40, 0.002);

    assertThat(peak).isEqualTo(40);
    assertThat(motor.currentAmps()).isCloseTo(40, within(1e-9));
    assertThat(motor.velocityRadPerSec()).isCloseTo(ACCELERATION_AT_40_AMPS * 0.002, within(1e-9));
    assertThat(motor.positionRadians())
        .isCloseTo(ACCELERATION_AT_40_AMPS * 0.002 * 0.002 / 2, within(1e-12));
    double backEmf = motor.velocityRadPerSec() * GEARING / MOTOR.Kv;
    assertThat(motor.appliedVolts())
        .as("the motor's back EMF, and the limit's worth across its windings")
        .isCloseTo(backEmf + 40 * MOTOR.R, within(1e-9));
  }

  @Test
  void itStopsAtItsLimitToo() {
    motor.run(12, Double.POSITIVE_INFINITY, 0.05); // at full speed
    double speed = motor.velocityRadPerSec();

    // Full reverse: twice the stall current, back EMF and battery adding up. Held to 40 A.
    double peak = motor.run(-12, 40, 0.002);

    assertThat(peak).isEqualTo(40);
    assertThat(motor.currentAmps()).isCloseTo(-40, within(1e-9));
    assertThat(motor.velocityRadPerSec())
        .isCloseTo(speed - ACCELERATION_AT_40_AMPS * 0.002, within(1e-9));
  }

  /**
   * The limit holds however the time is cut: the motion is the same in one piece or a hundred, and
   * the current is never past the limit, then falls below it as the motor reaches the voltage's
   * speed.
   */
  @Test
  void theLimitHoldsHoweverTheTimeIsCut() {
    CurrentLimitedMotor whole = new CurrentLimitedMotor(MOTOR, INERTIA_KG_M2, GEARING);
    whole.run(12, 40, 0.02);

    double most = 0;
    for (int i = 0; i < 100; i++) {
      most = Math.max(most, motor.run(12, 40, 0.0002));
      assertThat(Math.abs(motor.currentAmps())).isLessThanOrEqualTo(40 + 1e-9);
    }

    assertThat(motor.positionRadians()).isCloseTo(whole.positionRadians(), within(1e-9));
    assertThat(motor.velocityRadPerSec()).isCloseTo(whole.velocityRadPerSec(), within(1e-9));
    assertThat(most).isEqualTo(40);
    assertThat(motor.currentAmps()).as("near full speed").isBetween(0.0, 5.0);
    assertThat(motor.appliedVolts()).isEqualTo(12);
  }

  /**
   * Below the limit, a steady torque on the load is the same as the voltage that would drive its
   * current through the windings: gravity on a joint moves it exactly as WPILib's model does with
   * that voltage added.
   */
  @Test
  void aSteadyLoadIsTheVoltageItsCurrentWouldTake() {
    double torque = -0.6; // newton meters at the load: gravity on a light arm
    double asVolts = torque / (GEARING * MOTOR.Kt) * MOTOR.R;
    CurrentLimitedMotor loaded = new CurrentLimitedMotor(MOTOR, INERTIA_KG_M2, GEARING);

    for (double volts : new double[] {12, -6, 3, 0}) {
      loaded.run(volts, Double.POSITIVE_INFINITY, torque, 0.004);
      motor.run(volts + asVolts, Double.POSITIVE_INFINITY, 0.004);

      assertThat(loaded.positionRadians()).isCloseTo(motor.positionRadians(), within(1e-9));
      assertThat(loaded.velocityRadPerSec()).isCloseTo(motor.velocityRadPerSec(), within(1e-9));
    }
  }

  @Test
  void atItsLimitALoadAddsToTheSteadyTorque() {
    double torque = -0.6;

    motor.run(12, 40, torque, 0.002);

    assertThat(motor.velocityRadPerSec())
        .isCloseTo((ACCELERATION_AT_40_AMPS + torque / INERTIA_KG_M2) * 0.002, within(1e-9));
  }

  /** A load more than the limit can hold: the motor pushes at its limit and still loses. */
  @Test
  void aLoadTheLimitCantHoldWins() {
    double torque = -2 * 40 * MOTOR.Kt * GEARING; // twice what 40 A gives at the load

    double amps = motor.run(12, 40, torque, 0.02);

    assertThat(amps).isEqualTo(40);
    assertThat(motor.velocityRadPerSec())
        .isCloseTo((ACCELERATION_AT_40_AMPS + torque / INERTIA_KG_M2) * 0.02, within(1e-9));
    assertThat(motor.velocityRadPerSec()).isNegative();
  }

  /** Coasting, no current flows: only the load moves it, as a body falling. */
  @Test
  void coastingOnlyTheLoadMovesIt() {
    motor.run(12, Double.POSITIVE_INFINITY, 0.05);
    double speed = motor.velocityRadPerSec();
    double position = motor.positionRadians();

    motor.coast(-0.6, 0.02);

    double acceleration = -0.6 / INERTIA_KG_M2;
    assertThat(motor.velocityRadPerSec()).isCloseTo(speed + acceleration * 0.02, within(1e-9));
    assertThat(motor.positionRadians())
        .isCloseTo(position + speed * 0.02 + acceleration * 0.02 * 0.02 / 2, within(1e-9));
    assertThat(motor.currentAmps()).isZero();
    assertThat(motor.appliedVolts()).isZero();
  }
}

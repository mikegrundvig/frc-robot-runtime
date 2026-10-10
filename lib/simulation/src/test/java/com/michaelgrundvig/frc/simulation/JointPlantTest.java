package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;

/**
 * A joint as the simulator moves it, from the definition alone: an arm of 2 kg 0.3 m out on a
 * Kraken through 50:1, an unweighed one held by its tuned kG, an elevator on two Krakens through
 * 10:1 onto a 1 in drum, and a turret gravity can't move.
 */
class JointPlantTest {
  private static final double STEP = 0.001;
  private static final DCMotor KRAKEN = Motor.KRAKEN_X60.model();

  private static final Joint.Revolute ARM =
      Joint.pivot("arm", Part.CHASSIS, new Translation3d(0.1, 0, 0.5), Joint.PITCH_UP)
          .range(Range.degrees(-60, 90))
          .rest(Degrees.of(0))
          .carries(
              Link.weighed(
                  "arm link",
                  Kilograms.of(2),
                  new Translation3d(0.3, 0, 0),
                  new Translation3d(0.6, 0.05, 0.05)))
          .motor(Motor.KRAKEN_X60, Gearing.of(50))
          .control(MechanismControl.position(30));

  private static final Joint.Revolute BARE_ARM =
      Joint.pivot("bare arm", Part.CHASSIS, new Translation3d(-0.1, 0, 0.5), Joint.PITCH_UP)
          .range(Range.degrees(-60, 90))
          .rest(Degrees.of(0))
          .motor(Motor.KRAKEN_X60, Gearing.of(50))
          .control(MechanismControl.position(30).withKg(0.4, Degrees.of(-30)));

  private static final Joint.Prismatic LIFT =
      Joint.elevator("lift", Part.CHASSIS, new Translation3d(0, 0.2, 0.1))
          .range(Range.meters(0, 1))
          .rest(Meters.of(0))
          .motors(Motor.KRAKEN_X60, Gearing.of(10), Inches.of(1), 2)
          .control(MechanismControl.position(60).withKg(0.5));

  private static final Joint.Revolute TURRET =
      Joint.roller("turret", Part.CHASSIS, new Translation3d(0, -0.2, 0.4), Vector3.UP)
          .motor(Motor.KRAKEN_X60, Gearing.of(20))
          .control(MechanismControl.position(20));

  private static final RobotMath MATH = math(List.of(ARM, BARE_ARM, LIFT, TURRET));

  private static RobotMath math(List<Joint> joints) {
    List<Drive> drives = new ArrayList<>();
    joints.forEach(joint -> drives.addAll(joint.ownDrives()));
    return RobotMath.of(TestRobot.BODY, joints, List.of(), drives, List.of());
  }

  /** Runs a joint at a voltage for a time, gravity worked out where it is at each step. */
  private static void run(JointPlant plant, double volts, double limitAmps, double seconds) {
    for (int i = 0; i < Math.round(seconds / STEP); i++) {
      double gravity = plant.gravityTorque(Map.of(plant.joint(), plant.position()));
      plant.run(volts, limitAmps, gravity, STEP);
    }
  }

  @Test
  void itStartsStillWhereItsDefinitionRestsIt() {
    JointPlant lift = JointPlant.of(MATH, LIFT);

    assertThat(lift.joint()).isEqualTo(LIFT);
    assertThat(lift.motors()).isEqualTo(2);
    assertThat(lift.position()).isEqualTo(LIFT.restPosition());
    assertThat(lift.velocity()).isZero();
    assertThat(lift.supplyAmps()).isZero();
  }

  /**
   * Below its current limit, a joint gravity can't move closes on the speed its voltage gives
   * through its gearing: a Kraken's at 6 V, through 20:1.
   */
  @Test
  void itClosesOnTheSpeedItsVoltageGives() {
    JointPlant turret = JointPlant.of(MATH, TURRET);

    run(turret, 6, 1000, 1);

    assertThat(turret.gravityTorque(Map.of())).isZero();
    assertThat(turret.velocity()).isCloseTo(6 * KRAKEN.Kv / 20, within(1e-9));
    assertThat(turret.appliedVolts()).isEqualTo(6);
  }

  /** At its limit, each motor's current is held to it, and the joint speeds up steadily. */
  @Test
  void atItsLimitEachMotorIsHeldToIt() {
    JointPlant turret = JointPlant.of(MATH, TURRET);
    double inertia = MATH.inertia(TURRET, Map.of());

    turret.run(12, 20, 0, STEP);

    assertThat(turret.statorAmps()).isCloseTo(20, within(1e-9));
    assertThat(turret.velocity()).isCloseTo(20 * KRAKEN.Kt * 20 / inertia * STEP, within(1e-9));
  }

  /**
   * A tuned kG holds an arm with no mass: gravity on it is what that kG holds through its motors,
   * along one cosine about the angle given with it; a slide's, the same all along.
   */
  @Test
  void gravityIsWhatTheBlocksTunedKgHolds() {
    JointPlant arm = JointPlant.of(MATH, BARE_ARM);
    JointPlant lift = JointPlant.of(MATH, LIFT);
    double armPerVolt = 50 * KRAKEN.stallTorque / KRAKEN.nominalVoltage;
    double liftPerVolt = 2 * 10 / Inches.of(1).in(Meters) * KRAKEN.stallTorque / 12;

    arm.place(Math.toRadians(-30));
    double hardest = arm.gravityTorque(Map.of());
    arm.place(Math.toRadians(60));
    double hanging = arm.gravityTorque(Map.of());
    lift.place(0.7);

    assertThat(hardest).isCloseTo(-0.4 * armPerVolt, within(1e-9));
    assertThat(hanging).isCloseTo(0, within(1e-9));
    assertThat(lift.gravityTorque(Map.of())).isCloseTo(-0.5 * liftPerVolt, within(1e-6));
  }

  /** Without a tuned kG, gravity is its links' weight: m g r, at the angle the joints stand. */
  @Test
  void withoutATunedKgGravityIsItsLinksWeight() {
    JointPlant arm = JointPlant.of(MATH, ARM);

    assertThat(arm.gravityTorque(Map.of(ARM, 0.0)))
        .isCloseTo(-2 * RobotMath.GRAVITY * 0.3, within(1e-9));
    assertThat(arm.gravityTorque(Map.of(ARM, Math.toRadians(60))))
        .isCloseTo(-2 * RobotMath.GRAVITY * 0.3 / 2, within(1e-9));
  }

  /** Coasting, it falls under its weight onto its lower stop, and stays there, still. */
  @Test
  void itsHardStopsStopItDead() {
    JointPlant arm = JointPlant.of(MATH, ARM);

    for (int i = 0; i < 2000; i++) {
      arm.coast(arm.gravityTorque(Map.of(ARM, arm.position())), STEP);
    }

    assertThat(arm.position()).isEqualTo(Math.toRadians(-60));
    assertThat(arm.velocity()).isZero();
    assertThat(arm.supplyAmps()).as("coasting, no current").isZero();
  }

  /**
   * Driven into its upper stop, it's held there, its motor stalled: at 3 V, a stalled Kraken would
   * take 91 A, so it's held to its 40 A, and the voltage across it is what drives 40 A through its
   * windings standing still, 1.3 V: 4.4 A from the battery, not the 15 A a motor still turning
   * would draw. At 0.5 V, under its limit, it takes what the voltage drives.
   */
  @Test
  void drivenIntoAStopItsMotorsStall() {
    JointPlant arm = JointPlant.of(MATH, ARM);
    arm.place(Math.toRadians(85));

    run(arm, 3, 40, 0.5);
    double pushed = arm.supplyAmps();
    double held = arm.statorAmps();
    double across = arm.appliedVolts();
    run(arm, 0.5, 40, 0.1);

    assertThat(arm.position()).isEqualTo(Math.toRadians(90));
    assertThat(arm.velocity()).isZero();
    assertThat(held).isCloseTo(40, within(1e-9));
    assertThat(across).isCloseTo(40 * KRAKEN.R, within(1e-12));
    assertThat(pushed).isCloseTo(40 * 40 * KRAKEN.R / 12, within(1e-9)).isCloseTo(4.4, within(0.1));
    assertThat(arm.statorAmps()).isCloseTo(0.5 / KRAKEN.R, within(1e-9));
    assertThat(arm.supplyAmps()).isCloseTo(0.5 / KRAKEN.R * 0.5 / 12, within(1e-9));
  }

  /**
   * Jammed, it stays where it is, its motors stalled against what holds it either way, as against a
   * stop, under its weight too; freed, it moves again.
   */
  @Test
  void jammedItsMotorsStallWhereItIs() {
    JointPlant arm = JointPlant.of(MATH, ARM);
    arm.place(Math.toRadians(30));

    arm.jam(true);
    run(arm, 3, 40, 0.5);
    double up = arm.position();
    double held = arm.statorAmps();
    run(arm, -3, 40, 0.5);
    double down = arm.position();
    arm.jam(false);
    run(arm, 3, 40, 0.1);

    assertThat(up).isEqualTo(Math.toRadians(30));
    assertThat(held).isCloseTo(40, within(1e-9));
    assertThat(down).isEqualTo(Math.toRadians(30));
    assertThat(arm.position()).as("freed").isGreaterThan(Math.toRadians(35));
  }

  /**
   * Driven by one of its two motors, the other left neutral, the lift speeds up at its limit half
   * as fast, each motor that drives it held to the limit; driven by both again, as fast as ever. It
   * has two motors to drive it, no fewer than one.
   */
  @Test
  void drivenByFewerOfItsMotorsItSpeedsUpSlower() {
    JointPlant both = JointPlant.of(MATH, LIFT);
    JointPlant one = JointPlant.of(MATH, LIFT);
    both.place(0.5);
    one.place(0.5);

    one.drivenBy(1);
    both.run(12, 20, 0, STEP);
    one.run(12, 20, 0, STEP);
    double alone = one.velocity();
    double held = one.statorAmps();
    one.drivenBy(2);
    one.run(12, 20, 0, STEP);

    assertThat(alone).isCloseTo(both.velocity() / 2, within(1e-12));
    assertThat(held).isCloseTo(20, within(1e-9));
    assertThat(one.velocity() - alone).isCloseTo(both.velocity(), within(1e-9));
    assertThatIllegalArgumentException().isThrownBy(() -> one.drivenBy(0));
    assertThatIllegalArgumentException().isThrownBy(() -> one.drivenBy(3));
  }

  /** Stepped the same, it comes out the same, to the bit. */
  @Test
  void itRepeatsToTheBit() {
    JointPlant first = JointPlant.of(MATH, LIFT);
    JointPlant second = JointPlant.of(MATH, LIFT);

    run(first, 9, 30, 0.4);
    run(second, 9, 30, 0.4);

    assertThat(second.position()).isEqualTo(first.position());
    assertThat(second.velocity()).isEqualTo(first.velocity());
    assertThat(second.supplyAmps()).isEqualTo(first.supplyAmps());
  }
}

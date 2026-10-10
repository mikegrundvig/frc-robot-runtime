package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism.State;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;

/**
 * How a joint finds where it is, a loop at a time: into a hard stop, onto its switch, or from an
 * absolute encoder the robot reads, and where it stands meanwhile. No simulator: each loop is what
 * a test hands it, and where it's found is what it tells the joint.
 */
class JointHomingTest {
  private static final long LOOP = 20_000_000L;

  /** Where each homing told its joint it is. */
  private final List<Double> found = new ArrayList<>();

  private long now = 1_000_000_000L;

  private JointHoming homing(Joint joint, @Nullable Homing recipe) {
    return new JointHoming(ROBOT.math(), joint, recipe, false, false, -1, found::add);
  }

  private JointHoming byEncoder(boolean closesLoop) {
    return new JointHoming(ROBOT.math(), PIVOT, null, true, closesLoop, 1, found::add);
  }

  /** A loop of a joint homed by its recipe, enabled throughout, moving at a speed. */
  private void loop(JointHoming homing, boolean enabled, boolean enabling, double velocity) {
    now += LOOP;
    homing.goOn(now, enabled, enabling, velocity, null);
  }

  /** A joint with neither a recipe nor an encoder the robot reads is ready from the start. */
  @Test
  void aJointWithNothingToFindIsReady() {
    JointHoming homing = homing(PIVOT, null);

    homing.start(now);

    assertThat(homing.state()).isEqualTo(State.READY);
    assertThat(homing.shown()).isEqualTo("READY");
    assertThat(homing.waits()).isFalse();
    assertThat(homing.fault()).isNull();
  }

  /**
   * Pushed into its stop on the first enable, the elevator is found at its range's end once it's
   * been still a quarter second, and isn't driven the loop it's found.
   */
  @Test
  void intoAHardStopItsFoundOnceStill() {
    JointHoming homing = homing(ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER));
    homing.start(now);
    State before = homing.state();
    loop(homing, true, true, -0.3);
    String shown = homing.shown();
    double volts = homing.volts();
    for (int i = 0; i < 13; i++) {
      loop(homing, true, false, 0);
    }
    List<Double> early = List.copyOf(found);
    loop(homing, true, false, 0);
    boolean justFound = homing.justFound();
    homing.settled();

    assertThat(before).isEqualTo(State.UNHOMED);
    assertThat(shown).isEqualTo("HOMING: toward its lower stop, for at most 9.5 s");
    assertThat(volts).isEqualTo(-2);
    assertThat(early).as("not still a quarter second yet").isEmpty();
    assertThat(found).containsExactly(0.0);
    assertThat(justFound).isTrue();
    assertThat(homing.state()).isEqualTo(State.READY);
    assertThat(homing.justFound()).isFalse();
  }

  /**
   * One that never finds its stop in its time is faulted, saying so, and homes again when asked;
   * disabled while it homes, it starts again on the next enable.
   */
  @Test
  void aHomingThatFindsNoStopIsFaulted() {
    JointHoming homing =
        homing(ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER, Seconds.of(0.3)));
    homing.start(now);
    loop(homing, true, true, -0.3);
    for (int i = 0; i < 15; i++) {
      loop(homing, true, false, -0.3);
    }
    String fault = homing.fault();
    homing.again(now, true, null);
    State again = homing.state();
    loop(homing, false, false, 0);
    State disabled = homing.state();
    loop(homing, true, true, 0);

    assertThat(fault)
        .isEqualTo("elevator's homing found no stop in 0.3 s, pushing toward its lower one");
    assertThat(again).isEqualTo(State.HOMING);
    assertThat(disabled).isEqualTo(State.UNHOMED);
    assertThat(homing.state()).isEqualTo(State.HOMING);
    homing.stop();
    assertThat(homing.state()).as("its command canceled").isEqualTo(State.UNHOMED);
  }

  /**
   * Its homing canceled with the robot enabled, it's unhomed; a move waiting on it homes it again
   * at once. Disabled, the move waits for the next enable. Ready, or with nothing to find, nothing
   * waits.
   */
  @Test
  void aMoveWaitingOnACanceledHomingHomesItAgain() {
    JointHoming homing = homing(ELEVATOR, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER));
    homing.start(now);
    loop(homing, true, true, -0.3);
    homing.stop();
    State canceled = homing.state();
    boolean waitsDisabled = homing.awaited(now, false, null);
    State disabled = homing.state();
    boolean waits = homing.awaited(now, true, null);
    JointHoming none = homing(PIVOT, null);
    none.start(now);

    assertThat(canceled).isEqualTo(State.UNHOMED);
    assertThat(waitsDisabled).isTrue();
    assertThat(disabled).as("it homes on the next enable").isEqualTo(State.UNHOMED);
    assertThat(waits).isTrue();
    assertThat(homing.state()).isEqualTo(State.HOMING);
    assertThat(homing.shown()).isEqualTo("HOMING: toward its lower stop, for at most 9.5 s");
    assertThat(none.awaited(now, true, null)).isFalse();
    assertThat(none.state()).isEqualTo(State.READY);
  }

  /** The pivot's bottom switch, tripping at 5°: above where it rests, at 1.5°. */
  private static final Sensor.Switch PIVOT_DOWN =
      JointFixtures.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Math.toRadians(5));

  /**
   * Starting on its switch, it backs off until the switch clears, then comes back until it trips,
   * and it's found halfway along that loop's travel past where the switch trips.
   */
  @Test
  void ontoItsSwitchItBacksOffFirstWhenItStartsOnIt() {
    SwitchReadings switches = new SwitchReadings(PIVOT_DOWN);
    JointHoming homing = homing(PIVOT, Homing.toSwitch(Sensor.Switch.Limit.End.LOWER));
    homing.start(now);
    homing.goOn(now, true, true, 0, PIVOT_DOWN);
    double pushing = homing.volts();
    switches.trip(PIVOT_DOWN, true);
    homing.sensed(now, 0.02, PIVOT_DOWN, switches.sensors());
    double backingOff = homing.volts();
    homing.sensed(now += LOOP, 0.06, PIVOT_DOWN, switches.sensors());
    switches.trip(PIVOT_DOWN, false);
    homing.sensed(now += LOOP, 0.1, PIVOT_DOWN, switches.sensors());
    double headingIn = homing.volts();
    switches.trip(PIVOT_DOWN, true);
    homing.sensed(now += LOOP, 0.08, PIVOT_DOWN, switches.sensors());

    assertThat(homing.switchEnd()).isEqualTo(Sensor.Switch.Limit.End.LOWER);
    assertThat(pushing).isEqualTo(-2);
    assertThat(backingOff).isEqualTo(2);
    assertThat(headingIn).isEqualTo(-2);
    assertThat(found).hasSize(1);
    assertThat(found.get(0)).isCloseTo(Math.toRadians(5) - 0.01, within(1e-12));
    assertThat(homing.justFound()).isTrue();
  }

  /** With its switch lost, it's faulted once its time has passed, saying the switch is lost. */
  @Test
  void aLostSwitchFaultsItsHoming() {
    SwitchReadings switches = new SwitchReadings(PIVOT_DOWN).lose(PIVOT_DOWN, true);
    JointHoming homing = homing(PIVOT, Homing.toSwitch(Sensor.Switch.Limit.End.LOWER));
    homing.start(now);
    homing.goOn(now, true, true, 0, PIVOT_DOWN);
    for (int i = 0; i < 82; i++) {
      homing.sensed(now += LOOP, 0.5, PIVOT_DOWN, switches.sensors());
    }

    assertThat(homing.fault())
        .isEqualTo(
            "pivot's homing didn't trip pivot down in 1.6 s (its travel takes 0.6 s at 2.0 V):"
                + " pivot down is lost");
  }

  /**
   * From its absolute encoder, it waits for the first reading, which tells the joint where it is:
   * driven from the next loop by its motor's new count, or at once by an encoder its loop closes
   * on.
   */
  @Test
  void anEncodersFirstReadingSaysWhereItIs() {
    JointHoming seeded = byEncoder(false);
    JointHoming closed = byEncoder(true);
    seeded.start(now);
    closed.start(now);
    State waiting = seeded.state();

    seeded.seek(now, true, 0.7);
    closed.seek(now, true, 0.7);

    assertThat(waiting).isEqualTo(State.UNHOMED);
    assertThat(seeded.waits()).isFalse();
    assertThat(found).containsExactly(0.7, 0.7);
    assertThat(seeded.justFound()).isTrue();
    assertThat(closed.justFound()).isFalse();
    assertThat(closed.state()).isEqualTo(State.READY);
    assertThat(seeded.awaiting()).isFalse();
  }

  /** With no reading in 2 s, it's faulted, saying so; a reading readies it. */
  @Test
  void anEncoderThatGivesNoReadingFaultsUntilItDoes() {
    JointHoming homing = byEncoder(false);
    homing.start(now);
    homing.seek(now + 1_980_000_000L, false, Double.NaN);
    State early = homing.state();
    homing.seek(now + 2_000_000_000L, false, Double.NaN);
    String fault = homing.fault();
    homing.seek(now + 3_000_000_000L, true, 0.4);

    assertThat(early).isEqualTo(State.UNHOMED);
    assertThat(fault)
        .isEqualTo(
            "pivot's absolute encoder gave no reading in 2.0 s, on port 1: it's seeded when it"
                + " does");
    assertThat(found).containsExactly(0.4);
    assertThat(homing.state()).isEqualTo(State.READY);
  }

  /**
   * The encoder its loop closes on lost, it's faulted and holds where it stops by its motor's
   * count; disabled, wherever it is as it's enabled; homed again meanwhile, it still holds there
   * while it waits; and the encoder's next reading readies it.
   */
  @Test
  void anEncoderItsLoopClosesOnLostHoldsItOnItsMotor() {
    JointHoming homing = byEncoder(true);
    homing.start(now);
    homing.seek(now, true, 0.5);

    homing.watch(now += LOOP, true, false, 0.62, 0.6);
    String fault = homing.fault();
    OptionalDouble stopped = homing.heldOnMotor();
    homing.watch(now += LOOP, false, false, 0.7, 0.68);
    OptionalDouble disabled = homing.heldOnMotor();
    homing.again(now, true, null);
    State again = homing.state();
    OptionalDouble waiting = homing.heldOnMotor();
    homing.seek(now += LOOP, true, 0.69);

    assertThat(fault)
        .isEqualTo(
            "pivot's absolute encoder is lost, on port 1: its loop closes on it, so it holds"
                + " where its motor says until the encoder answers");
    assertThat(stopped).hasValue(0.62);
    assertThat(disabled).hasValue(0.68);
    assertThat(again).isEqualTo(State.UNHOMED);
    assertThat(waiting).hasValue(0.68);
    assertThat(homing.state()).isEqualTo(State.READY);
    assertThat(homing.heldOnMotor()).isEmpty();
  }

  /**
   * Faulted for settings its motor controllers didn't take, its encoder's reading still tells its
   * motor where it is, and it stays faulted, homed again or not.
   */
  @Test
  void faultedOtherwiseItsEncoderStillSeedsIt() {
    JointHoming homing = byEncoder(false);
    homing.start(now);
    homing.faulted("pivot's motor 32 didn't take its settings");
    homing.seek(now, true, 0.3);
    boolean awaitingAfter = homing.awaiting();
    homing.again(now, true, null);
    homing.seek(now, true, 0.35);

    assertThat(found).containsExactly(0.3, 0.35);
    assertThat(awaitingAfter).isFalse();
    assertThat(homing.state()).isEqualTo(State.FAULTED);
    assertThat(homing.shown()).isEqualTo("FAULTED: pivot's motor 32 didn't take its settings");
  }
}

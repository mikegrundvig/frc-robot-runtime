package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;

/**
 * A joint's interlocks, a loop at a time: what they leave of what it's asked, from where it is,
 * what the other joints publish and what the sensors read. No simulator: the other joint is a view
 * a test sets, and the pivot brakes at its profile's 30 rad/s².
 */
class InterlocksTest {
  /** What another joint publishes, as a test sets it. */
  private static final class Other implements JointView {
    boolean answers = true;
    boolean knowsWhere = true;
    double position;
    double velocity;

    Other at(double position) {
      this.position = position;
      return this;
    }

    @Override
    public boolean answers() {
      return answers;
    }

    @Override
    public boolean knowsWhere() {
      return knowsWhere;
    }

    @Override
    public double position() {
      return position;
    }

    @Override
    public double velocity() {
      return velocity;
    }
  }

  private static Travel travel(Joint joint) {
    return Travel.of(ROBOT.math(), joint, ControllerSettings.control(joint));
  }

  private static double radians(double degrees) {
    return Math.toRadians(degrees);
  }

  private static Demand toward(double degrees) {
    return Demand.of(Request.POSITION, radians(degrees), 0.5);
  }

  private final Other pivotView = new Other();
  private final Other elevatorView = new Other();
  private final Other rollerView = new Other();

  /** The roller runs only with the pivot past 110°: stopped until then, left alone after. */
  @Test
  void aJointThatRunsOnlyWhenAnotherIsPastAPlaceStopsUntilItIs() {
    Interlocks roller = new Interlocks(travel(ROLLER), false);
    Interlock rule = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)));
    roller.add(rule, pivotView.at(radians(1.5)));
    Demand intaking = Demand.of(Request.VELOCITY, 50, 1);

    Demand stopped = roller.apply(intaking, 0, 0, 0, null);
    pivotView.at(radians(120));
    Demand running = roller.apply(intaking, 0, 0, 0, null);

    assertThat(stopped.kind()).isEqualTo(Request.NEUTRAL);
    assertThat(stopped.heldBy()).isEqualTo("roller runs only when pivot at least 110°");
    assertThat(running).isSameAs(intaking);
  }

  /** A joint that doesn't know where it is restricts: the roller stops with the pivot unhomed. */
  @Test
  void aJointThatDoesntKnowWhereItIsRestricts() {
    Interlocks roller = new Interlocks(travel(ROLLER), false);
    roller.add(
        Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110))),
        pivotView.at(radians(120)));
    pivotView.knowsWhere = false;

    assertThat(roller.apply(Demand.of(Request.VELOCITY, 50, 1), 0, 0, 0, null).kind())
        .isEqualTo(Request.NEUTRAL);
  }

  /**
   * The turret moves only with the roller up to speed, read from its motors whether or not it knows
   * where it is. Held, it holds where it comes to a stop, worked out once as the hold starts; let
   * go, its command's goal again, its profile from where it is.
   */
  @Test
  void aJointHeldAtPositionsHoldsWhereItStopsOnceAndRestartsWhenLetGo() {
    Interlocks turret = new Interlocks(travel(TURRET), true);
    turret.add(Interlock.that(TURRET).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(1000))), rollerView);
    rollerView.knowsWhere = false;
    Demand back = toward(180);

    Demand held = turret.apply(back, 1, 3, 0, null);
    Demand stillHeld = turret.apply(back, 1.1, 1, 0, null);
    rollerView.velocity = 1100.0 / 60 * 2 * Math.PI;
    Demand letGo = turret.apply(back, 1.2, 0, 0, null);
    Demand after = turret.apply(back, 1.2, 0, 0, null);

    assertThat(held.kind()).isEqualTo(Request.POSITION);
    assertThat(held.value())
        .as("1 rad, braking from 3 rad/s at 10 rad/s²")
        .isCloseTo(1.45, within(1e-12));
    assertThat(held.fraction()).isEqualTo(1);
    assertThat(held.heldBy()).isEqualTo("turret runs only when roller at least 1000 rpm");
    assertThat(stillHeld.value()).as("worked out once").isEqualTo(held.value());
    assertThat(letGo).isEqualTo(back.restarted());
    assertThat(after).isSameAs(back);
  }

  /** The pivot kept out of 40° to 80° while the elevator is below clear. */
  private Interlocks keptOut() {
    Interlocks pivot = new Interlocks(travel(PIVOT), true);
    pivot.add(
        Interlock.that(PIVOT).keepsOut(Range.degrees(40, 80)).when(ELEVATOR.atMost(ELEVATOR_CLEAR)),
        elevatorView.at(0));
    return pivot;
  }

  /**
   * Below the region, it's held at its lower edge, at its cruise; above, its upper; a target short
   * of it is left alone, at its command's share of the cruise; open, it's left alone.
   */
  @Test
  void aClosedRegionHoldsTheJointAtItsEdgeOnItsSide() {
    Interlocks pivot = keptOut();

    Demand fromBelow = pivot.apply(toward(130), radians(10), 0, 0, null);
    Demand fromAbove = pivot.apply(toward(1.5), radians(120), 0, 0, null);
    Demand short_ = pivot.apply(toward(30), radians(10), 0, 0, null);
    elevatorView.at(0.5);
    Demand open = pivot.apply(toward(130), radians(10), 0, 0, null);

    assertThat(fromBelow.value()).isCloseTo(radians(40), within(1e-12));
    assertThat(fromBelow.fraction()).isEqualTo(1);
    assertThat(fromBelow.heldBy())
        .isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(fromAbove.value()).isCloseTo(radians(80), within(1e-12));
    assertThat(short_).as("a target short of it isn't held").isEqualTo(toward(30));
    assertThat(open).isEqualTo(toward(130));
  }

  /**
   * Overshot into the region from below, it may leave the way it came, and otherwise holds where it
   * would stop as the hold starts, the point worked out once however fast it then goes: worked out
   * again each loop, it ran ahead of a joint its loop hadn't yet slowed, and the joint through the
   * region after it.
   */
  @Test
  void insideItMayLeaveTheWayItCameAndOtherwiseHoldsWhereItWouldStopOnce() {
    Interlocks pivot = keptOut();
    pivot.apply(toward(130), radians(35), 2, 0, null);

    Demand held = pivot.apply(toward(130), radians(45), 2, 0, null);
    Demand later = pivot.apply(toward(130), radians(47), 0.5, 0, null);
    Demand leaving = pivot.apply(toward(1.5), radians(47), 0, 0, null);

    double stops = radians(45) + 4 / 60.0;
    assertThat(held.value()).isCloseTo(stops, within(1e-12));
    assertThat(later.value()).as("worked out once").isEqualTo(held.value());
    assertThat(leaving.value()).isCloseTo(radians(1.5), within(1e-12));
    assertThat(leaving.heldBy()).isEmpty();
  }

  /**
   * A region reaching the end of its range leaves no room past that end: at its 135° stop, inside a
   * region of 100° to 135°, it leaves by the region's other edge, and is held going deeper. Read as
   * past the region once, it was held at its stop.
   */
  @Test
  void aJointAtItsEndInsideARegionReachingItLeavesByTheOtherEdge() {
    Interlocks pivot = new Interlocks(travel(PIVOT), true);
    pivot.add(
        Interlock.that(PIVOT)
            .keepsOut(Range.degrees(100, 135))
            .when(ELEVATOR.atMost(ELEVATOR_CLEAR)),
        elevatorView.at(0));

    Demand down = pivot.apply(toward(1.5), radians(135), 0, 0, null);
    Demand deeper = pivot.apply(toward(120), radians(135), 0, 0, null);

    assertThat(down.value()).isCloseTo(radians(1.5), within(1e-12));
    assertThat(down.heldBy()).isEmpty();
    assertThat(deeper.value()).isCloseTo(radians(135), within(1e-12));
    assertThat(deeper.heldBy()).isNotEmpty();
  }

  /**
   * Run at an output toward the region, it's held at the edge once where it would stop, held a loop
   * on, would reach the region; neutral, it's left alone. Chasing 2 rad/s at 2 rad/s, braking at 30
   * rad/s², it stops in 3.8°, and goes 2.3° in a loop.
   */
  @Test
  void anOutputIsHeldAtTheEdgeOnceItWouldStopInside() {
    Interlocks pivot = keptOut();
    Demand up = Demand.of(Request.VELOCITY, 2, 1);

    Demand farOff = pivot.apply(up, radians(20), 2, 0, null);
    Demand near = pivot.apply(up, radians(38), 2, 0, null);
    Demand neutral = pivot.apply(Demand.of(Request.NEUTRAL, 0, 1), radians(38), 2, 0, null);

    assertThat(farOff).isSameAs(up);
    assertThat(near.kind()).isEqualTo(Request.POSITION);
    assertThat(near.value()).isCloseTo(radians(40), within(1e-12));
    assertThat(near.heldBy()).isNotEmpty();
    assertThat(neutral.kind()).isEqualTo(Request.NEUTRAL);
  }

  /**
   * Held a loop before it could no longer stop short of the edge: at 34.5° it would stop at 38.3°
   * from now, but at 40.6° held a loop on, and 33° pushed on, speeding up at 25 rad/s², stops past
   * the edge held a loop on, though not at its speed alone. Held a loop late, it overshot.
   */
  @Test
  void anOutputIsHeldALoopBeforeItCouldNoLongerStopShort() {
    Demand up = Demand.of(Request.VELOCITY, 2, 1);

    Demand aLoopOn = keptOut().apply(up, radians(34.5), 2, 0, null);
    Demand steady = keptOut().apply(up, radians(33), 2, 0, null);
    Demand speedingUp = keptOut().apply(up, radians(33), 2, 25, null);
    Demand slowingDown = keptOut().apply(up, radians(33), 2, -25, null);

    assertThat(aLoopOn.kind()).isEqualTo(Request.POSITION);
    assertThat(aLoopOn.value()).isCloseTo(radians(40), within(1e-12));
    assertThat(steady).isSameAs(up);
    assertThat(speedingUp.kind()).isEqualTo(Request.POSITION);
    assertThat(slowingDown).as("slowing down never holds it later").isSameAs(up);
  }

  /**
   * At rest, it's held once where it would stop from the speed its output asks for, held a loop on,
   * would reach the region: a light joint's motors reach it within the loop. Asked for 2 rad/s, at
   * 33° it's left alone, and at 35° held; at 3 V, which runs it at about 6 rad/s free, it's held
   * from 20°. Until 2026-10-07 a joint at rest was held only once it was moving, by when a light
   * one could no longer stop short of the edge.
   */
  @Test
  void anOutputFromRestIsHeldOnceTheSpeedItAsksForWouldReach() {
    Demand up = Demand.of(Request.VELOCITY, 2, 1);
    Demand pushed = Demand.of(Request.VOLTAGE, 3, 1);

    Demand short_ = keptOut().apply(up, radians(33), 0, 0, null);
    Demand near = keptOut().apply(up, radians(35), 0, 0, null);
    Demand atVolts = keptOut().apply(pushed, radians(20), 0, 0, null);

    assertThat(short_).isSameAs(up);
    assertThat(near.kind()).isEqualTo(Request.POSITION);
    assertThat(near.value()).isCloseTo(radians(40), within(1e-12));
    assertThat(atVolts.kind()).isEqualTo(Request.POSITION);
  }

  /**
   * Held at the edge, it stays held there while its output pushes it toward the region, though at
   * rest it could stop short: let go then, pushed on, it would cross. Pushed away, it's let go;
   * pushed toward it again from just short, it's held again, at once.
   */
  @Test
  void anOutputHeldAtTheEdgeStaysHeldWhileItPushesToward() {
    Interlocks pivot = keptOut();
    Demand up = Demand.of(Request.VOLTAGE, 3, 1);
    Demand down = Demand.of(Request.VOLTAGE, -3, 1);
    pivot.apply(up, radians(38), 2, 0, null);

    Demand atRest = pivot.apply(up, radians(39.95), 0, 0, null);
    Demand away = pivot.apply(down, radians(39.95), 0, 0, null);
    Demand again = pivot.apply(up, radians(39.9), -0.1, 0, null);

    assertThat(atRest.kind()).isEqualTo(Request.POSITION);
    assertThat(atRest.value()).isCloseTo(radians(40), within(1e-12));
    assertThat(away).isSameAs(down);
    assertThat(again.kind()).isEqualTo(Request.POSITION);
    assertThat(again.value()).isCloseTo(radians(40), within(1e-12));
  }

  private static final Sensor.Switch CLAW_BEAM = Sensor.Switch.beam("claw beam", PIVOT);

  /** The beam's reading, as a test sets it. */
  private final SensorIO.SensorIOInputs beam = new SensorIO.SensorIOInputs();

  private Sensors sensors() {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT).sensors(List.of(CLAW_BEAM)).build();
    return new Sensors(
        robot,
        placed ->
            new SensorIO() {
              @Override
              public void updateInputs(SensorIOInputs inputs) {
                inputs.connected = beam.connected;
                inputs.tripped = beam.tripped;
              }
            },
        Scheduler.createIndependentScheduler());
  }

  /**
   * The roller runs only while the beam is clear, and with the beam lost it's held, as with it
   * tripped; held by two rules, it says both.
   */
  @Test
  void aSensorsConditionHoldsAndALostSensorRestricts() {
    Sensors sensors = sensors();
    Interlocks roller = new Interlocks(travel(ROLLER), false);
    roller.add(Interlock.that(ROLLER).runsOnlyWhen(CLAW_BEAM.clear()), null);
    roller.add(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110))), pivotView);
    Demand intaking = Demand.of(Request.VELOCITY, 50, 1);
    pivotView.at(radians(120));

    beam.connected = true;
    sensors.read();
    Demand clear = roller.apply(intaking, 0, 0, 0, sensors);
    beam.tripped = true;
    sensors.read();
    Demand tripped = roller.apply(intaking, 0, 0, 0, sensors);
    beam.tripped = false;
    beam.connected = false;
    pivotView.at(0);
    sensors.read();
    Demand lost = roller.apply(intaking, 0, 0, 0, sensors);

    assertThat(clear).isSameAs(intaking);
    assertThat(tripped.kind()).isEqualTo(Request.NEUTRAL);
    assertThat(tripped.heldBy()).isEqualTo("roller runs only when claw beam clear");
    assertThat(lost.heldBy())
        .isEqualTo(
            "roller runs only when claw beam clear; roller runs only when pivot at least 110°");
  }
}

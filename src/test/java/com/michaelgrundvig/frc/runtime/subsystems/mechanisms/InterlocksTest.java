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
import com.michaelgrundvig.frc.world.robot.JointAsk;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Travel;
import java.util.List;
import java.util.Optional;
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
    return Demands.toPosition(radians(degrees), 0.5);
  }

  private final Other pivotView = new Other();
  private final Other elevatorView = new Other();
  private final Other rollerView = new Other();

  /** The roller runs only with the pivot past 110°: stopped until then, left alone after. */
  @Test
  void aJointThatRunsOnlyWhenAnotherIsPastAPlaceStopsUntilItIs() {
    Interlocks roller = new Interlocks(travel(ROLLER));
    Interlock rule = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)));
    roller.add(rule, Optional.of(pivotView.at(radians(1.5))));
    Demand intaking = Demands.atVelocity(50);

    Demand stopped = roller.apply(intaking, 0, 0, 0, Demands.NO_SENSORS);
    pivotView.at(radians(120));
    Demand running = roller.apply(intaking, 0, 0, 0, Demands.NO_SENSORS);

    assertThat(stopped.ask()).isEqualTo(JointAsk.NEUTRAL);
    assertThat(stopped.heldBy()).isEqualTo("roller runs only when pivot at least 110°");
    assertThat(running).isSameAs(intaking);
  }

  /** A joint that doesn't know where it is restricts: the roller stops with the pivot unhomed. */
  @Test
  void aJointThatDoesntKnowWhereItIsRestricts() {
    Interlocks roller = new Interlocks(travel(ROLLER));
    roller.add(
        Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110))),
        Optional.of(pivotView.at(radians(120))));
    pivotView.knowsWhere = false;

    assertThat(roller.apply(Demands.atVelocity(50), 0, 0, 0, Demands.NO_SENSORS).ask())
        .isEqualTo(JointAsk.NEUTRAL);
  }

  /**
   * The turret moves only with the roller up to speed, read from its motors whether or not it knows
   * where it is. Held, it holds where it comes to a stop, worked out once as the hold starts; let
   * go, its command's goal again, its profile from where it is.
   */
  @Test
  void aJointHeldAtPositionsHoldsWhereItStopsOnceAndRestartsWhenLetGo() {
    Interlocks turret = new Interlocks(travel(TURRET));
    turret.add(
        Interlock.that(TURRET).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(1000))),
        Optional.of(rollerView));
    rollerView.knowsWhere = false;
    Demand back = toward(180);

    Demand held = turret.apply(back, 1, 3, 0, Demands.NO_SENSORS);
    Demand stillHeld = turret.apply(back, 1.1, 1, 0, Demands.NO_SENSORS);
    rollerView.velocity = 1100.0 / 60 * 2 * Math.PI;
    Demand letGo = turret.apply(back, 1.2, 0, 0, Demands.NO_SENSORS);
    Demand after = turret.apply(back, 1.2, 0, 0, Demands.NO_SENSORS);

    assertThat(held.ask()).isInstanceOf(JointAsk.ToPosition.class);
    assertThat(Demands.position(held))
        .as("1 rad, braking from 3 rad/s at 10 rad/s²")
        .isCloseTo(1.45, within(1e-12));
    assertThat(Demands.cruiseShare(held)).isEqualTo(1);
    assertThat(held.heldBy()).isEqualTo("turret runs only when roller at least 1000 rpm");
    assertThat(Demands.position(stillHeld)).as("worked out once").isEqualTo(Demands.position(held));
    assertThat(letGo).isEqualTo(back.restartingProfile());
    assertThat(after).isSameAs(back);
  }

  /** The pivot kept out of 40° to 80° while the elevator is below clear. */
  private Interlocks keptOut() {
    Interlocks pivot = new Interlocks(travel(PIVOT));
    pivot.add(
        Interlock.that(PIVOT).keepsOut(Range.degrees(40, 80)).when(ELEVATOR.atMost(ELEVATOR_CLEAR)),
        Optional.of(elevatorView.at(0)));
    return pivot;
  }

  /**
   * Below the region, it's held at its lower edge, at its cruise; above, its upper; a target short
   * of it is left alone, at its command's share of the cruise; open, it's left alone.
   */
  @Test
  void aClosedRegionHoldsTheJointAtItsEdgeOnItsSide() {
    Interlocks pivot = keptOut();

    Demand fromBelow = pivot.apply(toward(130), radians(10), 0, 0, Demands.NO_SENSORS);
    Demand fromAbove = pivot.apply(toward(1.5), radians(120), 0, 0, Demands.NO_SENSORS);
    Demand short_ = pivot.apply(toward(30), radians(10), 0, 0, Demands.NO_SENSORS);
    elevatorView.at(0.5);
    Demand open = pivot.apply(toward(130), radians(10), 0, 0, Demands.NO_SENSORS);

    assertThat(Demands.position(fromBelow)).isCloseTo(radians(40), within(1e-12));
    assertThat(Demands.cruiseShare(fromBelow)).isEqualTo(1);
    assertThat(fromBelow.heldBy())
        .isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(Demands.position(fromAbove)).isCloseTo(radians(80), within(1e-12));
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
    pivot.apply(toward(130), radians(35), 2, 0, Demands.NO_SENSORS);

    Demand held = pivot.apply(toward(130), radians(45), 2, 0, Demands.NO_SENSORS);
    Demand later = pivot.apply(toward(130), radians(47), 0.5, 0, Demands.NO_SENSORS);
    Demand leaving = pivot.apply(toward(1.5), radians(47), 0, 0, Demands.NO_SENSORS);

    double stops = radians(45) + 4 / 60.0;
    assertThat(Demands.position(held)).isCloseTo(stops, within(1e-12));
    assertThat(Demands.position(later)).as("worked out once").isEqualTo(Demands.position(held));
    assertThat(Demands.position(leaving)).isCloseTo(radians(1.5), within(1e-12));
    assertThat(leaving.heldBy()).isEmpty();
  }

  /**
   * A region reaching the end of its range leaves no room past that end: at its 135° stop, inside a
   * region of 100° to 135°, it leaves by the region's other edge, and is held going deeper. Read as
   * past the region once, it was held at its stop.
   */
  @Test
  void aJointAtItsEndInsideARegionReachingItLeavesByTheOtherEdge() {
    Interlocks pivot = new Interlocks(travel(PIVOT));
    pivot.add(
        Interlock.that(PIVOT)
            .keepsOut(Range.degrees(100, 135))
            .when(ELEVATOR.atMost(ELEVATOR_CLEAR)),
        Optional.of(elevatorView.at(0)));

    Demand down = pivot.apply(toward(1.5), radians(135), 0, 0, Demands.NO_SENSORS);
    Demand deeper = pivot.apply(toward(120), radians(135), 0, 0, Demands.NO_SENSORS);

    assertThat(Demands.position(down)).isCloseTo(radians(1.5), within(1e-12));
    assertThat(down.heldBy()).isEmpty();
    assertThat(Demands.position(deeper)).isCloseTo(radians(135), within(1e-12));
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
    Demand up = Demands.atVelocity(2);

    Demand farOff = pivot.apply(up, radians(20), 2, 0, Demands.NO_SENSORS);
    Demand near = pivot.apply(up, radians(38), 2, 0, Demands.NO_SENSORS);
    Demand neutral = pivot.apply(Demand.NEUTRAL, radians(38), 2, 0, Demands.NO_SENSORS);

    assertThat(farOff).isSameAs(up);
    assertThat(near.ask()).isInstanceOf(JointAsk.ToPosition.class);
    assertThat(Demands.position(near)).isCloseTo(radians(40), within(1e-12));
    assertThat(near.heldBy()).isNotEmpty();
    assertThat(neutral.ask()).isEqualTo(JointAsk.NEUTRAL);
  }

  /**
   * Held a loop before it could no longer stop short of the edge: at 34.5° it would stop at 38.3°
   * from now, but at 40.6° held a loop on, and 33° pushed on, speeding up at 25 rad/s², stops past
   * the edge held a loop on, though not at its speed alone. Held a loop late, it overshot.
   */
  @Test
  void anOutputIsHeldALoopBeforeItCouldNoLongerStopShort() {
    Demand up = Demands.atVelocity(2);

    Demand aLoopOn = keptOut().apply(up, radians(34.5), 2, 0, Demands.NO_SENSORS);
    Demand steady = keptOut().apply(up, radians(33), 2, 0, Demands.NO_SENSORS);
    Demand speedingUp = keptOut().apply(up, radians(33), 2, 25, Demands.NO_SENSORS);
    Demand slowingDown = keptOut().apply(up, radians(33), 2, -25, Demands.NO_SENSORS);

    assertThat(aLoopOn.ask()).isInstanceOf(JointAsk.ToPosition.class);
    assertThat(Demands.position(aLoopOn)).isCloseTo(radians(40), within(1e-12));
    assertThat(steady).isSameAs(up);
    assertThat(speedingUp.ask()).isInstanceOf(JointAsk.ToPosition.class);
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
    Demand up = Demands.atVelocity(2);
    Demand pushed = Demands.atVoltage(3);

    Demand short_ = keptOut().apply(up, radians(33), 0, 0, Demands.NO_SENSORS);
    Demand near = keptOut().apply(up, radians(35), 0, 0, Demands.NO_SENSORS);
    Demand atVolts = keptOut().apply(pushed, radians(20), 0, 0, Demands.NO_SENSORS);

    assertThat(short_).isSameAs(up);
    assertThat(near.ask()).isInstanceOf(JointAsk.ToPosition.class);
    assertThat(Demands.position(near)).isCloseTo(radians(40), within(1e-12));
    assertThat(atVolts.ask()).isInstanceOf(JointAsk.ToPosition.class);
  }

  /**
   * Held at the edge, it stays held there while its output pushes it toward the region, though at
   * rest it could stop short: let go then, pushed on, it would cross. Pushed away, it's let go;
   * pushed toward it again from just short, it's held again, at once.
   */
  @Test
  void anOutputHeldAtTheEdgeStaysHeldWhileItPushesToward() {
    Interlocks pivot = keptOut();
    Demand up = Demands.atVoltage(3);
    Demand down = Demands.atVoltage(-3);
    pivot.apply(up, radians(38), 2, 0, Demands.NO_SENSORS);

    Demand atRest = pivot.apply(up, radians(39.95), 0, 0, Demands.NO_SENSORS);
    Demand away = pivot.apply(down, radians(39.95), 0, 0, Demands.NO_SENSORS);
    Demand again = pivot.apply(up, radians(39.9), -0.1, 0, Demands.NO_SENSORS);

    assertThat(atRest.ask()).isInstanceOf(JointAsk.ToPosition.class);
    assertThat(Demands.position(atRest)).isCloseTo(radians(40), within(1e-12));
    assertThat(away).isSameAs(down);
    assertThat(again.ask()).isInstanceOf(JointAsk.ToPosition.class);
    assertThat(Demands.position(again)).isCloseTo(radians(40), within(1e-12));
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
    Interlocks roller = new Interlocks(travel(ROLLER));
    roller.add(Interlock.that(ROLLER).runsOnlyWhen(CLAW_BEAM.clear()), Optional.empty());
    roller.add(
        Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110))),
        Optional.of(pivotView));
    Demand intaking = Demands.atVelocity(50);
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
    assertThat(tripped.ask()).isEqualTo(JointAsk.NEUTRAL);
    assertThat(tripped.heldBy()).isEqualTo("roller runs only when claw beam clear");
    assertThat(lost.heldBy())
        .isEqualTo(
            "roller runs only when claw beam clear; roller runs only when pivot at least 110°");
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_FRONT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.record.MoveReason;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A turret whose range spans more than a turn, -200° to 200°, so facing back it can be at 180° or
 * at -180°. A computed heading goes to whichever turn of it inside its limits is nearest where the
 * turret is; a named position goes exactly where it's named. A joint narrower than a turn takes an
 * angle as it is.
 */
@ExtendWith(RobotSim.class)
class TurretTurnsSimTest {
  /** Facing back, named a whole turn round from 180°. */
  private static final Position BACK = Position.degrees("back", -180);

  private static final Joint.Revolute TURRET =
      JointRevoluteBuilder.builder(JointFixtures.TURRET)
          .range(Range.degrees(-200, 200))
          .positions(List.of(TURRET_FRONT, BACK))
          .build();

  /** The pivot, narrower than a turn, beside the turret. */
  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .joints(List.of(PIVOT, TURRET))
          .configurations(List.of())
          .build();

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final JointMechanism turret = sim.joints.of(TURRET);

  /** Schedules a move and runs loops until it ends, at most {@code seconds}: how long it took. */
  private static double runToEnd(JointsSimHarness sim, Move move, double seconds) {
    sim.scheduler.schedule(move);
    return sim.runUntil(() -> move.outcome().isPresent(), seconds);
  }

  private static double degrees(JointsSimHarness sim) {
    return Math.toDegrees(sim.simulation.position(TURRET));
  }

  /** At 170°, -170° is 20° on, at 190°, inside its 200°: not 340° back the other way. */
  @Test
  void aHeadingGoesToItsTurnNearestWhereTheTurretIs() {
    sim.simulation.place(TURRET, Math.toRadians(170));
    Move on = turret.goTo(Degrees.of(-170));

    double took = runToEnd(sim, on, 3);

    assertThat(on.arrived()).isTrue();
    assertThat(took).isLessThan(0.6);
    assertThat(degrees(sim)).isCloseTo(190, within(2.0));
    assertThat(Math.toDegrees(turret.goal())).isCloseTo(190, within(1e-9));
  }

  /**
   * At 170°, asked for 210°, past its 200° stop: the same heading as -150°, which it can reach only
   * the long way round, 320°, so it goes the long way.
   */
  @Test
  void aHeadingReachableOnlyTheLongWayGoesTheLongWay() {
    sim.simulation.place(TURRET, Math.toRadians(170));
    Move round = turret.goTo(Degrees.of(210));

    double took = runToEnd(sim, round, 5);

    assertThat(round.arrived()).isTrue();
    assertThat(took).as("320° at its 3 rad/s cruise").isGreaterThan(1.8);
    assertThat(degrees(sim)).isCloseTo(-150, within(2.0));
  }

  /**
   * At 170°, the heading -180° is 10° on, at 180°. "back", named at -180°, is where it was named, a
   * whole turn round from there: a team named that very angle.
   */
  @Test
  void aNamedPositionIsExact() {
    sim.simulation.place(TURRET, Math.toRadians(170));
    Move heading = turret.goTo(Degrees.of(-180));
    Move back = turret.goTo(BACK);

    runToEnd(sim, heading, 3);
    double first = degrees(sim);
    runToEnd(sim, back, 5);

    assertThat(heading.arrived()).isTrue();
    assertThat(first).isCloseTo(180, within(2.0));
    assertThat(back.arrived()).isTrue();
    assertThat(degrees(sim)).isCloseTo(-180, within(2.0));
  }

  /** Chasing a heading, it takes the turn of it nearest where it is, as a move does. */
  @Test
  void followChasesAHeadingAtItsNearestTurn() {
    sim.simulation.place(TURRET, Math.toRadians(170));
    double[] heading = {-170};
    Move follow = turret.follow(() -> Degrees.of(heading[0]));

    sim.run(follow, 1);
    double first = degrees(sim);
    heading[0] = -150; // only the long way round, past its 200° stop
    sim.run(3);

    assertThat(first).isCloseTo(190, within(1.0));
    assertThat(degrees(sim)).isCloseTo(-150, within(1.0));
    assertThat(follow.outcome()).as("it never ends on its own").isEmpty();
  }

  /**
   * Soft limits narrower than a turn, -190° to 160°, leave some headings with no turn inside them.
   * 200° has one, -160°, and goes there. 165° has none: a move to it is refused, saying so; a chase
   * of 163° holds at the limit nearer that heading either way round, 160°, 3° off it, though -190°
   * is nearer where the turret is.
   */
  @Test
  void softLimitsNarrowerThanATurnLeaveSomeHeadingsOutside() {
    JointsSimHarness limited =
        new JointsSimHarness(
            ROBOT.with(
                TURRET.retuned(control -> control.withSoftLimits(Range.degrees(-190, 160)))));
    JointMechanism soft = limited.joints.of(TURRET);
    Move wrapped = soft.goTo(Degrees.of(200));
    Move outside = soft.goTo(Degrees.of(165));

    runToEnd(limited, wrapped, 3);
    double first = degrees(limited);
    runToEnd(limited, outside, 1);
    limited.run(soft.follow(() -> Degrees.of(163)), 3);

    assertThat(wrapped.arrived()).isTrue();
    assertThat(first).isCloseTo(-160, within(2.0));
    assertThat(outside.outcome()).contains(Outcome.REFUSED);
    ended(outside)
        .named("turret → 165°")
        .refused()
        .because(MoveReason.TARGET_UNREACHABLE)
        .saying("165°, at every turn of it, is outside turret's limits, -190° to 160°");
    assertThat(degrees(limited)).isCloseTo(160, within(2.0));
  }

  /**
   * An angle far too big for a double to tell its turns apart, a units slip gone wrong, still ends
   * the search for its turn, a move on it ends, and the turret stays inside its limits.
   */
  @Test
  void anAngleTooBigToTurnStillEnds() {
    Move huge = turret.goTo(Radians.of(1e300));

    runToEnd(sim, huge, 1);
    double moved = Math.toDegrees(turret.goal());
    sim.run(turret.follow(() -> Radians.of(-1e300)), 0.1);

    assertThat(huge.outcome()).isPresent();
    assertThat(moved).isBetween(-200.0, 200.0);
    assertThat(Math.toDegrees(turret.goal())).isBetween(-200.0, 200.0);
  }

  /**
   * The pivot's range, 0° to 135°, is narrower than a turn: 370° is refused as it is, though 10° is
   * the same angle. Outside a joint that can face every way, an angle off by whole turns is a slip,
   * degrees given as radians, more often than a heading.
   */
  @Test
  void aJointNarrowerThanATurnTakesItsAngleAsItIs() {
    Move far = sim.joints.of(PIVOT).goTo(Degrees.of(370));

    runToEnd(sim, far, 1);

    assertThat(far.outcome()).contains(Outcome.REFUSED);
    ended(far)
        .named("pivot → 370°")
        .refused()
        .because(MoveReason.TARGET_UNREACHABLE)
        .saying("370° is outside pivot's limits, 0° to 135°");
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_DOWN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_PUSH;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.simulation.AlertSim;

/**
 * The definition's interlocks, beneath every command: the roller runs only with the pivot past
 * "run", and the pivot keeps out of 40° to 80° while the elevator is below "clear".
 */
@ExtendWith(RobotSim.class)
class InterlocksSimTest {
  static final Interlock ROLLER_ONLY_WHEN_OUT =
      Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN));

  static final Interlock PIVOT_CLEARS_ELEVATOR =
      Interlock.that(PIVOT).keepsOut(Range.degrees(40, 80)).when(ELEVATOR.atMost(ELEVATOR_CLEAR));

  /** The pivot out and the elevator clear: the order between them is the interlock's. */
  static final Configuration DEPLOYED =
      Configuration.of("deployed", PIVOT.at(PIVOT_OUT), ELEVATOR.at(ELEVATOR_CLEAR));

  static final RobotDefinition GUARDED =
      RobotDefinitionBuilder.builder(ROBOT)
          .name("guarded")
          .configurations(List.of(DEPLOYED))
          .interlocks(List.of(ROLLER_ONLY_WHEN_OUT, PIVOT_CLEARS_ELEVATOR))
          .build();

  private final JointsSimHarness sim = new JointsSimHarness(GUARDED);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);

  private double pivotDegrees() {
    return pivotDegrees(sim);
  }

  private static double pivotDegrees(JointsSimHarness on) {
    return Math.toDegrees(on.simulation.position(PIVOT));
  }

  /**
   * Asked to run with the pivot stowed, the roller stays still, and says what holds it; once
   * another command deploys the pivot past "run", it runs, its command never having stopped.
   */
  @Test
  void theRollerWaitsForThePivotToPassRun() {
    Move intaking = roller.run(ROLLER_INTAKING);
    sim.run(intaking, 0.5);
    double waiting = sim.simulation.velocity(ROLLER);
    String heldBy = roller.heldBy();

    sim.run(sim.joints.goTo(DEPLOYED), 2);

    assertThat(waiting).isCloseTo(0, within(0.5));
    assertThat(heldBy).isEqualTo("roller runs only when pivot at least run");
    assertThat(intaking.outcome()).as("still running").isEmpty();
    assertThat(roller.heldBy()).isEmpty();
    assertThat(sim.simulation.velocity(ROLLER))
        .isCloseTo(RPM.of(1500).baseUnitMagnitude(), within(0.03 * 157.08));
  }

  @Test
  void theRuleHoldsWhicheverCommandAsks() {
    sim.run(roller.follow(() -> RPM.of(1000)), 0.5);

    assertThat(sim.simulation.velocity(ROLLER)).isCloseTo(0, within(0.5));
    assertThat(roller.heldBy()).isEqualTo("roller runs only when pivot at least run");
  }

  /**
   * Both together: the pivot swings out to 40° and waits there while the elevator rises, then goes
   * on once it's clear. No planner: the interlock orders them.
   */
  @Test
  void aConfigurationMovesInTheOrderTheInterlocksAllow() {
    Move deploy = sim.joints.goTo(DEPLOYED);
    sim.scheduler.schedule(deploy);
    double[] furthestWhileLow = {0};

    double took =
        sim.runUntil(
            () -> {
              if (sim.simulation.position(ELEVATOR) < 0.4) {
                furthestWhileLow[0] = Math.max(furthestWhileLow[0], pivotDegrees());
              }
              return deploy.outcome().isPresent();
            },
            3);

    assertThat(deploy.arrived()).isTrue();
    assertThat(took).isLessThan(1.6);
    assertThat(furthestWhileLow[0])
        .as("short of the region while the elevator was low")
        .isLessThan(41);
    assertThat(furthestWhileLow[0]).as("waiting at its edge").isGreaterThan(38);
    assertThat(pivotDegrees()).isCloseTo(130, within(2.0));
  }

  @Test
  void heldPastItsDeadlineAMoveTimesOutAndSaysWhatHeldIt() {
    Move out = pivot.goTo(PIVOT_OUT);
    sim.scheduler.schedule(out);

    sim.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(out.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(out.reason())
        .startsWith("pivot → out: TIMED_OUT after 1.6")
        .endsWith(", held: pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(pivotDegrees()).isCloseTo(40, within(1.5));
  }

  /**
   * A closed region that leaves a move's target alone leaves its share of the cruise too: the
   * pivot, below the closed region, sent to 30° at a fifth of its cruise, goes no faster than with
   * no region at all. Until 2026-10-07 any closed region on the joint put it at full cruise.
   */
  @Test
  void aClosedRegionThatLeavesTheTargetAloneLeavesItsShareOfTheCruise() {
    JointsSimHarness free = new JointsSimHarness(ROBOT);
    double unguarded =
        fastest(free, free.joints.of(PIVOT).goTo(Degrees.of(30)).atSpeed(Percent.of(20)), 1);

    double guarded = fastest(sim, pivot.goTo(Degrees.of(30)).atSpeed(Percent.of(20)), 1);

    assertThat(unguarded).as("a fifth of its 8 rad/s").isCloseTo(1.6, within(0.1));
    assertThat(guarded).isCloseTo(unguarded, within(1e-9));
    assertThat(pivot.heldBy()).isEmpty();
    assertThat(pivotDegrees()).isCloseTo(30, within(1.0));
  }

  /** Runs a command on a harness for a time: the pivot's fastest meanwhile, in rad/s. */
  private static double fastest(JointsSimHarness on, Command command, double seconds) {
    on.scheduler.schedule(command);
    double fastest = 0;
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      on.loop();
      fastest = Math.max(fastest, Math.abs(on.simulation.velocity(PIVOT)));
    }
    return fastest;
  }

  /** Inside the region when it closes, the pivot may leave it, but goes no deeper. */
  @Test
  void insideTheClosedRegionItMayLeaveButNotGoDeeper() {
    sim.simulation.place(PIVOT, Math.toRadians(60));
    sim.loop();
    Move deeper = pivot.goTo(Degrees.of(75)).within(Seconds.of(0.5));
    sim.scheduler.schedule(deeper);
    sim.runUntil(() -> deeper.outcome().isPresent(), 2);
    double afterDeeper = pivotDegrees();

    Move in = pivot.goTo(PIVOT_IN);
    sim.scheduler.schedule(in);
    sim.runUntil(() -> in.outcome().isPresent(), 2);

    assertThat(deeper.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(afterDeeper).isCloseTo(60, within(1.5));
    assertThat(in.arrived()).isTrue();
  }

  /**
   * A joint that overshoots a region's edge never goes on across it. Its profile asking more
   * braking than its motor has, the pivot runs 19° past 40° into the region; the interlock had it
   * heading for the edge, and now inside, its goal across the region is refused: it may leave only
   * the way it came. Until 2026-10-05 a joint inside could head for any goal outside, and a team's
   * arm crossed the arc its interlock guards once it overshot the edge.
   */
  @Test
  void aJointThatOvershootsTheEdgeNeverGoesOnAcross() {
    JointsSimHarness swinging =
        new JointsSimHarness(
            GUARDED.with(
                PIVOT.retuned(old -> JointFixtures.assumed(control(20, 0, profile(12, 200))))));
    JointMechanism swung = swinging.joints.of(PIVOT);
    Move out = swung.goTo(PIVOT_OUT);
    swinging.scheduler.schedule(out);
    double furthest = 0;
    for (int i = 0; i < 100; i++) {
      swinging.loop();
      furthest = Math.max(furthest, Math.toDegrees(swinging.simulation.position(PIVOT)));
    }

    assertThat(furthest).as("it overshot the edge into the region").isGreaterThan(41);
    assertThat(furthest).as("and never crossed it").isLessThan(80);
    assertThat(out.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(out.reason())
        .endsWith("held: pivot keeps out of 40° to 80° while elevator at most clear");
  }

  /**
   * A pair kept out of each other's way, as an arm and its elevator are: the pivot out of 40° to
   * 80° while the elevator is below "clear", and the elevator out of its lowest 0.3 m while the
   * pivot is at 90° or beyond. The definition takes it, since neither holds the other for good, and
   * each rule holds as it says: the elevator, sent down with the pivot out, waits at 0.3 m; the
   * pivot, sent in with the elevator that low, waits at 80°, and short of 90°, lets the elevator
   * down.
   */
  @Test
  void aPairKeptOutOfEachOthersWayHoldsEachAndNeverDeadlocks() {
    Interlock elevatorUpWhilePivotOut =
        Interlock.that(ELEVATOR).keepsOut(Range.meters(0, 0.3)).when(PIVOT.atLeast(Degrees.of(90)));
    RobotDefinition pair =
        RobotDefinitionBuilder.builder(GUARDED)
            .name("pair")
            .interlocks(List.of(PIVOT_CLEARS_ELEVATOR, elevatorUpWhilePivotOut))
            .build();
    JointsSimHarness both = new JointsSimHarness(pair);
    JointMechanism elevator = both.joints.of(ELEVATOR);
    JointMechanism held = both.joints.of(PIVOT);
    Move deploy = both.joints.goTo(DEPLOYED);
    Move down = elevator.goTo(ELEVATOR_DOWN);
    Move in = held.goTo(PIVOT_IN);
    Move downAgain = elevator.goTo(ELEVATOR_DOWN);

    for (Move move : List.of(deploy, down, in, downAgain)) {
      both.scheduler.schedule(move);
      both.runUntil(() -> move.outcome().isPresent(), 3);
    }

    assertThat(deploy.arrived()).isTrue();
    assertThat(down.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(down.reason())
        .endsWith("held: elevator keeps out of 0 m to 0.3 m while pivot at least 90°");
    assertThat(in.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(in.reason())
        .endsWith("held: pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(Math.toDegrees(both.simulation.position(PIVOT))).isCloseTo(80, within(2.0));
    assertThat(downAgain.arrived()).as("the pivot's hold let the elevator go").isTrue();
    assertThat(both.simulation.position(ELEVATOR)).isCloseTo(0, within(0.02));
  }

  /**
   * The same rules written in the restricted joints' blocks hold the same: the roller, asked to run
   * with the pivot stowed, stays still and says what holds it.
   */
  @Test
  void rulesInTheBlocksHoldAsTheRobotsDo() {
    RobotDefinition inBlocks =
        RobotDefinitionBuilder.builder(ROBOT)
            .name("guarded in blocks")
            .configurations(List.of(DEPLOYED))
            .build()
            .with(
                ROLLER.runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN)),
                PIVOT.keepsOut(Range.degrees(40, 80)).when(ELEVATOR.atMost(ELEVATOR_CLEAR)));
    JointsSimHarness blocks = new JointsSimHarness(inBlocks);
    JointMechanism held = blocks.joints.of(ROLLER);

    blocks.run(held.run(ROLLER_INTAKING), 0.5);

    assertThat(blocks.simulation.velocity(ROLLER)).isCloseTo(0, within(0.5));
    assertThat(held.heldBy()).isEqualTo("roller runs only when pivot at least run");
  }

  /**
   * A joint held until another is up to speed: the pivot, asked out, waits where it is while the
   * roller is still, and goes once the roller runs at its 1500 rpm, past the 1000 it waits for.
   */
  @Test
  void aSpeedHoldsAJointUntilAnotherIsUpToIt() {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .name("spun up")
            .configurations(List.of())
            .interlocks(List.of(Interlock.that(PIVOT).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(1000)))))
            .build();
    JointsSimHarness spun = new JointsSimHarness(robot);
    JointMechanism held = spun.joints.of(PIVOT);
    Move out = held.goTo(PIVOT_OUT);
    spun.scheduler.schedule(out);

    spun.run(0.5);
    double waiting = Math.toDegrees(spun.simulation.position(PIVOT));
    String heldBy = held.heldBy();
    spun.scheduler.schedule(spun.joints.of(ROLLER).run(ROLLER_INTAKING));
    spun.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(waiting).isCloseTo(1.5, within(1.0));
    assertThat(heldBy).startsWith("pivot runs only when roller at least 1000");
    assertThat(out.arrived()).isTrue();
    assertThat(Math.toDegrees(spun.simulation.position(PIVOT))).isCloseTo(130, within(2.0));
  }

  /** The elevator, that runs only with the pivot out past "run". */
  static final RobotDefinition LIFT_WAITS =
      RobotDefinitionBuilder.builder(ROBOT)
          .name("lift waits")
          .configurations(List.of())
          .interlocks(List.of(Interlock.that(ELEVATOR).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN))))
          .build();

  /**
   * The elevator, jammed on its way up at 0.3 m while its profile runs on to the top, is held as
   * the pivot swings in past "run": held where it stops, where it's stuck. Freed, it stays there:
   * its profile brakes from where the joint is, not from where it had it, 0.8 m higher. Until
   * 2026-10-08 the profile came back down from the top, and the freed elevator rose to meet it,
   * 0.37 m past where it was held.
   */
  @Test
  void aRunsOnlyWhenHoldBrakesFromWhereTheJointIsNotItsProfile() {
    JointsSimHarness lagging = new JointsSimHarness(LIFT_WAITS);
    JointMechanism lift = lagging.joints.of(ELEVATOR);
    lagging.simulation.place(PIVOT, Math.toRadians(130));
    lagging.loop();
    lagging.scheduler.schedule(lift.goTo(JointFixtures.ELEVATOR_TOP));
    lagging.runUntil(() -> lagging.simulation.position(ELEVATOR) >= 0.3, 2);
    lagging.simulation.jam(ELEVATOR, true);
    lagging.run(1);

    lagging.scheduler.schedule(lagging.joints.of(PIVOT).goTo(PIVOT_IN));
    lagging.runUntil(() -> !lift.heldBy().isEmpty(), 1);
    double held = lift.goal();
    lagging.simulation.jam(ELEVATOR, false);
    double highest = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < RobotLoop.loops(1); i++) {
      lagging.loop();
      highest = Math.max(highest, lagging.simulation.position(ELEVATOR));
    }

    assertThat(held).as("where it's stuck").isCloseTo(0.3, within(0.03));
    assertThat(lift.heldBy()).isEqualTo("elevator runs only when pivot at least run");
    assertThat(highest).as("no further than where it's held").isCloseTo(held, within(0.01));
  }

  /**
   * The pivot, jammed at 65° on its way in while the region's open, its profile running on to "in",
   * is held where it's stuck, inside the region, as the elevator comes down and closes it. Freed,
   * it stays there: its profile brakes from where the joint is, not from where it had it, 63°
   * lower. Until 2026-10-08 the profile came back up from "in", and the freed pivot fell to meet
   * it, on through the closed region and out its far side.
   */
  @Test
  void aJointHeldInsideAClosedRegionBrakesFromWhereItIsNotItsProfile() {
    sim.simulation.place(PIVOT, Math.toRadians(130));
    sim.simulation.place(ELEVATOR, 0.45);
    sim.loop();
    sim.scheduler.schedule(pivot.goTo(PIVOT_IN));
    sim.runUntil(() -> pivotDegrees() <= 70, 2);
    sim.simulation.jam(PIVOT, true);
    sim.run(0.3);

    sim.scheduler.schedule(sim.joints.of(ELEVATOR).goTo(ELEVATOR_DOWN));
    sim.runUntil(() -> !pivot.heldBy().isEmpty(), 2);
    double held = Math.toDegrees(pivot.goal());
    String heldBy = pivot.heldBy();
    sim.simulation.jam(PIVOT, false);
    double lowest = Double.POSITIVE_INFINITY;
    for (int i = 0; i < RobotLoop.loops(1); i++) {
      sim.loop();
      lowest = Math.min(lowest, pivotDegrees());
    }

    assertThat(held).as("inside, where it's stuck").isBetween(40.0, 80.0);
    assertThat(heldBy).isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(lowest).as("no deeper than where it's held").isCloseTo(held, within(1.0));
  }

  /**
   * A region reaching the end of the joint's range leaves no room past that end: a pivot sitting at
   * its 135° stop, inside a region of 100° to 135° closed while the elevator is low, is inside it,
   * and leaves by the region's other edge, down to where it was sent. Until 2026-10-07 it read as
   * past the region and was held at its stop.
   */
  @Test
  void aJointAtItsEndInsideARegionReachingItLeavesByTheOtherEdge() {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .name("end region")
            .configurations(List.of())
            .interlocks(
                List.of(
                    Interlock.that(PIVOT)
                        .keepsOut(Range.degrees(100, 135))
                        .when(ELEVATOR.atMost(ELEVATOR_CLEAR))))
            .build();
    JointsSimHarness atEnd = new JointsSimHarness(robot);
    JointMechanism held = atEnd.joints.of(PIVOT);
    atEnd.simulation.place(PIVOT, Math.toRadians(135));
    atEnd.run(0.1);
    Move down = held.goTo(PIVOT_IN);
    atEnd.scheduler.schedule(down);

    atEnd.runUntil(() -> down.outcome().isPresent(), 3);

    assertThat(down.arrived()).isTrue();
    assertThat(Math.toDegrees(atEnd.simulation.position(PIVOT))).isCloseTo(1.5, within(2.0));
  }

  /** Runs a command on a harness for a time: the furthest a joint got meanwhile, in degrees. */
  private static double furthest(
      JointsSimHarness on, Joint joint, Command command, double seconds) {
    on.scheduler.schedule(command);
    double furthest = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      on.loop();
      furthest = Math.max(furthest, Math.toDegrees(on.simulation.position(joint)));
    }
    return furthest;
  }

  /**
   * Run at an output toward a closed region, the pivot is held at the region's edge, and says what
   * holds it, while it pushes on; raised clear, the elevator opens the region and the pivot runs on
   * through it. At a voltage, its own 3 V push: held from the loop before it could no longer stop
   * short of the edge, its profile brakes it there. Until 2026-10-07 the hold started once where it
   * would stop from where it was reached the edge, a loop's travel late, and the pivot ran 5° into
   * the region, to 45.3°, braking at its profile's 30 rad/s² as it does here.
   */
  @Test
  void aJointPushedAtAVoltageIsHeldAtTheRegionsEdgeUntilItOpens() {
    double held = furthest(sim, PIVOT, pivot.run(PIVOT_PUSH), 1);
    double stays = pivotDegrees();
    String heldBy = pivot.heldBy();
    sim.scheduler.schedule(sim.joints.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    sim.run(1.5);

    assertThat(held).as("at the region's edge").isCloseTo(40, within(0.1));
    assertThat(stays).as("and held there, pushing on").isCloseTo(40, within(0.1));
    assertThat(heldBy).isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(pivotDegrees()).as("through the open region").isGreaterThan(80);
    assertThat(pivot.heldBy()).isEmpty();
  }

  /** The same chasing a speed, on its velocity loop: at the edge. */
  @Test
  void aJointChasingASpeedIsHeldAtTheRegionsEdgeUntilItOpens() {
    double held = furthest(sim, PIVOT, pivot.follow(() -> DegreesPerSecond.of(90)), 1.5);
    String heldBy = pivot.heldBy();
    sim.scheduler.schedule(sim.joints.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    sim.run(1.5);

    assertThat(held).as("at the region's edge").isCloseTo(40, within(0.1));
    assertThat(heldBy).isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(pivotDegrees()).as("through the open region").isGreaterThan(80);
  }

  /** The pivot with a share of the battery to run at: a quarter of it, up. */
  static final Output PIVOT_NUDGE = Output.dutyCycle("nudge", 0.25);

  static final Joint.Revolute NUDGED =
      JointRevoluteBuilder.builder(PIVOT).outputs(List.of(PIVOT_PUSH, PIVOT_NUDGE)).build();

  /**
   * The same at a duty cycle, a pivot of its own with one, kept out of the same region: at the
   * edge. Until 2026-10-07, 2.4° into the region, at 42.4°, as a voltage's was.
   */
  @Test
  void aJointRunAtADutyCycleIsHeldAtTheRegionsEdgeUntilItOpens() {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .name("nudged")
            .joints(List.of(NUDGED, ELEVATOR))
            .configurations(List.of())
            .interlocks(
                List.of(
                    Interlock.that(NUDGED)
                        .keepsOut(Range.degrees(40, 80))
                        .when(ELEVATOR.atMost(ELEVATOR_CLEAR))))
            .build();
    JointsSimHarness nudged = new JointsSimHarness(robot);
    JointMechanism held = nudged.joints.of(NUDGED);

    double furthest = furthest(nudged, NUDGED, held.run(PIVOT_NUDGE), 1);
    String heldBy = held.heldBy();
    nudged.scheduler.schedule(nudged.joints.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    nudged.run(1.5);

    assertThat(furthest).as("at the region's edge").isCloseTo(40, within(0.1));
    assertThat(heldBy).isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(Math.toDegrees(nudged.simulation.position(NUDGED)))
        .as("through the open region")
        .isGreaterThan(80);
  }

  /**
   * A light, strong joint chasing a speed from rest just short of the closed region, its motors
   * reaching the speed within a loop: held at the edge from the start, since from that speed its
   * profile couldn't stop it short. Until 2026-10-07 the hold waited for the joint to be moving, by
   * when its profile couldn't stop it at the edge, and the pivot ran 19° into the region, to 59.2°;
   * the guide's arm, chasing a speed up from its stop below the bumper's band, 12° into it.
   */
  @Test
  void aJointThatReachesItsSpeedWithinALoopIsHeldAtTheEdgeFromRest() {
    JointsSimHarness strong =
        new JointsSimHarness(
            GUARDED.with(
                PIVOT.retuned(
                    old ->
                        JointFixtures.assumed(
                            control(30, 1, Optional.empty(), profile(8, 30), 120)))));
    strong.simulation.place(PIVOT, Math.toRadians(35));
    strong.loop();
    JointMechanism chasing = strong.joints.of(PIVOT);

    double furthest = furthest(strong, PIVOT, chasing.follow(() -> RadiansPerSecond.of(5)), 1);

    assertThat(furthest).as("at the region's edge").isCloseTo(40, within(0.5));
    assertThat(chasing.heldBy())
        .isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
  }

  /**
   * Above the closed region, pushed away from it, the pivot runs on to its stop, held by nothing.
   */
  @Test
  void anOutputAwayFromTheRegionIsApplied() {
    sim.simulation.place(PIVOT, Math.toRadians(100));
    sim.loop();

    double furthest = furthest(sim, PIVOT, pivot.run(PIVOT_PUSH), 1);

    assertThat(furthest).as("up to its stop").isGreaterThan(130);
    assertThat(pivot.heldBy()).isEmpty();
  }

  /** A harness on a robot whose joints' IO a test can silence, each by its joint. */
  private static JointsSimHarness silenceable(RobotDefinition robot, Map<Joint, Silenceable> ios) {
    return new JointsSimHarness(
        robot,
        (joint, io) -> {
          Silenceable each = new Silenceable(io);
          ios.put(joint, each);
          return each;
        });
  }

  /** A joint's silenceable IO. */
  private static Silenceable io(Map<Joint, Silenceable> ios, Joint joint) {
    return Objects.requireNonNull(ios.get(joint), joint.label());
  }

  /** Runs a move to its end, at most for a time. */
  private static Move run(JointsSimHarness on, Move move, double seconds) {
    on.scheduler.schedule(move);
    on.runUntil(() -> move.outcome().isPresent(), seconds);
    return move;
  }

  /**
   * A target that isn't a number, as a computed one divided by zero gives, is no request: for its
   * loop the pivot idles, beneath its interlocks as ever, its motor controller told nothing that
   * isn't a number, and it's logged with who asked, an alert raised. Chasing 130°, held at the
   * closed region's edge, then not a number for a loop, then 130° again, it's held at the edge
   * throughout; the next target, 30°, it goes to. Until 2026-10-07 a NaN went past the region as no
   * restriction, its profile's point stayed NaN, and NaN went to the motor controller every loop.
   */
  @Test
  void aTargetThatIsntANumberIsNoRequestAndTheRegionStillHolds() {
    Map<Joint, Silenceable> ios = new HashMap<>();
    JointsSimHarness watched = silenceable(GUARDED, ios);
    JointMechanism chased = watched.joints.of(PIVOT);
    double[] target = {130};
    Command aim =
        Command.requiring(chased)
            .executing(co -> co.await(chased.follow(() -> Degrees.of(target[0]))))
            .named("Aim");
    watched.scheduler.schedule(aim);
    double furthest = 0;
    for (int i = 0; i < 50; i++) {
      watched.loop();
      furthest = Math.max(furthest, Math.toDegrees(watched.simulation.position(PIVOT)));
    }
    double before = Math.toDegrees(watched.simulation.position(PIVOT));
    target[0] = Double.NaN;
    watched.loop();
    String rejected = chased.rejected();
    double during = Math.toDegrees(watched.simulation.position(PIVOT));
    target[0] = 130;
    for (int i = 0; i < 50; i++) {
      watched.loop();
      furthest = Math.max(furthest, Math.toDegrees(watched.simulation.position(PIVOT)));
    }
    String heldBy = chased.heldBy();
    target[0] = 30;
    watched.run(1.5);

    assertThat(before).as("held at the edge").isCloseTo(40, within(0.1));
    assertThat(rejected)
        .isEqualTo(
            "Aim asked pivot for a position of NaN, which isn't a number: pivot idles instead;"
                + " check how Aim works it out");
    assertThat(during).isCloseTo(before, within(0.5));
    assertThat(furthest).as("never past the edge").isLessThan(40.1);
    assertThat(heldBy).isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(chased.rejected()).as("a loop's").isEmpty();
    assertThat(io(ios, PIVOT).notNumbers).isEmpty();
    assertThat(pivotDegrees(watched)).as("the next target").isCloseTo(30, within(0.5));
    assertThat(
            Arrays.stream(AlertSim.getActive())
                .filter(alert -> alert.group.equals("Joints"))
                .map(alert -> alert.text))
        .containsExactly(rejected);
  }

  /**
   * The alert a joint raises as it rejects a target goes with its robot, so the next robot built, a
   * test's, raises its own. Until 2026-10-07 it outlived its robot, and the next robot's joint of
   * the same name threw as it rejected a target: "Alert already allocated".
   */
  @Test
  void aRejectedTargetsAlertGoesWithItsRobot() {
    for (int robot = 0; robot < 2; robot++) {
      JointsSimHarness built = new JointsSimHarness(GUARDED);
      JointMechanism chased = built.joints.of(PIVOT);
      built.scheduler.schedule(chased.follow(() -> Degrees.of(Double.NaN)));

      built.loop();

      assertThat(chased.rejected())
          .startsWith("pivot follows a target asked pivot for a position of NaN");
      built.close();
      assertThat(AlertSim.getActive()).noneMatch(alert -> alert.group.equals("Joints"));
    }
  }

  /**
   * The elevator gone silent, where it is isn't known, so the region it opens closes, fail-safe:
   * the pivot, sent out with the elevator raised clear, is held at the region's edge. The elevator
   * answering again, it's let go.
   */
  @Test
  void aSilentJointsKeepOutHoldsFailSafe() {
    Map<Joint, Silenceable> ios = new HashMap<>();
    JointsSimHarness silent = silenceable(GUARDED, ios);
    JointMechanism held = silent.joints.of(PIVOT);
    run(silent, silent.joints.of(ELEVATOR).goTo(ELEVATOR_CLEAR), 2);
    io(ios, ELEVATOR).silent = true;

    Move out = held.goTo(PIVOT_OUT).within(Seconds.of(5));
    silent.scheduler.schedule(out);
    silent.run(1.5);
    double waiting = Math.toDegrees(silent.simulation.position(PIVOT));
    String heldBy = held.heldBy();
    io(ios, ELEVATOR).silent = false;
    silent.runUntil(() -> out.outcome().isPresent(), 2);

    assertThat(waiting).as("at the region's edge").isBetween(38.0, 41.0);
    assertThat(heldBy).isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(out.arrived()).as("let go").isTrue();
  }

  /**
   * The elevator's frames not coming, its reading isn't fresh, so the region it opens closes at
   * once, without waiting the half second that would make it silent; its own moves aren't refused.
   * Its frames coming again, the pivot is let go.
   */
  @Test
  void aJointWhoseReadingIsntFreshHoldsAtOnce() {
    Map<Joint, Silenceable> ios = new HashMap<>();
    JointsSimHarness stale = silenceable(GUARDED, ios);
    JointMechanism held = stale.joints.of(PIVOT);
    JointMechanism elevator = stale.joints.of(ELEVATOR);
    run(stale, elevator.goTo(ELEVATOR_CLEAR), 2);
    Move out = held.goTo(PIVOT_OUT).within(Seconds.of(5));
    stale.scheduler.schedule(out);
    stale.run(0.2);
    io(ios, ELEVATOR).stale = true;
    stale.loop();
    String heldBy = held.heldBy();
    boolean refused = elevator.fault().orElse(null) != null;
    io(ios, ELEVATOR).stale = false;
    stale.runUntil(() -> out.outcome().isPresent(), 2);

    assertThat(heldBy).isEqualTo("pivot keeps out of 40° to 80° while elevator at most clear");
    assertThat(refused).as("its moves refused").isFalse();
    assertThat(out.arrived()).as("let go").isTrue();
  }

  /**
   * The roller gone silent, how fast it runs isn't known, so the pivot that runs only with it up to
   * speed is held where it comes to a stop. The roller answering again and run again, it's let go.
   */
  @Test
  void aSilentJointsSpeedHoldsFailSafe() {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .name("spun up")
            .configurations(List.of())
            .interlocks(List.of(Interlock.that(PIVOT).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(1000)))))
            .build();
    Map<Joint, Silenceable> ios = new HashMap<>();
    JointsSimHarness spun = silenceable(robot, ios);
    JointMechanism held = spun.joints.of(PIVOT);
    JointMechanism roller = spun.joints.of(ROLLER);
    spun.scheduler.schedule(roller.run(ROLLER_INTAKING));
    spun.run(0.5);

    Move out = held.goTo(PIVOT_OUT).within(Seconds.of(5));
    spun.scheduler.schedule(out);
    spun.run(0.1);
    io(ios, ROLLER).silent = true;
    spun.run(1);
    double stopped = Math.toDegrees(spun.simulation.position(PIVOT));
    spun.run(0.5);
    double still = Math.toDegrees(spun.simulation.position(PIVOT));
    String heldBy = held.heldBy();
    io(ios, ROLLER).silent = false;
    spun.scheduler.schedule(roller.run(ROLLER_INTAKING));
    spun.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(stopped).as("held on its way out").isBetween(5.0, 125.0);
    assertThat(still).as("where it stopped").isCloseTo(stopped, within(0.5));
    assertThat(heldBy).startsWith("pivot runs only when roller at least 1000");
    assertThat(out.arrived()).as("let go").isTrue();
  }

  /**
   * The elevator faulted, its homing finding no stop in its time, where it is isn't known, so the
   * region it opens stays closed: the pivot sent out waits at its edge. Homed again and raised
   * clear, the elevator lets it go.
   */
  @Test
  void aFaultedJointsKeepOutHoldsFailSafe() {
    RobotDefinition robot =
        GUARDED.with(
            ELEVATOR.retuned(
                control ->
                    JointFixtures.homed(
                        control,
                        MechanismControl.Homing.toHardStop(
                            Sensor.Switch.Limit.End.LOWER, Seconds.of(0.3)))));
    JointsSimHarness faulty = new JointsSimHarness(robot);
    JointMechanism held = faulty.joints.of(PIVOT);
    JointMechanism elevator = faulty.joints.of(ELEVATOR);
    faulty.simulation.powerOnAt(ELEVATOR, 1.1);

    Move out = held.goTo(PIVOT_OUT).within(Seconds.of(8));
    faulty.scheduler.schedule(out);
    faulty.run(1.5);
    JointMechanism.State faulted = elevator.state();
    double waiting = Math.toDegrees(faulty.simulation.position(PIVOT));
    faulty.simulation.place(ELEVATOR, 0); // lowered by hand
    Move home = run(faulty, elevator.home(), 1);
    run(faulty, elevator.goTo(ELEVATOR_CLEAR), 2);
    faulty.runUntil(() -> out.outcome().isPresent(), 2);

    assertThat(faulted).isEqualTo(JointMechanism.State.FAULTED);
    assertThat(waiting).as("at the region's edge").isBetween(38.0, 41.0);
    assertThat(home.arrived()).isTrue();
    assertThat(out.arrived()).as("let go").isTrue();
  }

  /**
   * A turret that turns freely, with no profile, kept out of its back half turn, 180° to 360°,
   * while the elevator is low. At 90°, sent to -170° at 98% of its cruise, or chasing it, it goes
   * the short way round, toward 190°: it's held at 180°, the edge, and says so. Until 2026-10-08
   * the region judged -170° as it was given, below the region and left alone, and the turret's
   * profile took it to 190°, 10° inside.
   */
  @Test
  void aJointThatTurnsFreelyIsKeptOutTheWayItsProfileGoes() {
    Joint.Revolute turret =
        TURRET.retuned(
            old -> JointFixtures.assumed(control(20, 0.5, Optional.empty(), Optional.empty(), 40)));
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .name("turret kept out")
            .joints(List.of(PIVOT, ROLLER, ELEVATOR, turret))
            .configurations(List.of())
            .interlocks(
                List.of(
                    Interlock.that(turret)
                        .keepsOut(Range.degrees(180, 360))
                        .when(ELEVATOR.atMost(ELEVATOR_CLEAR))))
            .build();
    for (boolean chasing : new boolean[] {false, true}) {
      JointsSimHarness kept = new JointsSimHarness(robot);
      kept.simulation.place(TURRET, Math.toRadians(90));
      kept.loop();
      JointMechanism held = kept.joints.of(TURRET);
      Command toward =
          chasing
              ? held.follow(() -> Degrees.of(-170))
              : held.goTo(Degrees.of(-170)).atSpeed(Percent.of(98));

      double furthest = furthest(kept, TURRET, toward, 2);

      String how = chasing ? "chasing" : "a move";
      assertThat(furthest).as(how).isCloseTo(180, within(1.0));
      assertThat(Math.toDegrees(kept.simulation.position(TURRET)))
          .as(how)
          .isCloseTo(180, within(1.0));
      assertThat(held.heldBy())
          .as(how)
          .isEqualTo("turret keeps out of 180° to 360° while elevator at most clear");
    }
  }

  /** An interlock about a joint the template doesn't drive can't be enforced, so it's refused. */
  @Test
  void anInterlockTheTemplateCantEnforceIsRefused() {
    var hatch =
        JointRevoluteBuilder.builder(PIVOT)
            .label("hatch")
            .link(JointFixtures.link("hatch", 1, new Translation3d()))
            .driven(Driven.NONE)
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .joints(List.of(PIVOT, ROLLER, ELEVATOR, TURRET, hatch))
            .configurations(List.of())
            .interlocks(List.of(Interlock.that(ROLLER).runsOnlyWhen(hatch.atLeast(PIVOT_RUN))))
            .build();

    assertThatThrownBy(
            () ->
                new Joints(
                    robot,
                    joint -> new JointIO() {},
                    Joints.untuned(),
                    Scheduler.createIndependentScheduler()))
        .hasMessage(
            "\"roller runs only when hatch at least run\" can't be enforced: hatch has no mechanism, as"
                + " nothing drives it");
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_PUSH;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_EJECTING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_OUTTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_BACK;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.assumed;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The joints' pre-built commands, run in the simulator as the robot runs them: each move ends with
 * the outcome and reason it should, by its deadline, and each joint idles as it should after.
 * {@link JointFixtures} has the joints: a pivot out to 130°, a roller on it, an elevator, a turret.
 */
@ExtendWith(RobotSim.class)
class JointMovesSimTest {
  private final JointsSimHarness sim = new JointsSimHarness(ROBOT);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);
  private final JointMechanism elevator = sim.joints.of(ELEVATOR);
  private final JointMechanism turret = sim.joints.of(TURRET);

  /** The roller's free speed through its gearing, radians a second. */
  private final double rollerFreeSpeed = ROBOT.math().freeSpeed(ROLLER);

  /** Schedules a move and runs loops until it ends, at most {@code seconds}: how long it took. */
  private double runToEnd(Move move, double seconds) {
    sim.scheduler.schedule(move);
    return sim.runUntil(() -> move.outcome().isPresent(), seconds);
  }

  private double pivotDegrees() {
    return Math.toDegrees(sim.simulation.position(PIVOT));
  }

  /** Several parts' mechanisms, as one requirement, in the order named. */
  @Test
  void jointsDrivenTogetherAreOneRequirement() {
    assertThat(sim.joints.all(PIVOT, ROLLER)).containsExactly(pivot, roller);
  }

  @Test
  void aMoveArrivesAndSaysSo() {
    Move out = pivot.goTo(PIVOT_OUT);

    double took = runToEnd(out, 3);

    assertThat(out.outcome()).contains(Outcome.ARRIVED);
    assertThat(out.arrived()).isTrue();
    assertThat(out.reason()).matches("pivot → out: ARRIVED in 0\\.\\d\\d s");
    assertThat(pivot.lastOutcome()).isEqualTo(out.reason());
    assertThat(took).as("the profile's 0.55 s, and settling").isLessThan(0.8);
    assertThat(pivotDegrees()).isCloseTo(130, within(2.0));
  }

  @Test
  void aShareOfTheCruiseTakesAsLongAsItsProfile() {
    double full = runToEnd(pivot.goTo(PIVOT_OUT), 3);
    sim.scheduler.schedule(pivot.goTo(PIVOT_IN));
    sim.run(2);
    Move half = pivot.goTo(PIVOT_OUT).atSpeed(Percent.of(50));

    double slower = runToEnd(half, 3);

    assertThat(half.name()).isEqualTo("pivot → out at 50%");
    assertThat(half.arrived()).isTrue();
    assertThat(slower - full)
        .as("the profiles take 0.547 s and 0.694 s from in to out")
        .isCloseTo(0.147, within(0.06));
  }

  /**
   * A move that ends as it starts, refused or there already, still takes its loop, so a loop of
   * moves, as the guide's shooting feeds whenever it's ready, goes round once a loop rather than
   * spinning in one. Until 2026-10-07 it ended without a loop: with the guide's feed refused as it
   * started, a feeder's motor controller silent, and the shot still ready, its loop went round
   * without end, and the robot's loop never finished.
   */
  @Test
  void aMoveThatEndsAsItStartsStillTakesItsLoop() {
    int[] refused = {0};
    int[] arrived = {0};
    sim.scheduler.schedule(
        Command.noRequirements(
                co -> {
                  while (refused[0] < 100) {
                    refused[0]++;
                    co.await(pivot.goTo(Degrees.of(140)));
                  }
                })
            .named("Refused, again and again"));
    sim.scheduler.schedule(
        Command.noRequirements(
                co -> {
                  while (arrived[0] < 100) {
                    arrived[0]++;
                    co.await(roller.runUntil(ROLLER_INTAKING, () -> true));
                  }
                })
            .named("There, again and again"));

    sim.loop();
    int refusedInALoop = refused[0];
    int arrivedInALoop = arrived[0];
    sim.loop();

    assertThat(refusedInALoop).isEqualTo(1);
    assertThat(arrivedInALoop).isEqualTo(1);
    assertThat(refused[0]).as("again, the next loop").isEqualTo(2);
    assertThat(arrived[0]).isEqualTo(2);
  }

  @Test
  void aTargetOutsideTheLimitsIsRefusedAndNothingMoves() {
    Move tooFar = pivot.goTo(Degrees.of(140));

    runToEnd(tooFar, 1);
    sim.run(0.5);

    assertThat(tooFar.outcome()).contains(Outcome.REFUSED);
    assertThat(tooFar.reason())
        .isEqualTo("pivot → 140°: REFUSED: 140° is outside pivot's limits, 0° to 135°");
    assertThat(pivotDegrees()).as("it holds where it was").isCloseTo(1.5, within(1.0));
  }

  /**
   * An elevator whose motors can't lift it, at 1 A each: its move to "clear" times out at its
   * default deadline, twice its profile's 0.573 s plus half a second, and then it holds where it is
   * rather than pushing on.
   */
  @Test
  void aMoveThatCantGetThereTimesOutAtItsDefaultDeadline() {
    JointsSimHarness stuck =
        new JointsSimHarness(
            ROBOT.with(
                ELEVATOR.retuned(
                    old -> assumed(control(60, 2, Optional.empty(), profile(1.2, 5), 1)))));
    JointMechanism lift = stuck.joints.of(ELEVATOR);
    Move clear = lift.goTo(ELEVATOR_CLEAR);

    stuck.scheduler.schedule(clear);
    double took = stuck.runUntil(() -> clear.outcome().isPresent(), 3);

    assertThat(clear.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(clear.reason())
        .isEqualTo("elevator → clear: TIMED_OUT after 1.66 s, elevator at 0 m");
    assertThat(took).isCloseTo(1.68, within(0.001));
    assertThat(lift.goal()).as("it holds where it stopped").isCloseTo(0, within(0.005));
  }

  @Test
  void aTighterDeadlineIsKept() {
    Move out = pivot.goTo(PIVOT_OUT).within(Seconds.of(0.2));

    double took = runToEnd(out, 2);

    assertThat(out.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(out.reason()).startsWith("pivot → out: TIMED_OUT after 0.20 s, pivot at ");
    assertThat(took).isCloseTo(0.22, within(0.001));
  }

  /**
   * A move taken over mid-way ends canceled, and the next starts from where the joint is and how
   * fast it's moving: it slows to a stop before turning back, no further out than its acceleration
   * allows, rather than jumping its setpoint.
   */
  @Test
  void anInterruptedMoveIsCanceledAndTheNextStartsFromWhereTheJointIs() {
    Move out = pivot.goTo(PIVOT_OUT);
    sim.run(out, 0.3);
    Move in = pivot.goTo(PIVOT_IN);
    sim.scheduler.schedule(in);
    sim.loop(); // it starts, from what the joint reads this loop
    double at = pivot.position();
    double speed = pivot.velocity();
    double[] furthest = {at};

    double took =
        sim.runUntil(
            () -> {
              furthest[0] = Math.max(furthest[0], sim.simulation.position(PIVOT));
              return in.outcome().isPresent();
            },
            3);

    assertThat(out.outcome()).contains(Outcome.CANCELED);
    assertThat(out.reason()).startsWith("pivot → out: CANCELED after 0.30 s, pivot at ");
    assertThat(in.arrived()).isTrue();
    assertThat(took).isLessThan(1);
    assertThat(speed).as("still moving out when taken over").isGreaterThan(5);
    double stopping = speed * speed / (2 * 30);
    assertThat(furthest[0] - at)
        .as("it stops within its acceleration's distance")
        .isLessThan(stopping + Math.toRadians(3));
  }

  @Test
  void afterArrivingIdleHoldsTheTarget() {
    runToEnd(pivot.goTo(PIVOT_OUT), 3);

    sim.run(3);

    assertThat(pivot.goal()).isCloseTo(Math.toRadians(130), within(1e-12));
    assertThat(pivotDegrees()).isCloseTo(130, within(2.0));
  }

  @Test
  void afterACancelIdleHoldsWhereTheJointStopped() {
    Move out = pivot.goTo(PIVOT_OUT);
    sim.run(out, 0.3);
    sim.scheduler.cancel(out);

    sim.run(2);

    assertThat(out.outcome()).contains(Outcome.CANCELED);
    assertThat(Math.toDegrees(pivot.goal())).isBetween(40.0, 125.0);
    assertThat(sim.simulation.position(PIVOT)).isCloseTo(pivot.goal(), within(Math.toRadians(2)));
  }

  @Test
  void holdOwnsTheJointAndKeepsItsGoal() {
    runToEnd(pivot.goTo(PIVOT_OUT), 3);
    Move hold = pivot.hold();

    sim.run(hold, 2);

    assertThat(hold.requirements()).containsExactly(pivot);
    assertThat(sim.scheduler.isRunning(hold)).isTrue();
    assertThat(hold.outcome()).isEmpty();
    assertThat(pivotDegrees()).isCloseTo(130, within(2.0));
  }

  @Test
  void aRollerRunsAtItsNamedSpeedAndStopsWhenIdle() {
    Move intaking = roller.run(ROLLER_INTAKING);
    sim.run(intaking, 1);
    double running = sim.simulation.velocity(ROLLER);

    sim.scheduler.cancel(intaking);
    sim.run(1);

    assertThat(running).isCloseTo(RPM.of(1500).baseUnitMagnitude(), within(0.03 * 157.08));
    assertThat(intaking.outcome()).contains(Outcome.CANCELED);
    assertThat(intaking.reason()).isEqualTo("roller: intaking: CANCELED after 0.98 s");
    assertThat(sim.simulation.velocity(ROLLER)).isCloseTo(0, within(0.5));
  }

  /** A voltage and a duty cycle: -6 V and half of 12 V backward turn it at half its free speed. */
  @Test
  void aVoltageAndADutyCycleAreOutputsToo() {
    sim.run(roller.run(ROLLER_OUTTAKING), 1);
    double outtaking = sim.simulation.velocity(ROLLER);
    sim.run(roller.run(ROLLER_EJECTING), 1);
    double ejecting = sim.simulation.velocity(ROLLER);

    assertThat(outtaking).isCloseTo(-0.5 * rollerFreeSpeed, within(0.01 * rollerFreeSpeed));
    assertThat(ejecting).isCloseTo(-0.5 * rollerFreeSpeed, within(0.01 * rollerFreeSpeed));
  }

  @Test
  void runUntilArrivesWhenItsConditionHolds() {
    Move fast =
        roller.runUntil(
            ROLLER_INTAKING, () -> roller.velocity() > RPM.of(1000).baseUnitMagnitude());

    double took = runToEnd(fast, 2);

    assertThat(fast.name()).isEqualTo("roller: intaking until");
    assertThat(fast.arrived()).isTrue();
    assertThat(fast.reason()).startsWith("roller: intaking until: ARRIVED in 0.");
    assertThat(took).isLessThan(0.5);
  }

  /**
   * A run-until has no natural duration, so it has no deadline of its own: with a condition that
   * never holds it runs on until it is canceled. Until 2026-10-11 it timed out after 3 s, which
   * stopped a held intake early for no reason its code showed.
   */
  @Test
  void runUntilHasNoDeadlineOfItsOwn() {
    Move never = roller.runUntil(ROLLER_INTAKING, () -> false);

    double took = runToEnd(never, 10);
    double stillIntaking = sim.simulation.velocity(ROLLER);
    sim.scheduler.cancel(never);

    assertThat(took).as("it never ended in 10 s").isNaN();
    assertThat(stillIntaking).isCloseTo(RPM.of(1500).baseUnitMagnitude(), within(0.03 * 157.08));
    assertThat(never.reason()).isEqualTo("roller: intaking until: CANCELED after 9.98 s");
  }

  /** Given a deadline, a run-until whose condition never holds times out at it. */
  @Test
  void runUntilTimesOutAtTheDeadlineItIsGiven() {
    Move brief = roller.runUntil(ROLLER_INTAKING, () -> false).within(Seconds.of(1));

    double took = runToEnd(brief, 2);

    assertThat(brief.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(brief.reason()).isEqualTo("roller: intaking until: TIMED_OUT after 1.00 s");
    assertThat(took).isCloseTo(1.02, within(0.001));
  }

  /**
   * A command with a law of its own asks for a voltage every loop; when it ends, the roller idles.
   */
  @Test
  void aCommandOfItsOwnSetsAVoltageEachLoop() {
    Command push =
        roller
            .run(
                co -> {
                  while (true) {
                    roller.setVoltage(3);
                    co.yield();
                  }
                })
            .named("Push");
    sim.run(push, 1);
    double pushing = sim.simulation.velocity(ROLLER);

    sim.scheduler.cancel(push);
    sim.run(1);

    assertThat(pushing).isCloseTo(0.25 * rollerFreeSpeed, within(0.01 * rollerFreeSpeed));
    assertThat(sim.simulation.velocity(ROLLER)).isCloseTo(0, within(0.5));
  }

  @Test
  void followChasesItsTargetInsideTheLimits() {
    double[] meters = {0.5};
    Move follow = elevator.follow(() -> Meters.of(meters[0]));

    sim.run(follow, 1.5);
    double first = sim.simulation.position(ELEVATOR);
    meters[0] = 2.0; // above the top: held at it
    sim.run(2);

    assertThat(first).isCloseTo(0.5, within(0.01));
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(1.2, within(0.01));
    assertThat(follow.outcome()).as("it never ends on its own").isEmpty();
  }

  @Test
  void followCanChaseASpeed() {
    sim.run(roller.follow(() -> RPM.of(1000)), 1);

    assertThat(sim.simulation.velocity(ROLLER))
        .isCloseTo(RPM.of(1000).baseUnitMagnitude(), within(0.03 * 104.72));
  }

  /**
   * A place or an output of another joint's, built as the robot starts (a binding, or an auto built
   * with them), throws there, saying whose it is and what the joint has: the robot doesn't start,
   * and a test that builds it fails, before anyone presses a button.
   */
  @Test
  void aPlaceOfAnotherJointsBuiltAsTheRobotStartsThrows() {
    assertThatThrownBy(() -> pivot.goTo(ELEVATOR_CLEAR))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("pivot has no position clear at 0.4 m: it has in, run, out");
    assertThatThrownBy(() -> roller.run(PIVOT_PUSH))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("roller has no output push: it has intaking, outtaking, ejecting, and stopped");
    assertThatThrownBy(() -> roller.runUntil(PIVOT_PUSH, () -> true))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("roller has no output push: it has intaking, outtaking, ejecting, and stopped");
  }

  /**
   * Built once the robot runs, by a command or anywhere else, the same move is refused as it
   * starts, its reason logged, and the joint stays where it is: a robot on the field is never
   * stopped by one.
   */
  @Test
  void aPlaceOfAnotherJointsBuiltWhileTheRobotRunsIsRefused() {
    List<Move> built = new ArrayList<>();
    Command wrong =
        Command.requiring(pivot, roller)
            .executing(
                co -> {
                  built.add(pivot.goTo(ELEVATOR_CLEAR));
                  built.add(roller.run(PIVOT_PUSH));
                  for (Move move : built) {
                    co.await(move);
                  }
                })
            .named("Wrong places");

    sim.run(wrong, 0.5);
    Move later = pivot.goTo(ELEVATOR_CLEAR);
    runToEnd(later, 1);

    assertThat(built).hasSize(2);
    assertThat(built.get(0).outcome()).contains(Outcome.REFUSED);
    assertThat(built.get(0).reason())
        .isEqualTo(
            "pivot → clear: REFUSED: pivot has no position clear at 0.4 m: it has in, run, out");
    assertThat(built.get(1).reason())
        .isEqualTo(
            "roller: push: REFUSED: roller has no output push: it has intaking, outtaking,"
                + " ejecting, and stopped");
    assertThat(later.outcome()).as("outside a command, once it runs").contains(Outcome.REFUSED);
    assertThat(pivot.lastOutcome()).isEqualTo(later.reason());
    assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(1.5), within(0.01));
  }

  @Test
  void followRefusesATargetOfTheWrongKind() {
    Move wrong = pivot.follow(() -> Meters.of(1));

    runToEnd(wrong, 1);

    assertThat(wrong.outcome()).contains(Outcome.REFUSED);
    assertThat(wrong.reason())
        .isEqualTo(
            "pivot follows: REFUSED: pivot can't follow 1.0 Meter: it takes an angle or an angular"
                + " velocity");
  }

  /** At -170°, "back" at 180° is 10° away the short way round, not 350° the long way. */
  @Test
  void aJointThatTurnsFreelyGoesTheShortWayRound() {
    sim.simulation.place(TURRET, Math.toRadians(-170));
    Move back = turret.goTo(TURRET_BACK);

    double took = runToEnd(back, 4);

    assertThat(back.arrived()).isTrue();
    assertThat(took).isLessThan(0.6);
    assertThat(Math.toDegrees(sim.simulation.position(TURRET))).isCloseTo(-180, within(2.0));
  }

  /**
   * A configuration requires exactly the joints it names, and arrives when every one is there: the
   * pivot out, the elevator clear, the roller within its band of its speed.
   */
  @Test
  void aConfigurationMovesItsJointsTogether() {
    Move intaking = sim.joints.goTo(INTAKING);

    double took = runToEnd(intaking, 3);

    assertThat(intaking.requirements()).containsExactlyInAnyOrder(pivot, roller, elevator);
    assertThat(intaking.arrived()).isTrue();
    assertThat(intaking.reason()).startsWith("→ intaking: ARRIVED in ");
    assertThat(took).isLessThan(1);
    assertThat(pivotDegrees()).isCloseTo(130, within(2.0));
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0.4, within(0.01));
    assertThat(pivot.lastOutcome()).isEqualTo(intaking.reason());
    // Each joint's own arrival, the slowest the move's.
    assertThat(intaking.arrivals()).containsOnlyKeys(PIVOT, ROLLER, ELEVATOR);
    assertThat(intaking.arrivals().values()).allSatisfy(at -> assertThat(at).isLessThan(took));
    assertThat(intaking.arrivals().get(ROLLER))
        .as("the roller's speed, before the pivot swings out")
        .isLessThan(intaking.arrivals().get(PIVOT));
    assertThat(intaking.reason()).contains("(pivot ").contains(", roller ").contains(", elevator ");
  }

  @Test
  void runningAConfigurationHoldsItUntilCanceled() {
    Move run = sim.joints.run(INTAKING);

    sim.run(run, 2);

    assertThat(run.outcome()).isEmpty();
    assertThat(sim.simulation.velocity(ROLLER))
        .isCloseTo(RPM.of(1500).baseUnitMagnitude(), within(0.03 * 157.08));
  }

  /** The same move bound to a button runs again and again: each run starts afresh. */
  @Test
  void aMoveRunAgainStartsAfresh() {
    Move out = pivot.goTo(PIVOT_OUT);
    runToEnd(out, 3);
    runToEnd(pivot.goTo(PIVOT_IN), 3);

    sim.scheduler.schedule(out);
    sim.loop();
    Optional<Outcome> running = out.outcome();
    sim.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(running).as("no outcome while it runs").isEmpty();
    assertThat(out.arrived()).isTrue();
    assertThat(pivotDegrees()).isCloseTo(130, within(2.0));
  }

  /**
   * Disabled, every joint goes neutral, and the pivot sinks; enabled again, it holds where it is
   * then, not where it was before.
   */
  @Test
  void enabledAgainAJointHoldsWhereItIsNow() {
    runToEnd(pivot.goTo(Degrees.of(45)), 3);
    sim.setEnabled(false);
    sim.run(2);
    double sank = sim.simulation.position(PIVOT);

    sim.setEnabled(true);
    sim.run(1);

    assertThat(Math.toDegrees(sank)).as("braking, it sinks under its weight").isLessThan(25);
    assertThat(pivot.goal()).isCloseTo(sank, within(Math.toRadians(1)));
    assertThat(sim.simulation.position(PIVOT)).isCloseTo(sank, within(Math.toRadians(2)));
  }

  @Test
  void movesNameTheirJointAndRequireIt() {
    assertThat(pivot.goTo(PIVOT_OUT).requirements()).containsExactly(pivot);
    assertThat(pivot.goTo(Degrees.of(45)).name()).isEqualTo("pivot → 45°");
    assertThat(elevator.goTo(Meters.of(0.25)).name()).isEqualTo("elevator → 0.25 m");
    assertThat(roller.run(ROLLER_INTAKING).name()).isEqualTo("roller: intaking");
    assertThat(roller.run(Output.STOPPED).name()).isEqualTo("roller: stopped");
    assertThat(pivot.getName()).isEqualTo("pivot");
  }

  @Test
  void aJointThatIsntTheRobotsOrIsntDrivenHasNoMechanism() {
    assertThatThrownBy(
            () -> sim.joints.of(JointRevoluteBuilder.builder(PIVOT).label("stranger").build()))
        .hasMessage("stranger isn't one of joints' joints");
    assertThat(sim.joints.all()).containsExactly(pivot, roller, elevator, turret);
  }

  /**
   * A configuration naming a joint the template doesn't drive (its motor needs another controller,
   * say) is refused whenever it would start, saying why, and its joints that are driven stay where
   * they are: the robot still runs, rather than failing as its commands are made.
   */
  @Test
  void aConfigurationNamingAJointNotDrivenIsRefusedWhenItWouldStart() {
    Joint.Revolute hatch =
        JointRevoluteBuilder.builder(PIVOT)
            .label("hatch")
            .origin(new Transform3d(new Translation3d(-0.2, 0, 0.3), Rotation3d.ZERO))
            .link(JointFixtures.link("hatch", 1, new Translation3d()))
            .driven(Driven.NONE)
            .build();
    Configuration open = Configuration.of("open", PIVOT.at(PIVOT_OUT), hatch.at(PIVOT_OUT));
    RobotDefinition withHatch =
        RobotDefinitionBuilder.builder(ROBOT)
            .joints(List.of(PIVOT, ROLLER, ELEVATOR, TURRET, hatch))
            .configurations(List.of(INTAKING, open))
            .build();
    JointsSimHarness hatched = new JointsSimHarness(withHatch);
    hatched.run(0.5);
    double before = hatched.simulation.position(PIVOT);

    Move opening = hatched.joints.goTo(open);
    hatched.run(opening, 0.5);

    assertThat(Joints.cantRun(withHatch, open))
        .isEqualTo("open can't run: hatch has no mechanism: nothing drives it");
    assertThat(Joints.cantRun(withHatch, INTAKING)).isNull();
    assertThat(opening.outcome()).contains(Outcome.REFUSED);
    assertThat(opening.reason())
        .isEqualTo("→ open: REFUSED: hatch has no mechanism: nothing drives it");
    assertThat(opening.requirements()).containsExactly(hatched.joints.of(PIVOT));
    assertThat(hatched.simulation.position(PIVOT)).isCloseTo(before, within(1e-3));
    assertThat(hatched.joints.isAt(open).getAsBoolean()).isFalse();
  }

  /** A configuration none of whose joints the template drives is refused just the same. */
  @Test
  void aConfigurationOfNoDrivenJointIsRefusedJustTheSame() {
    Joint.Revolute hatch =
        JointRevoluteBuilder.builder(PIVOT)
            .label("hatch")
            .origin(new Transform3d(new Translation3d(-0.2, 0, 0.3), Rotation3d.ZERO))
            .link(JointFixtures.link("hatch", 1, new Translation3d()))
            .driven(Driven.NONE)
            .build();
    Configuration shut = Configuration.of("shut", hatch.at(PIVOT_IN));
    RobotDefinition withHatch =
        RobotDefinitionBuilder.builder(ROBOT)
            .joints(List.of(PIVOT, ROLLER, ELEVATOR, TURRET, hatch))
            .configurations(List.of(INTAKING, shut))
            .build();
    JointsSimHarness hatched = new JointsSimHarness(withHatch);

    Move shutting = hatched.joints.goTo(shut);
    hatched.run(shutting, 0.1);

    assertThat(shutting.requirements()).isEmpty();
    assertThat(shutting.reason())
        .isEqualTo("→ shut: REFUSED: hatch has no mechanism: nothing drives it");
  }
}

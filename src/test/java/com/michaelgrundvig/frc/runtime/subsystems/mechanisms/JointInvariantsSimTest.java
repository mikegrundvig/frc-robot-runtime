package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_DOWN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.Monkey;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.examples.ExampleMonkey;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointPrismaticBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.system.RobotController;

/**
 * What the monkey checks every loop ({@link JointInvariants}), on the fixtures' joints in moments
 * picked to judge: it lets be what the robot can't help, and still finds what it could.
 */
@ExtendWith(RobotSim.class)
class JointInvariantsSimTest {
  /** The pivot stowed and the elevator down: the pivot swings in through 80° to 40°. */
  static final Configuration STOWED =
      Configuration.of("stowed", PIVOT.at(PIVOT_IN), ELEVATOR.at(ELEVATOR_DOWN));

  /**
   * The interlocks' robot, the pivot kept out of 40° to 80° while the elevator is below "clear",
   * stowing.
   */
  static final RobotDefinition GUARDED =
      RobotDefinitionBuilder.builder(InterlocksSimTest.GUARDED)
          .configurations(List.of(STOWED))
          .build();

  /** A robot's joints in the simulator, each loop checked as the monkey checks it. */
  private static final class Checked {
    final JointsSimHarness sim;
    final JointInvariants invariants;
    final Map<Joint, WatchedJoint> watched = new LinkedHashMap<>();

    Checked(RobotDefinition robot) {
      sim =
          new JointsSimHarness(
              robot,
              (joint, io) -> {
                WatchedJoint each = new WatchedJoint(io);
                watched.put(joint, each);
                return each;
              });
      invariants =
          new JointInvariants(
              robot, sim.joints, watched, sim.scheduler, joint -> false, device -> false);
    }

    /** Runs loops for a time, each checked: the first thing found wrong, or null. */
    @Nullable String run(double seconds) {
      String found = null;
      for (int i = 0; i < RobotLoop.loops(seconds); i++) {
        sim.loop();
        String broken = invariants.broken(RobotController.getTime(), true);
        found = found == null ? broken : found;
      }
      return found;
    }

    double pivotDegrees() {
      return Math.toDegrees(sim.simulation.position(PIVOT));
    }
  }

  /**
   * Swinging in from 130° as the elevator comes down from 0.5 m, the pivot is 15° short of 80° at 5
   * rad/s as the region closes, 32° from stopping at its profile's 30 rad/s²: it brakes into the
   * region, and stops where braking took it. Kept out lets it, all the way. Until 2026-10-08 it
   * allowed its 4° band from the loop after the region closed, the pivot still outside, braking.
   */
  @Test
  void aJointTheRegionClosesOnShortOfItsEdgeMayBrakeIntoIt() {
    Checked checked = new Checked(GUARDED);
    checked.sim.simulation.place(PIVOT, Math.toRadians(130));
    checked.sim.simulation.place(ELEVATOR, 0.5);
    checked.sim.loop();
    checked.sim.scheduler.schedule(checked.sim.joints.goTo(STOWED));
    double deepest = 180;

    String broken = null;
    for (int i = 0; i < 100; i++) {
      broken = broken == null ? checked.run(RobotLoop.PERIOD_SECONDS) : broken;
      deepest = Math.min(deepest, checked.pivotDegrees());
    }

    assertThat(deepest).as("past its band into the region").isLessThan(76);
    assertThat(broken).isNull();
  }

  /**
   * The same from 0.55 m: the region closes with the pivot inside it, at 70°, too fast to stop
   * before 40°, so it carries on out the far side, as its interlock lets it, and on to "in". Kept
   * out lets it, judging it from that side once it's out. Until 2026-10-08 it went on judging it
   * from 80°, the pivot outside the region and 78.5° past that edge.
   */
  @Test
  void aJointThatCarriesOnOutTheFarSideIsKeptOutFromThere() {
    Checked checked = new Checked(GUARDED);
    checked.sim.simulation.place(PIVOT, Math.toRadians(130));
    checked.sim.simulation.place(ELEVATOR, 0.55);
    checked.sim.loop();
    checked.sim.scheduler.schedule(checked.sim.joints.goTo(STOWED));

    String broken = checked.run(2);

    assertThat(checked.pivotDegrees()).as("in").isLessThan(2);
    assertThat(broken).isNull();
  }

  /**
   * An elevator carrying 15 kg that coasts when told nothing, its profile braking at 50 m/s², more
   * than its motors can against its weight, kept out of its lowest 0.3 m while the pivot is in.
   */
  static final Joint.Prismatic WEIGHED =
      JointPrismaticBuilder.builder(ELEVATOR)
          .link(JointFixtures.link("carriage", 15, new Translation3d(0, 0, 0.1)))
          .build()
          .retuned(
              old ->
                  JointFixtures.assumed(
                      MechanismControlBuilder.builder(control(60, 2, profile(1.2, 50)))
                          .neutralMode(MechanismControl.NeutralMode.COAST)
                          .build()));

  static final RobotDefinition WEIGHED_ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .name("weighed")
          .joints(List.of(PIVOT, JointFixtures.ROLLER, WEIGHED, JointFixtures.TURRET))
          .configurations(List.of())
          .interlocks(
              List.of(
                  Interlock.that(WEIGHED)
                      .keepsOut(Range.meters(0, 0.3))
                      .when(PIVOT.atMost(JointFixtures.PIVOT_RUN))))
          .build();

  /**
   * Held at 0.35 m, the elevator's motor controllers go off the CAN bus for 0.2 s: it falls, into
   * the closed region, and is still falling fast as they answer again, so it takes a few loops to
   * catch, further than its profile's braking. Kept out lets it be while they're off the bus and
   * for a settle after; then it judges it again, and finds it put in the region by hand. Until
   * 2026-10-08 it judged it from the loop they answered, at its profile's braking.
   */
  @Test
  void aJointWhoseControllersAreOffTheBusIsLetBeAndASettleAfter() {
    Checked checked = new Checked(WEIGHED_ROBOT);
    checked.sim.simulation.place(WEIGHED, 0.35);
    String held = checked.run(0.5);

    checked.sim.simulation.silence(WEIGHED, true);
    String off = checked.run(0.2);
    checked.sim.simulation.silence(WEIGHED, false);
    double back = checked.sim.simulation.position(WEIGHED);
    String settling = checked.run(1);
    checked.sim.simulation.place(WEIGHED, 0.1);
    String inside = checked.run(RobotLoop.PERIOD_SECONDS);

    assertThat(held).isNull();
    assertThat(off).isNull();
    assertThat(back).as("fallen into the region").isLessThan(0.2);
    assertThat(settling).isNull();
    assertThat(inside).startsWith("kept out: elevator is at 0.1");
  }

  /**
   * The weighed elevator on a leader and a follower, its loop soft: kG holds its weight on both
   * motors, and on one, its loop makes up the rest only some 6 cm below where it's held.
   */
  static final Joint.Prismatic SOFT =
      WEIGHED
          .motorsOn(TALON_FX.can(20), TALON_FX.can(21))
          .retuned(old -> JointFixtures.assumed(control(5, 0.5, profile(1.2, 5))));

  static final RobotDefinition SOFT_ROBOT =
      RobotDefinitionBuilder.builder(WEIGHED_ROBOT)
          .name("soft")
          .joints(List.of(PIVOT, JointFixtures.ROLLER, SOFT, JointFixtures.TURRET))
          .interlocks(
              List.of(
                  Interlock.that(SOFT)
                      .keepsOut(Range.meters(0, 0.3))
                      .when(PIVOT.atMost(JointFixtures.PIVOT_RUN))))
          .build();

  /**
   * Held inside the closed region at 0.25 m, the elevator's follower goes off the CAN bus: its
   * leader alone can't hold its weight where both did, and it sags further into the region than its
   * band, slowly. Kept out lets it be while the follower is off the bus and for a settle after, as
   * for its leader; then it judges it again, and finds it put deeper by hand. Until 2026-10-08 it
   * judged it all along: a seeded elevator on three motors, one of its followers off, sank 5.9 cm
   * to its hard stop.
   */
  @Test
  void aJointWithAFollowerOffTheBusIsLetBeAndASettleAfter() {
    Checked checked = new Checked(SOFT_ROBOT);
    checked.sim.simulation.place(SOFT, 0.25);
    String held = checked.run(0.5);
    double holding = checked.sim.simulation.position(SOFT);

    checked.sim.simulation.silenceFollower(SOFT, 0, true);
    String off = checked.run(2);
    double sagged = checked.sim.simulation.position(SOFT);
    checked.sim.simulation.silenceFollower(SOFT, 0, false);
    String settling = checked.run(1);
    checked.sim.simulation.place(SOFT, 0.1);
    String deeper = checked.run(RobotLoop.PERIOD_SECONDS);

    assertThat(held).isNull();
    assertThat(holding - sagged).as("sagged past its band").isGreaterThan(0.03);
    assertThat(off).isNull();
    assertThat(settling).isNull();
    assertThat(deeper).startsWith("kept out: elevator is at 0.1");
  }

  /**
   * Out of the closed region a while, the pivot put inside it by hand, 20° past its edge: kept out
   * says so, its band all it allows.
   */
  @Test
  void aJointInsideAClosedRegionPastWhereItCouldStopIsFound() {
    Checked checked = new Checked(GUARDED);
    checked.sim.simulation.place(PIVOT, Math.toRadians(30));
    String before = checked.run(0.5);

    checked.sim.simulation.place(PIVOT, Math.toRadians(60));
    String broken = checked.run(RobotLoop.PERIOD_SECONDS);

    assertThat(before).isNull();
    assertThat(broken).startsWith("kept out: pivot is at 5").contains("where 4° is allowed");
  }

  /** The pivot, its loop closed on its through-bore on port 1. */
  static final Joint.Revolute LOOPED =
      PIVOT
          .absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(1))
          .retuned(old -> control(30, 1, profile(8, 30)))
          .encoderOn(MechanismWiring.PortEncoder.dutyCycle(1).closesLoop());

  /**
   * The encoder its loop closes on comes unplugged as the pivot swings out: it's faulted, and holds
   * where it comes to a stop by its motor's count. Jammed a few loops on, still far short of that
   * hold, it's checked afresh from where it's stuck; the hold its mechanism keeps telling it is its
   * own, and known before driven still lets it. A position told it from elsewhere, a command
   * reaching the faulted joint, is still found. Until 2026-10-08 the jam made it flag the hold it
   * had just allowed.
   */
  @Test
  void aFaultedJointsOwnHoldIsLetAfterALaterFaultAndACommandReachingItIsFound() {
    Checked checked = new Checked(JointFixtures.ROBOT.with(LOOPED));
    JointMechanism pivot = checked.sim.joints.of(PIVOT);
    checked.run(0.1);
    checked.sim.scheduler.schedule(pivot.goTo(JointFixtures.PIVOT_OUT));
    checked.run(0.3);

    checked.sim.simulation.unplug(PIVOT, true);
    String lost = checked.run(3 * RobotLoop.PERIOD_SECONDS);
    double stuck = checked.sim.simulation.position(PIVOT);
    checked.sim.simulation.jam(PIVOT, true);
    checked.invariants.touched(PIVOT);
    String jammed = checked.run(0.5);
    checked.sim.loop();
    WatchedJoint io = Objects.requireNonNull(checked.watched.get(PIVOT));
    WatchedJoint.Told hold = io.told().getLast();
    io.setPosition(stuck + Math.toRadians(10), 0, 0);
    String reached = checked.invariants.broken(RobotController.getTime(), true);

    assertThat(pivot.state()).isEqualTo(JointMechanism.State.FAULTED);
    assertThat(lost).isNull();
    assertThat(Math.toDegrees(hold.values()[0] - stuck))
        .as("its hold past where it's stuck")
        .isGreaterThan(10);
    assertThat(jammed).isNull();
    assertThat(reached)
        .startsWith("known before driven: pivot was run to a position while FAULTED");
  }

  /**
   * A monkey built here, not in the template's own monkey test, names this test to run it again.
   * Until 2026-10-08 every failure named com.michaelgrundvig.frc.runtime.MonkeySimTest.
   */
  @Test
  void aMonkeyNamesTheTestThatBuiltItToRunItAgain() {
    Monkey monkey = new Monkey(new RobotSim(), ExampleMonkey.ROBOT, 17, 1, true);

    assertThat(monkey.rerun())
        .isEqualTo(
            "./gradlew test --tests com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointInvariantsSimTest"
                + " -PmonkeySeed=17");
  }
}

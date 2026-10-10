package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.ARM;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ELEVATOR_DOWN;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER_FEEDING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INDEXER_FEEDING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Centimeters;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.DrivenBuilder;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Translation3d;

/**
 * The robot docs/mechanisms.md describes is one the robot would start, and its commands do what the
 * guide says, in the simulator: the pivot swings out, and the roller runs once it's out.
 */
@ExtendWith(RobotSim.class)
class ExampleRobotSimTest {
  /** A hopper sliding forward, its "down" at 0 m as the elevator's is, and a "clear" of its own. */
  private static final Position HOPPER_DOWN = Position.of("down", Meters.of(0));

  private static final Position HOPPER_CLEAR = Position.of("clear", Meters.of(0.1));

  private static final Joint.Prismatic HOPPER =
      Joint.slide("hopper", Part.CHASSIS, new Translation3d(0.1, 0, 0.2), new Vector3(1, 0, 0))
          .range(Range.meters(0, 0.3))
          .rest(HOPPER_DOWN)
          .positions(HOPPER_DOWN, HOPPER_CLEAR)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(10)).drum(Inches.of(0.5)))
          .control(MechanismControl.position(30).withHoming(Homing.assumeAtRest()));

  /**
   * Both robots of the design, listed as Robots.all() lists them, are configurations of their own,
   * each passing every check the robot makes as it starts, and each runs in the simulator: the
   * practice robot is the competition robot's joints and cameras, each changed where it's given and
   * kept otherwise.
   */
  @Test
  void theGuidesRobotsAreOnesTheRobotWouldStart() {
    for (RobotConfig robot : ExampleRobot.all()) {
      JointsSimHarness sim = new JointsSimHarness(robot.definition());
      Move out = sim.joints.of(PIVOT).goTo(PIVOT_OUT);
      sim.run(out, 1.5);

      assertThat(out.arrived()).as(robot.name()).isTrue();
    }
    RobotDefinition competition = ExampleRobot.COMPETITION.definition();
    RobotDefinition practice = ExampleRobot.PRACTICE.definition();
    assertThat(ExampleRobot.all())
        .extracting(RobotConfig::name)
        .containsExactly("example", "practice");
    assertThat(practice.joints())
        .as("the same joints, wired otherwise")
        .containsExactlyElementsOf(competition.joints());
    Joint.Revolute pivot = (Joint.Revolute) practice.joints().getFirst();
    assertThat(pivot.driven().motors().orElseThrow().controllers())
        .containsExactly(TALON_FX.can(33).pdh(4, Amps.of(40)));
    assertThat(pivot.driven().motors().orElseThrow().gearing())
        .isEqualTo(PIVOT.driven().motors().orElseThrow().gearing());
    Joint arm = practice.joints().get(4);
    assertThat(arm.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder))
        .contains(CANCODER.can(23).offset(Rotations.of(0.47)));
    assertThat(arm.driven().control().orElseThrow().gains().kG()).contains(0.3);
    assertThat(arm.driven().control().orElseThrow().gains().gravityAngle())
        .as("the rest of its control kept")
        .contains(Degrees.of(-90));
    assertThat(practice.sensors())
        .first()
        .extracting(sensor -> ((Sensor.Switch) sensor).wiring())
        .isEqualTo(Optional.of(CANDI.can(36).s1().normallyOpen()));
    assertThat(practice.cameras())
        .extracting(CameraConfig::name)
        .containsExactlyElementsOf(competition.cameras().stream().map(CameraConfig::name).toList());
    CameraConfig backLeft = practice.cameras().get(2);
    assertThat(backLeft.name()).isEqualTo("back-left");
    assertThat(backLeft.up()).as("half an inch lower").isEqualTo(Inches.of(20.5));
    assertThat(backLeft.yaw())
        .as("the rest of its mount kept")
        .isEqualTo(competition.cameras().get(2).yaw());
  }

  /**
   * The same code drives each robot to its own places: {@code goTo(PIVOT_OUT)} and the intake's
   * configuration, written of the competition robot's constants, swing the practice robot's pivot
   * to its own out, 128°, and its gate on out holds there. Its roller's interlock, a clearance at
   * 110°, lets it run.
   */
  @Test
  void theSameCodeDrivesEachRobotToItsOwnPlaces() {
    for (RobotConfig robot : ExampleRobot.all()) {
      double out = robot == ExampleRobot.PRACTICE ? 128 : 130;
      JointsSimHarness sim = new JointsSimHarness(robot.definition());
      Gate atOut = sim.joints.of(PIVOT).isAt(PIVOT_OUT);
      Move move = sim.joints.of(PIVOT).goTo(PIVOT_OUT);
      sim.run(move, 1.5);

      assertThat(move.arrived()).as(robot.name()).isTrue();
      assertThat(Math.toDegrees(sim.simulation.position(PIVOT)))
          .as(robot.name())
          .isCloseTo(out, within(1.0));
      assertThat(atOut.getAsBoolean()).as(robot.name()).isTrue();

      JointsSimHarness intaking = new JointsSimHarness(robot.definition());
      Move intake = intaking.joints.goTo(ExampleRobot.INTAKE);
      intaking.run(intake, 2);
      assertThat(intake.arrived()).as(robot.name()).isTrue();
      assertThat(Math.toDegrees(intaking.simulation.position(PIVOT)))
          .as(robot.name())
          .isCloseTo(out, within(1.0));
    }
  }

  /**
   * A practice robot's stops are its own: a computed move past them is refused there, and taken on
   * the competition robot, whose stops are farther.
   */
  @Test
  void aPracticeRobotsStopsAreItsOwn() {
    RobotConfig shortStops = ExampleRobot.COMPETITION.with(PIVOT.range(Range.degrees(0, 131)));
    JointsSimHarness practice = new JointsSimHarness(shortStops.definition());
    Move past = practice.joints.of(PIVOT).goTo(Degrees.of(133));
    practice.run(past, 0.5);

    assertThat(past.outcome()).contains(Outcome.REFUSED);
    JointsSimHarness competition = new JointsSimHarness(ExampleRobot.COMPETITION.definition());
    Move there = competition.joints.of(PIVOT).goTo(Degrees.of(133));
    competition.run(there, 1.5);
    assertThat(there.arrived()).isTrue();
  }

  /**
   * Each robot's joint starts where it rests, its own: the practice robot's arm, stowed at -30°
   * where the competition robot's is at -45°, starts and holds there.
   */
  @Test
  void eachRobotsJointStartsWhereItRests() {
    for (RobotConfig robot : ExampleRobot.all()) {
      double stowed = robot == ExampleRobot.PRACTICE ? -30 : -45;
      JointsSimHarness sim = new JointsSimHarness(robot.definition());
      sim.run(0.5);

      assertThat(Math.toDegrees(sim.simulation.position(ARM)))
          .as(robot.name())
          .isCloseTo(stowed, within(1.0));
      assertThat(sim.joints.of(ARM).isReady().getAsBoolean()).as(robot.name()).isTrue();
    }
  }

  /**
   * The guide's elevator knows where it is from the moment the robot starts, from its CANcoder
   * geared twelve to one down from its drum: it's ready at once, with no homing. On the drum's
   * shaft, which turns ten times over its travel, the CANcoder is refused, saying the gearing that
   * covers the travel, or to home the elevator instead: the refusal the guide quotes.
   */
  @Test
  void theElevatorsEncoderCoversItsTravelInOneTurn() {
    JointsSimHarness sim = new JointsSimHarness(ExampleRobot.COMPETITION.definition());
    sim.loop();
    Joint.Prismatic onTheShaft =
        ELEVATOR
            .driven(
                DrivenBuilder.builder(ELEVATOR.driven()).absoluteEncoder(Optional.empty()).build())
            .absoluteEncoder(CANCODER.can(24).offset(Rotations.of(0.31)));

    assertThat(sim.joints.of(ELEVATOR).state()).isEqualTo(JointMechanism.State.READY);
    assertThatThrownBy(() -> ExampleRobot.COMPETITION.definition().with(onTheShaft))
        .hasMessage(
            "RobotDefinition[example].joints: elevator's absolute encoder turns 10.03 times over"
                + " its travel, 0 m to 1.2 m: one of its turns must cover the travel, with a"
                + " fortieth of it to spare at each end, to tell where elevator is. Gear it to at"
                + " most 0.0947 of its turns for each turn of elevator's drum, or leave it out and"
                + " give elevator's control a homing recipe");
  }

  /**
   * An interlock written in a joint's block is the robot's own, as its definition would list it.
   */
  @Test
  void aBlocksInterlocksAreTheRobotsOwn() {
    assertThat(ROLLER.ownInterlocks())
        .containsExactly(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110))));
    assertThat(ARM.ownInterlocks())
        .containsExactly(
            Interlock.that(ARM)
                .keepsOut(Range.degrees(-10, 30))
                .when(ELEVATOR.atMost(Inches.of(32))));
  }

  /**
   * A joint held by one written after it has its rule in the robot's definition: the elevator, kept
   * out of its lowest 0.3 m while the arm riding on it reaches down past 130°, is taken beside the
   * arm's own rule. Kept out of 0.1 m to 0.6 m while the arm is at most 0° instead, it would hold
   * the arm, resting below the bumper's band, as the arm holds it: a deadlock where the robot
   * starts and where its stowed configuration puts them, refused as the guide quotes it.
   */
  @Test
  void aJointHeldByOneWrittenAfterItHasItsRuleInTheDefinition() {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ExampleRobot.COMPETITION.definition())
            .interlocks(
                List.of(
                    // The elevator keeps out of its lowest 0.3 m while the arm reaches down past
                    // 130°.
                    Interlock.that(ELEVATOR)
                        .keepsOut(Range.meters(0, 0.3))
                        .when(ARM.atLeast(Degrees.of(130)))))
            .build();

    assertThat(robot.everyInterlock())
        .contains(
            Interlock.that(ARM)
                .keepsOut(Range.degrees(-10, 30))
                .when(ELEVATOR.atMost(Inches.of(32))),
            Interlock.that(ELEVATOR)
                .keepsOut(Range.meters(0, 0.3))
                .when(ARM.atLeast(Degrees.of(130))));
    assertThatThrownBy(
            () ->
                RobotDefinitionBuilder.builder(ExampleRobot.COMPETITION.definition())
                    .interlocks(
                        List.of(
                            Interlock.that(ELEVATOR)
                                .keepsOut(Range.meters(0.1, 0.6))
                                .when(ARM.atMost(Degrees.of(0)))))
                    .build())
        .hasMessage(
            "RobotDefinition[example].configurations: configuration stowed leaves arm and elevator"
                + " holding each other by their interlocks, so neither can free the other: a"
                + " deadlock (arm keeps out of -10° to 30° while elevator at most 0.8128 m, so arm"
                + " stays between -60° and -10°; elevator keeps out of 0.1 m to 0.6 m while arm at"
                + " most 0°, so elevator stays between 0 m and 0.1 m)\n"
                + "RobotDefinition[example].interlocks: as the robot starts, it leaves arm and"
                + " elevator holding each other by their interlocks, so neither can free the other:"
                + " a deadlock (arm keeps out of -10° to 30° while elevator at most 0.8128 m, so arm"
                + " stays between -60° and -10°; elevator keeps out of 0.1 m to 0.6 m while arm at"
                + " most 0°, so elevator stays between 0 m and 0.1 m)");
  }

  /**
   * A place is a name on its joint, and its own at a value it has, or had before it was re-placed:
   * the feeder and the indexer each have a "feeding", at 1000 and 1500 rpm, and {@code
   * feeder.run(INDEXER_FEEDING)}, a slip between two constants, would otherwise run the feeder at
   * its own without a word. It's refused, and so is the indexer's feeding in a gate on the feeder
   * or in a configuration's move, saying where the feeder's is. Its own constant runs, and on a
   * practice robot that re-valued it, the competition robot's constant runs the practice robot's
   * own.
   */
  @Test
  void anotherJointsPlaceOfTheSameNameIsRefused() {
    JointsSimHarness sim = new JointsSimHarness(ExampleRobot.COMPETITION.definition());
    String why =
        "feeder's feeding runs at 1000 rpm, and this feeding, at 1500 rpm, isn't one it has had:"
            + " another joint's or another robot's; give feeder's own";

    assertThatThrownBy(() -> sim.joints.of(FEEDER).run(INDEXER_FEEDING))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage(why);
    assertThatThrownBy(() -> sim.joints.of(FEEDER).isAt(INDEXER_FEEDING))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage(why);
    assertThatThrownBy(() -> sim.joints.gate(FEEDER.at(INDEXER_FEEDING)))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage(why);
    assertThatThrownBy(
            () -> sim.joints.run(Configuration.of("feeding", FEEDER.at(INDEXER_FEEDING))))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage(why);
    assertThat(sim.joints.of(FEEDER).run(FEEDER_FEEDING)).isNotNull();

    JointsSimHarness practice =
        new JointsSimHarness(
            ExampleRobot.COMPETITION
                .definition()
                .with(FEEDER.replaced(FEEDER_FEEDING, RPM.of(1200))));
    assertThatThrownBy(() -> practice.joints.of(FEEDER).run(INDEXER_FEEDING))
        .hasMessageStartingWith("feeder's feeding runs at 1200 rpm, and this feeding, at 1500 rpm");
    Move feeding = practice.joints.of(FEEDER).run(FEEDER_FEEDING);
    practice.run(feeding, 1);
    assertThat(practice.simulation.velocity(FEEDER))
        .as("its own feeding, 1200 rpm")
        .isCloseTo(
            RPM.of(1200).in(RadiansPerSecond), within(0.06 * RPM.of(1200).in(RadiansPerSecond)));
  }

  /**
   * Two joints' places of one name and value are each one's own: the elevator's "down" and a
   * hopper's, each its own constant, are both 0 m, as two joints' often are. On a practice robot
   * whose elevator sits a centimeter up, each constant still drives its joint to its own: the
   * elevator's to 1 cm, the hopper's to 0, and either one is either joint's, since nothing tells
   * them apart.
   */
  @Test
  void twoJointsPlacesOfOneNameAndValueEachDriveTheirOwn() {
    RobotDefinition practice =
        withHopper().with(ELEVATOR.replaced(ELEVATOR_DOWN, Centimeters.of(1)));
    JointsSimHarness sim = new JointsSimHarness(practice);
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    JointMechanism hopper = sim.joints.of(HOPPER);
    Gate elevatorDown = elevator.isAt(ELEVATOR_DOWN);

    Move up = elevator.goTo(ELEVATOR_CLEAR);
    sim.scheduler.schedule(hopper.goTo(HOPPER_CLEAR));
    sim.run(up, 3);
    Move down = elevator.goTo(ELEVATOR_DOWN);
    Move hopperDown = hopper.goTo(HOPPER_DOWN);
    sim.scheduler.schedule(hopperDown);
    sim.run(down, 3);

    assertThat(up.arrived()).isTrue();
    assertThat(down.arrived()).isTrue();
    assertThat(hopperDown.arrived()).isTrue();
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0.01, within(0.005));
    assertThat(sim.simulation.position(HOPPER)).isCloseTo(0, within(0.005));
    assertThat(elevatorDown.getAsBoolean()).isTrue();
    assertThat(elevator.goTo(HOPPER_DOWN)).as("the same down, either joint's").isNotNull();
    assertThat(hopper.goTo(ELEVATOR_DOWN)).isNotNull();
  }

  /**
   * Another joint's place of a name this joint has, elsewhere, is refused wherever it's given, as a
   * move to it is: the hopper's "clear" in the elevator's move, its gate, a gate on a condition on
   * it, or a configuration's move, each saying where the elevator's clear is.
   */
  @Test
  void anotherJointsPlaceIsRefusedWhereverItsGiven() {
    JointsSimHarness sim = new JointsSimHarness(withHopper());
    String why =
        "elevator's clear is at 0.8128 m, and this clear, at 0.1 m, isn't one it has had: another"
            + " joint's or another robot's; give elevator's own";

    assertThatThrownBy(() -> sim.joints.of(ELEVATOR).goTo(HOPPER_CLEAR)).hasMessage(why);
    assertThatThrownBy(() -> sim.joints.of(ELEVATOR).isAt(HOPPER_CLEAR)).hasMessage(why);
    assertThatThrownBy(() -> sim.joints.gate(ELEVATOR.atMost(HOPPER_CLEAR))).hasMessage(why);
    assertThatThrownBy(() -> sim.joints.goTo(Configuration.of("clear", ELEVATOR.at(HOPPER_CLEAR))))
        .hasMessage(why);
  }

  /** The guide's competition robot, the hopper beside its elevator. */
  private static RobotDefinition withHopper() {
    RobotDefinition competition = ExampleRobot.COMPETITION.definition();
    List<Joint> joints = new ArrayList<>(competition.joints());
    joints.add(HOPPER);
    return RobotDefinitionBuilder.builder(competition).joints(joints).build();
  }

  /** The guide's test of one mechanism alone: the pivot swings out. */
  @Test
  void thePivotSwingsOut() {
    RobotDefinition definition = ExampleRobot.COMPETITION.definition();
    JointsSimHarness sim = new JointsSimHarness(definition);
    Move out = sim.joints.of(PIVOT).goTo(PIVOT_OUT);
    sim.run(out, 1.5);
    assertThat(out.arrived()).isTrue();
    assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
  }

  /** The guide's recovery: deployed, it doesn't back off. */
  @Test
  void aDeployThatArrivesStaysOut() {
    JointsSimHarness sim = new JointsSimHarness(ExampleRobot.COMPETITION.definition());
    ExampleCommands commands = new ExampleCommands(sim.scheduler, sim.joints, () -> 0, () -> 3);

    sim.run(commands.deployOrBackOff(), 1.5);

    assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
  }
}

package com.michaelgrundvig.frc.runtime.examples;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Experiments;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.experiment.StepResponse;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.ControllerSettings;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointExperiments;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.system.RobotController;

/**
 * The guide's robot tuned by experiment with nothing written per joint: each joint's experiments
 * come from its own places and speeds, and run on the joint as its commands would.
 */
@ExtendWith(RobotSim.class)
class ExampleRobotExperimentsTest {
  /**
   * A step between each two places, in the order the block names them, and one up to each speed:
   * the hood's lob and the shooter's shooting give both a step. The roller's outtaking is a
   * voltage, with no loop to tune, so it has none.
   */
  @Test
  void eachJointStepsBetweenItsOwnPlacesAndSpeeds() {
    JointsSimHarness sim = new JointsSimHarness(ExampleRobot.COMPETITION.definition());
    List<Experiment> made = made(ExampleRobot.COMPETITION.definition(), sim.joints);

    assertThat(made)
        .extracting(Experiment::name)
        .containsExactly(
            "pivot: in to out",
            "roller: up to intaking",
            "hood: down to lob",
            "elevator: down to clear",
            "arm: stowed to scoring",
            "feeder: up to feeding",
            "indexer: up to feeding",
            "shooter: up to shooting");
    assertThat(made.get(3).rig().gains())
        .as("the joint's own gains, as Tune names them")
        .containsExactly("gains.kP", "gains.kD", "gains.kS");
    Experiments page = experiments();
    made.forEach(page::add); // refused if a step didn't fit its rig, or ran too long
    assertThat(page.rigs()).hasSize(8);
  }

  private static final Position WRIST_STOWED = Position.degrees("stowed", 0);

  /** A wrist with one place: aimed by a computed angle, it names nothing to step to. */
  private static final Joint.Revolute WRIST =
      Joint.pivot("wrist", Part.CHASSIS, new Translation3d(0.3, 0, 0.3), Joint.PITCH_UP)
          .range(Range.degrees(0, 90))
          .rest(WRIST_STOWED)
          .positions(WRIST_STOWED)
          .motors(Motor.KRAKEN_X44.through(Gearing.of(50)))
          .control(MechanismControl.position(30).withHoming(Homing.assumeAtRest()));

  /** A joint with one place and no speed has nothing to step between, so none. */
  @Test
  void aJointWithNothingToStepBetweenHasNone() {
    RobotDefinition competition = ExampleRobot.COMPETITION.definition();
    List<Joint> joints = new ArrayList<>(competition.joints());
    joints.add(WRIST);
    RobotDefinition withWrist = RobotDefinitionBuilder.builder(competition).joints(joints).build();
    JointsSimHarness sim = new JointsSimHarness(withWrist);

    assertThat(made(withWrist, sim.joints))
        .hasSize(8)
        .extracting(Experiment::name)
        .noneMatch(name -> name.startsWith("wrist"));
  }

  /**
   * The elevator's step runs on the elevator as a move would: up to clear and back down, each jump
   * risen, settled, and arrived.
   */
  @Test
  void aStepBetweenPlacesMovesTheJointThereAndBack() {
    List<StepResponse> jumps = jumps("elevator: down to clear", "Position");

    assertThat(jumps).hasSize(2);
    for (StepResponse jump : jumps) {
      assertThat(jump.riseSeconds()).isPresent();
      assertThat(jump.settlingSeconds()).isPresent();
      assertThat(jump.steadyStateError()).isCloseTo(0, within(0.01)); // meters
    }
  }

  /** The shooter's step spins it up to its shooting speed on its velocity loop, and back. */
  @Test
  void aStepToASpeedSpinsTheJointUpAndBack() {
    List<StepResponse> jumps = jumps("shooter: up to shooting", "Speed");

    assertThat(jumps).hasSize(2);
    assertThat(jumps.get(0).riseSeconds()).as("up to 3000 rpm").isPresent();
    assertThat(jumps.get(0).steadyStateError()).isCloseTo(0, within(150.0)); // rpm
  }

  /** Runs one experiment on the competition robot's joints to its end: its jumps' scores. */
  private static List<StepResponse> jumps(String name, String measurement) {
    RobotDefinition definition = ExampleRobot.COMPETITION.definition();
    JointsSimHarness sim = new JointsSimHarness(definition);
    Experiments page = experiments();
    made(definition, sim.joints).forEach(page::add);
    page.requestStart(name, null);
    Actions.Gate utility = new Actions.Gate(true, true, false);
    int loops = RobotLoop.loops(Experiments.LONGEST_SECONDS + 1);
    for (int i = 0; i < loops; i++) {
      page.heard(null); // the page keeping it going, as its heartbeats do
      page.periodic(utility);
      sim.loop();
      Run run = page.run();
      if (run != null && run.state().phase() != Run.Phase.RUNNING) {
        break;
      }
    }
    Run run = Objects.requireNonNull(page.run());
    assertThat(run.state().phase()).as(run.state().detail()).isEqualTo(Run.Phase.ENDED);
    return Objects.requireNonNull(
            Objects.requireNonNull(run.state().scores()).steps().get(measurement))
        .stream()
        .map(Optional::orElseThrow)
        .toList();
  }

  /** The experiments the robot program adds, from each driven joint's tuning as it publishes it. */
  private static List<Experiment> made(RobotDefinition definition, Joints joints) {
    Map<String, LiveConfig<MechanismControl>> tunings = new LinkedHashMap<>();
    for (Joint joint : Joints.driven(definition)) {
      tunings.put(joint.label(), JointMechanism.tuning(joint, ControllerSettings.control(joint)));
    }
    return JointExperiments.of(joints, tunings);
  }

  private static Experiments experiments() {
    return new Experiments(
        new Experiments.Robot("example", true, ""),
        inputs -> {},
        RobotController::getTime,
        false,
        ended -> {});
  }
}

package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_PUSH;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * What a team's code builds on the joints that can't be, refused by one rule ({@code Refusals}):
 * built as the robot starts, as its bindings and the autos built with them are, it throws, saying
 * why, so the robot doesn't start and the test that builds every robot fails; built once the robot
 * runs, inside a command, it's a stand-in that says the same, a move that ends REFUSED and a gate
 * that holds false, and the robot carries on. Moves to places of another joint's are {@link
 * JointMovesSimTest}'s.
 */
@ExtendWith(RobotSim.class)
class RefusalsSimTest {
  /** A hatch the robot has, which nothing drives: the template has no mechanism of it. */
  private static final Joint.Revolute HATCH =
      JointRevoluteBuilder.builder(PIVOT)
          .label("hatch")
          .origin(new Transform3d(new Translation3d(-0.2, 0, 0.3), Rotation3d.ZERO))
          .link(JointFixtures.link("hatch", 1, new Translation3d()))
          .driven(Driven.NONE)
          .build();

  /** A joint of no robot's. */
  private static final Joint.Revolute STRANGER =
      JointRevoluteBuilder.builder(PIVOT).label("stranger").build();

  private static final RobotDefinition WITH_HATCH =
      RobotDefinitionBuilder.builder(ROBOT)
          .joints(List.of(PIVOT, ROLLER, ELEVATOR, TURRET, HATCH))
          .configurations(List.of(INTAKING))
          .build();

  private final JointsSimHarness sim = new JointsSimHarness(WITH_HATCH);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);
  private final JointMechanism elevator = sim.joints.of(ELEVATOR);

  /** What can't be, each with why: what a move built of it ends REFUSED for, or a gate is false. */
  private record CantBe(String why, Supplier<Object> built) {}

  private List<CantBe> whatCantBe() {
    return List.of(
        new CantBe(
            "a share of its cruise speed is above 0% and at most 100%, not 150%",
            () -> pivot.goTo(PIVOT_OUT).atSpeed(Percent.of(150))),
        new CantBe(
            "it isn't a move to a place, so it has no profile to run at a share of",
            () -> roller.run(ROLLER_INTAKING).atSpeed(Percent.of(50))),
        new CantBe(
            "it never ends on its own, so it has no deadline: use withTimeout",
            () -> roller.run(ROLLER_INTAKING).within(Seconds.of(1))),
        new CantBe(
            "it never ends on its own, so it has no deadline: use withTimeout",
            () -> pivot.hold().within(Seconds.of(1))),
        new CantBe(
            "it never ends on its own, so it has no deadline: use withTimeout",
            () -> pivot.follow(() -> Degrees.of(60)).within(Seconds.of(1))),
        new CantBe("pivot turns: send it to an angle", () -> pivot.goTo(Meters.of(1))),
        new CantBe("elevator slides: send it to a distance", () -> elevator.goTo(Degrees.of(1))),
        new CantBe(
            "pivot has no position clear at 0.4 m: it has in, run, out",
            () -> pivot.isAt(ELEVATOR_CLEAR)),
        new CantBe(
            "hatch has no mechanism: nothing drives it",
            () -> sim.joints.gate(HATCH.atLeast(PIVOT_OUT))),
        new CantBe(
            "pivot has no position clear at 0.4 m: it has in, run, out",
            () -> sim.joints.gate(PIVOT.between(PIVOT_OUT, ELEVATOR_CLEAR))),
        new CantBe(
            "pivot has no position clear at 0.4 m: it has in, run, out",
            () -> sim.joints.goTo(Configuration.of("wrong", PIVOT.at(ELEVATOR_CLEAR)))),
        new CantBe(
            "roller has no output push: it has intaking, outtaking, ejecting, and stopped",
            () -> roller.isAt(PIVOT_PUSH)),
        new CantBe("hatch has no mechanism: nothing drives it", () -> sim.joints.of(HATCH)),
        new CantBe("stranger isn't one of joints' joints", () -> sim.joints.of(STRANGER)));
  }

  /** As the robot starts, each throws, saying why: after the move's name, for a move's option. */
  @Test
  void eachBuiltAsTheRobotStartsThrows() {
    for (CantBe each : whatCantBe()) {
      assertThatThrownBy(each.built()::get)
          .isInstanceOf(IllegalArgumentException.class)
          .hasMessageEndingWith(each.why());
    }
    assertThatThrownBy(() -> pivot.goTo(PIVOT_OUT).atSpeed(Percent.of(150)))
        .hasMessage(
            "pivot → out: a share of its cruise speed is above 0% and at most 100%, not 150%");
    assertThatThrownBy(() -> roller.run(ROLLER_INTAKING).within(Seconds.of(1)))
        .hasMessage(
            "roller: intaking: it never ends on its own, so it has no deadline: use withTimeout");
  }

  /**
   * Built inside a running command, each is a stand-in saying the same: a move ends REFUSED as it
   * starts, a gate holds false, a joint with no mechanism has a stand-in whose every move is
   * refused and every gate false. The command runs on to its end, and the pivot then goes out.
   */
  @Test
  void eachBuiltWhileTheRobotRunsIsRefusedAndTheRobotCarriesOn() {
    List<Object> built = new ArrayList<>();
    List<Move> standIns = new ArrayList<>();
    List<Gate> standInGates = new ArrayList<>();
    boolean[] ended = {false};
    Command wrong =
        Command.requiring(pivot, roller, elevator)
            .executing(
                co -> {
                  for (CantBe each : whatCantBe()) {
                    built.add(each.built().get());
                  }
                  for (Object each : built) {
                    if (each instanceof Move move) {
                      co.await(move);
                    } else if (each instanceof JointMechanism standIn) {
                      for (Move move :
                          List.of(
                              standIn.goTo(PIVOT_OUT),
                              standIn.goTo(Degrees.of(30)),
                              standIn.hold(),
                              standIn.home(),
                              standIn.follow(() -> Degrees.of(30)))) {
                        standIns.add(move);
                        co.await(move);
                      }
                      standInGates.add(standIn.isAt(PIVOT_OUT));
                      standInGates.add(standIn.isReady());
                    }
                  }
                  ended[0] = true;
                })
            .named("What can't be");

    sim.run(wrong, 0.5);
    Move out = pivot.goTo(PIVOT_OUT);
    sim.run(out, 2);

    List<CantBe> cantBe = whatCantBe();
    for (int i = 0; i < cantBe.size(); i++) {
      String why = cantBe.get(i).why();
      switch (built.get(i)) {
        case Move move -> {
          assertThat(move.outcome()).as(why).contains(Outcome.REFUSED);
          assertThat(move.reason()).isEqualTo(move.name() + ": REFUSED: " + why);
        }
        case Gate gate -> {
          assertThat(gate.getAsBoolean()).as(why).isFalse();
          assertThat(gate.blocking()).containsExactly(why);
        }
        case JointMechanism standIn -> {
          assertThat(standIn.state()).as(why).isEqualTo(JointMechanism.State.FAULTED);
          assertThat(standIn.getName()).isEqualTo(why.substring(0, why.indexOf(' ')));
        }
        default -> throw new AssertionError(built.get(i));
      }
    }
    assertThat(standIns).hasSize(10);
    for (Move move : standIns) {
      assertThat(move.outcome()).contains(Outcome.REFUSED);
    }
    assertThat(standIns.get(0).reason())
        .isEqualTo("hatch → out: REFUSED: hatch has no mechanism: nothing drives it");
    assertThat(standIns.get(5).reason())
        .isEqualTo("stranger → out: REFUSED: stranger isn't one of joints' joints");
    assertThat(standInGates).allMatch(gate -> !gate.getAsBoolean());
    assertThat(standInGates.get(1).why())
        .isEqualTo("hatch ready is false: hatch has no mechanism: nothing drives it");
    assertThat(sim.joints.of(HATCH))
        .as("one stand-in a joint")
        .isSameAs(built.get(cantBe.size() - 2));
    assertThat(ended[0]).as("the command ran on to its end").isTrue();
    assertThat(out.arrived()).isTrue();
    assertThat(sim.simulation.position(PIVOT)).isCloseTo(Math.toRadians(130), within(0.03));
  }
}

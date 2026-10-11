package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_BACK;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Outcome;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command3.Scheduler;
import org.wpilib.system.RobotController;

/**
 * A joint whose position stops being a number while a move has it following its profile, in the
 * simulator. The joint's motor controller is never told anything that isn't a number: the joint
 * goes neutral, its move ends saying why, and its doubt says so until it reads a number again,
 * where it then holds. One scenario, run for each way a joint's position reaches its controller.
 */
@ExtendWith(RobotSim.class)
class JointPositionLostSimTest {
  /** The pivot's through-bore on port 1, which its loop closes on. */
  private static final MechanismWiring.PortEncoder PORT_1 =
      MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.3)).inverted();

  private static final Joint.Revolute PIVOT_ON_ITS_ENCODER =
      PIVOT
          .absoluteEncoder(PORT_1)
          .retuned(old -> control(30, 1, profile(8, 30)))
          .absoluteEncoder(encoder -> encoder.on(PORT_1.closesLoop()));

  /** Whether a joint left holding can go on holding there while its position isn't a number. */
  private enum LeftHolding {
    /** Its motor controller holds it on the motor's own count, which needs no reading here. */
    HOLDS_ON,
    /** Holding it takes its reading, so without one it goes neutral. */
    GOES_NEUTRAL
  }

  static Stream<Arguments> joints() {
    return Stream.of(
        argumentSet(
            "a pivot whose motor controller closes its loop",
            ROBOT,
            PIVOT,
            PIVOT_OUT,
            LeftHolding.HOLDS_ON),
        argumentSet(
            "a pivot whose loop the robot closes on its encoder",
            ROBOT.with(PIVOT_ON_ITS_ENCODER),
            PIVOT_ON_ITS_ENCODER,
            PIVOT_OUT,
            LeftHolding.GOES_NEUTRAL),
        argumentSet(
            "a turret, which goes the short way round",
            ROBOT,
            TURRET,
            TURRET_BACK,
            LeftHolding.GOES_NEUTRAL));
  }

  /** A joint's simulated absolute encoder, which a test can make read as not a number. */
  private static final class LosableEncoder implements AbsoluteEncoderIO {
    private final AbsoluteEncoderIO encoder;
    boolean readsNoNumber;

    LosableEncoder(AbsoluteEncoderIO encoder) {
      this.encoder = encoder;
    }

    @Override
    public void updateInputs(AbsoluteEncoderIOInputs inputs) {
      encoder.updateInputs(inputs);
      if (readsNoNumber) {
        inputs.share = Double.NaN;
      }
    }
  }

  /** What the joint did over some loops: what held it each loop, and each doubt said of it. */
  private record Did(List<String> heldBy, List<String> doubts) {}

  /** One robot of the scenario: its joints on the simulator, with the one joint's readings. */
  private static final class Scene implements AutoCloseable {
    final Scheduler scheduler = Scheduler.createIndependentScheduler();
    final JointSimulation simulation;
    final Joints joints;
    final JointMechanism joint;
    Silenceable io = new Silenceable(new JointIO() {});
    LosableEncoder encoder = new LosableEncoder(new AbsoluteEncoderIO() {});

    Scene(RobotDefinition robot, Joint lost) {
      simulation = new JointSimulation(robot);
      joints =
          new Joints(
              robot,
              new Joints.Hardware() {
                @Override
                public JointIO joint(Joint joint) {
                  Silenceable wrapped = new Silenceable(simulation.io(joint));
                  if (joint.equals(lost)) {
                    io = wrapped;
                  }
                  return wrapped;
                }

                @Override
                public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
                  LosableEncoder wrapped = new LosableEncoder(simulation.absoluteEncoder(joint));
                  if (joint.equals(lost)) {
                    encoder = wrapped;
                  }
                  return wrapped;
                }
              },
              Joints.untuned(),
              scheduler);
      joint = joints.of(lost);
    }

    /** From now on the joint's position, however it is read, is or isn't a number. */
    void readsNoNumber(boolean lost) {
      io.readsNoNumber = lost;
      encoder.readsNoNumber = lost;
    }

    Did run(int loops) {
      Set<String> heldBy = new LinkedHashSet<>();
      Set<String> doubts = new LinkedHashSet<>();
      for (int i = 0; i < loops; i++) {
        RobotSim.stepClock();
        simulation.step(RobotLoop.PERIOD_SECONDS);
        joints.read(RobotController.getTime(), true);
        scheduler.run();
        joints.write();
        heldBy.add(joint.heldBy());
        doubts.addAll(joints.switchDoubts());
      }
      return new Did(List.copyOf(heldBy), List.copyOf(doubts));
    }

    @Override
    public void close() {
      joints.close();
    }
  }

  /**
   * The joint is sent to a place. Part of the way there its position stops being a number: the move
   * ends REFUSED in that loop, saying so, and the joint is neutral with nowhere to hold, its doubt
   * raised. Reading a number again, it holds where it is, and a new move takes it to the place.
   * Nothing that isn't a number is ever told to its motor controller. Until 2026-10-11 the reading
   * went on into the joint's profile, and from there a turret's controller was told a position that
   * wasn't a number, and a joint closed on its encoder a voltage that wasn't one.
   */
  @ParameterizedTest
  @MethodSource("joints")
  void aJointWhosePositionStopsBeingANumberMidMoveGoesNeutralAndItsMoveSaysSo(
      RobotDefinition robot, Joint lost, Position place, LeftHolding leftHolding) {
    try (Scene scene = new Scene(robot, lost)) {
      String label = lost.label();
      String doubt = doubtOf(label);
      Move there = scene.joint.goTo(place);
      scene.run(5);
      double started = scene.simulation.position(lost);
      scene.scheduler.schedule(there);
      Did moving = scene.run(10);
      double partWay = scene.simulation.position(lost);

      scene.readsNoNumber(true);
      Did lostFor = scene.run(10);
      scene.readsNoNumber(false);
      Did found = scene.run(25);
      Move again = scene.joint.goTo(place);
      scene.scheduler.schedule(again);
      scene.run(RobotLoop.loops(3));

      assertThat(Math.abs(partWay - started))
          .as("it was part of the way there")
          .isGreaterThan(0.01);
      assertThat(moving).isEqualTo(new Did(List.of(""), List.of()));
      assertThat(there.outcome()).contains(Outcome.REFUSED);
      assertThat(there.reason())
          .isEqualTo(
              there.name()
                  + ": REFUSED: can't go on: "
                  + label
                  + "'s position stopped being a number: check its encoder and its motor"
                  + " controller's settings");
      assertThat(lostFor)
          .as("neutral with nowhere to hold")
          .isEqualTo(new Did(List.of(label + "'s position isn't a number"), List.of(doubt)));
      assertThat(found.heldBy()).as("it holds again").endsWith("");
      assertThat(found.doubts()).isEmpty();
      assertThat(again.arrived()).as("a new move gets there").isTrue();
      assertThat(scene.simulation.position(lost))
          .isCloseTo(place.value(), within(Math.toRadians(3)));
      assertThat(scene.io.notNumbers).as("told its motor controller").isEmpty();
    }
  }

  private static String doubtOf(String label) {
    return label
        + "'s position reads NaN, which isn't a number: "
        + label
        + " is neutral while it has nowhere to hold; check its encoder and its motor"
        + " controller's settings";
  }

  /**
   * The joint is left holding where it rests, with no move on it, when its position stops being a
   * number. Its motor controller is still never told anything that isn't a number. A joint its
   * motor controller holds on the motor's own count holds on. A joint that needs its reading to be
   * held (the robot closes its loop, or its goal is the turn nearest where it reads) goes neutral,
   * saying why. Either holds where it is once it reads a number again.
   */
  @ParameterizedTest
  @MethodSource("joints")
  void aJointLeftHoldingWhosePositionStopsBeingANumberIsNeverToldOne(
      RobotDefinition robot, Joint lost, Position place, LeftHolding leftHolding) {
    try (Scene scene = new Scene(robot, lost)) {
      String label = lost.label();
      Did holding = scene.run(5);

      scene.readsNoNumber(true);
      Did lostFor = scene.run(10);
      scene.readsNoNumber(false);
      Did found = scene.run(25);

      String heldBy =
          switch (leftHolding) {
            case HOLDS_ON -> "";
            case GOES_NEUTRAL -> label + "'s position isn't a number";
          };
      assertThat(holding).isEqualTo(new Did(List.of(""), List.of()));
      assertThat(lostFor).isEqualTo(new Did(List.of(heldBy), List.of(doubtOf(label))));
      assertThat(found.heldBy()).as("it holds again").endsWith("");
      assertThat(found.doubts()).isEmpty();
      assertThat(scene.joint.goal())
          .as("it holds where it is")
          .isCloseTo(scene.simulation.position(lost), within(Math.toRadians(1)));
      assertThat(scene.io.notNumbers).as("told its motor controller").isEmpty();
    }
  }
}

package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * What the robot's definition doesn't know is said where it costs: what the robot's code reads,
 * wherever the program runs; what only the simulator reads, a link's mass, in the simulator alone.
 */
class DefinitionDoubtsTest {
  /** An arm whose plate nobody weighed, its kG not written. */
  private static final Joint.Revolute ARM =
      Joint.pivot("arm", Part.CHASSIS, new Translation3d(0.2, 0, 0.4), Joint.PITCH_UP)
          .range(Range.degrees(0, 90))
          .rest(Degrees.of(0))
          .carries(
              Link.box(
                  "arm plate", new Translation3d(0.2, 0, 0), new Translation3d(0.4, 0.05, 0.05)))
          .motors(Motor.KRAKEN_X60.through(Gearing.of(50)))
          .control(
              MechanismControl.position(30).withHoming(MechanismControl.Homing.assumeAtRest()));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(ExampleRobot.SIM.definition())
          .name("doubts")
          .joints(List.of(ARM))
          .build();

  /**
   * The arm's missing kG is the robot's doubt; its plate's missing mass, the body's pitch and roll,
   * and the mass in no link are the simulator's alone.
   */
  @Test
  void theRobotSaysWhatItsCodeReadsAndTheSimulatorWhatOnlyItReads() {
    List<String> doubts = Doubts.lines(Doubts.of(ROBOT), Finding.Scope.ROBOT);
    List<String> notes = Doubts.lines(Doubts.of(ROBOT), Finding.Scope.SIMULATOR);

    assertThat(doubts)
        .contains(
            "not known: arm has no kG: it holds against gravity by kP alone, so it sits below its"
                + " targets until its kG is tuned and written, withKg(...), or written as 0 for a"
                + " counterbalanced joint")
        .noneMatch(doubt -> doubt.contains("mass:") || doubt.contains("in no link"));
    assertThat(notes)
        .allMatch(note -> note.startsWith("simulator: "))
        .contains(
            "simulator: arm plate has no mass: arm's gravity comes from its kG, and arm plate adds"
                + " nothing to arm's inertia",
            "simulator: the body's pitch and roll are a box's, beside its given yaw: how the"
                + " simulator tips the robot is approximate")
        .anyMatch(note -> note.contains("is in no link"));
  }

  /**
   * The template's robot leaves its tread's grip and its modules' inertias to the defaults: the
   * grip is the robot's doubt, since its drive moves and slip check read it; the inertias are the
   * simulator's notes. Its center of mass is given, so that's no note.
   */
  @Test
  void theTemplatesUnmeasuredGripIsTheRobotsDoubtAndItsInertiasTheSimulators() {
    RobotDefinition template = ExampleRobot.SIM.definition();
    List<String> doubts = Doubts.lines(Doubts.of(template), Finding.Scope.ROBOT);
    List<String> notes = Doubts.lines(Doubts.of(template), Finding.Scope.SIMULATOR);

    assertThat(doubts)
        .contains(
            "not known: the tread's grip isn't measured: it's 1.2, PathPlanner's default: drive"
                + " moves plan their acceleration on it, and the slip check judges by it: measure it"
                + " with a pull test (docs/characterization.md, tread friction)")
        .noneMatch(doubt -> doubt.contains("inertia isn't measured"));
    assertThat(notes)
        .contains(
            "simulator: the drive's own inertia isn't measured: it's 0.001 kg m², an estimate for"
                + " a 4 in wheel and a Kraken X60's rotor: how fast the simulator's wheels spin up"
                + " is a guess, though a small one beside the robot's mass they carry",
            "simulator: the steering's inertia isn't measured: it's 0.004 kg m², Phoenix Tuner X's"
                + " simulation value: how fast the simulator's modules steer at their current limit"
                + " is a guess")
        .noneMatch(note -> note.contains("grip") || note.contains("center of mass"));
  }

  /**
   * The template's roller, feeder and indexer have no mass, so the simulator turns each on its
   * motors' rotors alone, and the note says how soon they bring it to its fastest speed. Its
   * shooter carries a flywheel, so it has no note.
   */
  @Test
  void theTemplatesRollersSayHowFastTheirRotorsAloneSpinThemUp() {
    RobotDefinition example = ExampleRobot.COMPETITION.definition();
    List<String> notes = Doubts.lines(Doubts.of(example), Finding.Scope.SIMULATOR);

    assertThat(notes)
        .contains(
            "simulator: roller has no mass: roller adds nothing to roller's inertia, so its rotors"
                + " alone bring it to intaking, 2000 rpm, in about 11 ms",
            "simulator: feeder has no mass: feeder adds nothing to feeder's inertia, so its rotors"
                + " alone bring it to feeding, 1000 rpm, in about 7 ms",
            "simulator: indexer has no mass: indexer adds nothing to indexer's inertia, so its"
                + " rotors alone bring it to feeding, 1500 rpm, in about 8 ms")
        .noneMatch(note -> note.contains("shooter has no mass"));
  }

  /** A kG written, even 0 for a counterbalanced arm, is no doubt. */
  @Test
  void aWrittenKgIsNoDoubt() {
    RobotDefinition tuned = ROBOT.with(ARM.retuned(control -> control.withKg(0)));

    assertThat(Doubts.lines(Doubts.of(tuned), Finding.Scope.ROBOT))
        .noneMatch(doubt -> doubt.contains("kG"));
  }

  /**
   * With a PDH declared, a wired motor controller that doesn't say which channel feeds it is the
   * robot's doubt. The template's robot declares none, so it says nothing of feeds.
   */
  @Test
  void aWiredControllerWithNoChannelIsADoubtOnceThereIsAPdh() {
    RobotDefinition wired =
        RobotDefinitionBuilder.builder(
                ROBOT.with(ARM.motors(motors -> motors.on(TALON_FX.can(32)))))
            .pdh(Pdh.can(1))
            .build();

    assertThat(Doubts.lines(Doubts.of(wired), Finding.Scope.ROBOT))
        .contains(
            "not known: arm's motor controller 32 has no PDH channel: which channel feeds each, and"
                + " through what breaker, isn't checked: give each its own where it's wired,"
                + " .pdh(channel, breaker)");
    assertThat(Doubts.lines(Doubts.of(ExampleRobot.SIM.definition()), Finding.Scope.ROBOT))
        .noneMatch(doubt -> doubt.contains("PDH"));
  }
}

package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveMove;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * What disabling the robot does to a command that is running, and what enabling it again does: the
 * guide's robot with the guide's controls, one scenario a test.
 *
 * <p>Disabling cancels nothing. WPILib's v3 scheduler runs every command in every mode, and the
 * runtime ends only one command itself, the auto, in the loop autonomous ends. So a command that is
 * running when the robot is disabled is still running while it is disabled: its coroutine takes its
 * turns, its deadline's clock runs, and its gamepad binding is still polled. What changes is
 * beneath it. A disabled joint is not driven, whatever its command asks, and reads as not at its
 * goal. When the robot is enabled again, an idle joint holds where it is then, and a joint whose
 * command is still running is driven where that command asks, with no new press.
 *
 * <p>Some of that is surprising. Those scenarios say "under review" where they pin it: they record
 * what the robot does today, which is not a decision that it should.
 */
@SimulationTest
class DisablingSimTest {
  private static final double RPM = 2 * Math.PI / 60;

  @RegisterExtension final RobotSim sim = new RobotSim();

  /** The guide's robot with the guide's controls, enabled in teleop, a few loops in. */
  private JointSimulation enabledInTeleop() {
    sim.build(ExampleRobot.COMPETITION, SensorErrors.NONE);
    sim.controls(ExampleControls::new);
    RobotSim.setEnabled(true);
    sim.run(0.1);
    return Objects.requireNonNull(sim.world().joints());
  }

  private void disabledFor(double seconds) {
    RobotSim.setEnabled(false);
    sim.run(seconds);
  }

  private void enabledAgainFor(double seconds) {
    RobotSim.setEnabled(true);
    sim.run(seconds);
  }

  private void holdTheOperatorsTrigger(boolean held) {
    sim.operator().setRightTriggerAxis(held ? 1 : 0);
    sim.operator().notifyNewData();
  }

  private void holdTheDriversTrigger(boolean held) {
    sim.driver().setRightTriggerAxis(held ? 1 : 0);
    sim.driver().notifyNewData();
  }

  private void pushTheDriversStickForward() {
    sim.driver().setLeftY(-1.0); // gamepad Y is negative when pushed forward
    sim.driver().notifyNewData();
  }

  /** How far the simulated robot really is from a pose, in meters. */
  private double metersFrom(Pose2d pose) {
    return sim.world().truePose().getTranslation().getDistance(pose.getTranslation());
  }

  /** The names of the commands running now. */
  private List<String> running() {
    return sim.scheduler().getRunningCommands().stream().map(Command::name).toList();
  }

  private static double degrees(double radians) {
    return Math.toDegrees(radians);
  }

  // ---- A joint's move ----

  /**
   * A move running when the robot is disabled is not canceled, and writes no outcome. Its joint
   * goes neutral. Enabled again before the move's deadline, the move carries on to where it was
   * going.
   */
  @Test
  void aJointMoveRunningAtDisableCarriesOnWhenTheRobotIsEnabledAgain() {
    JointSimulation truth = enabledInTeleop();
    JointMechanism pivot = sim.joints().of(PIVOT);
    Move out = pivot.goTo(PIVOT_OUT);
    sim.scheduler().schedule(out);
    sim.run(0.2);

    disabledFor(0.5);
    List<String> runningWhileDisabled = running();
    boolean endedWhileDisabled = out.outcome().isPresent();
    double whereItWasLeft = degrees(truth.position(PIVOT));
    enabledAgainFor(1);

    assertThat(runningWhileDisabled).contains("pivot → out");
    assertThat(endedWhileDisabled).as("no outcome while disabled").isFalse();
    assertThat(whereItWasLeft).as("neutral part way out, it went no further").isBetween(20.0, 60.0);
    // Under review: enabled again, the pivot swings on to a target that was asked for before the
    // robot was disabled, without anyone asking again. The move's time counts the disabled half
    // second, so its line says 1.22 s for a swing that takes 0.60 s.
    assertThat(out.reason()).isEqualTo("pivot → out: ARRIVED in 1.22 s");
    assertThat(degrees(truth.position(PIVOT))).isCloseTo(130, within(2.0));
  }

  /**
   * A move's deadline runs while the robot is disabled. Left disabled past it, the move times out,
   * saying where the joint was. Enabled again, the joint is idle, and holds where it is.
   */
  @Test
  void aJointMoveLeftDisabledPastItsDeadlineTimesOutAndItsJointHoldsWhereItIs() {
    JointSimulation truth = enabledInTeleop();
    JointMechanism pivot = sim.joints().of(PIVOT);
    Move out = pivot.goTo(PIVOT_OUT);
    sim.scheduler().schedule(out);
    sim.run(0.2);

    disabledFor(2);
    List<String> runningWhileDisabled = running();
    String endedWhileDisabled = out.reason();
    double whereItWasLeft = truth.position(PIVOT);
    enabledAgainFor(1);

    assertThat(runningWhileDisabled).doesNotContain("pivot → out");
    assertThat(endedWhileDisabled)
        .as("its deadline, twice its 0.56 s profile and half a second, passed while disabled")
        .matches("pivot → out: TIMED_OUT after 1\\.62 s, pivot at \\d+(\\.\\d+)?°");
    assertThat(pivot.goal()).as("it holds where it is").isCloseTo(whereItWasLeft, within(0.02));
    assertThat(truth.position(PIVOT)).isCloseTo(whereItWasLeft, within(0.04));
    assertThat(degrees(truth.position(PIVOT))).as("nowhere near out").isLessThan(60);
  }

  // ---- A held binding ----

  /**
   * A command a held trigger started is still running while the robot is disabled, though its
   * joints are not driven. Still held when the robot is enabled again, it drives them again.
   */
  @Test
  void aHeldBindingsCommandRunsThroughDisableAndDrivesAgainWhenEnabled() {
    JointSimulation truth = enabledInTeleop();
    holdTheOperatorsTrigger(true);
    sim.run(1.5);
    double intaking = truth.velocity(ROLLER);

    disabledFor(2);
    List<String> runningWhileDisabled = running();
    double whileDisabled = truth.velocity(ROLLER);
    String outcomeWhileDisabled = sim.joints().of(ROLLER).lastOutcome();
    enabledAgainFor(1);

    assertThat(intaking).isCloseTo(2000 * RPM, within(20.0));
    assertThat(runningWhileDisabled).contains("Intake", "roller: intaking until piece in");
    assertThat(whileDisabled).as("not driven while disabled").isCloseTo(0, within(1.0));
    assertThat(outcomeWhileDisabled).as("not canceled, so no outcome").isEmpty();
    // Under review: the roller spins up again the moment the robot is enabled, on a press made
    // before it was disabled. The move's 8 s deadline ran through the 2 s disabled as well.
    assertThat(truth.velocity(ROLLER)).isCloseTo(2000 * RPM, within(20.0));
    assertThat(running()).contains("Intake");
  }

  /**
   * A binding is polled while the robot is disabled. Let go then, a held trigger cancels its
   * command as it would enabled, and the command's move says so. Enabled again, nothing runs.
   */
  @Test
  void aHeldBindingLetGoWhileDisabledCancelsItsCommandThen() {
    JointSimulation truth = enabledInTeleop();
    holdTheOperatorsTrigger(true);
    sim.run(1.5);

    disabledFor(1);
    holdTheOperatorsTrigger(false);
    sim.run(0.1);
    List<String> runningWhileDisabled = running();
    String outcomeWhileDisabled = sim.joints().of(ROLLER).lastOutcome();
    enabledAgainFor(1);

    assertThat(runningWhileDisabled).doesNotContain("Intake");
    assertThat(outcomeWhileDisabled)
        .isEqualTo("roller: intaking until piece in: CANCELED after 2.50 s");
    assertThat(truth.velocity(ROLLER)).as("idle, it stays stopped").isCloseTo(0, within(1.0));
    assertThat(degrees(truth.position(PIVOT))).as("idle, it holds out").isCloseTo(130, within(2.0));
  }

  /**
   * A trigger pressed while the robot is disabled starts its command then. The command drives
   * nothing until the robot is enabled, and then it does.
   */
  @Test
  void aBindingPressedWhileDisabledStartsItsCommandWhichDrivesOnceEnabled() {
    JointSimulation truth = enabledInTeleop();

    disabledFor(0.5);
    holdTheOperatorsTrigger(true);
    sim.run(0.3);
    List<String> runningWhileDisabled = running();
    double pivotWhileDisabled = degrees(truth.position(PIVOT));
    enabledAgainFor(1.5);

    // Under review: a command starts while the robot is disabled, and the robot moves as it is
    // enabled, on a trigger pressed before that. The intake's forked move to "out" also spent
    // 0.3 s of its deadline before the pivot could move.
    assertThat(runningWhileDisabled).contains("Intake", "pivot → out");
    assertThat(pivotWhileDisabled).as("not driven while disabled").isCloseTo(1.5, within(1.0));
    assertThat(degrees(truth.position(PIVOT))).isCloseTo(130, within(2.0));
    assertThat(truth.velocity(ROLLER)).isCloseTo(2000 * RPM, within(20.0));
  }

  // ---- A forked child ----

  /**
   * A command's forked children live exactly as long as it does, disabled or not. The shot's forked
   * follow and aim stay running through a disable. Its feed, awaited only while the robot is ready,
   * is canceled by its own gate, because a disabled shooter is not at its goal. Enabled again with
   * the trigger still held, the shooter is driven and the feed starts again. Let go while disabled,
   * the parent is canceled and its children with it.
   */
  @Test
  void aForkedChildRunsThroughDisableWithItsParentAndEndsWithIt() {
    JointSimulation truth = enabledInTeleop();
    holdTheDriversTrigger(true);
    sim.run(2);
    double feeding = truth.velocity(FEEDER);

    disabledFor(2);
    List<String> runningWhileDisabled = running();
    String feedWhileDisabled = sim.joints().of(FEEDER).lastOutcome();
    enabledAgainFor(1.5);
    List<String> runningEnabledAgain = running();
    double feedingAgain = truth.velocity(FEEDER);
    disabledFor(0.5);
    holdTheDriversTrigger(false);
    sim.run(0.1);

    assertThat(feeding).isCloseTo(1000 * RPM, within(10.0));
    assertThat(runningWhileDisabled)
        .contains("Shoot", "shooter follows a target", "Drive facing the Hub")
        .doesNotContain("feeder and indexer: feeding");
    assertThat(feedWhileDisabled)
        .isEqualTo(
            "feeder and indexer: feeding: CANCELED after 2.00 s: ready to feed is false: shooter"
                + " at goal");
    // Under review: enabled again, the shooter is driven to speed and the feed runs, on a trigger
    // pressed before the robot was disabled.
    assertThat(runningEnabledAgain).contains("Shoot", "feeder and indexer: feeding");
    assertThat(feedingAgain).isCloseTo(1000 * RPM, within(10.0));
    assertThat(running()).as("let go while disabled").doesNotContain("Shoot");
    assertThat(sim.joints().of(SHOOTER).lastOutcome())
        .matches(
            "shooter follows a target: CANCELED after 6\\.00 s, shooter at \\d+ rpm, its target 3000"
                + " rpm");
  }

  // ---- The drivetrain ----

  /**
   * The drivetrain refuses to be driven while the robot is disabled, as a joint does, whatever
   * asks. With the stick pushed the drive's default command asks the wheels to drive every loop,
   * and a move asks them to follow its path: the wheels stay neutral, and the robot stays where it
   * stands.
   */
  @Test
  void theDrivetrainIsNotDrivenWhileDisabledWhateverAsks() {
    enabledInTeleop();
    disabledFor(0.2);
    Pose2d stood = sim.world().truePose();

    pushTheDriversStickForward();
    sim.run(1);
    double drivenByTheSticks = metersFrom(stood);
    DriveMove ahead = sim.drive().goTo(stood.plus(new Transform2d(2, 0, Rotation2d.ZERO)));
    sim.scheduler().schedule(ahead);
    sim.run(1);
    List<String> runningWhileDisabled = running();

    assertThat(runningWhileDisabled).containsExactly(ahead.name());
    assertThat(drivenByTheSticks).as("meters driven by the sticks, disabled").isLessThan(0.001);
    assertThat(metersFrom(stood)).as("meters driven by a move, disabled").isLessThan(0.001);
  }

  /**
   * A robot that is moving when it is disabled is not driven on: its wheels go neutral, it rolls to
   * a stop, and it stays there with the stick still pushed.
   */
  @Test
  void aRobotDrivingWhenItIsDisabledRollsToAStop() {
    enabledInTeleop();
    pushTheDriversStickForward();
    sim.run(1);

    Pose2d atDisable = sim.world().truePose();
    disabledFor(2);
    double rolled = metersFrom(atDisable);
    Pose2d stopped = sim.world().truePose();
    sim.run(1);

    assertThat(rolled)
        .as("meters it rolled on from 4 m/s, braked by its motors and not driven")
        .isBetween(1.0, 3.0);
    assertThat(metersFrom(stopped)).as("stopped").isLessThan(0.001);
  }

  // ---- An auto ----

  /**
   * The one command the runtime ends itself: an auto still running when autonomous ends is canceled
   * in that loop, straight into teleop, and every move it had running says so. The drive goes back
   * to the sticks, the roller stops, and the pivot holds where the auto left it.
   */
  @Test
  void anAutoRunningWhenAutonomousEndsIsCutOffAsTeleopBegins() {
    JointSimulation truth = enabledInTeleop();
    RobotSim.setEnabled(false);
    sim.autos().choose("Grab and shoot");
    sim.loops(3);
    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    sim.run(1);
    List<String> runningInAutonomous = running();

    RobotSim.enable(RobotMode.TELEOPERATED, true);
    sim.loop();
    List<String> runningInTeleop = running();
    sim.run(1);

    assertThat(runningInAutonomous).contains("Grab and shoot", "drive → Pickup", "Intake");
    assertThat(runningInTeleop).containsExactly("Drive with the sticks");
    assertThat(sim.autos().lastRun().orElseThrow().describe())
        .isEqualTo("Grab and shoot: CUT OFF after 1.00 s, as autonomous ended");
    assertThat(sim.joints().of(ROLLER).lastOutcome())
        .isEqualTo("roller: intaking until piece in: CANCELED after 0.98 s");
    assertThat(truth.velocity(ROLLER)).as("idle, it stops").isCloseTo(0, within(1.0));
    assertThat(degrees(truth.position(PIVOT))).as("idle, it holds out").isCloseTo(130, within(2.0));
  }

  // ---- Utility mode ----

  /**
   * Going from teleop to utility mode, enabled, changes nothing for a running command: it is not
   * canceled, its joints stay driven, and its binding still holds it. A move started in utility
   * mode runs on into teleop the same way.
   */
  @Test
  void aCommandRunsOnFromTeleopIntoUtilityModeAndBack() {
    JointSimulation truth = enabledInTeleop();
    holdTheOperatorsTrigger(true);
    sim.run(1.5);

    RobotSim.enable(RobotMode.UTILITY, true);
    sim.run(1);
    List<String> runningInUtility = running();
    double intakingInUtility = truth.velocity(ROLLER);
    Move in = sim.joints().of(PIVOT).goTo(ExampleRobot.PIVOT_IN);
    sim.scheduler().schedule(in);
    sim.run(0.2);
    RobotSim.enable(RobotMode.TELEOPERATED, true);
    sim.run(1);

    assertThat(runningInUtility).contains("Intake", "roller: intaking until piece in");
    assertThat(intakingInUtility).isCloseTo(2000 * RPM, within(20.0));
    assertThat(sim.joints().of(ROLLER).lastOutcome())
        .as("the move to in took the pivot from Intake, as it would in teleop")
        .isEqualTo("roller: intaking until piece in: CANCELED after 2.50 s");
    assertThat(in.reason()).isEqualTo("pivot → in: ARRIVED in 0.60 s");
  }
}

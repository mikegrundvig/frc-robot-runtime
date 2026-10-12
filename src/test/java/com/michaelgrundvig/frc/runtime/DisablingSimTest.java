package com.michaelgrundvig.frc.runtime;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.pitwall.Actions;
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
 * <p>Disabled means disabled. In the loop the robot is disabled, every running command is canceled,
 * and each move says so in its outcome. While it is disabled no joint and no wheel is driven,
 * whatever asks. When it is enabled again nothing carries on from before: each joint holds where it
 * is then, and a command starts only when something asks for it afresh. The one command that starts
 * by itself is a default command, which the scheduler starts whenever its mechanism is free.
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

  private void letGoOfTheDriversStick() {
    sim.driver().setLeftY(0);
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
   * A move running when the robot is disabled is canceled in that loop, and says why. Nothing of it
   * is left to count while the robot is disabled, however long that is. Enabled again, its joint
   * holds where it is, and does not go on to where the move was taking it.
   */
  @Test
  void aJointMoveRunningAtDisableIsCanceledAndItsJointHoldsWhereItIsWhenEnabledAgain() {
    JointSimulation truth = enabledInTeleop();
    JointMechanism pivot = sim.joints().of(PIVOT);
    Move out = pivot.goTo(PIVOT_OUT);
    sim.scheduler().schedule(out);
    sim.run(0.2);

    disabledFor(0.02);
    List<String> runningAsItWasDisabled = running();
    String endedAsItWasDisabled = out.reason();
    sim.run(2);
    String afterItsOldDeadline = out.reason();
    double whereItWasLeft = truth.position(PIVOT);
    enabledAgainFor(1);

    assertThat(runningAsItWasDisabled).doesNotContain("pivot → out");
    assertThat(endedAsItWasDisabled)
        .matches(
            "pivot → out: CANCELED after 0\\.20 s, pivot at \\d+(\\.\\d+)?°: the robot was disabled");
    assertThat(afterItsOldDeadline)
        .as("its deadline, 1.62 s after it started, passed while disabled, and nothing timed out")
        .isEqualTo(endedAsItWasDisabled);
    assertThat(degrees(whereItWasLeft)).as("neutral part way out").isBetween(20.0, 60.0);
    assertThat(pivot.goal()).as("it holds where it is").isCloseTo(whereItWasLeft, within(0.02));
    assertThat(truth.position(PIVOT)).isCloseTo(whereItWasLeft, within(0.04));
    assertThat(running()).containsExactly("Drive with the sticks");
  }

  /**
   * A move asked for again once the robot is enabled starts afresh from where its joint is: its
   * profile, its time and its deadline are this run's, and owe nothing to the run that was
   * canceled.
   */
  @Test
  void aMoveAskedAgainOnceEnabledStartsAfreshFromWhereItsJointIs() {
    JointSimulation truth = enabledInTeleop();
    Move out = sim.joints().of(PIVOT).goTo(PIVOT_OUT);
    sim.scheduler().schedule(out);
    sim.run(0.2);
    disabledFor(2);
    double whereItWasLeft = degrees(truth.position(PIVOT));
    enabledAgainFor(0.1);

    sim.scheduler().schedule(out);
    sim.run(1);

    assertThat(whereItWasLeft).isBetween(20.0, 60.0);
    assertThat(out.reason())
        .as("the rest of the way, where the whole swing takes 0.60 s")
        .matches("pivot → out: ARRIVED in 0\\.5\\d s");
    assertThat(degrees(truth.position(PIVOT))).isCloseTo(130, within(2.0));
  }

  // ---- A held binding ----

  /**
   * A command a held trigger started is canceled as the robot is disabled, and its move says why.
   * Still held when the robot is enabled again, the trigger does not start it again: the roller
   * stays stopped, and the pivot holds where it is. Let go and pressed again, it starts.
   */
  @Test
  void aHeldBindingsCommandIsCanceledAtDisableAndStartsAgainOnlyOnANewPress() {
    JointSimulation truth = enabledInTeleop();
    holdTheOperatorsTrigger(true);
    sim.run(1.5);
    double intaking = truth.velocity(ROLLER);

    disabledFor(2);
    List<String> runningWhileDisabled = running();
    double whileDisabled = truth.velocity(ROLLER);
    String outcomeWhileDisabled = sim.joints().of(ROLLER).lastOutcome();
    enabledAgainFor(1);
    List<String> runningStillHeld = running();
    double stillHeld = truth.velocity(ROLLER);
    holdTheOperatorsTrigger(false);
    sim.run(0.1);
    holdTheOperatorsTrigger(true);
    sim.run(1);

    assertThat(intaking).isCloseTo(2000 * RPM, within(20.0));
    assertThat(runningWhileDisabled).containsExactly("Drive with the sticks");
    assertThat(whileDisabled).as("not driven while disabled").isCloseTo(0, within(1.0));
    assertThat(outcomeWhileDisabled)
        .isEqualTo(
            "roller: intaking until piece in: CANCELED after 1.50 s: the robot was disabled");
    assertThat(runningStillHeld).as("held through the disable").doesNotContain("Intake");
    assertThat(stillHeld).as("idle, it stays stopped").isCloseTo(0, within(1.0));
    assertThat(running()).as("pressed again").contains("Intake");
    assertThat(truth.velocity(ROLLER)).isCloseTo(2000 * RPM, within(20.0));
  }

  // ---- A forked child ----

  /**
   * A command's forked children are canceled with it as the robot is disabled, each move saying
   * why: the shot's follow, its aim and its feed. Enabled again with the trigger still held,
   * nothing of the shot runs: the feed stays stopped, the flywheel is left to coast, and the drive
   * is back on the sticks.
   */
  @Test
  void aCommandsForkedChildrenAreCanceledWithItAtDisable() {
    JointSimulation truth = enabledInTeleop();
    holdTheDriversTrigger(true);
    sim.run(2);
    double feeding = truth.velocity(FEEDER);

    disabledFor(2);
    List<String> runningWhileDisabled = running();
    String followWhileDisabled = sim.joints().of(SHOOTER).lastOutcome();
    String feedWhileDisabled = sim.joints().of(FEEDER).lastOutcome();
    enabledAgainFor(1.5);

    assertThat(feeding).isCloseTo(1000 * RPM, within(10.0));
    assertThat(runningWhileDisabled).containsExactly("Drive with the sticks");
    assertThat(followWhileDisabled)
        .matches(
            "shooter follows a target: CANCELED after 2\\.00 s, shooter at \\d+ rpm, its target 3000"
                + " rpm: the robot was disabled");
    assertThat(feedWhileDisabled)
        .matches(
            "feeder and indexer: feeding: CANCELED after \\d\\.\\d\\d s: the robot was disabled");
    assertThat(running()).as("held through the disable").containsExactly("Drive with the sticks");
    assertThat(truth.velocity(FEEDER)).as("idle, it stays stopped").isCloseTo(0, within(1.0));
  }

  // ---- The drivetrain ----

  /**
   * The drivetrain refuses to be driven while the robot is disabled, as a joint does, whatever
   * asks. With the stick pushed the drive's default command asks the wheels to drive every loop,
   * and a move started while the robot is disabled asks them to follow its path: the wheels stay
   * neutral, and the robot stays where it stands.
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

  /**
   * The drive's default command, driving with the sticks, is canceled as the robot is disabled like
   * any other command, and the scheduler starts it again in the same loop, as it starts a default
   * command whenever its mechanism is free. While the robot is disabled it drives nothing. Enabled
   * again with the stick still pushed, the robot drives, with no new press.
   *
   * <p>That is right, because a default command is not started by a press that could be stale: it
   * reads the stick every loop, and drives what the stick asks now. And it is safe, in the way
   * every FRC robot is: the driver is holding that stick as the robot is enabled, and letting it go
   * stops the robot.
   */
  @Test
  void theDrivesDefaultCommandDrivesAgainOnceEnabledWithTheStickStillPushed() {
    enabledInTeleop();
    pushTheDriversStickForward();
    sim.run(1);

    disabledFor(2);
    List<String> runningWhileDisabled = running();
    Pose2d stopped = sim.world().truePose();
    enabledAgainFor(1);
    double drivenEnabledAgain = metersFrom(stopped);
    letGoOfTheDriversStick();
    sim.run(1.5);
    Pose2d stoppedAgain = sim.world().truePose();
    sim.run(0.5);

    assertThat(runningWhileDisabled).containsExactly("Drive with the sticks");
    assertThat(drivenEnabledAgain).as("meters driven in 1 s, enabled again").isGreaterThan(2.0);
    assertThat(metersFrom(stoppedAgain)).as("stopped, the stick let go").isLessThan(0.01);
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

  /**
   * An auto running when the robot is disabled in autonomous ends the same way, cut off as
   * autonomous ended, and its moves with it. Enabled in teleop afterwards, nothing of it runs.
   */
  @Test
  void anAutoRunningWhenTheRobotIsDisabledIsCutOffAndDoesNotCarryOn() {
    JointSimulation truth = enabledInTeleop();
    RobotSim.setEnabled(false);
    sim.autos().choose("Grab and shoot");
    sim.loops(3);
    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    sim.run(1);

    RobotSim.enable(RobotMode.AUTONOMOUS, false);
    sim.run(0.5);
    List<String> runningWhileDisabled = running();
    enabledAgainFor(1);

    assertThat(runningWhileDisabled).containsExactly("Drive with the sticks");
    assertThat(sim.autos().lastRun().orElseThrow().describe())
        .isEqualTo("Grab and shoot: CUT OFF after 1.00 s, as autonomous ended");
    assertThat(sim.joints().of(ROLLER).lastOutcome())
        .isEqualTo("roller: intaking until piece in: CANCELED after 0.98 s");
    assertThat(running()).containsExactly("Drive with the sticks");
    assertThat(truth.velocity(ROLLER)).as("idle, it stays stopped").isCloseTo(0, within(1.0));
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

  /**
   * A pit action running in utility mode is canceled as the robot is disabled, by Pitwall, which
   * says why on its page. The robot rolls to a stop. Enabled in utility mode again, the action does
   * not start again: someone has to press it on the page.
   */
  @Test
  void aPitActionRunningInUtilityModeIsCanceledAtDisableAndDoesNotStartAgain() {
    enabledInTeleop();
    RobotSim.enable(RobotMode.UTILITY, true);
    sim.pitwall().actions().requestStart(RobotContainer.DRIVE_CHECK);
    sim.run(1);
    List<String> runningInUtility = running();
    Pose2d atDisable = sim.world().truePose();

    RobotSim.enable(RobotMode.UTILITY, false);
    sim.run(1);
    List<String> runningWhileDisabled = running();
    Actions.State whileDisabled = driveCheck();
    Pose2d stopped = sim.world().truePose();
    RobotSim.enable(RobotMode.UTILITY, true);
    sim.run(1);

    assertThat(runningInUtility).containsExactly("Drive forward");
    assertThat(runningWhileDisabled).containsExactly("Drive with the sticks");
    assertThat(whileDisabled.phase()).isEqualTo(Actions.Phase.CANCELED);
    assertThat(whileDisabled.detail())
        .isEqualTo("Canceled: actions run only with the robot enabled in utility mode");
    assertThat(stopped.getTranslation().getDistance(atDisable.getTranslation()))
        .as("meters it rolled on from half a meter a second")
        .isLessThan(0.1);
    assertThat(running()).as("enabled again").containsExactly("Drive with the sticks");
    assertThat(metersFrom(stopped)).as("it stays stopped").isLessThan(0.001);
  }

  /** The drive check, as the Pitwall page shows it. */
  private Actions.State driveCheck() {
    for (Actions.State action : sim.pitwall().actions().view()) {
      if (action.name().equals(RobotContainer.DRIVE_CHECK)) {
        return action;
      }
    }
    throw new AssertionError("the page has no drive check");
  }
}

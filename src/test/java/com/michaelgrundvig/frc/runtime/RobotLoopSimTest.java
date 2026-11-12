package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.entry;

import com.michaelgrundvig.frc.runtime.lib.timing.LoopMonitor;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Trigger;
import org.wpilib.util.AlertDataJNI;

/**
 * What the robot does when its loop throws: on a robot it carries on without what threw, and says
 * so; in the simulator and tests it stops at once, so a bug fails its test.
 */
@SimulationTest
class RobotLoopSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  /**
   * On a robot, a command that throws is ended, the loop goes on to write the joints, and the robot
   * drives on, its other commands running. Where it threw is reported once, though the same binding
   * throws again.
   */
  @Test
  void onARobotACommandThatThrowsIsEndedAndTheRobotDrivesOn() {
    sim.build(RobotSim.WITHOUT_CAMERAS, SensorErrors.NONE);
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1.0); // pushed forward
    sim.driver().notifyNewData();
    int[] counted = {0};
    sim.scheduler()
        .schedule(
            Command.noRequirements(
                    co -> {
                      while (true) {
                        counted[0]++;
                        co.yield();
                      }
                    })
                .named("Counting"));
    Command broken =
        Command.noRequirements(
                co -> {
                  co.yield();
                  throw new IllegalStateException("broken on purpose");
                })
            .named("Broken");

    List<Boolean> running = new ArrayList<>();
    List<String> sections = List.of();
    double xAtTheThrow = 0;
    for (int time = 0; time < 2; time++) {
      sim.scheduler().schedule(broken);
      sim.loopOnRobot(); // it starts
      sim.loopOnRobot(); // it throws
      running.add(sim.scheduler().isRunning(broken));
      sections = sim.lastLoop().sections().stream().map(LoopMonitor.Section::name).toList();
      xAtTheThrow = sim.drive().getPose().getX();
    }
    int countedAtTheThrow = counted[0];
    for (int i = 0; i < 25; i++) {
      sim.loopOnRobot();
    }

    assertThat(running).containsExactly(false, false);
    assertThat(sections)
        .as("the joints written as the loop goes on")
        .endsWith("Commands", "Outputs");
    assertThat(counted[0]).as("the other command ran every loop").isEqualTo(countedAtTheThrow + 25);
    assertThat(sim.drive().getPose().getX() - xAtTheThrow).as("driving on").isGreaterThan(0.5);
    assertThat(raised())
        .singleElement()
        .asString()
        .startsWith(
            "The loop threw, and the robot carried on: Commands threw IllegalStateException: broken"
                + " on purpose, at com.michaelgrundvig.frc.runtime.RobotLoopSimTest.lambda$")
        .containsOnlyOnce("broken on purpose");
  }

  /**
   * On a robot, a command that throws as it is canceled, in the loop the robot is disabled, does
   * not end the program, and leaves nothing running: the commands the scheduler had not reached
   * when it threw are canceled in the next loop. The throw is reported.
   */
  @Test
  void onARobotACommandThatThrowsAsItIsCanceledAtDisableLeavesNothingOfItsOwnRunning() {
    sim.build(RobotSim.WITHOUT_CAMERAS, SensorErrors.NONE);
    RobotSim.setEnabled(true);
    Command broken =
        Command.noRequirements(RobotLoopSimTest::runUntilCanceled)
            .whenCanceled(
                () -> {
                  throw new IllegalStateException("broken on purpose");
                })
            .named("Broken");
    Command after = Command.noRequirements(RobotLoopSimTest::runUntilCanceled).named("After it");
    sim.scheduler().schedule(broken);
    sim.scheduler().schedule(after);
    sim.loopOnRobot();

    RobotSim.setEnabled(false);
    sim.loopOnRobot(); // the loop of the disable: canceling throws part way through
    sim.loopOnRobot();

    assertThat(sim.scheduler().getRunningCommands())
        .extracting(Command::name)
        .containsExactly("Drive with the sticks");
    assertThat(raised())
        .singleElement()
        .asString()
        .startsWith(
            "The loop threw, and the robot carried on: Commands threw IllegalStateException: broken"
                + " on purpose");
  }

  /** A command's body that runs until it is canceled. */
  private static void runUntilCanceled(Coroutine coroutine) {
    while (true) {
      coroutine.yield();
    }
  }

  /**
   * On a robot, a binding whose condition throws every loop runs nothing, so after half a second of
   * it the program ends (and restarts) rather than drive on with nothing running.
   */
  @Test
  void onARobotWhatThrowsEveryLoopEndsTheProgramAfterHalfASecond() {
    IllegalStateException thrown = bindingThatAlwaysThrows();

    for (int i = 1; i < LoopHealth.FAILING_LOOPS; i++) {
      sim.loopOnRobot();
    }

    assertThatThrownBy(sim::loopOnRobot).isSameAs(thrown);
  }

  /** In the simulator, what the commands throw ends the loop at once: a bug fails its test. */
  @Test
  void inTheSimulatorWhatThrowsEndsTheLoopAtOnce() {
    IllegalStateException thrown = bindingThatAlwaysThrows();

    assertThatThrownBy(sim::loop).isSameAs(thrown);
  }

  /** Binds a command to a condition that throws whenever it's read, as the robot is built. */
  private IllegalStateException bindingThatAlwaysThrows() {
    IllegalStateException thrown = new IllegalStateException("thrown every loop");
    sim.robot();
    new Trigger(
            sim.scheduler(),
            () -> {
              throw thrown;
            })
        .onTrue(Command.noRequirements(co -> {}).named("Never runs"));
    return thrown;
  }

  /** How many loops a scenario keeps the robot broken: fewer than end the program. */
  private static final int BROKEN_LOOPS = 15;

  private final BreakableRobot breakable = new BreakableRobot();

  /**
   * What the robot did while it was broken, and once it was mended.
   *
   * @param commandsRan whether a command ran in every loop it was broken
   * @param droveWhileBroken whether the sticks drove the robot while it was broken
   * @param hoodMovedWhileBroken whether the hood moved toward where it was sent while it was broken
   * @param drovewhenMended whether the sticks drove the robot once it was mended
   * @param hoodArrivedWhenMended whether the hood got where it was sent once it was mended
   */
  private record Seen(
      boolean commandsRan,
      boolean droveWhileBroken,
      boolean hoodMovedWhileBroken,
      boolean drovewhenMended,
      boolean hoodArrivedWhenMended) {}

  /**
   * On a robot, each section of the loop is guarded on its own, and the loop carries on past a
   * throw, as the section's part says:
   *
   * <ul>
   *   <li>The dashboard serves the drive team. The robot drives the same while it throws.
   *   <li>A joint's reading leaves the loop's readings untrusted: the commands don't run, the
   *       drivetrain stands still, and the joints idle. Nothing a command asked moves.
   *   <li>A joint's output, the pivot's, keeps the other joints' outputs: the hood still moves.
   * </ul>
   *
   * Once mended, the robot drives, and the hood gets where it was sent. Each throw is reported in
   * the alert, by its section.
   */
  @Test
  void onARobotEachSectionThatThrowsIsReportedAndTheRestOfTheLoopCarriesOn() {
    Map<BreakableRobot.Broken, Seen> seen = new LinkedHashMap<>();
    Map<BreakableRobot.Broken, List<String>> alerts = new LinkedHashMap<>();
    breakable.watchTheDashboard();
    for (BreakableRobot.Broken broken :
        List.of(
            BreakableRobot.Broken.THE_DASHBOARD,
            BreakableRobot.Broken.A_JOINTS_READING,
            BreakableRobot.Broken.A_JOINTS_OUTPUT)) {
      seen.put(broken, brokenFor(broken, BROKEN_LOOPS));
      alerts.put(broken, raised());
    }

    assertThat(seen)
        .containsExactly(
            entry(BreakableRobot.Broken.THE_DASHBOARD, new Seen(true, true, true, true, true)),
            entry(
                BreakableRobot.Broken.A_JOINTS_READING, new Seen(false, false, false, true, true)),
            entry(BreakableRobot.Broken.A_JOINTS_OUTPUT, new Seen(true, true, true, true, true)));
    assertThat(alerts)
        .allSatisfy((broken, raised) -> assertThat(raised).hasSize(1))
        .hasEntrySatisfying(
            BreakableRobot.Broken.THE_DASHBOARD,
            raised -> assertThat(raised.getFirst()).startsWith(said("Dashboard")))
        .hasEntrySatisfying(
            BreakableRobot.Broken.A_JOINTS_READING,
            raised -> assertThat(raised.getFirst()).startsWith(said("Joints")))
        .hasEntrySatisfying(
            BreakableRobot.Broken.A_JOINTS_OUTPUT,
            raised -> assertThat(raised.getFirst()).startsWith(said("Outputs")));
  }

  /** How the alert begins for a section that threw what the breakable robot throws. */
  private static String said(String section) {
    return "The loop threw, and the robot carried on: "
        + section
        + " threw IllegalStateException: "
        + BreakableRobot.THROWN
        + ", at ";
  }

  /**
   * Builds the robot with its pivot breakable, enables it with the driver's stick pushed forward,
   * sends the hood to its lob, and breaks the robot for so many loops, and then mends it.
   */
  private Seen brokenFor(BreakableRobot.Broken broken, int loops) {
    sim.build(
        BreakableRobot.JOINTS_WITHOUT_CAMERAS,
        SensorErrors.NONE,
        hardware -> breakable.breakingAJoint(hardware, ExampleRobot.PIVOT));
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1.0); // pushed forward
    sim.driver().notifyNewData();
    int[] counted = {0};
    sim.scheduler().schedule(counting(counted));
    JointMechanism hood = sim.joints().of(ExampleRobot.HOOD);
    Move lob = hood.goTo(ExampleRobot.HOOD_LOB);
    sim.scheduler().schedule(lob);
    sim.loopOnRobot(); // the robot sees it is enabled

    double xBefore = sim.drive().getPose().getX();
    double hoodBefore = hood.position();
    int countedBefore = counted[0];
    breakable.breakAt(broken);
    for (int i = 0; i < loops; i++) {
      sim.loopOnRobot();
    }
    breakable.mend();
    double xBroken = sim.drive().getPose().getX();
    double hoodBroken = hood.position();
    int countedBroken = counted[0];
    for (int i = 0; i < 50; i++) {
      sim.loopOnRobot();
    }

    return new Seen(
        countedBroken - countedBefore == loops,
        xBroken - xBefore > 0.05,
        Math.abs(hoodBroken - hoodBefore) > Math.toRadians(1),
        sim.drive().getPose().getX() - xBroken > 0.5,
        lob.arrived());
  }

  /** A command that counts the loops it runs in, until it is canceled. */
  private static Command counting(int[] counted) {
    return Command.noRequirements(
            co -> {
              while (true) {
                counted[0]++;
                co.yield();
              }
            })
        .named("Counting");
  }

  /**
   * On a robot, a joint whose reading throws every loop fails every loop: the robot stands still,
   * so after half a second the program ends (and restarts). A dashboard that throws every loop
   * fails none: the robot drives on, however long it throws.
   */
  @Test
  void onARobotOnlyWhatTheRobotsControlDependsOnEndsTheProgram() {
    breakable.watchTheDashboard();
    Map<BreakableRobot.Broken, Integer> loopsUntilTheEnd = new LinkedHashMap<>();
    for (BreakableRobot.Broken broken :
        List.of(BreakableRobot.Broken.A_JOINTS_READING, BreakableRobot.Broken.THE_DASHBOARD)) {
      sim.build(
          BreakableRobot.JOINTS_WITHOUT_CAMERAS,
          SensorErrors.NONE,
          hardware -> breakable.breakingAJoint(hardware, ExampleRobot.PIVOT));
      RobotSim.setEnabled(true);
      breakable.breakAt(broken);
      loopsUntilTheEnd.put(broken, loopsUntilTheProgramEnds(4 * LoopHealth.FAILING_LOOPS));
      breakable.mend();
    }

    assertThat(loopsUntilTheEnd)
        .containsExactly(
            entry(BreakableRobot.Broken.A_JOINTS_READING, LoopHealth.FAILING_LOOPS),
            entry(BreakableRobot.Broken.THE_DASHBOARD, 0));
  }

  /**
   * How many loops on a robot ran until the program ended, the last with it, or 0 if it ran them
   * all.
   */
  private int loopsUntilTheProgramEnds(int most) {
    for (int loop = 1; loop <= most; loop++) {
      try {
        sim.loopOnRobot();
      } catch (IllegalStateException e) {
        assertThat(e).hasMessage(BreakableRobot.THROWN);
        return loop;
      }
    }
    return 0;
  }

  /**
   * In the simulator, what any section of the loop throws ends the loop at once, wherever it is: a
   * bug fails its test.
   */
  @Test
  void inTheSimulatorWhatAnySectionThrowsEndsTheLoopAtOnce() {
    breakable.watchTheDashboard();
    List<String> ended = new ArrayList<>();
    for (BreakableRobot.Broken broken :
        List.of(
            BreakableRobot.Broken.THE_DASHBOARD,
            BreakableRobot.Broken.A_JOINTS_READING,
            BreakableRobot.Broken.A_JOINTS_OUTPUT)) {
      sim.build(
          BreakableRobot.JOINTS_WITHOUT_CAMERAS,
          SensorErrors.NONE,
          hardware -> breakable.breakingAJoint(hardware, ExampleRobot.PIVOT));
      RobotSim.setEnabled(true);
      sim.loop();
      breakable.breakAt(broken);
      try {
        sim.loop();
      } catch (IllegalStateException e) {
        ended.add(broken + ": " + e.getMessage());
      }
      breakable.mend();
    }

    assertThat(ended)
        .containsExactly(
            "THE_DASHBOARD: " + BreakableRobot.THROWN,
            "A_JOINTS_READING: " + BreakableRobot.THROWN,
            "A_JOINTS_OUTPUT: " + BreakableRobot.THROWN);
  }

  /** The alerts raised now that say the loop threw. */
  private static List<String> raised() {
    List<String> raised = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.activeStartTime != 0 && alert.text.startsWith("The loop threw")) {
        raised.add(alert.text);
      }
    }
    return raised;
  }
}

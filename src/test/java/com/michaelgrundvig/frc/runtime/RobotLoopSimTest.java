package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.runtime.lib.timing.LoopMonitor;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Trigger;
import org.wpilib.util.AlertDataJNI;

/**
 * What the robot does when its commands throw: on a robot it carries on without what threw, and
 * says so; in the simulator and tests it stops at once, so a bug fails its test.
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
            "Commands threw, and the robot carried on without them: IllegalStateException: broken"
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
            "Commands threw, and the robot carried on without them: IllegalStateException: broken"
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

  /** The alerts raised now that say commands threw. */
  private static List<String> raised() {
    List<String> raised = new ArrayList<>();
    for (AlertDataJNI.AlertInfo alert : AlertDataJNI.getAlerts()) {
      if (alert.activeStartTime != 0 && alert.text.startsWith("Commands threw")) {
        raised.add(alert.text);
      }
    }
    return raised;
  }
}

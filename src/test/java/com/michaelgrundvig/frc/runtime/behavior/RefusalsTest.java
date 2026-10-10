package com.michaelgrundvig.frc.runtime.behavior;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.runtime.RobotSim;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

/**
 * The one rule for what can't be: thrown as the robot starts, before its command scheduler's first
 * run; refused once it runs, in a command or anywhere else.
 */
@ExtendWith(RobotSim.class)
class RefusalsTest {
  private final Scheduler scheduler = Scheduler.createIndependentScheduler();

  @Test
  void itStartsUntilTheSchedulerFirstRuns() {
    boolean before = Refusals.starting(scheduler);
    scheduler.run();

    assertThat(before).isTrue();
    assertThat(Refusals.starting(scheduler)).isFalse();
  }

  /** Inside a command, on the scheduler's first run, before the scheduler has said it ran. */
  @Test
  void aCommandRunningIsNeverStarting() {
    List<Boolean> inside = new ArrayList<>();
    scheduler.schedule(
        Command.noRequirements(co -> inside.add(Refusals.starting(scheduler))).named("in"));

    scheduler.run();

    assertThat(inside).containsExactly(false);
  }

  @Test
  void aReasonIsThrownAsTheRobotStartsAndKeptOnceItRuns() {
    assertThatThrownBy(() -> Refusals.asBuilt(scheduler, "pivot has no position clear"))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("pivot has no position clear");
    assertThat(Refusals.asBuilt(scheduler, null)).isNull();

    scheduler.run();

    assertThat(Refusals.asBuilt(scheduler, "pivot has no position clear"))
        .isEqualTo("pivot has no position clear");
  }

  /** A gate that can't be holds false once the robot runs, saying why, and never opens. */
  @Test
  void aGateThatCantBeHoldsFalseSayingWhy() {
    assertThatThrownBy(() -> Refusals.gate(scheduler, "pivot at clear", "no clear"))
        .hasMessage("no clear");
    scheduler.run();

    Gate gate = Refusals.gate(scheduler, "pivot at clear", "no clear");
    scheduler.run();

    assertThat(gate.getAsBoolean()).isFalse();
    assertThat(gate.why()).isEqualTo("pivot at clear is false: no clear");
  }
}

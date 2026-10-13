package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.record.AutoOutcome;
import com.michaelgrundvig.frc.world.record.BindingFired;
import com.michaelgrundvig.frc.world.record.BindingStart;
import com.michaelgrundvig.frc.world.record.DriveMoveTask;
import com.michaelgrundvig.frc.world.record.DriveOutcome;
import com.michaelgrundvig.frc.world.record.GateTerm;
import com.michaelgrundvig.frc.world.record.Holder;
import com.michaelgrundvig.frc.world.record.Identity;
import com.michaelgrundvig.frc.world.record.JointPlace;
import com.michaelgrundvig.frc.world.record.JointState;
import com.michaelgrundvig.frc.world.record.MoveOutcome;
import com.michaelgrundvig.frc.world.record.MoveProgress;
import com.michaelgrundvig.frc.world.record.MoveReason;
import com.michaelgrundvig.frc.world.record.MoveResult;
import com.michaelgrundvig.frc.world.record.MoveTarget;
import com.michaelgrundvig.frc.world.record.MoveTask;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

/**
 * AdvantageKit logs every record the robot records its facts in, and reads each back as it was. A
 * record with a field AdvantageKit cannot log, a text or a list, is skipped as it is logged, with
 * only a Driver Station error to say so: the key would sit empty in every log. So each type is put
 * into a log table here and read out again, alone and as the array the robot logs it in.
 */
class RecordedFactsTest {
  private static final MoveOutcome A_MOVE =
      new MoveOutcome(
          MoveResult.TIMED_OUT,
          MoveTask.GO_TO,
          MoveProgress.BEGUN,
          1.62,
          2,
          Identity.NONE,
          MoveTarget.PLACE,
          1,
          2.27,
          0.03,
          -0.5,
          Double.NaN,
          MoveReason.CONDITION_NEVER_HELD,
          Identity.NONE);

  private static final List<Record> EVERY_KIND =
      List.of(
          A_MOVE,
          new DriveOutcome(
              MoveResult.REFUSED, DriveMoveTask.TO_PLACE, 0, 1.5, 2.5, 0.7, MoveReason.NO_DRIVER),
          new AutoOutcome(AutoOutcome.Result.CUT_OFF, 15),
          JointState.homing(JointState.Toward.UPPER_SWITCH, 9.5),
          JointState.faulted(JointState.Fault.ENCODER_SLIPPED),
          Holder.limitSwitch(3),
          JointPlace.between(0, 2),
          new GateTerm(GateTerm.Kind.SENSOR_LOST, 4),
          new BindingFired(5, 1, BindingFired.Control.RIGHT_TRIGGER, 7, BindingStart.WHILE_HELD));

  @Test
  void everyRecordedFactIsLoggedAndReadBackAsItWas() {
    for (Record fact : EVERY_KIND) {
      LogTable table = new LogTable(0);

      table.put("Fact", fact);

      String kind = fact.getClass().getSimpleName();
      assertThat(table.get("Fact")).as("%s is in the log", kind).isNotNull();
      assertThat(readBack(table, fact)).as(kind).isEqualTo(fact).isNotSameAs(fact);
    }
  }

  /** Reads a fact back as its own type, starting from another value of that type. */
  private static <R extends Record> R readBack(LogTable table, R fact) {
    return table.get("Fact", fact);
  }

  /** The robot logs what holds a joint, a move's parts and a gate's terms as arrays. */
  @Test
  void arraysOfFactsAreLoggedTooAndAnEmptyOneStaysEmpty() {
    LogTable table = new LogTable(0);
    Holder[] holders = {Holder.interlock(0), Holder.noPosition(1)};

    table.put("HeldBy", holders);
    table.put("Outcome", new MoveOutcome[] {A_MOVE, A_MOVE});
    table.put("Nothing", new Holder[0]);

    assertThat(table.get("Nothing")).as("an empty array is in the log").isNotNull();
    assertThat(table.get("HeldBy", new Holder[0])).containsExactly(holders);
    assertThat(table.get("Outcome", new MoveOutcome[0])).containsExactly(A_MOVE, A_MOVE);
    assertThat(table.get("Nothing", new Holder[] {Holder.interlock(9)})).isEmpty();
  }
}

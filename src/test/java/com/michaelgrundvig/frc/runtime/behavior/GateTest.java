package com.michaelgrundvig.frc.runtime.behavior;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.world.record.GateTerm;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.simulation.AlertSim;
import org.wpilib.simulation.SimHooks;

/**
 * Gates on their own: worked out once a loop, before what the scheduler runs, with their reasons.
 */
@ExtendWith(RobotSim.class)
class GateTest {
  private final Scheduler scheduler = Scheduler.createIndependentScheduler();

  private void loop() {
    RobotSim.stepClock();
    scheduler.run();
  }

  @Test
  void aGateHoldsWhatItsConditionSaysFromTheStart() {
    boolean[] clear = {true};
    Gate gate = Gate.of(scheduler, "intake clear", () -> clear[0]);

    boolean first = gate.getAsBoolean();
    clear[0] = false;
    boolean beforeTheLoop = gate.getAsBoolean();
    loop();

    assertThat(first).as("worked out as it's made").isTrue();
    assertThat(beforeTheLoop).as("then once a loop").isTrue();
    assertThat(gate.getAsBoolean()).isFalse();
    assertThat(gate.name()).isEqualTo("intake clear");
    assertThat(gate.why()).isEqualTo("intake clear is false");
  }

  /**
   * Within 2° to become true, and it stays true out to 4°: an error that wavers doesn't flicker.
   */
  @Test
  void withinBecomesTrueInsideEnterAndStaysTrueInsideStay() {
    double[] error = {5};
    Gate aimed = Gate.within(scheduler, "aimed", () -> error[0], 2, 4);
    List<Boolean> seen = new ArrayList<>();

    for (double each : new double[] {3, 1.9, 3.5, -3.9, 4.1, 3, -1.5}) {
      error[0] = each;
      loop();
      seen.add(aimed.getAsBoolean());
    }

    assertThat(seen).containsExactly(false, true, true, true, false, false, true);
  }

  @Test
  void allIsBlockedByTheTermsThatAreFalse() {
    boolean[] fast = {false};
    double[] error = {10};
    Gate atSpeed = Gate.of(scheduler, "shooter at goal", () -> fast[0]);
    Gate aimed = Gate.within(scheduler, "aimed", () -> error[0], 2, 4);
    Gate ready = Gate.all("ready to shoot", atSpeed, aimed);

    loop();
    List<String> both = ready.blocking();
    List<GateTerm> bothRecorded = ready.blockingRecorded();
    List<String> bothText = ready.blockingText();
    fast[0] = true;
    loop();
    List<String> one = ready.blocking();
    error[0] = 0;
    loop();

    assertThat(both).containsExactly("shooter at goal", "aimed");
    // A term that is another gate is a team's own text: the log holds where each text is.
    assertThat(bothRecorded).containsExactly(GateTerm.text(0), GateTerm.text(1));
    assertThat(bothText).containsExactly("shooter at goal", "aimed");
    assertThat(one).containsExactly("aimed");
    assertThat(ready.getAsBoolean()).isTrue();
    assertThat(ready.blocking()).isEmpty();
  }

  /**
   * Debounced on the falling edge, a gate rides out a dip shorter than its time, falls after one
   * longer, and holds nothing until it first has held: a fresh "ready to shoot" isn't true for its
   * first 80 ms.
   */
  @Test
  void aFallingDebounceRidesOutADipButNotAFall() {
    boolean[] ready = {false};
    Gate gate =
        Gate.of(scheduler, "ready", () -> ready[0])
            .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING);

    loop();
    boolean fresh = gate.getAsBoolean();
    ready[0] = true;
    loop();
    boolean up = gate.getAsBoolean();
    ready[0] = false;
    loop();
    loop();
    loop();
    boolean dipping = gate.getAsBoolean(); // 40 ms in: still true
    List<String> dippingWhy = gate.blocking();
    for (int i = 0; i < 3; i++) {
      loop();
    }

    assertThat(fresh).isFalse();
    assertThat(up).isTrue();
    assertThat(dipping).isTrue();
    assertThat(dippingWhy).isEmpty();
    assertThat(gate.getAsBoolean()).as("false for 100 ms: fallen").isFalse();
    assertThat(gate.name()).isEqualTo("ready");
  }

  @Test
  void aRisingDebounceWaitsAndSaysItsSettling() {
    boolean[] ready = {false};
    Gate gate =
        Gate.of(scheduler, "ready", () -> ready[0])
            .debounce(Milliseconds.of(60), Debouncer.DebounceType.RISING);

    ready[0] = true;
    loop();
    boolean early = gate.getAsBoolean();
    List<String> why = gate.blocking();
    for (int i = 0; i < 4; i++) {
      loop();
    }

    assertThat(early).isFalse();
    assertThat(why).containsExactly("ready settling");
    assertThat(gate.getAsBoolean()).isTrue();
  }

  /**
   * A gate debounced for a whole number of loops (0.1 s is five) changes on the same loop whatever
   * the robot's clock read as it was made: one script, run with the clock stepped an odd amount
   * before each run, answers the same every time. Until 2026-10-11 the edge came a loop early or
   * late with the clock's reading, because WPILib's debouncer subtracts seconds held in doubles.
   */
  @Test
  void aDebounceOfWholeLoopsChangesOnTheSameLoopWhateverTheClockReads() {
    loop(); // the robot runs, so every run's gate is made the same way
    // A debounced gate holds nothing until it has first held, so a falling one starts false.
    Map<Debouncer.DebounceType, String> gates =
        Map.of(
            Debouncer.DebounceType.RISING, DebounceTest.ANSWERS.get(Debouncer.DebounceType.RISING),
            Debouncer.DebounceType.FALLING, "001111111111111000",
            Debouncer.DebounceType.BOTH, DebounceTest.ANSWERS.get(Debouncer.DebounceType.BOTH));
    Map<String, String> answered = new LinkedHashMap<>();
    Map<String, String> expected = new LinkedHashMap<>();

    for (Debouncer.DebounceType type : Debouncer.DebounceType.values()) {
      for (int run = 0; run < DebounceTest.CLOCK_STEPS.length; run++) {
        SimHooks.stepTiming(DebounceTest.CLOCK_STEPS[run]);
        boolean[] ready = {false};
        Gate gate =
            Gate.of(scheduler, "ready", () -> ready[0]).debounce(Milliseconds.of(100), type);
        StringBuilder answers = new StringBuilder();
        for (char condition : DebounceTest.SCRIPT.toCharArray()) {
          ready[0] = condition == '1';
          loop();
          answers.append(gate.getAsBoolean() ? '1' : '0');
        }
        answered.put(type + " run " + run, answers.toString());
        expected.put(type + " run " + run, gates.get(type));
      }
    }

    assertThat(answered).containsExactlyEntriesOf(expected);
  }

  /** A gate binds and awaits like any trigger: a command waits on it, and a binding runs on it. */
  @Test
  void itBindsAndAwaitsAsATrigger() {
    boolean[] ready = {false};
    Gate gate = Gate.of(scheduler, "ready", () -> ready[0]);
    int[] ran = {0};
    boolean[] waited = {false};
    gate.onTrue(Command.noRequirements(co -> ran[0]++).named("On ready"));
    scheduler.schedule(
        Command.noRequirements(
                co -> {
                  co.waitUntil(gate);
                  waited[0] = true;
                })
            .named("Wait"));

    loop();
    loop();
    boolean before = waited[0];
    ready[0] = true;
    loop();

    assertThat(before).isFalse();
    assertThat(waited[0]).isTrue();
    assertThat(ran[0]).isEqualTo(1);
  }

  /**
   * A gate whose condition throws holds false while it does, saying what it threw, and the
   * scheduler runs on: its alert is raised while it throws, and lowered once it holds again.
   */
  @Test
  void aGateWhoseConditionThrowsHoldsFalseSayingWhyAndTheSchedulerRunsOn() {
    boolean[] broken = {false};
    Gate gate =
        Gate.of(
            scheduler,
            "intake clear",
            () -> {
              if (broken[0]) {
                throw new IllegalStateException("no reading yet");
              }
              return true;
            });
    int[] loops = {0};
    scheduler.schedule(
        Command.noRequirements(
                co -> {
                  while (true) {
                    loops[0]++;
                    co.yield();
                  }
                })
            .named("Counting"));

    loop();
    broken[0] = true;
    loop();
    boolean throwing = gate.getAsBoolean();
    String why = gate.why();
    List<String> raised = raised();
    loop();
    broken[0] = false;
    loop();

    assertThat(throwing).isFalse();
    assertThat(why).isEqualTo("intake clear is false: intake clear threw: no reading yet");
    assertThat(raised)
        .containsExactly(
            "intake clear threw: no reading yet. It holds false while it throws: fix its condition"
                + " (the stack is in the Driver Station's log)");
    assertThat(gate.getAsBoolean()).as("holding again").isTrue();
    assertThat(gate.blocking()).isEmpty();
    assertThat(raised()).as("lowered").isEmpty();
    assertThat(loops[0]).as("the scheduler ran its command every loop").isEqualTo(4);
  }

  /**
   * A gate that throws as it's made is made all the same, holding false, and blocks what's built on
   * it; an exception that says nothing is named by its kind.
   */
  @Test
  void aGateThatThrowsFromTheStartBlocksWhatsBuiltOnIt() {
    boolean[] broken = {true};
    Gate aimed =
        Gate.of(
            scheduler,
            "aimed",
            () -> {
              if (broken[0]) {
                throw new IllegalStateException();
              }
              return true;
            });
    Gate ready = Gate.all("ready to shoot", aimed);
    Gate settled = aimed.debounce(Milliseconds.of(40), Debouncer.DebounceType.FALLING);

    loop();
    List<String> aimedWhy = aimed.blocking();
    List<String> readyWhy = ready.blocking();
    List<String> settledWhy = settled.blocking();
    broken[0] = false;
    loop();

    assertThat(aimedWhy).containsExactly("aimed threw: IllegalStateException");
    assertThat(readyWhy).containsExactly("aimed");
    assertThat(settledWhy).containsExactly("aimed threw: IllegalStateException");
    assertThat(ready.getAsBoolean()).isTrue();
  }

  /**
   * A command that makes its gate on every run makes a gate of the same name each time, and each
   * one's condition may throw. Every one of them is contained and shown by the one alert of that
   * name: the command's runs and the scheduler go on. Until 2026-10-11 the second such gate threw
   * "Alert already allocated" from inside the catch that contains a throwing condition.
   */
  @Test
  void gatesOfOneNameMadeRunAfterRunEachThrowAndAreContained() {
    String[] thrown = {"no reading yet", "no reading yet", "sensor unplugged", ""};
    int[] run = {0};
    List<String> whys = new ArrayList<>();
    Command seek =
        Command.noRequirements(
                co -> {
                  String says = thrown[run[0]++];
                  Gate seen =
                      Gate.of(
                          scheduler,
                          "target seen",
                          () -> {
                            if (says.isEmpty()) {
                              return false;
                            }
                            throw new IllegalStateException(says);
                          });
                  whys.add(seen.why());
                  co.waitUntil(seen);
                })
            .named("Seek");
    int[] loops = {0};
    scheduler.schedule(
        Command.noRequirements(
                co -> {
                  while (true) {
                    loops[0]++;
                    co.yield();
                  }
                })
            .named("Counting"));
    loop(); // the robot runs, so each run's gate is made late
    List<List<String>> raised = new ArrayList<>();

    for (int each = 0; each < thrown.length; each++) {
      scheduler.schedule(seek);
      loop();
      loop();
      raised.add(raised());
      scheduler.cancel(seek);
      loop();
    }

    String fix =
        ". It holds false while it throws: fix its condition (the stack is in the Driver Station's"
            + " log)";
    assertThat(whys)
        .containsExactly(
            "target seen is false: target seen threw: no reading yet",
            "target seen is false: target seen threw: no reading yet",
            "target seen is false: target seen threw: sensor unplugged",
            "target seen is false");
    assertThat(raised.subList(0, 3))
        .as("one alert for the name, saying what the newest gate threw")
        .containsExactly(
            List.of("target seen threw: no reading yet" + fix),
            List.of("target seen threw: no reading yet" + fix),
            List.of("target seen threw: sensor unplugged" + fix));
    assertThat(loops[0]).as("the scheduler ran every loop").isEqualTo(1 + 3 * thrown.length);
  }

  /** The gates' alerts raised now. */
  private static List<String> raised() {
    return Arrays.stream(AlertSim.getActive())
        .filter(alert -> alert.group.equals("Gates"))
        .map(alert -> alert.text)
        .toList();
  }

  /**
   * Bands that can't be, made once the robot runs, are refused as a joint's gate on a place it
   * hasn't is: a gate that holds false, saying why.
   */
  @Test
  void badBandsMadeWhileTheRobotRunsHoldFalseSayingWhy() {
    loop();

    Gate aimed = Gate.within(scheduler, "aimed", () -> 0, 4, 2);
    loop();

    assertThat(aimed.getAsBoolean()).isFalse();
    assertThat(aimed.why())
        .isEqualTo(
            "aimed is false: aimed: enter within a positive band, and stay within at least it, not"
                + " 4.0 and 2.0");
  }

  @Test
  void badBandsAndEmptyGatesAreRefused() {
    assertThatThrownBy(() -> Gate.within(scheduler, "aimed", () -> 0, 4, 2))
        .hasMessage(
            "aimed: enter within a positive band, and stay within at least it, not 4.0 and 2.0");
    assertThatThrownBy(() -> Gate.all("nothing")).hasMessage("nothing has no terms");
    Gate other = Gate.of(Scheduler.createIndependentScheduler(), "elsewhere", () -> true);
    assertThatThrownBy(() -> Gate.all("mixed", Gate.of(scheduler, "here", () -> true), other))
        .hasMessage("mixed's terms run on different schedulers");
  }

  /**
   * A gate made once the robot runs (in a command, or as an auto is built) is worked out as it's
   * read, not every loop: a run's gates leave nothing running when the run is over.
   */
  @Test
  void aGateMadeOnceTheRobotRunsIsWorkedOutAsItsRead() {
    loop(); // the robot runs
    int[] asked = {0};
    boolean[] clear = {false};
    Gate gate =
        Gate.of(
            scheduler,
            "made late",
            () -> {
              asked[0]++;
              return clear[0];
            });
    int asMade = asked[0];

    for (int i = 0; i < 5; i++) {
      loop();
    }
    int unread = asked[0];
    clear[0] = true;

    assertThat(asMade).as("worked out as it's made").isEqualTo(1);
    assertThat(unread).as("and not again while nothing reads it").isEqualTo(1);
    assertThat(gate.getAsBoolean()).as("fresh as it's read, with no loop between").isTrue();
    clear[0] = false;
    assertThat(gate.why()).isEqualTo("made late is false");
  }

  /** Terms made late are fresh too when a gate of them is read, and it still drives a binding. */
  @Test
  void aLateGateOfLateTermsIsFreshAndStillBinds() {
    loop();
    boolean[] fast = {false};
    Gate atSpeed = Gate.of(scheduler, "at speed", () -> fast[0]);
    Gate ready = Gate.all("ready", atSpeed);
    boolean[] ran = {false};
    ready.onTrue(Command.noRequirements(co -> ran[0] = true).named("Fire"));

    loop();
    boolean before = ran[0];
    fast[0] = true;
    loop();
    loop();

    assertThat(before).isFalse();
    assertThat(ready.getAsBoolean()).isTrue();
    assertThat(ran[0]).as("the binding saw it rise").isTrue();
  }

  /** A logic whose answer and whose question a test sets: a gate on what something was asked. */
  private static final class Asked implements Gate.Logic {
    boolean done;
    long question;

    @Override
    public boolean holds(Gate.Terms why) {
      return done;
    }

    @Override
    public long question() {
      return question;
    }
  }

  /**
   * A gate made as the robot starts is worked out once a loop, and its answer stands for the loop:
   * a fact does not change under the commands. A gate on what something was asked is worked out
   * again when it is read after its question has changed, so the answer to the old question is
   * never handed on. So are the gates made of it.
   */
  @Test
  void aGateIsWorkedOutAgainWhenItsQuestionHasChangedSinceTheLoopBegan() {
    Asked shooter = new Asked();
    shooter.done = true;
    Gate atGoal = new Gate(scheduler, "shooter at goal", shooter);
    Gate ready = Gate.all("ready", atGoal, Gate.of(scheduler, "aimed", () -> true));
    loop();
    boolean[] idle = {atGoal.getAsBoolean(), ready.getAsBoolean()};

    shooter.done = false;
    boolean[] sameQuestion = {atGoal.getAsBoolean(), ready.getAsBoolean()};
    shooter.question++;
    boolean[] newQuestion = {ready.getAsBoolean(), atGoal.getAsBoolean()};
    List<String> why = ready.blocking();

    assertThat(idle).containsExactly(true, true);
    assertThat(sameQuestion).as("worked out once for the loop").containsExactly(true, true);
    assertThat(newQuestion)
        .as("worked out again for the new question")
        .containsExactly(false, false);
    assertThat(why).containsExactly("shooter at goal");
  }

  /**
   * A falling debounce rides out a dip in the answer to one question, and starts over on a new one:
   * the answer about the idle shooter is not held, for the debounce's time, for the command that
   * has just asked the shooter to spin up. Once the shooter is up, a dip is ridden out as ever.
   */
  @Test
  void aDebounceStartsOverWhenTheQuestionChanges() {
    Asked shooter = new Asked();
    shooter.done = true;
    Gate ready =
        new Gate(scheduler, "ready", shooter)
            .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING);
    loop();
    boolean idle = ready.getAsBoolean();

    shooter.done = false;
    shooter.question++;
    boolean asked = ready.getAsBoolean();
    loop();
    boolean spinningUp = ready.getAsBoolean();
    shooter.done = true;
    loop();
    boolean up = ready.getAsBoolean();
    shooter.done = false;
    loop();
    boolean dipping = ready.getAsBoolean();

    assertThat(idle).isTrue();
    assertThat(asked).as("in the turn the question changed").isFalse();
    assertThat(spinningUp).as("and for as long as the new answer is no").isFalse();
    assertThat(up).isTrue();
    assertThat(dipping).as("a dip in the answer to the same question").isTrue();
  }
}

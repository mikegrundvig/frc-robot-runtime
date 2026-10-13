package com.michaelgrundvig.frc.runtime.behavior;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Seconds;

import com.google.errorprone.annotations.CheckReturnValue;
import com.michaelgrundvig.frc.world.record.ControlSentences;
import com.michaelgrundvig.frc.world.record.GateTerm;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Time;

/**
 * A named condition that says why it is false. It is a v3 {@link Trigger}, so it binds ({@code
 * operator.rightBumper().and(readyToShoot).onTrue(...)}) and can be waited on ({@code
 * co.waitUntil(readyToShoot)}), and it adds a name and named terms. Every joint's checks are gates
 * already ({@code shooter.isAtGoal()} is "shooter at goal"), and a team's code makes its own:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.ExampleCommands region=gates}
 *
 * <p><b>The log.</b> Each time a gate is worked out it is logged: {@code Gates/ready to shoot =
 * false}, and the terms that are false, {@code Gates/ready to shoot/Blocking}. A term the robot's
 * model can name is logged as a fact: the joint that does not know where it is, the sensor that is
 * lost ({@link GateTerm}). A term in a team's own words, such as the name of another gate, is text,
 * logged beside the terms ({@code Gates/ready to shoot/BlockingText = [aimed]}). Terms go one level
 * deep. A gate is made of other gates, each one a lambda, and there is no expression language.
 *
 * <p><b>When it is worked out.</b> A gate made as the robot starts is worked out once a loop from
 * then on, before the scheduler's bindings and commands read it. A gate made once the robot runs
 * (inside a command, or as an auto is built) is worked out each time it is read instead. A gate
 * that is waited on or bound is read every loop, so the two come to the same thing. The difference
 * is that the second kind costs nothing once nobody reads it, so a command can make a new gate on
 * every run without leaving anything behind.
 *
 * <p><b>A gate on what was asked.</b> Most gates state a fact about the robot: where a joint is,
 * what a sensor reads. Such a fact, worked out once for a loop, is true for all of that loop. A few
 * gates instead say whether something has done what it was asked ({@code shooter.isAtGoal()}), and
 * what it is asked can change in the middle of a loop, when a command takes it or asks something
 * new of it. An answer worked out before that is an answer to a question nobody is asking any more.
 * So such a gate counts its questions ({@link Logic#question}), and one that is worked out once a
 * loop is worked out again when it is read after its question has changed. A gate made of other
 * gates ({@link #all}, {@link #debounce}) answers a new question whenever one of them does. So a
 * command that takes the shooter and then waits on "ready to feed" never passes on the answer for
 * the idle shooter it took, whatever order the loop runs its parts in.
 *
 * <p><b>A logic that throws.</b> The gate holds false while its logic throws, and its one term says
 * so ({@code aimed threw: ...}). The rest of the robot runs on. The first time, the gate reports a
 * Driver Station error with the stack, and an alert stays raised for as long as the logic throws.
 * Gates of the same name share that one alert, so a command that makes its gate on every run can
 * make one that throws every time.
 *
 * <p>What a gate has worked out, and the working out, is its {@link GateState}.
 */
@CheckReturnValue
public final class Gate extends Trigger {
  /** What a gate works out. */
  public interface Logic {
    /**
     * Whether the gate holds now.
     *
     * @param why where to add each term that keeps the gate false
     */
    boolean holds(Terms why);

    /**
     * Which question the gate is answering now, as a count that moves on each time the question
     * changes. A gate that states a fact about the robot answers one question for good, which is
     * what this says unless a logic says otherwise. A gate on whether something has done what it
     * was asked answers a new question each time it is asked something new ({@link Gate}).
     */
    default long question() {
      return 0;
    }
  }

  /**
   * The terms that keep a gate false, as its logic names them while it is worked out. A term is a
   * team's own words ({@link #add(String)}), or a fact the robot's model can name, added with how
   * it reads ({@link #add(GateTerm, String)}).
   */
  public static final class Terms {
    /** How each term reads, in the order they were added. */
    private final List<String> shown = new ArrayList<>();

    /** What each term is, in the same order. A term in words points at no text until logged. */
    private final List<GateTerm> facts = new ArrayList<>();

    /**
     * The text of each term that has some, in the same order, and empty for a term that has none: a
     * team's own words, or what a logic threw.
     */
    private final List<String> text = new ArrayList<>();

    Terms() {}

    /** Adds a term in a team's own words: the name of a gate, or whatever says what is false. */
    public void add(String term) {
      facts.add(IN_WORDS);
      shown.add(term);
      text.add(term);
    }

    /**
     * Adds a term the robot's model can name.
     *
     * @param term what it is: a joint that does not know where it is, a sensor that is lost
     * @param shown how it reads, from the model's function for it ({@link
     *     ControlSentences#gateTerm}), worked out once and not each loop
     */
    public void add(GateTerm term, String shown) {
      facts.add(term);
      this.shown.add(shown);
      text.add("");
    }

    /**
     * Adds the one term of a gate whose logic threw.
     *
     * @param gate the gate's name
     * @param thrown what was thrown, as it says itself
     * @return the term as it reads: "aimed threw: no target"
     */
    String addThrew(String gate, String thrown) {
      String reads = ControlSentences.threw(gate, thrown);
      facts.add(THREW);
      shown.add(reads);
      text.add(thrown);
      return reads;
    }

    /** Adds every term of another gate, as they are. */
    void addAll(Terms other) {
      facts.addAll(other.facts);
      shown.addAll(other.shown);
      text.addAll(other.text);
    }

    void clear() {
      facts.clear();
      shown.clear();
      text.clear();
    }

    /** How many terms there are. */
    int size() {
      return facts.size();
    }

    /**
     * One term as the log holds it. A term that has text points at where its text is among the
     * gate's texts, which is the next place in them.
     *
     * @param index which term, in the order they were added
     * @param textsSoFar how many of the terms before it have text
     */
    GateTerm logged(int index, int textsSoFar) {
      GateTerm fact = facts.get(index);
      return hasText(index) ? new GateTerm(fact.kind(), textsSoFar) : fact;
    }

    /** Whether a term has text to log beside the terms. */
    boolean hasText(int index) {
      GateTerm.Kind kind = facts.get(index).kind();
      return kind == GateTerm.Kind.TEXT || kind == GateTerm.Kind.THREW;
    }

    /** The text of a term that has some. */
    String text(int index) {
      return text.get(index);
    }

    /** How each term reads, in the order they were added. This is the list itself. */
    List<String> shown() {
      return shown;
    }
  }

  /**
   * What a term in a team's own words is while a gate is worked out. Its text gets its place among
   * the gate's texts only when the terms are logged, so that adding one in a loop makes nothing.
   */
  private static final GateTerm IN_WORDS = GateTerm.text(0);

  /** What the term of a logic that threw is while a gate is worked out, likewise. */
  private static final GateTerm THREW = new GateTerm(GateTerm.Kind.THREW, 0);

  /** What a debounced gate adds while what it reads has not yet held for its time. */
  private static final GateTerm SETTLING = GateTerm.settling();

  /** What a gate with a band remembers between workings out: whether it held the last time. */
  private static final class Remembered {
    boolean held;
  }

  private final Scheduler scheduler;
  private final String name;
  private final GateState state;

  /**
   * A gate whose logic names what keeps it false. A mechanism's or a sensor's gate is built from
   * this.
   */
  public Gate(Scheduler scheduler, String name, Logic logic) {
    this(scheduler, name, new GateState(name, logic));
  }

  private Gate(Scheduler scheduler, String name, GateState state) {
    super(scheduler, state);
    this.scheduler = scheduler;
    this.name = name;
    this.state = state;
    state.workOut();
    if (Refusals.starting(scheduler)) {
      scheduler.addPeriodic(state::workOut);
      return;
    }
    // The scheduler keeps every periodic task for good. So a gate made once the robot runs is not
    // given one: it is worked out as it is read, and nothing of it is left running when the
    // command that made it is over. For the same reason its trigger is unbound here: a v3 trigger
    // polls from the moment it is made, and an unbound one starts polling again only when a
    // command is bound to it.
    state.workOutWhenRead();
    unbind();
  }

  /** A gate on any condition: {@code Gate.of(scheduler, "in range", () -> distance() <= 5)}. */
  public static Gate of(Scheduler scheduler, String name, BooleanSupplier condition) {
    return new Gate(scheduler, name, why -> condition.getAsBoolean());
  }

  /**
   * A gate on how far something is off, in either direction: {@code Gate.within(scheduler, "aimed",
   * () -> headingError, 2.0, 4.0)}. It becomes true once the error is within {@code enter} of zero,
   * and then stays true until the error is more than {@code stay} off. With one band, an error that
   * wavers at the edge would make the gate flicker.
   *
   * <p>Bands that cannot be are refused ({@link Refusals#gate}): as the robot starts that is a
   * throw, and once it runs it is a gate that holds false and says why.
   *
   * @throws IllegalArgumentException as the robot starts, for an {@code enter} that is not
   *     positive, or a {@code stay} below it
   */
  public static Gate within(
      Scheduler scheduler, String name, DoubleSupplier error, double enter, double stay) {
    boolean bandsCanBe = enter > 0 && stay >= enter;
    if (!bandsCanBe) {
      return Refusals.gate(
          scheduler,
          name,
          name
              + ": enter within a positive band, and stay within at least it, not "
              + enter
              + " and "
              + stay);
    }
    Remembered inside = new Remembered();
    return new Gate(
        scheduler,
        name,
        why -> {
          double off = Math.abs(error.getAsDouble());
          double band = inside.held ? stay : enter;
          inside.held = off <= band;
          return inside.held;
        });
  }

  /**
   * A gate that is true when all its terms are, and is blocked by the ones that are not: {@code
   * Gate.all("ready to shoot", shooter.isAtGoal(), aimed)}.
   *
   * @throws IllegalArgumentException with no terms, or with terms of different schedulers
   */
  public static Gate all(String name, Gate... terms) {
    if (terms.length == 0) {
      throw new IllegalArgumentException(name + " has no terms");
    }
    Scheduler scheduler = terms[0].scheduler;
    for (Gate term : terms) {
      if (term.scheduler != scheduler) {
        throw new IllegalArgumentException(possessive(name) + " terms run on different schedulers");
      }
    }
    List<Gate> all = List.of(terms);
    return new Gate(
        scheduler,
        name,
        new Logic() {
          @Override
          public boolean holds(Terms why) {
            boolean holds = true;
            // Every term is asked, even after one is false, so that all the false ones are named.
            for (Gate term : all) {
              if (!term.state.getAsBoolean()) {
                why.add(term.name);
                holds = false;
              }
            }
            return holds;
          }

          /**
           * The questions its terms have answered between them. Each term's count only ever goes
           * up, so their sum moves on whenever any one of them does.
           */
          @Override
          public long question() {
            long questions = 0;
            // Counted by index, since a for-each over the list would make an iterator every read.
            for (int index = 0; index < all.size(); index++) {
              questions += all.get(index).state.question();
            }
            return questions;
          }
        });
  }

  /**
   * The same gate, debounced. On the falling edge it stays true until this gate has been false for
   * the time, which rides out a flywheel's dip as a shot leaves. On the rising edge it becomes true
   * only once this gate has been true for the time.
   *
   * <p>The debounced gate is logged under the same name, in place of this one. It holds nothing
   * until this gate has first held. Its time is kept in whole nanoseconds ({@link Debounce}), so a
   * time that is a whole number of loops ends on the same loop every run.
   *
   * <p>A debounce rides out a dip in the answer to one question. It does not carry an answer over
   * to the next one: when this gate's question changes ({@link Logic#question}), the debounced gate
   * starts over, as if this gate had never held. So a shot that waits on a debounced "ready to
   * shoot" is not let through, for the debounce's time, on the answer for the idle shooter it took.
   */
  @Override
  public Gate debounce(Time time, Debouncer.DebounceType type) {
    Debounce debouncer = new Debounce(time.in(Seconds), type);
    GateState undebounced = state;
    undebounced.stopLogging();
    String settling = ControlSentences.settling(name);
    return new Gate(
        scheduler,
        name,
        new Logic() {
          /** Whether the undebounced gate has held since its question last changed. */
          private boolean everHeld;

          /** The question the undebounced gate was answering the last time this was worked out. */
          private long debouncing = undebounced.question();

          @Override
          public boolean holds(Terms why) {
            boolean holdsNow = undebounced.getAsBoolean();
            long question = undebounced.question();
            if (question != debouncing) {
              debouncing = question;
              debouncer.startOver();
              everHeld = false;
            }
            everHeld |= holdsNow;
            // The debouncer is asked every loop, whatever the answer, so that its timer keeps
            // time.
            boolean debounced = debouncer.calculate(holdsNow);
            if (everHeld && debounced) {
              return true;
            }
            if (holdsNow) {
              why.add(SETTLING, settling);
            } else {
              why.addAll(undebounced.blocking());
            }
            return false;
          }

          @Override
          public long question() {
            return undebounced.question();
          }
        });
  }

  /**
   * A trigger that is true while this gate and a condition both hold. When the condition is a
   * gamepad's button the result is the button's own kind of trigger, which records what is bound to
   * it: {@code readyToShoot.and(operator.rightBumper())} is "right bumper and ready to shoot", the
   * same as when it is written the other way round.
   */
  @Override
  public Trigger and(BooleanSupplier condition) {
    if (condition instanceof RecordingTrigger button) {
      return button.and(this);
    }
    return super.and(condition);
  }

  /**
   * A trigger that is true while this gate or a condition holds. With a gamepad's button it is the
   * button's own kind of trigger, as for {@link #and}.
   */
  @Override
  public Trigger or(BooleanSupplier condition) {
    if (condition instanceof RecordingTrigger button) {
      return button.or(this);
    }
    return super.or(condition);
  }

  /** Its name, as logged: "ready to shoot". */
  public String name() {
    return name;
  }

  /** Whether it holds this loop. */
  @Override
  public boolean getAsBoolean() {
    return state.getAsBoolean();
  }

  /**
   * The terms that keep it false this loop, each as it reads. Empty while it holds, and always
   * empty for a gate without terms.
   */
  public List<String> blocking() {
    state.bringUpToDate();
    return List.copyOf(state.blocking().shown());
  }

  /**
   * The terms that keep it false this loop, as {@code Gates/<name>/Blocking} logs them: a fact for
   * each term the robot's model can name, and for each term in a team's own words, where its text
   * is in {@link #blockingText}.
   */
  public List<GateTerm> blockingRecorded() {
    state.bringUpToDate();
    return List.of(state.recorded().terms());
  }

  /**
   * The text of the terms that keep it false this loop and have text, as {@code
   * Gates/<name>/BlockingText} logs it: a team's own words, such as the name of a gate that is one
   * of its terms.
   */
  public List<String> blockingText() {
    state.bringUpToDate();
    return List.of(state.recorded().text());
  }

  /**
   * Why it is false, as a reason reads: "ready to shoot is false: aimed", or just "ready to shoot
   * is false" for a gate without terms.
   */
  public String why() {
    state.bringUpToDate();
    return ControlSentences.gateWhy(name, state.blocking().shown());
  }

  @Override
  public String toString() {
    return name;
  }
}

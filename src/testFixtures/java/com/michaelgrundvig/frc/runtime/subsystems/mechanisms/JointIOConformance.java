package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointBench.Rig.LEADER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROBOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.gearing;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.unmotored;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointBench.Ability;
import com.michaelgrundvig.frc.world.robot.ControllerPins;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.SensorWiring.Direction;
import com.michaelgrundvig.frc.world.robot.SensorWiring.Polarity;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * The contract a joint's driver keeps ({@link JointIO}), as tests: every driver passes every one of
 * these, so "the template supports this motor controller" is something a build checks.
 *
 * <p>A driver's test class extends this and hands it the driver's bench ({@link JointBench}). The
 * scenarios here know the driver only through {@link JointIO} and the bench: they ask the driver as
 * the robot does, move the joint and fail its devices through the bench, and check what the driver
 * reports and what the device does. Each is named for the clauses it checks ({@link JointClause}).
 *
 * <p><b>How a scenario asks.</b> As the robot does: what it last asked of the driver, it asks again
 * every loop ({@link Asking}). The contract has a request hold until the next all the same, and one
 * scenario stops asking to check that it does.
 *
 * <p><b>How a scenario waits.</b> Some simulated devices run on their own clock, so nothing here
 * assumes how many loops a device takes to answer. A scenario says what must become true ({@link
 * #eventually}), or become true and stay so ({@link #settles}), and gives up after {@link
 * #PATIENCE_LOOPS} loops.
 *
 * <p><b>Known differences.</b> A clause the drivers differ on, which nobody has yet decided, is
 * checked through {@link #clause}: a driver whose bench lists it as a known difference must still
 * fail it, so fixing the driver fails the suite until the entry is taken away.
 */
@ExtendWith(RobotSim.class)
public abstract class JointIOConformance {
  /** How many loops a scenario waits for something to become true before it fails. */
  private static final int PATIENCE_LOOPS = 100;

  /** How many loops something must stay true to be called steady. */
  private static final int STEADY_LOOPS = 5;

  /** Half a second of loops: the loop of a silence in which a device is first called silent. */
  private static final int SILENT_AFTER_LOOPS = 25;

  /** Half a second of loops and a few more: long enough for a silent device to be called silent. */
  private static final int SILENT_LOOPS = SILENT_AFTER_LOOPS + 5;

  /** How close a voltage asked for outright must be to the one expected, in volts. */
  private static final double VOLTS = 0.08;

  /**
   * How close the volts of a control law must be to the law's: this many volts and this share of
   * them. A simulated controller takes a setpoint and reads its sensor in steps of its own, a few
   * thousandths of a radian, which a loop's kP turns into a tenth of a volt. So the scenarios ask
   * for volts by the volt, where a gain in the wrong units or left out is far outside this.
   */
  private static final double LAW_VOLTS = 0.2;

  private static final double LAW_SHARE = 0.03;

  /**
   * A sixteenth of a turn a second, in radians a second. The speeds the scenarios ask a position's
   * profile to pass at are whole numbers of these: Phoenix's simulated TalonFX takes a position
   * request's speed in sixteenths of a rotation a second, and drops what is left over.
   */
  private static final double SIXTEENTH = 2 * Math.PI / 16;

  /**
   * The joints' limit on the current through the motor, in amps: high enough that the volts the
   * scenarios ask for stall a motor without reaching it. The scenarios keep under 3.5 V on a
   * stalled motor all the same, which is what a Kraken's 40 A from the battery comes to.
   */
  private static final double ROOMY_AMPS = 200;

  /**
   * The pivot's soft limits in the scenarios that give it some, in radians: just inside its stops,
   * at 0 and 2.36 rad, with its named places between them.
   */
  private static final double LOWER = 0.02;

  private static final double UPPER = 2.3;

  /** Where the pivot is put to be past each soft limit, in radians. */
  private static final double BELOW = 0;

  private static final double ABOVE = 2.33;

  private final JointBench bench;
  private final JointIO.JointIOInputs inputs = new JointIO.JointIOInputs();
  private JointBench.@Nullable Rig poweredOn;
  private @Nullable Asking asking;
  private @Nullable ControllerPlan plan;

  /**
   * The driver as a scenario asks it: each request goes straight to the driver, and is kept to be
   * made again every loop, as the joint's mechanism makes one every loop on a robot.
   *
   * <p>It is not only for likeness. Phoenix's simulated TalonFXS now and then goes on with the
   * request before the last (once in 150 requests, measured with Phoenix 26.70.0-alpha-2), and
   * takes the new one when it is made again. A robot never notices, and neither does a scenario
   * that asks every loop.
   */
  private static final class Asking implements JointIO {
    private final JointIO driver;

    /** The last request made, to make again. Nothing until the first. */
    private Runnable lastRequest = () -> {};

    Asking(JointIO driver) {
      this.driver = driver;
    }

    /** Makes the last request again. */
    void again() {
      lastRequest.run();
    }

    /** Stops making the last request again: it was made, and must hold. */
    void noMore() {
      lastRequest = () -> {};
    }

    private void request(Runnable request) {
      lastRequest = request;
      request.run();
    }

    @Override
    public void updateInputs(JointIOInputs inputs) {
      driver.updateInputs(inputs);
    }

    @Override
    public void setPosition(double position, double velocity, double feedforwardVolts) {
      request(() -> driver.setPosition(position, velocity, feedforwardVolts));
    }

    @Override
    public void setVelocity(double velocity) {
      request(() -> driver.setVelocity(velocity));
    }

    @Override
    public void setVoltage(double volts) {
      request(() -> driver.setVoltage(volts));
    }

    @Override
    public void setDutyCycle(double share) {
      request(() -> driver.setDutyCycle(share));
    }

    @Override
    public void stop() {
      request(driver::stop);
    }

    @Override
    public void home(double volts) {
      request(() -> driver.home(volts));
    }

    @Override
    public void resetPosition(double position) {
      driver.resetPosition(position);
    }

    @Override
    public void retune(MechanismControl control) {
      driver.retune(control);
    }
  }

  protected JointIOConformance(JointBench bench) {
    this.bench = bench;
  }

  @AfterEach
  void powerOff() {
    if (poweredOn != null) {
      poweredOn.close();
      poweredOn = null;
      asking = null;
    }
  }

  // The joints on the bench: the fixtures' joints, turned by the bench's motors on its controllers.

  /** The pivot on the bench's first controller, with room under its current limit. */
  private Joint.Revolute pivot() {
    return pivot(bench.controller(0));
  }

  private Joint.Revolute pivot(MechanismWiring.Device... controllers) {
    return unmotored(PIVOT)
        .motors(bench.motor().through(gearing(25)).on(controllers))
        .retuned(control -> control.withStatorLimit(Amps.of(ROOMY_AMPS)));
  }

  /** The pivot with soft limits, {@link #LOWER} to {@link #UPPER}. */
  private Joint.Revolute limitedPivot() {
    return pivot()
        .retuned(control -> control.withSoftLimits(Range.of(Radians.of(LOWER), Radians.of(UPPER))));
  }

  /** The elevator on two of the bench's controllers, through its drum. */
  private Joint.Prismatic elevator(MechanismWiring.Device leader, MechanismWiring.Device follower) {
    return unmotored(ELEVATOR)
        .motors(bench.motor().through(gearing(10)).drum(Inches.of(1)).on(leader, follower))
        .retuned(control -> control.withStatorLimit(Amps.of(ROOMY_AMPS)));
  }

  /** The roller, which runs at speeds. */
  private Joint.Revolute roller() {
    return unmotored(ROLLER)
        .motors(bench.motor().through(gearing(3)).on(bench.controller(0)))
        .retuned(control -> control.withStatorLimit(Amps.of(ROOMY_AMPS)));
  }

  /** The turret, which turns freely. */
  private Joint.Revolute turret() {
    return unmotored(TURRET)
        .motors(bench.motor().through(gearing(50)).on(bench.controller(0)))
        .retuned(control -> control.withStatorLimit(Amps.of(ROOMY_AMPS)));
  }

  /** A joint's control without its homing recipe: an absolute encoder says where the joint is. */
  private static Joint.Revolute unhomed(Joint.Revolute joint) {
    return joint.retuned(
        control -> MechanismControlBuilder.builder(control).homing(Optional.empty()).build());
  }

  // Powering on, and letting time pass.

  /** Powers the driver on for a joint that is where its definition rests it. */
  private JointIO powerOn(Joint joint) {
    return powerOn(joint, ControllerPins.NONE, joint.restPosition());
  }

  /**
   * Powers the driver on for a joint, in place of any powered on before, and waits for it to
   * answer.
   *
   * @param poweredOnAt where the joint really is as its devices power on
   */
  private JointIO powerOn(Joint joint, ControllerPins pins, double poweredOnAt) {
    powerOff();
    RobotDefinition robot = ROBOT.with(joint);
    plan = ControllerPlan.of(robot.math(), joint, ControllerSettings.control(joint));
    JointBench.Rig rig = bench.powerOn(robot, joint, pins, poweredOnAt);
    poweredOn = rig;
    Asking io = new Asking(rig.io());
    asking = io;
    eventually("it answers", () -> inputs.connected);
    return io;
  }

  private Asking asking() {
    return Objects.requireNonNull(asking, "nothing is powered on");
  }

  private JointBench.Rig rig() {
    return Objects.requireNonNull(poweredOn, "nothing is powered on");
  }

  private ControllerPlan plan() {
    return Objects.requireNonNull(plan, "nothing is powered on");
  }

  /**
   * One loop, as the robot runs one: time passes, the driver's inputs are read, and what was last
   * asked of the driver is asked again.
   */
  private void loop() {
    rig().step();
    asking().updateInputs(inputs);
    asking().again();
  }

  private void loops(int count) {
    for (int i = 0; i < count; i++) {
      loop();
    }
  }

  /** Waits for something to become true, a loop at a time, and fails if it never does. */
  private void eventually(String what, BooleanSupplier done) {
    for (int i = 0; i < PATIENCE_LOOPS; i++) {
      loop();
      if (done.getAsBoolean()) {
        return;
      }
    }
    fail("%s: %s never came true in %d loops. %s", bench.name(), what, PATIENCE_LOOPS, state());
  }

  /**
   * Waits for something to become true and stay true for a few loops running, and fails if it never
   * does. What a device passes through on its way somewhere else does not count.
   */
  private void settles(String what, BooleanSupplier holds) {
    int held = 0;
    for (int i = 0; i < PATIENCE_LOOPS; i++) {
      loop();
      held = holds.getAsBoolean() ? held + 1 : 0;
      if (held == STEADY_LOOPS) {
        return;
      }
    }
    fail(
        "%s: %s never held for %d loops running in %d loops. %s",
        bench.name(), what, STEADY_LOOPS, PATIENCE_LOOPS, state());
  }

  /** Waits for the voltage across the leading motor to reach a value, and checks it stays there. */
  private void settlesAt(String what, double volts) {
    settles(
        String.format(Locale.ROOT, "%s: %.2f V across the motor", what, volts),
        () -> Math.abs(rig().motorVolts(LEADER) - volts) <= VOLTS);
  }

  /**
   * Waits for the voltage across the leading motor to reach what a control law says, within what a
   * simulated controller's steps allow, and checks it stays there.
   */
  private void runsTheLaw(String what, double volts) {
    double close = LAW_VOLTS + LAW_SHARE * Math.abs(volts);
    settles(
        String.format(Locale.ROOT, "%s: the law's %.2f V across the motor", what, volts),
        () -> Math.abs(rig().motorVolts(LEADER) - volts) <= close);
  }

  /**
   * Waits for the leading motor to stop driving the joint one way, and checks it stays stopped. A
   * controller may put nothing across the motor, or drive the joint back: neither drives it on.
   *
   * @param way the way it must not drive: 1 for forward, -1 for backward
   * @param before the volts across the motor before the request that should stop it, well away from
   *     zero, so that the motor still showing them has not yet taken the request
   */
  private void stopsDriving(String what, double way, double before) {
    BooleanSupplier stopped = () -> way * rig().motorVolts(LEADER) <= VOLTS;
    BooleanSupplier tookTheRequest = () -> Math.abs(rig().motorVolts(LEADER) - before) > 0.5;
    settles(what, () -> tookTheRequest.getAsBoolean() && stopped.getAsBoolean());
  }

  /** What the driver reads and the device does now, for a failure's message. */
  private String state() {
    String followers = "";
    if (can(Ability.SHOW_A_FOLLOWERS_OUTPUT)) {
      for (int i = 0; i < inputs.followersConnected.length; i++) {
        followers +=
            String.format(Locale.ROOT, ", %.3f V across follower %d's", rig().motorVolts(1 + i), i);
      }
    }
    return String.format(
        Locale.ROOT,
        "Now: %.3f V across the motor%s; inputs: connected=%b fresh=%b position=%.4f velocity=%.4f"
            + " appliedVolts=%.3f stator=%.2f A supply=%.2f A forwardLimit=%b reverseLimit=%b"
            + " restarted=%b absolutePosition=%.4f motorPosition=%.4f",
        rig().motorVolts(LEADER),
        followers,
        inputs.connected,
        inputs.fresh,
        inputs.position,
        inputs.velocity,
        inputs.appliedVolts,
        inputs.statorCurrentAmps,
        inputs.supplyCurrentAmps,
        inputs.forwardLimit,
        inputs.reverseLimit,
        inputs.restarted,
        inputs.absolutePosition,
        inputs.motorPosition);
  }

  /**
   * Checks a clause the drivers are known to differ on. A driver whose bench lists the clause as a
   * known difference must still fail the check: once it passes, its entry is out of date.
   */
  private void clause(JointClause clause, Runnable check) {
    Optional<String> difference = bench.knownDifference(clause);
    if (difference.isEmpty()) {
      check.run();
      return;
    }
    try {
      check.run();
    } catch (AssertionError stillDiffers) {
      return;
    }
    fail(
        "%s is listed as differing from %s (%s), but it keeps the clause now: take the entry out of"
            + " its bench's known differences",
        bench.name(), clause, difference.get());
  }

  /** Skips the rest of a scenario on a bench that cannot do what it needs, saying why. */
  private void needs(Ability ability) {
    Optional<String> why = bench.lacks(ability);
    Assumptions.assumeTrue(
        why.isEmpty(), () -> bench.name() + " cannot " + ability + ": " + why.orElse(""));
  }

  private boolean can(Ability ability) {
    return bench.lacks(ability).isEmpty();
  }

  // What is close enough, for the joint powered on.

  /** How close a position must read, in the joint's units: a five-hundredth of a turn of it. */
  private double near() {
    return plan().rotations().fromRotations(0.002);
  }

  private boolean reads(double position) {
    return Math.abs(inputs.position - position) <= near();
  }

  private boolean moves(double velocity) {
    return Math.abs(inputs.velocity - velocity) <= 5 * near() + 0.02 * Math.abs(velocity);
  }

  /**
   * The volts the position law puts across the motor for a joint that reads still: kP on how far
   * the joint reads from the position asked, kD on the speed asked, the plan's feedforward, and the
   * volts the request adds.
   */
  private double positionLaw(
      ControllerPlan plan, double asked, double velocity, double feedforwardVolts) {
    return plan.kP() * (asked - inputs.position)
        + plan.kD() * velocity
        + plan.feedforward(asked, velocity, 0)
        + feedforwardVolts;
  }

  // Startup, units and direction.

  /**
   * A driver read by its motor's count starts by reading where the definition rests the joint,
   * wherever the joint really is, with its motors neutral and nothing to report. From there its
   * reading moves as far as the joint does.
   */
  @Test
  void j4j20j24j25_itStartsNeutralReadingWhereItsDefinitionRestsIt() {
    Joint.Revolute pivot = pivot();
    double rest = pivot.restPosition();

    powerOn(pivot, ControllerPins.NONE, 0.9);
    loops(JointIO.HEALTH_LOOPS);
    double readAtStartup = inputs.position;
    double voltsAtStartup = rig().motorVolts(LEADER);
    rig().jointIs(0.9 + 0.5);
    eventually("it reads half a radian on", () -> reads(rest + 0.5));

    assertThat(readAtStartup).as("J20: where it rests, not 0.9").isCloseTo(rest, within(near()));
    assertThat(voltsAtStartup).as("J4: neutral until asked").isZero();
    assertThat(inputs.appliedVolts).isZero();
    assertThat(inputs.fresh).isTrue();
    assertThat(inputs.restarted).isFalse();
    assertThat(inputs.unconfigured).as("J24").isEmpty();
    assertThat(inputs.absolutePosition).as("no absolute encoder").isNaN();
    assertThat(inputs.motorPosition).isNaN();
    assertThat(inputs.forwardLimit || inputs.reverseLimit).as("nothing on its pins").isFalse();
    assertThat(inputs.faults).as("J25: one device, no faults").containsExactly(0);
    assertThat(inputs.followersConnected).as("J25: no followers").isEmpty();
    assertThat(inputs.followersRestarted).isEmpty();
  }

  /** A joint wired just before it is powered on, where it is put, and how fast it moves. */
  private record Moving(String joint, Supplier<Joint> wired, double position, double velocity) {}

  /**
   * Positions and speeds read in the joint's units with the joint's sign, through a gearbox,
   * through a drum, and with the motor mounted the other way round; and a positive voltage drives
   * each of them forward.
   */
  @Test
  void j1j2_itReadsInTheJointsUnitsAndForwardIsTheJointsForward() {
    Supplier<Joint> pivot = () -> pivot();
    Supplier<Joint> mirrored = () -> pivot(bench.controller(0).clockwise());
    Supplier<Joint> elevator = () -> elevator(bench.controller(0), bench.controller(1));
    List<Moving> rows =
        List.of(
            new Moving("the pivot", pivot, 0.8, 1.5),
            new Moving("the pivot", pivot, 0.4, -1.5),
            new Moving("the pivot, its motor mounted clockwise", mirrored, 0.8, 1.5),
            new Moving("the pivot, its motor mounted clockwise", mirrored, 0.4, -1.5),
            new Moving("the elevator", elevator, 0.6, 0.4),
            new Moving("the elevator", elevator, 0.2, -0.4));

    for (Moving row : rows) {
      JointIO io = powerOn(row.wired().get());
      rig().jointMoves(row.position(), row.velocity());
      String moving =
          String.format(
              Locale.ROOT,
              "%s at %.2f moving at %.2f",
              row.joint(),
              row.position(),
              row.velocity());
      eventually("J1: " + moving, () -> reads(row.position()) && moves(row.velocity()));

      rig().jointIs(row.position());
      io.setVoltage(Math.signum(row.velocity()) * 2);
      settlesAt(
          "J2: " + row.joint() + " driven the way it was moving", row.velocity() > 0 ? 2 : -2);
    }
  }

  // Requests.

  /** Something asked of the driver, and the volts it should put across the motor. */
  private record Asked(String what, Consumer<JointIO> ask, double volts) {}

  /**
   * A voltage goes across the motor as asked and a duty cycle as its share of the battery's 12 V,
   * either way. The driver reports the same voltage with its sign, and currents that are never
   * negative. A request made once holds.
   */
  @Test
  void j3j8j9j21j22j23_voltsAndDutyCyclesGoAcrossTheMotorAndHold() {
    JointIO io = powerOn(pivot());
    rig().jointIs(0.8);
    List<Asked> rows =
        List.of(
            new Asked("J8: 2 V", driver -> driver.setVoltage(2), 2),
            new Asked("J8: -2 V", driver -> driver.setVoltage(-2), -2),
            new Asked("J9: 15% of the battery", driver -> driver.setDutyCycle(0.15), 1.8),
            new Asked("J9: -15% of the battery", driver -> driver.setDutyCycle(-0.15), -1.8));

    for (Asked row : rows) {
      row.ask().accept(io);
      settlesAt(row.what(), row.volts());

      assertThat(inputs.appliedVolts)
          .as("J21: %s", row.what())
          .isCloseTo(row.volts(), within(VOLTS));
      assertThat(inputs.statorCurrentAmps).as("J22: stalled, %s", row.what()).isGreaterThan(1);
      assertThat(inputs.supplyCurrentAmps).as("J23: stalled, %s", row.what()).isNotNegative();
    }
    asking().noMore();
    loops(SILENT_LOOPS);
    settlesAt("J3: last asked half a second ago", -1.8);
  }

  /**
   * A joint carried along faster than its motor's volts would drive it turns the motor into a
   * generator, which sends current back to the battery. The currents the driver reports are sizes
   * all the same: neither is negative.
   */
  @Test
  void j22j23_currentsAreNeverNegativeWhileTheMotorSendsCurrentBack() {
    JointIO io = powerOn(pivot());
    rig().jointMoves(0.8, 12);
    io.setVoltage(3);
    eventually("current flows", () -> inputs.statorCurrentAmps > 5);

    double leastStator = Double.POSITIVE_INFINITY;
    double leastSupply = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 10; i++) {
      loop();
      leastStator = Math.min(leastStator, inputs.statorCurrentAmps);
      leastSupply = Math.min(leastSupply, inputs.supplyCurrentAmps);
    }

    assertThat(leastStator).as("J22").isNotNegative();
    double least = leastSupply;
    clause(JointClause.J23, () -> assertThat(least).as("J23").isNotNegative());
  }

  /** A point of a profile asked of a joint that reads still at 0.8 rad. */
  private record Point(String what, double position, double velocity, double feedforwardVolts) {}

  /**
   * A position runs the control law on the controller's loop: kP on the error either way, kD on the
   * profile's speed with the feedforward for it, and the volts the request adds.
   */
  @Test
  void j5_aPositionRunsThePositionLaw() {
    JointIO io = powerOn(pivot());
    double here = 0.8;
    rig().jointIs(here);
    eventually("it reads", () -> reads(here));
    List<Point> rows =
        List.of(
            new Point("ahead of where it is", here + 0.1, 0, 0),
            new Point("behind where it is", here - 0.1, 0, 0),
            new Point("passing where it is, forward", here, 5 * SIXTEENTH, 0),
            new Point("passing where it is, backward", here, -5 * SIXTEENTH, 0),
            new Point("where it is, with volts added", here, 0, 1.5));

    for (Point row : rows) {
      io.setPosition(row.position(), row.velocity(), row.feedforwardVolts());

      runsTheLaw(
          "J5: " + row.what(),
          positionLaw(plan(), row.position(), row.velocity(), row.feedforwardVolts()));
    }
  }

  /**
   * A speed runs the velocity law on the controller's loop: kP on how far the speed read is from
   * the speed asked, and the plan's feedforward for that speed. A retune's kP is the speed loop's
   * too. With a kI tuned in, the volts stay where they are while the error lasts: kI is the
   * position loop's, and the loop the speeds run on holds none.
   */
  @Test
  void j6j7_aVelocityRunsTheVelocityLawWithoutKi() {
    Joint.Revolute roller = roller();
    JointIO io = powerOn(roller);
    rig().jointIs(0);

    for (double speed : new double[] {20, -20}) {
      io.setVelocity(speed);

      runsTheLaw(
          "J6: asked for " + speed + " rad/s, reading still",
          plan().kP() * speed + plan().feedforward(0, speed, 0));
    }

    MechanismControl integrating =
        ControllerSettings.control(roller).withKp(3 * plan().kP()).withKi(0.05);
    ControllerPlan tuned = ControllerPlan.of(ROBOT.with(roller).math(), roller, integrating);
    double law = tuned.kP() * 20 + tuned.feedforward(0, 20, 0);
    io.retune(integrating);
    io.setVelocity(20);
    runsTheLaw("J6: the law with the tuned kP, a kI tuned in beside it", law);
    double rate = voltsPerSecond(SILENT_LOOPS);
    clause(
        JointClause.J7,
        () -> {
          assertThat(rate)
              .as("J7: how fast the volts change while the error lasts, in volts a second")
              .isCloseTo(0, within(0.1));
          rig()
              .speedKi()
              .ifPresent(kI -> assertThat(kI).as("J7: the kI its speed loop holds").isZero());
        });
  }

  /**
   * Neutral puts no voltage across the motor, from whatever was asked before, and the controller
   * brakes or coasts as the joint's control says.
   */
  @Test
  void j10_neutralIsNoVoltageBrakingOrCoastingAsTheControlSays() {
    Map<MechanismControl.NeutralMode, UnaryOperator<MechanismControl>> rows = new LinkedHashMap<>();
    rows.put(MechanismControl.NeutralMode.BRAKE, UnaryOperator.identity());
    rows.put(MechanismControl.NeutralMode.COAST, MechanismControl::coasting);

    rows.forEach(
        (mode, control) -> {
          JointIO io = powerOn(pivot().retuned(control));
          rig().jointIs(0.8);
          io.setVoltage(2);
          settlesAt("driven", 2);
          io.stop();

          settlesAt("J10: neutral", 0);
          assertThat(rig().neutralMode()).as("J10: as its control says").isEqualTo(mode);
        });
  }

  // Limits.

  /** Each kind of request, pushing the joint one way: forward for 1, backward for -1. */
  private static Map<String, Consumer<JointIO>> requestsPushing(double way, double from) {
    Map<String, Consumer<JointIO>> requests = new LinkedHashMap<>();
    requests.put("a voltage", io -> io.setVoltage(way * 2));
    requests.put("a duty cycle", io -> io.setDutyCycle(way * 0.15));
    requests.put("a position", io -> io.setPosition(from + way * 0.2, 0, 0));
    requests.put("a speed", io -> io.setVelocity(way * 1.0));
    return requests;
  }

  /**
   * Past either soft limit, no request but a homing push drives the joint further past it, and a
   * request back toward its range goes through.
   */
  @Test
  void j11j13_noRequestButAHomingPushDrivesPastASoftLimit() {
    JointIO io = powerOn(limitedPivot());
    Map<Double, Double> ends = new LinkedHashMap<>();
    ends.put(ABOVE, 1.0);
    ends.put(BELOW, -1.0);

    ends.forEach(
        (past, way) -> {
          rig().jointIs(past);
          eventually("it reads past its limit", () -> reads(past));
          requestsPushing(way, past)
              .forEach(
                  (name, request) -> {
                    io.setVoltage(-way * 3);
                    settlesAt("J13: back toward its range from " + past, -way * 3);
                    request.accept(io);
                    stopsDriving(
                        "J13: " + name + " further past its limit at " + past, way, -way * 3);
                  });
          io.home(way * 2);
          settlesAt("J11: a homing push further past its limit at " + past, way * 2);
        });
  }

  /**
   * A homing push found the joint: told where it is, the driver reads there, and its soft limits
   * are in force again where that puts them.
   */
  @Test
  void j11j12j14_aHomingThatFindsTheJointPutsItsSoftLimitsBack() {
    JointIO io = powerOn(limitedPivot());
    rig().jointIs(ABOVE);
    eventually("it reads above its upper limit", () -> reads(ABOVE));
    io.setVoltage(2);
    settlesAt("J13: above its upper limit as it reads", 0);
    io.home(2);
    settlesAt("J11: the homing push", 2);

    // The stop it found is near the bottom of its travel, not where it read.
    io.resetPosition(0.05);
    eventually("J12: it reads where it was told", () -> reads(0.05));
    io.setVoltage(-2);
    settlesAt("inside its limits now, driven down", -2);
    rig().jointIs(ABOVE - 0.05);

    eventually("J12: it counts on from there", () -> reads(0));
    settlesAt("J14: below its lower limit, as it reads now", 0);
  }

  /**
   * A homing that is abandoned never tells the driver where the joint is. Whatever is asked next
   * obeys the soft limits again all the same.
   */
  @Test
  void j14_anAbandonedHomingsSoftLimitsAreBackAtTheNextRequest() {
    JointIO io = powerOn(limitedPivot());
    rig().jointIs(ABOVE);
    eventually("it reads above its upper limit", () -> reads(ABOVE));

    requestsPushing(1, ABOVE)
        .forEach(
            (name, request) -> {
              io.home(2);
              settlesAt("J11: the homing push before " + name, 2);
              request.accept(io);
              stopsDriving("J14: " + name + " after the push, above its upper limit", 1, 2);
            });
  }

  private static ControllerPins pins(Polarity forward, Polarity reverse, boolean stop) {
    return ControllerPins.NONE
        .with(Direction.FORWARD, new ControllerPins.OnPin(forward, stop))
        .with(Direction.REVERSE, new ControllerPins.OnPin(reverse, stop));
  }

  private boolean limitReads(Direction direction) {
    return direction == Direction.FORWARD ? inputs.forwardLimit : inputs.reverseLimit;
  }

  private static Direction other(Direction direction) {
    return direction == Direction.FORWARD ? Direction.REVERSE : Direction.FORWARD;
  }

  /**
   * A limit switch on a pin of the controller, of either polarity, on either input: tripped, it
   * reads tripped and the other does not, and nothing drives the joint toward it, a homing push
   * included, while a request away from it goes through. Released, the joint is driven again.
   */
  @Test
  void j15j16_aTrippedLimitSwitchStopsEveryRequestTowardIt() {
    List<ControllerPins> rows =
        List.of(
            pins(Polarity.NORMALLY_OPEN, Polarity.NORMALLY_CLOSED, true),
            pins(Polarity.NORMALLY_CLOSED, Polarity.NORMALLY_OPEN, true));

    for (ControllerPins pins : rows) {
      JointIO io = powerOn(pivot(), pins, PIVOT.restPosition());
      rig().jointIs(0.8);
      for (Direction direction : Direction.values()) {
        double toward = direction == Direction.FORWARD ? 2 : -2;
        String switched = direction + " switch of " + pins;
        io.setVoltage(toward);
        settlesAt("released, the " + switched, toward);
        settles("J16: both released", () -> !inputs.forwardLimit && !inputs.reverseLimit);

        rig().limit(direction, true);
        eventually(
            "J16: tripped, the " + switched,
            () -> limitReads(direction) && !limitReads(other(direction)));
        settlesAt("J15: driven toward the tripped " + switched, 0);
        io.home(toward);
        settlesAt("J15: a homing push toward the tripped " + switched, 0);
        io.setVoltage(-toward);
        settlesAt("J15: driven away from the tripped " + switched, -toward);

        rig().limit(direction, false);
        io.setVoltage(toward);
        eventually("J16: released again, the " + switched, () -> !limitReads(direction));
        settlesAt("J15: released again, the " + switched, toward);
      }
    }
  }

  /** A switch the wiring says not to stop on, a beam, reads tripped and stops nothing. */
  @Test
  void j15j16_ASwitchThatDoesNotStopTheMotorIsOnlyReported() {
    JointIO io =
        powerOn(
            pivot(),
            pins(Polarity.NORMALLY_OPEN, Polarity.NORMALLY_OPEN, false),
            PIVOT.restPosition());
    rig().jointIs(0.8);
    io.setVoltage(2);
    settlesAt("driven forward", 2);

    rig().limit(Direction.FORWARD, true);

    eventually("J16: it reads tripped", () -> inputs.forwardLimit);
    settlesAt("J15: driven on", 2);
  }

  /**
   * A limit the controller reads from another device stops the motor as one on its pin does, and is
   * not reported with the joint: the robot reads that switch as a sensor of its own.
   */
  @Test
  void j17_aLimitReadElsewhereStopsTheMotorAndIsNotReported() {
    Optional<ControllerPins.Limit> elsewhere = bench.limitReadElsewhere();
    Assumptions.assumeTrue(
        elsewhere.isPresent(), bench.name() + " reads no limit from another device");
    ControllerPins pins = ControllerPins.NONE.with(Direction.FORWARD, elsewhere.orElseThrow());
    JointIO io = powerOn(pivot(), pins, PIVOT.restPosition());
    rig().jointIs(0.8);
    io.setVoltage(2);
    settlesAt("driven forward", 2);

    rig().limit(Direction.FORWARD, true);

    settlesAt("J17: stopped", 0);
    assertThat(inputs.forwardLimit).as("J17: not reported with the joint").isFalse();
  }

  /**
   * Told where the joint is, the driver reads there within a loop or two and counts on from there
   * as the joint moves. What was asked of it still stands.
   */
  @Test
  void j12_toldWhereTheJointIsItReadsThereAndCountsOn() {
    JointIO io = powerOn(pivot());
    rig().jointIs(0.8);
    io.setVoltage(1);
    settlesAt("driven", 1);

    io.resetPosition(0.2);
    eventually("J12: it reads where it was told", () -> reads(0.2));
    rig().jointIs(0.9);

    eventually("J12: a tenth of a radian on", () -> reads(0.3));
    settlesAt("J12: what was asked still stands", 1);
  }

  // Retune.

  /**
   * Everything a retune promises is in force at the next request: kP, kD and kS in the position
   * law, a kI that integrates at its rate and one tuned back to nothing that stops, the limit on
   * the current through the motor, and, where the device has one, the limit on what it draws.
   */
  @Test
  void j18_aRetunePutsItsGainsAndCurrentLimitsInForce() {
    Joint.Revolute pivot = pivot();
    JointIO io = powerOn(pivot);
    double here = 0.8;
    rig().jointIs(here);
    eventually("it reads", () -> reads(here));
    MechanismControl tuned = ControllerSettings.control(pivot).withKp(50).withKd(1.5).withKs(0.5);
    ControllerPlan tunedPlan = ControllerPlan.of(ROBOT.with(pivot).math(), pivot, tuned);

    io.retune(tuned);
    io.setPosition(here + 0.06, 0, 0);
    runsTheLaw("J18: the tuned kP", positionLaw(tunedPlan, here + 0.06, 0, 0));
    io.setPosition(here, 3 * SIXTEENTH, 0);
    runsTheLaw("J18: the tuned kD and kS", positionLaw(tunedPlan, here, 3 * SIXTEENTH, 0));

    io.retune(tuned.withKi(20));
    io.setPosition(here + 0.03, 0, 0);
    // 20 V a radian-second on an error of 0.03 rad.
    double rate = integralsRate(positionLaw(tunedPlan, here + 0.03, 0, 0), 0.03);
    io.retune(tuned);
    io.setPosition(here + 0.03, 0, 0);
    loops(STEADY_LOOPS);
    double rateTunedAway = integralsRate(positionLaw(tunedPlan, here + 0.03, 0, 0), 0.03);

    io.setVoltage(6);
    eventually("the motor stalls", () -> inputs.statorCurrentAmps > 45);
    io.retune(tuned.withStatorLimit(Amps.of(30)).withSupplyLimit(Amps.of(35)));
    io.setVoltage(6);
    settles(
        "J18: the tuned stator limit",
        () -> inputs.statorCurrentAmps < 30 * 1.15 && inputs.statorCurrentAmps > 30 * 0.6);

    assertThat(rate).as("J18: kI's rate, in volts a second").isCloseTo(0.6, within(0.2));
    assertThat(rateTunedAway).as("J18: a kI tuned back to nothing").isCloseTo(0, within(0.1));
    if (can(Ability.SHOW_THE_SUPPLY_LIMIT)) {
      eventually(
          "J18: the tuned supply limit",
          () -> Math.abs(rig().supplyLimitAmps().orElse(Double.NaN) - 35) < 0.5);
    }
  }

  /**
   * How fast the position loop's integral adds volts while the joint reads an error, in volts a
   * second. It is measured from when the motor shows the request's volts, over the half second
   * after. A bench whose simulated controller does not add its integral up in time says what kI the
   * controller took instead, and the rate is worked out from that.
   *
   * @param lawVolts the volts the law gives before the integral adds any
   * @param error how far the joint reads from the position asked, in its units
   */
  private double integralsRate(double lawVolts, double error) {
    if (!can(Ability.INTEGRATE_IN_TIME)) {
      return rig().positionKi() * error;
    }
    runsTheLaw("J18: the request the integral works on", lawVolts);
    return voltsPerSecond(SILENT_LOOPS);
  }

  /** How fast the voltage across the leading motor changes over some loops, in volts a second. */
  private double voltsPerSecond(int loops) {
    double volts = rig().motorVolts(LEADER);
    double seconds = rig().deviceSeconds();
    loops(loops);
    return (rig().motorVolts(LEADER) - volts) / (rig().deviceSeconds() - seconds);
  }

  /**
   * A retune changes the gains and the current limits and nothing else: tuned soft limits and a
   * tuned neutral mode leave the controller's as they were at startup.
   */
  @Test
  void j19_aRetuneChangesNothingButGainsAndCurrentLimits() {
    Joint.Revolute pivot = limitedPivot();
    JointIO io = powerOn(pivot);
    rig().jointIs(1.5);
    eventually("it reads between its limits", () -> reads(1.5));
    MechanismControl tuned =
        ControllerSettings.control(pivot)
            .withSoftLimits(Range.of(Radians.of(LOWER), Radians.of(1.0)))
            .coasting();

    io.retune(tuned);
    io.setVoltage(2);

    clause(
        JointClause.J19,
        () -> {
          settlesAt("J19: past the tuned upper limit, inside the one it started with", 2);
          assertThat(rig().neutralMode())
              .as("J19: as at startup")
              .isEqualTo(MechanismControl.NeutralMode.BRAKE);
        });
  }

  // Lost frames, silence, restarts.

  /**
   * A leading controller that restarts while it holds the joint says so in exactly one loop and
   * counts from zero. By the next request the tuning it had is in force again, and its soft limits.
   */
  @Test
  void j31j32_aRestartedLeaderSaysSoOnceAndKeepsItsTuningAndSoftLimits() {
    needs(Ability.RESTART_THE_LEADER);
    Joint.Revolute pivot = limitedPivot();
    JointIO io = powerOn(pivot);
    rig().jointIs(0.8);
    eventually("it reads", () -> reads(0.8));
    MechanismControl tuned = ControllerSettings.control(pivot).withKp(50);
    ControllerPlan tunedPlan = ControllerPlan.of(ROBOT.with(pivot).math(), pivot, tuned);
    io.retune(tuned);
    io.setPosition(0.8, 0, 0);
    runsTheLaw("holding where it is", positionLaw(tunedPlan, 0.8, 0, 0));

    rig().restart(LEADER);
    int saidSo = 0;
    for (int i = 0; i < 2 * STEADY_LOOPS; i++) {
      loop();
      saidSo += inputs.restarted ? 1 : 0;
    }
    assertThat(saidSo).as("J31: said in exactly one loop").isEqualTo(1);
    eventually("J31: it counts from zero", () -> reads(0));

    io.setVoltage(-2);
    settlesAt("J32: below its lower soft limit as it reads now", 0);
    io.setPosition(0.05, 0, 0);
    runsTheLaw("J32: the tuned kP", positionLaw(tunedPlan, 0.05, 0, 0));
  }

  /**
   * In a loop the leading controller's frame does not come, the joint's position and speed stand as
   * last read, the driver says the reading is not fresh, and the controller still counts as
   * connected. The next frame is fresh.
   */
  @Test
  void j27j28j29_aLostFrameLeavesTheLastReadingStanding() {
    needs(Ability.LOSE_A_FRAME);
    powerOn(pivot());
    rig().jointMoves(0.8, 1.0);
    eventually("it reads moving", () -> reads(0.8) && moves(1.0));
    double position = inputs.position;
    double velocity = inputs.velocity;

    rig().loseAFrame();
    rig().jointMoves(1.2, -1.0);
    loop();

    assertThat(inputs.position).as("J28: its position stands").isEqualTo(position);
    assertThat(inputs.velocity).as("J28: its speed stands").isEqualTo(velocity);
    assertThat(inputs.connected).as("J27: one frame is a glitch").isTrue();
    clause(JointClause.J29, () -> assertThat(inputs.fresh).as("J29: not fresh").isFalse());
    eventually("the next frame", () -> inputs.fresh && reads(1.2) && moves(-1.0));
  }

  /**
   * A leading controller that goes silent keeps its last readings and counts as connected for half
   * a second, to the loop, then as silent. When it answers again it counts as connected at once,
   * reads where the joint has got to, and does what it is asked next.
   */
  @Test
  void j26j27j28j30_aSilentLeaderIsCalledSilentAfterHalfASecondAndComesBack() {
    needs(Ability.SILENCE_THE_LEADER);
    JointIO io = powerOn(pivot());
    rig().jointIs(0.8);
    eventually("it reads", () -> reads(0.8));
    double position = inputs.position;

    rig().silence(LEADER, true);
    rig().jointIs(1.0);
    io.setVoltage(-1);
    int loopsReadConnected = 0;
    for (int i = 0; i < SILENT_LOOPS; i++) {
      loop();
      loopsReadConnected += inputs.connected ? 1 : 0;
    }
    assertThat(loopsReadConnected)
        .as(
            "J26, J27: the loops of a silence it reads connected in. Half a second is 25 loops, so"
                + " the twenty-fifth is the first to read silent, on every driver")
        .isEqualTo(SILENT_AFTER_LOOPS - 1);
    assertThat(inputs.connected).as("J26: called silent").isFalse();
    assertThat(inputs.position).as("J28: its last reading stands").isEqualTo(position);

    rig().silence(LEADER, false);
    eventually(
        "J26: connected again, where the joint is now", () -> inputs.connected && reads(1.0));
    io.setVoltage(2);
    settlesAt("J30: asked after the silence", 2);
  }

  /**
   * A joint on two controllers, the second mounted the same way round as the first or the other:
   * each follower is listed with the joint's devices and drives the joint the way its leader does.
   * A follower that restarts says so once and follows again. One that goes silent reads as silent
   * after half a second, while the leader still reads as connected.
   */
  @Test
  void j25j33j34j35_followersDriveTheJointAsTheirLeaderDoes() {
    Map<String, UnaryOperator<MechanismWiring.Device>> rows = new LinkedHashMap<>();
    rows.put("mounted as the leader is", UnaryOperator.identity());
    rows.put("mounted the other way round", MechanismWiring.Device::clockwise);
    int follower = LEADER + 1;

    rows.forEach(
        (mounted, mount) -> {
          JointIO io = powerOn(elevator(bench.controller(0), mount.apply(bench.controller(1))));
          rig().jointIs(0.6);
          eventually("its follower answers", () -> followerReadsConnected());
          assertThat(inputs.followersRestarted).as("J25").containsExactly(false);
          assertThat(inputs.faults).as("J25: leader and follower, no faults").containsExactly(0, 0);

          for (double volts : new double[] {2, -2}) {
            io.setVoltage(volts);
            settlesAt("the leader, its follower " + mounted, volts);
            if (can(Ability.SHOW_A_FOLLOWERS_OUTPUT)) {
              eventually(
                  "J33: the follower " + mounted + " drives the joint at " + volts + " V too",
                  () -> Math.abs(rig().motorVolts(follower) - volts) <= VOLTS);
              assertThat(rig().motorVolts(follower)).as("J33").isCloseTo(volts, within(VOLTS));
            }
          }

          if (can(Ability.RESTART_A_FOLLOWER)) {
            rig().restart(follower);
            int saidSo = 0;
            for (int i = 0; i < 2 * STEADY_LOOPS; i++) {
              loop();
              saidSo += inputs.followersRestarted[0] ? 1 : 0;
              assertThat(inputs.restarted).as("J34: the leader did not restart").isFalse();
            }
            assertThat(saidSo).as("J34: said in exactly one loop").isEqualTo(1);
            if (can(Ability.SHOW_A_FOLLOWERS_OUTPUT)) {
              eventually(
                  "J34: the follower " + mounted + " follows again",
                  () -> Math.abs(rig().motorVolts(follower) + 2) <= VOLTS);
            }
          }

          if (can(Ability.SILENCE_A_FOLLOWER)) {
            rig().silence(follower, true);
            eventually("J35: the follower reads as silent", () -> !followerReadsConnected());
            assertThat(inputs.connected).as("J35: the leader still answers").isTrue();
            rig().silence(follower, false);
            eventually("J35: the follower answers again", () -> followerReadsConnected());
          }
        });
  }

  private boolean followerReadsConnected() {
    return inputs.followersConnected.length == 1 && inputs.followersConnected[0];
  }

  // Absolute encoders.

  /**
   * An absolute encoder the controller reads beside its motor's count tells the count where the
   * joint is at startup, wherever it was powered on. From then on the position is the count's, and
   * the encoder's reading is reported beside it each loop, so an encoder that slips shows. A
   * restarted leader is told where the joint is by the encoder again.
   */
  @Test
  void j36j37_anEncoderBesideTheCountSeedsItAndIsReportedBesideIt() {
    Optional<MechanismWiring.EncoderWiring> wiring = bench.encoderBesideTheCount();
    Assumptions.assumeTrue(
        wiring.isPresent(), bench.name() + " reads no absolute encoder beside its count");
    Joint.Revolute pivot = pivot().absoluteEncoder(wiring.orElseThrow());
    double poweredOnAt = Math.toRadians(60);

    powerOn(pivot, ControllerPins.NONE, poweredOnAt);
    eventually(
        "J37: seeded from its encoder at startup",
        () -> reads(poweredOnAt) && Math.abs(inputs.absolutePosition - poweredOnAt) <= near());
    rig().jointIs(1.3);
    eventually(
        "J36: both follow the joint",
        () -> reads(1.3) && Math.abs(inputs.absolutePosition - 1.3) <= near());
    rig().encoderSlips(0.2);
    settles(
        "J36: the slipped encoder beside the count",
        () -> reads(1.3) && Math.abs(inputs.absolutePosition - 1.5) <= near());
    assertThat(inputs.motorPosition).as("its position is its motor's count").isNaN();

    if (can(Ability.RESTART_THE_LEADER)) {
      rig().restart(LEADER);
      eventually("J31: it says it restarted", () -> inputs.restarted);
      settles("J37: seeded from its encoder again", () -> reads(1.5));
    }
  }

  /**
   * An absolute encoder the controller's loop runs on is where the joint reads, every loop. A
   * driver that reads the motor's count beside it reports that too.
   */
  @Test
  void j38_anEncoderTheLoopRunsOnIsWhereTheJointReads() {
    Optional<MechanismWiring.EncoderWiring> wiring = bench.encoderTheLoopRunsOn();
    Assumptions.assumeTrue(
        wiring.isPresent(), bench.name() + " runs its loop on no absolute encoder");
    Joint.Revolute pivot = unhomed(pivot()).absoluteEncoder(wiring.orElseThrow());
    double poweredOnAt = Math.toRadians(60);

    powerOn(pivot, ControllerPins.NONE, poweredOnAt);
    eventually("J38: it reads where its encoder says at startup", () -> reads(poweredOnAt));
    rig().jointIs(1.3);
    eventually("J38: it follows the joint", () -> reads(1.3));
    rig().encoderSlips(0.2);

    settles("J38: the encoder's word, slipped", () -> reads(1.5));
    boolean showsTheCount =
        !Double.isNaN(inputs.motorPosition)
            && can(Ability.TURN_THE_MOTOR_APART_FROM_ITS_LOOPS_ENCODER);
    if (showsTheCount) {
      assertThat(inputs.motorPosition)
          .as("J38: the motor's count beside it, which did not slip")
          .isCloseTo(1.3, within(near()));
    }
  }

  // A joint that turns freely.

  /**
   * A joint with no stops reads on past a whole turn either way, and a position more than half a
   * turn off is gone to the way its number says.
   */
  @Test
  void j39j40_aJointThatTurnsFreelyReadsPastATurnAndGoesTheWayAsked() {
    JointIO io = powerOn(turret());

    for (double position : new double[] {7.5, -7.5}) {
      rig().jointIs(position);
      eventually("J39: it reads " + position + " rad", () -> reads(position));
    }
    rig().jointIs(0.5);
    eventually("it reads", () -> reads(0.5));
    io.setPosition(0.5 + 0.75 * 2 * Math.PI, 0, 0);

    clause(
        JointClause.J40,
        () ->
            settles(
                "J40: driven forward, the way to three quarters of a turn on",
                () -> rig().motorVolts(LEADER) > 1));
  }
}

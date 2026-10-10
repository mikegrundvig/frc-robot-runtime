package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_DOWN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ELEVATOR_TOP;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_BACK;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.TURRET_FRONT;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.control;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.homed;
import static com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointFixtures.profile;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.GoldenFile;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;

/**
 * What holds a joint beneath its commands, and how it finds where it is, recorded loop by loop: its
 * interlocks, on another joint's place and speed and on a sensor, a region overshot into and one
 * reaching the end of its range, with the joint and the sensor they read lost; its limit switches;
 * its homing, into a stop and to a switch, found and timed out; and an absolute encoder the robot
 * reads, waited for, seeded, closed on and lost. Every second loop, for each joint a scene drives:
 * what its IO was told last (a position, its speed and feedforward; a speed; volts; a share; a
 * stop; a push home) and the position it was last told it's at, since the row before, where it
 * really is and where it reads, its goal, and, as each changes, its state, what holds it, how its
 * last move ended, and its limit switches' doubts. {@link JointRecordings}' sessions record the
 * moves themselves.
 *
 * <p>It must come out as recorded: a change meant to change none of this, such as how the joint's
 * mechanism is divided, is shown to change nothing. When a change is meant to move it, record it
 * again with {@code GOLDEN_UPDATE=1} and say why it moved (docs/testing.md).
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class JointStagesRecordedSimTest {
  private static final String HEADER =
      "scene,loop,joint,told,a,b,c,reset to,position,read at,goal,state,held by,outcome,doubts";

  private final List<String> rows = new ArrayList<>(List.of(HEADER));

  /**
   * A joint's simulated IO, recording what it's told each loop, and faulted on the test's say: its
   * motor controllers silent, or one that didn't take its settings.
   */
  private static final class Told implements JointIO {
    private final JointIO io;
    String request = "";
    double[] values = {Double.NaN, Double.NaN, Double.NaN};
    double reset = Double.NaN;
    boolean silent;
    String unconfigured = "";

    Told(JointIO io) {
      this.io = io;
    }

    private void told(String request, double a, double b, double c) {
      this.request = request;
      values = new double[] {a, b, c};
    }

    @Override
    public void updateInputs(JointIOInputs inputs) {
      io.updateInputs(inputs);
      if (silent) {
        inputs.connected = false;
      }
      if (!unconfigured.isEmpty()) {
        inputs.unconfigured = unconfigured;
      }
    }

    @Override
    public void setPosition(double position, double velocity, double feedforwardVolts) {
      told("position", position, velocity, feedforwardVolts);
      io.setPosition(position, velocity, feedforwardVolts);
    }

    @Override
    public void setVelocity(double velocity) {
      told("velocity", velocity, Double.NaN, Double.NaN);
      io.setVelocity(velocity);
    }

    @Override
    public void setVoltage(double volts) {
      told("volts", volts, Double.NaN, Double.NaN);
      io.setVoltage(volts);
    }

    @Override
    public void setDutyCycle(double share) {
      told("share", share, Double.NaN, Double.NaN);
      io.setDutyCycle(share);
    }

    @Override
    public void stop() {
      told("stop", Double.NaN, Double.NaN, Double.NaN);
      io.stop();
    }

    @Override
    public void home(double volts) {
      told("home", volts, Double.NaN, Double.NaN);
      io.home(volts);
    }

    @Override
    public void resetPosition(double position) {
      reset = position;
      io.resetPosition(position);
    }

    @Override
    public void retune(MechanismControl control) {
      io.retune(control);
    }
  }

  /** One scene: a robot's joints in the simulator, the ones it records, and what each was told. */
  private final class Scene {
    final String name;
    final JointsSimHarness sim;
    final List<Joint> recorded;
    final Map<Joint, Told> told = new HashMap<>();
    final Map<String, String> last = new HashMap<>();
    int loop;

    Scene(String name, RobotDefinition robot, Joint... recorded) {
      this.name = name;
      this.recorded = List.of(recorded);
      sim =
          new JointsSimHarness(
              robot,
              (joint, io) -> {
                Told each = new Told(io);
                told.put(joint, each);
                return each;
              });
    }

    JointMechanism of(Joint joint) {
      return sim.joints.of(joint);
    }

    /** A joint's IO, as the scene's test has it. */
    Told io(Joint joint) {
      return Objects.requireNonNull(told.get(joint), joint.label());
    }

    void schedule(Command command) {
      sim.scheduler.schedule(command);
    }

    /** Runs loops for a time, recording every second one. */
    void run(double seconds) {
      for (int i = 0; i < Math.round(seconds / 0.02); i++) {
        loop();
      }
    }

    /** Runs loops until something holds, at most for a time. */
    void runUntil(BooleanSupplier done, double seconds) {
      for (int i = 0; i < Math.round(seconds / 0.02) && !done.getAsBoolean(); i++) {
        loop();
      }
    }

    private void loop() {
      sim.loop();
      if (++loop % 2 == 0) {
        for (Joint joint : recorded) {
          rows.add(row(joint));
        }
        for (Told each : told.values()) {
          each.reset = Double.NaN;
        }
      }
    }

    private String row(Joint joint) {
      JointMechanism mechanism = sim.joints.of(joint);
      Told io = io(joint);
      return String.join(
          ",",
          name,
          "" + loop,
          joint.label(),
          io.request,
          number(io.values[0]),
          number(io.values[1]),
          number(io.values[2]),
          number(io.reset),
          number(sim.simulation.position(joint)),
          number(mechanism.position()),
          number(mechanism.goal()),
          changed(joint, "state", mechanism.stateShown()),
          changed(joint, "held by", mechanism.heldBy()),
          changed(joint, "outcome", mechanism.lastOutcome()),
          changed(joint, "doubts", String.join(" / ", mechanism.switchDoubts())));
    }

    /** A text, the row its value changes on, and blank after: so a row says what happened. */
    private String changed(Joint joint, String column, String now) {
      String key = joint.label() + "/" + column;
      String before = last.put(key, now);
      String text = now.equals(before) ? "" : now.isEmpty() ? "(none)" : now;
      return text.replace(',', ';');
    }
  }

  private static String number(double value) {
    return Double.isNaN(value) ? "" : String.format(Locale.ROOT, "%.7f", value);
  }

  private static final Sensor.Switch CLAW_BEAM = Sensor.Switch.beam("claw beam", PIVOT);

  private static final Sensor.Rangefinder REEF =
      Sensor.Rangefinder.builder()
          .label("reef range")
          .part(Part.CHASSIS)
          .range(Meters.of(1))
          .fieldOfView(Degrees.of(27))
          .build();

  /**
   * The roller runs only with the pivot out and its beam clear; the turret only with the roller up
   * to speed; the pivot keeps out of 60° to 90° while the reef is near; and the elevator out of its
   * lowest 0.3 m, a region reaching its bottom stop, while the pivot is in.
   */
  private static final RobotDefinition GUARDED =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .name("guarded")
          .sensors(List.of(CLAW_BEAM, REEF))
          .interlocks(
              List.of(
                  Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110))),
                  Interlock.that(ROLLER).runsOnlyWhen(CLAW_BEAM.clear()),
                  Interlock.that(TURRET).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(1000))),
                  Interlock.that(PIVOT)
                      .keepsOut(Range.degrees(60, 90))
                      .when(REEF.nearerThan(Meters.of(0.3))),
                  Interlock.that(ELEVATOR)
                      .keepsOut(Range.meters(0, 0.3))
                      .when(PIVOT.atMost(Degrees.of(20)))))
          .build();

  /**
   * Every interlock holds and lets go: the turret held until the roller's up to speed, the roller
   * until the pivot's out; the pivot overshooting into the band as the reef comes near, held there,
   * then held at its edge coming back, the reef lost meanwhile; the pivot's motor controllers going
   * silent under the roller's rule; the beam tripping; the elevator leaving its region by its top
   * edge from its bottom stop, then held above it; and a disable and enable while held.
   */
  @Test
  void interlocks() throws IOException {
    Scene scene = new Scene("interlocks", GUARDED, PIVOT, ROLLER, ELEVATOR, TURRET);
    JointMechanism pivot = scene.of(PIVOT);
    scene.schedule(scene.of(ROLLER).run(ROLLER_INTAKING));
    scene.schedule(scene.of(TURRET).goTo(TURRET_BACK).within(Seconds.of(8)));
    scene.schedule(pivot.goTo(PIVOT_OUT).atSpeed(Percent.of(30)).within(Seconds.of(8)));
    scene.schedule(scene.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    scene.run(0.2);
    scene.runUntil(() -> Math.toDegrees(scene.sim.simulation.position(PIVOT)) > 58, 3);
    scene.sim.sensors.see(REEF, Meters.of(0.2));
    scene.run(0.6);
    scene.sim.sensors.see(REEF, Meters.of(0.8));
    scene.run(2.4);
    scene.io(PIVOT).silent = true;
    scene.run(0.3);
    scene.io(PIVOT).silent = false;
    scene.run(0.4);
    scene.sim.sensors.trip(CLAW_BEAM, true);
    scene.run(0.4);
    scene.sim.sensors.trip(CLAW_BEAM, false);
    scene.schedule(scene.of(ELEVATOR).goTo(ELEVATOR_TOP));
    scene.run(1.2);
    scene.sim.sensors.see(REEF, Meters.of(0.2));
    scene.schedule(pivot.goTo(PIVOT_IN));
    scene.run(0.8);
    scene.sim.sensors.silence(REEF, true);
    scene.run(0.4);
    scene.sim.sensors.silence(REEF, false);
    scene.sim.sensors.see(REEF, Meters.of(0.8));
    scene.run(1.2);
    scene.schedule(scene.of(ELEVATOR).goTo(ELEVATOR_DOWN));
    scene.run(1);
    scene.sim.setEnabled(false);
    scene.run(0.3);
    scene.sim.setEnabled(true);
    scene.run(0.5);
    record("stages-interlocks.csv");
  }

  /** The turret's switch at 90°, on its way to the back; the pivot's at 120°, short of out. */
  private static final Sensor.Switch TURRET_RIGHT =
      JointFixtures.limit(
          "turret right", TURRET, Sensor.Switch.Limit.End.UPPER, Math.toRadians(90));

  private static final Sensor.Switch PIVOT_UP =
      JointFixtures.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER, Math.toRadians(120));

  /**
   * Moves past a tripped switch hold where it tripped and time out; a voltage toward it goes
   * neutral, and away from it is applied; leaving it is free. The pivot's switch's wire cut, it
   * reads tripped far from where it trips: doubted, and still stopping the pivot until it's mended.
   */
  @Test
  void limitSwitches() throws IOException {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
            .name("switched")
            .sensors(List.of(TURRET_RIGHT, PIVOT_UP))
            .build();
    Scene scene = new Scene("limit switches", robot, TURRET, PIVOT);
    JointMechanism turret = scene.of(TURRET);
    Move back = turret.goTo(TURRET_BACK);
    Move out = scene.of(PIVOT).goTo(PIVOT_OUT);
    scene.schedule(back);
    scene.schedule(out);
    scene.runUntil(() -> back.outcome().isPresent() && out.outcome().isPresent(), 5);
    scene.schedule(push(turret, 2));
    scene.run(0.6);
    scene.schedule(push(turret, -2));
    scene.run(0.4);
    scene.schedule(scene.of(PIVOT).goTo(PIVOT_IN));
    scene.schedule(turret.goTo(TURRET_FRONT));
    scene.run(1.4);
    scene.sim.sensors.cut(PIVOT_UP, SensorWiring.Polarity.NORMALLY_CLOSED);
    scene.run(0.4);
    scene.schedule(scene.of(PIVOT).goTo(PIVOT_RUN));
    scene.run(0.6);
    scene.sim.sensors.cut(PIVOT_UP, null);
    scene.run(1);
    record("stages-switches.csv");
  }

  /** A command of its own pushing the turret at a voltage each loop. */
  private static Command push(JointMechanism turret, double volts) {
    return turret
        .run(
            co -> {
              while (true) {
                turret.setVoltage(volts);
                co.yield();
              }
            })
        .named("Push at " + volts + " V");
  }

  /** The pivot's bottom switch, tripping at 5°: above where it rests, at 1.5°. */
  private static final Sensor.Switch PIVOT_DOWN =
      JointFixtures.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Math.toRadians(5));

  /**
   * The elevator homes into its bottom stop, the pivot to its switch, starting on it, so it backs
   * off first; their moves wait. Disabled before, neither homes. Then, with too little time for the
   * elevator and the pivot's switch lost, both are faulted, refuse their moves, and home again.
   */
  @Test
  void homing() throws IOException {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
            .name("homing")
            .sensors(List.of(PIVOT_DOWN))
            .build()
            .with(
                ELEVATOR.retuned(
                    control -> homed(control, Homing.toHardStop(Sensor.Switch.Limit.End.LOWER))),
                PIVOT.retuned(
                    control -> homed(control, Homing.toSwitch(Sensor.Switch.Limit.End.LOWER))));
    Scene scene = new Scene("homing", robot, ELEVATOR, PIVOT);
    scene.sim.simulation.powerOnAt(ELEVATOR, 0.3);
    scene.sim.setEnabled(false);
    scene.schedule(scene.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    scene.schedule(scene.of(PIVOT).goTo(PIVOT_RUN));
    scene.run(0.2);
    scene.sim.setEnabled(true);
    scene.run(0.1);
    scene.sim.setEnabled(false);
    scene.run(0.1);
    scene.sim.setEnabled(true);
    scene.run(2.6);

    RobotDefinition quick =
        robot.with(
            ELEVATOR.retuned(
                control ->
                    homed(
                        control,
                        Homing.toHardStop(Sensor.Switch.Limit.End.LOWER, Seconds.of(0.3)))));
    Scene again = new Scene("homing timed out", quick, ELEVATOR, PIVOT);
    again.sim.simulation.powerOnAt(ELEVATOR, 1.1);
    again.sim.simulation.powerOnAt(PIVOT, Math.toRadians(40));
    again.sim.sensors.silence(PIVOT_DOWN, true);
    again.run(1.8);
    again.schedule(again.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    again.schedule(again.of(PIVOT).goTo(PIVOT_RUN));
    again.run(0.2);
    again.sim.simulation.place(ELEVATOR, 0);
    again.sim.sensors.silence(PIVOT_DOWN, false);
    again.schedule(again.of(ELEVATOR).home());
    again.schedule(again.of(PIVOT).home());
    again.run(1.6);
    record("stages-homing.csv");
  }

  /** The pivot's through-bore on port 1, mounted reversed and turned a third of a turn. */
  private static final MechanismWiring.PortEncoder PORT_1 =
      MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.3)).inverted();

  /** The elevator's on port 2, geared nine to one down from its drum. */
  private static final MechanismWiring.PortEncoder PORT_2 =
      MechanismWiring.PortEncoder.dutyCycle(2).offset(Rotations.of(0.3));

  /**
   * The pivot's loop closed on its encoder, which gives no reading for 2 s, so it's faulted until
   * it does; then lost as the pivot moves, which holds on its motor's count, disabled and enabled,
   * and homed again while it's lost. The elevator's encoder only tells its motor where it is, read
   * at once, and again once homed again.
   */
  @Test
  void absoluteEncoders() throws IOException {
    RobotDefinition robot =
        JointFixtures.ROBOT.with(
            PIVOT
                .absoluteEncoder(PORT_1.closesLoop())
                .retuned(old -> control(30, 1, profile(8, 30))),
            ELEVATOR
                .absoluteEncoder(Driven.AbsoluteEncoder.of().ratio(1.0 / 9).on(PORT_2))
                .retuned(old -> control(60, 2, profile(1.2, 5))));
    Scene scene = new Scene("absolute encoders", robot, PIVOT, ELEVATOR);
    JointMechanism pivot = scene.of(PIVOT);
    scene.sim.simulation.powerOnAt(PIVOT, Math.toRadians(30));
    scene.sim.simulation.powerOnAt(ELEVATOR, 0.2);
    scene.sim.simulation.unplug(PIVOT, true);
    scene.run(2.2);
    scene.sim.simulation.unplug(PIVOT, false);
    scene.run(0.2);
    scene.schedule(pivot.goTo(PIVOT_OUT));
    scene.schedule(scene.of(ELEVATOR).goTo(ELEVATOR_CLEAR));
    scene.run(0.3);
    scene.sim.simulation.unplug(PIVOT, true);
    scene.run(0.6);
    scene.sim.setEnabled(false);
    scene.run(0.3);
    scene.sim.setEnabled(true);
    scene.run(0.3);
    scene.schedule(pivot.home());
    scene.schedule(scene.of(ELEVATOR).home());
    scene.run(0.6);
    scene.sim.simulation.unplug(PIVOT, false);
    scene.run(0.3);
    scene.schedule(pivot.goTo(PIVOT_RUN));
    scene.run(1.2);

    Scene unset = new Scene("settings not taken", robot, ELEVATOR);
    unset.io(ELEVATOR).unconfigured = "motor 20";
    unset.sim.simulation.powerOnAt(ELEVATOR, 0.2);
    unset.run(0.2);
    unset.schedule(unset.of(ELEVATOR).home());
    unset.run(0.2);
    record("stages-encoders.csv");
  }

  private void record(String name) throws IOException {
    GoldenFile.check(getClass(), name, HEADER, rows);
  }
}

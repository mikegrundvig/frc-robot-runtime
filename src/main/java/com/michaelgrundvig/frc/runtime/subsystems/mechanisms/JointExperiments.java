package com.michaelgrundvig.frc.runtime.subsystems.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Experiments;
import com.michaelgrundvig.frc.pitwall.experiment.Profiles;
import com.michaelgrundvig.frc.pitwall.experiment.Rig;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.wpilib.units.Unit;

/**
 * An experiment for every driven joint, made from its definition, so tuning a joint's gains on
 * Pitwall's page needs no code: each step goes between places and speeds the definition already
 * names, the moves its commands make anyway.
 *
 * <ul>
 *   <li>A joint held at positions steps between each two of its places, in the order its block
 *       names them, and back: the pivot from "in" to "out", the elevator from "down" to "clear".
 *   <li>A joint run on its velocity loop steps from still to each of its speeds and back: the
 *       roller up to "intaking". A voltage or a duty cycle runs no loop, so there's nothing to tune
 *       by a step.
 * </ul>
 *
 * <p>A joint with fewer than two places, or no speed, gets none: name a place or a speed in its
 * block to give it one (a hood's "lob", a shooter's "shooting"). Each step is held twice as long as
 * the joint's profile takes to make the move, so it's seen arriving and settling: never less than a
 * profile's lead-in, and never so long the run passes the longest Pitwall takes.
 *
 * <p>A run commands the joint as {@link JointMechanism#follow} does, loop by loop, so the joint's
 * interlocks, limits, and range hold beneath it as they do beneath every command. An interlock that
 * holds the joint holds the run too: the run then never gets there, and its scores say so.
 */
public final class JointExperiments {
  private JointExperiments() {}

  /**
   * Every joint's experiments, by the joint's order in the definition.
   *
   * @param tunings each driven joint's tuning, by its label, as the robot program publishes them
   */
  public static List<Experiment> of(
      Joints joints, Map<String, LiveConfig<MechanismControl>> tunings) {
    List<Experiment> made = new ArrayList<>();
    for (JointMechanism mechanism : joints.all()) {
      LiveConfig<MechanismControl> tuning = tunings.get(mechanism.joint().label());
      if (tuning != null) {
        made.addAll(of(mechanism, tuning));
      }
    }
    return List.copyOf(made);
  }

  /** One joint's experiments: none for a joint with nothing named to step between. */
  static List<Experiment> of(JointMechanism mechanism, LiveConfig<MechanismControl> tuning) {
    Joint joint = mechanism.joint();
    boolean turns = joint instanceof Joint.Revolute;
    String label = joint.label();
    List<Experiment> made = new ArrayList<>();
    if (tuning.get().loop() == MechanismControl.Loop.POSITION) {
      List<Position> places = joint.positions();
      if (places.size() < 2) {
        return List.of();
      }
      Unit unit = turns ? Degrees : Meters;
      double lowest = places.stream().mapToDouble(Position::value).min().orElseThrow();
      double highest = places.stream().mapToDouble(Position::value).max().orElseThrow();
      Rig rig =
          rig(
              mechanism,
              tuning,
              unit,
              joint.range().map(Range::lower).orElse(lowest),
              joint.range().map(Range::upper).orElse(highest),
              true);
      for (int i = 0; i + 1 < places.size(); i++) {
        Position from = places.get(i);
        Position to = places.get(i + 1);
        double hold = 2 * mechanism.secondsBetween(from.value(), to.value());
        made.add(
            step(
                label + ": " + from.name() + " to " + to.name(),
                rig,
                unit,
                from.value(),
                to.value(),
                hold));
      }
    } else {
      List<Output> speeds =
          joint.outputs().stream()
              .filter(output -> output.kind() == Output.Kind.VELOCITY && output.value() != 0)
              .toList();
      if (speeds.isEmpty()) {
        return List.of();
      }
      Unit unit = turns ? RPM : MetersPerSecond;
      double slowest = Math.min(0, speeds.stream().mapToDouble(Output::value).min().orElseThrow());
      double fastest = Math.max(0, speeds.stream().mapToDouble(Output::value).max().orElseThrow());
      Rig rig = rig(mechanism, tuning, unit, slowest, fastest, false);
      for (Output speed : speeds) {
        double hold = 2 * mechanism.secondsToSpeed();
        made.add(step(label + ": up to " + speed.name(), rig, unit, 0, speed.value(), hold));
      }
    }
    return made;
  }

  /**
   * A step and back, its lead-in as long as a hold, so the joint is settled where it starts. Its
   * three holds fit the longest run Pitwall takes, which it refuses as the robot starts otherwise.
   */
  private static Experiment step(
      String name, Rig rig, Unit unit, double from, double to, double holdSeconds) {
    // Up to a tenth of a second, so the page says "held 3.3 s".
    double hold =
        Math.min(
            Math.ceil(Math.max(holdSeconds, Profiles.LEAD_IN.in(Seconds)) * 10) / 10,
            Math.floor(Experiments.LONGEST_SECONDS / 3 * 10) / 10);
    return new Experiment(
        name,
        rig,
        Profiles.stepAndBack()
            .from(unit.ofBaseUnits(from))
            .to(unit.ofBaseUnits(to))
            .hold(Seconds.of(hold))
            .leadIn(Seconds.of(hold)));
  }

  /**
   * The joint as a rig: its gains, a setpoint chased as {@link JointMechanism#follow} chases one (a
   * position profiled toward, from where the joint is as each step begins, or a speed), and its
   * position or speed, voltage, and current from its logged inputs, so a replay records the same.
   *
   * @param lowest the least it may be sent, in its own units: radians, meters, or per second
   * @param highest the most
   */
  private static Rig rig(
      JointMechanism mechanism,
      LiveConfig<MechanismControl> tuning,
      Unit unit,
      double lowest,
      double highest,
      boolean positional) {
    double[] asked = {Double.NaN};
    return Rig.builder()
        .name(mechanism.joint().label())
        .gains(tuning, "gains.kP", "gains.kD", "gains.kS")
        .setpoint(
            unit,
            setpoint -> {
              double target = unit.toBaseUnits(setpoint);
              if (positional) {
                // Each step a move of its own: its profile starts from where the joint is.
                if (target != asked[0]) {
                  mechanism.beginMove();
                }
                mechanism.askPosition(mechanism.reachable(target), 1, null);
              } else {
                mechanism.askVelocity(target);
              }
              asked[0] = target;
            })
        .limits(unit.ofBaseUnits(lowest), unit.ofBaseUnits(highest))
        .measurement(
            positional ? "Position" : "Speed",
            unit,
            () -> unit.fromBaseUnits(positional ? mechanism.position() : mechanism.velocity()))
        .signal("Voltage", Volts, mechanism::appliedVolts)
        .signal("Current", Amps, mechanism::statorCurrentAmps)
        .requires(mechanism)
        .build();
  }
}

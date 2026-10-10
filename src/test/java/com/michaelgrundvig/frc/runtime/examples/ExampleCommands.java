package com.michaelgrundvig.frc.runtime.examples;

import static com.michaelgrundvig.frc.runtime.ExampleRobot.CLAW_BEAM;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.FEEDING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.HOPPER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INDEXER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.INTAKING;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_IN;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.PIVOT_OUT;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.ROLLER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.SHOOTER;
import static com.michaelgrundvig.frc.runtime.ExampleRobot.STOWED;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RPM;

import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Move;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.filter.Debouncer;

/**
 * What the robot of docs/mechanisms.md does, as the guide writes it: its joints looked up by their
 * constants, a move that recovers, a law of its own, gates of its own and its sensors', and a shot
 * that feeds whenever it's ready. The guide's blocks, and the mechanisms' Javadoc, are this code
 * (DocExamplesTest).
 */
final class ExampleCommands {
  private final Joints joints;
  private final JointMechanism pivot;
  private final JointMechanism roller;
  private final JointMechanism shooter;
  private final JointMechanism feeder;
  private final JointMechanism indexer;
  private final DoubleSupplier headingError;
  private final DoubleSupplier distance;
  private final Gate readyToShoot;
  private final Gate intakeClear;
  private final Gate coralIn;
  private final Gate hoppers;
  private int pieces;

  /**
   * The robot's commands.
   *
   * @param headingError how far the robot faces from its target, in degrees
   * @param distance how far the target is, in meters
   */
  ExampleCommands(
      Scheduler scheduler, Joints joints, DoubleSupplier headingError, DoubleSupplier distance) {
    this.joints = joints;
    this.headingError = headingError;
    this.distance = distance;
    // @start region=look-up
    JointMechanism pivot = joints.of(PIVOT);
    JointMechanism roller = joints.of(ROLLER);
    // @end
    this.pivot = pivot;
    this.roller = roller;
    shooter = joints.of(SHOOTER);
    feeder = joints.of(FEEDER);
    indexer = joints.of(INDEXER);

    // @start region=gates
    Gate aimed =
        Gate.within(
            scheduler, "aimed", () -> headingErrorDegrees(), 2.0, 4.0); // in at 2°, out past 4°
    Gate readyToShoot =
        Gate.all("ready to shoot", shooter.isAtGoal(), aimed)
            .debounce(
                Milliseconds.of(80), Debouncer.DebounceType.FALLING); // rides out a flywheel's dip
    // @end
    this.readyToShoot = readyToShoot;

    // @start region=sensor-gates
    Gate coralIn = joints.gate("coral in claw", CLAW_BEAM.tripped());
    Gate hopperFull = joints.gate("hopper full", HOPPER.nearerThan(Inches.of(6)));
    // @end
    this.coralIn = coralIn;
    intakeClear = joints.gate("intake clear", CLAW_BEAM.clear());
    hoppers = hopperFull;
  }

  /** Deploys the intake, and backs it off if it doesn't get there. */
  Command deployOrBackOff() {
    // @start region=recovery
    return Command.requiring(pivot)
        .executing(
            co -> {
              var deploy = pivot.goTo(PIVOT_OUT);
              co.await(deploy);
              if (!deploy.arrived()) {
                co.await(pivot.goTo(PIVOT_IN)); // back off
              }
            })
        .named("Deploy or back off");
    // @end
  }

  /** A law of its own: the roller pushed at a voltage it works out every loop. */
  Command push() {
    return roller
        .run(
            co -> {
              while (true) {
                roller.setVoltage(pushVolts());
                co.yield();
              }
            })
        .named("Push");
  }

  // @start region=intake-one
  /** Intakes until the claw's beam trips, the loop it trips; then counts the piece. */
  Command intakeOne() {
    Move intake =
        roller.runUntil(INTAKING, coralIn); // ends ARRIVED the loop it trips; TIMED_OUT after 3 s
    return Command.requiring(roller)
        .executing(
            co -> {
              co.await(intake);
              co.waitUntil(coralIn); // holds a step
            })
        .named("Intake one");
  }

  // @end

  /** Counts each piece, once each time the beam trips, past a beam that flickers. */
  void countPieces() {
    coralIn.onTrue(countPiece()); // once each time it trips
    coralIn.debounce(Milliseconds.of(40), Debouncer.DebounceType.RISING).onTrue(countPiece());
  }

  /** Deploys the intake only if nothing's in the way, as it starts. */
  Command deploy() {
    return pivot.goTo(PIVOT_OUT).onlyIf(intakeClear); // checked once: REFUSED if false
  }

  // @start region=configurations
  /** Every joint of a configuration to its place, requiring exactly those joints. */
  Command stow() {
    return joints.goTo(STOWED);
  }

  /** Whether every joint of a configuration is at its place. */
  Gate stowed() {
    return joints.isAt(STOWED);
  }

  // @end

  /** Spins up, and feeds whenever it's ready, pausing when it's not. */
  Command shoot() {
    return Command.requiring(shooter, feeder, indexer)
        .executing(
            co -> {
              co.fork(shooter.follow(() -> RPM.of(speedFor(distance()))));
              while (true) {
                co.waitUntil(readyToShoot);
                co.await(joints.run(FEEDING).onlyWhile(readyToShoot));
              }
            })
        .named("Shoot");
  }

  /** Whether the hopper is full. */
  Gate hopperFull() {
    return hoppers;
  }

  /** How many pieces the claw has taken in. */
  int pieces() {
    return pieces;
  }

  private Command countPiece() {
    return Command.noRequirements(co -> pieces++).named("Count a piece");
  }

  private double headingErrorDegrees() {
    return headingError.getAsDouble();
  }

  private double distance() {
    return distance.getAsDouble();
  }

  /** A flywheel's speed for a distance, in rpm: a line through two measured shots. */
  private static double speedFor(double meters) {
    return 2500 + 400 * meters;
  }

  /** The roller's push, harder as it stalls. */
  private double pushVolts() {
    return roller.velocity() < 1 ? 4 : 2;
  }
}

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.michaelgrundvig.frc.runtime.sim;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.subsystems.drive.ModuleIO;
import com.michaelgrundvig.frc.runtime.subsystems.drive.PhoenixDrivetrain;
import com.michaelgrundvig.frc.simulation.CurrentLimitedMotor;
import com.michaelgrundvig.frc.simulation.Physics;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.simulation.WheelDrive;
import com.michaelgrundvig.frc.simulation.WheelTruth;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Motor;
import java.util.Arrays;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/**
 * A simulated swerve module's motor controllers and encoders: the actuators that drive one wheel of
 * the {@link Physics}, and the sensors that read it.
 *
 * <p><b>Each physics sub-step</b> ({@link #actuate}):
 *
 * <ul>
 *   <li><b>The drive's controller</b> works out the voltage it asks for, from its velocity loop at
 *       the wheel's true speed, and hands it to the physics with its stator current limit. The
 *       physics holds the motor to that limit, turns the wheel, and moves the robot.
 *   <li><b>The steering's controller</b> runs its position loop and turns the module here, in
 *       closed form ({@link CurrentLimitedMotor}), and the physics is told the angle.
 * </ul>
 *
 * <p><b>The steering runs as a TalonFX without a Phoenix Pro license runs it:</b> its loop every
 * millisecond, on the angle its CANcoder last sent over CAN, which is up to a CANcoder's period
 * old. That wait, with the current limit, sets how high its gains can go before it overshoots, as
 * on a robot.
 *
 * <p><b>What can be wrong with it.</b> Its absolute encoder's zero can be off ({@link
 * SensorErrors}), and knocked further off while it runs. Its encoders' readings can go stale: the
 * wheel turns on, and what the robot's code reads stops changing, as signals that stop arriving do.
 * Its wheel can slip, or keep spinning faster than the floor moves it.
 *
 * <p><b>What is left out.</b> The battery holds 12 V. The drive's supply current limit, with
 * Phoenix's step-down from it ({@code DrivetrainControl.Limits.driveSupplyLowerCurrent}), is not
 * modeled: the supply current reported is what the motor would draw from a steady 12 V with no
 * supply limit. So this tests the code and not the robot's power, and docs/stand-ins.md says when
 * that matters. Both motors' stator current limits are modeled: the drive's caps the torque that
 * accelerates the robot, and the steering's caps how hard the module turns and stops. The motors
 * and their gearing are the robot's definition's ({@code DrivetrainGeometry.ModuleType}).
 */
final class ModuleIOSim implements ModuleIO {
  private static final double NANOS_PER_SECOND = 1e9;

  /**
   * A TalonFX runs its closed loop every millisecond (CTRE's Phoenix 6 documentation: 1 kHz), in
   * nanoseconds. The module keeps its own time in whole nanoseconds, so the loop's rate, and the
   * CANcoder's, hold whatever length the physics' sub-steps are.
   */
  private static final long STEER_LOOP_NANOS = 1_000_000;

  /**
   * How often the module's CANcoder sends its angle to the steering motor, in nanoseconds: at the
   * rate the robot's code sets ({@link PhoenixDrivetrain#ENCODER_FREQUENCY_HZ}), every 10 ms at 100
   * Hz. Without a Phoenix Pro license the steering closes its loop on the CANcoder as a remote
   * sensor, and takes a new angle and speed only when the CANcoder sends them over CAN. In between,
   * its loop runs on what it last heard, up to a period old, and that wait, more than the loop's
   * rate or the current limit, is what makes the steering overshoot at high gains. The fraction of
   * a millisecond a frame takes on the bus is left out. With Pro's FusedCANcoder, the loop would
   * read the motor's own sensor every loop, and this would be {@link #STEER_LOOP_NANOS}.
   */
  private static final long ENCODER_PERIOD_NANOS =
      Math.round(NANOS_PER_SECOND / PhoenixDrivetrain.ENCODER_FREQUENCY_HZ);

  // The CANcoder sends on the steering loop's millisecond, and the same number of times every robot
  // loop, so each robot loop finds every CANcoder at the same point in its cycle.
  static {
    long robotLoopNanos = Math.round(RobotLoop.PERIOD_SECONDS * NANOS_PER_SECOND);
    boolean onTheSteerLoopsMillisecond = ENCODER_PERIOD_NANOS % STEER_LOOP_NANOS == 0;
    boolean dividesTheRobotsLoop = robotLoopNanos % ENCODER_PERIOD_NANOS == 0;
    if (!onTheSteerLoopsMillisecond || !dividesTheRobotsLoop) {
      throw new IllegalStateException(
          "the CANcoder's period, "
              + ENCODER_PERIOD_NANOS
              + " ns, must be a whole number of milliseconds and divide the robot's loop");
    }
  }

  /** What the robot last asked of the drive's motor controller. */
  private enum DriveAsk {
    /** A wheel speed, on its velocity loop, with a feedforward voltage. */
    VELOCITY,
    /** A voltage, held: open loop. Stopped is this, at 0 V. */
    VOLTAGE
  }

  /**
   * What the module's encoders read at the moment their readings went stale: what the robot's code
   * goes on reading until they catch up.
   */
  private record StaleReadings(
      double drivePositionRadians,
      double driveVelocityRadPerSec,
      double steerRadians,
      double steerRadPerSec) {}

  private final WheelTruth wheel;
  private final CurrentLimitedMotor steerMotor;

  /** The wheel's radius as configured, which the motor controllers convert with. */
  private final double wheelRadiusMeters;

  /** How far the absolute encoder's zero is off: it reads the true angle minus this. */
  private double steerOffsetRadians;

  /** How far it was off as the robot was built. */
  private final double builtSteerOffsetRadians;

  /** The readings the robot's code is stuck with while the encoders are stale. Empty while live. */
  private Optional<StaleReadings> stale = Optional.empty();

  /**
   * Rotation the wheel spun without rolling the robot along: its encoder saw it, the floor didn't.
   */
  private double slippedRadians;

  /** How fast it keeps spinning beyond what rolls the robot along, in radians per second. */
  private double slippingRadPerSec;

  // Readings taken since the last loop, for odometry: the first `samples` entries of each array.
  private long[] sampleTimes = new long[8];
  private double[] samplePositions = new double[8];
  private double[] sampleAngles = new double[8];
  private int samples;

  private double driveKp;
  private double steerKp;
  private double steerKd;

  private DriveAsk driveAsk = DriveAsk.VOLTAGE;
  private double driveSetpointRadPerSec;
  private double driveFeedforwardVolts;

  /** The voltage the drive's controller asks for: a voltage held, or its velocity loop's output. */
  private double driveVolts;

  private double driveStatorLimitAmps = Double.POSITIVE_INFINITY;

  /** Whether the drive's current limit held its motor back since the last loop read it. */
  private boolean driveLimited;

  /** Whether the physics has run a sub-step on this module's drive that hasn't been looked at. */
  private boolean subStepUnread;

  /** Whether the steering's position loop runs. Otherwise its motor is at 0 V. */
  private boolean steerLoopOn;

  private double steerSetpointRad;
  private double steerStatorLimitAmps = Double.POSITIVE_INFINITY;

  /** The angle and speed the steering's motor controller last heard from the CANcoder. */
  private double heardSteerRadians;

  private double heardSteerRadPerSec;

  /** How long until the CANcoder next sends, in nanoseconds: none, when it sends now. */
  private long untilEncoderSendsNanos;

  /** How long until the steering's loop next runs, in nanoseconds: none, when it runs now. */
  private long untilSteerLoopNanos;

  /** The voltage the steering's loop last asked for, which its motor holds until it runs again. */
  private double steerVolts;

  /** The most current through the steering motor since the last loop read it. */
  private double steerPeakAmps;

  /**
   * @param modules the modules as defined: their motors, gearing, and moving parts. The motor
   *     controllers convert with this wheel radius. The steering's inertia sets how fast it turns
   *     at its current limit, and so how high its gains can go (docs/stand-ins.md).
   * @param wheel the wheel it drives, as the physics has it: its encoders read it
   * @param steerOffsetRadians how far the absolute encoder's zero is off
   * @param encoderPhase when the CANcoder first sends, as a share of its period: each module's
   *     CANcoder sends on its own clock
   */
  ModuleIOSim(
      DrivetrainGeometry.ModuleType modules,
      WheelTruth wheel,
      double steerOffsetRadians,
      double encoderPhase) {
    this.wheel = wheel;
    wheelRadiusMeters = modules.wheelRadius().in(Meters);
    steerMotor =
        new CurrentLimitedMotor(
            modules.steerMotors().model(modules.steerGearing().efficiency()),
            modules.steerInertiaOrDefault().in(KilogramSquareMeters),
            modules.steerGearing().ratio());
    this.steerOffsetRadians = steerOffsetRadians;
    builtSteerOffsetRadians = steerOffsetRadians;
    heardSteerRadians = measuredSteerRadians();
    // The CANcoder's first send, on a whole millisecond of the steering's loop.
    long steerLoopsPerEncoderPeriod = ENCODER_PERIOD_NANOS / STEER_LOOP_NANOS;
    long steerLoopsUntilFirstSend = (long) (encoderPhase * steerLoopsPerEncoderPeriod);
    untilEncoderSendsNanos = steerLoopsUntilFirstSend * STEER_LOOP_NANOS;
  }

  /**
   * Its absolute encoder's zero off by this much from now on: the steering's loop, which runs on
   * the encoder, turns the wheel to match once the CANcoder next sends.
   */
  void setSteerOffset(double radians) {
    steerOffsetRadians = radians;
  }

  double steerOffsetRadians() {
    return steerOffsetRadians;
  }

  double builtSteerOffsetRadians() {
    return builtSteerOffsetRadians;
  }

  /**
   * The encoders' readings stop changing while the wheel turns on, or catch up again. The motor
   * controllers' own loops keep their live readings: only what the robot's code reads is stale.
   */
  void setStale(boolean goesStale) {
    if (!goesStale) {
      stale = Optional.empty();
      return;
    }
    if (stale.isPresent()) {
      return; // already stale: it stays stuck at what it read when it first went stale
    }
    stale =
        Optional.of(
            new StaleReadings(
                drivePositionRadians(),
                wheel.spinRadPerSec(),
                heardSteerRadians,
                heardSteerRadPerSec));
  }

  boolean isStale() {
    return stale.isPresent();
  }

  /**
   * Runs the motor controllers through a physics sub-step: the drive's loop once, at the wheel's
   * speed as the sub-step starts, and the steering's every millisecond, turning the module. The
   * drive's motor is about sixty times slower than the steering's, and reads its own sensor, so its
   * loop running once a sub-step changes nothing that matters. Its stator current limit goes to the
   * physics with what the loop asks for: the physics holds the motor to it through the sub-step, as
   * the motor controller would ({@link WheelDrive}).
   *
   * @param drive what this wheel's drive and steering do over the sub-step, for the physics
   */
  void actuate(double seconds, WheelDrive drive) {
    noteWhetherTheDriveWasLimited();
    if (driveAsk == DriveAsk.VELOCITY) {
      // The velocity loop: the feedforward, plus kP on the error in the wheel's rim speed. The
      // gain is in volts per meter a second, so the error in rad/s is put through the radius.
      double errorRadPerSec = driveSetpointRadPerSec - wheel.spinRadPerSec();
      driveVolts = driveFeedforwardVolts + driveKp * errorRadPerSec * wheelRadiusMeters;
    }
    driveVolts = Math.clamp(driveVolts, -Motor.NOMINAL_VOLTS, Motor.NOMINAL_VOLTS);
    drive.drive(driveVolts, driveStatorLimitAmps, Motor.NOMINAL_VOLTS);
    subStepUnread = true;

    slippedRadians += slippingRadPerSec * seconds;
    steer(Math.round(seconds * NANOS_PER_SECOND));
    drive.steer(trueSteerAngle().getRadians());
  }

  /**
   * Notes whether the drive's current limit held in the physics' last sub-step, once: the flag
   * holds until a loop reads it, as a motor controller's does.
   */
  private void noteWhetherTheDriveWasLimited() {
    if (subStepUnread) {
      driveLimited |= wheel.driveCurrentLimited();
      subStepUnread = false;
    }
  }

  /**
   * Turns the steering through this long, on its own clocks: the CANcoder sends every period, the
   * steering's loop runs every millisecond on what it last heard, and between them its motor holds
   * the loop's voltage, turned in closed form. Each runs when its time comes, however the time is
   * cut into sub-steps.
   *
   * <p>The physics' scrub on the steering ({@link WheelTruth#steerLoadNewtonMeters}) isn't felt
   * yet: today's physics models none, and a real module's needs measuring first, as a torque
   * against the motion that can stop the module, which {@link CurrentLimitedMotor} says how to add.
   */
  private void steer(long nanos) {
    long remainingNanos = nanos;
    while (remainingNanos > 0) {
      if (untilEncoderSendsNanos == 0) {
        cancoderSends();
      }
      if (untilSteerLoopNanos == 0) {
        steerLoopRuns();
      }
      // The motor holds the loop's voltage until the next thing happens: the loop runs again, the
      // CANcoder sends, or the time is up.
      long untilNextEvent = Math.min(untilSteerLoopNanos, untilEncoderSendsNanos);
      long runNanos = Math.min(remainingNanos, untilNextEvent);
      double peakAmps =
          steerMotor.run(steerVolts, steerStatorLimitAmps, runNanos / NANOS_PER_SECOND);
      steerPeakAmps = Math.max(steerPeakAmps, peakAmps);
      remainingNanos -= runNanos;
      untilSteerLoopNanos -= runNanos;
      untilEncoderSendsNanos -= runNanos;
    }
  }

  /** The CANcoder sends the steering's motor controller its angle and speed, as it reads them. */
  private void cancoderSends() {
    heardSteerRadians = measuredSteerRadians();
    heardSteerRadPerSec = steerMotor.velocityRadPerSec();
    untilEncoderSendsNanos = ENCODER_PERIOD_NANOS;
  }

  /**
   * The steering's position loop runs once: kP on the angle still to turn, the short way round,
   * less kD on the speed, within the battery's voltage. It runs on what the CANcoder last sent,
   * whether or not that is still right.
   */
  private void steerLoopRuns() {
    untilSteerLoopNanos = STEER_LOOP_NANOS;
    if (!steerLoopOn) {
      steerVolts = 0.0;
      return;
    }
    double errorRadians = MathUtil.angleModulus(steerSetpointRad - heardSteerRadians);
    steerVolts =
        Math.clamp(
            steerKp * errorRadians - steerKd * heardSteerRadPerSec,
            -Motor.NOMINAL_VOLTS,
            Motor.NOMINAL_VOLTS);
  }

  /**
   * Takes a reading for odometry, as a motor controller's high-rate signals would. The steering's
   * angle is the one its motor controller last heard from the CANcoder, as on a robot without
   * Phoenix Pro: up to a CANcoder's period old.
   */
  void sample(long timestampNanos) {
    if (samples == sampleTimes.length) {
      sampleTimes = Arrays.copyOf(sampleTimes, samples * 2);
      samplePositions = Arrays.copyOf(samplePositions, samples * 2);
      sampleAngles = Arrays.copyOf(sampleAngles, samples * 2);
    }
    sampleTimes[samples] = timestampNanos;
    samplePositions[samples] = drivePositionRadians();
    sampleAngles[samples] = steerReadingRadians();
    samples++;
  }

  /** Spins the wheel without the robot moving: a slip its encoder sees and the floor doesn't. */
  void spin(double radians) {
    slippedRadians += radians;
  }

  /**
   * From now on the wheel spins this much faster than the floor moves it, in meters per second at
   * its configured radius, as one losing its grip does: its encoder counts it, the floor doesn't.
   */
  void spinAt(double metersPerSecond) {
    slippingRadPerSec = metersPerSecond / wheelRadiusMeters;
  }

  /** How much faster than the floor moves it the wheel spins, in meters per second. */
  double spinMetersPerSecond() {
    return slippingRadPerSec * wheelRadiusMeters;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.odometryTimestampsNanos = Arrays.copyOf(sampleTimes, samples);
    inputs.odometryDrivePositionsRad = Arrays.copyOf(samplePositions, samples);
    inputs.odometrySteerAnglesRad = Arrays.copyOf(sampleAngles, samples);
    samples = 0;

    inputs.driveConnected = true;
    inputs.drivePositionRad = drivePositionRadians();
    inputs.driveVelocityRadPerSec = driveVelocityRadPerSec();
    inputs.driveAppliedVolts = wheel.driveVolts();
    inputs.driveStatorCurrentAmps = Math.abs(wheel.driveCurrentAmps());
    inputs.driveSupplyCurrentAmps = driveSupplyAmps();
    noteWhetherTheDriveWasLimited();
    inputs.driveTorqueLimited = driveLimited;
    driveLimited = false;

    inputs.steerConnected = true;
    // The steering's angle and speed as the robot hears them: what the CANcoder last sent.
    inputs.steerAngle = new Rotation2d(steerReadingRadians());
    inputs.steerVelocityRadPerSec = steerVelocityReadingRadPerSec();
    inputs.steerAppliedVolts = steerMotor.appliedVolts();
    // The steering's current is the most it drew since the last loop, not what it draws now. It
    // runs at its limit for a few milliseconds as the module starts and stops a turn, then falls to
    // next to nothing, so a reading at the loop's end would almost never see the peak, and the peak
    // is what matters: how close the motor came to its limit. The drive's current changes over many
    // loops (at its limit for as long as the wheel takes to get up to speed), so a reading now
    // shows it, and driveTorqueLimited says whether it reached the limit since the last loop.
    inputs.steerStatorCurrentAmps = steerPeakAmps;
    steerPeakAmps = 0;
  }

  /**
   * The current the drive motor draws from the battery, in amps: what its PDH channel reads. Power
   * in is power out: the battery's 12 V times the supply current is the voltage across the motor
   * times the current through it.
   */
  double driveSupplyAmps() {
    return Math.abs(wheel.driveVolts() * wheel.driveCurrentAmps()) / Motor.NOMINAL_VOLTS;
  }

  /**
   * The current the steering motor draws from the battery now, in amps, worked out the same way.
   */
  double steerSupplyAmps() {
    return Math.abs(steerMotor.appliedVolts() * steerMotor.currentAmps()) / Motor.NOMINAL_VOLTS;
  }

  /** What the drive encoder reads: every turn of the wheel, slips too, unless it's stale. */
  private double drivePositionRadians() {
    if (stale.isPresent()) {
      return stale.get().drivePositionRadians();
    }
    return wheel.rotationRadians() + slippedRadians;
  }

  /** How fast the drive encoder reads the wheel turning, its slipping too, unless it's stale. */
  private double driveVelocityRadPerSec() {
    if (stale.isPresent()) {
      return stale.get().driveVelocityRadPerSec();
    }
    return wheel.spinRadPerSec() + slippingRadPerSec;
  }

  /**
   * The steering's angle as the robot's code reads it: what the CANcoder last sent, unless stale.
   */
  private double steerReadingRadians() {
    if (stale.isPresent()) {
      return stale.get().steerRadians();
    }
    return heardSteerRadians;
  }

  private double steerVelocityReadingRadPerSec() {
    if (stale.isPresent()) {
      return stale.get().steerRadPerSec();
    }
    return heardSteerRadPerSec;
  }

  /**
   * Which way the wheel really points, whatever its encoder reads. As an angle, it is within half a
   * turn of straight ahead, however many turns the steering has made.
   */
  Rotation2d trueSteerAngle() {
    return new Rotation2d(steerMotor.positionRadians());
  }

  /** What the absolute encoder reads: the true angle, less how far its zero is off. */
  private double measuredSteerRadians() {
    return steerMotor.positionRadians() - steerOffsetRadians;
  }

  /**
   * Takes the gains, and both motors' stator current limits. The drive's supply limit isn't
   * modeled.
   */
  @Override
  public void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    driveKp = gains.driveKp();
    steerKp = gains.steerKp();
    steerKd = gains.steerKd();
    driveStatorLimitAmps = limits.driveStatorCurrent().in(Amps);
    steerStatorLimitAmps = limits.steerStatorCurrent().in(Amps);
  }

  @Override
  public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
    driveAsk = DriveAsk.VELOCITY;
    driveSetpointRadPerSec = wheelRadPerSec;
    driveFeedforwardVolts = feedforwardVolts;
  }

  /** Drives the wheel open loop at a voltage, for a test of the simulated drivetrain. */
  public void setDriveVoltage(double volts) {
    driveAsk = DriveAsk.VOLTAGE;
    driveVolts = volts;
  }

  @Override
  public void setSteerAngle(Rotation2d angle) {
    steerLoopOn = true;
    steerSetpointRad = angle.getRadians();
  }

  @Override
  public void stop() {
    driveAsk = DriveAsk.VOLTAGE;
    driveVolts = 0.0;
    steerLoopOn = false;
  }
}

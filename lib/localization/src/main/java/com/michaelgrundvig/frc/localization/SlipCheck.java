package com.michaelgrundvig.frc.localization;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import java.util.ArrayDeque;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Compares the wheels with the IMU, to catch the wheels sliding over the floor together, which
 * their disagreement with each other (the kinematic residual) can't show, and to tell a collision
 * from hard driving:
 *
 * <ul>
 *   <li><b>The turn.</b> Over a sliding window, the gyro and the wheels should turn the robot
 *       alike. Beyond the gyro's noise and a share of the turn (worn tread turns the wheels'
 *       reckoning a little more or less than the robot), the difference is how far the wheels slid
 *       around the robot's center: at their reach, over the window, how fast. A tank drive's two
 *       sides always agree with each other, so on one this is the only check.
 *   <li><b>The speed.</b> The wheels' velocity, carried to where the IMU sits (an IMU off the
 *       center is swung around it as the robot turns), should change as the accelerometer feels it
 *       change. What it doesn't feel, added up loop by loop, is how much faster or slower than the
 *       robot the wheels roll: wheels spinning in a launch, or a push they don't see. It's let go
 *       no faster than the most the accelerometer's bias could add, so a bias can't build up, while
 *       a slide that holds at a steady speed stays seen; when it stops (the robot catches up with
 *       its wheels, or the push lets go), the accelerometer feels that too, and it's gone. Once
 *       past the accelerometer's noise, all of it is how fast the wheels slide: the tolerance tells
 *       a slide from the noise, it isn't a share of every slide. Whenever the wheels and the gyro
 *       say the robot is standing still, a reading the accelerometer holds steady for seconds is
 *       its bias, whatever its size (a push can't keep the robot speeding up that long), which is
 *       taken off what it reads after; and no slide is counted faster than the robot could go. An
 *       accelerometer not yet known to read along the robot's axes ({@link
 *       ImuMount#accelerometerChecked}) is compared by size, not direction: the wheels' change of
 *       velocity and what it felt, each added up on its own, add up alike whichever way it's
 *       turned, and what one holds past the other is a slide. It's compared only once its bias is
 *       learned: until then, gravity leaking into one tilted on its mount would add up as a slide.
 *       A bias past what a part has is that gravity, and {@link #leaksGravity} says so.
 *   <li><b>A jolt.</b> What the accelerometer feels that the wheels' own change of velocity doesn't
 *       explain (a robot stopping hard is felt by both, a hit only by the IMU) is a collision as
 *       far as it's past the tread's grip, which is the most the floor can explain (the robot
 *       catching up with wheels that got ahead of it): a whole one at {@link
 *       Settings#joltMetersPerSecSq}, a share of one between. A reading at the accelerometer's
 *       range is a whole one, whatever the wheels did, and without a checked accelerometer the
 *       jolt's size alone decides. The speed comparison starts over after a whole one: the
 *       collision's doubt covers it. A share of one stays in it as well, so a slide it started is
 *       still seen. Treads said to grip at the jolt or past it leave no share between: a jolt up to
 *       the grip may be the floor's, so it's none, and one past it a whole one; by its size alone
 *       too, since driving could feel as much. A hit under the grip is then seen only as a slide.
 * </ul>
 *
 * <p>The turn and the speed each say how fast the wheels may be sliding over the floor; together,
 * the root of the sum of their squares, as a slide's turn at the wheels and its push add. The doubt
 * it adds is the distance the wheels may have slid, so it grows with the evidence.
 *
 * <p>Neither sees a slide that neither turns the robot nor changes its speed (wheels spinning at a
 * steady speed, wheels in the air): only the cameras find that. Nor does either ask whether the
 * drive motors are at their current limit: that says the motors push as hard as they may, not that
 * the wheels slip, and a limit set below the tread's grip keeps them from it.
 */
public final class SlipCheck {
  /**
   * How closely the wheels must agree with the IMU.
   *
   * @param windowSeconds how far back the turn's comparison looks: long enough for the gyro's noise
   *     to average out, short enough that a slide shows while it happens
   * @param turnToleranceDegrees how far apart the wheels and the gyro may turn over the window,
   *     however little it turned: the gyro's noise, and wheels worn unevenly, which turn the
   *     wheels' reckoning as the robot drives straight
   * @param turnScaleTolerance and this share of the turning on top: worn tread turns the wheels'
   *     reckoning a little further than the robot, the gyro's scale a little less
   * @param speedBiasMetersPerSecSq the most the accelerometer's bias may be: what it didn't feel is
   *     let go at this rate and no faster, so a bias up to it never builds up, and a slide stays
   *     seen until it stops
   * @param speedToleranceMetersPerSecond how far the wheels' speed and the accelerometer's
   *     reckoning of the robot's may drift apart without a slide: its noise, and its lag as the
   *     robot's acceleration changes. Past it, the whole of it is the slide's speed.
   * @param stillMetersPerSecond the robot is standing still when its wheels roll slower than this,
   *     and turn it slower than this at their reach, and the IMU turns it slower than this too
   * @param steadySeconds standing still, a reading the accelerometer holds steady this long is its
   *     bias, whatever its size, and its average over the older half of it is taken for it: longer
   *     than a push keeps the robot speeding up past wheels that don't roll, and long enough to
   *     average out the noise
   * @param steadyMetersPerSecond the reading holds steady when the speed it adds up to over that
   *     stretch strays no further than this from what its average adds up to: further, it rose or
   *     fell (a push building, or come and gone), and nothing is learned
   * @param largestBiasMetersPerSecSq the largest bias an accelerometer has: one learned past this
   *     is gravity's share, the IMU turned on the robot otherwise than its definition says
   * @param maxSlideMetersPerSecond the fastest the wheels and the robot can be sliding apart: past
   *     it, what the speed comparison holds is no slide (a bias it hasn't learned, adding up), and
   *     it counts no faster
   * @param joltMetersPerSecSq what the IMU feels and the wheels don't explain, past this, is a
   *     whole collision; past the tread's grip and short of this, a share of one. Past a grip at or
   *     above it, a whole one
   * @param accelerometerRangeMetersPerSecSq the most the accelerometer reports on each axis: a
   *     reading at it is a jolt past what it can say, whatever the wheels did
   */
  @ConfigRecord
  public record Settings(
      double windowSeconds,
      double turnToleranceDegrees,
      double turnScaleTolerance,
      double speedBiasMetersPerSecSq,
      double speedToleranceMetersPerSecond,
      double stillMetersPerSecond,
      double steadySeconds,
      double steadyMetersPerSecond,
      double largestBiasMetersPerSecSq,
      double maxSlideMetersPerSecond,
      double joltMetersPerSecSq,
      double accelerometerRangeMetersPerSecSq) {
    /**
     * The turn over a quarter second, within half a degree and 5% of it; the speed let go at 0.15
     * m/s² and within 0.3 m/s, and no slide past 5 m/s; a jolt past 15 m/s², under a Pigeon 2's ±2
     * g range.
     *
     * <p>The accelerometer's bias is a reading held for 2 s standing still, the speed it adds up to
     * keeping within 0.1 m/s of a steady one. A push sets the robot going in well under a second
     * (the simulator's at 3 m/s²) and then holds it at a speed, which reads as no acceleration, so
     * the bias is learned true; one that kept it speeding up steadily for 2 s, at even 0.5 m/s²,
     * would have it sliding 1 m/s past wheels that don't roll. The simulator's 0.3 m/s² of
     * vibration on each reading adds up to a speed about 0.06 m/s off the line, past 0.1 in one
     * window in 40, which only waits a loop to learn; a push building at 1 m/s³ strays past it half
     * a second in, still all in the newer half. One building slower than 0.2 m/s³ passes for
     * steady, and the bias follows it, 1.5 s behind. And 2 s learns a checked accelerometer's bias
     * of up to twice what the comparison lets go of (0.3 m/s²) before it holds the tolerance's
     * worth of it. Past 1 m/s², a bias is gravity: a Pigeon 2's is a tenth of that, and a case
     * tilted 6° on its mount leaks 1 m/s².
     */
    public static final Settings DEFAULTS =
        builder()
            .windowSeconds(0.25)
            .turnToleranceDegrees(0.5)
            .turnScaleTolerance(0.05)
            .speedBiasMetersPerSecSq(0.15)
            .speedToleranceMetersPerSecond(0.3)
            .stillMetersPerSecond(0.05)
            .steadySeconds(2)
            .steadyMetersPerSecond(0.1)
            .largestBiasMetersPerSecSq(1)
            .maxSlideMetersPerSecond(5)
            .joltMetersPerSecSq(15)
            .accelerometerRangeMetersPerSecSq(2 * ImuReadings.STANDARD_GRAVITY)
            .build();

    public Settings {
      Checks.on(Settings.class)
          .field("windowSeconds", windowSeconds, positive())
          .field("turnToleranceDegrees", turnToleranceDegrees, nonNegative())
          .field("turnScaleTolerance", turnScaleTolerance, nonNegative())
          .field("speedBiasMetersPerSecSq", speedBiasMetersPerSecSq, nonNegative())
          .field("speedToleranceMetersPerSecond", speedToleranceMetersPerSecond, nonNegative())
          .field("stillMetersPerSecond", stillMetersPerSecond, positive())
          .field("steadySeconds", steadySeconds, positive())
          .field("steadyMetersPerSecond", steadyMetersPerSecond, positive())
          .field("largestBiasMetersPerSecSq", largestBiasMetersPerSecSq, positive())
          .field("maxSlideMetersPerSecond", maxSlideMetersPerSecond, positive())
          .field("joltMetersPerSecSq", joltMetersPerSecSq, positive())
          .field("accelerometerRangeMetersPerSecSq", accelerometerRangeMetersPerSecSq, positive())
          .check(
              "joltMetersPerSecSq",
              () ->
                  joltMetersPerSecSq < accelerometerRangeMetersPerSecSq
                      ? null
                      : joltMetersPerSecSq
                          + " must be under the accelerometer's range ("
                          + accelerometerRangeMetersPerSecSq
                          + "), or a jolt along one axis can never pass it")
          .orThrow();
    }

    /** Starts building the settings, one named value at a time. */
    public static SlipCheckSettingsBuilder.WindowSecondsStage builder() {
      return SlipCheckSettingsBuilder.builder();
    }
  }

  /**
   * One loop of what the wheels and one IMU read. A part not a number starts its comparison over.
   *
   * @param seconds how long the loop was: since the last sample's wheel reading
   * @param wheelTurnRadians how far the wheels turned the robot over the loop
   * @param imuTurnRadians how far the IMU turned it over the same readings; not a number when this
   *     IMU didn't read both ends of them
   * @param yawRadians the IMU's yaw at the loop's end, which holds still while the robot turns: the
   *     frame the speed is compared in. Not a number without an IMU.
   * @param yawRateRadiansPerSecond how fast the IMU turns at the loop's end, counterclockwise; not
   *     a number if it can't say, and the wheels' turn rate stands in
   * @param imuPosition where the IMU sits, from the robot's origin (its center, on the floor)
   * @param accelerometerChecked whether this IMU's accelerometer is known to read along the robot's
   *     axes ({@link ImuMount#accelerometerChecked}): without it, speeds are compared by size, not
   *     direction, and a jolt is judged by its size alone
   * @param wheels the robot's velocity as its wheels measure it, robot-relative, at the loop's end
   * @param forwardAccelerationMetersPerSecSq the acceleration the IMU felt over the loop along the
   *     robot's forward axis, without gravity's share; not a number when it can't say
   * @param leftAccelerationMetersPerSecSq the same, along the robot's left axis
   * @param tiltRadians how far the robot is tilted from level, by the IMU's attitude: an
   *     accelerometer not yet checked may have gravity's share as the robot tilts taken off the
   *     wrong axis. Not a number, it's taken as level.
   */
  public record Sample(
      double seconds,
      double wheelTurnRadians,
      double imuTurnRadians,
      double yawRadians,
      double yawRateRadiansPerSecond,
      Translation3d imuPosition,
      boolean accelerometerChecked,
      ChassisVelocities wheels,
      double forwardAccelerationMetersPerSecSq,
      double leftAccelerationMetersPerSecSq,
      double tiltRadians) {
    public Sample {
      if (!(seconds > 0) || Double.isInfinite(seconds)) {
        throw new IllegalArgumentException("a loop takes time: " + seconds);
      }
    }
  }

  /** One loop's turns: how far the wheels, and the IMU, turned the robot over it. */
  private record Turn(double seconds, double wheels, double imu) {}

  /** One loop's accelerometer reading, along the robot's axes, standing still. */
  private record Reading(double seconds, double forward, double left) {}

  private final double windowSeconds;
  private final double turnToleranceRadians;
  private final double turnScaleTolerance;
  private final double speedBias;
  private final double speedTolerance;
  private final double stillSpeed;
  private final double steadySeconds;
  private final double steadySpeed;
  private final double largestBias;
  private final double maxSlide;

  /** Past this, a whole collision: the jolt's limit, or the tread's grip where that's more. */
  private final double whole;

  private final double grip;
  private final double saturation;
  private final double reachMeters;

  private final ArrayDeque<Turn> turns = new ArrayDeque<>();
  private double turnsSeconds;

  /**
   * The wheels' velocity carried to the IMU, at the last loop's end, in the IMU's fixed frame; not
   * a number at first.
   */
  private double lastVelocityX = Double.NaN;

  private double lastVelocityY = Double.NaN;

  /** How much faster than the robot the wheels roll, as the accelerometer reckons it. */
  private double slideX;

  private double slideY;

  /**
   * Not known to read along the robot's axes: how far the wheels' velocity has changed past what
   * the accelerometer felt, and how far what it felt has changed past the wheels, on the field. One
   * of the two is always none.
   */
  private double aheadX;

  private double aheadY;
  private double movedX;
  private double movedY;

  private double turnApart;
  private double turnSlip;
  private double speedSlip;
  private double collision;
  private double unexplained;

  /** The accelerometer's bias, along the robot's axes, as learned standing still. */
  private double biasForward;

  private double biasLeft;

  /** Whether the bias has been learned since the comparison started over: none is, at first. */
  private boolean biasLearned;

  /** The accelerometer's readings since the robot came to stand still, the last few seconds'. */
  private final ArrayDeque<Reading> standing = new ArrayDeque<>();

  private double standingSeconds;

  /** How long since the wheels were last seen sliding. */
  private double sinceSlide = Double.POSITIVE_INFINITY;

  /**
   * @param settings how closely the wheels must agree with the IMU
   * @param wheelReachMeters how far the wheels sit from the robot's center: a turn they see and the
   *     gyro doesn't, times this, is how far they slid
   * @param gripMetersPerSecSq the most the tread's grip can change the robot's velocity: its
   *     friction times g. Felt by the IMU and unexplained by the wheels, an acceleration up to it
   *     may be the floor bringing the robot to its wheels' speed, and is no collision. At {@link
   *     Settings#joltMetersPerSecSq} or past it, a collision is only what's past it.
   */
  public SlipCheck(Settings settings, double wheelReachMeters, double gripMetersPerSecSq) {
    if (!(wheelReachMeters > 0) || Double.isInfinite(wheelReachMeters)) {
      throw new IllegalArgumentException("the wheels' reach must be positive: " + wheelReachMeters);
    }
    windowSeconds = settings.windowSeconds();
    turnToleranceRadians = Math.toRadians(settings.turnToleranceDegrees());
    turnScaleTolerance = settings.turnScaleTolerance();
    speedBias = settings.speedBiasMetersPerSecSq();
    speedTolerance = settings.speedToleranceMetersPerSecond();
    stillSpeed = settings.stillMetersPerSecond();
    steadySeconds = settings.steadySeconds();
    steadySpeed = settings.steadyMetersPerSecond();
    largestBias = settings.largestBiasMetersPerSecSq();
    maxSlide = settings.maxSlideMetersPerSecond();
    if (!(gripMetersPerSecSq > 0) || Double.isInfinite(gripMetersPerSecSq)) {
      throw new IllegalArgumentException(
          "the tread's grip must be positive: " + gripMetersPerSecSq);
    }
    grip = gripMetersPerSecSq;
    whole = Math.max(settings.joltMetersPerSecSq(), grip);
    // A reading within a hair of the range is at it.
    saturation = 0.99 * settings.accelerometerRangeMetersPerSecSq();
    reachMeters = wheelReachMeters;
  }

  /** One loop's readings. */
  public void update(Sample sample) {
    compareTurns(sample.seconds(), sample.wheelTurnRadians(), sample.imuTurnRadians());
    compareSpeeds(sample);
  }

  /**
   * A loop without a sample to compare: no wheel readings, or no IMU, judged by the size of what an
   * IMU felt alone. Both comparisons start over.
   *
   * @param forwardAccelerationMetersPerSecSq what an IMU felt along the robot's forward axis,
   *     without gravity's share, or not a number without one
   * @param leftAccelerationMetersPerSecSq the same, along its left axis
   */
  public void startOver(
      double forwardAccelerationMetersPerSecSq, double leftAccelerationMetersPerSecSq) {
    turns.clear();
    turnsSeconds = 0;
    turnApart = 0;
    turnSlip = 0;
    forgetSpeeds();
    lastVelocityX = Double.NaN;
    lastVelocityY = Double.NaN;
    // Another IMU may read next, with a bias of its own.
    biasForward = 0;
    biasLeft = 0;
    biasLearned = false;
    standing.clear();
    standingSeconds = 0;
    sinceSlide = Double.POSITIVE_INFINITY;
    collision =
        joltBySize(forwardAccelerationMetersPerSecSq, leftAccelerationMetersPerSecSq) ? 1 : 0;
  }

  /**
   * Learns the accelerometer's bias while the robot stands still (its wheels and its gyro say it
   * isn't moving or turning) from a reading that holds steady: over the last {@link
   * Settings#steadySeconds} standing, the speed it adds up to strays no further than {@link
   * Settings#steadyMetersPerSecond} from what its average adds up to. The bias is then the older
   * half's average, which a push just begun, too small yet to tell, hasn't reached. Standing still,
   * the accelerometer feels its bias, whatever its size (gravity's share, from a case tilted on its
   * mount, is meters per second squared), or a push the wheels don't see. A push can't keep the
   * robot speeding up steadily for seconds: it sets the robot going and holds it at a speed, which
   * feels like standing, or it builds, or comes and goes, and the speed the reading adds up to
   * strays from a steady one. So the reading's size says nothing here, only whether it holds.
   */
  private void learnBias(Sample sample) {
    ChassisVelocities wheels = sample.wheels();
    double rate =
        Double.isFinite(sample.yawRateRadiansPerSecond()) ? sample.yawRateRadiansPerSecond() : 0;
    double forward = sample.forwardAccelerationMetersPerSecSq();
    double left = sample.leftAccelerationMetersPerSecSq();
    boolean still =
        Math.hypot(wheels.vx, wheels.vy) < stillSpeed
            && Math.abs(wheels.omega) * reachMeters < stillSpeed
            && Math.abs(rate) * reachMeters < stillSpeed
            && Double.isFinite(forward)
            && Double.isFinite(left);
    if (!still) {
      standing.clear();
      standingSeconds = 0;
      return;
    }
    standing.addLast(new Reading(sample.seconds(), forward, left));
    standingSeconds += sample.seconds();
    // The fewest loops that cover the stretch.
    while (standing.size() > 1
        && standingSeconds - standing.getFirst().seconds() >= steadySeconds) {
      standingSeconds -= standing.removeFirst().seconds();
    }
    if (standingSeconds < steadySeconds) {
      return;
    }
    // What the readings add up to, over the stretch and over its older half.
    double half = standingSeconds / 2;
    double elapsed = 0;
    double olderSeconds = 0;
    double olderForward = 0;
    double olderLeft = 0;
    double addedForward = 0;
    double addedLeft = 0;
    for (Reading reading : standing) {
      if (elapsed < half) {
        olderSeconds += reading.seconds();
        olderForward += reading.forward() * reading.seconds();
        olderLeft += reading.left() * reading.seconds();
      }
      addedForward += reading.forward() * reading.seconds();
      addedLeft += reading.left() * reading.seconds();
      elapsed += reading.seconds();
    }
    // Steady, the speed they add up to keeps to the line their average draws: it strays from it
    // where they rose or fell.
    double averageForward = addedForward / elapsed;
    double averageLeft = addedLeft / elapsed;
    double speedForward = 0;
    double speedLeft = 0;
    double since = 0;
    for (Reading reading : standing) {
      speedForward += reading.forward() * reading.seconds();
      speedLeft += reading.left() * reading.seconds();
      since += reading.seconds();
      if (Math.hypot(speedForward - averageForward * since, speedLeft - averageLeft * since)
          >= steadySpeed) {
        return;
      }
    }
    // The older half's: a push only just begun, too small yet to stray past the tolerance, is all
    // in the newer.
    biasForward = olderForward / olderSeconds;
    biasLeft = olderLeft / olderSeconds;
    biasLearned = true;
  }

  /**
   * How large the accelerometer's bias is, as learned standing still, in m/s²: zero until it's
   * learned.
   */
  public double biasMetersPerSecSq() {
    return Math.hypot(biasForward, biasLeft);
  }

  /**
   * Whether the accelerometer's bias, learned standing still, is past the largest a part has
   * ({@link Settings#largestBiasMetersPerSecSq}): gravity's share, leaking in because the IMU is
   * turned on the robot otherwise than its definition says, a case tilted on its mount.
   */
  public boolean leaksGravity() {
    return biasMetersPerSecSq() > largestBias;
  }

  /**
   * Whether an acceleration is a jolt by its size alone: past a whole collision's, which is past
   * what driving on the tread could feel, or at the range.
   */
  private boolean joltBySize(double forward, double left) {
    return Math.hypot(forward, left) > whole || saturated(forward, left);
  }

  /** Whether the accelerometer read at its range on either axis: past what it can say. */
  private boolean saturated(double forward, double left) {
    return Math.abs(forward) >= saturation || Math.abs(left) >= saturation;
  }

  /**
   * How far the wheels turned the robot beyond the gyro over the window, in radians, either way:
   * before the tolerances, for tuning them.
   */
  public double turnApartRadians() {
    return turnApart;
  }

  /**
   * How much faster or slower than the robot the wheels roll, as the accelerometer reckons it, in
   * meters per second: before the tolerance, for tuning it.
   */
  public double slideMetersPerSecond() {
    return Math.hypot(slideX, slideY) + Math.hypot(aheadX, aheadY) + Math.hypot(movedX, movedY);
  }

  /** How fast the wheels may be sliding, as the gyro sees them turning, in meters per second. */
  public double turnSlipMetersPerSecond() {
    return turnSlip;
  }

  /**
   * How fast the wheels may be sliding, as the accelerometer sees their speed, in meters per
   * second.
   */
  public double speedSlipMetersPerSecond() {
    return speedSlip;
  }

  /** How fast the wheels may be sliding over the floor, in meters per second: zero if they grip. */
  public double slipMetersPerSecond() {
    return Math.hypot(turnSlip, speedSlip);
  }

  /**
   * What the last loop's IMU felt that the wheels' own change of velocity doesn't explain, in m/s²:
   * zero when they explain it, or it felt less than they changed, or its accelerometer isn't
   * checked. For tuning the collision's limits.
   */
  public double unexplainedMetersPerSecSq() {
    return unexplained;
  }

  /**
   * How much of a collision the last loop's IMU felt, from none to a whole one: a jolt the wheels
   * don't explain past the limit, or one at the accelerometer's range, is a whole one; a jolt past
   * the limit the wheels partly explain, what they don't against the limit.
   */
  public double collisionShare() {
    return collision;
  }

  private void forgetSpeeds() {
    slideX = 0;
    slideY = 0;
    aheadX = 0;
    aheadY = 0;
    movedX = 0;
    movedY = 0;
    speedSlip = 0;
  }

  private void compareTurns(double seconds, double wheelTurn, double imuTurn) {
    if (!Double.isFinite(wheelTurn) || !Double.isFinite(imuTurn)) {
      turns.clear();
      turnsSeconds = 0;
      turnApart = 0;
      turnSlip = 0;
      return;
    }
    turns.addLast(new Turn(seconds, wheelTurn, imuTurn));
    turnsSeconds += seconds;
    // The fewest loops that cover the window.
    while (turns.size() > 1 && turnsSeconds - turns.getFirst().seconds() >= windowSeconds) {
      turnsSeconds -= turns.removeFirst().seconds();
    }
    double apart = 0;
    double turned = 0;
    for (Turn turn : turns) {
      apart += turn.wheels() - turn.imu();
      turned += turn.imu();
    }
    turnApart = Math.abs(apart);
    double beyond = turnApart - turnToleranceRadians - turnScaleTolerance * Math.abs(turned);
    // How fast, over the loops the window holds.
    turnSlip = beyond > 0 ? beyond * reachMeters / turnsSeconds : 0;
  }

  private void compareSpeeds(Sample sample) {
    double yaw = sample.yawRadians();
    ChassisVelocities wheels = sample.wheels();
    if (!Double.isFinite(yaw) || !Double.isFinite(wheels.vx) || !Double.isFinite(wheels.vy)) {
      startOver(
          sample.forwardAccelerationMetersPerSecSq(), sample.leftAccelerationMetersPerSecSq());
      return;
    }
    learnBias(sample);
    double readForward = sample.forwardAccelerationMetersPerSecSq();
    double readLeft = sample.leftAccelerationMetersPerSecSq();
    double forward = readForward - biasForward;
    double left = readLeft - biasLeft;
    // The wheels' velocity where the IMU sits: the robot's, plus its turn swinging the IMU around,
    // the turn's rate crossed with the IMU's position. Only the yaw rate is read, which swings it
    // level whatever its height: a robot pitching or rolling would swing a high IMU too.
    double rate =
        Double.isFinite(sample.yawRateRadiansPerSecond())
            ? sample.yawRateRadiansPerSecond()
            : wheels.omega;
    Translation3d position = sample.imuPosition();
    double atImuX = wheels.vx - rate * position.getY();
    double atImuY = wheels.vy + rate * position.getX();
    double cos = Math.cos(yaw);
    double sin = Math.sin(yaw);
    double velocityX = cos * atImuX - sin * atImuY;
    double velocityY = sin * atImuX + cos * atImuY;
    double feltX = cos * forward - sin * left;
    double feltY = sin * forward + cos * left;
    double seconds = sample.seconds();
    boolean before = Double.isFinite(lastVelocityX);
    // Its range is the part's, so what it read, its bias and all, reaches it.
    boolean saturated = saturated(readForward, readLeft);
    collision = Math.hypot(forward, left) > whole || saturated ? 1 : 0;
    boolean checked = sample.accelerometerChecked();
    unexplained = 0;
    if (checked && before && Double.isFinite(feltX) && Double.isFinite(feltY)) {
      double wheelsX = (velocityX - lastVelocityX) / seconds;
      double wheelsY = (velocityY - lastVelocityY) / seconds;
      if (sinceSlide < windowSeconds) {
        // Sliding, or just done (what the IMU feels of the robot catching up lags a little), the
        // wheels' change of velocity says nothing of the robot's: all it felt.
        unexplained = Math.hypot(feltX, feltY);
      } else if (Math.hypot(feltX, feltY) >= Math.hypot(wheelsX, wheelsY)) {
        // What the IMU felt that the wheels don't explain. Felt less than the wheels' own change,
        // it's the wheels changing faster than the robot can follow: a slide, not a hit.
        unexplained = Math.hypot(feltX - wheelsX, feltY - wheelsY);
      }
    }
    if (checked && before && !saturated) {
      // Past the grip, the floor can't explain it: a share of a collision, up to a whole one. A
      // grip at the jolt's limit or past it leaves no share between.
      collision =
          whole > grip
              ? Math.clamp((unexplained - grip) / (whole - grip), 0.0, 1.0)
              : unexplained > grip ? 1 : 0;
    }
    if (collision >= 1 || !Double.isFinite(feltX) || !Double.isFinite(feltY)) {
      forgetSpeeds();
    } else if (before && checked) {
      // How the wheels' velocity changed over the loop, less what the accelerometer felt...
      slideX += velocityX - lastVelocityX - feltX * seconds;
      slideY += velocityY - lastVelocityY - feltY * seconds;
      // ... let go toward none at the most a bias could have added, and no faster.
      double size = Math.hypot(slideX, slideY);
      double kept = kept(size, speedBias * seconds);
      slideX *= kept;
      slideY *= kept;
      speedSlip = slip(size * kept);
    } else if (before && biasLearned) {
      // Not before its bias is learned: gravity leaking into one tilted on its mount reads as the
      // robot speeding up, steadily, and would add up as a slide for as long as it ran.
      compareSizes(
          velocityX - lastVelocityX,
          velocityY - lastVelocityY,
          feltX * seconds,
          feltY * seconds,
          (speedBias + 2 * ImuReadings.STANDARD_GRAVITY * Math.sin(tilt(sample))) * seconds);
    }
    lastVelocityX = velocityX;
    lastVelocityY = velocityY;
    sinceSlide = speedSlip > 0 || turnSlip > 0 ? 0 : sinceSlide + seconds;
  }

  /** The robot's tilt, from level, or level if the IMU can't say. */
  private static double tilt(Sample sample) {
    return Double.isFinite(sample.tiltRadians()) ? Math.abs(sample.tiltRadians()) : 0;
  }

  /**
   * An accelerometer not known to read along the robot's axes compares sizes, not directions: what
   * it feels, turned on its mount, is the robot's every change of velocity turned the same way, so
   * the two add up to the same size whichever way it's turned. What both have added up alike is
   * driving, and is let go; what one holds past the other is a slide: the wheels getting ahead of
   * the robot, or the robot moved without them. A slide's size is at least that. Its bias, and
   * gravity's share as the robot tilts, which the robot's attitude may take off the wrong axis of a
   * mount not yet checked (at most twice the share), are let go of too.
   *
   * @param wheelsX how the wheels' velocity changed over the loop, on the field
   * @param wheelsY and along y
   * @param feltX what the accelerometer felt over the loop, as a change of velocity on the field
   * @param feltY and along y
   * @param letGo how much of a slide to let go of this loop
   */
  private void compareSizes(
      double wheelsX, double wheelsY, double feltX, double feltY, double letGo) {
    aheadX += wheelsX;
    aheadY += wheelsY;
    movedX += feltX;
    movedY += feltY;
    double ahead = Math.hypot(aheadX, aheadY);
    double moved = Math.hypot(movedX, movedY);
    double alike = Math.min(ahead, moved);
    double aheadKept = ahead > 0 ? (ahead - alike) / ahead : 0;
    double movedKept = moved > 0 ? (moved - alike) / moved : 0;
    aheadX *= aheadKept;
    aheadY *= aheadKept;
    movedX *= movedKept;
    movedY *= movedKept;
    // One of the two is now none: the other is the slide.
    double aheadSize = ahead - alike;
    double movedSize = moved - alike;
    double aheadLetGo = kept(aheadSize, letGo);
    double movedLetGo = kept(movedSize, letGo);
    aheadX *= aheadLetGo;
    aheadY *= aheadLetGo;
    movedX *= movedLetGo;
    movedY *= movedLetGo;
    speedSlip = slip(Math.max(aheadSize * aheadLetGo, movedSize * movedLetGo));
  }

  /**
   * How much of a slide of this size to keep, letting go of up to this much of it toward none, and
   * counting none faster than the robot can go: past that, what's held is a bias not yet learned,
   * adding up.
   */
  private double kept(double size, double letGo) {
    double held = Math.min(Math.max(size - letGo, 0), maxSlide);
    return size > 0 ? held / size : 0;
  }

  /**
   * How fast the wheels slide, from the slide held: past the tolerance, all of it, so the doubt
   * keeps up with the slide from where it's seen; within it, none.
   */
  private double slip(double held) {
    return held > speedTolerance ? held : 0;
  }
}

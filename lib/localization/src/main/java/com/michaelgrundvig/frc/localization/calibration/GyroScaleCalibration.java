package com.michaelgrundvig.frc.localization.calibration;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The gyro's scale, from whole turns in place against a heading reference: the robot's heading by
 * the cameras before and after, standing still where they see several tags. A gyro whose scale is
 * off by 0.3% misreads every turn by 0.3%: a few degrees over a match of spinning, and the same
 * share of every wheel radius measured against it.
 *
 * <p>The gyro counts the whole turns; the reference only has to say where the robot stopped, within
 * half a turn of what the gyro says. So spin enough turns for the error to show (five turns at 0.3%
 * is 5.4°), and no more than the error could stretch to half a turn.
 */
public final class GyroScaleCalibration {
  private double turned;
  private double lastYaw;
  private double startReference;
  private double trueTurn = Double.NaN;
  private boolean started;

  /** Standing still before the spin: the gyro's yaw, and the reference heading. */
  public void start(double gyroYawRadians, Rotation2d reference) {
    lastYaw = gyroYawRadians;
    startReference = reference.getRadians();
    turned = 0;
    trueTurn = Double.NaN;
    started = true;
  }

  /** A reading during the spin, at least once a loop so whole turns are counted. */
  public void update(double gyroYawRadians) {
    if (!started) {
      throw new IllegalStateException("start the spin first");
    }
    turned += MathUtil.angleModulus(gyroYawRadians - lastYaw);
    lastYaw = gyroYawRadians;
  }

  /** Standing still after the spin: the gyro's yaw, and the reference heading again. */
  public void finish(double gyroYawRadians, Rotation2d reference) {
    update(gyroYawRadians);
    // The turn the reference allows that is nearest the gyro's count.
    double referenceTurn = reference.getRadians() - startReference;
    trueTurn = turned + MathUtil.angleModulus(referenceTurn - turned);
  }

  /** How far the gyro says the robot turned, in radians, signed. */
  public double gyroTurnedRadians() {
    return turned;
  }

  /** How far the robot really turned, by the reference, in radians. NaN until finished. */
  public double trueTurnedRadians() {
    return trueTurn;
  }

  /**
   * The factor that corrects the gyro: its readings times this are the true rotation. NaN until
   * finished, or if it didn't turn.
   */
  public double scale() {
    return Double.isNaN(trueTurn) || turned == 0 ? Double.NaN : trueTurn / turned;
  }
}

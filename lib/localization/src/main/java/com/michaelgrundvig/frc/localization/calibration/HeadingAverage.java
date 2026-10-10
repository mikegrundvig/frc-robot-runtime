package com.michaelgrundvig.frc.localization.calibration;

import org.wpilib.math.geometry.Rotation2d;

/**
 * The average of headings, taken around the circle: 179° and -179° average to 180°, not 0°. For a
 * reference heading from many camera measurements while the robot stands still.
 */
public final class HeadingAverage {
  private double sumCos;
  private double sumSin;
  private int count;

  public void add(Rotation2d heading) {
    sumCos += heading.getCos();
    sumSin += heading.getSin();
    count++;
  }

  public int count() {
    return count;
  }

  /** The average heading; zero with nothing added. */
  public Rotation2d average() {
    return count == 0 ? Rotation2d.ZERO : new Rotation2d(Math.atan2(sumSin, sumCos));
  }

  /**
   * How tightly the headings agree, from 0 (spread evenly around) to 1 (all the same): the length
   * of their average direction.
   */
  public double agreement() {
    return count == 0 ? 0 : Math.hypot(sumCos, sumSin) / count;
  }

  public void clear() {
    sumCos = 0;
    sumSin = 0;
    count = 0;
  }
}
